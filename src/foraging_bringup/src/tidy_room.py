#!/usr/bin/env python3

import json
import random
import time

import os
import sys
import rospy
import math
import threading
import rospkg
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from foraging_msgs.msg import PuckRegistry, ArucoRegistry
from foraging_msgs.srv import (RandomWalkServerMessage, RandomWalkServerMessageRequest,
                                CenterPuckServerMessage, CenterPuckServerMessageRequest,
                                PickPuckServerMessage, PickPuckServerMessageRequest,
                                DropPuckServerMessage, DropPuckServerMessageRequest,
                                UpdatePuckStatusMessage, UpdatePuckStatusMessageRequest)

sys.path.insert(0, os.path.join(rospkg.RosPack().get_path('foraging_services'), 'src'))
from move_base_client import MoveBaseClient


# ---------------------------------------------------------------------------
# Physical constants (unlikely to change between experiments)
# ---------------------------------------------------------------------------
APPROACH_DIST              = 0.40   # must exceed puck obstacle radius (0.10) + inflation (0.15)
PICK_DISTANCE              = 0.40
PICK_SPEED                 = 0.1
DROP_DISTANCE              = 0.25
DROP_SPEED                 = 0.1
NAVIGATE_TO_PUCK_TIMEOUT   = 60.0   # seconds
NAVIGATE_TO_CORNER_TIMEOUT = 60.0
CORNER_APPROACH_DIST       = 0.40   # navigation stop point — kept away from wall inflation zone
DROP_FORWARD_DIST          = 0.20   # how far drop_puck drives forward to deposit at corner
NAVIGATE_TO_SITE_TIMEOUT   = 60.0   # timeout for site fidelity navigation


# ---------------------------------------------------------------------------
# TidyRoom — autonomous behaviour orchestrator
# ---------------------------------------------------------------------------

class TidyRoom:
    def __init__(self):
        # Sweepable experiment parameters
        self.num_pucks_expected  = rospy.get_param('~num_pucks_expected', 6)
        self.num_corners_expected = rospy.get_param('~num_corners_expected', 3)
        self.explore_puck_pct    = rospy.get_param('~explore_puck_pct', 0.5)
        self.explore_corner_pct  = rospy.get_param('~explore_corner_pct', 0.67)
        self._site_fidelity_rate = rospy.get_param('~site_fidelity_rate', 0.5)
        self._trial_duration     = rospy.get_param('~trial_duration', 1200)  # seconds (20 min)
        self._puck_selection     = rospy.get_param('~puck_selection', 'closest')

        self._start_time = None
        self._status_pub  = rospy.Publisher('/foraging/status', String, queue_size=1, latch=True)
        self._results_pub = rospy.Publisher('/foraging/results', String, queue_size=1, latch=True)
        self._registry_lock = threading.Lock()

        self._puck_registry = []     # list of PuckConfirmed
        self._aruco_registry = []    # list of ArucoConfirmed
        self._skipped_pucks = set()  # puck IDs to skip (failed attempts)
        self._placed_pucks  = set()  # puck IDs confirmed placed (local cache, avoids registry callback race)

        # CPFA state
        self._last_find_location = None  # (x, y) of last picked puck for site fidelity
        self._puck_events = []           # per-puck timestamps for results

        # Subscribe to registries
        rospy.Subscriber('/puck/registry', PuckRegistry, self._puck_registry_cb, queue_size=1)
        rospy.Subscriber('/aruco/registry', ArucoRegistry, self._aruco_registry_cb, queue_size=1)

        # Service clients
        self._random_walk        = rospy.ServiceProxy('/random_walk',        RandomWalkServerMessage)
        self._center_puck        = rospy.ServiceProxy('/center_puck',        CenterPuckServerMessage)
        self._pick_puck          = rospy.ServiceProxy('/pick_puck',          PickPuckServerMessage)
        self._drop_puck          = rospy.ServiceProxy('/drop_puck',          DropPuckServerMessage)
        self._update_puck_status = rospy.ServiceProxy('/update_puck_status', UpdatePuckStatusMessage)

        # Shared navigation client (handles TF + move_base connection)
        self._nav = MoveBaseClient()

    # -----------------------------------------------------------------------
    # Main FSM
    # -----------------------------------------------------------------------

    def _publish_status(self, state, mode=None, **extra):
        msg = {"state": state}
        if mode is not None:
            msg["mode"] = mode
        if self._start_time is not None:
            msg["elapsed_s"] = round(time.time() - self._start_time, 1)
        msg.update(extra)
        self._status_pub.publish(json.dumps(msg))

    def _trial_timed_out(self):
        """Check if the trial duration has been exceeded."""
        if self._start_time is None or self._trial_duration <= 0:
            return False
        return (time.time() - self._start_time) >= self._trial_duration

    def _publish_results(self, timed_out):
        """Publish structured trial results to /foraging/results."""
        elapsed = round(time.time() - self._start_time, 1)
        with self._registry_lock:
            placed = sum(1 for p in self._puck_registry if p.status == 1)
        results = {
            "pucks_placed": placed,
            "duration_s": elapsed,
            "timed_out": timed_out,
            "params": {
                "site_fidelity_rate": self._site_fidelity_rate,
                "explore_puck_pct": self.explore_puck_pct,
                "explore_corner_pct": self.explore_corner_pct,
                "puck_selection": self._puck_selection,
                "num_pucks_expected": self.num_pucks_expected,
                "trial_duration": self._trial_duration,
            },
            "puck_events": self._puck_events,
        }
        self._results_pub.publish(json.dumps(results))
        rospy.loginfo("Trial results: %d pucks placed in %.1fs (timed_out=%s)",
                      placed, elapsed, timed_out)

    def run(self):
        self._start_time = time.time()
        mode = 0
        timed_out = False
        self._publish_status("running", mode=0)
        rospy.loginfo("TidyRoom starting in Mode 0 (explore) — trial duration: %ds",
                      self._trial_duration)
        try:
            while not rospy.is_shutdown():
                if self._trial_timed_out():
                    rospy.loginfo("Trial duration reached (%ds). Stopping.", self._trial_duration)
                    timed_out = True
                    break

                if mode == 0:
                    mode = self._mode0_explore()
                    if mode != 0:
                        self._publish_status("running", mode=mode)
                elif mode == 1:
                    mode = self._mode1_mixed()
                    if mode == 2:
                        self._publish_status("running", mode=2)
                elif mode == 2:
                    mode = self._mode2_exploit()
                    break  # if mode2 returns, we're done

        except (rospy.ROSInterruptException, rospy.ServiceException):
            rospy.loginfo("Shutdown requested — stopping trial")
        except Exception as e:
            self._publish_status("error", error=str(e))
            raise
        finally:
            if self._start_time is not None:
                elapsed = round(time.time() - self._start_time, 1)
                with self._registry_lock:
                    placed = sum(1 for p in self._puck_registry if p.status == 1)
                self._publish_status("completed", pucks_placed=placed, duration_s=elapsed,
                                     timed_out=timed_out)
                self._publish_results(timed_out)

    # -----------------------------------------------------------------------
    # Mode 0 — pure exploration
    # -----------------------------------------------------------------------

    def _mode0_explore(self):
        target_pucks   = max(1, round(self.num_pucks_expected * self.explore_puck_pct))
        target_corners = max(1, round(self.num_corners_expected * self.explore_corner_pct))
        rospy.loginfo("Mode 0: exploring until %d pucks and %d corners found",
                      target_pucks, target_corners)
        resp = self._random_walk(target_pucks, target_corners)
        if resp.done:
            rospy.loginfo("Mode 0 complete → Mode 1")
            return 1
        return 0

    # -----------------------------------------------------------------------
    # Mode 1 — mixed explore / exploit
    # -----------------------------------------------------------------------

    def _mode1_mixed(self):
        with self._registry_lock:
            puck_registry = list(self._puck_registry)
            aruco_registry = list(self._aruco_registry)

        # Check if all found → Mode 2
        if self._all_found(puck_registry, aruco_registry):
            rospy.loginfo("All pucks and corners found → Mode 2")
            return 2

        puck = self._select_puck(puck_registry, aruco_registry)
        if puck is None:
            # No deliverable puck, explore more
            current_pucks   = len(puck_registry)
            current_corners = len(aruco_registry)
            rospy.loginfo(
                "No deliverable puck, exploring for more (pucks=%d corners=%d)",
                current_pucks, current_corners
            )
            # Return value intentionally discarded — we re-evaluate the full state on the next loop iteration
            self._random_walk(
                min(current_pucks + 1, self.num_pucks_expected),
                min(current_corners + 1, self.num_corners_expected),
            )
            return 1

        corner = self._get_corner_for_puck(puck, aruco_registry)
        if corner is None:
            # Should not happen if _select_puck filters correctly, but be safe
            self._skipped_pucks.add(puck.id)
            return 1

        success = self._pick_and_deliver(puck, corner)
        if not success:
            rospy.logwarn("Failed to pick/deliver puck %d (color=%d), skipping",
                          puck.id, puck.color)
            self._skipped_pucks.add(puck.id)
        else:
            # CPFA site fidelity: probabilistically return to last find location
            if random.random() < self._site_fidelity_rate and self._last_find_location:
                rospy.loginfo("Site fidelity: returning to (%.2f, %.2f)",
                              self._last_find_location[0], self._last_find_location[1])
                self._navigate_to_site(*self._last_find_location)

        return 1

    # -----------------------------------------------------------------------
    # Mode 2 — pure exploitation
    # -----------------------------------------------------------------------

    def _mode2_exploit(self):
        rospy.loginfo("Mode 2: pure exploitation")
        while not rospy.is_shutdown():
            if self._trial_timed_out():
                return 3

            with self._registry_lock:
                puck_registry = list(self._puck_registry)
                aruco_registry = list(self._aruco_registry)

            puck = self._select_puck(puck_registry, aruco_registry)
            if puck is None:
                rospy.loginfo("All pucks delivered (%d placed). Done!", len(self._placed_pucks))
                return 3  # done

            corner = self._get_corner_for_puck(puck, aruco_registry)
            if corner is None:
                rospy.logwarn("Puck %d has no known corner, skipping", puck.id)
                self._skipped_pucks.add(puck.id)
                continue

            success = self._pick_and_deliver(puck, corner)
            if not success:
                rospy.logwarn("Failed to deliver puck %d, skipping", puck.id)
                self._skipped_pucks.add(puck.id)
            else:
                # CPFA site fidelity: probabilistically return to last find location
                if random.random() < self._site_fidelity_rate and self._last_find_location:
                    rospy.loginfo("Site fidelity: returning to (%.2f, %.2f)",
                                  self._last_find_location[0], self._last_find_location[1])
                    self._navigate_to_site(*self._last_find_location)

        return 3

    # -----------------------------------------------------------------------
    # Puck selection
    # -----------------------------------------------------------------------

    def _select_puck(self, puck_registry, aruco_registry):
        """Return the next puck to collect, or None if nothing is available."""
        # Filter: unplaced (status==0), not skipped, corner known
        candidates = [
            p for p in puck_registry
            if p.status == 0
            and p.id not in self._skipped_pucks
            and p.id not in self._placed_pucks
            and self._get_corner_for_puck(p, aruco_registry) is not None
        ]
        if not candidates:
            return None

        robot_pos = self._nav.get_robot_position()

        if self._puck_selection == "color_order":
            # Sort by color (1→2→3), then distance
            def key(p):
                dist = (
                    math.sqrt((p.x - robot_pos[0])**2 + (p.y - robot_pos[1])**2)
                    if robot_pos else 0
                )
                return (p.color, dist)
        else:  # "closest"
            def key(p):
                if robot_pos is None:
                    return 0
                return math.sqrt((p.x - robot_pos[0])**2 + (p.y - robot_pos[1])**2)

        return min(candidates, key=key)

    def _get_corner_for_puck(self, puck, aruco_registry):
        """Find the ArUco marker whose marker_id matches the puck's color."""
        for marker in aruco_registry:
            if marker.marker_id == puck.color:
                return marker
        return None

    # -----------------------------------------------------------------------
    # Pick-and-deliver sequence
    # -----------------------------------------------------------------------

    def _pick_and_deliver(self, puck, corner):
        rospy.loginfo("Picking puck %d (color=%d) at (%.2f, %.2f)",
                      puck.id, puck.color, puck.x, puck.y)
        pick_start_s = round(time.time() - self._start_time, 1)

        # Step 1+2: Approach and center — retry from different angles if puck not visible
        centered = False
        for angle_offset in [0, math.pi / 2, -math.pi / 2, math.pi]:
            if not self._navigate_to_approach(puck, angle_offset=angle_offset):
                rospy.logwarn("Failed to reach approach point for puck %d (offset=%.0f°)",
                              puck.id, math.degrees(angle_offset))
                continue
            try:
                resp = self._center_puck(puck.color)
                if resp.success:
                    centered = True
                    break
                rospy.logwarn("center_puck failed for puck %d (offset=%.0f°), retrying",
                              puck.id, math.degrees(angle_offset))
            except rospy.ServiceException as e:
                rospy.logwarn("center_puck service error: %s", e)
        if not centered:
            rospy.logwarn("Could not find puck %d from any angle", puck.id)
            return False

        # Step 3: Pick up
        try:
            resp = self._pick_puck(PICK_DISTANCE, PICK_SPEED)
            if not resp.success:
                rospy.logwarn("pick_puck failed for puck %d", puck.id)
                return False
        except rospy.ServiceException as e:
            rospy.logwarn("pick_puck service error: %s", e)
            return False

        rospy.loginfo("Puck %d picked. Navigating to corner at (%.2f, %.2f)",
                      puck.id, corner.x, corner.y)

        # Step 3b: Remove puck from obstacle cloud so robot can navigate freely
        try:
            self._update_puck_status(puck.id, 1, puck.x, puck.y)
        except rospy.ServiceException:
            pass

        # Step 4: Navigate to corner
        robot_pos = self._nav.get_robot_position()
        dx = corner.x - (robot_pos[0] if robot_pos else 0)
        dy = corner.y - (robot_pos[1] if robot_pos else 0)
        dist = math.sqrt(dx * dx + dy * dy)
        ux, uy = (dx / dist, dy / dist) if dist > 0 else (0, 0)
        goal_x = corner.x - ux * CORNER_APPROACH_DIST
        goal_y = corner.y - uy * CORNER_APPROACH_DIST
        rospy.loginfo("Corner approach: robot=(%.2f, %.2f) corner=(%.2f, %.2f) goal=(%.2f, %.2f) dist=%.2f",
                      robot_pos[0] if robot_pos else 0, robot_pos[1] if robot_pos else 0,
                      corner.x, corner.y, goal_x, goal_y, dist)
        if not self._navigate_to_corner(corner):
            rospy.logwarn("Failed to reach corner for puck %d — goal was (%.2f, %.2f), %.2fm from corner",
                          puck.id, goal_x, goal_y, CORNER_APPROACH_DIST)
            # Puck is already picked — still drop it so gripper is free
            try:
                self._drop_puck(DROP_DISTANCE, DROP_SPEED, DROP_FORWARD_DIST)
            except rospy.ServiceException:
                pass
            # Revert puck status since delivery failed
            try:
                self._update_puck_status(puck.id, 0, puck.x, puck.y)
            except rospy.ServiceException:
                pass
            return False

        # Step 5: Drop puck
        try:
            resp = self._drop_puck(DROP_DISTANCE, DROP_SPEED, DROP_FORWARD_DIST)
            if not resp.success:
                rospy.logwarn("drop_puck service call returned failure for puck %d", puck.id)
                return False
        except rospy.ServiceException as e:
            rospy.logwarn("drop_puck service error: %s", e)
            return False

        # Step 6: Update final position in registry
        try:
            resp = self._update_puck_status(puck.id, 1, corner.x, corner.y)
            if resp.success:
                rospy.loginfo("Puck %d marked as placed home at (%.2f, %.2f)",
                              puck.id, corner.x, corner.y)
            else:
                rospy.logwarn("update_puck_status failed: %s", resp.error_message)
        except rospy.ServiceException as e:
            rospy.logwarn("update_puck_status service error: %s", e)

        # Mark as placed locally so _select_puck doesn't re-pick before registry callback fires
        self._placed_pucks.add(puck.id)

        # Record per-puck event timestamps and update site fidelity location
        delivered_at_s = round(time.time() - self._start_time, 1)
        self._puck_events.append({
            "puck_id": puck.id,
            "color": puck.color,
            "picked_at_s": pick_start_s,
            "delivered_at_s": delivered_at_s,
        })
        self._last_find_location = (puck.x, puck.y)

        return True

    # -----------------------------------------------------------------------
    # Navigation helpers
    # -----------------------------------------------------------------------

    def _navigate_to_approach(self, puck, angle_offset=0):
        """Navigate to a point APPROACH_DIST metres from the puck, facing the puck.
        angle_offset rotates the approach direction around the puck."""
        robot_pos = self._nav.get_robot_position()
        if robot_pos is None:
            rospy.logwarn("Cannot get robot position for approach")
            return False

        dx = puck.x - robot_pos[0]
        dy = puck.y - robot_pos[1]
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < APPROACH_DIST and angle_offset == 0:
            return True

        # Direction from puck to robot, then rotate by offset
        base_angle = math.atan2(-dy, -dx)
        approach_angle = base_angle + angle_offset
        goal_x = puck.x + APPROACH_DIST * math.cos(approach_angle)
        goal_y = puck.y + APPROACH_DIST * math.sin(approach_angle)
        yaw = math.atan2(puck.y - goal_y, puck.x - goal_x)

        return self._nav.go_to(goal_x, goal_y, yaw=yaw, timeout=NAVIGATE_TO_PUCK_TIMEOUT)

    def _navigate_to_corner(self, corner):
        """Navigate to a point CORNER_APPROACH_DIST metres from the corner, facing it.
        The robot stops here so drop_puck can drive forward to deposit at the corner."""
        robot_pos = self._nav.get_robot_position()
        if robot_pos is None:
            rospy.logwarn("Cannot get robot position for corner approach")
            return False

        dx = corner.x - robot_pos[0]
        dy = corner.y - robot_pos[1]
        dist = math.sqrt(dx * dx + dy * dy)
        ux, uy = dx / dist, dy / dist

        goal_x = corner.x - ux * CORNER_APPROACH_DIST
        goal_y = corner.y - uy * CORNER_APPROACH_DIST
        yaw = math.atan2(uy, ux)

        return self._nav.go_to(goal_x, goal_y, yaw=yaw, timeout=NAVIGATE_TO_CORNER_TIMEOUT)

    # -----------------------------------------------------------------------
    # Utility helpers
    # -----------------------------------------------------------------------

    def _navigate_to_site(self, x, y):
        """Navigate to a previously visited location (CPFA site fidelity)."""
        return self._nav.go_to(x, y, timeout=NAVIGATE_TO_SITE_TIMEOUT)

    def _all_found(self, puck_registry, aruco_registry):
        return (len(puck_registry) >= self.num_pucks_expected and
                len(aruco_registry) >= self.num_corners_expected)

    # -----------------------------------------------------------------------
    # Registry callbacks
    # -----------------------------------------------------------------------

    def _puck_registry_cb(self, msg):
        with self._registry_lock:
            self._puck_registry = list(msg.pucks)

    def _aruco_registry_cb(self, msg):
        with self._registry_lock:
            self._aruco_registry = list(msg.markers)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main():
    rospy.init_node('tidy_room')
    node = TidyRoom()

    if not node._nav.wait_until_ready():
        rospy.logfatal("Navigation not available — aborting")
        return
    rospy.loginfo("Waiting for services...")
    for svc_name in ['/random_walk', '/center_puck', '/pick_puck',
                     '/drop_puck', '/update_puck_status']:
        rospy.wait_for_service(svc_name)
    rospy.loginfo("All services ready. Starting.")
    node.run()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
