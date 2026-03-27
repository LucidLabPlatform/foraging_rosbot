#!/usr/bin/env python3

import rospy
import math
import threading
import tf
import actionlib
from actionlib_msgs.msg import GoalStatus
from foraging_msgs.msg import PuckRegistry, ArucoRegistry
from foraging_msgs.srv import (RandomWalkServerMessage, RandomWalkServerMessageRequest,
                                CenterPuckServerMessage, CenterPuckServerMessageRequest,
                                PickPuckServerMessage, PickPuckServerMessageRequest,
                                DropPuckServerMessage, DropPuckServerMessageRequest,
                                UpdatePuckStatusMessage, UpdatePuckStatusMessageRequest)
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion


# ---------------------------------------------------------------------------
# Parameters
# ---------------------------------------------------------------------------
NUM_CORNERS_EXPECTED     = 3
APPROACH_DIST            = 0.55   # meters offset from puck position for approach waypoint
PUCK_SELECTION_STRATEGY  = "closest"   # "closest" | "color_order"
PICK_DISTANCE            = 0.20
PICK_SPEED               = 0.1
DROP_DISTANCE            = 0.20
DROP_SPEED               = 0.1
NAVIGATE_TO_PUCK_TIMEOUT = 30.0  # seconds
NAVIGATE_TO_CORNER_TIMEOUT = 30.0
CORNER_APPROACH_DIST = 0.40   # meters offset from corner position for approach waypoint


# ---------------------------------------------------------------------------
# TidyRoom — autonomous behaviour orchestrator
# ---------------------------------------------------------------------------

class TidyRoom:
    def __init__(self):
        self.num_pucks_expected  = rospy.get_param('~num_pucks_expected', 6)
        self.explore_puck_pct    = rospy.get_param('~explore_puck_pct', 0.5)
        self.explore_corner_pct  = rospy.get_param('~explore_corner_pct', 0.67)

        self._registry_lock = threading.Lock()

        self._puck_registry = []     # list of PuckConfirmed
        self._aruco_registry = []    # list of ArucoConfirmed
        self._skipped_pucks = set()  # puck IDs to skip (failed attempts)

        # Subscribe to registries
        rospy.Subscriber('/puck/registry', PuckRegistry, self._puck_registry_cb, queue_size=1)
        rospy.Subscriber('/aruco/registry', ArucoRegistry, self._aruco_registry_cb, queue_size=1)

        # Service clients
        self._random_walk        = rospy.ServiceProxy('/random_walk',        RandomWalkServerMessage)
        self._center_puck        = rospy.ServiceProxy('/center_puck',        CenterPuckServerMessage)
        self._pick_puck          = rospy.ServiceProxy('/pick_puck',          PickPuckServerMessage)
        self._drop_puck          = rospy.ServiceProxy('/drop_puck',          DropPuckServerMessage)
        self._update_puck_status = rospy.ServiceProxy('/update_puck_status', UpdatePuckStatusMessage)

        # move_base action client
        self._move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        # TF listener
        self._tf_listener = tf.TransformListener()

    # -----------------------------------------------------------------------
    # Main FSM
    # -----------------------------------------------------------------------

    def run(self):
        mode = 0
        rospy.loginfo("TidyRoom starting in Mode 0 (explore)")
        while not rospy.is_shutdown():
            if mode == 0:
                mode = self._mode0_explore()
            elif mode == 1:
                mode = self._mode1_mixed()
            elif mode == 2:
                mode = self._mode2_exploit()
                break  # if mode2 returns, we're done

    # -----------------------------------------------------------------------
    # Mode 0 — pure exploration
    # -----------------------------------------------------------------------

    def _mode0_explore(self):
        target_pucks   = max(1, round(self.num_pucks_expected * self.explore_puck_pct))
        target_corners = max(1, round(NUM_CORNERS_EXPECTED * self.explore_corner_pct))
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
                min(current_corners + 1, NUM_CORNERS_EXPECTED),
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

        return 1

    # -----------------------------------------------------------------------
    # Mode 2 — pure exploitation
    # -----------------------------------------------------------------------

    def _mode2_exploit(self):
        rospy.loginfo("Mode 2: pure exploitation")
        while not rospy.is_shutdown():
            with self._registry_lock:
                puck_registry = list(self._puck_registry)
                aruco_registry = list(self._aruco_registry)

            puck = self._select_puck(puck_registry, aruco_registry)
            if puck is None:
                rospy.loginfo("All pucks delivered. Done!")
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
            and self._get_corner_for_puck(p, aruco_registry) is not None
        ]
        if not candidates:
            return None

        robot_pos = self._get_robot_position()

        if PUCK_SELECTION_STRATEGY == "color_order":
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

        # Step 1: Navigate to approach point
        if not self._navigate_to_approach(puck):
            rospy.logwarn("Failed to reach approach point for puck %d", puck.id)
            return False

        # Step 2: Center puck visually
        try:
            resp = self._center_puck(puck.color)
            if not resp.success:
                rospy.logwarn("center_puck failed for puck %d", puck.id)
                return False
        except rospy.ServiceException as e:
            rospy.logwarn("center_puck service error: %s", e)
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

        # Step 4: Navigate to corner
        if not self._navigate_to_corner(corner):
            rospy.logwarn("Failed to reach corner for puck %d", puck.id)
            # Puck is already picked — still drop it so gripper is free
            try:
                self._drop_puck(DROP_DISTANCE, DROP_SPEED)
            except rospy.ServiceException:
                pass
            return False

        # Step 5: Drop puck
        try:
            resp = self._drop_puck(DROP_DISTANCE, DROP_SPEED)
            if not resp.success:
                rospy.logwarn("drop_puck service call returned failure for puck %d", puck.id)
                return False
        except rospy.ServiceException as e:
            rospy.logwarn("drop_puck service error: %s", e)
            return False

        # Step 6: Update status in registry
        try:
            resp = self._update_puck_status(puck.id, 1, corner.x, corner.y)
            if resp.success:
                rospy.loginfo("Puck %d marked as placed home at (%.2f, %.2f)",
                              puck.id, corner.x, corner.y)
            else:
                rospy.logwarn("update_puck_status failed: %s", resp.error_message)
        except rospy.ServiceException as e:
            rospy.logwarn("update_puck_status service error: %s", e)

        return True

    # -----------------------------------------------------------------------
    # Navigation helpers
    # -----------------------------------------------------------------------

    def _navigate_to_approach(self, puck):
        """Navigate to a point APPROACH_DIST metres from the puck, facing the puck."""
        robot_pos = self._get_robot_position()
        if robot_pos is None:
            rospy.logwarn("Cannot get robot position for approach")
            return False

        dx = puck.x - robot_pos[0]
        dy = puck.y - robot_pos[1]
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < APPROACH_DIST:
            # Already close enough — no need to navigate
            return True

        ux, uy = dx / dist, dy / dist
        goal_x = puck.x - ux * APPROACH_DIST
        goal_y = puck.y - uy * APPROACH_DIST
        yaw = math.atan2(uy, ux)  # face toward the puck

        return self._send_nav_goal(goal_x, goal_y, yaw=yaw, timeout=NAVIGATE_TO_PUCK_TIMEOUT)

    def _navigate_to_corner(self, corner):
        """Navigate to a point CORNER_APPROACH_DIST metres from the corner, facing it.
        The robot stops here so drop_puck can drive forward to deposit at the corner."""
        robot_pos = self._get_robot_position()
        if robot_pos is None:
            rospy.logwarn("Cannot get robot position for corner approach")
            return False

        dx = corner.x - robot_pos[0]
        dy = corner.y - robot_pos[1]
        dist = math.sqrt(dx * dx + dy * dy)
        ux, uy = dx / dist, dy / dist

        goal_x = corner.x - ux * CORNER_APPROACH_DIST
        goal_y = corner.y - uy * CORNER_APPROACH_DIST
        yaw = math.atan2(uy, ux)  # face toward the corner

        return self._send_nav_goal(goal_x, goal_y, yaw=yaw, timeout=NAVIGATE_TO_CORNER_TIMEOUT)

    def _send_nav_goal(self, x, y, yaw=0.0, timeout=30.0):
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation = Quaternion(*q)

        self._move_base.send_goal(goal)
        finished = self._move_base.wait_for_result(rospy.Duration(timeout))
        if not finished:
            self._move_base.cancel_goal()
            rospy.logwarn("Navigation timed out after %.0fs", timeout)
            return False

        state = self._move_base.get_state()
        return state == GoalStatus.SUCCEEDED

    # -----------------------------------------------------------------------
    # Utility helpers
    # -----------------------------------------------------------------------

    def _get_robot_position(self):
        """Returns (x, y) in map frame, or None on failure."""
        try:
            self._tf_listener.waitForTransform(
                "map", "base_link", rospy.Time(0), rospy.Duration(1.0)
            )
            (trans, _) = self._tf_listener.lookupTransform("map", "base_link", rospy.Time(0))
            return (trans[0], trans[1])
        except (tf.Exception, tf.LookupException,
                tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("TF lookup failed: %s", e)
            return None

    def _all_found(self, puck_registry, aruco_registry):
        return (len(puck_registry) >= self.num_pucks_expected and
                len(aruco_registry) >= NUM_CORNERS_EXPECTED)

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
    rospy.loginfo("Waiting for move_base...")
    node._move_base.wait_for_server()
    rospy.loginfo("Waiting for services...")
    for svc_name in ['/random_walk', '/center_puck', '/pick_puck',
                     '/drop_puck', '/update_puck_status']:
        rospy.wait_for_service(svc_name)
    rospy.loginfo("All services ready. Starting.")
    rospy.loginfo("Waiting for TF map→base_link...")
    node._tf_listener.waitForTransform("map", "base_link", rospy.Time(0), rospy.Duration(10.0))
    rospy.loginfo("TF ready.")
    node.run()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
