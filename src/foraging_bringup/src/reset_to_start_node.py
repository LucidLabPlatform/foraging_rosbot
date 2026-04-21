#!/usr/bin/env python3
"""Drive the ROSbot to the start position and orientation using OptiTrack.

Three phases:
  1. Rotate to face goal        — orientation.z tracks atan2 direction to goal
  2. Drive to goal position     — forward speed proportional to distance,
                                   heading corrected via orientation.z
  3. Align to start orientation — rotate to goal_orientation_z reference

Launched via:
    roslaunch foraging_bringup reset_to_start.launch
"""

import json
import math
import sys
import threading

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String


class ResetToStart:

    CTRL_HZ = 20

    def __init__(self):
        # Start-position reference (OptiTrack frame)
        self.goal_x             = rospy.get_param("~goal_x",             2.9812698364257812)
        self.goal_y             = rospy.get_param("~goal_y",             1.8299084901809692)
        self.goal_orientation_z = rospy.get_param("~goal_orientation_z", -0.4383017122745514)

        # Tolerances
        self.pos_tolerance      = rospy.get_param("~pos_tolerance",      0.10)   # metres
        self.orient_tolerance   = rospy.get_param("~orient_tolerance",   0.01)   # orientation.z units (~1 deg)

        # Gains and limits
        self.max_linear_vel     = rospy.get_param("~max_linear_vel",     0.15)
        self.max_angular_vel    = rospy.get_param("~max_angular_vel",    0.40)
        self.k_lin              = rospy.get_param("~k_lin",              0.3)
        self.k_ang              = rospy.get_param("~k_ang",              2.0)

        # Interactive gating between phases (useful for step-by-step resets)
        self.require_enter_between_phases = rospy.get_param("~require_enter_between_phases", True)

        self._ot_pose  = None
        self._ot_lock  = threading.Lock()
        self._ot_event = threading.Event()

        self._status_pub  = rospy.Publisher("/reset_to_start/status", String, queue_size=1, latch=True)
        self._cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        rospy.Subscriber("/optitrack/rosbot/pose", PoseStamped, self._ot_cb, queue_size=1)

    # ── Callbacks / helpers ────────────────────────────────────────────────

    def _ot_cb(self, msg):
        with self._ot_lock:
            self._ot_pose = msg
        self._ot_event.set()

    def _pose(self):
        """Return (x, y, orientation_z) or None."""
        with self._ot_lock:
            msg = self._ot_pose
        if msg is None:
            return None
        p = msg.pose
        return p.position.x, p.position.y, p.orientation.z

    def _target_z_toward_goal(self, x, y):
        """orientation.z that corresponds to facing (goal_x, goal_y) from (x, y)."""
        yaw = math.atan2(self.goal_y - y, self.goal_x - x)
        return math.sin(yaw / 2.0)

    def _stop(self):
        self._cmd_vel_pub.publish(Twist())

    def _pub(self, state, **kw):
        payload = {"state": state, **kw}
        self._status_pub.publish(json.dumps(payload))
        rospy.loginfo("[reset] %s", payload)

    def _clamp(self, v, limit):
        return max(-limit, min(limit, v))

    def _wait_for_enter(self, prompt):
        """
        Block until user presses Enter (interactive terminal).
        Returns False if user aborts via Ctrl+C; returns True otherwise.
        """
        self._pub("waiting_for_enter", prompt=prompt)
        rospy.loginfo("[reset] %s", prompt)

        try:
            if sys.stdin is None:
                rospy.logwarn("[reset] stdin is not available; continuing without Enter.")
                return True
            input()
            return True
        except EOFError:
            rospy.logwarn("[reset] stdin is closed (EOF); continuing without Enter.")
            return True
        except KeyboardInterrupt:
            rospy.logwarn("[reset] user aborted while waiting for Enter.")
            return False

    # ── Phase 1: rotate to face the goal ──────────────────────────────────

    def _face_goal(self):
        """Rotate until orientation.z matches the direction toward goal."""
        rospy.loginfo("[reset] Phase 1: rotating to face goal")
        rate = rospy.Rate(self.CTRL_HZ)

        while not rospy.is_shutdown():
            pose = self._pose()
            if pose is None:
                self._stop()
                return False

            x, y, oz = pose
            target_z = self._target_z_toward_goal(x, y)
            err = target_z - oz

            rospy.loginfo_throttle(0.5,
                "[reset] face_goal  oz=%.4f  target_z=%.4f  err=%.4f", oz, target_z, err)

            if abs(err) <= self.orient_tolerance:
                self._stop()
                return True

            cmd = Twist()
            cmd.angular.z = self._clamp(self.k_ang * err, self.max_angular_vel)
            self._cmd_vel_pub.publish(cmd)
            rate.sleep()

        return False

    # ── Phase 2: drive to goal position ───────────────────────────────────

    def _drive_to_goal(self):
        """Drive forward while correcting heading. Returns True when within pos_tolerance."""
        rospy.loginfo("[reset] Phase 2: driving to goal")
        rate = rospy.Rate(self.CTRL_HZ)

        while not rospy.is_shutdown():
            pose = self._pose()
            if pose is None:
                self._stop()
                return False

            x, y, oz = pose
            dist = math.hypot(self.goal_x - x, self.goal_y - y)

            self._pub("navigating", pos_error=round(dist, 3))

            if dist <= self.pos_tolerance:
                self._stop()
                return True

            target_z = self._target_z_toward_goal(x, y)
            heading_err = target_z - oz

            # Forward speed proportional to distance, reduce when heading is off
            lin = self.k_lin * dist
            lin = min(lin, self.max_linear_vel)
            lin *= max(0.0, 1.0 - abs(heading_err) / 0.2)

            ang = self._clamp(self.k_ang * heading_err, self.max_angular_vel)

            rospy.loginfo_throttle(0.5,
                "[reset] drive  dist=%.3fm  oz=%.4f  target_z=%.4f  err=%.4f  lin=%.3f",
                dist, oz, target_z, heading_err, lin)

            cmd = Twist()
            cmd.linear.x  = lin
            cmd.angular.z = ang
            self._cmd_vel_pub.publish(cmd)
            rate.sleep()

        return False

    # ── Phase 3: align final orientation ──────────────────────────────────

    def _align_orientation(self):
        """Rotate to match goal_orientation_z exactly."""
        rospy.loginfo("[reset] Phase 3: aligning final orientation")
        rate = rospy.Rate(self.CTRL_HZ)

        while not rospy.is_shutdown():
            pose = self._pose()
            if pose is None:
                self._stop()
                return False

            oz = pose[2]
            err = self.goal_orientation_z - oz

            rospy.loginfo_throttle(0.5,
                "[reset] align  oz=%.4f  target=%.4f  err=%.4f",
                oz, self.goal_orientation_z, err)

            if abs(err) <= self.orient_tolerance:
                self._stop()
                return True

            cmd = Twist()
            cmd.angular.z = self._clamp(self.k_ang * err, self.max_angular_vel)
            self._cmd_vel_pub.publish(cmd)
            rate.sleep()

        return False

    # ── Main ──────────────────────────────────────────────────────────────

    def run(self):
        rospy.loginfo("[reset] Waiting for OptiTrack...")
        self._pub("waiting_for_optitrack")

        if not self._ot_event.wait(timeout=30.0):
            self._pub("error", error="OptiTrack timeout")
            return

        if not self._face_goal():
            self._pub("failed", error="face_goal_failed")
            return

        if self.require_enter_between_phases:
            if not self._wait_for_enter("Phase 1 complete. Press Enter to start Phase 2 (drive to goal)."):
                self._pub("failed", error="user_abort")
                return

        if not self._drive_to_goal():
            self._pub("failed", error="drive_failed")
            return

        if self.require_enter_between_phases:
            if not self._wait_for_enter("Phase 2 complete. Press Enter to start Phase 3 (final align)."):
                self._pub("failed", error="user_abort")
                return

        if not self._align_orientation():
            self._pub("failed", error="align_failed")
            return

        self._pub("success")
        rospy.loginfo("[reset] SUCCESS")


def main():
    rospy.init_node("reset_to_start")
    node = ResetToStart()
    node.run()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
