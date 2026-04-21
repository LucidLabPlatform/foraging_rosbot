#!/usr/bin/env python3
"""Drive the ROSbot to a target pose using OptiTrack.

Unicycle go-to-goal controller — one loop, no phases.
IR range sensors (fl/fr) provide front-obstacle safety.

Control law (per tick at 20 Hz):
    heading_error = atan2(goal_y - y, goal_x - x) - yaw
    angular_vel   = K_ANG * heading_error
    linear_vel    = K_LIN * dist * max(0, cos(heading_error))

Once within pos_tolerance, rotate in place to final yaw.

Launched via:
    roslaunch foraging_bringup reset_to_start.launch \
        goal_x:=2.98 goal_y:=1.83 goal_yaw:=-0.87
"""

import json
import math
import threading

import rospy
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Range
from std_msgs.msg import String


class ResetToStart:

    CTRL_HZ      = 20     # control loop rate
    FRONT_STOP_M = 0.10   # stop if obstacle within 10 cm
    FRONT_SLOW_M = 0.30   # slow linearly from 30 cm to 10 cm

    def __init__(self):
        self.goal_x          = rospy.get_param("~goal_x",          0.0)
        self.goal_y          = rospy.get_param("~goal_y",          0.0)
        self.goal_yaw        = rospy.get_param("~goal_yaw",        0.0)
        self.pos_tolerance   = rospy.get_param("~pos_tolerance",   0.10)
        self.yaw_tolerance   = rospy.get_param("~yaw_tolerance",   0.15)
        self.nav_timeout     = rospy.get_param("~nav_timeout",     60.0)
        self.max_linear_vel  = rospy.get_param("~max_linear_vel",  0.15)
        self.max_angular_vel = rospy.get_param("~max_angular_vel", 0.50)
        self.k_lin           = rospy.get_param("~k_lin",           0.4)
        self.k_ang           = rospy.get_param("~k_ang",           1.0)

        rospy.loginfo(
            "[reset] Goal: x=%.3f y=%.3f yaw=%.1f deg",
            self.goal_x, self.goal_y, math.degrees(self.goal_yaw),
        )

        # OptiTrack
        self._ot_pose  = None
        self._ot_lock  = threading.Lock()
        self._ot_event = threading.Event()

        # IR range sensors
        self._range      = {"fl": float("inf"), "fr": float("inf")}
        self._range_lock = threading.Lock()

        # Publishers
        self._status_pub  = rospy.Publisher("/reset_to_start/status", String, queue_size=1, latch=True)
        self._cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # Subscribers
        rospy.Subscriber("/optitrack/rosbot/pose", PoseStamped, self._ot_cb, queue_size=1)
        for name in ("fl", "fr"):
            rospy.Subscriber(f"/range/{name}", Range,
                             lambda msg, n=name: self._range_cb(msg, n), queue_size=1)

    # ── Callbacks ──────────────────────────────────────────────────────────

    def _ot_cb(self, msg):
        with self._ot_lock:
            self._ot_pose = msg
        self._ot_event.set()

    def _range_cb(self, msg, name):
        val = msg.range if msg.range >= msg.min_range else float("inf")
        with self._range_lock:
            self._range[name] = val

    # ── Helpers ────────────────────────────────────────────────────────────

    def _pose(self):
        with self._ot_lock:
            msg = self._ot_pose
        if msg is None:
            return None
        p = msg.pose
        q = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
        _, _, yaw = tft.euler_from_quaternion(q)
        return p.position.x, p.position.y, yaw

    def _fresh_pose(self, timeout=5.0):
        self._ot_event.clear()
        if not self._ot_event.wait(timeout=timeout):
            return None
        return self._pose()

    @staticmethod
    def _wrap(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def _front_clear(self):
        with self._range_lock:
            d = min(self._range["fl"], self._range["fr"])
        if d <= self.FRONT_STOP_M:
            return 0.0
        if d < self.FRONT_SLOW_M:
            return (d - self.FRONT_STOP_M) / (self.FRONT_SLOW_M - self.FRONT_STOP_M)
        return 1.0

    def _stop(self):
        self._cmd_vel_pub.publish(Twist())

    def _pub(self, state, **kw):
        payload = {"state": state, **kw}
        self._status_pub.publish(json.dumps(payload))
        rospy.loginfo("[reset] %s", payload)

    def _clamp(self, v, limit):
        return max(-limit, min(limit, v))

    # ── Phase 1: unicycle go-to-goal ───────────────────────────────────────

    def _go_to_goal(self):
        """Drive to (goal_x, goal_y) using unicycle controller. Returns True on success."""
        import time
        rate    = rospy.Rate(self.CTRL_HZ)
        deadline = time.time() + self.nav_timeout

        while not rospy.is_shutdown() and time.time() < deadline:
            pose = self._pose()
            if pose is None:
                rospy.logwarn("[reset] OptiTrack lost")
                self._stop()
                return False

            x, y, yaw = pose
            dx   = self.goal_x - x
            dy   = self.goal_y - y
            dist = math.hypot(dx, dy)

            if dist <= self.pos_tolerance:
                self._stop()
                return True

            heading_err = self._wrap(math.atan2(dy, dx) - yaw)

            # Rotate-only until roughly facing goal, then drive
            if abs(heading_err) > 0.5:   # ~30 deg — rotate in place
                lin = 0.0
            else:
                lin = self.k_lin * dist * max(0.0, math.cos(heading_err))
                lin = min(lin, self.max_linear_vel)
                # Range safety (only matters when driving forward)
                scale = self._front_clear()
                lin  *= scale
                if scale == 0.0:
                    rospy.logwarn_throttle(1.0, "[reset] Obstacle — holding")

            ang = self._clamp(self.k_ang * heading_err, self.max_angular_vel)

            cmd = Twist()
            cmd.linear.x  = lin
            cmd.angular.z = ang
            self._cmd_vel_pub.publish(cmd)

            self._pub("navigating",
                      pos_error=round(dist, 3),
                      yaw_error=round(abs(self._wrap(self.goal_yaw - yaw)), 3))
            rate.sleep()

        self._stop()
        rospy.logwarn("[reset] Go-to-goal timed out")
        return False

    # ── Phase 2: rotate to final yaw ──────────────────────────────────────

    def _align_yaw(self):
        """Rotate in place to goal_yaw. Returns True on success."""
        import time
        rate     = rospy.Rate(self.CTRL_HZ)
        deadline = time.time() + 20.0

        while not rospy.is_shutdown() and time.time() < deadline:
            pose = self._pose()
            if pose is None:
                self._stop()
                return False

            err = self._wrap(self.goal_yaw - pose[2])
            if abs(err) <= self.yaw_tolerance:
                self._stop()
                return True

            cmd = Twist()
            cmd.angular.z = self._clamp(self.k_ang * err, self.max_angular_vel)
            self._cmd_vel_pub.publish(cmd)
            rate.sleep()

        self._stop()
        rospy.logwarn("[reset] Yaw alignment timed out")
        return False

    # ── Main ───────────────────────────────────────────────────────────────

    def run(self):
        self._pub("waiting_for_optitrack")
        rospy.loginfo("[reset] Waiting for OptiTrack...")
        if not self._ot_event.wait(timeout=30.0):
            self._pub("error", error="OptiTrack timeout")
            return False

        pose = self._pose()
        x, y, yaw = pose
        pos_err = math.hypot(x - self.goal_x, y - self.goal_y)
        yaw_err = abs(self._wrap(self.goal_yaw - yaw))

        if pos_err <= self.pos_tolerance and yaw_err <= self.yaw_tolerance:
            self._pub("success", pos_error=round(pos_err, 3), yaw_error=round(yaw_err, 3))
            return True

        rospy.loginfo("[reset] Starting: pos_err=%.3fm yaw_err=%.2frad", pos_err, yaw_err)

        if not self._go_to_goal():
            self._pub("failed", error="go_to_goal_failed")
            return False

        if not self._align_yaw():
            self._pub("failed", error="yaw_align_failed")
            return False

        pose = self._fresh_pose(timeout=5.0)
        if pose is None:
            self._pub("error", error="OptiTrack stale after nav")
            return False

        x, y, yaw = pose
        pos_err = math.hypot(x - self.goal_x, y - self.goal_y)
        yaw_err = abs(self._wrap(self.goal_yaw - yaw))
        rospy.loginfo("[reset] Final: pos=%.3fm yaw=%.2frad", pos_err, yaw_err)

        if pos_err <= self.pos_tolerance and yaw_err <= self.yaw_tolerance:
            self._pub("success", pos_error=round(pos_err, 3), yaw_error=round(yaw_err, 3))
            rospy.loginfo("[reset] SUCCESS")
            return True

        self._pub("failed", pos_error=round(pos_err, 3), yaw_error=round(yaw_err, 3))
        rospy.logwarn("[reset] FAILED pos=%.3fm yaw=%.2frad", pos_err, yaw_err)
        return False


def main():
    rospy.init_node("reset_to_start")
    node = ResetToStart()
    success = node.run()
    rospy.loginfo("[reset] Exiting %s", "success" if success else "failure")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
