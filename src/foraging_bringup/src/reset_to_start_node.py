#!/usr/bin/env python3
"""Drive the ROSbot to a target pose using OptiTrack.

Direct-drive PID control in OptiTrack frame — no move_base, no odom/TF.
IR range sensors (fl/fr/rl/rr) provide front-obstacle safety.

Three-phase:
  1. Rotate in place to face the goal position
  2. Drive to goal position (linear + heading PD, range safety)
  3. Rotate in place to final yaw

Launched via:
    roslaunch foraging_bringup reset_to_start.launch \
        goal_x:=1.5 goal_y:=2.0 goal_yaw:=0.78
"""

import json
import math
import threading
import time

import rospy
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Range
from std_msgs.msg import String


class PID:
    """PD controller (no integral to avoid windup)."""

    def __init__(self, kp, kd, out_min, out_max):
        self.kp = kp
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self._prev_error = None
        self._prev_time = None

    def reset(self):
        self._prev_error = None
        self._prev_time = None

    def compute(self, error):
        now = time.time()
        dt = (now - self._prev_time) if self._prev_time is not None else 0.05
        dt = max(dt, 0.001)
        derivative = ((error - self._prev_error) / dt
                      if self._prev_error is not None else 0.0)
        self._prev_error = error
        self._prev_time = now
        output = self.kp * error + self.kd * derivative
        return max(self.out_min, min(self.out_max, output))


class ResetToStart:
    """OptiTrack-guided direct-drive go-to-pose with IR range-sensor safety."""

    # IR sensor safety (valid readings are 0.03–0.90 m; <0 means no obstacle)
    FRONT_STOP_M = 0.10   # stop forward motion if obstacle within 10 cm
    FRONT_SLOW_M = 0.25   # linearly reduce speed between 25 cm and 10 cm

    CTRL_HZ = 20          # PID loop rate

    def __init__(self):
        # ── Parameters ────────────────────────────────────────────────────
        # New parameter names (preferred)
        self.goal_x   = rospy.get_param("~goal_x",   0)
        self.goal_y   = rospy.get_param("~goal_y",   0)
        self.goal_yaw = rospy.get_param("~goal_yaw", 0)

        self.pos_tolerance   = rospy.get_param("~pos_tolerance",   0.10)
        self.yaw_tolerance   = rospy.get_param("~yaw_tolerance",   0.15)
        self.nav_timeout     = rospy.get_param("~nav_timeout",     60.0)
        self.max_linear_vel  = rospy.get_param("~max_linear_vel",  0.20)
        self.max_angular_vel = rospy.get_param("~max_angular_vel", 0.60)

        rospy.loginfo(
            "[reset] Goal: x=%.3f y=%.3f yaw=%.1f deg  tol=%.2fm/%.2frad",
            self.goal_x, self.goal_y, math.degrees(self.goal_yaw),
            self.pos_tolerance, self.yaw_tolerance,
        )

        # ── PID controllers ────────────────────────────────────────────────
        self._lin_pid = PID(
            kp=0.4, kd=0.05,
            out_min=0.0, out_max=self.max_linear_vel,
        )
        self._ang_pid = PID(
            kp=1.0, kd=0.08,
            out_min=-self.max_angular_vel, out_max=self.max_angular_vel,
        )

        # ── OptiTrack pose ─────────────────────────────────────────────────
        self._ot_pose = None
        self._ot_lock = threading.Lock()
        self._ot_event = threading.Event()

        # ── IR range sensors ───────────────────────────────────────────────
        self._range = {"fl": float("inf"), "fr": float("inf"),
                       "rl": float("inf"), "rr": float("inf")}
        self._range_lock = threading.Lock()

        # ── Publishers ─────────────────────────────────────────────────────
        self._status_pub = rospy.Publisher(
            "/reset_to_start/status", String, queue_size=1, latch=True,
        )
        self._cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # ── Subscribers ────────────────────────────────────────────────────
        rospy.Subscriber(
            "/optitrack/rosbot/pose", PoseStamped,
            self._ot_cb, queue_size=1,
        )
        for name in ("fl", "fr", "rl", "rr"):
            rospy.Subscriber(
                f"/range/{name}", Range,
                lambda msg, n=name: self._range_cb(msg, n),
                queue_size=1,
            )

    # ── Callbacks ──────────────────────────────────────────────────────────

    def _ot_cb(self, msg):
        with self._ot_lock:
            self._ot_pose = msg
        self._ot_event.set()

    def _range_cb(self, msg, name):
        # Values < min_range (including -1.0) mean no obstacle / out of range
        value = msg.range if msg.range >= msg.min_range else float("inf")
        with self._range_lock:
            self._range[name] = value

    # ── Status ─────────────────────────────────────────────────────────────

    def _pub_status(self, state, **kw):
        payload = {"state": state, **kw}
        self._status_pub.publish(json.dumps(payload))
        rospy.loginfo("[reset] %s", payload)

    # ── Pose helpers ───────────────────────────────────────────────────────

    @staticmethod
    def _to_xyyaw(pose_stamped):
        p = pose_stamped.pose
        q = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
        _, _, yaw = tft.euler_from_quaternion(q)
        return p.position.x, p.position.y, yaw

    def _pose(self):
        """Return (x, y, yaw) from latest OptiTrack reading, or None."""
        with self._ot_lock:
            msg = self._ot_pose
        return self._to_xyyaw(msg) if msg is not None else None

    def _fresh_pose(self, timeout=5.0):
        """Wait for a fresh OptiTrack message. Returns (x,y,yaw) or None."""
        self._ot_event.clear()
        if not self._ot_event.wait(timeout=timeout):
            return None
        return self._pose()

    # ── Range safety ───────────────────────────────────────────────────────

    def _forward_scale(self):
        """Speed scale [0, 1] based on front IR sensors."""
        with self._range_lock:
            front = min(self._range["fl"], self._range["fr"])
        if front <= self.FRONT_STOP_M:
            return 0.0
        if front < self.FRONT_SLOW_M:
            return (front - self.FRONT_STOP_M) / (self.FRONT_SLOW_M - self.FRONT_STOP_M)
        return 1.0

    # ── Helpers ────────────────────────────────────────────────────────────

    @staticmethod
    def _yaw_err(target, current):
        return math.atan2(math.sin(target - current), math.cos(target - current))

    def _stop(self):
        self._cmd_vel_pub.publish(Twist())

    # ── Phase 1 & 3: rotate in place ───────────────────────────────────────

    def _rotate_to(self, target_yaw, tolerance=None, timeout=20.0):
        """Spin to target_yaw using angular PID. Returns True on success."""
        if tolerance is None:
            tolerance = self.yaw_tolerance
        self._ang_pid.reset()
        rate = rospy.Rate(self.CTRL_HZ)
        deadline = time.time() + timeout

        while not rospy.is_shutdown() and time.time() < deadline:
            pose = self._pose()
            if pose is None:
                rospy.logwarn("[reset] OptiTrack lost during rotation")
                self._stop()
                return False

            err = self._yaw_err(target_yaw, pose[2])
            if abs(err) <= tolerance:
                self._stop()
                return True

            cmd = Twist()
            cmd.angular.z = self._ang_pid.compute(err)
            self._cmd_vel_pub.publish(cmd)
            rate.sleep()

        self._stop()
        rospy.logwarn("[reset] Rotation timed out (target=%.1f deg)", math.degrees(target_yaw))
        return False

    # ── Phase 2: drive to position ─────────────────────────────────────────

    # If heading error exceeds this during drive, stop and re-align before continuing
    REALIGN_THRESH = 0.35   # rad (~20 deg)

    def _drive_to(self, tx, ty):
        """Drive to (tx, ty). Stops to re-align if heading drifts. Returns True on success."""
        rate = rospy.Rate(self.CTRL_HZ)
        deadline = time.time() + self.nav_timeout

        while not rospy.is_shutdown() and time.time() < deadline:
            pose = self._pose()
            if pose is None:
                rospy.logwarn("[reset] OptiTrack lost during drive")
                self._stop()
                return False

            x, y, yaw = pose
            dx, dy = tx - x, ty - y
            dist = math.hypot(dx, dy)

            if dist <= self.pos_tolerance:
                self._stop()
                return True

            target_heading = math.atan2(dy, dx)
            heading_err = self._yaw_err(target_heading, yaw)

            # If heading is too far off, stop and rotate back on track
            if abs(heading_err) > self.REALIGN_THRESH:
                self._stop()
                self._rotate_to(target_heading, tolerance=0.15)
                continue

            # Heading is good: drive straight with gentle correction only
            range_scale = self._forward_scale()
            if range_scale == 0.0:
                rospy.logwarn_throttle(1.0, "[reset] Front obstacle — waiting")
                self._stop()
                rate.sleep()
                continue

            # Decel ramp: slow down as we approach
            decel_cap = min(self.max_linear_vel, dist * 0.5)
            lin_vel = decel_cap * range_scale
            ang_vel = max(-self.max_angular_vel * 0.4,
                          min(self.max_angular_vel * 0.4, heading_err * 0.6))

            cmd = Twist()
            cmd.linear.x = lin_vel
            cmd.angular.z = ang_vel
            self._cmd_vel_pub.publish(cmd)

            self._pub_status(
                "navigating",
                pos_error=round(dist, 3),
                yaw_error=round(abs(self._yaw_err(self.goal_yaw, yaw)), 3),
            )
            rate.sleep()

        self._stop()
        rospy.logwarn("[reset] Drive timed out")
        return False

    # ── Main ───────────────────────────────────────────────────────────────

    def run(self):
        """Single-shot go-to-goal. Returns True on success."""

        # 1. Wait for OptiTrack
        self._pub_status("waiting_for_optitrack")
        rospy.loginfo("[reset] Waiting for OptiTrack on /optitrack/rosbot/pose ...")
        if not self._ot_event.wait(timeout=30.0):
            self._pub_status("error", error="OptiTrack timeout (30s)")
            return False

        # 2. Already at start?
        pose = self._pose()
        x, y, yaw = pose
        pos_err = math.hypot(x - self.goal_x, y - self.goal_y)
        yaw_err = abs(self._yaw_err(self.goal_yaw, yaw))
        rospy.loginfo("[reset] Initial error: pos=%.3fm yaw=%.2frad", pos_err, yaw_err)

        if pos_err <= self.pos_tolerance and yaw_err <= self.yaw_tolerance:
            self._pub_status("success",
                             pos_error=round(pos_err, 3),
                             yaw_error=round(yaw_err, 3))
            return True

        # 3. One shot: rotate → drive → align
        if rospy.is_shutdown():
            return False

        # Phase 1: face the goal position
        heading = math.atan2(self.goal_y - y, self.goal_x - x)
        rospy.loginfo("[reset]  → rotate to heading %.1f°", math.degrees(heading))
        if not self._rotate_to(heading, tolerance=0.12):
            self._pub_status("failed", error="rotation_to_heading_failed")
            return False

        # Phase 2: drive there
        rospy.loginfo("[reset]  → drive to (%.2f, %.2f)", self.goal_x, self.goal_y)
        if not self._drive_to(self.goal_x, self.goal_y):
            self._pub_status("failed", error="drive_failed")
            return False

        # Phase 3: align to final yaw
        rospy.loginfo("[reset]  → align to yaw %.1f°", math.degrees(self.goal_yaw))
        if not self._rotate_to(self.goal_yaw):
            self._pub_status("failed", error="final_yaw_alignment_failed")
            return False

        # Check result with fresh OptiTrack reading
        pose = self._fresh_pose(timeout=5.0)
        if pose is None:
            self._pub_status("error", error="OptiTrack stale after nav")
            return False
        x, y, yaw = pose
        pos_err = math.hypot(x - self.goal_x, y - self.goal_y)
        yaw_err = abs(self._yaw_err(self.goal_yaw, yaw))
        rospy.loginfo("[reset] Final error: pos=%.3fm yaw=%.2frad", pos_err, yaw_err)

        if pos_err <= self.pos_tolerance and yaw_err <= self.yaw_tolerance:
            self._pub_status("success",
                             pos_error=round(pos_err, 3),
                             yaw_error=round(yaw_err, 3))
            rospy.loginfo("[reset] SUCCESS")
            return True

        self._pub_status("failed",
                         pos_error=round(pos_err, 3),
                         yaw_error=round(yaw_err, 3))
        rospy.logwarn("[reset] FAILED (pos=%.3fm yaw=%.2frad)", pos_err, yaw_err)
        return False


def main():
    rospy.init_node("reset_to_start")
    node = ResetToStart()
    success = node.run()
    if success:
        rospy.loginfo("[reset] Exiting with success")
    else:
        rospy.logwarn("[reset] Exiting with failure")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
