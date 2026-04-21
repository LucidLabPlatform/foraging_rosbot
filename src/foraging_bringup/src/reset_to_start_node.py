#!/usr/bin/env python3
"""Navigate the ROSbot back to its starting position using OptiTrack.

Uses move_base (raw-topic interface, no actionlib handshake) in odom frame
for obstacle avoidance via LIDAR.  OptiTrack provides absolute positioning;
iterative correction compensates for odometry drift.

Launched via:
    roslaunch foraging_bringup reset_to_start.launch \
        start_x:=1.5 start_y:=2.0 start_yaw:=0.78

Or via LUCID MQTT roslaunch_start command with the same args.
"""

import json
import math
import threading
import time
import uuid

import rospy
import tf
import tf.transformations as tft
from actionlib_msgs.msg import GoalID, GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionResult
from std_msgs.msg import String


class ResetToStart:
    """OptiTrack-guided reset-to-start with iterative correction."""

    def __init__(self):
        # ── Parameters ────────────────────────────────────────────────────
        self.start_x = rospy.get_param("~start_x", 0.0)
        self.start_y = rospy.get_param("~start_y", 0.0)
        self.start_yaw = rospy.get_param("~start_yaw", 0.0)
        self.pos_tolerance = rospy.get_param("~pos_tolerance", 0.10)
        self.yaw_tolerance = rospy.get_param("~yaw_tolerance", 0.15)
        self.max_corrections = int(rospy.get_param("~max_corrections", 3))
        self.nav_timeout = rospy.get_param("~nav_timeout", 60.0)

        rospy.loginfo(
            "[reset] Start pose (OptiTrack): x=%.3f y=%.3f yaw=%.2f deg",
            self.start_x, self.start_y, math.degrees(self.start_yaw),
        )
        rospy.loginfo(
            "[reset] Tolerances: pos=%.3fm yaw=%.2f rad  max_corrections=%d",
            self.pos_tolerance, self.yaw_tolerance, self.max_corrections,
        )

        # ── TF listener (odom → base_link) ────────────────────────────────
        self.tf_listener = tf.TransformListener()

        # ── OptiTrack pose (updated by subscriber) ────────────────────────
        self._optitrack_pose = None
        self._optitrack_lock = threading.Lock()
        self._optitrack_event = threading.Event()

        # ── Status publisher (latched for MQTT bridge) ────────────────────
        self._status_pub = rospy.Publisher(
            "/reset_to_start/status", String, queue_size=1, latch=True,
        )

        # ── Navigating flag (set True while move_base is active) ─────────
        self._navigating = False

        # ── OptiTrack subscriber ──────────────────────────────────────────
        rospy.Subscriber(
            "/optitrack/rosbot/pose", PoseStamped,
            self._optitrack_cb, queue_size=1,
        )

        # ── move_base raw-topic interface ─────────────────────────────────
        # actionlib TCPROS handshake fails in distributed ROS setup;
        # use raw topics (same pattern as move_base_client.py).
        self._goal_pub = rospy.Publisher(
            "/move_base/goal", MoveBaseActionGoal, queue_size=1,
        )
        self._cancel_pub = rospy.Publisher(
            "/move_base/cancel", GoalID, queue_size=1,
        )
        self._current_goal_id = None
        self._result_event = threading.Event()
        self._result_status = None
        self._server_up = threading.Event()

        rospy.Subscriber(
            "/move_base/status", GoalStatusArray, self._status_cb,
        )
        rospy.Subscriber(
            "/move_base/result", MoveBaseActionResult, self._result_cb,
        )

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _optitrack_cb(self, msg):
        with self._optitrack_lock:
            self._optitrack_pose = msg
        self._optitrack_event.set()
        if self._navigating:
            pos_err, yaw_err = self._check_at_start(msg)
            self._publish_status(
                "navigating",
                pos_error=round(pos_err, 3),
                yaw_error=round(yaw_err, 3),
            )

    def _status_cb(self, _msg):
        if not self._server_up.is_set():
            self._server_up.set()

    def _result_cb(self, msg):
        if (
            self._current_goal_id is not None
            and msg.status.goal_id.id == self._current_goal_id
        ):
            self._result_status = msg.status.status
            self._result_event.set()

    # ── Status publishing ─────────────────────────────────────────────────

    def _publish_status(self, state, **extra):
        payload = {"state": state}
        payload.update(extra)
        self._status_pub.publish(json.dumps(payload))
        rospy.loginfo("[reset] %s", payload)

    # ── Pose helpers ──────────────────────────────────────────────────────

    @staticmethod
    def _pose_msg_to_xyyaw(pose_stamped):
        """Extract (x, y, yaw) from a PoseStamped."""
        p = pose_stamped.pose
        q = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
        _, _, yaw = tft.euler_from_quaternion(q)
        return p.position.x, p.position.y, yaw

    def _get_odom_pose(self):
        """Get robot pose in odom frame as (x, y, yaw). Returns None on failure."""
        try:
            self.tf_listener.waitForTransform(
                "odom", "base_link", rospy.Time(0), rospy.Duration(5.0),
            )
            trans, rot = self.tf_listener.lookupTransform(
                "odom", "base_link", rospy.Time(0),
            )
            _, _, yaw = tft.euler_from_quaternion(rot)
            return trans[0], trans[1], yaw
        except (
            tf.Exception,
            tf.LookupException,
            tf.ConnectivityException,
            tf.ExtrapolationException,
        ) as exc:
            rospy.logwarn("[reset] TF odom→base_link failed: %s", exc)
            return None

    # ── Coordinate transform ──────────────────────────────────────────────

    def _compute_start_in_odom(self, optitrack_msg):
        """Transform start pose from OptiTrack frame to odom frame.

        Given two corresponding readings of the same robot:
          OptiTrack: (ot_x, ot_y, ot_yaw)
          Odom:      (od_x, od_y, od_yaw)

        Compute a rigid-body transform T: OptiTrack → odom, then apply it
        to the start pose (sx, sy, syaw) in OptiTrack frame.

        Math:
          delta_yaw = od_yaw - ot_yaw
          v = (sx - ot_x, sy - ot_y)           # vector in OptiTrack frame
          v_rot = R(delta_yaw) * v              # rotate into odom frame
          goal = v_rot + (od_x, od_y)           # translate
          goal_yaw = syaw + delta_yaw
        """
        ot_x, ot_y, ot_yaw = self._pose_msg_to_xyyaw(optitrack_msg)

        odom = self._get_odom_pose()
        if odom is None:
            return None
        od_x, od_y, od_yaw = odom

        delta = od_yaw - ot_yaw
        vx = self.start_x - ot_x
        vy = self.start_y - ot_y

        cos_d = math.cos(delta)
        sin_d = math.sin(delta)
        goal_x = vx * cos_d - vy * sin_d + od_x
        goal_y = vx * sin_d + vy * cos_d + od_y
        goal_yaw = math.atan2(
            math.sin(self.start_yaw + delta),
            math.cos(self.start_yaw + delta),
        )

        rospy.loginfo(
            "[reset] Transform: OT=(%.2f,%.2f,%.0f°) Odom=(%.2f,%.2f,%.0f°) "
            "Δ=%.0f° → Goal_odom=(%.2f,%.2f,%.0f°)",
            ot_x, ot_y, math.degrees(ot_yaw),
            od_x, od_y, math.degrees(od_yaw),
            math.degrees(delta),
            goal_x, goal_y, math.degrees(goal_yaw),
        )
        return goal_x, goal_y, goal_yaw

    # ── Navigation ────────────────────────────────────────────────────────

    def _send_goal(self, x, y, yaw):
        """Publish a move_base goal in odom frame."""
        q = tft.quaternion_from_euler(0, 0, yaw)
        goal_id = "reset_%s" % uuid.uuid4().hex[:8]
        self._current_goal_id = goal_id
        self._result_event.clear()
        self._result_status = None

        msg = MoveBaseActionGoal()
        msg.header.stamp = rospy.Time.now()
        msg.goal_id.stamp = rospy.Time(0)
        msg.goal_id.id = goal_id
        msg.goal.target_pose.header.frame_id = "odom"
        msg.goal.target_pose.header.stamp = rospy.Time.now()
        msg.goal.target_pose.pose.position.x = x
        msg.goal.target_pose.pose.position.y = y
        msg.goal.target_pose.pose.orientation = Quaternion(*q)

        self._goal_pub.publish(msg)
        rospy.loginfo(
            "[reset] Goal sent: (%.2f, %.2f, yaw=%.0f°) in odom frame",
            x, y, math.degrees(yaw),
        )

    def _wait_for_result(self, timeout):
        """Wait for move_base result. Returns GoalStatus int."""
        deadline = time.time() + timeout
        while not self._result_event.is_set() and time.time() < deadline:
            if rospy.is_shutdown():
                self._cancel_goal()
                return GoalStatus.LOST
            self._result_event.wait(0.5)

        if self._result_event.is_set():
            return self._result_status

        rospy.logwarn("[reset] Navigation timed out (%.0fs)", timeout)
        self._cancel_goal()
        return GoalStatus.ABORTED

    def _cancel_goal(self):
        if self._current_goal_id:
            cancel_msg = GoalID()
            cancel_msg.id = self._current_goal_id
            self._cancel_pub.publish(cancel_msg)

    # ── Error checking ────────────────────────────────────────────────────

    def _check_at_start(self, optitrack_msg):
        """Return (pos_error, yaw_error) relative to start in OptiTrack frame."""
        x, y, yaw = self._pose_msg_to_xyyaw(optitrack_msg)
        pos_error = math.sqrt((x - self.start_x) ** 2 + (y - self.start_y) ** 2)
        yaw_error = abs(
            math.atan2(
                math.sin(yaw - self.start_yaw),
                math.cos(yaw - self.start_yaw),
            )
        )
        return pos_error, yaw_error

    def _get_fresh_optitrack(self, timeout=5.0):
        """Wait for a fresh OptiTrack reading. Returns PoseStamped or None."""
        self._optitrack_event.clear()
        if not self._optitrack_event.wait(timeout=timeout):
            return None
        with self._optitrack_lock:
            return self._optitrack_pose

    # ── Main algorithm ────────────────────────────────────────────────────

    def run(self):
        """Iterative reset-to-start. Returns True on success."""

        # 1. Wait for OptiTrack
        self._publish_status("waiting_for_optitrack")
        rospy.loginfo("[reset] Waiting for OptiTrack on /optitrack/rosbot/pose ...")
        if not self._optitrack_event.wait(timeout=30.0):
            self._publish_status("error", error="OptiTrack pose timeout (30s)")
            return False

        # 2. Wait for TF odom → base_link
        self._publish_status("waiting_for_tf")
        rospy.loginfo("[reset] Waiting for TF odom → base_link ...")
        try:
            self.tf_listener.waitForTransform(
                "odom", "base_link", rospy.Time(0), rospy.Duration(30.0),
            )
        except tf.Exception:
            self._publish_status("error", error="TF odom→base_link timeout (30s)")
            return False

        # 3. Wait for move_base
        self._publish_status("waiting_for_move_base")
        rospy.loginfo("[reset] Waiting for move_base ...")
        deadline = time.time() + 30.0
        while not self._server_up.is_set() and time.time() < deadline:
            if rospy.is_shutdown():
                return False
            self._server_up.wait(1.0)
        if not self._server_up.is_set():
            self._publish_status("error", error="move_base not available (30s)")
            return False

        # 4. Check if already at start
        with self._optitrack_lock:
            ot_pose = self._optitrack_pose
        pos_err, yaw_err = self._check_at_start(ot_pose)
        rospy.loginfo(
            "[reset] Initial error: pos=%.3fm yaw=%.2f rad", pos_err, yaw_err,
        )
        if pos_err <= self.pos_tolerance and yaw_err <= self.yaw_tolerance:
            self._publish_status(
                "success", pos_error=round(pos_err, 3), yaw_error=round(yaw_err, 3),
            )
            return True

        # 5. Iterative correction loop
        for attempt in range(1, self.max_corrections + 1):
            if rospy.is_shutdown():
                return False

            self._publish_status(
                "navigating",
                attempt=attempt,
                max_attempts=self.max_corrections,
                pos_error=round(pos_err, 3),
                yaw_error=round(yaw_err, 3),
            )

            # 5a. Fresh OptiTrack reading
            ot_pose = self._get_fresh_optitrack(timeout=5.0)
            if ot_pose is None:
                self._publish_status(
                    "error", error="OptiTrack stale", attempt=attempt,
                )
                return False

            # 5b. Compute start in odom frame
            start_odom = self._compute_start_in_odom(ot_pose)
            if start_odom is None:
                self._publish_status(
                    "error", error="TF lookup failed", attempt=attempt,
                )
                return False

            # 5c. Navigate
            self._send_goal(*start_odom)
            self._navigating = True
            status = self._wait_for_result(self.nav_timeout)
            self._navigating = False

            if status == GoalStatus.SUCCEEDED:
                rospy.loginfo("[reset] move_base succeeded on attempt %d", attempt)
            else:
                rospy.logwarn(
                    "[reset] move_base status=%d on attempt %d (continuing)",
                    status, attempt,
                )

            # 5d. Settle
            rospy.sleep(1.0)

            # 5e. Check OptiTrack
            ot_pose = self._get_fresh_optitrack(timeout=5.0)
            if ot_pose is None:
                self._publish_status(
                    "error", error="OptiTrack stale after nav", attempt=attempt,
                )
                return False

            pos_err, yaw_err = self._check_at_start(ot_pose)
            rospy.loginfo(
                "[reset] Attempt %d: pos_err=%.3fm yaw_err=%.2f rad",
                attempt, pos_err, yaw_err,
            )

            if pos_err <= self.pos_tolerance and yaw_err <= self.yaw_tolerance:
                self._publish_status(
                    "success",
                    attempt=attempt,
                    pos_error=round(pos_err, 3),
                    yaw_error=round(yaw_err, 3),
                )
                rospy.loginfo("[reset] SUCCESS after %d attempt(s)", attempt)
                return True

        # Exhausted all attempts
        self._publish_status(
            "failed",
            pos_error=round(pos_err, 3),
            yaw_error=round(yaw_err, 3),
            attempts_used=self.max_corrections,
        )
        rospy.logwarn(
            "[reset] FAILED after %d attempts (pos=%.3f yaw=%.2f)",
            self.max_corrections, pos_err, yaw_err,
        )
        return False


def main():
    rospy.init_node("reset_to_start")
    node = ResetToStart()
    success = node.run()
    # Node exits — roslaunch detects this, ros_bridge sets state to "exited"
    if success:
        rospy.loginfo("[reset] Exiting with success")
    else:
        rospy.logwarn("[reset] Exiting with failure")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
