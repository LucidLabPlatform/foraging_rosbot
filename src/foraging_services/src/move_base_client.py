#!/usr/bin/env python3
"""Shared move_base client using raw topics (no actionlib handshake).

actionlib's 5-topic TCPROS handshake fails reliably on our distributed
ROS setup (master on 10.205.3.25, robot on 10.205.3.125). This module
bypasses it entirely: publish goals to /move_base/goal, subscribe to
/move_base/result and /move_base/status — plain ROS topics that work.
"""

import math
import threading
import time
import uuid

import rospy
import tf
from actionlib_msgs.msg import GoalID, GoalStatus, GoalStatusArray
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionResult, MoveBaseGoal
from geometry_msgs.msg import Quaternion

_STATUS_NAMES = {
    GoalStatus.PENDING:   "PENDING",
    GoalStatus.ACTIVE:    "ACTIVE",
    GoalStatus.PREEMPTED: "PREEMPTED",
    GoalStatus.SUCCEEDED: "SUCCEEDED",
    GoalStatus.ABORTED:   "ABORTED",
    GoalStatus.REJECTED:  "REJECTED",
    GoalStatus.LOST:      "LOST",
}


class MoveBaseClient:
    """Raw-topic wrapper around move_base (no actionlib).

    Usage:
        nav = MoveBaseClient()
        if not nav.wait_until_ready():
            rospy.logfatal("Navigation unavailable")
            return
        nav.go_to(1.0, 2.0, yaw=0.5)
    """

    def __init__(self):
        self._tf = tf.TransformListener()

        # Publishers
        self._goal_pub = rospy.Publisher(
            "/move_base/goal", MoveBaseActionGoal, queue_size=1)
        self._cancel_pub = rospy.Publisher(
            "/move_base/cancel", GoalID, queue_size=1)

        # State tracking
        self._current_goal_id = None
        self._result_event = threading.Event()
        self._result_status = None
        self._server_up = threading.Event()
        self._shutting_down = False

        # Subscribers
        rospy.Subscriber(
            "/move_base/status", GoalStatusArray, self._status_cb)
        rospy.Subscriber(
            "/move_base/result", MoveBaseActionResult, self._result_cb)

        rospy.on_shutdown(self._on_shutdown)

    def _status_cb(self, msg):
        """Any status message means the action server is alive."""
        if not self._server_up.is_set():
            self._server_up.set()

    def _result_cb(self, msg):
        """Capture result for our current goal."""
        if (self._current_goal_id is not None and
                msg.status.goal_id.id == self._current_goal_id):
            self._result_status = msg.status.status
            self._result_event.set()

    def _on_shutdown(self):
        self._shutting_down = True
        self.cancel()
        self._result_event.set()
        self._server_up.set()

    def wait_until_ready(self, tf_timeout=90.0, server_timeout=90.0):
        """Block until TF map->base_link and move_base status topic are up.
        Returns True when ready, False on timeout."""

        # Phase 1: wait for TF
        rospy.loginfo("[nav] Waiting for TF map -> base_link ...")
        tf_deadline = rospy.Time.now() + rospy.Duration(tf_timeout)
        while not rospy.is_shutdown() and rospy.Time.now() < tf_deadline:
            try:
                self._tf.waitForTransform(
                    "map", "base_link", rospy.Time(0), rospy.Duration(5.0))
                rospy.loginfo("[nav] TF ready.")
                break
            except (tf.Exception, tf.LookupException, tf.ConnectivityException):
                rospy.logwarn("[nav] TF not available yet, retrying ...")
        else:
            if not rospy.is_shutdown():
                rospy.logerr("[nav] TF not available after %.0fs", tf_timeout)
            return False

        # Phase 2: wait for move_base /status topic (proves server is alive)
        rospy.loginfo("[nav] Waiting for move_base status (%.0fs timeout) ...",
                      server_timeout)
        deadline = time.time() + server_timeout
        while not self._server_up.is_set() and time.time() < deadline:
            if self._shutting_down:
                return False
            self._server_up.wait(1.0)
        if self._server_up.is_set() and not self._shutting_down:
            rospy.loginfo("[nav] move_base is up.")
            return True

        rospy.logerr("[nav] move_base status not received after %.0fs",
                     server_timeout)
        return False

    def go_to(self, x, y, yaw=0.0, timeout=30.0):
        """Navigate to (x, y, yaw) in map frame. Returns True on success."""
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)

        goal_id = "goal_%s" % uuid.uuid4().hex[:8]
        self._current_goal_id = goal_id
        self._result_event.clear()
        self._result_status = None

        msg = MoveBaseActionGoal()
        msg.header.stamp = rospy.Time.now()
        msg.goal_id.stamp = rospy.Time.now()
        msg.goal_id.id = goal_id
        msg.goal.target_pose.header.frame_id = "map"
        msg.goal.target_pose.header.stamp = rospy.Time.now()
        msg.goal.target_pose.pose.position.x = x
        msg.goal.target_pose.pose.position.y = y
        msg.goal.target_pose.pose.orientation = Quaternion(*q)

        self._goal_pub.publish(msg)
        t_start = time.time()
        rospy.loginfo("[nav] Goal %s sent: (%.2f, %.2f, yaw=%.2f)",
                      goal_id, x, y, yaw)

        # Poll so we can exit promptly on shutdown
        deadline = time.time() + timeout
        while not self._result_event.is_set() and time.time() < deadline:
            if self._shutting_down:
                self.cancel()
                return False
            self._result_event.wait(0.5)

        if self._result_event.is_set() and not self._shutting_down:
            elapsed = time.time() - t_start
            if self._result_status == GoalStatus.SUCCEEDED:
                rospy.loginfo("[nav] Goal %s succeeded in %.1fs.", goal_id, elapsed)
                return True
            else:
                status_name = _STATUS_NAMES.get(self._result_status,
                                                str(self._result_status))
                pos = self.get_robot_position()
                if pos:
                    dist_remaining = math.sqrt((x - pos[0])**2 + (y - pos[1])**2)
                    rospy.logwarn(
                        "[nav] Goal %s FAILED — status=%s elapsed=%.1fs "
                        "robot=(%.2f, %.2f) goal=(%.2f, %.2f) dist_remaining=%.2fm%s",
                        goal_id, status_name, elapsed,
                        pos[0], pos[1], x, y, dist_remaining,
                        " (likely: no valid plan)" if elapsed < 2.0 else
                        " (likely: controller could not execute plan)" if elapsed >= 9.0
                        else "")
                else:
                    rospy.logwarn(
                        "[nav] Goal %s FAILED — status=%s elapsed=%.1fs "
                        "goal=(%.2f, %.2f) (robot position unavailable)",
                        goal_id, status_name, elapsed, x, y)
                return False

        # Our timeout expired — cancel the goal
        elapsed = time.time() - t_start
        self.cancel()
        pos = self.get_robot_position()
        if pos:
            dist_remaining = math.sqrt((x - pos[0])**2 + (y - pos[1])**2)
            rospy.logwarn(
                "[nav] Goal %s TIMED OUT (%.0fs) — "
                "robot=(%.2f, %.2f) goal=(%.2f, %.2f) dist_remaining=%.2fm",
                goal_id, elapsed, pos[0], pos[1], x, y, dist_remaining)
        else:
            rospy.logwarn("[nav] Goal %s TIMED OUT (%.0fs) — goal=(%.2f, %.2f)",
                          goal_id, elapsed, x, y)
        return False

    def cancel(self):
        """Cancel the current navigation goal."""
        if self._current_goal_id:
            cancel_msg = GoalID()
            cancel_msg.id = self._current_goal_id
            self._cancel_pub.publish(cancel_msg)

    def get_robot_position(self):
        """Returns (x, y) in map frame, or None."""
        try:
            self._tf.waitForTransform(
                "map", "base_link", rospy.Time(0), rospy.Duration(1.0))
            (trans, _) = self._tf.lookupTransform("map", "base_link", rospy.Time(0))
            return (trans[0], trans[1])
        except (tf.Exception, tf.LookupException,
                tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("[nav] TF lookup failed: %s", e)
            return None

    def get_robot_pose(self):
        """Returns (x, y, yaw) in map frame, or None."""
        try:
            self._tf.waitForTransform(
                "map", "base_link", rospy.Time(0), rospy.Duration(1.0))
            (trans, rot) = self._tf.lookupTransform("map", "base_link", rospy.Time(0))
            _, _, yaw = tf.transformations.euler_from_quaternion(rot)
            return (trans[0], trans[1], yaw)
        except (tf.Exception, tf.LookupException,
                tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("[nav] TF lookup failed: %s", e)
            return None
