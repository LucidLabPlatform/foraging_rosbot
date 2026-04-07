#!/usr/bin/env python3
"""Shared move_base client with robust startup handling.

Both tidy_room and random_walk use this instead of rolling their own
action client with different timeout logic.
"""

import rospy
import tf
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion


class MoveBaseClient:
    """Robust wrapper around the move_base action client.

    Usage:
        nav = MoveBaseClient()
        if not nav.wait_until_ready():
            rospy.logfatal("Navigation unavailable")
            return
        nav.go_to(1.0, 2.0, yaw=0.5)
    """

    def __init__(self):
        self._tf = tf.TransformListener()
        self._client = None

    def wait_until_ready(self, tf_timeout=90.0, server_timeout=30.0):
        """Block until TF map->base_link and move_base action server are up.
        Each phase has its own timeout. Returns True when ready, False on timeout."""

        # Phase 1: wait for TF (gmapping can take 30-60s to start publishing)
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

        # Phase 2: connect to move_base action server (full timeout, independent of phase 1)
        rospy.loginfo("[nav] Connecting to move_base (%.0fs timeout) ...", server_timeout)
        self._client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        if not self._client.wait_for_server(rospy.Duration(server_timeout)):
            rospy.logerr("[nav] move_base not available after %.0fs", server_timeout)
            return False

        rospy.loginfo("[nav] move_base ready.")
        return True

    def go_to(self, x, y, yaw=0.0, timeout=30.0):
        """Navigate to (x, y, yaw) in map frame. Returns True on success."""
        if self._client is None:
            rospy.logerr("[nav] Not connected -- call wait_until_ready() first")
            return False

        q = tf.transformations.quaternion_from_euler(0, 0, yaw)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation = Quaternion(*q)

        self._client.send_goal(goal)
        finished = self._client.wait_for_result(rospy.Duration(timeout))
        if not finished:
            self._client.cancel_goal()
            rospy.logwarn("[nav] Goal timed out (%.0fs)", timeout)
            return False

        return self._client.get_state() == GoalStatus.SUCCEEDED

    def cancel(self):
        """Cancel the current navigation goal."""
        if self._client:
            self._client.cancel_goal()

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
