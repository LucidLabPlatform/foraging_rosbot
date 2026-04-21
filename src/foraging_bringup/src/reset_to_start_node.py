#!/usr/bin/env python3
"""Rotate the ROSbot in place until it faces the start position.

Uses OptiTrack for both position (where the robot is) and orientation
(which way it is currently facing). Rotates until the robot's heading
points toward (goal_x, goal_y) within yaw_tolerance radians.

Launched via:
    roslaunch foraging_bringup reset_to_start.launch \
        goal_x:=2.98 goal_y:=1.83
"""

import json
import math
import threading

import rospy
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String


class ResetToStart:

    CTRL_HZ = 20

    def __init__(self):
        self.goal_x          = rospy.get_param("~goal_x",          2.9812698364257812)
        self.goal_y          = rospy.get_param("~goal_y",          1.8299084901809692)
        self.yaw_tolerance   = rospy.get_param("~yaw_tolerance",   0.10)   # ~6 deg
        self.max_angular_vel = rospy.get_param("~max_angular_vel", 0.40)
        self.k_ang           = rospy.get_param("~k_ang",           1.0)

        self._ot_pose  = None
        self._ot_lock  = threading.Lock()
        self._ot_event = threading.Event()

        self._status_pub  = rospy.Publisher("/reset_to_start/status", String, queue_size=1, latch=True)
        self._cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        rospy.Subscriber("/optitrack/rosbot/pose", PoseStamped, self._ot_cb, queue_size=1)

    def _ot_cb(self, msg):
        with self._ot_lock:
            self._ot_pose = msg
        self._ot_event.set()

    def _pose(self):
        """Return (x, y, yaw_rad) from the latest OptiTrack message."""
        with self._ot_lock:
            msg = self._ot_pose
        if msg is None:
            return None
        p = msg.pose
        q = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
        _, _, yaw = tft.euler_from_quaternion(q)
        return p.position.x, p.position.y, yaw

    @staticmethod
    def _wrap(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def _stop(self):
        self._cmd_vel_pub.publish(Twist())

    def _pub(self, state, **kw):
        payload = {"state": state, **kw}
        self._status_pub.publish(json.dumps(payload))
        rospy.loginfo("[reset] %s", payload)

    def run(self):
        rospy.loginfo("[reset] Waiting for OptiTrack...")
        self._pub("waiting_for_optitrack")

        if not self._ot_event.wait(timeout=30.0):
            self._pub("error", error="OptiTrack timeout")
            return

        rate = rospy.Rate(self.CTRL_HZ)

        while not rospy.is_shutdown():
            pose = self._pose()
            if pose is None:
                self._stop()
                self._pub("error", error="OptiTrack lost")
                return

            x, y, yaw = pose

            # Direction from current position toward the goal
            target_heading = math.atan2(self.goal_y - y, self.goal_x - x)
            err = self._wrap(target_heading - yaw)

            rospy.loginfo_throttle(
                0.5,
                "[reset] pos=(%.3f, %.3f)  yaw=%.1f°  target=%.1f°  err=%.1f°",
                x, y,
                math.degrees(yaw),
                math.degrees(target_heading),
                math.degrees(err),
            )

            self._pub("rotating",
                      current_yaw_deg=round(math.degrees(yaw), 1),
                      target_heading_deg=round(math.degrees(target_heading), 1),
                      heading_error_deg=round(math.degrees(err), 1))

            if abs(err) <= self.yaw_tolerance:
                self._stop()
                self._pub("success",
                          current_yaw_deg=round(math.degrees(yaw), 1),
                          target_heading_deg=round(math.degrees(target_heading), 1),
                          heading_error_deg=round(math.degrees(err), 1))
                rospy.loginfo("[reset] Done — robot is facing the start position.")
                return

            # Proportional rotation, capped to max_angular_vel
            cmd = Twist()
            cmd.angular.z = max(-self.max_angular_vel,
                                min(self.max_angular_vel, self.k_ang * err))
            self._cmd_vel_pub.publish(cmd)
            rate.sleep()


def main():
    rospy.init_node("reset_to_start")
    node = ResetToStart()
    node.run()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
