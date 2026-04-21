#!/usr/bin/env python3
"""Rotate the ROSbot in place to match the start orientation.

Uses pose.orientation.z from OptiTrack as the heading signal for both
the target (recorded at start position) and the current robot state.

Launched via:
    roslaunch foraging_bringup reset_to_start.launch
"""

import json
import math
import threading

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String


class ResetToStart:

    CTRL_HZ = 20

    def __init__(self):
        # orientation.z from the reference pose recorded at start position
        self.goal_orientation_z  = rospy.get_param("~goal_orientation_z", -0.4383017122745514)
        self.tolerance           = rospy.get_param("~tolerance",           0.01)   # in orientation.z units
        self.max_angular_vel     = rospy.get_param("~max_angular_vel",     0.40)
        self.k_ang               = rospy.get_param("~k_ang",               2.0)

        rospy.loginfo("[reset] Target orientation.z = %.4f  tolerance = %.4f",
                      self.goal_orientation_z, self.tolerance)

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

    def _get_orientation_z(self):
        with self._ot_lock:
            msg = self._ot_pose
        if msg is None:
            return None
        return msg.pose.orientation.z

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
            oz = self._get_orientation_z()
            if oz is None:
                self._stop()
                self._pub("error", error="OptiTrack lost")
                return

            err = self.goal_orientation_z - oz

            rospy.loginfo_throttle(
                0.5,
                "[reset] orientation.z = %.4f  target = %.4f  err = %.4f",
                oz, self.goal_orientation_z, err,
            )

            self._pub("rotating",
                      orientation_z=round(oz, 4),
                      target_z=round(self.goal_orientation_z, 4),
                      error_z=round(err, 4))

            if abs(err) <= self.tolerance:
                self._stop()
                self._pub("success",
                          orientation_z=round(oz, 4),
                          target_z=round(self.goal_orientation_z, 4),
                          error_z=round(err, 4))
                rospy.loginfo("[reset] Done — orientation matched.")
                return

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
