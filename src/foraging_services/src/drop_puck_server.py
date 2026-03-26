#!/usr/bin/env python3
"""
ROS service server that drops a puck currently in the robot's gripper to the ground.
Opens the gripper and moves backward to drop-off the puck.
"""

import math
import rospy
from std_msgs.msg import UInt16
from geometry_msgs.msg import Twist
from foraging_msgs.srv import DropPuckServerMessage, DropPuckServerMessageResponse

GRIPPER_OPEN  = 0
GRIPPER_CLOSE = 170
TURN_SPEED    = 1.0   # rad/s for the 180° turn


def move_backward(distance=0.1, speed=0.1):
    if speed <= 0:
        rospy.logerr("Speed must be > 0")
        return

    duration = distance / speed
    cmd_vel_msg = Twist()
    rate = rospy.Rate(10)

    rospy.loginfo(f"Moving backward {distance} m at {speed} m/s for {duration:.2f} s")

    start_time = rospy.Time.now()

    while not rospy.is_shutdown() and (rospy.Time.now() - start_time).to_sec() < duration:
        cmd_vel_msg.linear.x = -speed
        cmd_vel_pub.publish(cmd_vel_msg)
        rate.sleep()

    cmd_vel_msg.linear.x = 0.0  # Stop the robot
    cmd_vel_pub.publish(cmd_vel_msg)
    rospy.loginfo("Robot stopped")


def open_gripper():
    gripper_msg = UInt16()
    gripper_msg.data = GRIPPER_OPEN
    gripper_pub.publish(gripper_msg)


def close_gripper():
    gripper_msg = UInt16()
    gripper_msg.data = GRIPPER_CLOSE
    gripper_pub.publish(gripper_msg)


def turn_180():
    duration = math.pi / TURN_SPEED
    twist = Twist()
    twist.angular.z = TURN_SPEED
    rate = rospy.Rate(10)
    start = rospy.Time.now()
    rospy.loginfo("Turning 180°...")
    while not rospy.is_shutdown() and (rospy.Time.now() - start).to_sec() < duration:
        cmd_vel_pub.publish(twist)
        rate.sleep()
    cmd_vel_pub.publish(Twist())
    rospy.loginfo("Turn complete")


def handle_drop_puck(req):
    try:
        rospy.loginfo(f"Drop puck service called — distance={req.distance}, speed={req.speed}")

        rospy.loginfo("Opening gripper...")
        open_gripper()

        rospy.loginfo("Moving backward...")
        move_backward(req.distance, req.speed)

        rospy.sleep(1.0)

        rospy.loginfo("Closing gripper...")
        close_gripper()

        turn_180()

        rospy.loginfo("Puck dropped successfully")
        return DropPuckServerMessageResponse(success=True)

    except Exception as e:
        rospy.logerr(f"Drop puck service failed: {e}")
        return DropPuckServerMessageResponse(success=False)


def main():
    global gripper_pub, cmd_vel_pub

    rospy.init_node('drop_puck_server')
    rospy.loginfo("Drop puck server node started")

    gripper_pub = rospy.Publisher('servo', UInt16, queue_size=10)
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    rospy.sleep(1.0)

    service = rospy.Service('drop_puck', DropPuckServerMessage, handle_drop_puck)
    rospy.loginfo("Drop puck service ready")

    rospy.spin()


if __name__ == '__main__':
    main()
