#!/usr/bin/env python3
"""
ROS service server that picks up a puck in the center of the robot's field of view.
Accepts distance and speed parameters, moves forward, and activates the gripper.
"""

import rospy
from std_msgs.msg import UInt16
from geometry_msgs.msg import Twist
from foraging_msgs.srv import PickPuckServerMessage, PickPuckServerMessageResponse

GRIPPER_OPEN = 0
GRIPPER_CLOSE = 140


def move_forward(distance=0.2, speed=0.1):
    if speed <= 0:
        rospy.logerr("Speed must be > 0")
        return

    duration = distance / speed
    cmd_vel_msg = Twist()
    rate = rospy.Rate(10)

    rospy.loginfo(f"Moving forward {distance} m at {speed} m/s for {duration:.2f} s")

    start_time = rospy.Time.now()

    while not rospy.is_shutdown() and (rospy.Time.now() - start_time).to_sec() < duration:
        cmd_vel_msg.linear.x = speed
        cmd_vel_pub.publish(cmd_vel_msg)
        rate.sleep()

    cmd_vel_msg.linear.x = 0.0
    cmd_vel_pub.publish(cmd_vel_msg)
    rospy.loginfo("Stopped")


def open_gripper():
    gripper_msg = UInt16()
    gripper_msg.data = GRIPPER_OPEN
    gripper_pub.publish(gripper_msg)


def close_gripper():
    gripper_msg = UInt16()
    gripper_msg.data = GRIPPER_CLOSE
    gripper_pub.publish(gripper_msg)


def handle_pick_puck(req):
    try:
        rospy.loginfo(f"Pick puck service called — distance={req.distance}, speed={req.speed}")

        rospy.loginfo("Opening gripper...")
        open_gripper()

        rospy.loginfo("Moving forward...")
        move_forward(req.distance, req.speed)

        rospy.sleep(1.0)

        rospy.loginfo("Closing gripper...")
        close_gripper()

        rospy.loginfo("Puck picked up successfully")
        return PickPuckServerMessageResponse(success=True)

    except Exception as e:
        rospy.logerr(f"Pick puck service failed: {e}")
        return PickPuckServerMessageResponse(success=False)


def main():
    global gripper_pub, cmd_vel_pub

    rospy.init_node('pick_puck_server')
    rospy.loginfo("Pick puck server node started")

    gripper_pub = rospy.Publisher('servo', UInt16, queue_size=10)
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    rospy.sleep(1.0)

    service = rospy.Service('pick_puck', PickPuckServerMessage, handle_pick_puck)
    rospy.loginfo("Pick puck service ready")

    rospy.spin()


if __name__ == '__main__':
    main()
