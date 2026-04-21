#!/usr/bin/env python3

import math
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from tf.transformations import euler_from_quaternion


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class GoToHardcodedGoal:
    def __init__(self):
        rospy.init_node("reset_to_start_node")

        self.pose_topic = rospy.get_param("~pose_topic", "/optitrack/rosbot/pose")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/cmd_vel")

        # Controller gains
        self.k_linear = rospy.get_param("~k_linear", 0.7)
        self.k_angular = rospy.get_param("~k_angular", 2.0)
        self.k_final_yaw = rospy.get_param("~k_final_yaw", 1.5)

        # Velocity limits
        self.max_linear = rospy.get_param("~max_linear", 0.10)
        self.max_angular = rospy.get_param("~max_angular", 0.5)

        # Tolerances
        self.pos_tolerance = rospy.get_param("~pos_tolerance", 0.05)
        self.yaw_tolerance = rospy.get_param("~yaw_tolerance", 0.05)

        # If heading error is large, rotate first
        self.heading_threshold = rospy.get_param("~heading_threshold", 0.30)

        # Hardcoded goal pose
        self.goal_x = 2.981062650680542
        self.goal_y = 1.8255693912506104
        self.goal_z = 0.431333988904953

        self.goal_qx = 0.04384467005729675
        self.goal_qy = -0.012470778077840805
        self.goal_qz = -0.4383017122745514
        self.goal_qw = 0.8976712822914124

        _, _, self.goal_yaw = euler_from_quaternion(
            [self.goal_qx, self.goal_qy, self.goal_qz, self.goal_qw]
        )

        self.current_pose = None
        self.goal_reached = False

        self.pose_sub = rospy.Subscriber(
            self.pose_topic, PoseStamped, self.pose_callback, queue_size=1
        )
        self.cmd_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=1)

        rospy.on_shutdown(self.stop_robot)

        rospy.loginfo(
            f"Hardcoded goal loaded: x={self.goal_x:.3f}, y={self.goal_y:.3f}, yaw={self.goal_yaw:.3f}"
        )

    def pose_callback(self, msg: PoseStamped):
        self.current_pose = msg

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def get_yaw_from_pose(self, pose_msg: PoseStamped) -> float:
        q = pose_msg.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return yaw

    def run(self):
        rate = rospy.Rate(20)

        while not rospy.is_shutdown():
            if self.current_pose is None:
                rospy.logwarn_throttle(2.0, "Waiting for current pose...")
                rate.sleep()
                continue

            x = self.current_pose.pose.position.x
            y = self.current_pose.pose.position.y
            yaw = self.get_yaw_from_pose(self.current_pose)

            dx = self.goal_x - x
            dy = self.goal_y - y
            distance = math.hypot(dx, dy)

            target_heading = math.atan2(dy, dx)
            heading_error = wrap_angle(target_heading - yaw)
            final_yaw_error = wrap_angle(self.goal_yaw - yaw)

            cmd = Twist()

            # Phase 1: go to target position
            if distance > self.pos_tolerance:
                self.goal_reached = False

                # Rotate first if not facing the goal enough
                if abs(heading_error) > self.heading_threshold:
                    cmd.linear.x = 0.0
                    cmd.angular.z = clamp(
                        self.k_angular * heading_error,
                        -self.max_angular,
                        self.max_angular,
                    )
                else:
                    cmd.linear.x = clamp(
                        self.k_linear * distance,
                        0.0,
                        self.max_linear,
                    )
                    cmd.angular.z = clamp(
                        self.k_angular * heading_error,
                        -self.max_angular,
                        self.max_angular,
                    )

            # Phase 2: position reached, align final yaw
            elif abs(final_yaw_error) > self.yaw_tolerance:
                cmd.linear.x = 0.0
                cmd.angular.z = clamp(
                    self.k_final_yaw * final_yaw_error,
                    -self.max_angular,
                    self.max_angular,
                )

            # Phase 3: done
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                if not self.goal_reached:
                    rospy.loginfo("Goal reached.")
                    self.goal_reached = True

            self.cmd_pub.publish(cmd)

            rospy.loginfo_throttle(
                1.0,
                f"current=(x={x:.3f}, y={y:.3f}, yaw={yaw:.3f}) | "
                f"goal=(x={self.goal_x:.3f}, y={self.goal_y:.3f}, yaw={self.goal_yaw:.3f}) | "
                f"dist={distance:.3f}, heading_err={heading_error:.3f}, final_yaw_err={final_yaw_error:.3f} | "
                f"cmd=(vx={cmd.linear.x:.3f}, wz={cmd.angular.z:.3f})"
            )

            rate.sleep()


if __name__ == "__main__":
    try:
        controller = GoToHardcodedGoal()
        controller.run()
    except rospy.ROSInterruptException:
        pass