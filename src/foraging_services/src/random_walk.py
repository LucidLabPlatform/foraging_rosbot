#!/usr/bin/env python3
"""
ROS service server that performs a random walk.
Navigates the robot around the arena until target puck and ArUco corner counts are met,
then returns done=True.

Each step:
  1. Choose a random orientation
  2. Rotate the robot in place to face that direction (cmd_vel)
  3. Send a move_base goal one step ahead — if move_base can plan and reach it, we moved
  4. If move_base returns ABORTED (no valid path), try a new random orientation
"""

import rospy
import math
import random
import threading
from foraging_msgs.msg import PuckRegistry, ArucoRegistry
from foraging_msgs.srv import RandomWalkServerMessage, RandomWalkServerMessageResponse
from geometry_msgs.msg import Twist
from move_base_client import MoveBaseClient

# ── Parameters ────────────────────────────────────────────────────────────────
STEP_SIZE_DEFAULT     = 0.7   # meters per step (overridable via ~step_size param)
STEP_TIMEOUT          = 10    # seconds — timeout for each move_base goal
MAX_ORIENTATION_TRIES = 12    # max attempts to find a free direction per step
ANGULAR_SPEED         = 0.6   # rad/s for in-place rotation
YAW_TOLERANCE         = 0.1   # rad — acceptable error when rotating to target yaw
ROTATE_TIMEOUT        = 8.0   # seconds — max time to complete an in-place rotation
PAUSE_AFTER_STEP      = 5.0   # seconds to pause after each step to allow puck detection


class RandomWalkServer:
    def __init__(self):
        self._puck_count: int = 0
        self._corner_count: int = 0
        self._count_lock = threading.Lock()

        # Correlated random walk: when walk_sigma > 0, successive turn angles
        # are drawn from N(previous_heading, sigma) instead of uniform.
        # This produces a CPFA-style correlated random walk (CRW).
        self._walk_sigma = rospy.get_param('~walk_sigma', 0.0)
        self._step_size = rospy.get_param('~step_size', STEP_SIZE_DEFAULT)
        self._last_heading = None

        # Subscribers
        rospy.Subscriber("/puck/registry", PuckRegistry, self._puck_registry_cb, queue_size=1)
        rospy.Subscriber("/aruco/registry", ArucoRegistry, self._aruco_registry_cb, queue_size=1)

        # cmd_vel publisher (for in-place rotation)
        self._cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # Navigation client (handles TF + move_base connection)
        self._nav = MoveBaseClient()
        if not self._nav.wait_until_ready():
            rospy.logfatal("move_base not available — random walk cannot start")
            rospy.signal_shutdown("move_base not available")
            return

        # Service server
        self._service = rospy.Service("random_walk", RandomWalkServerMessage, self.handle_random_walk)
        rospy.loginfo("Random walk service ready.")

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _puck_registry_cb(self, msg: PuckRegistry) -> None:
        with self._count_lock:
            self._puck_count = len(msg.pucks)

    def _aruco_registry_cb(self, msg: ArucoRegistry) -> None:
        with self._count_lock:
            self._corner_count = len(msg.markers)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _rotate_to_yaw(self, target_yaw: float) -> bool:
        """Rotate in place to face target_yaw (world frame) using cmd_vel. Returns True on success."""
        rate = rospy.Rate(20)
        deadline = rospy.Time.now() + rospy.Duration(ROTATE_TIMEOUT)
        twist = Twist()

        while not rospy.is_shutdown():
            if rospy.Time.now() > deadline:
                twist.angular.z = 0.0
                self._cmd_vel.publish(twist)
                rospy.logwarn("rotate_to_yaw timed out")
                return False

            pose = self._nav.get_robot_pose()
            if pose is None:
                rate.sleep()
                continue

            _, _, current_yaw = pose
            error = math.atan2(math.sin(target_yaw - current_yaw),
                               math.cos(target_yaw - current_yaw))

            if abs(error) < YAW_TOLERANCE:
                twist.angular.z = 0.0
                self._cmd_vel.publish(twist)
                return True

            twist.angular.z = ANGULAR_SPEED if error > 0 else -ANGULAR_SPEED
            self._cmd_vel.publish(twist)
            rate.sleep()

        twist.angular.z = 0.0
        self._cmd_vel.publish(twist)
        return False

    # ── Service handler ───────────────────────────────────────────────────────

    def handle_random_walk(self, req: RandomWalkServerMessage) -> RandomWalkServerMessageResponse:
        num_pucks = req.num_pucks_to_find
        num_corners = req.num_corners_to_find

        rospy.loginfo(
            f"Random walk requested — target pucks={num_pucks}, target corners={num_corners}"
        )

        # 1. Already at threshold?
        with self._count_lock:
            puck_count = self._puck_count
            corner_count = self._corner_count
        if puck_count >= num_pucks and corner_count >= num_corners:
            rospy.loginfo("Thresholds already met — returning done=True immediately.")
            return RandomWalkServerMessageResponse(done=True)

        step = 0

        while not rospy.is_shutdown():
            # Check thresholds
            with self._count_lock:
                puck_count = self._puck_count
                corner_count = self._corner_count
            if puck_count >= num_pucks and corner_count >= num_corners:
                rospy.loginfo(
                    f"Thresholds met — pucks={puck_count}/{num_pucks}, "
                    f"corners={corner_count}/{num_corners}. Done."
                )
                return RandomWalkServerMessageResponse(done=True)

            step += 1
            if step % 5 == 0:
                rospy.loginfo(
                    f"Step {step} — pucks={puck_count}/{num_pucks}, "
                    f"corners={corner_count}/{num_corners}"
                )

            pose = self._nav.get_robot_pose()
            if pose is None:
                rospy.sleep(0.5)
                continue
            robot_x, robot_y, robot_yaw = pose

            # Try to find a free direction: random angle → rotate → let move_base check path
            direction_found = False
            for attempt in range(MAX_ORIENTATION_TRIES):
                # Early exit if thresholds met during step
                with self._count_lock:
                    if self._puck_count >= num_pucks and self._corner_count >= num_corners:
                        rospy.loginfo(
                            f"Thresholds met mid-step — pucks={self._puck_count}/{num_pucks}, "
                            f"corners={self._corner_count}/{num_corners}. Done."
                        )
                        return RandomWalkServerMessageResponse(done=True)

                if self._walk_sigma > 0 and self._last_heading is not None:
                    delta = random.gauss(0, self._walk_sigma)
                    world_angle = self._last_heading + delta
                else:
                    world_angle = random.uniform(-math.pi, math.pi) + robot_yaw
                goal_x = robot_x + self._step_size * math.cos(world_angle)
                goal_y = robot_y + self._step_size * math.sin(world_angle)

                # Rotate robot to face this direction
                if not self._rotate_to_yaw(world_angle):
                    rospy.logdebug("Step %d attempt %d: rotation failed, trying new angle",
                                   step, attempt + 1)
                    continue

                # Refresh pose after rotation
                pose = self._nav.get_robot_pose()
                if pose is not None:
                    robot_x, robot_y, _ = pose
                    goal_x = robot_x + self._step_size * math.cos(world_angle)
                    goal_y = robot_y + self._step_size * math.sin(world_angle)

                # Send goal — move_base returns ABORTED if no valid path can be planned
                rospy.logdebug(
                    f"Step {step} attempt {attempt + 1}: "
                    f"angle={math.degrees(world_angle - robot_yaw):.1f}°, "
                    f"goal=({goal_x:.2f}, {goal_y:.2f})"
                )
                success = self._nav.go_to(goal_x, goal_y, yaw=world_angle, timeout=STEP_TIMEOUT)
                if success:
                    direction_found = True
                    self._last_heading = world_angle
                    rospy.sleep(PAUSE_AFTER_STEP)
                    break
                else:
                    rospy.logdebug(
                        "Step %d attempt %d: move_base failed, trying new angle",
                        step, attempt + 1
                    )

            if not direction_found:
                rospy.logwarn(
                    f"No valid direction found after {MAX_ORIENTATION_TRIES} tries — waiting 1 s."
                )
                rospy.sleep(1.0)

        # Shutdown before thresholds met
        return RandomWalkServerMessageResponse(done=False)


def main():
    rospy.init_node("random_walk_server")
    rospy.loginfo("Random walk server node started.")
    RandomWalkServer()
    rospy.spin()


if __name__ == "__main__":
    main()
