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
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetPlan, GetPlanRequest
from move_base_client import MoveBaseClient

# ── Parameters ────────────────────────────────────────────────────────────────
STEP_SIZE_DEFAULT     = 0.5   # meters per step (overridable via ~step_size param)   # <-- Decrease for better random walk?
STEP_TIMEOUT          = 15    # seconds — timeout for each move_base goal            # <-- Decrease for better random walk?
MAX_ORIENTATION_TRIES = 4     # max attempts to find a free direction per step       # <-- Decrease for better random walk?
ANGULAR_SPEED         = 0.6   # rad/s for in-place rotation
YAW_TOLERANCE         = 0.1   # rad — acceptable error when rotating to target yaw
ROTATE_TIMEOUT        = 10.0   # seconds — max time to complete an in-place rotation
PAUSE_AFTER_STEP      = 2.5   # seconds to pause after each step to allow puck detection

# ── Cell-based explorer parameters ───────────────────────────────────────────
# These drive the deterministic visited-grid explorer. The grid is anchored at
# world origin (map frame). A cell is "free" when its center sits inside the
# move_base global costmap with cost in [0, cost_free_threshold).
CELL_SIZE_DEFAULT               = 0.5    # meters per grid cell side
SEARCH_RADIUS_CELLS_DEFAULT     = 2      # initial Chebyshev radius around robot cell
SEARCH_RADIUS_MAX_CELLS_DEFAULT = 8      # max R before _visited reset
COST_FREE_THRESHOLD_DEFAULT     = 50     # costmap cost < this = free; >= = blocked
COSTMAP_WAIT_TIMEOUT_S_DEFAULT  = 10.0   # seconds to wait for first costmap msg
UNREACHABLE_COOLDOWN_S_DEFAULT  = 30.0   # seconds a failed cell stays banned
POSE_TICK_HZ_DEFAULT            = 5.0    # visited-tracking timer rate


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

        # Cell-based explorer state. ~cell_size_m falls back to ~step_size for
        # backward compatibility with launch files that haven't migrated yet.
        self._cell_size_m = float(rospy.get_param('~cell_size_m', self._step_size))
        self._search_radius_cells = int(rospy.get_param(
            '~search_radius_cells', SEARCH_RADIUS_CELLS_DEFAULT))
        self._search_radius_max_cells = int(rospy.get_param(
            '~search_radius_max_cells', SEARCH_RADIUS_MAX_CELLS_DEFAULT))
        self._cost_free_threshold = int(rospy.get_param(
            '~cost_free_threshold', COST_FREE_THRESHOLD_DEFAULT))
        self._costmap_wait_timeout_s = float(rospy.get_param(
            '~costmap_wait_timeout_s', COSTMAP_WAIT_TIMEOUT_S_DEFAULT))
        self._unreachable_cooldown_s = float(rospy.get_param(
            '~unreachable_cooldown_s', UNREACHABLE_COOLDOWN_S_DEFAULT))
        self._pose_tick_hz = float(rospy.get_param(
            '~pose_tick_hz', POSE_TICK_HZ_DEFAULT))

        # Latest costmap snapshot. Updated by _costmap_cb on every publish from
        # /move_base/global_costmap/costmap. None until the first message lands.
        self._costmap: OccupancyGrid = None
        self._costmap_lock = threading.Lock()

        # Visited grid state. Cells the robot center has entered, by integer
        # (i, j) coordinates. _unreachable maps cell -> wall-clock seconds at
        # which the cell becomes eligible again after a move_base failure.
        self._visited: set = set()
        self._unreachable: dict = {}
        self._visited_lock = threading.Lock()

        # Subscribers
        rospy.Subscriber("/puck/registry", PuckRegistry, self._puck_registry_cb, queue_size=1)
        rospy.Subscriber("/aruco/registry", ArucoRegistry, self._aruco_registry_cb, queue_size=1)
        rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid,
                         self._costmap_cb, queue_size=1)

        # cmd_vel publisher (for in-place rotation)
        self._cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        # Navigation client (handles TF + move_base connection)
        self._nav = MoveBaseClient()
        if not self._nav.wait_until_ready():
            rospy.logfatal("move_base not available — random walk cannot start")
            rospy.signal_shutdown("move_base not available")
            return

        # make_plan pre-check
        rospy.wait_for_service('/move_base/make_plan')
        self._make_plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)

        # Service server
        self._service = rospy.Service("random_walk", RandomWalkServerMessage, self.handle_random_walk)
        rospy.loginfo("Random walk service ready.")
        rospy.loginfo(
            "[explorer] config: cell_size_m=%.2f, search_radius_cells=%d (max=%d), "
            "cost_free_threshold=%d, costmap_wait_timeout_s=%.1f, "
            "unreachable_cooldown_s=%.1f, pose_tick_hz=%.1f",
            self._cell_size_m, self._search_radius_cells, self._search_radius_max_cells,
            self._cost_free_threshold, self._costmap_wait_timeout_s,
            self._unreachable_cooldown_s, self._pose_tick_hz,
        )

        # Visited-cell tracker — fires regardless of whether a service call is
        # in flight, so transit cells get marked too.
        self._pose_timer = rospy.Timer(
            rospy.Duration(1.0 / max(self._pose_tick_hz, 0.1)),
            self._pose_tick_cb,
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _puck_registry_cb(self, msg: PuckRegistry) -> None:
        with self._count_lock:
            self._puck_count = len(msg.pucks)

    def _aruco_registry_cb(self, msg: ArucoRegistry) -> None:
        with self._count_lock:
            self._corner_count = len(msg.markers)

    def _costmap_cb(self, msg: OccupancyGrid) -> None:
        with self._costmap_lock:
            first = self._costmap is None
            self._costmap = msg
        if first:
            rospy.loginfo(
                "[explorer] First costmap received: %dx%d cells, resolution=%.3fm, "
                "origin=(%.2f, %.2f)",
                msg.info.width, msg.info.height, msg.info.resolution,
                msg.info.origin.position.x, msg.info.origin.position.y,
            )

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _is_free(self, world_x: float, world_y: float) -> bool:
        """Return True iff (world_x, world_y) lies in the published costmap and
        its cell value is in [0, cost_free_threshold). Unknown (-1) and
        out-of-bounds points return False, so the explorer never tries to drive
        into unmapped or occupied space.
        """
        with self._costmap_lock:
            cm = self._costmap
        if cm is None:
            return False
        res = cm.info.resolution
        ox = cm.info.origin.position.x
        oy = cm.info.origin.position.y
        gx = int((world_x - ox) / res)
        gy = int((world_y - oy) / res)
        if not (0 <= gx < cm.info.width and 0 <= gy < cm.info.height):
            return False
        cost = cm.data[gy * cm.info.width + gx]
        # OccupancyGrid stores -1 for unknown; cost values are int8 in [-1, 100]
        # but the .data list usually surfaces them as Python ints already.
        return 0 <= cost < self._cost_free_threshold

    def _wait_for_costmap(self, timeout_s: float) -> bool:
        """Block until the first costmap message arrives or timeout_s elapses.
        Returns True if a costmap is available, False on timeout/shutdown.
        """
        deadline = rospy.Time.now() + rospy.Duration(timeout_s)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            with self._costmap_lock:
                if self._costmap is not None:
                    return True
            rate.sleep()
        with self._costmap_lock:
            return self._costmap is not None

    def _world_to_cell(self, world_x: float, world_y: float):
        """Map a world-frame point to its integer grid cell. The grid is
        anchored at the world origin and tiles space with squares of side
        cell_size_m. Returns a (i, j) tuple."""
        s = self._cell_size_m
        return (int(math.floor(world_x / s)), int(math.floor(world_y / s)))

    def _cell_to_world_center(self, cell):
        """Inverse of _world_to_cell up to cell granularity. Returns the
        center of cell (i, j) in world coordinates."""
        s = self._cell_size_m
        i, j = cell
        return (i * s + 0.5 * s, j * s + 0.5 * s)

    def _pose_tick_cb(self, _event):
        """Mark the cell the robot is currently in as visited. Runs at
        pose_tick_hz, independent of service activity, so we capture transit
        cells too. Failures are silently ignored — the timer keeps firing."""
        try:
            pose = self._nav.get_robot_pose()
        except Exception:
            return
        if pose is None:
            return
        cell = self._world_to_cell(pose[0], pose[1])
        with self._visited_lock:
            self._visited.add(cell)

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

    def _is_goal_reachable(self, goal_x, goal_y):
        """Pre-check goal reachability via make_plan before rotating toward it."""
        self._nav.cancel()
        rospy.sleep(0.5)
        try:
            pose = self._nav.get_robot_pose()
            if pose is None:
                return True  # can't check, assume reachable

            start = PoseStamped()
            start.header.frame_id = "map"
            start.header.stamp = rospy.Time.now()
            start.pose.position.x = pose[0]
            start.pose.position.y = pose[1]
            start.pose.orientation.w = 1.0

            goal = PoseStamped()
            goal.header.frame_id = "map"
            goal.header.stamp = rospy.Time.now()
            goal.pose.position.x = goal_x
            goal.pose.position.y = goal_y
            goal.pose.orientation.w = 1.0

            req = GetPlanRequest()
            req.start = start
            req.goal = goal
            req.tolerance = 0.25  # matches xy_goal_tolerance

            resp = self._make_plan(req)
            reachable = len(resp.plan.poses) > 0
            if not reachable:
                rospy.logdebug("[walk] make_plan: goal (%.2f, %.2f) unreachable — skipping",
                               goal_x, goal_y)
            return reachable
        except rospy.ServiceException as e:
            rospy.logwarn("[walk] make_plan service error: %s — assuming reachable", e)
            return True  # fail open so walk doesn't stall

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
            # Spread attempts evenly across the circle with a random phase so successive
            # failed steps don't keep retrying the same directions
            sector_size = 2 * math.pi / MAX_ORIENTATION_TRIES
            sector_offset = random.uniform(0, sector_size)
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
                    world_angle = robot_yaw + sector_offset + attempt * sector_size
                goal_x = robot_x + self._step_size * math.cos(world_angle)
                goal_y = robot_y + self._step_size * math.sin(world_angle)

                # Pre-check goal reachability before rotating toward it
                if not self._is_goal_reachable(goal_x, goal_y):
                    continue

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
                # Fallback: try a shorter step (half size) with a fresh set of directions
                short_step = self._step_size * 0.5
                sector_size = 2 * math.pi / MAX_ORIENTATION_TRIES
                sector_offset = random.uniform(0, sector_size)
                for attempt in range(MAX_ORIENTATION_TRIES):
                    if rospy.is_shutdown():
                        break
                    with self._count_lock:
                        if self._puck_count >= num_pucks and self._corner_count >= num_corners:
                            return RandomWalkServerMessageResponse(done=True)
                    world_angle = robot_yaw + sector_offset + attempt * sector_size
                    goal_x = robot_x + short_step * math.cos(world_angle)
                    goal_y = robot_y + short_step * math.sin(world_angle)
                    if not self._is_goal_reachable(goal_x, goal_y):
                        continue
                    if not self._rotate_to_yaw(world_angle):
                        continue
                    pose = self._nav.get_robot_pose()
                    if pose is not None:
                        robot_x, robot_y, _ = pose
                        goal_x = robot_x + short_step * math.cos(world_angle)
                        goal_y = robot_y + short_step * math.sin(world_angle)
                    success = self._nav.go_to(goal_x, goal_y, yaw=world_angle, timeout=STEP_TIMEOUT)
                    if success:
                        direction_found = True
                        self._last_heading = world_angle
                        rospy.sleep(PAUSE_AFTER_STEP)
                        break

            if not direction_found:
                rospy.logwarn("No valid direction found (full + short step) — waiting 1 s.")
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
