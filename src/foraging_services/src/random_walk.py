#!/usr/bin/env python3
"""
ROS service server that performs deterministic, cell-based exploration.
Navigates the robot around the arena until target puck and ArUco corner counts
are met, then returns done=True.

Algorithm:
  1. Overlay a cell_size_m grid on the world frame, anchored at the origin.
  2. Track the cells the robot has entered (visited) via a 5 Hz pose timer.
  3. On each step, pick the closest unvisited cell within Chebyshev radius R
     whose center is FREE in the move_base global costmap and that
     /move_base/make_plan can route to. Send a move_base goal at cell center.
  4. If R is exhausted, expand up to search_radius_max_cells. If still no
     candidate exists, clear _visited and continue (the robot keeps moving
     until thresholds are met or the trial is cancelled externally).
  5. On move_base failure for a candidate, ban that cell for
     unreachable_cooldown_s seconds and try the next candidate.

The service contract (RandomWalkServerMessage) is unchanged: caller specifies
target puck and corner counts; service returns done=True the moment both
thresholds are met.
"""

import rospy
import math
import threading
from foraging_msgs.msg import PuckRegistry, ArucoRegistry
from foraging_msgs.srv import RandomWalkServerMessage, RandomWalkServerMessageResponse
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetPlan, GetPlanRequest
from move_base_client import MoveBaseClient

# ── Parameters ────────────────────────────────────────────────────────────────
STEP_SIZE_DEFAULT     = 0.5   # meters per step (legacy ~step_size; aliased to ~cell_size_m)
STEP_TIMEOUT          = 15    # seconds — timeout for each move_base goal
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

        # ~cell_size_m falls back to ~step_size for backward compatibility
        # with launch files that haven't migrated to the new param name yet.
        legacy_step_size = rospy.get_param('~step_size', STEP_SIZE_DEFAULT)
        self._cell_size_m = float(rospy.get_param('~cell_size_m', legacy_step_size))
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

    def _mark_unreachable(self, cell) -> None:
        """Ban a cell from selection for unreachable_cooldown_s seconds."""
        until = rospy.Time.now() + rospy.Duration(self._unreachable_cooldown_s)
        with self._visited_lock:
            self._unreachable[cell] = until

    def _can_plan_to(self, goal_x: float, goal_y: float) -> bool:
        """Lightweight reachability pre-check via /move_base/make_plan. The
        explorer only calls this between goals, so there's no in-flight goal
        to cancel before asking the planner."""
        try:
            pose = self._nav.get_robot_pose()
        except Exception:
            return True
        if pose is None:
            return True

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
        req.tolerance = 0.25

        try:
            resp = self._make_plan(req)
            return len(resp.plan.poses) > 0
        except rospy.ServiceException as e:
            rospy.logwarn("[explorer] make_plan service error: %s — assuming reachable", e)
            return True

    def _pick_next_cell(self):
        """Choose the next cell to drive to.

        Returns (target_x, target_y, target_yaw, cell) where cell = (i, j),
        or None if no eligible cell is found within search_radius_max_cells.

        Search expands the Chebyshev radius from search_radius_cells up to
        search_radius_max_cells. At each radius we generate all candidate
        cells, filter by visited / unreachable / costmap-free, sort by
        Euclidean distance from the robot (with (j, i) ascending tie-break
        for determinism), and take the first one that move_base can plan to.
        """
        try:
            pose = self._nav.get_robot_pose()
        except Exception:
            return None
        if pose is None:
            return None
        rx, ry, _ = pose
        robot_cell = self._world_to_cell(rx, ry)

        # Snapshot visited / unreachable so we don't hold the lock through
        # candidate scoring.
        now = rospy.Time.now()
        with self._visited_lock:
            visited = set(self._visited)
            # Drop expired bans so the dict doesn't grow without bound.
            self._unreachable = {
                c: t for c, t in self._unreachable.items() if t > now
            }
            banned = set(self._unreachable.keys())

        for radius in range(self._search_radius_cells, self._search_radius_max_cells + 1):
            candidates = []
            for di in range(-radius, radius + 1):
                for dj in range(-radius, radius + 1):
                    if di == 0 and dj == 0:
                        continue
                    cell = (robot_cell[0] + di, robot_cell[1] + dj)
                    if cell in visited or cell in banned:
                        continue
                    cx, cy = self._cell_to_world_center(cell)
                    if not self._is_free(cx, cy):
                        continue
                    dist = math.hypot(cx - rx, cy - ry)
                    candidates.append((dist, cell, cx, cy))

            # Sort by distance, then (j, i) ascending so ties are deterministic.
            candidates.sort(key=lambda t: (t[0], t[1][1], t[1][0]))

            for dist, cell, cx, cy in candidates:
                if not self._can_plan_to(cx, cy):
                    continue
                yaw = math.atan2(cy - ry, cx - rx)
                return (cx, cy, yaw, cell)

        return None

    # ── Service handler ───────────────────────────────────────────────────────

    def handle_random_walk(self, req: RandomWalkServerMessage) -> RandomWalkServerMessageResponse:
        num_pucks = req.num_pucks_to_find
        num_corners = req.num_corners_to_find
        rospy.loginfo(
            "[explorer] requested: target pucks=%d, target corners=%d",
            num_pucks, num_corners,
        )

        # 1. Already at threshold?
        with self._count_lock:
            puck_count = self._puck_count
            corner_count = self._corner_count
        if puck_count >= num_pucks and corner_count >= num_corners:
            rospy.loginfo("[explorer] thresholds already met — done")
            return RandomWalkServerMessageResponse(done=True)

        # 2. Wait for the costmap. Without it, _is_free returns False for
        #    every cell and we'd loop forever rejecting candidates.
        if not self._wait_for_costmap(self._costmap_wait_timeout_s):
            rospy.logwarn(
                "[explorer] no costmap after %.1fs — aborting (done=False)",
                self._costmap_wait_timeout_s,
            )
            return RandomWalkServerMessageResponse(done=False)

        step = 0
        while not rospy.is_shutdown():
            # Check thresholds.
            with self._count_lock:
                puck_count = self._puck_count
                corner_count = self._corner_count
            if puck_count >= num_pucks and corner_count >= num_corners:
                rospy.loginfo(
                    "[explorer] thresholds met — pucks=%d/%d, corners=%d/%d. Done.",
                    puck_count, num_pucks, corner_count, num_corners,
                )
                return RandomWalkServerMessageResponse(done=True)

            step += 1
            if step % 5 == 0:
                with self._visited_lock:
                    n_visited = len(self._visited)
                    n_banned = len(self._unreachable)
                rospy.loginfo(
                    "[explorer] step %d — pucks=%d/%d, corners=%d/%d, visited=%d, banned=%d",
                    step, puck_count, num_pucks, corner_count, num_corners,
                    n_visited, n_banned,
                )

            result = self._pick_next_cell()
            if result is None:
                # All reachable cells exhausted within R_max. Per design, reset
                # _visited and keep going; the explorer never returns done=False
                # while the trial is alive — the orchestrator's wait timeout is
                # the trial-level safety net.
                with self._visited_lock:
                    n_visited = len(self._visited)
                    n_banned = len(self._unreachable)
                    self._visited.clear()
                    self._unreachable.clear()
                rospy.loginfo(
                    "[explorer] coverage exhausted (visited=%d, banned=%d) — "
                    "clearing visited set and continuing",
                    n_visited, n_banned,
                )
                rospy.sleep(1.0)
                continue

            target_x, target_y, target_yaw, cell = result
            rospy.loginfo(
                "[explorer] step %d: target cell %s at (%.2f, %.2f), yaw=%.2f rad",
                step, cell, target_x, target_y, target_yaw,
            )
            success = self._nav.go_to(target_x, target_y, yaw=target_yaw, timeout=STEP_TIMEOUT)
            if success:
                rospy.loginfo(
                    "[explorer] reached cell %s — pausing %.1fs for perception",
                    cell, PAUSE_AFTER_STEP,
                )
                rospy.sleep(PAUSE_AFTER_STEP)
            else:
                rospy.logwarn(
                    "[explorer] move_base failed for cell %s — banning for %.0fs",
                    cell, self._unreachable_cooldown_s,
                )
                self._mark_unreachable(cell)

        # rospy shutdown before thresholds met.
        return RandomWalkServerMessageResponse(done=False)


def main():
    rospy.init_node("random_walk_server")
    rospy.loginfo("Random walk server node started.")
    RandomWalkServer()
    rospy.spin()


if __name__ == "__main__":
    main()
