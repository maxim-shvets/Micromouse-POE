"""Flood-fill maze planner that sits above the reactive controller.

Architecture (matches the README design):

  - `KnownMap` holds the walls grid.  Walls start as unknown (None) and are
    filled in incrementally as the three ToF sensors detect a wall in the
    current cell.  Border walls are pre-set to True.
  - `flood_fill` runs BFS from the goal cell back through the open-or-
    unknown graph -- "optimistic" planning: unknowns count as open.  This
    matches the classic Tellaroli / Harrison flood-fill micromouse trick:
    you don't need a fully-mapped maze before driving toward the goal.
  - `FloodFillPlanner` ties them together.  Each tick the controller calls
    `observe()` (record walls) and `desired_heading()` (read off the
    cardinal step toward the lowest-cost open neighbour).

The reactive controller in `algorithm.py` stays as a safety / recovery
layer: if a wall surprises us mid-cell, the existing REVERSE+PIVOT machine
fires.  When the planner is wired in, it picks the desired cardinal; the
controller turns to it and drives forward.

Wall thresholding is geometry-derived: with the side sensors at 45 deg
forward-diagonal (`side_sensor_angle_rad = pi/4`), a side wall in the
current cell returns at ~`cell_size/sqrt(2)` and a forward wall at
~`cell_size/2 - sensor_offset`.  A single threshold a touch above
`cell_size/sqrt(2)` catches all "wall present" cases reliably.  When a
forward wall *is* present the diagonal rays are dominated by it, so we
only trust side observations when the front is clear -- this is the
trade-off baked into `observe()`.

CircuitPython-portable: no `@dataclass`, no `typing`, no f-strings.
"""

import heapq
import math


# Cardinal directions -- index convention matches sim/maze.py.
N = 0
E = 1
S = 2
W = 3

_DC = (0, 1, 0, -1)   # delta col per direction
_DR = (1, 0, -1, 0)   # delta row per direction
_OPPOSITE = (S, W, N, E)

_INF_COST = 1 << 30


def _turn_class(d_from, d_to):
    """0 = straight, 1 = 90-deg turn, 2 = 180-deg about-face."""
    if d_from == d_to:
        return 0
    if (d_from + 2) % 4 == d_to:
        return 2
    return 1


def _turn_penalty(d_from, d_to, turn_cost, reverse_cost):
    cls = _turn_class(d_from, d_to)
    if cls == 0:
        return 0.0
    if cls == 2:
        return reverse_cost
    return turn_cost


# -----------------------------------------------------------------------------
# Heading <-> theta helpers
# -----------------------------------------------------------------------------

def theta_from_heading(d):
    """Continuous theta (rad) for cardinal direction d.

    Matches the sim's world frame: theta=0 -> +x (E), theta=pi/2 -> +y (N).
    """
    if d == N:
        return 0.5 * math.pi
    if d == E:
        return 0.0
    if d == S:
        return -0.5 * math.pi
    return math.pi  # W


def heading_from_theta(theta):
    """Snap continuous theta to the nearest cardinal direction (N/E/S/W)."""
    a = theta % (2.0 * math.pi)
    # Quadrants in (E, N, W, S) order, spaced at pi/2.
    idx = int(round(a / (math.pi / 2.0))) % 4
    return (E, N, W, S)[idx]


def wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


# -----------------------------------------------------------------------------
# Known walls grid
# -----------------------------------------------------------------------------

class KnownMap(object):
    """Grid of incrementally-observed walls.

    walls[c][r] is a 4-list [N, E, S, W] where each entry is:
      None  -- never observed; treated as open for optimistic planning
      True  -- known to be a wall
      False -- known to be open

    Border walls are pre-set to True.  Every `set_wall` call mirrors the
    update to the neighbour's opposite side so the map stays consistent.
    """

    def __init__(self, cols, rows):
        self.cols = cols
        self.rows = rows
        self.walls = [[[None, None, None, None] for _ in range(rows)]
                      for _ in range(cols)]
        for c in range(cols):
            self.walls[c][0][S] = True
            self.walls[c][rows - 1][N] = True
        for r in range(rows):
            self.walls[0][r][W] = True
            self.walls[cols - 1][r][E] = True

    def set_wall(self, c, r, d, present):
        """Record an observation.  Mirrored to the neighbour's opposite side."""
        if not (0 <= c < self.cols and 0 <= r < self.rows):
            return
        v = bool(present)
        self.walls[c][r][d] = v
        nc = c + _DC[d]
        nr = r + _DR[d]
        if 0 <= nc < self.cols and 0 <= nr < self.rows:
            self.walls[nc][nr][_OPPOSITE[d]] = v

    def is_blocked(self, c, r, d):
        """True iff a wall is known to be present.  Unknown counts as open."""
        return self.walls[c][r][d] is True


# -----------------------------------------------------------------------------
# Flood-fill
# -----------------------------------------------------------------------------

def flood_fill(known_map, goal_cell):
    """Plain BFS step-distance from `goal_cell` to every reachable cell.

    Unknown walls are treated as open.  Returns a 2D list `g` where
    `g[c][r]` is the step count from (c, r) to the goal, or `_INF_COST`
    if disconnected.  Kept as a reference implementation and for any
    caller that wants the un-weighted heuristic.
    """
    cols = known_map.cols
    rows = known_map.rows
    grid = [[_INF_COST] * rows for _ in range(cols)]
    gc, gr = goal_cell
    grid[gc][gr] = 0
    queue = [(gc, gr)]
    head = 0
    while head < len(queue):
        c, r = queue[head]
        head += 1
        d_cur = grid[c][r]
        for direction in (N, E, S, W):
            if known_map.is_blocked(c, r, direction):
                continue
            nc = c + _DC[direction]
            nr = r + _DR[direction]
            if not (0 <= nc < cols and 0 <= nr < rows):
                continue
            if grid[nc][nr] > d_cur + 1:
                grid[nc][nr] = d_cur + 1
                queue.append((nc, nr))
    return grid


def flood_fill_weighted(known_map, goal_cell,
                        turn_cost=1.0, reverse_cost=4.0, unknown_cost=0.5):
    """Risk-weighted Dijkstra over (cell, facing_direction) states.

    For each cell + facing direction, computes the minimum forward cost to
    reach `goal_cell`.  Turning into a different direction costs
    `turn_cost` (90 deg) or `reverse_cost` (180 deg) on top of the unit
    move cost.  Crossing an unknown wall costs an extra `unknown_cost`
    (unknowns are still treated as traversable -- the standard optimistic
    flood-fill trick -- but with a risk premium so the planner prefers
    known-open paths).

    Args:
        known_map:    `KnownMap`.
        goal_cell:    (col, row).
        turn_cost:    extra cost per 90-deg direction change.
        reverse_cost: extra cost per 180-deg direction change.
        unknown_cost: extra cost per crossing of an unknown wall.

    Returns:
        g[c][r][d] : optimal forward cost-to-goal for a robot at (c, r)
                    facing direction `d`, `_INF_COST` if unreachable.

    Notes:
      - The expanded state space is 4 * cols * rows -- 1600 states at
        20x20.  Dijkstra is comfortably under 1 ms for that on CPython.
      - On CircuitPython, `heapq` is available in the standard build.
    """
    cols = known_map.cols
    rows = known_map.rows
    INF = _INF_COST
    g = [[[INF, INF, INF, INF] for _ in range(rows)] for _ in range(cols)]

    pq = []
    gc, gr = goal_cell
    # At the goal, any facing direction is fine -- cost 0.
    for d in range(4):
        g[gc][gr][d] = 0.0
        heapq.heappush(pq, (0.0, gc, gr, d))

    while pq:
        cost, c, r, d = heapq.heappop(pq)
        if cost > g[c][r][d]:
            continue  # stale entry from a later better push

        # Predecessor of (c, r, d) is a state (pc, pr, pd) from which the
        # robot turned to face `d` and moved one cell to land at (c, r, d).
        # So pc = c - DC[d], pr = r - DR[d], and pd is any of the four
        # directions (the previous facing before the optional turn).
        pc = c - _DC[d]
        pr = r - _DR[d]
        if not (0 <= pc < cols and 0 <= pr < rows):
            continue
        wall_state = known_map.walls[pc][pr][d]
        if wall_state is True:
            continue  # known wall blocks the move
        unk = unknown_cost if wall_state is None else 0.0
        for pd in range(4):
            turn = _turn_penalty(pd, d, turn_cost, reverse_cost)
            new_cost = cost + 1.0 + turn + unk
            if new_cost < g[pc][pr][pd]:
                g[pc][pr][pd] = new_cost
                heapq.heappush(pq, (new_cost, pc, pr, pd))

    return g


# -----------------------------------------------------------------------------
# Planner
# -----------------------------------------------------------------------------

class FloodFillPlanner(object):
    """Flood-fill driver above the reactive layer.

    Lifecycle each tick (driven by the controller):
        1. observe(pose, cell, heading, reading)  -- record walls
        2. desired_heading(cell, heading)         -- next cardinal to drive

    The planner re-floods lazily, only when an observation actually changed
    the map.  Flooding 8x8 = 64 cells is cheap, but the lazy gate keeps the
    cost trivially constant once the maze is mapped.

    Wall-detection thresholds:
      - Forward sensor uses a *pose-aware* threshold: we compute the
        distance from the sensor to the current cell's forward edge given
        the robot's pose, then call the wall present iff the reading is
        within `wall_tolerance_m` of that expected distance.  A static
        threshold can't disambiguate "wall right at the cell's forward
        edge" from "wall one cell further away with the robot still near
        this cell's entry" -- both give similar absolute distances.
      - Side sensors (45 deg forward-diagonal) hit the side wall of the
        *next* cell at a position-independent distance of cell_size /
        sqrt(2) when the open forward boundary is crossed by the ray.
        We compare against that, with the same tolerance.
    """

    def __init__(self, cols, rows, goal_cell, cell_size_m,
                 sensor_forward_offset_m=0.03, wall_tolerance_m=0.05,
                 turn_cost=1.0, reverse_cost=4.0, unknown_cost=0.5):
        self.cols = cols
        self.rows = rows
        self.goal_cell = (int(goal_cell[0]), int(goal_cell[1]))
        self.cell_size_m = float(cell_size_m)
        self.sensor_forward_offset_m = float(sensor_forward_offset_m)
        self.wall_tolerance_m = float(wall_tolerance_m)
        # Risk-weighted-path cost factors.  See `flood_fill_weighted`.
        self.turn_cost = float(turn_cost)
        self.reverse_cost = float(reverse_cost)
        self.unknown_cost = float(unknown_cost)
        self.map = KnownMap(cols, rows)
        # Expected side-ray distance when a side wall is present (and no
        # forward wall blocks the ray).  Position-independent because the
        # ray hits a wall line whose perpendicular offset is s/2.
        self._side_expected_m = self.cell_size_m / math.sqrt(2.0)
        # `_dist[c][r][d]` is the optimal cost to goal from facing d at (c, r).
        self._dist = None
        self._dirty = True
        # Bookkeeping for telemetry / debugging.
        self.replan_count = 0
        self.observe_count = 0

    # ---- pose -> cell -----------------------------------------------------

    def pose_to_cell(self, x, y):
        """Snap world-frame (x, y) to a (col, row).  Clamps to the grid."""
        s = self.cell_size_m
        c = int(x / s)
        r = int(y / s)
        if c < 0:
            c = 0
        elif c >= self.cols:
            c = self.cols - 1
        if r < 0:
            r = 0
        elif r >= self.rows:
            r = self.rows - 1
        return (c, r)

    # ---- observation ------------------------------------------------------

    def observe(self, pose, cell, heading, reading, observe_sides=True):
        """Update the known map from one (pose, cell, heading, reading) tuple.

        Pose-aware front-wall test:
            expected = perpendicular distance from sensor origin to the
                       current cell's forward edge
            wall iff |reading.front - expected| <= wall_tolerance_m
            (treat readings much larger than expected as "no wall in this
             cell"; the ray will have continued into the next cell.)

        Side-ray test: when there's no forward wall, the diagonal rays
        hit a side-wall line that is perpendicular-offset s/2 from the
        sensor, so the geometric distance is s/sqrt(2) regardless of
        the robot's longitudinal position.  Same tolerance.  Sides are
        attributed to the cell one step ahead in heading direction --
        that's where the rays actually intersect when the robot has
        crossed the current cell's midpoint (caller checks).

        Caller invariants:
          - `pose = (x, y, theta)` is the robot's current world-frame pose.
          - `cell` is `pose_to_cell(x, y)`.
          - `heading` is the cardinal direction the robot is facing
             (caller has already confirmed alignment via `theta`).
          - `observe_sides=True` only when the robot is past the cell
             midpoint in the heading direction.
        """
        x, y, _theta = pose
        c, r = cell
        s = self.cell_size_m
        off = self.sensor_forward_offset_m
        tol = self.wall_tolerance_m
        self.observe_count += 1

        # Forward edge of current cell along heading.
        if heading == N:
            fwd_expected = (r + 1) * s - y - off
        elif heading == E:
            fwd_expected = (c + 1) * s - x - off
        elif heading == S:
            fwd_expected = y - r * s - off
        else:  # W
            fwd_expected = x - c * s - off
        # If the geometry says the wall would be behind the sensor (the
        # robot is essentially on top of the forward edge), don't try to
        # classify -- the next tick at a safer position will do it.
        if fwd_expected < 0.0:
            fwd_known = False
            fwd_wall = False
        else:
            fwd_wall = reading.front <= fwd_expected + tol
            fwd_known = True

        if fwd_known:
            self._record(c, r, heading, fwd_wall)
        if fwd_wall or not observe_sides:
            return
        # Sides: attribute to the next cell.  Expected distance s/sqrt(2).
        next_c = c + _DC[heading]
        next_r = r + _DR[heading]
        if not (0 <= next_c < self.cols and 0 <= next_r < self.rows):
            return
        expected = self._side_expected_m
        left_wall = reading.left <= expected + tol
        right_wall = reading.right <= expected + tol
        left_dir = (heading + 3) % 4   # CCW of heading
        right_dir = (heading + 1) % 4  # CW of heading
        self._record(next_c, next_r, left_dir, left_wall)
        self._record(next_c, next_r, right_dir, right_wall)

    def _record(self, c, r, d, present):
        before = self.map.walls[c][r][d]
        self.map.set_wall(c, r, d, present)
        if before != self.map.walls[c][r][d]:
            self._dirty = True

    # ---- planning ---------------------------------------------------------

    def replan(self):
        self._dist = flood_fill_weighted(
            self.map, self.goal_cell,
            turn_cost=self.turn_cost,
            reverse_cost=self.reverse_cost,
            unknown_cost=self.unknown_cost,
        )
        self._dirty = False
        self.replan_count += 1

    def cost(self, c, r, facing=None):
        """Cost-to-goal from (c, r).  If `facing` is given, returns the
        cost for that arrival direction; else returns the best across all
        four facings.  `_INF_COST` if unreachable (unknowns counted as open)."""
        if self._dirty or self._dist is None:
            self.replan()
        if facing is not None:
            return self._dist[c][r][facing]
        cell = self._dist[c][r]
        return min(cell)

    def desired_heading(self, cell, current_heading):
        """Pick the cardinal that minimises (turn + move + cost-to-goal).

        The Dijkstra cost map already accounts for turns *downstream* from
        the next cell.  Here we add the turn cost from the current heading
        to the candidate direction so the planner doesn't pick an
        equal-cost path that requires turning right now over one that
        keeps the robot going straight.

        Ties favour `current_heading` (stability while mid-turn).  If
        every neighbour is walled, returns `current_heading` so the
        controller can let recovery sort it out.
        """
        if self._dirty or self._dist is None:
            self.replan()
        c, r = cell
        if cell == self.goal_cell:
            return current_heading
        best_d = None
        best_cost = _INF_COST
        # Iterate with current_heading first so ties resolve toward straight.
        order = [current_heading]
        for d in (N, E, S, W):
            if d != current_heading:
                order.append(d)
        for d in order:
            if self.map.is_blocked(c, r, d):
                continue
            nc = c + _DC[d]
            nr = r + _DR[d]
            if not (0 <= nc < self.cols and 0 <= nr < self.rows):
                continue
            wall_state = self.map.walls[c][r][d]
            unk = self.unknown_cost if wall_state is None else 0.0
            turn = _turn_penalty(current_heading, d,
                                 self.turn_cost, self.reverse_cost)
            cost = 1.0 + turn + unk + self._dist[nc][nr][d]
            if cost < best_cost:
                best_cost = cost
                best_d = d
        if best_d is None:
            return current_heading
        return best_d


# -----------------------------------------------------------------------------
# Pose helpers
# -----------------------------------------------------------------------------

class EncoderOdometry(object):
    """Wheel-encoder integrator for real-hardware pose.

    Sim doesn't need this -- the SimWorld carries ground-truth pose.  On
    the RP2040, instantiate this with the start cell's center + initial
    heading, then call `update()` each control tick with the measured
    wheel speeds.  Tunables exposes `wheel_base_m`; the wheel diameter is
    folded into the measured m/s by `DriverN20` upstream.
    """

    def __init__(self, x0, y0, theta0, wheel_base_m):
        self.x = float(x0)
        self.y = float(y0)
        self.theta = float(theta0)
        self.wheel_base_m = float(wheel_base_m)

    def update(self, left_mps, right_mps, dt):
        v = 0.5 * (left_mps + right_mps)
        omega = (right_mps - left_mps) / self.wheel_base_m
        # Midpoint integration: rotate first half, translate, rotate second
        # half.  For the small dt we run at this is overkill, but it avoids
        # bias when the robot is simultaneously turning + driving.
        self.theta += 0.5 * omega * dt
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += 0.5 * omega * dt

    def pose(self):
        return (self.x, self.y, self.theta)
