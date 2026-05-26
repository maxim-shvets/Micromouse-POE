"""Pure-pursuit path-tracking controller.

Planner-driven race controller that turns the flood-fill cell plan into a
continuous waypoint path, then tracks it with pure pursuit.  Recovery and
perf/rate-limit hooks are delegated to the existing ReactiveController so
the safety behaviour stays consistent with the legacy path.

CircuitPython-portable: no typing, no dataclasses, no f-strings.
"""

import math

from algorithm import ReactiveController, _safe_forward_speed
from interfaces import WheelSpeeds


N = 0
E = 1
S = 2
W = 3

_DC = (0, 1, 0, -1)
_DR = (1, 0, -1, 0)
_DIR_X = (0.0, 1.0, 0.0, -1.0)
_DIR_Y = (1.0, 0.0, -1.0, 0.0)


class PathController(object):
    """Path-generating pure-pursuit controller.

    Public surface intentionally mirrors ReactiveController where tester,
    telemetry, and stress harnesses expect it.
    """

    _S_TRACKING = 0
    _S_RECOVERING = 1
    # Compatibility alias for stress.py's legacy idle-window check.
    _S_REACT = 0

    STATE_NAMES = {0: "TRACKING", 1: "RECOVERING"}

    def __init__(self, tunables, planner, pose_provider,
                 estimator=None, observation_pose_provider=None):
        self.t = tunables
        self.planner = planner
        self.pose_provider = pose_provider
        self.observation_pose_provider = observation_pose_provider or pose_provider
        self.estimator = estimator

        # Wrapped legacy layer.  planner=None makes it the pure reactive
        # controller; we force it into REVERSE/PIVOT when path tracking
        # declares a recovery event.
        self._legacy = ReactiveController(
            tunables, planner=None, pose_provider=pose_provider,
            estimator=None, observation_pose_provider=observation_pose_provider)

        self._state = self._S_TRACKING
        self.recovery_count = 0
        self._last_imu_reading = None
        self._sim_t = 0.0
        self._planner_t = 0.0

        self._path = None
        self._closest_idx = 0
        self._full_closest_search = True
        self._force_repath = True
        self._last_path_dirty = True
        self._last_cmd_l = 0.0
        self._last_cmd_r = 0.0

    @property
    def state(self):
        return self._state

    @property
    def stuck_t(self):
        if self._state == self._S_RECOVERING:
            return self._legacy.stuck_t
        return 0.0

    @property
    def imu_reading(self):
        return self._last_imu_reading

    @property
    def perf_last_overrun(self):
        return self._legacy.perf_last_overrun

    def step(self, reading, encoders, dt, imu_reading=None):
        self._last_imu_reading = imu_reading
        if self.estimator is not None:
            try:
                self.estimator.update(encoders[0], encoders[1], dt,
                                      imu_reading=imu_reading, reading=reading)
            except TypeError:
                self.estimator.update(encoders[0], encoders[1], dt,
                                      imu_reading=imu_reading)

        self._sim_t += dt

        if self.planner is None or self.pose_provider is None:
            return self._legacy.step(reading, encoders, dt,
                                     imu_reading=imu_reading)

        if self._state == self._S_RECOVERING:
            cmd = self._legacy.step(reading, encoders, dt,
                                    imu_reading=imu_reading)
            if self._legacy.state == self._legacy._S_REACT:
                self._state = self._S_TRACKING
                self._force_repath = True
                self._full_closest_search = True
            return cmd

        x, y, theta = self.pose_provider()
        self._observe_planner(reading, x, y, theta, dt)

        if not self._ensure_path(x, y, theta):
            return WheelSpeeds(0.0, 0.0)

        path = self._path
        if path is None or len(path) == 0:
            return WheelSpeeds(0.0, 0.0)

        idx, closest_d = self._closest_path_index(x, y)
        offpath = self.t.path_offpath_recover_m
        if offpath > 0.0 and closest_d > offpath:
            self._force_repath = True
            if not self._ensure_path(x, y, theta):
                return WheelSpeeds(0.0, 0.0)
            path = self._path
            idx, closest_d = self._closest_path_index(x, y)

        gx, gy = path[-1]
        end_d = _dist_xy(x, y, gx, gy)
        if end_d < 0.5 * self.planner.cell_size_m:
            return WheelSpeeds(0.0, 0.0)

        v_cur = self._current_forward_speed()
        lookahead = self._lookahead_distance_from_speed(v_cur)
        lp = self._lookahead_point(idx, lookahead)
        dx = lp[0] - x
        dy = lp[1] - y
        c = math.cos(theta)
        s = math.sin(theta)
        local_x = dx * c + dy * s
        local_y = -dx * s + dy * c

        # If the selected point has slipped behind the robot, bias toward
        # the next available point instead of commanding a tight reversal.
        if local_x < 0.0 and idx + 1 < len(path):
            lp = path[idx + 1]
            dx = lp[0] - x
            dy = lp[1] - y
            local_x = dx * c + dy * s
            local_y = -dx * s + dy * c

        # When starting from rest with a path that leaves sideways (common
        # at the start cell when the forward wall is known/observed), do a
        # pure heading-acquisition pivot.  Pure pursuit with a positive
        # minimum forward speed would otherwise scrape the front wall before
        # the robot has turned onto the path.
        lateral = abs(local_y)
        if (lateral > self.t.path_waypoint_spacing_m
                and local_x <= self.t.path_waypoint_spacing_m
                and v_cur < 2.0 * self.t.min_speed_mps):
            return self._pivot_toward(local_y)

        front_guard = self._front_guard_m()
        if reading.front < front_guard:
            if local_x <= 2.0 * front_guard:
                return self._pivot_toward(local_y)
            if reading.front < self.t.front_stop_m:
                self.recovery_count += 1
                return self._legacy.step(reading, encoders, dt,
                                         imu_reading=imu_reading)

        denom = local_x * local_x + local_y * local_y
        if denom <= 1e-9:
            kappa = 0.0
        else:
            kappa = 2.0 * local_y / denom

        v_cmd = _safe_forward_speed(reading.front, self.t)
        usable_guard = reading.front - front_guard
        if usable_guard <= 0.0:
            v_cmd = 0.0
        elif self.t.max_decel_mps2 > 0.0:
            v_guard = math.sqrt(2.0 * self.t.max_decel_mps2 * usable_guard)
            if v_cmd > v_guard:
                v_cmd = v_guard
        if abs(kappa) > 1e-6 and self.t.max_decel_mps2 > 0.0:
            v_curve = math.sqrt(0.20 * self.t.max_decel_mps2 / abs(kappa))
            if v_cmd > v_curve:
                v_cmd = v_curve
        vmax = self.t.path_track_v_max_mps
        if vmax > 0.0 and v_cmd > vmax:
            v_cmd = vmax
        omega = kappa * v_cmd
        half = 0.5 * self.t.wheel_base_m
        return WheelSpeeds(v_cmd - omega * half,
                           v_cmd + omega * half)

    def _rate_limit(self, cmd, dt):
        limited = self._legacy._rate_limit(cmd, dt)
        self._last_cmd_l = limited.left
        self._last_cmd_r = limited.right
        return limited

    def _perf_tick(self, measured_us):
        self._legacy._perf_tick(measured_us)

    def perf_summary(self):
        return self._legacy.perf_summary()

    # ---- planner observation / invalidation -------------------------------

    def _observe_planner(self, reading, x, y, theta, dt):
        from planner import heading_from_theta, theta_from_heading, wrap_pi
        heading = heading_from_theta(theta)
        cardinal_theta = theta_from_heading(heading)
        align_err = abs(wrap_pi(cardinal_theta - theta))
        cell = self.planner.pose_to_cell(x, y)
        s = self.planner.cell_size_m
        c_idx, r_idx = cell
        if heading == N:
            forward_in_cell = y - r_idx * s
        elif heading == E:
            forward_in_cell = x - c_idx * s
        elif heading == S:
            forward_in_cell = (r_idx + 1) * s - y
        else:
            forward_in_cell = (c_idx + 1) * s - x

        if align_err < self.t.planner_observe_tol_rad:
            observe_sides = forward_in_cell >= 0.5 * s
            ox, oy, otheta = self.observation_pose_provider()
            obs_cell = self.planner.pose_to_cell(ox, oy)
            self.planner.observe((ox, oy, otheta), obs_cell, heading,
                                 reading, observe_sides=observe_sides)
        elif align_err < 0.65:
            # Path tracking can be 20-35 degrees off-cardinal while still
            # moving down a cardinal corridor.  Front-wall classification
            # remains useful in that band; side rays do not.
            ox, oy, otheta = self.observation_pose_provider()
            obs_cell = self.planner.pose_to_cell(ox, oy)
            self.planner.observe((ox, oy, otheta), obs_cell, heading,
                                 reading, observe_sides=False)

        self._planner_t += dt
        if self._planner_t >= self.t.planner_replan_period_s:
            self._planner_t = 0.0
            if self.planner._dirty or self.planner._dist is None:
                self._force_repath = True

        if self.planner._dirty != self._last_path_dirty:
            if self.planner._dirty:
                self._force_repath = True
            self._last_path_dirty = self.planner._dirty

    def _ensure_path(self, x, y, theta):
        if (not self._force_repath and self._path is not None
                and not self.planner._dirty):
            return True
        from planner import heading_from_theta
        cell = self.planner.pose_to_cell(x, y)
        heading = heading_from_theta(theta)
        self._path = self._generate_path(cell, heading)
        if self._path is not None and len(self._path) > 0:
            px, py = self._path[0]
            if _dist_xy(x, y, px, py) > self.t.path_waypoint_spacing_m:
                self._path.insert(0, (x, y))
        self._closest_idx = 0
        self._full_closest_search = True
        self._force_repath = False
        self._last_path_dirty = self.planner._dirty
        return self._path is not None and len(self._path) > 0

    # ---- path generation ---------------------------------------------------

    def _generate_path(self, start_cell, start_heading):
        if self.planner._dirty or self.planner._dist is None:
            self.planner.replan()
        cells, dirs = self._extract_cardinal_plan(start_cell, start_heading)
        if len(cells) == 0:
            return []

        spacing = self.t.path_waypoint_spacing_m
        if spacing <= 0.0:
            spacing = 0.02

        points = []
        x0, y0 = self.planner.cell_center_xy(cells[0][0], cells[0][1])
        _append_point(points, x0, y0)

        radius = self._arc_radius()
        i = 0
        while i < len(dirs):
            if i + 1 < len(dirs) and self._is_l_turn(dirs[i], dirs[i + 1]):
                d1 = dirs[i]
                d2 = dirs[i + 1]
                if self._diagonal_passable(cells[i], d1, d2):
                    tx, ty = self.planner.cell_center_xy(
                        cells[i + 2][0], cells[i + 2][1])
                    _append_line_from_last(points, tx, ty, spacing)
                    i += 2
                else:
                    if radius > 0.0:
                        self._append_corner_arc(points, cells[i + 1],
                                                d1, d2, radius, spacing)
                    else:
                        bx, by = self.planner.cell_center_xy(
                            cells[i + 1][0], cells[i + 1][1])
                        _append_line_from_last(points, bx, by, spacing)
                    i += 1
            else:
                tx, ty = self.planner.cell_center_xy(
                    cells[i + 1][0], cells[i + 1][1])
                _append_line_from_last(points, tx, ty, spacing)
                i += 1
        return points

    def _extract_cardinal_plan(self, start_cell, start_heading):
        cells = [start_cell]
        dirs = []
        cell = start_cell
        heading = start_heading
        cap = self.planner.cols * self.planner.rows
        if cap < 1:
            cap = 1
        for _i in range(cap):
            if cell == self.planner.goal_cell:
                break
            d = self.planner.desired_heading(cell, heading)
            if self.planner.map.is_blocked(cell[0], cell[1], d):
                break
            nc = cell[0] + _DC[d]
            nr = cell[1] + _DR[d]
            if not (0 <= nc < self.planner.cols and 0 <= nr < self.planner.rows):
                break
            dirs.append(d)
            cell = (nc, nr)
            cells.append(cell)
            heading = d
        return cells, dirs

    def _is_l_turn(self, d1, d2):
        if d1 == d2:
            return False
        if (d1 + 2) % 4 == d2:
            return False
        return True

    def _diagonal_passable(self, cell, d1, d2):
        if not getattr(self.planner, "use_diagonals", False):
            return False
        try:
            from planner import _corner_passable
            return _corner_passable(
                self.planner.map.walls, self.planner.cols, self.planner.rows,
                cell[0], cell[1], d1, d2,
                strict=getattr(self.planner, "diagonal_strict", True))
        except (ImportError, AttributeError):
            return False

    def _arc_radius(self):
        v_arc = self.t.arc_turn_v_mps
        if v_arc <= 0.0:
            return 0.25 * self.planner.cell_size_m
        if self.t.arc_turn_omega_rps > 0.0:
            r = v_arc / self.t.arc_turn_omega_rps
        elif self.t.turn_speed_mps > 0.0:
            r = v_arc * self.t.wheel_base_m / (2.0 * self.t.turn_speed_mps)
        else:
            r = 0.0
        max_r = 0.45 * self.planner.cell_size_m
        if r > max_r:
            r = max_r
        return r

    def _append_corner_arc(self, points, corner_cell, d1, d2, radius, spacing):
        bx, by = self.planner.cell_center_xy(corner_cell[0], corner_cell[1])
        ux1 = _DIR_X[d1]
        uy1 = _DIR_Y[d1]
        ux2 = _DIR_X[d2]
        uy2 = _DIR_Y[d2]

        sx = bx - radius * ux1
        sy = by - radius * uy1
        ex = bx + radius * ux2
        ey = by + radius * uy2
        cx = sx + radius * ux2
        cy = sy + radius * uy2

        _append_line_from_last(points, sx, sy, spacing)

        a0 = math.atan2(sy - cy, sx - cx)
        a1 = math.atan2(ey - cy, ex - cx)
        cross = ux1 * uy2 - uy1 * ux2
        if cross > 0.0:
            while a1 < a0:
                a1 += 2.0 * math.pi
        else:
            while a1 > a0:
                a1 -= 2.0 * math.pi
        _append_arc(points, cx, cy, radius, a0, a1, spacing)

    # ---- pure pursuit helpers ---------------------------------------------

    def _closest_path_index(self, x, y):
        path = self._path
        n = len(path)
        if self._full_closest_search or self._closest_idx >= n:
            start = 0
            end = n
            self._full_closest_search = False
        else:
            start = self._closest_idx
            end = self._closest_idx + 24
            if end > n:
                end = n
        best_i = start
        best_d2 = None
        for i in range(start, end):
            px, py = path[i]
            dx = px - x
            dy = py - y
            d2 = dx * dx + dy * dy
            if best_d2 is None or d2 < best_d2:
                best_d2 = d2
                best_i = i
        self._closest_idx = best_i
        if best_d2 is None:
            return 0, 0.0
        return best_i, math.sqrt(best_d2)

    def _current_forward_speed(self):
        v_cur = 0.5 * (self._last_cmd_l + self._last_cmd_r)
        if v_cur < 0.0:
            v_cur = 0.0
        return v_cur

    def _lookahead_distance(self):
        return self._lookahead_distance_from_speed(self._current_forward_speed())

    def _lookahead_distance_from_speed(self, v_cur):
        L = self.t.path_lookahead_min_m + self.t.path_lookahead_gain * v_cur
        if L < self.t.path_lookahead_min_m:
            L = self.t.path_lookahead_min_m
        return L

    def _lookahead_point(self, idx, lookahead):
        path = self._path
        if idx >= len(path) - 1:
            return path[-1]
        remain = lookahead
        prev = path[idx]
        i = idx + 1
        while i < len(path):
            cur = path[i]
            seg = _dist_xy(prev[0], prev[1], cur[0], cur[1])
            if seg >= remain and seg > 1e-9:
                f = remain / seg
                return (prev[0] + (cur[0] - prev[0]) * f,
                        prev[1] + (cur[1] - prev[1]) * f)
            remain -= seg
            prev = cur
            i += 1
        return path[-1]

    # ---- recovery ----------------------------------------------------------

    def _enter_recovery(self, reading):
        if self._state == self._S_RECOVERING:
            return
        self._state = self._S_RECOVERING
        self.recovery_count += 1
        self._force_repath = True
        self._full_closest_search = True
        self._legacy._last_reading = reading
        self._legacy._sim_t = self._sim_t
        self._legacy._stuck_t = 0.0
        self._legacy._enter(self._legacy._S_REVERSE)

    def _pivot_toward(self, local_y):
        s = self.t.turn_speed_mps
        if local_y >= 0.0:
            return WheelSpeeds(-s, +s)
        return WheelSpeeds(+s, -s)

    def _front_guard_m(self):
        guard = self.t.chassis_radius_m + self.t.sensor_forward_offset_m + 0.03
        if guard < self.t.front_stop_m:
            guard = self.t.front_stop_m
        return guard


def _dist_xy(x0, y0, x1, y1):
    dx = x1 - x0
    dy = y1 - y0
    return math.sqrt(dx * dx + dy * dy)


def _append_point(points, x, y):
    if points:
        px, py = points[-1]
        if (x - px) * (x - px) + (y - py) * (y - py) < 1e-12:
            return
    points.append((x, y))


def _append_line_from_last(points, x, y, spacing):
    if not points:
        _append_point(points, x, y)
        return
    x0, y0 = points[-1]
    _append_line(points, x0, y0, x, y, spacing)


def _append_line(points, x0, y0, x1, y1, spacing):
    dist = _dist_xy(x0, y0, x1, y1)
    if dist <= 1e-9:
        _append_point(points, x1, y1)
        return
    steps = int(dist / spacing)
    if steps < 1:
        steps = 1
    for i in range(1, steps + 1):
        f = (spacing * i) / dist
        if f > 1.0:
            f = 1.0
        _append_point(points, x0 + (x1 - x0) * f,
                      y0 + (y1 - y0) * f)
    _append_point(points, x1, y1)


def _append_arc(points, cx, cy, radius, a0, a1, spacing):
    da = a1 - a0
    length = abs(da) * radius
    if length <= 1e-9:
        _append_point(points, cx + radius * math.cos(a1),
                      cy + radius * math.sin(a1))
        return
    steps = int(length / spacing)
    if steps < 1:
        steps = 1
    for i in range(1, steps + 1):
        f = (spacing * i) / length
        if f > 1.0:
            f = 1.0
        a = a0 + da * f
        _append_point(points, cx + radius * math.cos(a),
                      cy + radius * math.sin(a))
    _append_point(points, cx + radius * math.cos(a1),
                  cy + radius * math.sin(a1))
