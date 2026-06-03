"""Reactive obstacle-avoidance algorithm.

Matches the project spec:

    LOOP:
      1. Read front, left, right distances.
      2. If front distance < threshold:
            stop, turn toward more open side
      3. Else if left too close:
            steer right slightly
      4. Else if right too close:
            steer left slightly
      5. Else:
            drive forward
      6. Use encoder feedback to keep both wheels at similar speed.

Three layers, separated by rate and concern:

  - `step(reading, encoders, t)` (pure function)
      Outer reactive decision @ loop_hz.  Returns commanded WheelSpeeds in m/s.
      Wall-centering bias is folded in when both side rays return in-range
      hits -- keeps the robot off corners that pure threshold logic blunders
      into.  Trivial to unit-test.

  - `ReactiveController`
      Wraps `step` with stuck-detection / recovery state machine driven by
      encoder feedback.  Without this, three-ray reactive nav wedges at
      diagonal corners where all rays see past but the chassis doesn't fit.

  - `WheelController`
      Inner per-wheel PI loop, cmd_mps -> PWM duty.  Used by the hardware
      adapter; the sim ignores it (commanded speed feeds the sim wheel
      model directly).

All three accept a single `Tunables` object -- nothing else here knows
about magic numbers.

CircuitPython-portable: no `typing`, no `@dataclass`, no f-strings.
"""

import time

from interfaces import WheelSpeeds


# -----------------------------------------------------------------------------
# Portable microsecond timer.  Three runtimes to support:
#   CPython 3.7+         -- time.monotonic_ns (ns precision)
#   CircuitPython 4.0+   -- time.monotonic    (float seconds)
#   MicroPython          -- time.ticks_us + time.ticks_diff (wrap-aware)
# Used by `run()` to measure per-tick controller work for the Path-1
# tick-budget instrumentation.
# -----------------------------------------------------------------------------

if hasattr(time, "monotonic_ns"):
    def _perf_now_us():
        return time.monotonic_ns() / 1000.0

    def _perf_diff_us(end, start):
        return end - start
elif hasattr(time, "monotonic"):
    def _perf_now_us():
        return time.monotonic() * 1e6

    def _perf_diff_us(end, start):
        return end - start
elif hasattr(time, "ticks_us"):
    _perf_now_us = time.ticks_us  # type: ignore[attr-defined]
    _ticks_diff = time.ticks_diff  # type: ignore[attr-defined]

    def _perf_diff_us(end, start):
        return _ticks_diff(end, start)
else:
    # No timer available -- degrade gracefully (no instrumentation).
    def _perf_now_us():
        return 0.0

    def _perf_diff_us(end, start):
        return 0.0


# -----------------------------------------------------------------------------
# Optional flood-fill planner integration.  Imported lazily inside the
# controller so that environments without `planner.py` (or hardware that
# doesn't need it) still load `algorithm` cleanly.
# -----------------------------------------------------------------------------


# -----------------------------------------------------------------------------
# Outer reactive loop (pure function)
# -----------------------------------------------------------------------------

def step(reading, encoders, t):
    """Compute commanded wheel speeds for one control tick.

    Args:
        reading:  `Reading` -- front/left/right distances in meters.
        encoders: (left_mps, right_mps) measured wheel speeds.  Unused at
                  this layer (the inner controller closes that loop); kept
                  in the signature for telemetry / future wall-following.
        t:        `Tunables`.

    Returns:
        `WheelSpeeds` in m/s.
    """
    front = reading.front
    left = reading.left
    right = reading.right

    # 1. Front blocked -> stop forward motion, pivot toward more open side.
    if front < t.front_stop_m:
        if left >= right:
            return WheelSpeeds(-t.turn_speed_mps, +t.turn_speed_mps)
        return WheelSpeeds(+t.turn_speed_mps, -t.turn_speed_mps)

    # Risk-aware forward speed: cap so braking distance fits inside front
    # clearance.  v_max = sqrt(2 * a_max * usable).
    base = _safe_forward_speed(front, t)

    # 2/3. Hard steer-away when a side gets too close.  Threshold logic
    # from the spec; the wall-centering term below smooths the more
    # common in-corridor case.
    if left < t.side_min_m:
        return WheelSpeeds(base, base * (1.0 - t.steer_gain))
    if right < t.side_min_m:
        return WheelSpeeds(base * (1.0 - t.steer_gain), base)

    # 4. Both sides have headroom -> drive forward with a centering bias.
    # When both sides return finite (< sensor_max_range), the robot is in
    # something corridor-shaped -- bias toward the geometric center.
    bias = _wall_center_bias(left, right, t)
    if bias != 0.0:
        # bias > 0 means "veer right"; bias < 0 means "veer left".
        return WheelSpeeds(base * (1.0 + bias), base * (1.0 - bias))
    return WheelSpeeds(base, base)


def _safe_forward_speed(front_clearance, t):
    """Cap commanded speed by available braking distance."""
    import math
    usable = front_clearance - t.front_stop_m
    if usable <= 0.0:
        return t.min_speed_mps
    v_brake = math.sqrt(2.0 * t.max_decel_mps2 * usable)
    v = v_brake
    if v > t.max_speed_mps:
        v = t.max_speed_mps
    if v > t.cruise_speed_mps:
        v = t.cruise_speed_mps
    if v < t.min_speed_mps:
        v = t.min_speed_mps
    return v


def _wall_center_bias(left, right, t):
    """Steering bias in [-max_bias, +max_bias].  Positive = veer right.

    Only fires when both side rays see something within sensor range -- if
    one ray maxed out, the robot is at a corridor mouth or in an open area
    and centering doesn't apply.
    """
    if t.wall_center_gain <= 0.0:
        return 0.0
    if left >= t.sensor_max_range_m or right >= t.sensor_max_range_m:
        return 0.0
    total = left + right
    if total <= 1e-6:
        return 0.0
    # Closer to the left wall -> left < right -> err < 0 -> veer right (+).
    err = (right - left) / total
    bias = t.wall_center_gain * err
    if bias > t.wall_center_max_bias:
        bias = t.wall_center_max_bias
    elif bias < -t.wall_center_max_bias:
        bias = -t.wall_center_max_bias
    return bias


# -----------------------------------------------------------------------------
# Inner loop: per-wheel speed -> PWM duty (encoder feedback)
# -----------------------------------------------------------------------------

class WheelController(object):
    """PI controller on a single wheel.  cmd_mps in, PWM duty in [-1, 1].

    Anti-windup via conditional integration; integrator reset on commanded
    direction reversal.
    """

    __slots__ = ("kp", "ki", "_integral", "_last_cmd_sign", "duty_min", "duty_max")

    def __init__(self, kp, ki, duty_min=-1.0, duty_max=1.0, duty_deadband=0.0):
        self.kp = kp
        self.ki = ki
        self._integral = 0.0
        self._last_cmd_sign = 0
        self.duty_min = duty_min
        self.duty_max = duty_max
        self.duty_deadband = duty_deadband  # minimum duty when motor is commanded

    def update(self, cmd_mps, measured_mps, dt):
        sign = 0 if cmd_mps == 0.0 else (1 if cmd_mps > 0.0 else -1)
        if sign != self._last_cmd_sign:
            self._integral = 0.0
            self._last_cmd_sign = sign

        if sign == 0:
            return 0.0

        err = cmd_mps - measured_mps
        unclamped = self.kp * err + self.ki * self._integral
        if unclamped > self.duty_max:
            duty = self.duty_max
        elif unclamped < self.duty_min:
            duty = self.duty_min
        else:
            duty = unclamped
            self._integral += err * dt

        # Kickstart only: apply minimum duty when the wheel is near-stationary
        # to overcome stiction.  Once moving, let the PID run freely so slow
        # turn commands don't overshoot.
        if abs(measured_mps) < 0.02:
            if duty > 0.0 and duty < self.duty_deadband:
                duty = self.duty_deadband
            elif duty < 0.0 and duty > -self.duty_deadband:
                duty = -self.duty_deadband
        return duty


# -----------------------------------------------------------------------------
# Stateful controller: stuck detection + recovery
# -----------------------------------------------------------------------------

class ReactiveController(object):
    """Wraps `step` with encoder-driven stuck recovery.

    States:
      REACT    -- normal reactive control.
      REVERSE  -- briefly back up (open-loop, turn_speed_mps for reverse_time_s).
      PIVOT    -- briefly pivot toward the more open side (pivot_time_s).

    A "stuck" event is: commanded |speed| > stuck_speed_threshold_mps while
    measured |speed| stays below the same threshold for stuck_time_s.  The
    spec hints at this with "use encoder feedback"; this is the outer-loop
    use of that signal.
    """

    _S_REACT = 0
    _S_REVERSE = 1
    _S_PIVOT = 2

    STATE_NAMES = {0: "REACT", 1: "REVERSE", 2: "PIVOT"}

    def __init__(self, tunables, planner=None, pose_provider=None,
                 estimator=None, observation_pose_provider=None):
        """Args:
            tunables:       `Tunables`.
            planner:        Optional flood-fill planner (see `planner.py`).
                            When provided, REACT runs as plan-then-execute
                            (turn to the planner's cardinal, then drive
                            forward).  When None, behaves exactly as the
                            original reactive controller.
            pose_provider:  Required when `planner` is set.  Callable
                            returning the current world-frame pose
                            (x, y, theta).  Sim ground-truth, or the
                            `.pose` of a fused/SLAM estimator.
            estimator:      Optional pose estimator (FusedOdometry or
                            ScanMatchSlam).  When set, its `update()` is
                            called at the top of each control tick so
                            `pose_provider()` reads fresh state.  Leave
                            None when using ground-truth pose.
            observation_pose_provider:
                            Optional separate pose-provider for
                            planner.observe() calls.  When wiring SLAM,
                            pass `estimator.dead_reckoning_pose` here so
                            wall observations are attributed to cells
                            using a smooth (un-corrected) pose; small
                            SLAM corrections at cell boundaries would
                            otherwise flip the cell index and poison
                            the known map (Bug #29).  Defaults to
                            `pose_provider`.
        """
        self.t = tunables
        self.planner = planner
        self.pose_provider = pose_provider
        self.observation_pose_provider = observation_pose_provider or pose_provider
        self.estimator = estimator
        self._state = self._S_REACT
        self._state_t = 0.0       # time spent in current state
        self._stuck_t = 0.0       # accumulated stuck time in REACT
        self._last_reading = None # cached on entry to recovery
        self.recovery_count = 0
        # Pivot hysteresis: when step() picks an in-place pivot direction,
        # latch it for at least pivot_hysteresis_s.  Prevents chatter from
        # near-equal L/R readings.  +1 = pivot CCW (left forward), -1 = CW.
        self._pivot_dir = 0
        self._pivot_t = 0.0       # time accumulated in current pivot direction
        self._pivot_total_t = 0.0 # total continuous time pivoting in REACT
        # Planner replan throttle and bookkeeping.
        self._planner_t = 0.0
        self._desired_heading = None
        # When the planner offers a diagonal cut, the controller latches
        # the target until the robot arrives at the cell center.  Tuple:
        # (world_x, world_y, target_theta, target_cell).  None = cardinal
        # mode (default).  See `_planner_diagonal_step`.
        self._diagonal_target = None
        # Last per-tick IMU reading (set by step() when one is passed in).
        # Exposed for telemetry / fusion / SLAM consumers.
        self._last_imu_reading = None
        # Last commanded wheel speeds (for inter-tick rate limiting).
        # Caps |cmd[t] - cmd[t-1]| to max_wheel_accel_mps2 * dt so the
        # algorithm can't slam from cruise to pivot in one tick (which
        # would heat the DRV8833 on real hardware; sim hides the issue).
        self._last_cmd_l = 0.0
        self._last_cmd_r = 0.0
        # Adaptive-recovery state: a recovery that fires soon after the
        # previous one (within `recovery_cluster_s`) is treated as part
        # of the same "incident".  Each repeat tries a different escape
        # strategy (flip pivot side, extend pivot duration).
        self._consec_recoveries = 0
        self._time_at_last_recovery = -1e9
        # The latched recovery-pivot direction.  Set on REVERSE entry,
        # flipped on consecutive recoveries.  +1 = CCW (right wheel
        # forward), -1 = CW.
        self._recovery_pivot_dir = +1
        # Sim/wall-clock time accumulator the controller maintains
        # privately (mirrors the planner's _planner_t).
        self._sim_t = 0.0
        # Path-1 perf instrumentation.  algorithm.run() times the
        # controller.step()+_rate_limit() span and feeds each result
        # back through `_perf_tick()`.  The aggregates project to the
        # XIAO via cpu_slowdown_factor and compare against perf_budget_us
        # (which defaults to 1e6/loop_hz).  No effect on physics; purely
        # diagnostic.
        self._perf_tick_count = 0
        self._perf_overrun_count = 0
        self._perf_total_us = 0.0
        self._perf_max_us = 0.0
        self._perf_total_projected_us = 0.0
        self._perf_max_projected_us = 0.0
        self._perf_last_us = 0.0
        self._perf_last_projected_us = 0.0
        self._perf_last_overrun = False

    @property
    def state(self):
        return self._state

    @property
    def stuck_t(self):
        return self._stuck_t

    @property
    def imu_reading(self):
        """The most recent IMUReading the controller saw (None if no IMU)."""
        return self._last_imu_reading

    def step(self, reading, encoders, dt, imu_reading=None):
        # Stash IMU first so any sub-stage (planner, future fusion/SLAM)
        # and the telemetry recorder can read it via `controller.imu_reading`.
        self._last_imu_reading = imu_reading
        # If an estimator is wired, advance it before any pose_provider()
        # call so the planner sees this tick's fresh state.  ScanMatchSlam
        # accepts an extra `reading` kwarg; FusedOdometry ignores it.
        if self.estimator is not None:
            try:
                self.estimator.update(encoders[0], encoders[1], dt,
                                      imu_reading=imu_reading, reading=reading)
            except TypeError:
                self.estimator.update(encoders[0], encoders[1], dt,
                                      imu_reading=imu_reading)
        self._state_t += dt
        self._sim_t += dt
        T = self.t

        if self._state == self._S_REACT:
            if self.planner is not None:
                cmd = self._planner_step(reading, dt)
            else:
                cmd = step(reading, encoders, T)
                cmd = self._apply_pivot_hysteresis(cmd, reading, dt)

            cmd_mag = abs(cmd.left) if abs(cmd.left) > abs(cmd.right) else abs(cmd.right)
            meas_mag = abs(encoders[0]) if abs(encoders[0]) > abs(encoders[1]) else abs(encoders[1])
            if cmd_mag > T.stuck_speed_threshold_mps and meas_mag < T.stuck_speed_threshold_mps:
                self._stuck_t += dt
                if self._stuck_t >= T.stuck_time_s:
                    self._enter(self._S_REVERSE)
                    self._last_reading = reading
            else:
                self._stuck_t = 0.0
            # Pivot-stall escalation only applies when the reactive layer is
            # doing the pivoting -- planner-driven turns are intentional and
            # should not be treated as a stall.
            if self.planner is None and self._pivot_total_t >= T.pivot_stall_s:
                self._enter(self._S_REVERSE)
                self._last_reading = reading
                self._pivot_total_t = 0.0
            return cmd

        if self._state == self._S_REVERSE:
            # Early exit if reverse is wedged: we're commanding backward
            # motion but encoders confirm no motion happened.  Spending
            # the full reverse_time_s pushing into a wall behind us is
            # wasted budget; skip to PIVOT and try another angle.
            if (self._state_t >= 0.5 * T.reverse_time_s
                    and abs(encoders[0]) < T.stuck_speed_threshold_mps
                    and abs(encoders[1]) < T.stuck_speed_threshold_mps):
                self._enter(self._S_PIVOT)
            elif self._state_t >= T.reverse_time_s:
                self._enter(self._S_PIVOT)
            else:
                s = T.turn_speed_mps
                return WheelSpeeds(-s, -s)

        if self._state == self._S_PIVOT:
            # Third+ consecutive recovery in this incident -> stretch the
            # pivot so we explore a bigger arc.  Bounded at 4x to keep
            # the controller responsive.
            stretch = min(self._consec_recoveries, 4)
            pivot_window = T.pivot_time_s * stretch
            if self._state_t >= pivot_window:
                self._enter(self._S_REACT)
                self._stuck_t = 0.0
                return step(reading, encoders, T)
            s = T.turn_speed_mps
            d = self._recovery_pivot_dir
            return WheelSpeeds(-d * s, +d * s)

        return WheelSpeeds(0.0, 0.0)

    def _enter(self, new_state):
        self._state = new_state
        self._state_t = 0.0
        if new_state == self._S_REVERSE:
            self.recovery_count += 1
            # Clear in-REACT pivot state when escaping to recovery.
            self._pivot_dir = 0
            self._pivot_t = 0.0
            self._pivot_total_t = 0.0
            # Adaptive recovery: if this recovery fires soon after the
            # previous one, treat it as a continuation of the same
            # incident and try a different escape strategy.
            recovery_cluster_s = max(2.0, 2.0 * self.t.reverse_time_s
                                          + 2.0 * self.t.pivot_time_s)
            since = self._sim_t - self._time_at_last_recovery
            if since < recovery_cluster_s:
                self._consec_recoveries += 1
                # On every odd consecutive recovery, flip the pivot side.
                if self._consec_recoveries % 2 == 1:
                    self._recovery_pivot_dir = -self._recovery_pivot_dir
            else:
                # New incident -- pick direction based on the more-open
                # side of the cached reading.
                self._consec_recoveries = 1
                if self._last_reading is not None:
                    self._recovery_pivot_dir = (
                        +1 if self._last_reading.left >= self._last_reading.right
                        else -1)
                else:
                    self._recovery_pivot_dir = +1
            self._time_at_last_recovery = self._sim_t

    def _rate_limit(self, cmd, dt):
        """Cap |cmd - last_cmd| per wheel to max_wheel_accel_mps2 * dt.

        Prevents algorithm-level command discontinuities (e.g. from a
        turn_speed pivot to a cruise_speed forward in a single tick) that
        the DRV8833 inner-PI would translate into a thermal spike.  In
        sim this matches what SimWorld already does to the actual wheel
        speed; the difference is that the algorithm now SEES the limit.
        """
        a_max = self.t.max_wheel_accel_mps2
        if a_max <= 0.0:
            return cmd
        step = a_max * dt
        def clip(new, old):
            d = new - old
            if d > step:
                return old + step
            if d < -step:
                return old - step
            return new
        nl = clip(cmd.left, self._last_cmd_l)
        nr = clip(cmd.right, self._last_cmd_r)
        self._last_cmd_l = nl
        self._last_cmd_r = nr
        return WheelSpeeds(nl, nr)

    # -- perf instrumentation (Path 1) ---------------------------------------

    def _perf_tick(self, measured_us):
        """Record one tick's measured CPU time and project to target MCU.

        Called from algorithm.run() with the wall-clock microseconds taken
        by controller.step()+_rate_limit() on the host (Mac CPython).  The
        projection multiplies by `cpu_slowdown_factor` so we can ask:
        "would this tick fit in budget on the XIAO?"  Each call updates
        the rolling aggregates; `perf_summary()` exposes them.

        No-op semantics if cpu_slowdown_factor == 1.0: measured == projected
        and overrun is judged against the same budget.
        """
        T = self.t
        factor = T.cpu_slowdown_factor
        if factor < 1.0:
            factor = 1.0
        projected_us = measured_us * factor
        budget_us = T.perf_budget_us
        if budget_us <= 0.0:
            budget_us = 1e6 / T.loop_hz
        over = projected_us > budget_us

        self._perf_tick_count += 1
        self._perf_total_us += measured_us
        self._perf_total_projected_us += projected_us
        if measured_us > self._perf_max_us:
            self._perf_max_us = measured_us
        if projected_us > self._perf_max_projected_us:
            self._perf_max_projected_us = projected_us
        if over:
            self._perf_overrun_count += 1
        self._perf_last_us = measured_us
        self._perf_last_projected_us = projected_us
        self._perf_last_overrun = over

    def perf_summary(self):
        """Return a dict of aggregated perf stats, or {} if no ticks ran.

        Keys:
            ticks            -- total ticks measured
            budget_us        -- per-tick budget (from perf_budget_us or
                                derived from loop_hz)
            slowdown         -- cpu_slowdown_factor used
            avg_us / max_us  -- measured host CPU time per tick
            avg_proj_us / max_proj_us -- projected MCU time per tick
            overrun_count    -- ticks where projected > budget
            overrun_pct      -- overrun_count / ticks * 100
        """
        if self._perf_tick_count == 0:
            return {}
        T = self.t
        budget_us = T.perf_budget_us
        if budget_us <= 0.0:
            budget_us = 1e6 / T.loop_hz
        n = self._perf_tick_count
        return {
            "ticks": n,
            "budget_us": budget_us,
            "slowdown": T.cpu_slowdown_factor,
            "avg_us": self._perf_total_us / n,
            "max_us": self._perf_max_us,
            "avg_proj_us": self._perf_total_projected_us / n,
            "max_proj_us": self._perf_max_projected_us,
            "overrun_count": self._perf_overrun_count,
            "overrun_pct": 100.0 * self._perf_overrun_count / n,
        }

    @property
    def perf_last_overrun(self):
        return self._perf_last_overrun

    def _apply_pivot_hysteresis(self, cmd, reading, dt):
        """If step() picked an in-place pivot, latch the direction.

        Without this, when |left - right| is small and noisy, step() flips
        pivot direction every tick and the robot dithers in place.
        """
        T = self.t
        is_pivot = (cmd.left * cmd.right < 0
                    and abs(cmd.left) > T.min_speed_mps)
        if not is_pivot:
            self._pivot_dir = 0
            self._pivot_t = 0.0
            self._pivot_total_t = 0.0
            return cmd

        self._pivot_total_t += dt
        if self._pivot_dir == 0:
            # First tick of a new pivot -- adopt step's choice.
            self._pivot_dir = +1 if cmd.right > cmd.left else -1
            self._pivot_t = 0.0
        else:
            self._pivot_t += dt
            if self._pivot_t >= T.pivot_hysteresis_s:
                # Window expired -- allow step's fresh pick to take over.
                fresh = +1 if cmd.right > cmd.left else -1
                if fresh != self._pivot_dir:
                    self._pivot_dir = fresh
                    self._pivot_t = 0.0
        s = T.turn_speed_mps
        return WheelSpeeds(-self._pivot_dir * s, +self._pivot_dir * s)

    # -- planner-driven REACT ------------------------------------------------

    def _planner_step(self, reading, dt):
        """Plan-then-execute step.

        Each tick:
          1. Pull pose from `pose_provider` and resolve current cell + heading.
          2. If a diagonal cut is in progress, drive toward its target cell
             and return; diagonals are not interrupted mid-flight.
          3. If aligned with a cardinal, let the planner record walls from
             the current ToF reading.
          4. If at-or-past the cell centre along the current heading, ask
             the planner for the next motion (cardinal OR diagonal).  Before
             centre, commit to the current heading -- turning mid-cell would
             leave the robot off-axis in the perpendicular dimension and
             clip the next cell's walls.
          5. Issue motion -- in-place pivot when mis-aligned, else forward.

        The reactive layer's stuck detection + REVERSE/PIVOT recovery still
        wraps this (see `step()`), so a surprise wall mid-cell still gets
        the safety bounce.
        """
        from planner import (theta_from_heading, heading_from_theta, wrap_pi)
        T = self.t
        x, y, theta = self.pose_provider()

        # Diagonal mode takes precedence: once committed, drive to the
        # target cell center without observing or re-planning.  This is a
        # race optimisation -- the maze should be well-mapped before
        # diagonals fire, and a 45-deg ray-trace doesn't map cleanly onto
        # cell walls so observation would poison the map.
        if self._diagonal_target is not None:
            return self._planner_diagonal_step(reading, x, y, theta, T)

        cell = self.planner.pose_to_cell(x, y)
        heading = heading_from_theta(theta)
        cardinal_theta = theta_from_heading(heading)
        align_err = abs(wrap_pi(cardinal_theta - theta))

        s = self.planner.cell_size_m
        c_idx, r_idx = cell
        # Forward-in-cell projection (distance from cell rear edge along
        # heading).  Used both for "are sides observable from here?" and
        # "are we at-or-past cell centre for a turn decision?".
        if heading == 0:      # N
            forward_in_cell = y - r_idx * s
        elif heading == 1:    # E
            forward_in_cell = x - c_idx * s
        elif heading == 2:    # S
            forward_in_cell = (r_idx + 1) * s - y
        else:                 # W
            forward_in_cell = (c_idx + 1) * s - x
        progress_past_centre = forward_in_cell - 0.5 * s

        # Only update the map when reasonably well aligned with a cardinal.
        # Use the OBSERVATION pose (smooth dead-reckoning when SLAM is wired)
        # for cell-attribution.  See Bug #29 -- mm-scale SLAM corrections
        # at cell boundaries flip the cell index and poison the map.
        if align_err < T.planner_observe_tol_rad:
            observe_sides = forward_in_cell >= 0.5 * s
            ox, oy, otheta = self.observation_pose_provider()
            obs_cell = self.planner.pose_to_cell(ox, oy)
            self.planner.observe((ox, oy, otheta), obs_cell, heading, reading,
                                 observe_sides=observe_sides)

        # Replan heartbeat (lazy replan inside the planner covers the rest).
        self._planner_t += dt
        if self._planner_t >= T.planner_replan_period_s:
            self._planner_t = 0.0
            if self.planner._dirty or self.planner._dist is None:
                self.planner.replan()

        # Tangent-arc trigger point.  For pure pivot (arc_turn_v_mps=0)
        # this equals cell center -- legacy "decide at cell center" path.
        # For arc turning, the arc starts r metres BEFORE cell center so
        # the arc's midpoint lands on the cell center diagonal -- robot
        # enters at (cell_x, cell_y - r) heading N, exits at (cell_x + r,
        # cell_y) heading E.  Both endpoints sit on the perpendicular
        # cell-center axes, so the next straight segment is automatically
        # axis-aligned and no lateral drift accumulates across turns.
        arc_r = self._arc_radius(T)
        arc_start_fwd = 0.5 * s - arc_r
        progress_past_arc_start = forward_in_cell - arc_start_fwd

        # Pick a heading.  We commit to a cardinal turn at arc-start (which
        # is earlier than cell centre when arc_turn_v_mps>0).  Diagonals
        # still wait for cell centre -- they're a per-cell commit to a
        # full 1.4-cell cut and benefit from the robot being centered.
        if progress_past_arc_start >= 0.0:
            motion = self.planner.desired_motion(cell, heading)
            if motion[0] == 'diagonal':
                # Diagonals are unchanged: wait until cell centre.
                if progress_past_centre >= 0.0:
                    _, target_theta_diag, target_cell = motion
                    wx, wy = self.planner.cell_center_xy(*target_cell)
                    self._diagonal_target = (wx, wy, target_theta_diag, target_cell)
                    self.planner.diagonals_taken += 1
                    self._desired_heading = None
                    return self._planner_diagonal_step(reading, x, y, theta, T)
                desired = heading
            else:
                desired = motion[1]
        else:
            desired = heading
        self._desired_heading = desired
        target_theta = theta_from_heading(desired)
        err = wrap_pi(target_theta - theta)

        if abs(err) > T.planner_turn_threshold_rad:
            # Turn toward target heading.  Pure pivot when
            # arc_turn_v_mps=0, otherwise a forward-arc that shaves the
            # stop-and-restart penalty off every planner turn.
            return self._planner_turn_command(err, reading, T)

        # Aligned -- drive forward, risk-aware.  A small heading-error
        # P-correction bleeds residual `err` off without flipping back to
        # full pivot (err is bounded here by planner_turn_threshold_rad).
        base = _safe_forward_speed(reading.front, T)
        # Pre-turn deceleration: cap cruise so the rate limiter can bring
        # us to the arc/pivot speed before the next turn point.  Without
        # this, race-mode cruise overshoots cell boundaries by 1+ cells
        # because the wheels can't decelerate fast enough.  See
        # `cells_to_next_turn` -- O(few) per tick, negligible cost.
        base = self._brake_for_next_turn(base, cell, desired,
                                         forward_in_cell, T)
        # Heading-error P-correction (positive bias = need CCW).
        hdg_bias = T.steer_gain * err
        # Wall-centering pull: only when arc turning is enabled.  Arc
        # turns can leave the robot slightly off-axis; wall-centering
        # pulls it back between turns.  In legacy stop-and-pivot mode
        # the robot is always cell-axis-aligned right after the pivot,
        # so wall-centering only adds noise (it conflicts with the
        # heading-correction in mid-cell observations).
        if T.arc_turn_v_mps > 0.0:
            wall_bias = _wall_center_bias(reading.left, reading.right, T)
            bias = hdg_bias - wall_bias
        else:
            bias = hdg_bias
        max_bias = 0.5
        if bias > max_bias:
            bias = max_bias
        elif bias < -max_bias:
            bias = -max_bias
        return WheelSpeeds(base * (1.0 - bias), base * (1.0 + bias))

    def _brake_for_next_turn(self, v_cruise, cell, heading, forward_in_cell, T):
        """Cap forward speed so we can decelerate to arc/pivot by the turn.

        Physics: starting at v0 and decelerating at a, we reach v_target
        after covering (v0^2 - v_target^2) / (2a) metres.  Solving for
        the maximum v0 that still fits in distance `d`:

            v_max(d) = sqrt(v_target^2 + 2 * a * d)

        Target point: the arc-start, which is `arc_r` metres before the
        apex cell's centre when v_arc > 0 (tangent-arc geometry), or the
        apex cell's centre for pure pivot.  Distance to it:

            d = (n + 0.5) * cell_size - forward_in_cell - arc_r

        where `n` is `cells_to_next_turn` (lookahead, planner-driven).
        """
        import math
        n = self.planner.cells_to_next_turn(cell, heading)
        v_target = T.arc_turn_v_mps if T.arc_turn_v_mps > 0.0 else 0.0
        arc_r = self._arc_radius(T)
        d = (n + 0.5) * T.planner_cell_size_m - forward_in_cell - arc_r
        if d <= 0.0:
            d = 0.0
        v_max = math.sqrt(v_target * v_target + 2.0 * T.max_decel_mps2 * d)
        if v_max < v_cruise:
            return v_max
        return v_cruise

    # -- planner-driven REACT, turn execution helper --------------------------

    def _arc_radius(self, T):
        """Geometric radius of the planner-arc, in metres.  0 if no arc.

        Uses `arc_turn_omega_rps` when set (decoupled mode), otherwise
        falls back to omega = 2 * turn_speed_mps / wheel_base_m (legacy
        same-rate-as-pivot).
        """
        v_arc = T.arc_turn_v_mps
        if v_arc <= 0.0:
            return 0.0
        if T.arc_turn_omega_rps > 0.0:
            return v_arc / T.arc_turn_omega_rps
        return v_arc * T.wheel_base_m / (2.0 * T.turn_speed_mps)

    def _planner_turn_command(self, err, reading, T):
        """Build a wheel command that turns toward `err` heading offset.

        Returns either a pure pivot (legacy behaviour, used when
        arc_turn_v_mps is 0 or |err| is too large for safe arc) or an
        arc: forward at `arc_turn_v_mps` while rotating at omega.

        Omega selection:
          arc_turn_omega_rps > 0 -> use it directly (decoupled from
                                    legacy turn_speed_mps).
          otherwise              -> omega = 2 * turn_speed_mps /
                                    wheel_base_m  (legacy: same rate as
                                    in-place pivot).

        Forward velocity is clamped by `_safe_forward_speed` so an arc
        into a wall degrades to a pivot before contact.
        """
        v_arc = T.arc_turn_v_mps
        if v_arc <= 0.0 or abs(err) > T.arc_turn_max_err_rad:
            # Pure pivot: legacy turn_speed_mps.
            s_turn = T.turn_speed_mps
            if err > 0.0:
                return WheelSpeeds(-s_turn, +s_turn)  # CCW
            return WheelSpeeds(+s_turn, -s_turn)      # CW
        # Cap by forward clearance so we don't arc into a wall.
        v_safe = _safe_forward_speed(reading.front, T)
        if v_arc > v_safe:
            v_arc = v_safe
        # Choose omega: decoupled if arc_turn_omega_rps > 0, else legacy.
        if T.arc_turn_omega_rps > 0.0:
            omega = T.arc_turn_omega_rps
        else:
            omega = 2.0 * T.turn_speed_mps / T.wheel_base_m
        s_diff = omega * T.wheel_base_m / 2.0   # half-difference per wheel
        if err > 0.0:
            return WheelSpeeds(v_arc - s_diff, v_arc + s_diff)  # CCW
        return WheelSpeeds(v_arc + s_diff, v_arc - s_diff)      # CW

    # -- planner-driven REACT, diagonal branch --------------------------------

    def _planner_diagonal_step(self, reading, x, y, theta, T):
        """Drive toward a latched diagonal target.

        Same shape as the cardinal drive branch but with the target
        theta coming from `_diagonal_target` (45-deg off cardinal)
        instead of a heading enum, and arrival measured as Euclidean
        distance to the destination cell's center.

        When the robot is within `planner_diagonal_arrive_frac * cell_size`
        of the target, clear the latch.  The next tick re-enters
        `_planner_step` proper and gets a fresh cardinal-or-diagonal
        decision from the planner.  We return WheelSpeeds(0, 0) on that
        arrival tick so the rate limiter has one cycle to brake before
        the next motion -- the freeze is one dt (5 ms at race mode), so
        it's not visible in trajectory.
        """
        from planner import wrap_pi
        wx, wy, target_theta, _target_cell = self._diagonal_target
        dx = wx - x
        dy = wy - y
        d_sq = dx * dx + dy * dy
        arrive_r = T.planner_cell_size_m * T.planner_diagonal_arrive_frac
        if d_sq < arrive_r * arrive_r:
            self._diagonal_target = None
            return WheelSpeeds(0.0, 0.0)
        err = wrap_pi(target_theta - theta)
        if abs(err) > T.planner_turn_threshold_rad:
            return self._planner_turn_command(err, reading, T)
        base = _safe_forward_speed(reading.front, T)
        bias = T.steer_gain * err
        max_bias = 0.5
        if bias > max_bias:
            bias = max_bias
        elif bias < -max_bias:
            bias = -max_bias
        return WheelSpeeds(base * (1.0 - bias), base * (1.0 + bias))


# -----------------------------------------------------------------------------
# Top-level runner
# -----------------------------------------------------------------------------

def run(sensors, drive, clock, tunables, imu=None, max_steps=None,
        on_step=None, controller=None):
    """Spin the control loop.

    Args:
        sensors:    `RangeSensors` impl.
        drive:      `Drive` impl.
        clock:      `Clock` impl.
        tunables:   `Tunables`.
        imu:        Optional `IMU` impl.  Polled once per tick and stashed
                    on the controller; downstream consumers (telemetry,
                    fusion, SLAM) read via `controller.imu_reading`.
        max_steps:  Stop after this many ticks (None = forever).
        on_step:    Optional callback(step_idx, reading, encoders, cmd,
                    controller) invoked each tick.
        controller: Optional pre-built `ReactiveController`.  Default: a
                    fresh one bound to `tunables`.
    """
    dt = 1.0 / tunables.loop_hz
    if controller is None:
        controller = ReactiveController(tunables)
    i = 0
    try:
        while max_steps is None or i < max_steps:
            reading = sensors.read()
            encoders = drive.read_encoders()
            imu_r = imu.read() if imu is not None else None
            # Path-1 perf instrumentation.  Time only the CPU-bound
            # work (controller.step + rate-limit) -- NOT sensor/IMU/drive
            # I/O, which on hardware is I2C-bound and doesn't scale with
            # cpu_slowdown_factor.  See `_perf_now_us` polyfill above.
            _t_perf0 = _perf_now_us()
            cmd = controller.step(reading, encoders, dt, imu_reading=imu_r)
            # Rate-limit at the algorithm/hardware boundary: caps the
            # delta between this tick's command and last tick's to
            # max_wheel_accel_mps2 * dt so the DRV8833 + N20 don't see
            # step-input commands.  See ReactiveController._rate_limit.
            cmd = controller._rate_limit(cmd, dt)
            _t_perf1 = _perf_now_us()
            _measured_us = _perf_diff_us(_t_perf1, _t_perf0)
            controller._perf_tick(_measured_us)
            # Path-2 wall-clock emulation.  When enabled, sleep the
            # difference between measured and projected per-tick time so
            # the wall-clock pacing matches the projected MCU.  Has no
            # effect on the simulated trajectory (which uses sim time via
            # clock.sleep below) -- only the matplotlib visualizer and
            # any external observers see the slowdown.  Skipped when the
            # extra delay would be negative or zero.
            if tunables.cpu_wallclock_emulate:
                _extra_us = _measured_us * (tunables.cpu_slowdown_factor - 1.0)
                if _extra_us > 0.0:
                    time.sleep(_extra_us * 1e-6)
            drive.set_wheel_speeds(cmd)
            if on_step is not None:
                on_step(i, reading, encoders, cmd, controller)
            clock.sleep(dt)
            i += 1
    finally:
        drive.stop()
    return i
