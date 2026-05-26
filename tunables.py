"""Central tunable parameter table.

One flat class -- every magic number in the project lives here.  Names are
prefix-grouped for readability but the namespace is flat for trivially
simple overrides.

Three ways to populate one:
  - `Tunables()`                                  defaults
  - `Tunables.from_json_file("profiles/x.json")`  saved profile
  - `Tunables.from_overrides(["cruise_speed_mps=0.4", ...])`  CLI tweaks

Defaults are sized for the demo hardware:
  Maker Pi RP2040, 2x N20 6V 500 RPM (~85 mm wheelbase), 32 mm wheels,
  3x VL53L0X (front + left/right at +/- 45 deg) via TCA9548A.

Keep this CircuitPython-portable: plain class, no @dataclass, no `typing`.
"""

import json


# -----------------------------------------------------------------------------
# Defaults (single source of truth).  Each key is its own line so adding a
# new tunable is a one-line patch.
# -----------------------------------------------------------------------------

_DEFAULTS = {
    # --- Robot geometry --------------------------------------------------
    "wheel_diameter_m":           0.032,
    "wheel_base_m":               0.085,
    "chassis_radius_m":           0.035,
    "sensor_forward_offset_m":    0.03,
    "side_sensor_angle_rad":      0.7853981633974483,  # pi/4

    # --- Mechanical limits ----------------------------------------------
    # Used both by the risk-aware speed scaler (algorithm) and the sim
    # wheel model (acceleration cap).
    "max_decel_mps2":             1.5,
    "max_wheel_accel_mps2":       5.0,

    # --- Sensor model ----------------------------------------------------
    "sensor_max_range_m":         1.2,
    "sensor_noise_sigma_m":       0.003,

    # --- Outer-loop control thresholds -----------------------------------
    "front_stop_m":               0.12,
    "side_min_m":                 0.07,
    "side_target_m":              0.08,

    # --- Speeds & steering -----------------------------------------------
    "cruise_speed_mps":           0.30,
    "max_speed_mps":              0.50,
    "min_speed_mps":              0.05,
    "turn_speed_mps":             0.15,
    "steer_gain":                 0.5,
    # Wall-centering bias: when both sides see finite walls, gently steer
    # toward balanced clearance.  0.0 disables.
    "wall_center_gain":           0.4,
    "wall_center_max_bias":       0.35,

    # --- Inner-loop wheel PI (cmd_mps -> PWM duty) -----------------------
    "encoder_kp":                 2.0,
    "encoder_ki":                 4.0,

    # --- Loop rate -------------------------------------------------------
    "loop_hz":                    50.0,

    # --- Stuck recovery --------------------------------------------------
    "stuck_speed_threshold_mps":  0.02,
    "stuck_time_s":               0.5,
    "reverse_time_s":             0.25,
    "pivot_time_s":               0.25,
    # When step() picks a pivot direction (front blocked), hold that
    # direction for at least this long.  Without hysteresis, near-equal
    # L/R readings flip the pivot direction every tick -> chatter.
    "pivot_hysteresis_s":         0.35,
    # If front stays blocked beyond this -> escalate to full recovery
    # (reverse + force-pivot).  Prevents endless in-place spin.
    "pivot_stall_s":              2.0,

    # --- Flood-fill planner (only used when ReactiveController is wired
    # with a `planner=` -- otherwise these are inert).  See planner.py. ---
    # Replan period.  The planner re-floods lazily on every observed wall
    # delta, so this is just a guard rail / heartbeat -- keep it small.
    "planner_replan_period_s":    0.1,
    # Cell size used by the planner's pose->cell math.  Must match the
    # maze the robot is actually running in (the tester sets this from
    # --cell-size).  Defaults to standard micromouse cell size (0.18 m).
    "planner_cell_size_m":        0.18,
    # When heading error exceeds this, the controller pivots in place
    # instead of driving forward.  Below it, drive forward with a small
    # proportional correction.
    "planner_turn_threshold_rad": 0.20,
    # Wall observations only happen when |theta - cardinal| is within this
    # band -- mid-pivot the 45-degree side rays don't map cleanly onto
    # cell sides, so we skip the observation rather than poison the map.
    "planner_observe_tol_rad":    0.15,

    # --- Risk-weighted planner cost factors ------------------------------
    # Each cell-to-cell step has cost 1.0 + (these).  Tuning these is the
    # core lever of "aggression": cautious mode raises planner_turn_cost
    # so the planner prefers fewer-turn paths even when longer; aggressive
    # lowers it so turns are cheap (you take them at speed).
    # 90-degree turn extra cost (cell units).
    "planner_turn_cost":          1.0,
    # 180-degree about-face extra cost.  Almost never useful -- the planner
    # should turn around only when there's no alternative.
    "planner_reverse_cost":       4.0,
    # Extra cost for crossing an unknown wall.  Higher = more conservative
    # about untrusted territory; the planner prefers known-open paths even
    # when slightly longer.
    "planner_unknown_cost":       0.5,
    # When True, the planner may upgrade an L-shaped pair of cardinal
    # steps into a single 45-deg diagonal cut through the shared corner
    # (`FloodFillPlanner.desired_motion`).  Saves ~29% of distance per L
    # and removes one stop-and-pivot.  Default off so cautious / normal
    # modes stay cardinal-only; enable via race.json or
    # `--tune planner_use_diagonals=true`.
    "planner_use_diagonals":      False,
    # When inside this fraction of a cell of the diagonal target's
    # center, the controller declares the diagonal done and re-enters
    # cardinal mode.  Smaller = more accurate but more likely to overshoot
    # and double back; larger = early-exit at the cost of trajectory
    # smoothness.  0.35 = 0.063 m for 0.18 m cells -- ~ chassis_radius.
    "planner_diagonal_arrive_frac": 0.35,
    # `_corner_passable` strictness.  True (default) requires all four
    # walls bracketing the diagonal corner to be **known-open**; unknowns
    # block the cut.  False = Maxim's original optimistic semantics, where
    # unknowns are treated as open -- only safe when the maze is already
    # fully mapped (e.g. final race lap after exploration is complete).
    "planner_diagonal_strict":    True,

    # --- Organic (arc) turning ----------------------------------------------
    # Forward velocity carried through a planner-driven turn.  0.0 = pure
    # stop-and-pivot (legacy default).  Above 0, the controller drives
    # forward at this speed *while* rotating, tracing an arc with radius
    #     r = arc_turn_v_mps * wheel_base_m / (2 * turn_speed_mps)
    # For wheelbase=0.085, turn_speed=0.5, arc_turn_v=0.4 -> r = 3.4 cm.
    # Comfortably fits inside a 0.18 m cell.
    #
    # Status (2026-05-23): the feature is implemented end-to-end including
    # pre-turn deceleration (`_brake_for_next_turn`) so the rate limiter
    # has time to bring the wheels to arc speed before the cell center.
    # BUT each arc displaces the robot ~3 cm off the cell center along the
    # turn diagonal.  Over many turns the drift accumulates and the
    # robot can wedge.  Default is therefore 0 (stop-and-pivot) until we
    # add either post-turn wall-centering correction or a path-based
    # controller.  Opt-in via `--tune arc_turn_v_mps=0.3` for testing.
    "arc_turn_v_mps":             0.0,
    # If the heading error to the target exceeds this, the controller
    # falls back to a pure pivot.  Arc turning a 180-deg about-face would
    # displace the robot ~2 chassis_radius -- too much.  Cap at ~103-deg.
    "arc_turn_max_err_rad":       1.8,
    # Arc angular velocity in rad/s.  When > 0, decouples the arc's
    # rotation rate from `turn_speed_mps` (which then only governs the
    # legacy in-place pivot).  Combined with `arc_turn_v_mps` this lets
    # you choose arc geometry directly:
    #     radius = arc_turn_v_mps / arc_turn_omega_rps
    # E.g. (v=0.5, omega=20) -> r=0.025m, duration=pi/40=0.079s,
    # displacement_perp=0.025m.  Default 0 = use turn_speed_mps
    # (legacy behaviour: radius = v * wheel_base / (2 * turn_speed)).
    "arc_turn_omega_rps":         0.0,

    # --- Path-tracking controller ----------------------------------------
    # Selects the planner-driven control strategy when --planner=flood_fill.
    # 'cell' = current cell-by-cell controller (default).
    # 'path' = pure-pursuit path tracker; smoother trajectories, handles
    # tight race speeds without wedging.
    "controller_mode":            "cell",
    # Pure-pursuit lookahead in metres: L = min + gain * v_cur.  Smaller =
    # tighter tracking but more oscillation; larger = smoother but cuts
    # corners.  At race tunables (cruise=2 m/s, gain=0.3, min=0.05) the
    # lookahead is ~0.65 m = 3-4 cells.
    "path_lookahead_min_m":       0.05,
    "path_lookahead_gain":        0.30,
    # Spacing of discrete waypoints along the planned path, in metres.
    "path_waypoint_spacing_m":    0.02,
    # Forward-speed cap for the path tracker (m/s).  0.0 = no cap.
    "path_track_v_max_mps":       0.0,
    # Off-path distance that triggers recovery and a forced replan.
    "path_offpath_recover_m":     0.08,

    # --- Aggression label (informational; behaviour comes from the
    # numeric tunables above set by a profile).  Echoed into the
    # telemetry header so logs are self-identifying. -------------------
    "aggression_mode":            "normal",

    # --- IMU (LSM6DS3TR-C on XIAO nRF52840 Sense; sim derives from
    # kinematics with these noise + bias params layered on) --------------
    # Accelerometer white-noise stdev in m/s^2.  LSM6DS3TR-C @ 2g is ~
    # 0.04 m/s^2/sqrt(Hz) -> ~0.4 m/s^2 @ 100 Hz BW.  Conservative default.
    "imu_noise_accel_mps2":       0.05,
    # Gyroscope white-noise stdev in rad/s.  LSM6DS3TR-C @ 250 dps is
    # ~0.007 rad/s/sqrt(Hz) -> ~0.07 rad/s @ 100 Hz BW.
    "imu_noise_gyro_rps":         0.015,
    # Constant gyro-z bias in rad/s.  Real boards have a non-zero offset
    # at boot; the fusion layer estimates and subtracts it.  Sim defaults
    # to a small non-zero value so the fusion code gets to demonstrate
    # bias rejection.
    "imu_bias_gyro_z_rps":        0.005,

    # --- Pose fusion (encoder + gyro complementary filter) ---------------
    # Weight on the gyro (high-freq) channel.  alpha in (0, 1); the
    # encoder-odometry term gets weight (1 - alpha).  Higher = trust the
    # gyro more (catches drift from wheel slip).  Sensible: 0.95-0.995.
    "fusion_gyro_alpha":          0.98,
    # Time constant for the bias estimator (s).  When the robot is moving
    # slowly enough that wheel-encoder heading is reliable, the bias
    # estimator drifts toward (gyro - encoder_rate).  Long tau = stable;
    # short tau = adaptive.
    "fusion_bias_tau_s":          20.0,
    # Boot-time tau for the bias estimator: for the first
    # `fusion_bias_warmup_s` seconds the estimator uses this shorter tau,
    # then switches to `fusion_bias_tau_s`.  Lets the bias converge from
    # 0 to its true value in a few seconds rather than ~20, so race-mode
    # runs don't accumulate lateral drift before the bias settles.
    "fusion_bias_warmup_tau_s":   2.0,
    "fusion_bias_warmup_s":       3.0,

    # --- Scan-matching SLAM (planner.KnownMap + ToF readings) ------------
    # Backward-compatible disable switch for the EKF.  Values above 0 no
    # longer scale corrections; EKF trust comes from covariance + R/Q.
    "slam_correction_gain":       0.3,
    # Legacy scalar-correction knobs kept for advisor/profile compatibility;
    # the EKF uses the process/measurement noise and gate tunables below.
    "slam_min_clearance_m":       0.02,
    "slam_max_residual_m":        0.08,
    "slam_deadband_m":            0.008,
    "slam_observe_tol_rad":       0.15,
    "slam_process_noise_x":       1.0e-4,   # m^2 / s
    "slam_process_noise_y":       1.0e-4,   # m^2 / s
    "slam_process_noise_theta":   1.0e-5,   # rad^2 / s
    "slam_measurement_noise":     9.0e-6,   # m^2 (sensor std ~3 mm)
    "slam_gate_sigma":            3.0,      # Mahalanobis gate width
    "slam_init_pos_var":          1.0e-4,
    "slam_init_theta_var":        1.0e-6,
    # Sub-sample the EKF measurement update.  Prediction still runs every
    # tick (cheap pose integration), but the expensive ray-cast +
    # Jacobian update only runs every Nth tick.  N=1 = every tick (legacy
    # default); N=3 cuts SLAM CPU by ~3x with ~15 ms of correction lag at
    # race speeds -- well within the planner's cell-attribution tolerance.
    "slam_measurement_period_ticks": 1,
    # SLAM measurement Jacobian flavour:
    #   "analytical" -- closed-form partials w.r.t. (x, y, theta) for the
    #     active wall segment each ray hits.  3 ray-casts per tick total.
    #     Default.  7-10x faster than central differences.
    #   "central"    -- legacy 6-perturbation central-difference Jacobian
    #     (18 ray-casts per tick).  Slower but more robust at ray/wall
    #     corner transitions.  Use for regression testing or if the
    #     analytical version has issues on a specific maze geometry.
    "slam_jacobian_mode":         "analytical",

    # --- Software PWM cap (DRV8833 thermal protection, from new spec) ----
    # Maximum duty cycle the inner-loop wheel controller is allowed to
    # request.  1.0 = no cap (default), 0.85 = 85% peak duty.  Useful as
    # a safety net for sustained-stall scenarios.
    "motor_duty_cap":             1.0,

    # --- Sim-only (ignored on real hardware) -----------------------------
    "sim_wheel_tau_s":            0.04,    # 1st-order lag time constant
    # Max integration substep inside SimClock.sleep().  At high commanded
    # speeds, 5 ms substep lets the robot cover more than half its chassis
    # radius before the collision check runs -- bring this down for race
    # mode.  Default is safe up to ~3 m/s.
    "sim_max_substep_s":          0.005,

    # --- CPU performance emulation (Path 1 diagnostic) -------------------
    # The Mac runs the algorithm orders of magnitude faster than the XIAO
    # nRF52840 Sense (64 MHz Cortex-M4 + CircuitPython interpreter).  We
    # measure the wall-clock time of controller.step()+_rate_limit() in
    # sim, then multiply by this factor to project the equivalent on-MCU
    # time.  Each tick is flagged as "overrun" when projected > budget.
    # No effect on the simulated physics -- pure diagnostic.
    #   1.0  = no emulation (default; pure timing without projection)
    #   ~50  = CPython on a Raspberry Pi class machine
    #   ~150 = MicroPython on Cortex-M4 @ 64 MHz
    #   ~250 = CircuitPython on Cortex-M4 @ 64 MHz (conservative)
    "cpu_slowdown_factor":        1.0,
    # Tick budget in microseconds.  0 = derived from loop_hz (1e6/loop_hz).
    # Explicit override is useful for "what if I had X µs to spare per
    # tick" what-if analysis.
    "perf_budget_us":             0.0,
    # Path-2 wall-clock emulation.  When True, after each tick the loop
    # sleeps for `measured_us * (cpu_slowdown_factor - 1)` microseconds,
    # forcing the *wall-clock* per-tick time to scale up to the projected
    # MCU value.  Sim physics still advances by dt = 1/loop_hz, so the
    # simulated trajectory is unchanged -- but the visualizer renders at
    # the projected hardware speed, useful for "feel" testing whether the
    # algorithm copes when each control decision lags real time.  Slows
    # the run dramatically; use cpu_slowdown_factor=10-50 for casual feel
    # testing rather than the full 200x.
    "cpu_wallclock_emulate":      False,

    # --- Telemetry -------------------------------------------------------
    "telem_enabled":              True,
    "telem_log_every_nth":        1,
}


# Keys whose default is a bool -- needed for from_overrides coercion since
# bool is a subclass of int in Python.
_BOOL_KEYS = tuple(k for k, v in _DEFAULTS.items() if isinstance(v, bool))


class Tunables(object):
    """Central tunable parameter table.  See module docstring."""

    # Slots = keys of _DEFAULTS, so a typo on assignment AttributeError's.
    __slots__ = tuple(_DEFAULTS.keys())

    def __init__(self, **overrides):
        for k, v in _DEFAULTS.items():
            setattr(self, k, v)
        for k, v in overrides.items():
            if k not in _DEFAULTS:
                raise KeyError("Unknown tunable: {}".format(k))
            setattr(self, k, v)

    # ---- introspection -----------------------------------------------------

    def to_dict(self):
        return {k: getattr(self, k) for k in self.__slots__}

    def diff(self, other=None):
        """Return only the keys whose value differs from defaults (or other)."""
        baseline = other.to_dict() if other is not None else _DEFAULTS
        out = {}
        for k in self.__slots__:
            v = getattr(self, k)
            if v != baseline.get(k):
                out[k] = v
        return out

    # ---- JSON I/O ---------------------------------------------------------

    @classmethod
    def from_dict(cls, d):
        return cls(**d)

    @classmethod
    def from_json_file(cls, path):
        f = open(path, "r")
        try:
            d = json.load(f)
        finally:
            f.close()
        return cls(**d)

    def to_json_file(self, path, only_diff=False):
        d = self.diff() if only_diff else self.to_dict()
        f = open(path, "w")
        try:
            # `json.dump(... indent=2)` works in CircuitPython too.
            json.dump(d, f)
            f.write("\n")
        finally:
            f.close()

    # ---- CLI override parsing --------------------------------------------

    @classmethod
    def from_overrides(cls, items, base=None):
        """Apply ['key=value', ...] strings on top of `base` (or defaults).

        Value type is coerced from the default's type (bool/int/float/str).
        Raises KeyError on unknown keys, ValueError on bad values.
        """
        if base is None:
            t = cls()
        else:
            t = cls(**base.to_dict())
        for item in items:
            if "=" not in item:
                raise ValueError("Override must be key=value: {!r}".format(item))
            k, _, raw = item.partition("=")
            k = k.strip()
            raw = raw.strip()
            if k not in _DEFAULTS:
                raise KeyError("Unknown tunable: {}".format(k))
            default = _DEFAULTS[k]
            if k in _BOOL_KEYS:
                v = raw.lower() in ("1", "true", "yes", "on")
            elif isinstance(default, int):
                v = int(raw)
            elif isinstance(default, float):
                v = float(raw)
            else:
                v = raw
            setattr(t, k, v)
        return t

    # ---- pretty print -----------------------------------------------------

    def __repr__(self):
        diffs = self.diff()
        if not diffs:
            return "Tunables(defaults)"
        parts = ["{}={}".format(k, v) for k, v in sorted(diffs.items())]
        return "Tunables({})".format(", ".join(parts))


def default_keys():
    """Public accessor for the set of valid keys (e.g. for help text)."""
    return tuple(_DEFAULTS.keys())


def default_value(key):
    return _DEFAULTS[key]
