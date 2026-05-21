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

    # --- Aggression label (informational; behaviour comes from the
    # numeric tunables above set by a profile).  Echoed into the
    # telemetry header so logs are self-identifying. -------------------
    "aggression_mode":            "normal",

    # --- Sim-only (ignored on real hardware) -----------------------------
    "sim_wheel_tau_s":            0.04,    # 1st-order lag time constant
    # Max integration substep inside SimClock.sleep().  At high commanded
    # speeds, 5 ms substep lets the robot cover more than half its chassis
    # radius before the collision check runs -- bring this down for race
    # mode.  Default is safe up to ~3 m/s.
    "sim_max_substep_s":          0.005,

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
