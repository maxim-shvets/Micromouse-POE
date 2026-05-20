"""Heuristic tuning advisor.

Eats a telemetry log (list of sample dicts from `telemetry.TelemetryRecorder`)
and emits ranked `Suggestion`s pointing at specific `Tunables` keys.

The rules are deliberately conservative -- each one fires only on clear
patterns, and each names *one* tunable to nudge along with the direction.
The intent is: when the demo misbehaves on real hardware, run this and
read it like a checklist of physically meaningful first guesses.

Pure stdlib so it can run on the host or be ported to the board.
"""

import math


# Severity ordering for printing.  Higher = more urgent.
SEV_LOW = 1
SEV_MED = 2
SEV_HIGH = 3

_SEV_LABEL = {SEV_LOW: "LOW", SEV_MED: "MED", SEV_HIGH: "HIGH"}


# State ints from algorithm.ReactiveController -- duplicated to avoid import
# cycles and keep this analyser standalone.
_S_REACT = 0
_S_REVERSE = 1
_S_PIVOT = 2


class Suggestion(object):
    __slots__ = ("severity", "tunable", "direction", "current", "reason")

    def __init__(self, severity, tunable, direction, current, reason):
        self.severity = severity
        self.tunable = tunable
        self.direction = direction  # "UP", "DOWN", "TUNE"
        self.current = current
        self.reason = reason

    def __repr__(self):
        return "{} {}={} -> {}  {}".format(
            _SEV_LABEL[self.severity], self.tunable, self.current,
            self.direction, self.reason,
        )


# -----------------------------------------------------------------------------
# Entry point
# -----------------------------------------------------------------------------

def analyse(samples, tunables):
    """Return a severity-sorted list of `Suggestion`s.

    Args:
        samples:  list of telemetry sample dicts.
        tunables: `Tunables` snapshot used for the run.
    """
    if not samples:
        return []
    m = _metrics(samples, tunables)
    out = []
    for rule in _RULES:
        s = rule(m, tunables)
        if s is not None:
            out.append(s)
    out.sort(key=lambda s: -s.severity)
    return out


def format_report(suggestions, metrics=None):
    """Render suggestions + (optionally) raw metrics as plain text."""
    lines = []
    if not suggestions:
        lines.append("No tuning suggestions -- run looks within nominal envelope.")
    else:
        lines.append("Tuning suggestions ({}):".format(len(suggestions)))
        for s in suggestions:
            lines.append("  [{}] {:24s} = {:<8} -> {:4s}   {}".format(
                _SEV_LABEL[s.severity], s.tunable, _fmt_val(s.current),
                s.direction, s.reason,
            ))
    if metrics is not None:
        lines.append("")
        lines.append("Run metrics:")
        for k, v in sorted(metrics.items()):
            lines.append("  {:32s} {}".format(k, _fmt_val(v)))
    return "\n".join(lines)


def _fmt_val(v):
    if isinstance(v, float):
        return "{:.3f}".format(v)
    return str(v)


# -----------------------------------------------------------------------------
# Metric extraction
# -----------------------------------------------------------------------------

def _metrics(samples, T):
    """Crunch a list of samples into the named scalars the rules consume."""
    n = len(samples)
    duration = samples[-1]["t"] - samples[0]["t"]
    if duration <= 0.0:
        duration = 1e-6

    # Counts by state.
    react = 0
    reverse = 0
    pivot = 0
    pivot_in_react = 0     # opposite-sign cmds during REACT (front-block pivots)
    track_err_acc = 0.0
    track_err_n = 0
    speed_clipped_n = 0
    speed_clipped_denom = 0
    heading_rates = []     # for oscillation
    cmd_jerk = []          # consecutive cmd diffs (proxy for control chatter)
    min_side_during_react = []

    prev_theta = None
    prev_t = None
    prev_cmd_l = None
    prev_cmd_r = None
    cruise = T.cruise_speed_mps
    min_floor = T.min_speed_mps
    front_stop = T.front_stop_m

    for s in samples:
        st = s.get("state", 0)
        if st == _S_REACT:
            react += 1
        elif st == _S_REVERSE:
            reverse += 1
        elif st == _S_PIVOT:
            pivot += 1

        cmd_l = s["cmd_l"]
        cmd_r = s["cmd_r"]
        enc_l = s["enc_l"]
        enc_r = s["enc_r"]

        # Pivot in REACT: opposite-sign cmds while reactive (front blocked).
        if st == _S_REACT and cmd_l * cmd_r < 0:
            pivot_in_react += 1

        # Encoder tracking error (only during REACT and only when commanding
        # appreciable motion, to avoid divide-by-near-zero noise).
        if st == _S_REACT:
            for cmd, enc in ((cmd_l, enc_l), (cmd_r, enc_r)):
                if abs(cmd) > 2 * min_floor:
                    track_err_acc += abs(cmd - enc) / abs(cmd)
                    track_err_n += 1

        # Speed clipping: REACT, going forward (both wheels same sign +) and
        # front clearance well above the stop threshold but cmd well below
        # cruise.
        if st == _S_REACT and cmd_l > 0 and cmd_r > 0 and s["front"] > 2 * front_stop:
            speed_clipped_denom += 1
            avg_cmd = 0.5 * (cmd_l + cmd_r)
            if avg_cmd < 0.7 * cruise:
                speed_clipped_n += 1

        # Heading rate -- proxy for oscillation when not commanded to pivot.
        if "theta" in s:
            if prev_theta is not None and st == _S_REACT and cmd_l > 0 and cmd_r > 0:
                dt = s["t"] - prev_t
                if dt > 1e-9:
                    dth = _wrap(s["theta"] - prev_theta)
                    heading_rates.append(dth / dt)
            prev_theta = s["theta"]
            prev_t = s["t"]

        if prev_cmd_l is not None:
            cmd_jerk.append(abs(cmd_l - prev_cmd_l) + abs(cmd_r - prev_cmd_r))
        prev_cmd_l = cmd_l
        prev_cmd_r = cmd_r

        if st == _S_REACT:
            min_side_during_react.append(min(s["left"], s["right"]))

    last = samples[-1]
    dist = last.get("dist", 0.0)
    coll = last.get("coll", 0)
    recov = last.get("recov", 0)

    return {
        "duration_s": duration,
        "n_samples": n,
        "frac_react":       react / n,
        "frac_reverse":     reverse / n,
        "frac_pivot":       pivot / n,
        "pivot_in_react":   pivot_in_react / max(react, 1),
        "recoveries_per_s": recov / duration,
        "collisions":       coll,
        "distance_m":       dist,
        "coll_per_m":       (coll / dist) if dist > 0.05 else 0.0,
        "avg_speed_mps":    dist / duration,
        "track_err_med":    _median(track_err_acc / max(track_err_n, 1)
                                    if track_err_n else 0.0),
        "speed_clipped_frac": (speed_clipped_n / speed_clipped_denom)
                              if speed_clipped_denom else 0.0,
        "heading_rate_stdev": _stdev(heading_rates),
        "cmd_jerk_mean":    sum(cmd_jerk) / len(cmd_jerk) if cmd_jerk else 0.0,
        "side_avg_react":   (sum(min_side_during_react) / len(min_side_during_react))
                            if min_side_during_react else 0.0,
    }


def _median(x):
    # `_metrics` already accumulated a mean -- this is a passthrough kept
    # for symmetry / future swap to true median.
    return x


def _stdev(xs):
    n = len(xs)
    if n < 2:
        return 0.0
    mean = sum(xs) / n
    var = sum((x - mean) ** 2 for x in xs) / (n - 1)
    return math.sqrt(var)


def _wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


# -----------------------------------------------------------------------------
# Rules
# -----------------------------------------------------------------------------

def _rule_collisions(m, T):
    if m["distance_m"] < 0.5:
        return None
    cpm = m["coll_per_m"]
    if cpm > 0.3:
        sev = SEV_HIGH
    elif cpm > 0.1:
        sev = SEV_MED
    else:
        return None
    return Suggestion(sev, "cruise_speed_mps", "DOWN", T.cruise_speed_mps,
        "{:.2f} collisions/m -- consider also raising front_stop_m / side_min_m".format(cpm))


def _rule_recoveries(m, T):
    rps = m["recoveries_per_s"]
    if rps > 0.4:
        sev = SEV_HIGH
    elif rps > 0.15:
        sev = SEV_MED
    else:
        return None
    return Suggestion(sev, "front_stop_m", "UP", T.front_stop_m,
        "{:.2f} recoveries/s -- wedging at corners; raise stop threshold or wall_center_gain".format(rps))


def _rule_track_err(m, T):
    e = m["track_err_med"]
    if e > 0.35:
        sev = SEV_HIGH
    elif e > 0.2:
        sev = SEV_MED
    else:
        return None
    return Suggestion(sev, "encoder_kp", "UP", T.encoder_kp,
        "{:.0%} median wheel-speed error -- inner-loop too slack OR max_wheel_accel_mps2 limiting".format(e))


def _rule_speed_clipped(m, T):
    f = m["speed_clipped_frac"]
    if f > 0.6:
        sev = SEV_MED
    elif f > 0.3:
        sev = SEV_LOW
    else:
        return None
    return Suggestion(sev, "max_decel_mps2", "UP", T.max_decel_mps2,
        "speed clipped below cruise {:.0%} of forward-cruise time -- claim more braking OR lower cruise_speed_mps".format(f))


def _rule_pivot_dominance(m, T):
    f = m["pivot_in_react"]
    if f > 0.4:
        sev = SEV_MED
    elif f > 0.2:
        sev = SEV_LOW
    else:
        return None
    return Suggestion(sev, "wall_center_gain", "UP", T.wall_center_gain,
        "front-block pivots {:.0%} of REACT ticks -- centering may be missing OR front_stop_m too high".format(f))


def _rule_oscillation(m, T):
    sd = m["heading_rate_stdev"]
    # rad/s during nominally straight cruise.  Tune threshold by feel.
    if sd > 3.0:
        sev = SEV_MED
    elif sd > 1.5:
        sev = SEV_LOW
    else:
        return None
    return Suggestion(sev, "steer_gain", "DOWN", T.steer_gain,
        "heading-rate stdev {:.2f} rad/s during cruise -- reduce steer_gain or wall_center_gain".format(sd))


def _rule_slow_average(m, T):
    if m["distance_m"] < 0.5:
        return None
    if m["avg_speed_mps"] < 0.25 * T.cruise_speed_mps:
        return Suggestion(SEV_MED, "cruise_speed_mps", "TUNE", T.cruise_speed_mps,
            "avg speed {:.2f} m/s << cruise -- check stuck recovery / sensor noise / risk floor".format(
                m["avg_speed_mps"]))
    return None


def _rule_tight_side_avg(m, T):
    s_avg = m["side_avg_react"]
    if s_avg == 0.0:
        return None
    # If the minimum side ray during REACT averages near side_min_m, the
    # robot is constantly skimming the trigger threshold.
    if s_avg < T.side_min_m * 1.1:
        return Suggestion(SEV_LOW, "side_min_m", "TUNE", T.side_min_m,
            "min-side avg {:.3f}m hovers near side_min_m -- threshold may be chattering".format(s_avg))
    return None


_RULES = (
    _rule_collisions,
    _rule_recoveries,
    _rule_track_err,
    _rule_speed_clipped,
    _rule_pivot_dominance,
    _rule_oscillation,
    _rule_slow_average,
    _rule_tight_side_avg,
)


# -----------------------------------------------------------------------------
# Convenience: full report from a recorder
# -----------------------------------------------------------------------------

def report_from_recorder(recorder, verbose=False):
    """Take a `TelemetryRecorder` and return the formatted advisor text."""
    metrics = _metrics(recorder.samples, recorder.tunables)
    suggestions = analyse(recorder.samples, recorder.tunables)
    return format_report(suggestions, metrics if verbose else None)
