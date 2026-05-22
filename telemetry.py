"""Per-tick telemetry recorder.

A `TelemetryRecorder` is a callable you pass into `algorithm.run` as
`on_step`.  Each tick it captures one `Sample` (a plain dict) summarising
the controller's view of the world and what it commanded.

In-memory storage is the source of truth.  Optionally, samples are
streamed to a JSONL file (one JSON object per line) for offline analysis
or replay.

Designed to run on both the sim and the RP2040 -- only standard library
imports and small per-sample memory.  Cap the buffer with `max_samples`
on the board to avoid OOM.

The companion analyser is `tuning.py`.
"""

import json


SCHEMA_VERSION = 1


class TelemetryRecorder(object):
    """Callable.  Pass instance as algorithm.run(on_step=recorder)."""

    def __init__(self, tunables, log_path=None, max_samples=None,
                 world=None):
        """Args:
            tunables:    `Tunables`.  Recorded once in the header for
                         later tuning analysis.
            log_path:    Optional path to write JSONL.  None = memory only.
            max_samples: Cap the in-memory ring (oldest dropped).  None =
                         unbounded.
            world:       Optional `SimWorld`.  When present, pose +
                         per-event counts (collisions, recoveries) are
                         recorded too.  Pass None on real hardware.
        """
        self.tunables = tunables
        self.log_path = log_path
        self.max_samples = max_samples
        self.world = world
        self.samples = []
        # Per-stride downsampling (uses tunables.telem_log_every_nth).
        self._n = max(1, int(getattr(tunables, "telem_log_every_nth", 1)))
        self._file = None
        if log_path is not None:
            self._file = open(log_path, "w")
            header = {
                "schema": SCHEMA_VERSION,
                "tunables": tunables.to_dict(),
            }
            self._file.write(json.dumps(header))
            self._file.write("\n")

    # ---- on_step callback signature ---------------------------------------

    def __call__(self, i, reading, encoders, cmd, controller):
        if (i % self._n) != 0:
            return
        sample = {
            "i": i,
            "t": reading.timestamp,
            "front": _r3(reading.front),
            "left": _r3(reading.left),
            "right": _r3(reading.right),
            "enc_l": _r3(encoders[0]),
            "enc_r": _r3(encoders[1]),
            "cmd_l": _r3(cmd.left),
            "cmd_r": _r3(cmd.right),
            "state": controller.state,
            "stuck_t": _r3(controller.stuck_t),
            "recov": controller.recovery_count,
        }
        # IMU fields are present iff an IMU was wired into algorithm.run
        # this run; otherwise we omit them to keep the log small.
        imu_r = getattr(controller, "imu_reading", None)
        if imu_r is not None:
            sample["ax"] = _r3(imu_r.accel_x)
            sample["ay"] = _r3(imu_r.accel_y)
            sample["az"] = _r3(imu_r.accel_z)
            sample["wx"] = _r3(imu_r.gyro_x)
            sample["wy"] = _r3(imu_r.gyro_y)
            sample["wz"] = _r3(imu_r.gyro_z)
        if self.world is not None:
            sample["x"] = _r3(self.world.x)
            sample["y"] = _r3(self.world.y)
            sample["theta"] = _r3(self.world.theta)
            sample["coll"] = self.world.collisions
            sample["dist"] = _r3(self.world.distance_traveled)
        self.samples.append(sample)
        if self.max_samples is not None and len(self.samples) > self.max_samples:
            # Drop oldest.  O(n) but only happens if cap is set + exceeded.
            excess = len(self.samples) - self.max_samples
            del self.samples[:excess]
        if self._file is not None:
            self._file.write(json.dumps(sample))
            self._file.write("\n")

    # ---- lifecycle --------------------------------------------------------

    def close(self):
        if self._file is not None:
            self._file.close()
            self._file = None

    # ---- post-hoc helpers -------------------------------------------------

    def duration_s(self):
        if not self.samples:
            return 0.0
        return self.samples[-1]["t"] - self.samples[0]["t"]

    def tick_count(self):
        return len(self.samples)


def _r3(v):
    """Round to 3 decimals to keep the log compact."""
    try:
        return round(float(v), 3)
    except (TypeError, ValueError):
        return v


def load_jsonl(path):
    """Read a JSONL telemetry file -> (header_dict, [sample, ...]).

    Returns (None, samples) when no header is present.
    """
    header = None
    samples = []
    f = open(path, "r")
    try:
        for ln, raw in enumerate(f):
            raw = raw.strip()
            if not raw:
                continue
            obj = json.loads(raw)
            if ln == 0 and "schema" in obj:
                header = obj
                continue
            samples.append(obj)
    finally:
        f.close()
    return header, samples
