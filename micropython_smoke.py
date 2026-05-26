"""Smoke test for CircuitPython-compatible code under MicroPython.

Path 3 of the XIAO performance emulation series.  Whereas Path 1
projects timing and Path 2 enforces wall-clock pacing, this test
runs the CircuitPython-portable modules under MicroPython (a close
cousin of CircuitPython) to catch runtime compatibility issues
before they bite on real hardware:

  - missing stdlib methods (e.g. dict.fromkeys signature differences)
  - subtle class/__slots__ behavior diffs
  - features that parse on CPython but fail at runtime on MicroPython

The host-only modules (sim/, tester.py, stress.py, telemetry.py,
tuning.py) are NOT exercised -- they are CPython-only by design.

Setup (one-time):
  brew install micropython

Run:
  micropython micropython_smoke.py

A non-zero exit code or any 'FAIL' line indicates an incompatibility
the XIAO would also hit.
"""

import sys

# Make the project root importable when run from any cwd.  MicroPython
# accepts sys.path.insert the same way CPython does.
sys.path.insert(0, ".")


def _step(label):
    print("  [{}] ...".format(label), end="")


def _ok():
    print(" OK")


# -----------------------------------------------------------------------------
# 1. Module imports.  Catches syntax-level incompatibility (f-strings, typing
#    imports, dataclass usage, etc.) which would fail at parse time.
# -----------------------------------------------------------------------------

_step("import interfaces")
import interfaces  # noqa: E402
_ok()

_step("import tunables")
import tunables  # noqa: E402
_ok()

_step("import planner")
import planner  # noqa: E402
_ok()

_step("import pose_fusion")
import pose_fusion  # noqa: E402
_ok()

_step("import slam")
import slam  # noqa: E402
_ok()

_step("import algorithm")
import algorithm  # noqa: E402
_ok()


# -----------------------------------------------------------------------------
# 2. Tunables: instantiation, override, JSON-style round-trip.
# -----------------------------------------------------------------------------

_step("Tunables defaults + override")
T = tunables.Tunables()
assert T.loop_hz == 50.0, "loop_hz default wrong"
T.cruise_speed_mps = 0.4
assert T.cruise_speed_mps == 0.4
_ok()

_step("Tunables.from_overrides")
T2 = tunables.Tunables.from_overrides(["loop_hz=100", "cpu_slowdown_factor=200"])
assert T2.loop_hz == 100.0
assert T2.cpu_slowdown_factor == 200.0
_ok()


# -----------------------------------------------------------------------------
# 3. Planner: KnownMap + FloodFillPlanner basic flow.
# -----------------------------------------------------------------------------

_step("FloodFillPlanner construction + replan")
plan = planner.FloodFillPlanner(
    cols=8, rows=8, goal_cell=(7, 7),
    cell_size_m=0.18,
    turn_cost=1.0, reverse_cost=4.0, unknown_cost=0.5,
)
plan.replan()
_ok()

_step("planner.desired_heading from (0,0) heading N")
desired = plan.desired_heading((0, 0), 0)
assert desired in (0, 1, 2, 3), "bad desired heading: {}".format(desired)
_ok()


# -----------------------------------------------------------------------------
# 4. FusedOdometry: encoder + gyro fusion.
# -----------------------------------------------------------------------------

_step("FusedOdometry update + pose")
fused = pose_fusion.FusedOdometry(0.0, 0.0, 0.0, T)
imu_r = interfaces.IMUReading(0.0, 0.0, 9.81, 0.0, 0.0, 0.1, timestamp=0.0)
fused.update(0.1, 0.1, 0.01, imu_reading=imu_r)
x, y, theta = fused.pose()
# After a tiny straight-forward step the robot should have moved forward
# along its initial heading (north -> +y on the convention used here).
assert abs(x) < 0.01, "unexpected lateral drift: {}".format(x)
_ok()

# NOTE: FusedOdometry intentionally has no dead_reckoning_pose() -- only
# SLAM does (to expose the pre-correction smooth pose; see Bug #29).


# -----------------------------------------------------------------------------
# 5. SLAM EKF: prediction + measurement update.
# -----------------------------------------------------------------------------

_step("ScanMatchSlam construction")
ekf = slam.ScanMatchSlam(0.0, 0.0, 0.0, plan.map, T)
_ok()

_step("ScanMatchSlam.update (one tick, no walls seen yet)")
reading = interfaces.Reading(1.0, 1.0, 1.0, timestamp=0.0)
ekf.update(0.1, 0.1, 0.01, imu_reading=imu_r, reading=reading)
_ok()

_step("ScanMatchSlam.pose vs dead_reckoning_pose")
ekf_pose = ekf.pose()
ekf_dr = ekf.dead_reckoning_pose()
# Both are 3-tuples (x, y, theta); SLAM correction so far is ~0 (no
# innovations yet on no-wall observation), so they should be close.
assert len(ekf_pose) == 3 and len(ekf_dr) == 3
_ok()


# -----------------------------------------------------------------------------
# 6. ReactiveController: full closed-loop tick.
# -----------------------------------------------------------------------------

_step("ReactiveController.step (planner + estimator wired)")
ctrl = algorithm.ReactiveController(
    T,
    planner=plan,
    pose_provider=fused.pose,
    estimator=fused,
)
encoders = (0.1, 0.1)
cmd = ctrl.step(reading, encoders, 0.02, imu_reading=imu_r)
assert hasattr(cmd, "left") and hasattr(cmd, "right"), "bad cmd type"
_ok()

_step("ReactiveController._rate_limit")
cmd2 = ctrl._rate_limit(cmd, 0.02)
assert hasattr(cmd2, "left")
_ok()


# -----------------------------------------------------------------------------
# 7. Path-1 perf instrumentation hooks.
# -----------------------------------------------------------------------------

_step("perf_tick + perf_summary")
ctrl._perf_tick(100.0)  # 100 us measured
ctrl._perf_tick(200.0)
summary = ctrl.perf_summary()
assert summary["ticks"] == 2
assert summary["avg_us"] == 150.0
assert summary["max_us"] == 200.0
_ok()


# -----------------------------------------------------------------------------
# 8. algorithm.step() (the pure-function reactive layer).
# -----------------------------------------------------------------------------

_step("algorithm.step pure function")
near = interfaces.Reading(0.05, 1.0, 1.0, timestamp=0.0)  # front blocked
cmd_p = algorithm.step(near, (0.0, 0.0), T)
# Front blocked at 5cm < front_stop_m (12cm) -> pivot.  cmd.left * cmd.right < 0.
assert cmd_p.left * cmd_p.right < 0, "expected pivot when front blocked"
_ok()


print()
print("ALL CHECKS PASSED on", sys.version)
