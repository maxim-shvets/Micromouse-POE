"""Benchmark the controller tick under MicroPython vs CPython.

Run under both runtimes to get a slowdown ratio:

  $ python3   micropython_bench.py
  $ micropython micropython_bench.py

The output's "us per tick" lets us calibrate `cpu_slowdown_factor` in
tunables.py.  Mac CircuitPython on Cortex-M4 @ 64 MHz is expected to be
~3-5x slower than MicroPython on Mac AArch64 (clock-speed scaling), and
MicroPython on Mac is itself ~30-80x slower than CPython on Mac due to
interpreter overhead alone.  Combined estimate: ~150-250x.
"""

import sys

sys.path.insert(0, ".")

import interfaces
import tunables
import planner
import pose_fusion
import slam
import algorithm
# Pull the same portable timer the production loop uses.
_now_us = algorithm._perf_now_us
_diff_us = algorithm._perf_diff_us


# Set up the same workload as a real tick: planner-wrapped reactive
# controller with SLAM estimator, planner-driven path.

T = tunables.Tunables()
T.loop_hz = 200.0   # race-mode tick rate

plan = planner.FloodFillPlanner(
    cols=8, rows=8, goal_cell=(7, 7),
    cell_size_m=0.18,
    turn_cost=1.0, reverse_cost=4.0, unknown_cost=0.5,
)
plan.replan()

ekf = slam.ScanMatchSlam(0.0, 0.0, 0.0, plan.map, T)
ctrl = algorithm.ReactiveController(
    T, planner=plan,
    pose_provider=ekf.pose,
    estimator=ekf,
    observation_pose_provider=ekf.dead_reckoning_pose,
)

# Synthetic but realistic per-tick inputs (mid-corridor, in-cell).
reading = interfaces.Reading(1.0, 0.18, 0.18, timestamp=0.0)
imu_r = interfaces.IMUReading(0.0, 0.0, 9.81, 0.0, 0.0, 0.0, timestamp=0.0)
encoders = (0.5, 0.5)
dt = 1.0 / T.loop_hz

# Warm up (planner first replan dominates the first few ticks).
for _ in range(20):
    ctrl.step(reading, encoders, dt, imu_reading=imu_r)

# Time the steady state.
N = 500
t0 = _now_us()
for _ in range(N):
    cmd = ctrl.step(reading, encoders, dt, imu_reading=imu_r)
    ctrl._rate_limit(cmd, dt)
t1 = _now_us()

elapsed_us = _diff_us(t1, t0)
per_tick_us = elapsed_us / N
print("runtime           :", sys.version)
print("ticks measured    :", N)
print("total elapsed     : {:.0f} us".format(elapsed_us))
print("us per tick       : {:.1f}".format(per_tick_us))
print("equivalent loop_hz: {:.0f}".format(1e6 / per_tick_us))
