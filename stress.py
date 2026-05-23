"""Stress test runner.

Sweeps across (seed, cols, rows, mode, pose_source).  Each run records
rich diagnostics so we can categorise failure modes:

  - result: GOAL | TIMEOUT | COLL_BUDGET
  - sim_time, distance_m, avg_speed, peak_speed
  - collisions, recovery_count
  - goal_closest_m -- closest the robot ever got to the goal cell center
  - max_idle_window_s -- longest contiguous window of near-zero motion
    while the controller commanded non-zero speed (= "stuck without
    recovery firing", a separate failure mode from REACT/REVERSE/PIVOT
    oscillation)
  - reverses_per_min -- recovery rate (high = oscillation loop)

Writes a CSV; prints a summary by failure category.

Run with:  python3 stress.py [--quick]
"""

import argparse
import csv
import math
import os
import sys
import time

# Make the package importable when run from any cwd.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from algorithm import run, ReactiveController
from sim.maze import Maze
from sim.world import SimWorld, SimSensors, SimDrive, SimClock, SimIMU
from tunables import Tunables


# Sweep grids.  --quick reduces sizes for fast iteration.
_DEFAULT_SEEDS = [1, 7, 13, 21, 42, 99, 100, 314, 777, 1000]
_DEFAULT_SIZES = [(5, 5), (8, 8), (10, 10), (12, 12), (16, 16)]
_DEFAULT_MODES = ["cautious", "normal", "aggressive", "race"]
# slam is included so the harness exercises the on-hardware code path.
_DEFAULT_POSE = ["ground_truth", "fused", "slam"]

_QUICK_SEEDS = [1, 42, 100]
_QUICK_SIZES = [(8, 8), (12, 12)]
_QUICK_MODES = ["normal", "aggressive"]
_QUICK_POSE = ["ground_truth"]


def _load_mode_tunables(mode, cell_size_m):
    """Load profile tunables + force cell size to match the sim maze."""
    import json
    here = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(here, "profiles", mode + ".json")
    if os.path.exists(path):
        t = Tunables.from_json_file(path)
    else:
        t = Tunables()
    t.planner_cell_size_m = cell_size_m
    return t


def _build_controller(pose_source, tun, world, maze):
    from planner import FloodFillPlanner
    planner = FloodFillPlanner(
        cols=maze.cols, rows=maze.rows,
        goal_cell=maze.goal_cell,
        cell_size_m=tun.planner_cell_size_m,
        turn_cost=tun.planner_turn_cost,
        reverse_cost=tun.planner_reverse_cost,
        unknown_cost=tun.planner_unknown_cost,
    )
    estimator = None
    if pose_source == "ground_truth":
        pose_provider = lambda: (world.x, world.y, world.theta)
    elif pose_source == "fused":
        from pose_fusion import FusedOdometry
        estimator = FusedOdometry(world.x, world.y, world.theta, tun)
        pose_provider = estimator.pose
    elif pose_source == "slam":
        from slam import ScanMatchSlam
        estimator = ScanMatchSlam(world.x, world.y, world.theta,
                                  planner.map, tun)
        pose_provider = estimator.pose
    else:
        raise ValueError("unknown pose_source: " + pose_source)
    return ReactiveController(tun, planner=planner,
                              pose_provider=pose_provider,
                              estimator=estimator)


def _run_one(seed, cols, rows, mode, pose_source, sim_time_cap_s,
             collision_budget):
    """Execute one stress run; return a dict of diagnostics."""
    tun = _load_mode_tunables(mode, 0.18)
    maze = Maze(cols, rows, cell_size_m=0.18, seed=seed)
    world = SimWorld(maze, tun)
    sensors = SimSensors(world)
    drive = SimDrive(world)
    clock = SimClock(world)
    imu = SimIMU(world)
    controller = _build_controller(pose_source, tun, world, maze)

    goal_x, goal_y = maze.cell_center(*maze.goal_cell)
    diag = {
        "seed": seed, "cols": cols, "rows": rows,
        "mode": mode, "pose": pose_source,
        "result": "TIMEOUT",
        "goal_closest_m": float("inf"),
        "max_idle_window_s": 0.0,
        "peak_speed": 0.0,
        "low_v_state_react": 0,   # ticks where state=REACT and speed near zero
        # Number of times the robot entered then left the 1-cell-radius
        # neighbourhood of the goal without reaching.  Captures the
        # 'orbits the goal' pattern.
        "goal_orbits": 0,
    }
    # State for idle-window detection: "idle" = cmd commanded but
    # measured velocity near zero, while in REACT.  Counts the longest
    # contiguous run of that condition.
    idle_window_s = [0.0, 0.0]    # current, max
    SLOW_THRESH = 0.02            # m/s
    # Goal-orbit tracking: each contiguous "inside 1 cell of goal" episode
    # without reaching is one orbit.
    GOAL_NEIGHBOURHOOD = 1.0 * maze.cell_size_m
    in_neighbourhood = [False]

    class _Done(Exception):
        def __init__(self, msg):
            self.msg = msg

    def on_step(i, reading, encoders, cmd, ctrl):
        # Closest approach to goal.
        d = math.hypot(world.x - goal_x, world.y - goal_y)
        if d < diag["goal_closest_m"]:
            diag["goal_closest_m"] = d
        # Peak speed (linear).
        v = 0.5 * (abs(encoders[0]) + abs(encoders[1]))
        if v > diag["peak_speed"]:
            diag["peak_speed"] = v
        # Idle window: REACT + cmd>thresh + meas<thresh.
        cmd_mag = max(abs(cmd.left), abs(cmd.right))
        meas_mag = max(abs(encoders[0]), abs(encoders[1]))
        if ctrl.state == ctrl._S_REACT and cmd_mag > 0.05 and meas_mag < SLOW_THRESH:
            diag["low_v_state_react"] += 1
            idle_window_s[0] += 1.0 / tun.loop_hz
            if idle_window_s[0] > idle_window_s[1]:
                idle_window_s[1] = idle_window_s[0]
        else:
            idle_window_s[0] = 0.0
        # Goal-orbit detection.
        if d < GOAL_NEIGHBOURHOOD:
            in_neighbourhood[0] = True
        elif in_neighbourhood[0] and d > GOAL_NEIGHBOURHOOD * 1.4:
            # Exited the neighbourhood without reaching the goal -> one orbit.
            diag["goal_orbits"] += 1
            in_neighbourhood[0] = False
        if world.reached_goal():
            raise _Done("GOAL")
        if world.collisions > collision_budget:
            raise _Done("COLL_BUDGET")

    max_steps = int(sim_time_cap_s * tun.loop_hz)
    try:
        run(sensors, drive, clock, tun, imu=imu,
            max_steps=max_steps, on_step=on_step, controller=controller)
    except _Done as d:
        diag["result"] = d.msg

    diag["sim_time"] = world.t
    diag["distance_m"] = world.distance_traveled
    diag["avg_speed"] = (world.distance_traveled / world.t) if world.t > 0 else 0.0
    diag["collisions"] = world.collisions
    diag["recoveries"] = controller.recovery_count
    diag["max_idle_window_s"] = idle_window_s[1]
    diag["reverses_per_min"] = (
        controller.recovery_count * 60.0 / world.t) if world.t > 0 else 0.0
    return diag


def _categorise(d, cell_size_m=0.18):
    """Classify a run by its failure pattern."""
    if d["result"] == "GOAL":
        if d["collisions"] > 0 or d["recoveries"] > 0:
            return "GOAL_dirty"  # reached but with incidents
        return "GOAL_clean"
    if d["result"] == "COLL_BUDGET":
        return "COLL_BUDGET"
    # TIMEOUT.  Sub-categorise by behaviour.
    near_goal = d["goal_closest_m"] < 1.5 * cell_size_m
    high_recov = d["reverses_per_min"] > 8.0
    long_idle = d["max_idle_window_s"] > 5.0
    orbits = d.get("goal_orbits", 0)
    if d["distance_m"] < 0.3:
        return "TIMEOUT_immediate_wedge"
    if long_idle and d["recoveries"] == 0:
        return "TIMEOUT_silent_freeze"
    if high_recov:
        return "TIMEOUT_recovery_loop"
    if orbits >= 2:
        # Repeatedly approached the goal and bounced off without reaching.
        return "TIMEOUT_orbits_goal"
    if near_goal:
        return "TIMEOUT_passed_goal"
    return "TIMEOUT_other"


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--quick", action="store_true",
                   help="small sweep (~24 runs) for fast iteration")
    p.add_argument("--sim-time", type=float, default=45.0,
                   help="per-run cap in simulated seconds (base)")
    p.add_argument("--sim-time-per-cell-side", type=float, default=0.0,
                   help="additional sim-time per maze cell-side; e.g. 4 -> "
                        "a 16x16 maze gets sim_time + 64 s")
    p.add_argument("--collision-budget", type=int, default=30,
                   help="abort run if collisions exceed this")
    p.add_argument("--csv", default="stress_results.csv")
    args = p.parse_args()

    if args.quick:
        seeds = _QUICK_SEEDS
        sizes = _QUICK_SIZES
        modes = _QUICK_MODES
        pose_sources = _QUICK_POSE
    else:
        seeds = _DEFAULT_SEEDS
        sizes = _DEFAULT_SIZES
        modes = _DEFAULT_MODES
        pose_sources = _DEFAULT_POSE

    total = len(seeds) * len(sizes) * len(modes) * len(pose_sources)
    print("Stress sweep: {} runs".format(total))
    print("  seeds={} sizes={} modes={} pose={}".format(
        seeds, sizes, modes, pose_sources))
    print()
    rows = []
    t0 = time.monotonic()
    n = 0
    for seed in seeds:
        for cols, rs in sizes:
            for mode in modes:
                for pose in pose_sources:
                    n += 1
                    sim_t = args.sim_time + args.sim_time_per_cell_side * (cols + rs)
                    d = _run_one(seed, cols, rs, mode, pose,
                                 sim_t, args.collision_budget)
                    d["category"] = _categorise(d)
                    rows.append(d)
                    if n % 10 == 0 or n == total:
                        elapsed = time.monotonic() - t0
                        eta = elapsed * (total - n) / max(n, 1)
                        sys.stdout.write(
                            "\r  {}/{}  elapsed={:.0f}s  eta={:.0f}s ".format(
                                n, total, elapsed, eta))
                        sys.stdout.flush()
    print()
    print()

    # Write CSV.
    if rows:
        fields = list(rows[0].keys())
        with open(args.csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=fields)
            w.writeheader()
            w.writerows(rows)
        print("CSV: {}".format(args.csv))

    # Summary tables.
    print()
    print("=== category counts ===")
    cats = {}
    for r in rows:
        cats.setdefault(r["category"], 0)
        cats[r["category"]] += 1
    for k in sorted(cats.keys()):
        print("  {:32s} {:3d}  ({:.0%})".format(
            k, cats[k], cats[k] / len(rows)))

    print()
    print("=== by mode (success rate) ===")
    by_mode = {}
    for r in rows:
        d = by_mode.setdefault(r["mode"], {"total": 0, "clean": 0, "dirty": 0,
                                           "timeout": 0, "coll": 0})
        d["total"] += 1
        if r["category"] == "GOAL_clean":
            d["clean"] += 1
        elif r["category"] == "GOAL_dirty":
            d["dirty"] += 1
        elif r["category"] == "COLL_BUDGET":
            d["coll"] += 1
        else:
            d["timeout"] += 1
    print("  {:12s} {:>6s} {:>8s} {:>8s} {:>10s} {:>8s}".format(
        "mode", "total", "clean%", "dirty%", "timeout%", "coll%"))
    for k in sorted(by_mode.keys()):
        d = by_mode[k]
        t = d["total"]
        print("  {:12s} {:>6d} {:>7.0%} {:>7.0%} {:>9.0%} {:>7.0%}".format(
            k, t, d["clean"]/t, d["dirty"]/t, d["timeout"]/t, d["coll"]/t))

    print()
    print("=== worst offenders (timeouts + collisions) ===")
    bad = [r for r in rows if r["result"] != "GOAL"]
    bad.sort(key=lambda r: (-r["recoveries"], -r["collisions"]))
    for r in bad[:12]:
        print(("  [{:>22s}] mode={:<10s} {}x{} seed={:>4d} pose={:<12s} "
               "dist={:>5.2f}m recov={:>3d} coll={:>3d} "
               "closest={:>5.2f}m idle={:>4.1f}s").format(
            r["category"], r["mode"], r["cols"], r["rows"], r["seed"],
            r["pose"], r["distance_m"], r["recoveries"], r["collisions"],
            r["goal_closest_m"], r["max_idle_window_s"]))


if __name__ == "__main__":
    main()
