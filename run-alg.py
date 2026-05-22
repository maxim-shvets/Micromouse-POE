#!/usr/bin/env python3
"""Full micromouse race algorithm: explore -> return -> race.

Usage:
  python3 run-alg.py                              # auto exploration, lift return
  python3 run-alg.py --viz matplotlib             # live animated window
  python3 run-alg.py --explore-runs 3             # fixed 3 mapping runs
  python3 run-alg.py --return-mode drive          # drive back, exploring on the way
  python3 run-alg.py --no-diagonals               # cardinal-only race path
  python3 run-alg.py --race-speed 1.5             # race at 1.5 m/s
  python3 run-alg.py --seed 42 --viz none         # headless

Exit codes:
  0  race run completed (goal reached)
  1  time budget exhausted before race run finished
  2  collision budget exceeded
"""

import argparse
import importlib.util
import math
import os
import sys

from algorithm import ReactiveController, run as _alg_run
from interfaces import WheelSpeeds
from planner import (
    FloodFillPlanner,
    theta_from_heading, heading_from_theta, wrap_pi,
    N, E, S, W, _DC, _DR,
)
from sim.world import SimWorld, SimSensors, SimDrive, SimClock, SimIMU
from tunables import Tunables

_DIAG_ANGLE = {
    (N, E): math.pi / 4,
    (E, N): math.pi / 4,
    (N, W): 3 * math.pi / 4,
    (W, N): 3 * math.pi / 4,
    (S, W): -3 * math.pi / 4,
    (W, S): -3 * math.pi / 4,
    (S, E): -math.pi / 4,
    (E, S): -math.pi / 4,
}

COMPETITION_TIME_S = 600.0


# ---------------------------------------------------------------------------
# Maze loading
# ---------------------------------------------------------------------------

def _load_maze(cols, rows, seed, cell_size, start_corner='bl', loops=0.15):
    here = os.path.dirname(os.path.abspath(__file__))
    spec = importlib.util.spec_from_file_location(
        "new_maze", os.path.join(here, "sim", "new-maze.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.MazeGenerator(
        cols=cols, rows=rows, cell_size_m=cell_size,
        seed=seed, start_corner=start_corner, loops=loops,
    )


# ---------------------------------------------------------------------------
# Tunables construction
# ---------------------------------------------------------------------------

def _build_tunables(args):
    if args.mode is not None:
        here = os.path.dirname(os.path.abspath(__file__))
        mode_path = os.path.join(here, "profiles", args.mode + ".json")
        t = Tunables.from_json_file(mode_path)
    else:
        t = Tunables()
    if args.profile is not None:
        layer = Tunables.from_json_file(args.profile)
        merged = t.to_dict()
        for k, v in layer.diff().items():
            merged[k] = v
        t = Tunables.from_dict(merged)
    if args.tune:
        t = Tunables.from_overrides(args.tune, base=t)
    return t


# ---------------------------------------------------------------------------
# Map coverage
# ---------------------------------------------------------------------------

def _map_coverage(known_map):
    total = 0
    known = 0
    for c in range(known_map.cols):
        for r in range(known_map.rows):
            for d in range(4):
                total += 1
                if known_map.walls[c][r][d] is not None:
                    known += 1
    return known / total if total > 0 else 0.0


# ---------------------------------------------------------------------------
# Optimal path extraction (BFS, None walls = open)
# ---------------------------------------------------------------------------

def _extract_path(known_map, start_cell, goal_cell):
    cols, rows = known_map.cols, known_map.rows
    sc, sr = start_cell
    gc, gr = goal_cell
    dist = [[-1] * rows for _ in range(cols)]
    pred = [[None] * rows for _ in range(cols)]
    dist[sc][sr] = 0
    queue = [(sc, sr)]
    head = 0
    while head < len(queue):
        c, r = queue[head]
        head += 1
        if (c, r) == (gc, gr):
            break
        for d in (N, E, S, W):
            if known_map.walls[c][r][d] is True:
                continue
            nc, nr = c + _DC[d], r + _DR[d]
            if not (0 <= nc < cols and 0 <= nr < rows):
                continue
            if dist[nc][nr] == -1:
                dist[nc][nr] = dist[c][r] + 1
                pred[nc][nr] = (c, r)
                queue.append((nc, nr))
    if dist[gc][gr] == -1:
        return [start_cell, goal_cell]
    path = []
    cur = (gc, gr)
    while cur is not None:
        path.append(cur)
        c, r = cur
        cur = pred[c][r]
    path.reverse()
    return path


# ---------------------------------------------------------------------------
# Diagonal optimisation
# ---------------------------------------------------------------------------

def _corner_passable(walls, cols, rows, c, r, d1, d2):
    nc1, nr1 = c + _DC[d1], r + _DR[d1]
    nc2, nr2 = nc1 + _DC[d2], nr1 + _DR[d2]
    if not (0 <= nc2 < cols and 0 <= nr2 < rows):
        return False
    if walls[c][r][d1] is True:
        return False
    if walls[nc1][nr1][d2] is True:
        return False
    if walls[c][r][d2] is True:
        return False
    nc1b, nr1b = c + _DC[d2], r + _DR[d2]
    if not (0 <= nc1b < cols and 0 <= nr1b < rows):
        return False
    if walls[nc1b][nr1b][d1] is True:
        return False
    return True


def _cell_dir(a, b):
    dc, dr = b[0] - a[0], b[1] - a[1]
    for d in (N, E, S, W):
        if (_DC[d], _DR[d]) == (dc, dr):
            return d
    return None


def _build_waypoints(path, maze, use_diagonals):
    s = maze.cell_size_m
    walls = maze.walls
    cols, rows = maze.cols, maze.rows

    def cc(c, r):
        return ((c + 0.5) * s, (r + 0.5) * s)

    def corner(c, r, d1, d2):
        cx = c + max(_DC[d1], 0) + max(_DC[d2], 0)
        cy = r + max(_DR[d1], 0) + max(_DR[d2], 0)
        return (cx * s, cy * s)

    wps = []
    i = 0
    while i < len(path):
        c, r = path[i]
        if use_diagonals and i + 2 < len(path):
            nc, nr = path[i + 1]
            nc2, nr2 = path[i + 2]
            d1 = _cell_dir((c, r), (nc, nr))
            d2 = _cell_dir((nc, nr), (nc2, nr2))
            if (d1 is not None and d2 is not None and d1 != d2
                    and (d1 + 2) % 4 != d2
                    and (d1, d2) in _DIAG_ANGLE
                    and _corner_passable(walls, cols, rows, c, r, d1, d2)):
                wx, wy = corner(c, r, d1, d2)
                wps.append((wx, wy, _DIAG_ANGLE[(d1, d2)]))
                i += 2
                continue
        if i + 1 < len(path):
            nc, nr = path[i + 1]
            wx, wy = cc(nc, nr)
            d = _cell_dir((c, r), (nc, nr))
            theta = theta_from_heading(d) if d is not None else 0.0
            wps.append((wx, wy, theta))
        i += 1
    return wps


# ---------------------------------------------------------------------------
# Proxy controller for the race phase (used by the visualizer)
# ---------------------------------------------------------------------------

class _RaceProxy:
    """Minimal duck-type of ReactiveController for the visualizer."""
    STATE_NAMES = {0: "RACE"}
    state = 0
    recovery_count = 0
    _desired_heading = None

    def __init__(self, planner):
        self.planner = planner


# ---------------------------------------------------------------------------
# Race driver (drives SimWorld directly — no reactive layer)
# ---------------------------------------------------------------------------

class RaceDriver:
    def __init__(self, waypoints, race_speed_mps, tunables):
        self.waypoints = waypoints
        self.race_speed = race_speed_mps
        self.tun = tunables
        self.dt = 1.0 / tunables.loop_hz
        self.pivot_kp = 3.0
        self.arrive_radius = tunables.planner_cell_size_m * 0.35
        self.align_tol = 0.08

    def run(self, world, sensors, drive, clock,
            on_step=None, visualizer=None, proxy_ctrl=None, step_offset=0):
        dt = self.dt
        tun = self.tun
        step_i = step_offset

        for wx, wy, target_theta in self.waypoints:
            # Pivot to align
            for _ in range(int(3.0 / dt)):
                err = wrap_pi(target_theta - world.theta)
                if abs(err) < self.align_tol:
                    break
                s = tun.turn_speed_mps
                cmd = WheelSpeeds(-s, +s) if err > 0 else WheelSpeeds(+s, -s)
                drive.set_wheel_speeds(cmd)
                clock.sleep(dt)
                if on_step is not None:
                    on_step(step_i)
                if visualizer is not None:
                    reading = sensors.read()
                    encoders = drive.read_encoders()
                    visualizer(step_i, reading, encoders, cmd, proxy_ctrl)
                step_i += 1

            # Drive to waypoint
            for _ in range(int(10.0 / dt)):
                dx = wx - world.x
                dy = wy - world.y
                dist = math.hypot(dx, dy)
                if dist < self.arrive_radius:
                    break
                desired_theta = math.atan2(dy, dx)
                heading_err = wrap_pi(desired_theta - world.theta)
                bias = max(-0.6, min(0.6, self.pivot_kp * heading_err))
                v = self.race_speed
                cmd = WheelSpeeds(v * (1.0 - bias), v * (1.0 + bias))
                drive.set_wheel_speeds(cmd)
                clock.sleep(dt)
                if on_step is not None:
                    on_step(step_i)
                if visualizer is not None:
                    reading = sensors.read()
                    encoders = drive.read_encoders()
                    visualizer(step_i, reading, encoders, cmd, proxy_ctrl)
                step_i += 1

        drive.stop()
        return step_i


# ---------------------------------------------------------------------------
# Phase helpers
# ---------------------------------------------------------------------------

class _Done(Exception):
    def __init__(self, code, msg):
        self.code = code
        self.msg = msg


def _run_to_goal(world, sensors, drive, clock, imu, tun, controller,
                 collision_budget, time_limit_s, step_offset=0, visualizer=None):
    max_steps = int((time_limit_s - world.t) * tun.loop_hz)
    if max_steps <= 0:
        raise _Done(1, "time budget exhausted before exploration run")
    step_i = [step_offset]

    def on_step(i, reading, encoders, cmd, ctrl):
        step_i[0] = step_offset + i
        if visualizer is not None:
            visualizer(step_i[0], reading, encoders, cmd, ctrl)
        if world.reached_goal():
            raise _Done(0, "goal")
        if world.collisions > collision_budget:
            raise _Done(2, "collision budget exceeded")

    try:
        _alg_run(sensors, drive, clock, tun, imu=imu,
                 max_steps=max_steps, on_step=on_step,
                 controller=controller)
    except _Done as d:
        if d.code == 0:
            return step_i[0]
        raise
    raise _Done(1, "time budget exhausted mid-exploration")


def _run_to_cell(world, sensors, drive, clock, imu, tun, controller,
                 target_cell, maze, collision_budget, time_limit_s,
                 step_offset=0, visualizer=None):
    s = maze.cell_size_m
    tx, ty = maze.cell_center(*target_cell)
    arrive_r = s * 0.4
    max_steps = int((time_limit_s - world.t) * tun.loop_hz)
    if max_steps <= 0:
        raise _Done(1, "time budget exhausted before return run")
    step_i = [step_offset]

    def on_step(i, reading, encoders, cmd, ctrl):
        step_i[0] = step_offset + i
        if visualizer is not None:
            visualizer(step_i[0], reading, encoders, cmd, ctrl)
        if math.hypot(world.x - tx, world.y - ty) < arrive_r:
            raise _Done(0, "start")
        if world.collisions > collision_budget:
            raise _Done(2, "collision budget exceeded")

    try:
        _alg_run(sensors, drive, clock, tun, imu=imu,
                 max_steps=max_steps, on_step=on_step,
                 controller=controller)
    except _Done as d:
        if d.code == 0:
            return step_i[0]
        raise
    raise _Done(1, "time budget exhausted mid-return")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _parse_args(argv):
    p = argparse.ArgumentParser(description="Micromouse full race algorithm.")
    p.add_argument("--cols", type=int, default=16)
    p.add_argument("--rows", type=int, default=16)
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--cell-size", type=float, default=0.18)
    p.add_argument("--start-corner",
                   choices=("bl", "br", "tl", "tr"), default="bl")
    p.add_argument("--loops", type=float, default=0.15)

    p.add_argument("--explore-runs", type=int, default=None,
                   help="Fixed number of explore runs (default: auto)")
    p.add_argument("--return-mode", choices=("lift", "drive"), default="drive",
                   help="How to return to start: 'drive' navigates back through "
                        "unexplored areas (default); 'lift' teleports instantly")
    p.add_argument("--no-diagonals", action="store_true",
                   help="Disable diagonal shortcuts in race run")
    p.add_argument("--race-speed", type=float, default=None,
                   help="Race run speed in m/s (default: max_speed_mps * 1.5)")
    p.add_argument("--coverage-threshold", type=float, default=0.85,
                   help="Wall coverage fraction to trigger auto-stop (default 0.85)")
    p.add_argument("--race-reserve-s", type=float, default=90.0,
                   help="Seconds to reserve for race run(s) (default 90)")
    p.add_argument("--time-limit-s", type=float, default=COMPETITION_TIME_S,
                   help="Total simulated time budget in seconds (default 600)")
    p.add_argument("--collision-budget", type=int, default=10)

    p.add_argument("--mode", choices=("cautious", "normal", "aggressive", "race"),
                   default=None)
    p.add_argument("--profile", default=None)
    p.add_argument("--tune", action="append", default=[], metavar="KEY=VALUE")

    p.add_argument("--viz", choices=("ascii", "matplotlib", "none"), default="ascii")
    p.add_argument("--viz-hz", type=float, default=20.0,
                   help="Matplotlib redraw rate in sim Hz (default 20)")
    p.add_argument("--viz-hold", action="store_true",
                   help="Keep matplotlib window open after run ends")
    p.add_argument("--render-hz", type=float, default=2.0,
                   help="ASCII redraw rate (default 2)")

    return p.parse_args(argv)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def _print_phase(msg):
    sys.stdout.write("\n=== {} ===\n".format(msg))
    sys.stdout.flush()


def main(argv=None):
    args = _parse_args(argv or sys.argv[1:])

    if args.cols % 2 != 0 or args.rows % 2 != 0:
        sys.exit("Error: --cols and --rows must be even for new-maze")
    if args.cols < 4 or args.rows < 4:
        sys.exit("Error: --cols and --rows must be >= 4")

    tun = _build_tunables(args)
    tun.planner_cell_size_m = args.cell_size
    tun.sim_max_substep_s = min(tun.sim_max_substep_s, 0.002)

    race_speed = args.race_speed or min(tun.max_speed_mps * 1.5, 5.0)

    _print_phase("Generating maze {}x{} seed={}".format(
        args.cols, args.rows, args.seed))
    maze = _load_maze(args.cols, args.rows, args.seed,
                      args.cell_size, args.start_corner, args.loops)
    if args.viz != "matplotlib":
        print(maze.render_ascii())

    world = SimWorld(maze, tun)
    sensors = SimSensors(world)
    drive = SimDrive(world)
    clock = SimClock(world)
    imu = SimIMU(world)

    # Build visualizer
    visualizer = None
    if args.viz == "matplotlib":
        from sim.visualizer import MatplotlibVisualizer
        visualizer = MatplotlibVisualizer(
            world, tun,
            viz_hz=args.viz_hz,
            show_rays=True,
            show_planner=True,
        )

    shared_plan = FloodFillPlanner(
        cols=maze.cols, rows=maze.rows,
        goal_cell=maze.goal_cell,
        cell_size_m=args.cell_size,
        turn_cost=tun.planner_turn_cost,
        reverse_cost=tun.planner_reverse_cost,
        unknown_cost=tun.planner_unknown_cost,
    )
    pose_provider = lambda: (world.x, world.y, world.theta)

    time_limit = args.time_limit_s
    explore_run_count = 0
    prev_path_cost = None
    prev_coverage = None
    step_i = 0

    # -----------------------------------------------------------------------
    # Exploration + return loop
    # -----------------------------------------------------------------------
    while True:
        remaining = time_limit - world.t
        if remaining < args.race_reserve_s:
            _print_phase("Time budget low ({:.1f}s left) — switching to race".format(remaining))
            break

        _print_phase("Explore run {} (t={:.1f}s)".format(
            explore_run_count + 1, world.t))

        ctrl = ReactiveController(tun, planner=shared_plan,
                                  pose_provider=pose_provider)
        try:
            step_i = _run_to_goal(world, sensors, drive, clock, imu, tun, ctrl,
                                   args.collision_budget, time_limit, step_i,
                                   visualizer=visualizer)
        except _Done as d:
            if d.code != 0:
                print("Aborted during exploration: {}".format(d.msg))
                if visualizer is not None:
                    visualizer.close(hold=args.viz_hold)
                return d.code
        explore_run_count += 1

        coverage = _map_coverage(shared_plan.map)
        path_cost = shared_plan.cost(*maze.start_cell)
        cost_stable = (prev_path_cost is not None and path_cost == prev_path_cost)
        coverage_stable = (prev_coverage is not None and coverage == prev_coverage)
        prev_path_cost = path_cost
        prev_coverage = coverage
        print("  coverage={:.1%}  path_cost={:.1f}  cost_stable={}  cov_stable={}".format(
            coverage, path_cost, cost_stable, coverage_stable))

        if args.explore_runs is not None and explore_run_count >= args.explore_runs:
            _print_phase("Reached fixed explore count ({})".format(args.explore_runs))
            break

        if args.explore_runs is None:
            if explore_run_count >= 2:
                high_coverage_done = (coverage >= args.coverage_threshold and cost_stable)
                stalled = (cost_stable and coverage_stable)
                if high_coverage_done or stalled:
                    reason = "stalled (no new walls found)" if stalled else \
                             "coverage={:.1%} stable".format(coverage)
                    _print_phase("Auto-stop: {}".format(reason))
                    break
            if explore_run_count >= 8:
                _print_phase("Auto-stop: hard cap of 8 runs")
                break

        # ---- Return to start ----
        remaining = time_limit - world.t
        if remaining < args.race_reserve_s:
            _print_phase("Time budget low — skipping return, going to race")
            break

        if args.return_mode == "lift":
            _print_phase("Return: lift (teleport)")
            cx, cy = maze.cell_center(*maze.start_cell)
            world.x = cx
            world.y = cy
            world.theta = theta_from_heading(maze.exit_dir)

        else:
            _print_phase("Return: drive (t={:.1f}s)".format(world.t))
            return_plan = FloodFillPlanner(
                cols=maze.cols, rows=maze.rows,
                goal_cell=maze.start_cell,
                cell_size_m=args.cell_size,
                turn_cost=tun.planner_turn_cost,
                reverse_cost=tun.planner_reverse_cost,
                unknown_cost=-1.0,  # strongly prefer unexplored corridors over already-mapped outgoing path
            )
            return_plan.map = shared_plan.map
            return_ctrl = ReactiveController(tun, planner=return_plan,
                                             pose_provider=pose_provider)
            try:
                step_i = _run_to_cell(world, sensors, drive, clock, imu, tun,
                                       return_ctrl, maze.start_cell, maze,
                                       args.collision_budget, time_limit, step_i,
                                       visualizer=visualizer)
            except _Done as d:
                if d.code != 0:
                    print("Aborted during return: {}".format(d.msg))
                    if visualizer is not None:
                        visualizer.close(hold=args.viz_hold)
                    return d.code

            new_coverage = _map_coverage(shared_plan.map)
            new_cost = shared_plan.cost(*maze.start_cell)
            print("  post-return coverage={:.1%}  path_cost={:.1f}".format(
                new_coverage, new_cost))

            if (args.explore_runs is None
                    and new_coverage > coverage + 0.02
                    and explore_run_count < 8):
                prev_path_cost = new_cost
                continue

    # -----------------------------------------------------------------------
    # Race run
    # -----------------------------------------------------------------------
    _print_phase("Race run (speed={:.2f} m/s, diagonals={})".format(
        race_speed, not args.no_diagonals))

    cx, cy = maze.cell_center(*maze.start_cell)
    world.x = cx
    world.y = cy
    world.theta = theta_from_heading(maze.exit_dir)

    path = _extract_path(shared_plan.map, maze.start_cell, maze.goal_cell)
    wps = _build_waypoints(path, maze, use_diagonals=not args.no_diagonals)
    print("  path: {} cells  ->  {} waypoints".format(len(path), len(wps)))

    race_start_t = world.t
    race_start_dist = world.distance_traveled

    proxy = _RaceProxy(shared_plan)
    race_driver = RaceDriver(wps, race_speed, tun)

    reached = False

    def _race_check(si):
        nonlocal reached
        if world.reached_goal():
            reached = True
            raise _Done(0, "race goal reached")
        if world.collisions > args.collision_budget:
            raise _Done(2, "collision budget exceeded during race")

    try:
        race_driver.run(world, sensors, drive, clock,
                        on_step=_race_check,
                        visualizer=visualizer,
                        proxy_ctrl=proxy,
                        step_offset=step_i)
        if not reached and world.reached_goal():
            reached = True
    except _Done as d:
        if d.code == 0:
            reached = True
        elif d.code == 2:
            _print_summary(world, maze, race_start_t, race_start_dist, explore_run_count)
            if visualizer is not None:
                visualizer.close(hold=args.viz_hold)
            return 2

    _print_summary(world, maze, race_start_t, race_start_dist, explore_run_count)

    if visualizer is not None:
        visualizer.close(hold=args.viz_hold)

    return 0 if reached else 1


def _print_summary(world, maze, race_start_t, race_start_dist, explore_run_count):
    race_time = world.t - race_start_t
    race_dist = world.distance_traveled - race_start_dist
    print()
    print("==== run summary ====")
    print("explore runs     : {}".format(explore_run_count))
    print("explore time     : {:.2f} s".format(race_start_t))
    print("race time        : {:.2f} s".format(race_time))
    print("race distance    : {:.2f} m".format(race_dist))
    print("race avg speed   : {:.2f} m/s".format(
        race_dist / race_time if race_time > 0 else 0.0))
    print("total time       : {:.2f} s".format(world.t))
    print("total collisions : {}".format(world.collisions))
    print("maze             : {}x{} @ {:.2f}m/cell  seed={}".format(
        maze.cols, maze.rows, maze.cell_size_m, maze.seed))


if __name__ == "__main__":
    sys.exit(main())
