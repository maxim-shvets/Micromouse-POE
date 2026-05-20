"""Entry point: build a random maze, run the reactive algorithm in sim.

Usage:
  python3 tester.py                                # defaults
  python3 tester.py --cols 12 --rows 12            # bigger maze
  python3 tester.py --seed 7                       # reproducible
  python3 tester.py --no-render                    # headless
  python3 tester.py --sim-time 30                  # cap simulated time
  python3 tester.py --profile profiles/demo.json   # load tunables
  python3 tester.py --tune cruise_speed_mps=0.4 \
                    --tune front_stop_m=0.10       # one-off overrides
  python3 tester.py --telemetry out/run.jsonl      # save log
  python3 tester.py --no-advice                    # skip tuning advisor
  python3 tester.py --list-tunables                # show all knobs

Exit codes:
  0  reached goal
  1  ran out of sim time
  2  collisions exceeded threshold
"""

import argparse
import sys
import time

from algorithm import run, ReactiveController
from sim.maze import Maze
from sim.world import SimWorld, SimSensors, SimDrive, SimClock
from sim import render as render_mod
from telemetry import TelemetryRecorder
from tunables import Tunables, default_keys, default_value
import tuning


def _dim_arg(s):
    """argparse type: integer in [3, 20] for maze dimensions."""
    v = int(s)
    if not (3 <= v <= 20):
        raise argparse.ArgumentTypeError(
            "maze dimension must be in [3, 20]; got {}".format(v))
    return v


def _parse_args(argv):
    p = argparse.ArgumentParser(description="Micromouse simulator.")
    p.add_argument("--cols", type=_dim_arg, default=8,
                   help="maze width in cells, integer in [3, 20] (default 8)")
    p.add_argument("--rows", type=_dim_arg, default=8,
                   help="maze height in cells, integer in [3, 20] (default 8)")
    p.add_argument("--cell-size", type=float, default=0.18,
                   help="meters per cell (default 0.18 -- standard micromouse "
                        "competition spec)")
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--sim-time", type=float, default=60.0,
                   help="max simulated seconds")
    p.add_argument("--no-render", action="store_true")
    p.add_argument("--render-hz", type=float, default=5.0)
    p.add_argument("--collision-budget", type=int, default=5)
    p.add_argument("--profile", default=None,
                   help="JSON file of tunables to load")
    p.add_argument("--tune", action="append", default=[],
                   metavar="KEY=VALUE",
                   help="override a tunable (repeatable)")
    p.add_argument("--telemetry", default=None,
                   help="JSONL output path for telemetry")
    p.add_argument("--no-advice", action="store_true",
                   help="skip post-run tuning advisor")
    p.add_argument("--metrics", action="store_true",
                   help="print raw run metrics alongside advice")
    p.add_argument("--list-tunables", action="store_true",
                   help="print all tunable keys + defaults and exit")
    p.add_argument("--planner", choices=("none", "flood_fill"), default="none",
                   help="enable the flood-fill planner above the reactive "
                        "controller (default: none -- legacy reactive-only)")
    return p.parse_args(argv)


def _list_tunables():
    print("Tunables (key = default):")
    for k in default_keys():
        v = default_value(k)
        print("  {:32s} {}".format(k, v))


def _build_tunables(args):
    if args.profile is not None:
        t = Tunables.from_json_file(args.profile)
    else:
        t = Tunables()
    if args.tune:
        t = Tunables.from_overrides(args.tune, base=t)
    return t


def _build_controller(args, tun, world, maze):
    """Construct the controller, optionally wrapping the planner.

    In sim we hand the planner ground-truth pose from `SimWorld`.  On real
    hardware (`hardware/rp2040.py`) you'd wire `pose_provider` to an
    `EncoderOdometry` instance integrating wheel encoders from the known
    start cell.
    """
    if args.planner == "none":
        return ReactiveController(tun)
    if args.planner == "flood_fill":
        from planner import FloodFillPlanner
        plan = FloodFillPlanner(
            cols=maze.cols, rows=maze.rows,
            goal_cell=maze.goal_cell,
            cell_size_m=tun.planner_cell_size_m,
        )
        pose_provider = lambda: (world.x, world.y, world.theta)
        return ReactiveController(tun, planner=plan, pose_provider=pose_provider)
    raise ValueError("Unknown planner: {}".format(args.planner))


def main(argv=None):
    args = _parse_args(argv or sys.argv[1:])

    if args.list_tunables:
        _list_tunables()
        return 0

    tun = _build_tunables(args)
    if tun.diff():
        sys.stderr.write("tunables overrides: {}\n".format(tun.diff()))

    maze = Maze(args.cols, args.rows, cell_size_m=args.cell_size, seed=args.seed)
    # The planner's cell-size tunable must match the maze actually being
    # driven.  Honour an explicit override (--tune planner_cell_size_m=...)
    # if the user set one; otherwise inherit from --cell-size.
    if "planner_cell_size_m" not in tun.diff():
        tun.planner_cell_size_m = args.cell_size
    world = SimWorld(maze, tun)
    sensors = SimSensors(world)
    drive = SimDrive(world)
    clock = SimClock(world)

    controller = _build_controller(args, tun, world, maze)

    max_steps = int(args.sim_time * tun.loop_hz)
    render_period_steps = max(1, int(tun.loop_hz / args.render_hz))

    recorder = TelemetryRecorder(tun, log_path=args.telemetry, world=world) \
        if tun.telem_enabled else None

    last_print_step = [-render_period_steps]

    def render_step(i, reading, encoders, cmd, controller):
        if args.no_render:
            return
        if i - last_print_step[0] < render_period_steps:
            return
        last_print_step[0] = i
        frame = render_mod.render(world)
        info = (
            "t={:5.2f}s  step={:5d}  pos=({:.2f},{:.2f}) th={:+.2f}rad  "
            "state={}\n"
            "front={:.2f}  left={:.2f}  right={:.2f}\n"
            "cmd L/R={:+.2f}/{:+.2f}  enc L/R={:+.2f}/{:+.2f}  "
            "dist={:.2f}m  collisions={}  recoveries={}"
        ).format(
            world.t, i, world.x, world.y, world.theta,
            controller.STATE_NAMES[controller.state],
            reading.front, reading.left, reading.right,
            cmd.left, cmd.right, encoders[0], encoders[1],
            world.distance_traveled, world.collisions, controller.recovery_count,
        )
        sys.stdout.write("\x1b[H\x1b[2J")
        sys.stdout.write(frame)
        sys.stdout.write("\n")
        sys.stdout.write(info)
        sys.stdout.write("\n")
        sys.stdout.flush()

    # Compose on_step: telemetry first, then render, then termination check.
    class _Done(Exception):
        def __init__(self, code, msg):
            self.code = code
            self.msg = msg

    def on_step(i, reading, encoders, cmd, controller):
        if recorder is not None:
            recorder(i, reading, encoders, cmd, controller)
        render_step(i, reading, encoders, cmd, controller)
        if world.reached_goal():
            raise _Done(0, "GOAL reached")
        if world.collisions > args.collision_budget:
            raise _Done(2, "Collision budget exceeded")

    exit_code = 1
    exit_msg = "ran out of sim time"
    try:
        run(sensors, drive, clock, tun, max_steps=max_steps, on_step=on_step,
            controller=controller)
    except _Done as d:
        exit_code = d.code
        exit_msg = d.msg
    finally:
        if recorder is not None:
            recorder.close()

    if not args.no_render:
        sys.stdout.write("\x1b[H\x1b[2J")
        sys.stdout.write(render_mod.render(world))
        sys.stdout.write("\n")

    print("---- run summary ----")
    print("result        : {}".format(exit_msg))
    print("sim time      : {:.2f} s".format(world.t))
    print("distance      : {:.2f} m".format(world.distance_traveled))
    print("collisions    : {}".format(world.collisions))
    print("avg speed     : {:.2f} m/s".format(
        world.distance_traveled / world.t if world.t > 0 else 0.0))
    print("maze          : {}x{} cells @ {:.2f} m, seed={}".format(
        args.cols, args.rows, args.cell_size, args.seed))

    if recorder is not None and not args.no_advice:
        print()
        print(tuning.report_from_recorder(recorder, verbose=args.metrics))

    return exit_code


if __name__ == "__main__":
    sys.exit(main())
