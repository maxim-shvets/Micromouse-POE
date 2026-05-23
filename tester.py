"""Entry point: build a random maze, run the reactive algorithm in sim.

Usage:
  python3 tester.py                                # defaults (ASCII viz)
  python3 tester.py --viz matplotlib               # live graphical window
  python3 tester.py --viz matplotlib --viz-hold    # ...and keep it open
  python3 tester.py --viz matplotlib --planner flood_fill --mode race
                                                   # everything turned on
  python3 tester.py --cols 12 --rows 12            # bigger maze
  python3 tester.py --seed 7                       # reproducible
  python3 tester.py --no-render                    # headless (alias --viz none)
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
from sim.world import SimWorld, SimSensors, SimDrive, SimClock, SimIMU
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
    p.add_argument("--no-render", action="store_true",
                   help="shorthand for --viz none")
    p.add_argument("--render-hz", type=float, default=5.0,
                   help="ASCII redraw rate (only used when --viz ascii)")
    p.add_argument("--viz", choices=("ascii", "matplotlib", "none"),
                   default="ascii",
                   help="visualization mode (default: ascii). 'matplotlib' "
                        "opens a live window with maze, mouse, sensor rays, "
                        "and -- when --planner is set -- the planner overlay.")
    p.add_argument("--viz-hz", type=float, default=20.0,
                   help="matplotlib redraw rate in sim Hz (default 20)")
    p.add_argument("--viz-hold", action="store_true",
                   help="leave the matplotlib window open after the run "
                        "ends so you can inspect the final state")
    p.add_argument("--viz-no-rays", action="store_true",
                   help="hide the three ToF sensor rays")
    p.add_argument("--viz-no-planner-overlay", action="store_true",
                   help="hide the planner's known-map + cost overlay even "
                        "when --planner is enabled")
    p.add_argument("--viz-save-frames", default=None,
                   help="directory: save one PNG per matplotlib frame")
    p.add_argument("--collision-budget", type=int, default=5)
    p.add_argument("--mode",
                   choices=("cautious", "normal", "aggressive", "race"),
                   default=None,
                   help="load a built-in aggression preset from profiles/. "
                        "race targets 5 m/s peak. --profile and --tune layer "
                        "on top of --mode.")
    p.add_argument("--profile", default=None,
                   help="JSON file of tunables to load (after --mode)")
    p.add_argument("--tune", action="append", default=[],
                   metavar="KEY=VALUE",
                   help="override a tunable (repeatable; applied last)")
    p.add_argument("--telemetry", default=None,
                   help="JSONL output path for telemetry")
    p.add_argument("--no-advice", action="store_true",
                   help="skip post-run tuning advisor")
    p.add_argument("--metrics", action="store_true",
                   help="print raw run metrics alongside advice")
    p.add_argument("--list-tunables", action="store_true",
                   help="print all tunable keys + defaults and exit")
    p.add_argument("--maze-algo", choices=("new", "sim"), default="new",
                   help="maze generation algorithm: 'new' = competition-spec "
                        "(new-maze.py, 2x2 center, corner start, anti-wall-hug; "
                        "requires even cols/rows >= 4); "
                        "'sim' = original recursive backtracker (sim/maze.py, "
                        "any size in [3,20]). Default: new")
    p.add_argument("--planner", choices=("none", "flood_fill"), default="none",
                   help="enable the flood-fill planner above the reactive "
                        "controller (default: none -- legacy reactive-only)")
    p.add_argument("--pose-source",
                   choices=("ground_truth", "fused", "slam"),
                   default="ground_truth",
                   help="how the planner gets pose.  ground_truth: SimWorld "
                        "(default; sim only).  fused: encoder + gyro "
                        "complementary filter (FusedOdometry).  slam: fused "
                        "+ ToF scan-match.  NOTE: in the sim, dead reckoning "
                        "is essentially perfect, so 'slam' adds correction "
                        "noise the planner mis-attributes -- prefer 'fused' "
                        "for sim demos.  On real hardware where dead "
                        "reckoning drifts, 'slam' (used by hardware/"
                        "xiao_nrf52840.py::main) is the right choice.")
    return p.parse_args(argv)


def _list_tunables():
    print("Tunables (key = default):")
    for k in default_keys():
        v = default_value(k)
        print("  {:32s} {}".format(k, v))


def _build_tunables(args):
    """Layered tunable construction:  defaults -> mode preset -> profile -> CLI.

    `--mode` resolves to `profiles/<mode>.json`.  `--profile` then overlays
    a user-chosen file (so you can do e.g. `--mode aggressive --profile my.json`
    to start from aggressive and tweak a few keys).  `--tune` overrides come
    last, so a single CLI key wins over everything.
    """
    if args.mode is not None:
        # Resolve relative to the tester.py file, so it works from any cwd.
        import os
        here = os.path.dirname(os.path.abspath(__file__))
        mode_path = os.path.join(here, "profiles", args.mode + ".json")
        t = Tunables.from_json_file(mode_path)
    else:
        t = Tunables()
    if args.profile is not None:
        # Layer profile on top of mode by merging dicts.
        layer = Tunables.from_json_file(args.profile)
        merged = t.to_dict()
        for k, v in layer.diff().items():
            merged[k] = v
        t = Tunables.from_dict(merged)
    if args.tune:
        t = Tunables.from_overrides(args.tune, base=t)
    return t


def _build_controller(args, tun, world, maze):
    """Construct the controller, optionally wrapping the planner.

    `--pose-source` controls how the planner sees its pose:
      ground_truth: SimWorld's known (x, y, theta).  Sim only.  Default.
      fused:        encoder + gyro complementary filter (FusedOdometry).
      slam:         fused + ToF scan-match against the planner's known
                    map (ScanMatchSlam).  Matches the hardware path.
    """
    if args.planner == "none":
        return ReactiveController(tun), None
    if args.planner != "flood_fill":
        raise ValueError("Unknown planner: {}".format(args.planner))

    from planner import FloodFillPlanner
    plan = FloodFillPlanner(
        cols=maze.cols, rows=maze.rows,
        goal_cell=maze.goal_cell,
        cell_size_m=tun.planner_cell_size_m,
        turn_cost=tun.planner_turn_cost,
        reverse_cost=tun.planner_reverse_cost,
        unknown_cost=tun.planner_unknown_cost,
    )

    # Estimator owns the pose the planner sees.  Returns (controller, est).
    estimator = None
    observation_pose_provider = None
    if args.pose_source == "ground_truth":
        pose_provider = lambda: (world.x, world.y, world.theta)
    elif args.pose_source == "fused":
        from pose_fusion import FusedOdometry
        estimator = FusedOdometry(world.x, world.y, world.theta, tun)
        pose_provider = estimator.pose
    elif args.pose_source == "slam":
        from slam import ScanMatchSlam
        estimator = ScanMatchSlam(world.x, world.y, world.theta,
                                  plan.map, tun)
        pose_provider = estimator.pose
        # Wall-observation uses the smooth pre-correction pose so SLAM
        # corrections at cell boundaries don't poison the map (Bug #29).
        observation_pose_provider = estimator.dead_reckoning_pose
    else:
        raise ValueError("Unknown pose source: {}".format(args.pose_source))

    return (ReactiveController(tun, planner=plan,
                               pose_provider=pose_provider,
                               estimator=estimator,
                               observation_pose_provider=observation_pose_provider),
            estimator)


def _build_maze(args):
    """Construct the maze using the algorithm selected by --maze-algo."""
    if args.maze_algo == "sim":
        return Maze(args.cols, args.rows,
                    cell_size_m=args.cell_size, seed=args.seed)

    # "new" — competition-spec generator from sim/new-maze.py
    import importlib.util
    import os
    here = os.path.dirname(os.path.abspath(__file__))
    spec = importlib.util.spec_from_file_location(
        "new_maze", os.path.join(here, "sim", "new-maze.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

    if args.cols % 2 != 0 or args.rows % 2 != 0:
        raise SystemExit(
            "Error: --maze-algo new requires even --cols and --rows")
    if args.cols < 4 or args.rows < 4:
        raise SystemExit(
            "Error: --maze-algo new requires --cols and --rows >= 4")

    return mod.MazeGenerator(
        cols=args.cols,
        rows=args.rows,
        cell_size_m=args.cell_size,
        seed=args.seed,
    )


def main(argv=None):
    args = _parse_args(argv or sys.argv[1:])

    if args.list_tunables:
        _list_tunables()
        return 0

    tun = _build_tunables(args)
    if tun.diff():
        sys.stderr.write("tunables overrides: {}\n".format(tun.diff()))

    maze = _build_maze(args)
    # The planner's cell-size tunable must match the maze actually being
    # driven.  Honour an explicit override (--tune planner_cell_size_m=...)
    # if the user set one; otherwise inherit from --cell-size.
    if "planner_cell_size_m" not in tun.diff():
        tun.planner_cell_size_m = args.cell_size
    world = SimWorld(maze, tun)
    sensors = SimSensors(world)
    drive = SimDrive(world)
    clock = SimClock(world)
    imu = SimIMU(world)

    controller, estimator = _build_controller(args, tun, world, maze)

    max_steps = int(args.sim_time * tun.loop_hz)

    # --no-render is the legacy headless flag; collapse to --viz none.
    viz_mode = "none" if args.no_render else args.viz

    recorder = TelemetryRecorder(tun, log_path=args.telemetry, world=world) \
        if tun.telem_enabled else None

    # --- visualizer setup --------------------------------------------------
    visualizer = None
    render_step = lambda *_: None
    if viz_mode == "matplotlib":
        from sim.visualizer import MatplotlibVisualizer
        visualizer = MatplotlibVisualizer(
            world, tun,
            viz_hz=args.viz_hz,
            show_rays=not args.viz_no_rays,
            show_planner=not args.viz_no_planner_overlay,
            save_frames_to=args.viz_save_frames,
        )
        render_step = visualizer
    elif viz_mode == "ascii":
        render_period_steps = max(1, int(tun.loop_hz / args.render_hz))
        last_print_step = [-render_period_steps]

        def render_step(i, reading, encoders, cmd, controller):
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
                world.distance_traveled, world.collisions,
                controller.recovery_count,
            )
            sys.stdout.write("\x1b[H\x1b[2J")
            sys.stdout.write(frame)
            sys.stdout.write("\n")
            sys.stdout.write(info)
            sys.stdout.write("\n")
            sys.stdout.flush()

    # --- compose on_step: telemetry, then render, then termination check.
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
        run(sensors, drive, clock, tun, imu=imu,
            max_steps=max_steps, on_step=on_step,
            controller=controller)
    except _Done as d:
        exit_code = d.code
        exit_msg = d.msg
    finally:
        if recorder is not None:
            recorder.close()

    if viz_mode == "ascii":
        sys.stdout.write("\x1b[H\x1b[2J")
        sys.stdout.write(render_mod.render(world))
        sys.stdout.write("\n")
    elif visualizer is not None:
        # One last redraw so the final frame is up to date, then either
        # hold the window open or close it.
        # We synthesize a final on_step-shaped call via the controller's
        # cached state and the world's current sensor reading.
        try:
            final_reading = sensors.read()
            final_encoders = drive.read_encoders()
            from interfaces import WheelSpeeds
            visualizer(0, final_reading, final_encoders,
                       WheelSpeeds(0.0, 0.0), controller)
        except Exception:
            pass
        visualizer.close(hold=args.viz_hold)

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
