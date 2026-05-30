"""Demo: long diagonal runs end-to-end (generator + planner).

Generates a diagonal-rich maze, pre-maps the planner from ground truth,
walks the optimal path detecting long diagonal runs, and renders the
maze with the planner's drivable weaving trajectories overlaid.

This shows the whole long-diagonal pipeline:
  - sim/new-maze.py  injects + detects the staircase corridors
  - planner.diagonal_run  chains consecutive corner-cuts into a run and
    returns the weaving waypoints (the offset 45-degree line that clears
    the interior posts)

Usage:
  python3 diagonal_demo.py --seed 7 --diagonal-runs 5
  python3 diagonal_demo.py --seed 7 --save out.png    # headless
"""

import argparse
import importlib.util
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from planner import FloodFillPlanner, N, _DC, _DR


def _load_maze_mod():
    here = os.path.dirname(os.path.abspath(__file__))
    spec = importlib.util.spec_from_file_location(
        "new_maze", os.path.join(here, "sim", "new-maze.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def planner_runs_on_path(plan):
    """Walk the optimal path from (0,0), collecting planner diagonal runs."""
    cell, heading = (0, 0), N
    runs = []
    visited = set()
    for _ in range(400):
        if cell == plan.goal_cell or cell in visited:
            break
        visited.add(cell)
        run = plan.diagonal_run(cell, heading)
        if run:
            runs.append(run)
            cell, heading = run['exit_cell'], run['exit_heading']
        else:
            h = plan.desired_heading(cell, heading)
            cell = (cell[0] + _DC[h], cell[1] + _DR[h])
            heading = h
    return runs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cols", type=int, default=16)
    ap.add_argument("--rows", type=int, default=16)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--diagonal-runs", type=int, default=5)
    ap.add_argument("--save", default=None, help="PNG path (headless)")
    args = ap.parse_args()

    mod = _load_maze_mod()
    maze = mod.MazeGenerator(cols=args.cols, rows=args.rows, seed=args.seed,
                             diagonal_runs=args.diagonal_runs)
    plan = FloodFillPlanner(cols=args.cols, rows=args.rows,
                            goal_cell=maze.goal_cell, cell_size_m=1.0,
                            use_diagonals=True, turn_cost=0.3,
                            reverse_cost=2.0, unknown_cost=0.2)
    for c in range(args.cols):
        for r in range(args.rows):
            for d in range(4):
                plan.map.set_wall(c, r, d, bool(maze.walls[c][r][d]))
    plan.replan()

    gen_runs = maze.find_diagonal_runs(min_cells=4, thin_only=True)
    runs = planner_runs_on_path(plan)
    print("maze diagonal runs (>=4 cells): {}".format(len(gen_runs)))
    print("planner runs on optimal path  : {}".format(len(runs)))
    for run in runs:
        print("  {} cells  theta={:+.2f}  {} -> {}".format(
            len(run['cells']), run['theta'], run['cells'][0], run['exit_cell']))

    import matplotlib
    if args.save:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(9, 9))
    ax.set_aspect('equal')
    ax.axis('off')
    for c in range(args.cols):
        for r in range(args.rows):
            w = maze.walls[c][r]
            x0, y0 = float(c), float(r)
            x1, y1 = x0 + 1, y0 + 1
            kw = dict(color='black', lw=1.3)
            if w[0]: ax.plot([x0, x1], [y1, y1], **kw)
            if w[1]: ax.plot([x1, x1], [y0, y1], **kw)
            if w[2]: ax.plot([x0, x1], [y0, y0], **kw)
            if w[3]: ax.plot([x0, x0], [y0, y1], **kw)
    lab = False
    for run in runs:
        wp = run['waypoints']
        ax.plot([p[0] for p in wp], [p[1] for p in wp], color='crimson',
                lw=2.5, marker='o', ms=3, zorder=6,
                label='planner weave' if not lab else None)
        lab = True
    sc, sr = maze.start_cell
    ax.text(sc + 0.5, sr + 0.5, 'S', ha='center', va='center',
            fontweight='bold', color='green')
    ax.set_title("Long diagonal runs: planner weaving trajectories  "
                 "seed={}  runs={}".format(args.seed, len(runs)))
    if lab:
        ax.legend(loc='upper right')
    plt.tight_layout()
    if args.save:
        plt.savefig(args.save, dpi=80)
        print("saved", args.save)
    else:
        plt.show()


if __name__ == "__main__":
    main()
