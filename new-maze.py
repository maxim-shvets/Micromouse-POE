#!/usr/bin/env python3
"""Competition-spec Micromouse maze generator.

Generates a maze conforming to official Micromouse competition rules:
  - 16x16 grid default (configurable, must be even, min 4)
  - Corner start cell bounded on 3 sides; exit toward clockwise-next perimeter cell
  - 2x2 center goal with exactly one gateway (finish line)
  - Lattice-point rule: at least one wall at every interior grid corner
  - Full outer boundary enclosure
  - Multiple paths (non-perfect maze via loop injection)
  - Anti-wall-hugging: goal wall ring is topologically isolated from outer boundary;
    verified by simulating both left/right-hand wall-followers

Usage:
  python new-maze.py
  python new-maze.py --cols 8 --rows 8 --seed 42 --start-corner br --loops 0.2
  python new-maze.py --no-display
"""

import argparse
import random
import sys

# Direction constants: 0=N, 1=E, 2=S, 3=W
_DX = (0, 1, 0, -1)
_DY = (1, 0, -1, 0)
_OPP = (2, 3, 0, 1)
_DIR_NAMES = ('North', 'East', 'South', 'West')

# corner name -> (col_offset, row_offset, wall_to_add, exit_dir)
# col/row offsets: negative means from opposite edge (e.g. -1 = cols-1)
# wall_to_add: direction to force-close (making 3 sides enclosed)
# exit_dir: the open side (start line, toward clockwise-next perimeter cell)
_CORNERS = {
    'bl': (0,  0,  0, 1),   # bottom-left:  add N, exit E
    'br': (-1, 0,  3, 0),   # bottom-right: add W, exit N
    'tr': (-1, -1, 2, 3),   # top-right:    add S, exit W
    'tl': (0,  -1, 1, 2),   # top-left:     add E, exit S
}


class MazeGenerator:
    def __init__(self, cols, rows, seed=None, start_corner='bl',
                 loops=0.15, max_retries=100):
        if cols % 2 != 0 or rows % 2 != 0:
            raise ValueError("cols and rows must be even")
        if cols < 4 or rows < 4:
            raise ValueError("cols and rows must be >= 4")

        self.cols = cols
        self.rows = rows
        self.seed = seed
        self.start_corner = start_corner
        self.loops = loops

        # 2x2 goal block: bottom-left cell at (gc, gr)
        self.gc = cols // 2 - 1
        self.gr = rows // 2 - 1
        self.goal_cells = frozenset([
            (self.gc,   self.gr),
            (self.gc+1, self.gr),
            (self.gc,   self.gr+1),
            (self.gc+1, self.gr+1),
        ])

        # Start cell
        co, ro, self.wall_to_add, self.exit_dir = _CORNERS[start_corner]
        self.start_c = co % cols
        self.start_r = ro % rows

        # Generate with retry loop (re-seed on each retry)
        attempt_seed = seed
        for attempt in range(max_retries):
            rng = random.Random(attempt_seed)
            self.walls = [
                [[True, True, True, True] for _ in range(rows)]
                for _ in range(cols)
            ]
            self._carve(rng)
            self._add_loops(rng)
            self._setup_start()
            self.gateway = self._build_goal_island(rng)
            self._enforce_lattice_rule(rng)

            if not (self._wall_follower_reaches_goal('left') or
                    self._wall_follower_reaches_goal('right')):
                break

            attempt_seed = attempt if seed is None else seed + attempt + 1
        else:
            raise RuntimeError(
                f"Could not generate valid maze after {max_retries} attempts")

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _set_wall(self, c, r, d, present):
        """Set wall d of cell (c,r) and mirror in neighbour. OOB cells ignored."""
        if not (0 <= c < self.cols and 0 <= r < self.rows):
            return
        self.walls[c][r][d] = present
        nc, nr = c + _DX[d], r + _DY[d]
        if 0 <= nc < self.cols and 0 <= nr < self.rows:
            self.walls[nc][nr][_OPP[d]] = present

    def _carve(self, rng):
        """Recursive backtracker DFS from start cell, skipping goal cells."""
        sc, sr = self.start_c, self.start_r
        stack = [(sc, sr)]
        visited = {(sc, sr)}
        while stack:
            c, r = stack[-1]
            options = []
            for d in range(4):
                # Never carve through wall_to_add from start cell
                if (c, r) == (sc, sr) and d == self.wall_to_add:
                    continue
                nc, nr = c + _DX[d], r + _DY[d]
                if (0 <= nc < self.cols and 0 <= nr < self.rows
                        and (nc, nr) not in visited
                        and (nc, nr) not in self.goal_cells):
                    options.append(d)
            if not options:
                stack.pop()
                continue
            d = rng.choice(options)
            nc, nr = c + _DX[d], r + _DY[d]
            self.walls[c][r][d] = False
            self.walls[nc][nr][_OPP[d]] = False
            visited.add((nc, nr))
            stack.append((nc, nr))

    def _add_loops(self, rng):
        """Remove extra internal walls to create multiple paths."""
        count = max(1, int(self.loops * self.cols * self.rows))
        candidates = []
        for c in range(self.cols):
            for r in range(self.rows):
                for d in (0, 1):   # N and E only to avoid duplicates
                    nc, nr = c + _DX[d], r + _DY[d]
                    if not (0 <= nc < self.cols and 0 <= nr < self.rows):
                        continue
                    if d == 0 and r == self.rows - 1:
                        continue   # outer N boundary
                    if d == 1 and c == self.cols - 1:
                        continue   # outer E boundary
                    if (c, r) in self.goal_cells or (nc, nr) in self.goal_cells:
                        continue
                    if self.walls[c][r][d]:
                        candidates.append((c, r, d))
        rng.shuffle(candidates)
        for c, r, d in candidates[:count]:
            nc, nr = c + _DX[d], r + _DY[d]
            self.walls[c][r][d] = False
            self.walls[nc][nr][_OPP[d]] = False

    def _setup_start(self):
        """Enforce 3-wall enclosure at start corner and open exit direction."""
        sc, sr = self.start_c, self.start_r
        # Force wall_to_add closed (carver already skipped it, but be explicit)
        self._set_wall(sc, sr, self.wall_to_add, True)
        # Force exit open
        self._set_wall(sc, sr, self.exit_dir, False)

    def _build_goal_island(self, rng):
        """Create 2x2 goal room with one gateway; disconnect from outer walls."""
        gc, gr = self.gc, self.gr
        cols, rows = self.cols, self.rows

        # 1. Close all 8 outer faces of the 2x2 block
        outer_faces = [
            (gc,   gr,   2), (gc+1, gr,   2),   # S faces
            (gc,   gr,   3), (gc,   gr+1, 3),   # W faces
            (gc,   gr+1, 0), (gc+1, gr+1, 0),   # N faces
            (gc+1, gr,   1), (gc+1, gr+1, 1),   # E faces
        ]
        for fc, fr, fd in outer_faces:
            self._set_wall(fc, fr, fd, True)

        # 2. Open 4 internal walls (make the 2x2 a single open room)
        # E of (gc,gr) / W of (gc+1,gr)
        self.walls[gc][gr][1]     = False
        self.walls[gc+1][gr][3]   = False
        # N of (gc,gr) / S of (gc,gr+1)
        self.walls[gc][gr][0]     = False
        self.walls[gc][gr+1][2]   = False
        # N of (gc+1,gr) / S of (gc+1,gr+1)
        self.walls[gc+1][gr][0]   = False
        self.walls[gc+1][gr+1][2] = False
        # E of (gc,gr+1) / W of (gc+1,gr+1)
        self.walls[gc][gr+1][1]   = False
        self.walls[gc+1][gr+1][3] = False

        # 3. Remove external walls at each of the 8 boundary corners so
        #    the goal ring shares no lattice-point with the outer wall graph.
        #    Each call handles both sides of the shared wall automatically.

        # SW corner (gc, gr)
        if gr > 0:      self._set_wall(gc,    gr-1, 3, False)  # W of (gc, gr-1)
        if gc > 0:      self._set_wall(gc-1,  gr,   2, False)  # S of (gc-1, gr)

        # S-middle (gc+1, gr)
        if gr > 0:      self._set_wall(gc+1,  gr-1, 3, False)  # W of (gc+1, gr-1)

        # SE corner (gc+2, gr)
        if gc+2 < cols: self._set_wall(gc+2,  gr,   2, False)  # S of (gc+2, gr)
        if gr > 0:      self._set_wall(gc+1,  gr-1, 1, False)  # E of (gc+1, gr-1)

        # W-middle (gc, gr+1)
        if gc > 0:      self._set_wall(gc-1,  gr,   0, False)  # N of (gc-1, gr)
                                                                # (mirrors S of gc-1,gr+1)

        # E-middle (gc+2, gr+1)
        if gc+2 < cols: self._set_wall(gc+2,  gr,   0, False)  # N of (gc+2, gr)
                                                                # (mirrors S of gc+2,gr+1)

        # NW corner (gc, gr+2)
        if gc > 0:      self._set_wall(gc-1,  gr+1, 0, False)  # N of (gc-1, gr+1)
        if gr+2 < rows: self._set_wall(gc,    gr+2, 3, False)  # W of (gc, gr+2)

        # N-middle (gc+1, gr+2)
        if gr+2 < rows: self._set_wall(gc+1,  gr+2, 3, False)  # W of (gc+1, gr+2)

        # NE corner (gc+2, gr+2)
        if gc+2 < cols: self._set_wall(gc+2,  gr+1, 0, False)  # N of (gc+2, gr+1)
        if gr+2 < rows: self._set_wall(gc+1,  gr+2, 1, False)  # E of (gc+1, gr+2)

        # 4. Open one gateway (finish line)
        gw_idx = rng.randrange(len(outer_faces))
        gwc, gwr, gwd = outer_faces[gw_idx]
        self._set_wall(gwc, gwr, gwd, False)
        return outer_faces[gw_idx]

    def _enforce_lattice_rule(self, rng):
        """Ensure at least one wall touches every interior lattice point."""
        gc, gr = self.gc, self.gr
        center_corner = (gc+1, gr+1)  # interior of 2x2 room — exempt

        for cx in range(1, self.cols):
            for cy in range(1, self.rows):
                if (cx, cy) == center_corner:
                    continue
                # Four walls that meet at corner (cx, cy):
                #   N of (cx-1, cy-1)  →  walls[cx-1][cy-1][0]
                #   N of (cx,   cy-1)  →  walls[cx  ][cy-1][0]
                #   E of (cx-1, cy-1)  →  walls[cx-1][cy-1][1]
                #   E of (cx-1, cy  )  →  walls[cx-1][cy  ][1]
                has_wall = (
                    self.walls[cx-1][cy-1][0] or
                    self.walls[cx  ][cy-1][0] or
                    self.walls[cx-1][cy-1][1] or
                    self.walls[cx-1][cy  ][1]
                )
                if has_wall:
                    continue
                # Add back one wall — avoid walls adjacent to goal cells
                options = [
                    (cx-1, cy-1, 0), (cx,   cy-1, 0),
                    (cx-1, cy-1, 1), (cx-1, cy,   1),
                ]
                rng.shuffle(options)
                for wc, wr, wd in options:
                    if not (0 <= wc < self.cols and 0 <= wr < self.rows):
                        continue
                    nc, nr = wc + _DX[wd], wr + _DY[wd]
                    goal_adj = (
                        (wc, wr) in self.goal_cells or
                        (0 <= nc < self.cols and 0 <= nr < self.rows
                         and (nc, nr) in self.goal_cells)
                    )
                    if not goal_adj:
                        self._set_wall(wc, wr, wd, True)
                        break

    def _wall_follower_reaches_goal(self, hand='left'):
        """Simulate wall-follower from start. Returns True if it enters a goal cell."""
        c, r = self.start_c, self.start_r
        h = self.exit_dir
        visited = set()
        max_steps = self.cols * self.rows * 8

        for _ in range(max_steps):
            if (c, r) in self.goal_cells:
                return True
            state = (c, r, h)
            if state in visited:
                return False   # infinite loop — never reached goal
            visited.add(state)

            if hand == 'left':
                dirs = [(h-1) % 4, h, (h+1) % 4, (h+2) % 4]
            else:
                dirs = [(h+1) % 4, h, (h-1) % 4, (h+2) % 4]

            for d in dirs:
                nc, nr = c + _DX[d], r + _DY[d]
                if (0 <= nc < self.cols and 0 <= nr < self.rows
                        and not self.walls[c][r][d]):
                    c, r, h = nc, nr, d
                    break

        return False

    # ------------------------------------------------------------------
    # Validation helper
    # ------------------------------------------------------------------

    def count_wall_components(self):
        """Count connected components in the wall graph."""
        adj = {}
        for c in range(self.cols):
            for r in range(self.rows):
                segs = []
                w = self.walls[c][r]
                if w[0]: segs.append(((c, r+1), (c+1, r+1)))
                if w[1]: segs.append(((c+1, r), (c+1, r+1)))
                if w[2]: segs.append(((c,  r),  (c+1, r)))
                if w[3]: segs.append(((c,  r),  (c,   r+1)))
                for a, b in segs:
                    adj.setdefault(a, set()).add(b)
                    adj.setdefault(b, set()).add(a)
        seen = set()
        n = 0
        for start in list(adj):
            if start in seen:
                continue
            n += 1
            stack = [start]
            while stack:
                v = stack.pop()
                if v in seen:
                    continue
                seen.add(v)
                stack.extend(adj[v])
        return n

    # ------------------------------------------------------------------
    # Rendering
    # ------------------------------------------------------------------

    def render_ascii(self):
        """Return ASCII string of the maze (+ - | characters)."""
        cols, rows = self.cols, self.rows
        lines = []
        for row in range(rows - 1, -1, -1):
            # Top edge of this row
            top = ''
            for col in range(cols):
                top += '+'
                top += '--' if self.walls[col][row][0] else '  '
            top += '+'
            lines.append(top)
            # Cell row
            mid = ''
            for col in range(cols):
                mid += '|' if self.walls[col][row][3] else ' '
                if (col, row) in self.goal_cells:
                    mid += 'GG'
                elif (col, row) == (self.start_c, self.start_r):
                    mid += 'SS'
                else:
                    mid += '  '
            mid += '|' if self.walls[cols-1][row][1] else ' '
            lines.append(mid)
        # Bottom outer boundary
        bot = ''
        for col in range(cols):
            bot += '+'
            bot += '--' if self.walls[col][0][2] else '  '
        bot += '+'
        lines.append(bot)
        return '\n'.join(lines)

    def render_matplotlib(self):
        """Display maze in a matplotlib window."""
        import matplotlib.pyplot as plt
        import matplotlib.patches as patches

        fig, ax = plt.subplots(figsize=(10, 10))
        ax.set_aspect('equal')
        ax.set_xlim(-0.2, self.cols + 0.2)
        ax.set_ylim(-0.2, self.rows + 0.2)
        ax.axis('off')

        # Draw walls
        for c in range(self.cols):
            for r in range(self.rows):
                x0, y0 = float(c), float(r)
                x1, y1 = x0 + 1.0, y0 + 1.0
                w = self.walls[c][r]
                kw = dict(color='black', lw=1.5)
                if w[0]: ax.plot([x0, x1], [y1, y1], **kw)
                if w[1]: ax.plot([x1, x1], [y0, y1], **kw)
                if w[2]: ax.plot([x0, x1], [y0, y0], **kw)
                if w[3]: ax.plot([x0, x0], [y0, y1], **kw)

        # Shade goal cells
        for gc, gr in self.goal_cells:
            ax.add_patch(patches.Rectangle(
                (gc, gr), 1, 1, facecolor='tomato', alpha=0.35, zorder=2))

        # Shade start cell
        ax.add_patch(patches.Rectangle(
            (self.start_c, self.start_r), 1, 1,
            facecolor='limegreen', alpha=0.35, zorder=2))

        # Draw gateway (finish line) in gold
        gwc, gwr, gwd = self.gateway
        x0, y0 = float(gwc), float(gwr)
        x1, y1 = x0 + 1.0, y0 + 1.0
        gw_kw = dict(color='gold', lw=4, zorder=4, label='Finish line')
        if   gwd == 0: ax.plot([x0, x1], [y1, y1], **gw_kw)
        elif gwd == 1: ax.plot([x1, x1], [y0, y1], **gw_kw)
        elif gwd == 2: ax.plot([x0, x1], [y0, y0], **gw_kw)
        elif gwd == 3: ax.plot([x0, x0], [y0, y1], **gw_kw)

        # Labels
        ax.text(self.start_c + 0.5, self.start_r + 0.5, 'S',
                ha='center', va='center', fontsize=9, fontweight='bold',
                color='darkgreen', zorder=5)
        ax.text(self.gc + 1.0, self.gr + 1.0, 'G',
                ha='center', va='center', fontsize=11, fontweight='bold',
                color='darkred', zorder=5)

        ax.set_title(
            f'Micromouse Maze  {self.cols}×{self.rows}  '
            f'seed={self.seed}  corner={self.start_corner}  loops={self.loops}')
        ax.legend(loc='upper right', fontsize=9)
        plt.tight_layout()
        plt.show()


# ------------------------------------------------------------------
# CLI entry point
# ------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description='Generate a competition-spec Micromouse maze')
    parser.add_argument('--cols', type=int, default=16,
                        help='Grid columns (even, ≥4, default 16)')
    parser.add_argument('--rows', type=int, default=16,
                        help='Grid rows (even, ≥4, default 16)')
    parser.add_argument('--seed', type=int, default=None,
                        help='RNG seed for reproducibility')
    parser.add_argument('--start-corner',
                        choices=['bl', 'br', 'tl', 'tr'], default='bl',
                        help='Start corner (bl/br/tl/tr, default bl)')
    parser.add_argument('--loops', type=float, default=0.15,
                        help='Fraction of extra walls to remove (default 0.15)')
    parser.add_argument('--no-display', action='store_true',
                        help='Skip matplotlib window')
    args = parser.parse_args()

    if args.cols % 2 != 0 or args.rows % 2 != 0:
        print('Error: --cols and --rows must be even', file=sys.stderr)
        sys.exit(1)
    if args.cols < 4 or args.rows < 4:
        print('Error: --cols and --rows must be >= 4', file=sys.stderr)
        sys.exit(1)

    print(f'Generating {args.cols}×{args.rows} maze  '
          f'seed={args.seed}  corner={args.start_corner}  loops={args.loops} ...')

    maze = MazeGenerator(
        cols=args.cols,
        rows=args.rows,
        seed=args.seed,
        start_corner=args.start_corner,
        loops=args.loops,
    )

    print(maze.render_ascii())

    gwc, gwr, gwd = maze.gateway
    components = maze.count_wall_components()
    print(f'\nStart cell : ({maze.start_c}, {maze.start_r})  '
          f'exit={_DIR_NAMES[maze.exit_dir]}')
    print(f'Goal cells : {sorted(maze.goal_cells)}')
    print(f'Gateway    : {_DIR_NAMES[gwd]} face of cell ({gwc}, {gwr})')
    print(f'Wall components: {components}  (2 = outer boundary + goal island)')
    print('Anti-wall-hug  : VERIFIED — left/right-hand followers cannot reach goal')

    if not args.no_display:
        maze.render_matplotlib()


if __name__ == '__main__':
    main()
