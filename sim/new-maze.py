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
    def __init__(self, cols, rows, cell_size_m=0.18, seed=None,
                 start_corner='bl', loops=0.15, max_retries=100,
                 diagonal_runs=0, diagonal_min_len=8, diagonal_max_len=14):
        if cols % 2 != 0 or rows % 2 != 0:
            raise ValueError("cols and rows must be even")
        if cols < 4 or rows < 4:
            raise ValueError("cols and rows must be >= 4")

        self.cols = cols
        self.rows = rows
        self.cell_size_m = float(cell_size_m)
        self.width_m = cols * self.cell_size_m
        self.height_m = rows * self.cell_size_m
        self.seed = seed
        self.start_corner = start_corner
        self.loops = loops
        # Long-diagonal injection.  When > 0, carve that many staircase
        # corridors (alternating-turn zigzags) so the maze contains
        # genuine micromouse-style diagonal runs.  See _inject_diagonal_runs.
        self.diagonal_runs = diagonal_runs
        self.diagonal_min_len = diagonal_min_len
        self.diagonal_max_len = diagonal_max_len
        self._diagonal_seeds = []   # staircases carved this attempt

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
        self.start_cell = (self.start_c, self.start_r)

        # Generate with retry loop (re-seed on each retry)
        attempt_seed = seed
        for attempt in range(max_retries):
            rng = random.Random(attempt_seed)
            self.walls = [
                [[True, True, True, True] for _ in range(rows)]
                for _ in range(cols)
            ]
            self._diagonal_seeds = []
            self._carve(rng)
            self._add_loops(rng)
            self._inject_diagonal_runs(rng)
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

        # goal_cell (singular) = the gateway cell — where the robot enters
        gwc, gwr, _ = self.gateway
        self.goal_cell = (gwc, gwr)

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

    # ------------------------------------------------------------------
    # Long diagonal runs (micromouse-style staircase corridors)
    # ------------------------------------------------------------------

    def _inject_diagonal_runs(self, rng):
        """Carve `self.diagonal_runs` staircase corridors into the maze.

        A staircase is an alternating two-cardinal zigzag (e.g. E, N, E,
        N, ...) whose net travel is a 45-degree diagonal.  Carving the
        zigzag open creates a thin corridor whose shared-edge midpoints
        lie on a single straight 45-degree line -- the trajectory a
        micromouse drives when "going diagonal".  Detected afterwards by
        `find_diagonal_runs`.

        The corridor is left THIN (we do not open the 2x2 blocks), so the
        interior posts keep their walls and the lattice rule stays
        satisfiable.  The mouse clears those posts by weaving along the
        edge-midpoint line, half a cell off the cell centers.
        """
        if self.diagonal_runs <= 0:
            return
        # (d1, d2) alternating-cardinal pairs -> net diagonal direction.
        diag_pairs = [
            (1, 0),   # E, N -> NE
            (1, 2),   # E, S -> SE
            (3, 0),   # W, N -> NW
            (3, 2),   # W, S -> SW
        ]
        placed = 0
        attempts = 0
        budget = max(40, self.diagonal_runs * 40)
        while placed < self.diagonal_runs and attempts < budget:
            attempts += 1
            length = rng.randint(self.diagonal_min_len, self.diagonal_max_len)
            d1, d2 = rng.choice(diag_pairs)
            start_with = rng.choice((d1, d2))
            c = rng.randrange(self.cols)
            r = rng.randrange(self.rows)
            if self._carve_staircase(c, r, d1, d2, start_with, length):
                placed += 1

    def _carve_staircase(self, c, r, d1, d2, first, length):
        """Try to carve a `length`-step alternating staircase from (c, r).

        Alternates `first` then the other of {d1, d2}.  Aborts (carving
        nothing) if the path would leave the grid or touch the start /
        goal cells.  Returns True iff the staircase was carved.
        """
        cells = [(c, r)]
        moves = []
        cur_c, cur_r = c, r
        d = first
        other = d2 if first == d1 else d1
        for _ in range(length):
            if (cur_c, cur_r) in self.goal_cells or (cur_c, cur_r) == self.start_cell:
                return False
            nc, nr = cur_c + _DX[d], cur_r + _DY[d]
            if not (0 <= nc < self.cols and 0 <= nr < self.rows):
                return False
            if (nc, nr) in self.goal_cells or (nc, nr) == self.start_cell:
                return False
            moves.append((cur_c, cur_r, d))
            cells.append((nc, nr))
            cur_c, cur_r = nc, nr
            d = other if d == first else first
        for mc, mr, md in moves:
            self.walls[mc][mr][md] = False
            nc, nr = mc + _DX[md], mr + _DY[md]
            self.walls[nc][nr][_OPP[md]] = False
        self._diagonal_seeds.append(list(cells))
        return True

    def _diagonal_routes(self, c, r, dcx, dcy):
        """Return (route_a_open, route_b_open) for a diagonal move
        (c,r) -> (c+dcx, r+dcy).

        route_a = horizontal-first L (via the cell east/west of (c,r)),
        route_b = vertical-first L (via the cell north/south of (c,r)).
        Out-of-bounds target -> (False, False).
        """
        nc, nr = c + dcx, r + dcy
        if not (0 <= nc < self.cols and 0 <= nr < self.rows):
            return (False, False)
        d_h = 1 if dcx > 0 else 3   # E or W
        d_v = 0 if dcy > 0 else 2   # N or S
        bx, by = c + dcx, r
        route_a = (not self.walls[c][r][d_h]) and (not self.walls[bx][by][d_v])
        ax, ay = c, r + dcy
        route_b = (not self.walls[c][r][d_v]) and (not self.walls[ax][ay][d_h])
        return (route_a, route_b)

    def _diagonal_step_ok(self, c, r, dcx, dcy, thin_only=True):
        """Whether a diagonal move (c,r) -> (c+dcx, r+dcy) counts.

        `thin_only=True` (default): exactly ONE L-route open -- a forced
        staircase corridor, the genuine micromouse diagonal.  Excludes
        wide-open 2x2 areas (both routes open), which aren't distinctive
        diagonals.

        `thin_only=False`: at least one L-route open -- "can the mouse cut
        this corner at all" (used by the planner's cut logic).
        """
        ra, rb = self._diagonal_routes(c, r, dcx, dcy)
        if thin_only:
            return ra != rb       # exactly one -> thin corridor
        return ra or rb

    def find_diagonal_runs(self, min_cells=3, thin_only=True):
        """Find maximal straight diagonal runs of >= `min_cells` cells.

        Returns a list of dicts: {'cells': [(c,r), ...], 'dir': (dcx, dcy)}.
        Only the two dcx>0 families (NE, SE) are scanned, so each straight
        diagonal is reported once (a NE run and its SW reverse are the
        same physical line).

        `thin_only=True` reports only forced staircase corridors (the
        prominent micromouse diagonals); False also reports diagonals
        through open areas.
        """
        runs = []
        for dcx, dcy in ((1, 1), (1, -1)):
            for c in range(self.cols):
                for r in range(self.rows):
                    pc, pr = c - dcx, r - dcy
                    prev_ok = (0 <= pc < self.cols and 0 <= pr < self.rows
                               and self._diagonal_step_ok(pc, pr, dcx, dcy,
                                                          thin_only))
                    if prev_ok:
                        continue
                    run = [(c, r)]
                    cc, cr = c, r
                    while self._diagonal_step_ok(cc, cr, dcx, dcy, thin_only):
                        cc, cr = cc + dcx, cr + dcy
                        run.append((cc, cr))
                    if len(run) >= min_cells:
                        runs.append({'cells': run, 'dir': (dcx, dcy)})
        return runs

    def diagonal_trajectory_cells(self, run):
        """Edge-midpoint polyline (in cell units) for a diagonal run.

        For a straight diagonal run the staircase's shared-edge midpoints
        are colinear at 45 deg -- this returns them as the drivable
        weaving line (offset half a cell from the cell centers, so it
        clears the interior posts).
        """
        cells = run['cells']
        pts = []
        # Enter from the first cell's center.
        c0, r0 = cells[0]
        pts.append((c0 + 0.5, r0 + 0.5))
        for i in range(len(cells) - 1):
            (ca, ra), (cb, rb) = cells[i], cells[i + 1]
            # Midpoint of the two diagonal-neighbour cell centers == the
            # shared post; nudge to the open side's edge midpoint.
            # Edge-midpoint = average of the two centers (lies on the
            # weaving line by construction for a thin staircase).
            mx = (ca + 0.5 + cb + 0.5) / 2.0
            my = (ra + 0.5 + rb + 0.5) / 2.0
            pts.append((mx, my))
        cn, rn = cells[-1]
        pts.append((cn + 0.5, rn + 0.5))
        return pts

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
    # SimWorld compatibility (duck-type matches sim/maze.py Maze interface)
    # ------------------------------------------------------------------

    def cell_center(self, c, r):
        """World-frame center of cell (c, r) in meters."""
        s = self.cell_size_m
        return (c * s + s / 2.0, r * s + s / 2.0)

    def wall_segments(self):
        """List of (x1, y1, x2, y2) wall segments in meters, de-duplicated."""
        s = self.cell_size_m
        segs = []
        seen = set()

        def add(x1, y1, x2, y2):
            key = (round(x1, 6), round(y1, 6), round(x2, 6), round(y2, 6))
            rkey = (round(x2, 6), round(y2, 6), round(x1, 6), round(y1, 6))
            if key not in seen and rkey not in seen:
                seen.add(key)
                segs.append((x1, y1, x2, y2))

        for c in range(self.cols):
            for r in range(self.rows):
                x0, y0 = c * s, r * s
                x1, y1 = x0 + s, y0 + s
                w = self.walls[c][r]
                if w[0]: add(x0, y1, x1, y1)
                if w[1]: add(x1, y0, x1, y1)
                if w[2]: add(x0, y0, x1, y0)
                if w[3]: add(x0, y0, x0, y1)
        return segs

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

    def render_matplotlib(self, diagonal_runs=None):
        """Display maze in a matplotlib window.

        If `diagonal_runs` is provided (from `find_diagonal_runs`), draws
        the weaving 45-degree trajectory of each run as a coloured line.
        """
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

        # Diagonal run overlay -- weaving 45-degree trajectories.
        if diagonal_runs:
            labelled = False
            for run in diagonal_runs:
                pts = self.diagonal_trajectory_cells(run)
                xs = [p[0] for p in pts]
                ys = [p[1] for p in pts]
                ax.plot(xs, ys, color='royalblue', lw=2.5, alpha=0.8,
                        zorder=6, solid_capstyle='round',
                        label='Diagonal run' if not labelled else None)
                # Mark the run's cell centers.
                for (cc, cr) in run['cells']:
                    ax.add_patch(patches.Rectangle(
                        (cc, cr), 1, 1, facecolor='royalblue',
                        alpha=0.12, zorder=1))
                labelled = True

        ax.set_title(
            f'Micromouse Maze  {self.cols}×{self.rows}  '
            f'seed={self.seed}  corner={self.start_corner}  loops={self.loops}  '
            f'diagonals={len(diagonal_runs) if diagonal_runs else 0}')
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
    parser.add_argument('--diagonal-runs', type=int, default=0,
                        help='Inject N long staircase corridors that create '
                             'micromouse-style diagonal runs (default 0)')
    parser.add_argument('--diagonal-min-len', type=int, default=8,
                        help='Min staircase length in steps; a length-N '
                             'staircase yields ~N/2+1 diagonal cells '
                             '(default 8)')
    parser.add_argument('--diagonal-max-len', type=int, default=14,
                        help='Max staircase length in steps (default 14)')
    parser.add_argument('--diagonal-show', type=int, default=8,
                        help='How many of the longest diagonal runs to draw '
                             '(default 8; keeps the overlay readable)')
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
        diagonal_runs=args.diagonal_runs,
        diagonal_min_len=args.diagonal_min_len,
        diagonal_max_len=args.diagonal_max_len,
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

    all_runs = maze.find_diagonal_runs(min_cells=3, thin_only=True)
    all_runs.sort(key=lambda x: -len(x['cells']))
    long_runs = [r for r in all_runs if len(r['cells']) >= 4]
    short_count = len(all_runs) - len(long_runs)
    print(f'Diagonal runs  : {len(long_runs)} long (>= 4 cells), '
          f'{short_count} short (3 cells)  requested={args.diagonal_runs}')
    _dname = {(1, 1): 'NE', (1, -1): 'SE'}
    for run in long_runs[:args.diagonal_show]:
        dcx, dcy = run['dir']
        print(f'  {len(run["cells"]):2d} cells {_dname[(dcx, dcy)]}  '
              f'{run["cells"][0]} -> {run["cells"][-1]}')

    if not args.no_display:
        maze.render_matplotlib(diagonal_runs=long_runs[:args.diagonal_show])


if __name__ == '__main__':
    main()
