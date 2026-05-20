"""Random maze generation.

Recursive-backtracker carve over a `cols x rows` cell grid, then a small
"goal island" post-pass that disconnects the goal cell's perimeter walls
from every other wall in the maze.  This is the topological condition
that makes left/right-hand-rule wall-following unable to reach the goal:
since the goal's walls share no corner with the outer-boundary wall
graph, a follower starting on the outer wall traces back to itself
without ever touching the goal.

Conventions:
  - Cells indexed (col, row), (0, 0) at bottom-left.
  - Each cell is `cell_size_m` meters on a side.
  - `walls[c][r]` is [N, E, S, W], each True iff that wall exists.
  - Outer-boundary walls are never knocked down.

Default geometry: standard micromouse cells (0.18 m).  Custom dims allowed
in [3, 20] x [3, 20].
"""

import random


# Direction vectors: 0=N, 1=E, 2=S, 3=W
_DX = (0, 1, 0, -1)
_DY = (1, 0, -1, 0)
_OPPOSITE = (2, 3, 0, 1)

_DIM_MIN = 3
_DIM_MAX = 20


class Maze(object):
    """Rectangular grid maze with a center-cell wall-island goal."""

    def __init__(self, cols, rows, cell_size_m=0.18, seed=None,
                 start_cell=None, goal_cell=None):
        if not (_DIM_MIN <= cols <= _DIM_MAX and _DIM_MIN <= rows <= _DIM_MAX):
            raise ValueError(
                "cols and rows must each be in [{}, {}]; got cols={}, rows={}".format(
                    _DIM_MIN, _DIM_MAX, cols, rows))
        self.cols = cols
        self.rows = rows
        self.cell_size_m = float(cell_size_m)
        self.width_m = cols * self.cell_size_m
        self.height_m = rows * self.cell_size_m

        # walls[c][r] = [N, E, S, W] -- True = wall present.
        self.walls = [[[True, True, True, True] for _ in range(rows)]
                      for _ in range(cols)]

        rng = random.Random(seed)
        self._carve(rng)

        # Start: bottom-left corner.  Goal: nearest-cell to geometric centre.
        self.start_cell = start_cell if start_cell is not None else (0, 0)
        self.goal_cell = goal_cell if goal_cell is not None \
            else (cols // 2, rows // 2)

        if self.goal_cell == self.start_cell:
            # Degenerate (only possible if the caller overrode both); nothing
            # to islandify.  Bail rather than corrupt the maze.
            return

        self._build_goal_island(rng)

    # ---- recursive backtracker carve --------------------------------------

    def _carve(self, rng):
        stack = [(0, 0)]
        visited = {(0, 0)}
        while stack:
            c, r = stack[-1]
            options = []
            for d in range(4):
                nc, nr = c + _DX[d], r + _DY[d]
                if 0 <= nc < self.cols and 0 <= nr < self.rows:
                    if (nc, nr) not in visited:
                        options.append(d)
            if not options:
                stack.pop()
                continue
            d = rng.choice(options)
            nc, nr = c + _DX[d], r + _DY[d]
            self.walls[c][r][d] = False
            self.walls[nc][nr][_OPPOSITE[d]] = False
            visited.add((nc, nr))
            stack.append((nc, nr))

    # ---- low-level wall edit ---------------------------------------------

    def _set_wall(self, c, r, d, present):
        """Set wall `d` of cell (c, r), mirroring the change in the neighbour.

        Out-of-bounds cells are silently ignored -- callers pass them when
        the goal is at the maze boundary (which our default center placement
        avoids, but the helper stays robust).
        """
        if not (0 <= c < self.cols and 0 <= r < self.rows):
            return
        self.walls[c][r][d] = present
        nc, nr = c + _DX[d], r + _DY[d]
        if 0 <= nc < self.cols and 0 <= nr < self.rows:
            self.walls[nc][nr][_OPPOSITE[d]] = present

    # ---- goal island ------------------------------------------------------

    def _build_goal_island(self, rng):
        """Disconnect the goal's perimeter walls from the rest of the maze.

        Step 1: ensure the goal has exactly one open side (the entry).
        Step 2: at each of the 4 grid corners of the goal cell, knock down
                any external wall that shares that corner.  After this, the
                goal's perimeter walls form their own connected component:
                no shared endpoint with any external wall = no wall-graph
                edge to the rest of the maze.

        Side-effect: the knock-downs create a "ring" of passages one cell
        thick around the goal, which is exactly the topology of a
        competition micromouse centre.
        """
        gc, gr = self.goal_cell

        # Step 1: pick an entry direction.  After recursive backtracker every
        # cell has >= 1 open neighbour; we keep one of those open and close
        # the rest.  If somehow none is open (defensive), force-open one.
        open_dirs = [d for d in range(4) if not self.walls[gc][gr][d]]
        if open_dirs:
            entry = rng.choice(open_dirs)
        else:
            entry = rng.randrange(4)
            self._set_wall(gc, gr, entry, False)
        for d in range(4):
            if d != entry:
                self._set_wall(gc, gr, d, True)

        # Step 2: disconnect at each of the four corners.
        #
        # At grid corner (cx, cy), four wall segments can meet -- LEFT,
        # RIGHT, DOWN, UP.  Two of these are goal-perimeter walls (internal).
        # The other two are external and must be removed for the island to
        # be topologically disjoint.
        #
        # We enumerate the two externals per corner directly rather than
        # deriving them from the corner; this is the least error-prone shape
        # the code can take.

        cols, rows = self.cols, self.rows

        # SW corner = (gc, gr).
        # External walls meeting here: LEFT  = S of cell (gc-1, gr),
        #                              DOWN  = W of cell (gc, gr-1).
        if gc > 0:
            self._set_wall(gc - 1, gr, 2, False)
        if gr > 0:
            self._set_wall(gc, gr - 1, 3, False)

        # SE corner = (gc+1, gr).
        # External walls: RIGHT = S of (gc+1, gr),
        #                 DOWN  = W of (gc+1, gr-1).
        if gc + 1 < cols:
            self._set_wall(gc + 1, gr, 2, False)
            if gr > 0:
                self._set_wall(gc + 1, gr - 1, 3, False)

        # NW corner = (gc, gr+1).
        # External walls: LEFT = N of (gc-1, gr),
        #                 UP   = W of (gc, gr+1).
        if gc > 0:
            self._set_wall(gc - 1, gr, 0, False)
        if gr + 1 < rows:
            self._set_wall(gc, gr + 1, 3, False)

        # NE corner = (gc+1, gr+1).
        # External walls: RIGHT = N of (gc+1, gr),
        #                 UP    = W of (gc+1, gr+1).
        if gc + 1 < cols:
            self._set_wall(gc + 1, gr, 0, False)
            if gr + 1 < rows:
                self._set_wall(gc + 1, gr + 1, 3, False)

    # ---- query helpers ----------------------------------------------------

    def cell_center(self, c, r):
        """World-frame center of cell (c, r), in meters."""
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
            if key in seen or rkey in seen:
                return
            seen.add(key)
            segs.append((x1, y1, x2, y2))

        for c in range(self.cols):
            for r in range(self.rows):
                x0 = c * s
                y0 = r * s
                x1 = x0 + s
                y1 = y0 + s
                if self.walls[c][r][0]:
                    add(x0, y1, x1, y1)
                if self.walls[c][r][1]:
                    add(x1, y0, x1, y1)
                if self.walls[c][r][2]:
                    add(x0, y0, x1, y0)
                if self.walls[c][r][3]:
                    add(x0, y0, x0, y1)
        return segs

    # ---- topology helpers (used by the verifier; cheap to keep here) -----

    def wall_corners(self):
        """Set of (cx, cy) integer grid corners that have at least one wall.

        Used for connectivity analysis -- two walls are "connected" iff
        they share a corner.
        """
        seen = set()
        for c in range(self.cols):
            for r in range(self.rows):
                w = self.walls[c][r]
                if w[0]:  # N: (c, r+1)-(c+1, r+1)
                    seen.add((c, r + 1)); seen.add((c + 1, r + 1))
                if w[1]:  # E: (c+1, r)-(c+1, r+1)
                    seen.add((c + 1, r)); seen.add((c + 1, r + 1))
                if w[2]:  # S: (c, r)-(c+1, r)
                    seen.add((c, r)); seen.add((c + 1, r))
                if w[3]:  # W: (c, r)-(c, r+1)
                    seen.add((c, r)); seen.add((c, r + 1))
        return seen

    def wall_edges(self):
        """Set of frozenset({(cx1,cy1),(cx2,cy2)}) -- each present wall."""
        edges = set()
        for c in range(self.cols):
            for r in range(self.rows):
                w = self.walls[c][r]
                if w[0]:
                    edges.add(frozenset(((c, r + 1), (c + 1, r + 1))))
                if w[1]:
                    edges.add(frozenset(((c + 1, r), (c + 1, r + 1))))
                if w[2]:
                    edges.add(frozenset(((c, r), (c + 1, r))))
                if w[3]:
                    edges.add(frozenset(((c, r), (c, r + 1))))
        return edges

    def count_wall_components(self):
        """Number of connected components in the wall graph.

        Vertices = grid corners with at least one incident wall.  Edges =
        wall segments.  Two walls are in the same component iff they share
        a corner (transitively).
        """
        adj = {}
        for edge in self.wall_edges():
            a, b = tuple(edge)
            adj.setdefault(a, []).append(b)
            adj.setdefault(b, []).append(a)

        seen = set()
        components = 0
        for start in adj:
            if start in seen:
                continue
            components += 1
            stack = [start]
            while stack:
                v = stack.pop()
                if v in seen:
                    continue
                seen.add(v)
                stack.extend(adj[v])
        return components
