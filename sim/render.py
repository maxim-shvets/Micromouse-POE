"""Tiny ASCII renderer for the maze + mouse.

Each maze cell is drawn as a 2x2 character block, with one extra row/col for
the closing border:

    +-+-+-+
    | . .|
    +-+ + +
    |.   |
    +-+-+-+

  '+' corners, '-' / '|' walls, ' ' open, 'M' mouse (with > < ^ v heading
  arrow when it lines up), '*' breadcrumb trail.

Renders to a string; the tester prints it on a loop.
"""

import math


_HEADING_GLYPHS = (">", "^", "<", "v")  # E, N, W, S


def _heading_glyph(theta):
    # Snap to nearest of 4 cardinals for the arrow.
    a = theta % (2.0 * math.pi)
    idx = int(round(a / (math.pi / 2.0))) % 4
    return _HEADING_GLYPHS[idx]


def render(world, trail=True):
    maze = world.maze
    cols = maze.cols
    rows = maze.rows
    s = maze.cell_size_m

    # Grid of characters: (2*rows + 1) tall x (2*cols + 1) wide.
    grid = [[" "] * (2 * cols + 1) for _ in range(2 * rows + 1)]

    # Corners.
    for cy in range(rows + 1):
        for cx in range(cols + 1):
            grid[2 * cy][2 * cx] = "+"

    # Walls.
    for c in range(cols):
        for r in range(rows):
            walls = maze.walls[c][r]
            # N
            if walls[0]:
                grid[2 * (r + 1)][2 * c + 1] = "-"
            # E
            if walls[1]:
                grid[2 * r + 1][2 * (c + 1)] = "|"
            # S
            if walls[2]:
                grid[2 * r][2 * c + 1] = "-"
            # W
            if walls[3]:
                grid[2 * r + 1][2 * c] = "|"

    # Breadcrumb trail.
    if trail:
        # Subsample to avoid drawing every tick.
        path = world.path
        if len(path) > 1:
            for (px, py) in path[::5]:
                c = int(px / s)
                r = int(py / s)
                if 0 <= c < cols and 0 <= r < rows:
                    ch = grid[2 * r + 1][2 * c + 1]
                    if ch == " ":
                        grid[2 * r + 1][2 * c + 1] = "."

    # Mouse.
    c = int(world.x / s)
    r = int(world.y / s)
    if 0 <= c < cols and 0 <= r < rows:
        grid[2 * r + 1][2 * c + 1] = _heading_glyph(world.theta)

    # Goal.
    gc, gr = maze.goal_cell
    if grid[2 * gr + 1][2 * gc + 1] == " ":
        grid[2 * gr + 1][2 * gc + 1] = "G"

    # Bottom row first in our coords (y=0 is bottom); flip for display.
    lines = ["".join(row) for row in grid]
    return "\n".join(reversed(lines))
