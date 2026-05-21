"""Matplotlib live visualization of the simulator.

Pass an instance as `on_step` to `algorithm.run`.  Each tick (down-sampled
by `viz_hz`) it redraws:

  - Maze walls and goal/start cells (static -- drawn once at construction).
  - The mouse: a triangle showing position + heading.  Colour reflects the
    `ReactiveController` state (REACT / REVERSE / PIVOT) so you can see
    recovery events at a glance.
  - The mouse's breadcrumb trail.
  - The three ToF rays, drawn out to whatever distance each one measured
    this tick.  (Toggle with `show_rays=False`.)
  - When the controller carries a planner, an overlay showing
    (a) the planner's known walls in their three states (known-wall solid
    black, known-open faint grey, unknown dashed grey), and
    (b) a flood-fill cost heatmap.  (Toggle with `show_planner=False`.)
  - A status line: sim time, distance, collisions, recovery count,
    aggression mode, peak/instant speed, and -- when a planner is wired --
    its current desired heading.

CPython / host only.  Imported lazily so the rest of the codebase still
runs without matplotlib installed.
"""

import math
import os


# State colour palette -- must agree with ReactiveController state ints.
_STATE_COLORS = {
    0: "#3b82f6",  # REACT  -- blue
    1: "#f97316",  # REVERSE -- orange
    2: "#ef4444",  # PIVOT  -- red
}
_STATE_NAMES = {0: "REACT", 1: "REVERSE", 2: "PIVOT"}


def _import_mpl():
    """Lazy matplotlib import with a friendly error message."""
    try:
        import matplotlib                         # noqa: F401
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle, Polygon
        from matplotlib.lines import Line2D
        from matplotlib.collections import LineCollection
        return plt, Rectangle, Polygon, Line2D, LineCollection
    except ImportError as e:
        raise RuntimeError(
            "matplotlib is required for --viz matplotlib.  Install with:\n"
            "    python3 -m pip install matplotlib"
        ) from e


class MatplotlibVisualizer(object):
    """Live matplotlib visualizer.  Plug into algorithm.run as on_step."""

    def __init__(self, world, tunables,
                 viz_hz=20.0,
                 show_rays=True,
                 show_planner=True,
                 save_frames_to=None,
                 figsize=None):
        """Args:
            world:          `SimWorld` (host-only).
            tunables:       `Tunables`.
            viz_hz:         Frames per simulated second to redraw.  Lower
                            this if the sim runs faster than the GUI can
                            keep up.
            show_rays:      Draw the three sensor rays each frame.
            show_planner:   Draw the planner's known map + flood-fill heat
                            when a planner is wired into the controller.
            save_frames_to: Optional directory; one PNG per redrawn frame.
                            Useful for assembling videos offline.
            figsize:        Optional (w, h) inches.  Defaults to a square
                            sized to the maze aspect.
        """
        (self._plt, self._Rect, self._Poly,
         self._Line, self._LineColl) = _import_mpl()

        self.world = world
        self.t = tunables
        self.viz_period_ticks = max(1, int(round(tunables.loop_hz / viz_hz)))
        self.show_rays = show_rays
        self.show_planner = show_planner
        self.save_frames_to = save_frames_to
        if save_frames_to is not None:
            os.makedirs(save_frames_to, exist_ok=True)
        self._frame_idx = 0
        self._peak_speed = 0.0

        self._plt.ion()
        maze = world.maze
        if figsize is None:
            # Aspect-true; longest side around 7 inches.  Reserve a band on
            # top for the two-line status text.
            longest = max(maze.cols, maze.rows)
            scale = 7.0 / longest
            figsize = (max(maze.cols * scale, 6.0),
                       maze.rows * scale + 1.0)
        self._fig, self._ax = self._plt.subplots(figsize=figsize)
        self._fig.canvas.manager.set_window_title("Micromouse simulator")
        self._setup_static_artists()
        self._setup_dynamic_artists()
        # Reserve top ~10% for status text.
        self._fig.tight_layout(rect=(0.02, 0.02, 0.98, 0.90))
        self._plt.show(block=False)
        self._plt.pause(0.001)

    # -------------------------------------------------------------------------
    # static artists -- drawn once
    # -------------------------------------------------------------------------

    def _setup_static_artists(self):
        ax = self._ax
        maze = self.world.maze
        s = maze.cell_size_m

        ax.set_xlim(-0.02, maze.width_m + 0.02)
        ax.set_ylim(-0.02, maze.height_m + 0.02)
        ax.set_aspect("equal")
        ax.set_xticks([])
        ax.set_yticks([])

        # Goal cell highlight (drawn under the planner heatmap and walls).
        gc, gr = maze.goal_cell
        ax.add_patch(self._Rect(
            (gc * s, gr * s), s, s, facecolor="#fde68a", edgecolor="none",
            zorder=0.5))

        # Start cell.  A more saturated green so it stays visible under the
        # 35%-alpha heatmap overlay.
        sc, sr = maze.start_cell
        ax.add_patch(self._Rect(
            (sc * s, sr * s), s, s, facecolor="#22c55e", edgecolor="none",
            alpha=0.55, zorder=0.6))

        # Goal cell highlight gets the same treatment for consistency.

        # Planner cost overlay slot (filled later in _setup_dynamic_artists).
        # We use imshow at extent [0..width, 0..height] so cells line up.
        self._heatmap = None

        # True maze walls.  Drawn last among static artists so they sit on
        # top of the goal/start fills.
        segs = []
        for c in range(maze.cols):
            for r in range(maze.rows):
                w = maze.walls[c][r]
                x0, y0 = c * s, r * s
                x1, y1 = x0 + s, y0 + s
                if w[0]:  # N
                    segs.append(((x0, y1), (x1, y1)))
                if w[1]:  # E
                    segs.append(((x1, y0), (x1, y1)))
                if w[2]:  # S
                    segs.append(((x0, y0), (x1, y0)))
                if w[3]:  # W
                    segs.append(((x0, y0), (x0, y1)))
        ax.add_collection(self._LineColl(
            segs, colors="black", linewidths=2.0, zorder=3))

    # -------------------------------------------------------------------------
    # dynamic artists -- updated each frame
    # -------------------------------------------------------------------------

    def _setup_dynamic_artists(self):
        ax = self._ax
        # Trail.
        self._trail_xs = []
        self._trail_ys = []
        (self._trail_line,) = ax.plot(
            [], [], color="#0ea5e9", linewidth=1.3, alpha=0.8, zorder=4)

        # Sensor rays.  Three Line2D, updated via set_data.
        if self.show_rays:
            self._ray_lines = [
                ax.plot([], [], color=c, linewidth=1.0, alpha=0.75,
                        linestyle="--", zorder=4)[0]
                for c in ("#10b981", "#a855f7", "#ec4899")  # F, L, R
            ]
        else:
            self._ray_lines = None

        # Mouse triangle.  Initial geometry is arbitrary; we set_xy on update.
        self._mouse_patch = self._Poly(
            [[0, 0], [0, 0], [0, 0]], closed=True,
            facecolor=_STATE_COLORS[0], edgecolor="black",
            linewidth=1.0, zorder=5)
        ax.add_patch(self._mouse_patch)

        # Heading whisker -- a short line from mouse centre forward.  Helps
        # see direction when zoomed out.
        (self._heading_line,) = ax.plot(
            [], [], color="black", linewidth=1.2, zorder=5)

        # Planner desired-direction arrow (only used when planner is wired).
        self._planner_arrow = None  # built on demand
        self._known_wall_artist = None  # LineCollection rebuilt on map change
        self._known_wall_sig = None    # hash to detect when to rebuild
        self._heatmap_data = None

        # Status text at figure level (not axes) so it doesn't clip when
        # the axes are narrower than the line.  Two lines to fit even on
        # the smallest reasonable maze (3x3).
        self._title = self._fig.text(
            0.5, 0.975, "", ha="center", va="top",
            fontsize=9.5, family="monospace")

    # -------------------------------------------------------------------------
    # on_step callback
    # -------------------------------------------------------------------------

    def __call__(self, i, reading, encoders, cmd, controller):
        if (i % self.viz_period_ticks) != 0:
            return

        W = self.world
        # ---- mouse triangle ------------------------------------------------
        self._update_mouse_patch(W.x, W.y, W.theta,
                                 _STATE_COLORS.get(controller.state,
                                                   _STATE_COLORS[0]))

        # ---- trail ---------------------------------------------------------
        self._trail_xs.append(W.x)
        self._trail_ys.append(W.y)
        self._trail_line.set_data(self._trail_xs, self._trail_ys)

        # ---- sensor rays ---------------------------------------------------
        if self._ray_lines is not None:
            self._update_rays(reading)

        # ---- planner overlay ----------------------------------------------
        planner = getattr(controller, "planner", None)
        if self.show_planner and planner is not None:
            self._update_planner_overlay(planner)

        # ---- status --------------------------------------------------------
        speed = 0.5 * (abs(encoders[0]) + abs(encoders[1]))
        if speed > self._peak_speed:
            self._peak_speed = speed
        self._update_title(W, controller, speed, reading)

        # ---- flush ---------------------------------------------------------
        self._fig.canvas.draw_idle()
        # Pause yields to the GUI event loop.  Tiny pause keeps the sim
        # near real-time without spinning the host CPU.
        self._plt.pause(0.001)

        # Optional frame export.
        if self.save_frames_to is not None:
            path = os.path.join(
                self.save_frames_to,
                "frame_{:06d}.png".format(self._frame_idx))
            self._fig.savefig(path, dpi=110)
            self._frame_idx += 1

    # -------------------------------------------------------------------------
    # update helpers
    # -------------------------------------------------------------------------

    def _update_mouse_patch(self, x, y, theta, color):
        """Triangular footprint pointing along heading."""
        r = self.t.chassis_radius_m
        # Apex forward, two rear corners flared 130 deg from the apex.
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        nose = (x + r * 1.1 * cos_t, y + r * 1.1 * sin_t)
        left = (x + r * math.cos(theta + 2.3),
                y + r * math.sin(theta + 2.3))
        right = (x + r * math.cos(theta - 2.3),
                 y + r * math.sin(theta - 2.3))
        self._mouse_patch.set_xy([nose, left, right])
        self._mouse_patch.set_facecolor(color)

        # Heading whisker extends one chassis radius further forward.
        whisker_end = (x + r * 1.8 * cos_t, y + r * 1.8 * sin_t)
        self._heading_line.set_data(
            [x, whisker_end[0]], [y, whisker_end[1]])

    def _update_rays(self, reading):
        T = self.t
        W = self.world
        off = T.sensor_forward_offset_m
        side = T.side_sensor_angle_rad
        ox = W.x + off * math.cos(W.theta)
        oy = W.y + off * math.sin(W.theta)
        angles = (W.theta, W.theta + side, W.theta - side)
        dists = (reading.front, reading.left, reading.right)
        max_r = T.sensor_max_range_m
        for ln, ang, d in zip(self._ray_lines, angles, dists):
            d = min(d, max_r)
            ln.set_data([ox, ox + d * math.cos(ang)],
                        [oy, oy + d * math.sin(ang)])

    def _update_planner_overlay(self, planner):
        # Rebuild the known-wall LineCollection only when the map changed.
        sig = self._known_wall_signature(planner.map)
        if sig != self._known_wall_sig:
            self._known_wall_sig = sig
            self._refresh_known_walls(planner.map)

        # Heatmap of flood-fill cost.  `planner._dist[c][r][d]` is per
        # facing; for visualization we take min over directions == best-case
        # cost from that cell.
        if planner._dist is not None:
            grid = self._cost_grid(planner)
            self._refresh_heatmap(grid)

    def _known_wall_signature(self, kmap):
        """Cheap hash of the known map's wall states for change detection."""
        # Tuple-of-tuples; small enough to hash directly for our maze sizes.
        return tuple(
            tuple(tuple(kmap.walls[c][r]) for r in range(kmap.rows))
            for c in range(kmap.cols))

    def _refresh_known_walls(self, kmap):
        """Highlight walls the planner has *confirmed* (known-True).

        Unknown walls aren't drawn -- at start the whole maze is unknown
        and the resulting dashed mesh drowns out the true walls.  Drawing
        only known-True walls in a contrasting colour gives a clean
        visual cue for "the planner has actually seen this wall."
        """
        s = self.world.maze.cell_size_m
        known_solid = []
        for c in range(kmap.cols):
            for r in range(kmap.rows):
                w = kmap.walls[c][r]
                x0, y0 = c * s, r * s
                x1, y1 = x0 + s, y0 + s
                edges = (
                    (0, ((x0, y1), (x1, y1))),  # N
                    (1, ((x1, y0), (x1, y1))),  # E
                    (2, ((x0, y0), (x1, y0))),  # S
                    (3, ((x0, y0), (x0, y1))),  # W
                )
                for d, seg in edges:
                    if w[d] is True:
                        known_solid.append(seg)

        if self._known_wall_artist is not None:
            self._known_wall_artist.remove()

        # Red overlay sits ON TOP of the true black walls -- where the
        # planner has confirmed, you see a red+black band; where it hasn't,
        # the wall stays plain black.
        self._known_wall_artist = self._LineColl(
            known_solid, colors="#dc2626", linewidths=3.5, alpha=0.55,
            zorder=2.7)
        self._ax.add_collection(self._known_wall_artist)

    def _cost_grid(self, planner):
        # imshow wants a 2D array shape (rows, cols) with rows from top->bot
        # by default; we flip y so origin is bottom-left, matching the maze.
        cols, rows = planner.cols, planner.rows
        grid = [[None] * cols for _ in range(rows)]
        max_finite = 0.0
        INF = 1 << 30
        for c in range(cols):
            for r in range(rows):
                cell = planner._dist[c][r]
                # min over the four facing directions.
                m = min(cell)
                if m >= INF:
                    grid[r][c] = None
                else:
                    grid[r][c] = m
                    if m > max_finite:
                        max_finite = m
        # Substitute None with NaN so imshow renders as transparent.
        import math
        out = [[math.nan if v is None else v for v in row] for row in grid]
        return out, max_finite

    def _refresh_heatmap(self, grid_packed):
        grid, max_finite = grid_packed
        maze = self.world.maze
        extent = [0, maze.width_m, 0, maze.height_m]
        if self._heatmap is None:
            self._heatmap = self._ax.imshow(
                grid, extent=extent, origin="lower",
                cmap="viridis_r", alpha=0.35, zorder=1,
                vmin=0, vmax=max(1.0, max_finite),
                interpolation="nearest")
        else:
            self._heatmap.set_data(grid)
            self._heatmap.set_clim(0, max(1.0, max_finite))

    def _update_title(self, world, controller, speed, reading):
        mode = getattr(self.t, "aggression_mode", "?")
        plan = getattr(controller, "planner", None)
        plan_str = ""
        if plan is not None:
            desired = getattr(controller, "_desired_heading", None)
            cardinal_name = ("N", "E", "S", "W")
            d_str = cardinal_name[desired] if desired is not None else "-"
            plan_str = "   plan->{}".format(d_str)
        line1 = "mode={}   state={}   t={:5.2f}s{}".format(
            mode, _STATE_NAMES.get(controller.state, "?"),
            world.t, plan_str)
        line2 = ("v={:.2f} m/s (peak {:.2f})   dist={:.2f}m   "
                 "coll={}   recov={}").format(
            speed, self._peak_speed, world.distance_traveled,
            world.collisions, controller.recovery_count)
        self._title.set_text(line1 + "\n" + line2)

    # -------------------------------------------------------------------------
    # lifecycle
    # -------------------------------------------------------------------------

    def close(self, hold=False):
        """Stop interactive mode.

        If `hold` is True, leave the window open until the user dismisses
        it (useful for inspecting the final state after a run); otherwise
        close immediately.
        """
        if hold:
            self._plt.ioff()
            self._plt.show()
        self._plt.close(self._fig)
