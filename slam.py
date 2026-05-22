"""ToF scan-matching SLAM.

Sits on top of dead-reckoning (`FusedOdometry`).  Each tick, after the
dead-reckoning step, we compare the ToF reading against what we'd expect
given the planner's known walls and the current pose estimate, then nudge
the pose to make the two consistent.

Strategy: per-axis perpendicular-wall correction.
  - Snap to nearest cardinal heading; skip if not aligned (mid-pivot
    means the side rays don't project cleanly onto cell walls).
  - Determine current cell from pose.
  - For each sensor whose ray hits a *known-True* wall in the current
    cell, compute the implied robot position along the relevant axis and
    apply a fraction (`slam_correction_gain`) of the correction.

Forward sensor corrects position along heading.  Side sensors (at +/- 45
deg) correct position perpendicular to heading.  Together these bound
dead-reckoning error to the order of sensor noise; without them, position
error grows with travelled distance.

We deliberately don't try to correct theta -- the IMU + complementary
filter does that job well already (see `pose_fusion.py`), and trying to
co-correct theta and (x, y) from three rays is under-determined.

API matches FusedOdometry / EncoderOdometry so the planner uses it
transparently.

CircuitPython-portable.
"""

import math


# Cardinals -- match planner.py.  Duplicated rather than imported to keep
# slam.py self-contained when the planner module is being reworked.
_N, _E, _S, _W = 0, 1, 2, 3


def _theta_from_heading(d):
    if d == _N:
        return 0.5 * math.pi
    if d == _E:
        return 0.0
    if d == _S:
        return -0.5 * math.pi
    return math.pi  # W


def _wrap_pi(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


# Unit forward + left vectors for each cardinal heading (in world frame).
_HEADING_VECTORS = {
    _N: ((0.0, 1.0),  (-1.0, 0.0)),   # fwd=+y, left=-x
    _E: ((1.0, 0.0),  (0.0, 1.0)),    # fwd=+x, left=+y
    _S: ((0.0, -1.0), (1.0, 0.0)),    # fwd=-y, left=+x
    _W: ((-1.0, 0.0), (0.0, -1.0)),   # fwd=-x, left=-y
}


class ScanMatchSlam(object):
    """Pose estimator: dead-reckoning + per-tick scan-match correction.

    Owns a `FusedOdometry` for the prediction step, then nudges its state
    based on the ToF reading + the planner's `KnownMap`.

    Drop-in for FusedOdometry / EncoderOdometry signatures:
        slam = ScanMatchSlam(x0, y0, theta0, known_map, tunables)
        slam.update(left_mps, right_mps, dt,
                    imu_reading=imu_r, reading=ranges)
        x, y, theta = slam.pose()
    """

    def __init__(self, x0, y0, theta0, known_map, tunables):
        from pose_fusion import FusedOdometry
        self._fused = FusedOdometry(x0, y0, theta0, tunables)
        self.km = known_map
        self.t = tunables
        # Bookkeeping for telemetry / advisor.
        self.corrections_applied = 0
        self.corrections_skipped = 0
        self.last_correction_mag_m = 0.0

    def update(self, left_mps, right_mps, dt,
               imu_reading=None, reading=None):
        # Step 1: dead-reckoning forward prediction.
        self._fused.update(left_mps, right_mps, dt, imu_reading=imu_reading)
        # Step 2: scan-match correction.
        if reading is not None and self.t.slam_correction_gain > 0.0:
            self._scan_match(reading)

    def pose(self):
        return self._fused.pose()

    @property
    def bias_z(self):
        return self._fused.bias_z

    # -- internals ---------------------------------------------------------

    def _scan_match(self, reading):
        T = self.t
        x, y, theta = self._fused.pose()

        # Snap to cardinal; skip if too far off-axis.
        heading = None
        for h in (_N, _E, _S, _W):
            if abs(_wrap_pi(theta - _theta_from_heading(h))) < T.slam_observe_tol_rad:
                heading = h
                break
        if heading is None:
            self.corrections_skipped += 1
            return

        s = T.planner_cell_size_m
        c = int(x / s)
        r = int(y / s)
        if not (0 <= c < self.km.cols and 0 <= r < self.km.rows):
            self.corrections_skipped += 1
            return

        fwd, left = _HEADING_VECTORS[heading]
        right_vec = (-left[0], -left[1])

        off = T.sensor_forward_offset_m
        gain = T.slam_correction_gain
        max_res = T.slam_max_residual_m
        min_clr = T.slam_min_clearance_m
        max_rng = T.sensor_max_range_m
        deadband = T.slam_deadband_m

        dx_total = 0.0
        dy_total = 0.0
        applied = False

        # ---- FORWARD --------------------------------------------------
        # Only meaningful when a wall is known to be at the cell's
        # forward edge along heading.
        if self.km.is_blocked(c, r, heading):
            if min_clr < reading.front < max_rng * 0.95:
                expected = _forward_dist_to_edge(heading, c, r, x, y, s) - off
                if expected > 0.0:
                    residual = expected - reading.front
                    if deadband <= abs(residual) <= max_res:
                        dx_total += fwd[0] * gain * residual
                        dy_total += fwd[1] * gain * residual
                        applied = True

        # ---- LEFT side ------------------------------------------------
        left_wall_dir = (heading + 3) % 4   # CCW 90 deg
        if self.km.is_blocked(c, r, left_wall_dir):
            if min_clr < reading.left < max_rng * 0.95:
                perp = _perpendicular_dist_to_edge(left_wall_dir, c, r, x, y, s)
                expected = perp * math.sqrt(2.0)  # 45 deg ray geometry
                if expected > 0.0:
                    residual = expected - reading.left
                    if deadband <= abs(residual) <= max_res:
                        dx_total += left[0] * gain * residual
                        dy_total += left[1] * gain * residual
                        applied = True

        # ---- RIGHT side -----------------------------------------------
        right_wall_dir = (heading + 1) % 4   # CW 90 deg
        if self.km.is_blocked(c, r, right_wall_dir):
            if min_clr < reading.right < max_rng * 0.95:
                perp = _perpendicular_dist_to_edge(right_wall_dir, c, r, x, y, s)
                expected = perp * math.sqrt(2.0)
                if expected > 0.0:
                    residual = expected - reading.right
                    if deadband <= abs(residual) <= max_res:
                        dx_total += right_vec[0] * gain * residual
                        dy_total += right_vec[1] * gain * residual
                        applied = True

        if applied:
            self._fused.x += dx_total
            self._fused.y += dy_total
            self.corrections_applied += 1
            self.last_correction_mag_m = math.hypot(dx_total, dy_total)
        else:
            self.corrections_skipped += 1


# -----------------------------------------------------------------------------
# Geometry helpers
# -----------------------------------------------------------------------------

def _forward_dist_to_edge(heading, c, r, x, y, s):
    """Distance from (x, y) to the forward edge of cell (c, r) along heading.

    Always positive when the robot is inside the cell.
    """
    if heading == _N:
        return (r + 1) * s - y
    if heading == _E:
        return (c + 1) * s - x
    if heading == _S:
        return y - r * s
    return x - c * s  # W


def _perpendicular_dist_to_edge(wall_dir, c, r, x, y, s):
    """Distance from (x, y) to the cell edge in direction `wall_dir`."""
    # Same formula as forward distance -- the function generalises.
    return _forward_dist_to_edge(wall_dir, c, r, x, y, s)
