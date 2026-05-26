"""3-DOF EKF scan-matching SLAM.

`ScanMatchSlam` wraps the smooth `FusedOdometry` dead-reckoning layer with a
small Extended Kalman Filter over pose `(x, y, theta)`.  Prediction advances
the EKF mean by the same world-frame delta produced by `FusedOdometry`, so the
two estimators stay coherent while the gyro-bias estimator remains owned by
the fused layer.  Measurement updates ray-cast the three ToF channels against
the planner's known-True walls and numerically differentiate that ray model;
this avoids brittle analytic derivatives when the active wall segment changes.

The corrected EKF mean is returned by `pose()`.  `dead_reckoning_pose()` returns
the raw fused pose without any SLAM correction, preserving the Bug #29 invariant
that planner.observe() can attribute walls from a smooth, jitter-free pose.

CircuitPython-portable: plain classes, lists for matrices, no numpy.
"""

import math

from planner import wrap_pi


_N, _E, _S, _W = 0, 1, 2, 3
_INF = float("inf")


class ScanMatchSlam(object):
    """Drop-in pose estimator: FusedOdometry prediction plus EKF correction."""

    def __init__(self, x0, y0, theta0, known_map, tunables):
        from pose_fusion import FusedOdometry
        self._fused = FusedOdometry(x0, y0, theta0, tunables)
        self.km = known_map
        self.t = tunables

        init_pos_var = getattr(tunables, "slam_init_pos_var", 1.0e-4)
        init_theta_var = getattr(tunables, "slam_init_theta_var", 1.0e-6)
        self._x = [float(x0), float(y0), wrap_pi(float(theta0))]
        self._P = [[init_pos_var, 0.0, 0.0],
                   [0.0, init_pos_var, 0.0],
                   [0.0, 0.0, init_theta_var]]

        # Bookkeeping for telemetry / advisor.
        self.corrections_applied = 0
        self.corrections_skipped = 0
        self.last_correction_mag_m = 0.0
        # Sub-sampling: only run the expensive measurement update every
        # `slam_measurement_period_ticks` ticks.  Prediction runs every
        # tick regardless.  See tunable comment.
        self._meas_tick_counter = 0
        # Cached wall-segment list (optimization: rebuilding it every
        # measurement update was ~10-20 us / 3-4 ms on XIAO per call,
        # and the segments only change when KnownMap.walls changes).
        # We tag the cache with the KnownMap's identity + a checksum of
        # the True wall count.  Cheap to validate, eliminates the cost.
        self._segments_cache = None
        self._segments_wall_count = -1

    def update(self, left_mps, right_mps, dt,
               imu_reading=None, reading=None):
        # Prediction: advance fused odometry first, then apply the exact same
        # world-frame delta to the EKF mean.
        before = self._fused.pose()
        self._fused.update(left_mps, right_mps, dt, imu_reading=imu_reading)
        after = self._fused.pose()

        dx = after[0] - before[0]
        dy = after[1] - before[1]
        dtheta = wrap_pi(after[2] - before[2])
        dt_cov = dt if dt > 0.0 else 0.0
        self._predict(dx, dy, dtheta, left_mps, right_mps, dt_cov)

        # Backward-compatible disable switch.  The gain no longer scales EKF
        # corrections; covariance and measurement noise do that job.
        # Sub-sample: only every Nth tick.  N=1 = every tick (legacy).
        if reading is not None and self.t.slam_correction_gain > 0.0:
            self._meas_tick_counter += 1
            period = self.t.slam_measurement_period_ticks
            if period < 1:
                period = 1
            if self._meas_tick_counter >= period:
                self._meas_tick_counter = 0
                self._measurement_update(reading)

    def pose(self):
        """Best EKF estimate `(x, y, theta)`."""
        return (self._x[0], self._x[1], self._x[2])

    def dead_reckoning_pose(self):
        """Smooth pre-correction pose from the underlying FusedOdometry."""
        return self._fused.pose()

    @property
    def bias_z(self):
        """Gyro-z bias estimate from the underlying FusedOdometry."""
        return self._fused.bias_z

    # -- helpers ----------------------------------------------------------

    def _wall_segments_cached(self, cell_size_m):
        """Return the wall-segment list, rebuilding only when KnownMap
        has gained / changed walls since the last call.

        Uses `KnownMap.generation` (monotonic counter, bumped on every
        set_wall call) for an O(1) freshness check.  Eliminates the
        per-measurement-update cost of rebuilding the segment list
        (was ~4096 wall-checks for a 16x16 maze).
        """
        gen = self.km.generation
        if (self._segments_cache is not None
                and self._segments_wall_count == gen):
            return self._segments_cache
        self._segments_cache = _known_wall_segments(self.km, cell_size_m)
        self._segments_wall_count = gen
        return self._segments_cache

    # -- EKF stages -------------------------------------------------------

    def _predict(self, dx, dy, dtheta, left_mps, right_mps, dt):
        self._x[0] += dx
        self._x[1] += dy
        self._x[2] = wrap_pi(self._x[2] + dtheta)

        vdt = 0.5 * (left_mps + right_mps) * dt
        theta_mid = wrap_pi(self._x[2] - 0.5 * dtheta)
        F = [[1.0, 0.0, -vdt * math.sin(theta_mid)],
             [0.0, 1.0,  vdt * math.cos(theta_mid)],
             [0.0, 0.0,  1.0]]
        Q = [[dt * self.t.slam_process_noise_x, 0.0, 0.0],
             [0.0, dt * self.t.slam_process_noise_y, 0.0],
             [0.0, 0.0, dt * self.t.slam_process_noise_theta]]

        FP = _matmul_3x3(F, self._P)
        self._P = _symmetrise_cov(_add_3x3(_matmul_3x3(FP, _transpose_3x3(F)), Q))

    def _measurement_update(self, reading):
        T = self.t

        z = [_clamp_distance(reading.front, T.sensor_max_range_m),
             _clamp_distance(reading.left, T.sensor_max_range_m),
             _clamp_distance(reading.right, T.sensor_max_range_m)]
        # Optimization A1: analytical Jacobian.  Compute z_pred + remember
        # which wall each ray hit, then close-form the partials.  Reduces
        # ray-cast count from 21 (3 + 18 perturbations) to 3 per tick --
        # ~7x speedup on the SLAM measurement update.  Combined with the
        # DDA grid-traversal ray cast (5-10x on the cast itself), the
        # measurement update is ~25-35x faster than the original.  Set
        # slam_jacobian_mode="central" to fall back to legacy numerical.
        mode = T.slam_jacobian_mode
        if mode == "analytical":
            # DDA path: walks the ray cell-by-cell through KnownMap.
            # No segment list needed.
            z_pred, hits = _measurement_vector_with_hits(
                self._x[0], self._x[1], self._x[2], self.km, T)
            H = _measurement_jacobian_analytical(
                self._x[0], self._x[1], self._x[2], hits, T)
        else:
            # Legacy central-diff path uses the segment list + iteration.
            segments = self._wall_segments_cached(T.planner_cell_size_m)
            if not segments:
                self.corrections_skipped += 1
                return
            z_pred = _measurement_vector(self._x[0], self._x[1], self._x[2],
                                         segments, T)
            H = _measurement_jacobian(self._x[0], self._x[1], self._x[2],
                                      segments, T)
        residual = [z[i] - z_pred[i] for i in range(3)]

        R_var = T.slam_measurement_noise
        if R_var < 1.0e-12:
            R_var = 1.0e-12
        S_gate = _innovation_covariance(H, self._P, R_var)
        gate_sq = T.slam_gate_sigma
        gate_sq *= gate_sq

        active = [True, True, True]
        useful = 0
        for i in range(3):
            if z_pred[i] >= T.sensor_max_range_m * 0.95:
                active[i] = False
            elif S_gate[i][i] <= 1.0e-12:
                active[i] = False
            elif (residual[i] * residual[i]) / S_gate[i][i] > gate_sq:
                active[i] = False
            else:
                useful += 1

        if useful == 0:
            self.corrections_skipped += 1
            return

        for i in range(3):
            if not active[i]:
                H[i] = [0.0, 0.0, 0.0]
                residual[i] = 0.0

        S = _innovation_covariance(H, self._P, R_var)
        S_inv = _inverse_3x3(S)
        if S_inv is None:
            self.corrections_skipped += 1
            return

        Ht = _transpose_3x3(H)
        K = _matmul_3x3(_matmul_3x3(self._P, Ht), S_inv)
        delta = _matvec_3x3(K, residual)

        self._x[0] += delta[0]
        self._x[1] += delta[1]
        self._x[2] = wrap_pi(self._x[2] + delta[2])

        I = _identity_3x3()
        KH = _matmul_3x3(K, H)
        A = _sub_3x3(I, KH)
        R = [[R_var, 0.0, 0.0], [0.0, R_var, 0.0], [0.0, 0.0, R_var]]
        APAt = _matmul_3x3(_matmul_3x3(A, self._P), _transpose_3x3(A))
        KRKt = _matmul_3x3(_matmul_3x3(K, R), _transpose_3x3(K))
        self._P = _symmetrise_cov(_add_3x3(APAt, KRKt))

        self.corrections_applied += 1
        # MicroPython 1.x lacks math.hypot; use the equivalent sqrt form
        # so the file runs on CircuitPython AND MicroPython (smoke test).
        _dx, _dy = delta[0], delta[1]
        self.last_correction_mag_m = math.sqrt(_dx * _dx + _dy * _dy)


# -----------------------------------------------------------------------------
# Measurement model
# -----------------------------------------------------------------------------

def _known_wall_segments(known_map, cell_size_m):
    s = cell_size_m
    segments = []
    for c in range(known_map.cols):
        x0 = c * s
        x1 = (c + 1) * s
        for r in range(known_map.rows):
            y0 = r * s
            y1 = (r + 1) * s
            walls = known_map.walls[c][r]
            if walls[_N] is True:
                segments.append((x0, y1, x1, y1))
            if walls[_E] is True:
                segments.append((x1, y0, x1, y1))
            if r == 0 and walls[_S] is True:
                segments.append((x0, y0, x1, y0))
            if c == 0 and walls[_W] is True:
                segments.append((x0, y0, x0, y1))
    return segments


def _measurement_vector(x, y, theta, segments, tunables):
    side = tunables.side_sensor_angle_rad
    return [_predict_ray(x, y, theta, 0.0, segments, tunables),
            _predict_ray(x, y, theta, side, segments, tunables),
            _predict_ray(x, y, theta, -side, segments, tunables)]


def _predict_ray(x, y, theta, channel_offset, segments, tunables):
    off = tunables.sensor_forward_offset_m
    ox = x + off * math.cos(theta)
    oy = y + off * math.sin(theta)
    ray_theta = theta + channel_offset
    dx = math.cos(ray_theta)
    dy = math.sin(ray_theta)
    best = tunables.sensor_max_range_m
    for x1, y1, x2, y2 in segments:
        d = _ray_segment_distance(ox, oy, dx, dy, x1, y1, x2, y2)
        if d < best:
            best = d
    return best


def _measurement_jacobian(x, y, theta, segments, tunables):
    eps = 1.0e-4
    H = [[0.0, 0.0, 0.0],
         [0.0, 0.0, 0.0],
         [0.0, 0.0, 0.0]]
    perturb = ((eps, 0.0, 0.0), (0.0, eps, 0.0), (0.0, 0.0, eps))
    for j in range(3):
        dx, dy, dt = perturb[j]
        z_plus = _measurement_vector(x + dx, y + dy, wrap_pi(theta + dt),
                                     segments, tunables)
        z_minus = _measurement_vector(x - dx, y - dy, wrap_pi(theta - dt),
                                      segments, tunables)
        for i in range(3):
            h = (z_plus[i] - z_minus[i]) / (2.0 * eps)
            if h > 100.0:
                h = 100.0
            elif h < -100.0:
                h = -100.0
            H[i][j] = h
    return H


# -----------------------------------------------------------------------------
# Analytical Jacobian (optimization A1)
#
# For a ray from sensor origin (ox, oy) at world angle (theta + channel_offset)
# hitting wall segment ((x1, y1), (x2, y2)):
#
#     ray:     r(t) = (ox, oy) + t * (dx, dy)
#     segment: p(u) = (x1, y1) + u * (sx, sy)        sx = x2 - x1, sy = y2 - y1
#     ox = x + off*cos(theta),  oy = y + off*sin(theta)
#     dx = cos(theta + alpha),  dy = sin(theta + alpha)
#     denom = dx*sy - dy*sx
#     N     = (x1 - ox)*sy - (y1 - oy)*sx
#     t     = N / denom
#
# Closed-form partials of t w.r.t. (x, y, theta), assuming the *same* wall
# stays active across the perturbation (true for small dx,dy,dtheta away from
# wall corners; we clamp the magnitude to handle the corner cases):
#
#     dt/dx     = -sy / denom
#     dt/dy     =  sx / denom
#     dt/dtheta = (off*(sin(theta)*sy + cos(theta)*sx)
#                  + t*(dx*sx + dy*sy)) / denom
#
# Same |h| <= 100 clamp as the central-diff version, for safety at corners.
# Total cost: 3 ray-casts per tick (down from 21).  ~7x speedup.
# -----------------------------------------------------------------------------

def _predict_ray_with_hit(x, y, theta, channel_offset, known_map, tunables):
    """Cast one ray via DDA grid traversal.

    Returns (distance, sx, sy).  Uses `_ray_cast_dda` -- walks cells one
    by one along the ray, checks only the 1-2 walls of each cell that
    the ray could exit through.  ~5-10x faster than iterating the full
    wall segment list, since each ray now visits ~1-7 cells (each cell
    has 4 walls max) instead of scanning 40-80 segments.

    When no wall is hit within sensor range, returns
    (max_range, 0.0, 0.0) -- the analytical Jacobian row will be zero.
    """
    off = tunables.sensor_forward_offset_m
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    ox = x + off * cos_t
    oy = y + off * sin_t
    ray_theta = theta + channel_offset
    dx = math.cos(ray_theta)
    dy = math.sin(ray_theta)
    return _ray_cast_dda(ox, oy, dx, dy, known_map,
                         tunables.planner_cell_size_m,
                         tunables.sensor_max_range_m)


def _measurement_vector_with_hits(x, y, theta, known_map, tunables):
    """Compute z_pred for the 3 channels and the hit-wall info per channel.

    Returns (z_pred_list, hits_list) where hits_list[i] is
    (sx, sy, channel_offset, distance_t).  The analytical Jacobian uses
    all four fields.  When the ray missed, sx = sy = 0.0 and the row
    will be zeroed.
    """
    side = tunables.side_sensor_angle_rad
    offsets = (0.0, side, -side)
    z_pred = [0.0, 0.0, 0.0]
    hits = [(0.0, 0.0, 0.0, 0.0)] * 3
    for i in range(3):
        alpha = offsets[i]
        d, sx, sy = _predict_ray_with_hit(x, y, theta, alpha, known_map,
                                          tunables)
        z_pred[i] = d
        hits[i] = (sx, sy, alpha, d)
    return z_pred, hits


def _ray_cast_dda(ox, oy, dx, dy, known_map, cell_size, max_range):
    """DDA grid-traversal ray cast against a `KnownMap`.

    Returns (distance, sx, sy):
      - distance: ray-length to the nearest known-True wall along the
        ray, clamped to max_range.
      - (sx, sy): direction vector of the wall segment that was hit
        (= (0, cell_size) for an E/W vertical wall,
           (cell_size, 0) for an N/S horizontal wall).  Used by the
        analytical Jacobian.  (0.0, 0.0) when no wall hit.

    Algorithm: classic Amanatides-Woo DDA.  Step along the ray one cell
    at a time using `tMaxX` / `tMaxY` (parametric distances to the next
    grid line in each axis).  At each cell-exit boundary, check the
    corresponding wall of the current cell; if True (closed), the ray
    stops there.  Otherwise, advance to the neighbor cell and continue.

    Cost: O(cells_visited) ~ O(max_range / cell_size).  For 0.18 m cells
    and 1.2 m sensor range, that's ~7 cells max.  Each visit does 1-2
    wall lookups.  Total: ~10-14 wall checks per ray, vs ~40-80 with the
    segment-list iteration.  5-10x speedup on the ray cast.
    """
    s = cell_size
    cols = known_map.cols
    rows = known_map.rows
    walls = known_map.walls

    cx = int(ox / s)
    cy = int(oy / s)
    if cx < 0 or cx >= cols or cy < 0 or cy >= rows:
        # Origin outside grid -- shouldn't happen in normal operation
        # (robot is always in a cell), but bail safely if it does.
        return (max_range, 0.0, 0.0)

    # Parametric step sizes.  Tiny denominators -> treat as zero step.
    if dx > 1.0e-12:
        stepx = 1
        tDeltaX = s / dx
        tMaxX = ((cx + 1) * s - ox) / dx
    elif dx < -1.0e-12:
        stepx = -1
        tDeltaX = -s / dx
        tMaxX = (cx * s - ox) / dx
    else:
        stepx = 0
        tDeltaX = _INF
        tMaxX = _INF

    if dy > 1.0e-12:
        stepy = 1
        tDeltaY = s / dy
        tMaxY = ((cy + 1) * s - oy) / dy
    elif dy < -1.0e-12:
        stepy = -1
        tDeltaY = -s / dy
        tMaxY = (cy * s - oy) / dy
    else:
        stepy = 0
        tDeltaY = _INF
        tMaxY = _INF

    # Step through cells until we hit a wall or exit the grid / range.
    while True:
        if tMaxX < tMaxY:
            wall_t = tMaxX
            if wall_t > max_range:
                return (max_range, 0.0, 0.0)
            wall_dir = _E if stepx > 0 else _W
            if walls[cx][cy][wall_dir] is True:
                # Vertical wall.  (sx, sy) = (0, s).
                return (wall_t, 0.0, s)
            cx += stepx
            if cx < 0 or cx >= cols:
                return (max_range, 0.0, 0.0)
            tMaxX += tDeltaX
        else:
            wall_t = tMaxY
            if wall_t > max_range:
                return (max_range, 0.0, 0.0)
            wall_dir = _N if stepy > 0 else _S
            if walls[cx][cy][wall_dir] is True:
                # Horizontal wall.  (sx, sy) = (s, 0).
                return (wall_t, s, 0.0)
            cy += stepy
            if cy < 0 or cy >= rows:
                return (max_range, 0.0, 0.0)
            tMaxY += tDeltaY


def _measurement_jacobian_analytical(x, y, theta, hits, tunables):
    """Closed-form Jacobian H[i][j] = partial of ray i w.r.t. state j.

    Falls back to a zero row when the ray missed (sx == sy == 0) or the
    ray is near-parallel to the wall (denom near 0).  Same |h| <= 100
    clamp as the central-diff version, so corner-of-wall discontinuities
    don't inject huge spikes.
    """
    H = [[0.0, 0.0, 0.0],
         [0.0, 0.0, 0.0],
         [0.0, 0.0, 0.0]]
    off = tunables.sensor_forward_offset_m
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    for i in range(3):
        sx, sy, alpha, t = hits[i]
        if sx == 0.0 and sy == 0.0:
            continue
        ray_theta = theta + alpha
        dx = math.cos(ray_theta)
        dy = math.sin(ray_theta)
        denom = dx * sy - dy * sx
        if abs(denom) < 1.0e-9:
            continue
        dt_dx = -sy / denom
        dt_dy = sx / denom
        dt_dtheta = (off * (sin_t * sy + cos_t * sx)
                     + t * (dx * sx + dy * sy)) / denom
        if dt_dx > 100.0:
            dt_dx = 100.0
        elif dt_dx < -100.0:
            dt_dx = -100.0
        if dt_dy > 100.0:
            dt_dy = 100.0
        elif dt_dy < -100.0:
            dt_dy = -100.0
        if dt_dtheta > 100.0:
            dt_dtheta = 100.0
        elif dt_dtheta < -100.0:
            dt_dtheta = -100.0
        H[i][0] = dt_dx
        H[i][1] = dt_dy
        H[i][2] = dt_dtheta
    return H


def _ray_segment_distance(ox, oy, dx, dy, x1, y1, x2, y2):
    sx = x2 - x1
    sy = y2 - y1
    denom = dx * sy - dy * sx
    if abs(denom) < 1.0e-12:
        return _INF
    t = ((x1 - ox) * sy - (y1 - oy) * sx) / denom
    u = ((x1 - ox) * dy - (y1 - oy) * dx) / denom
    if t < 0.0 or u < 0.0 or u > 1.0:
        return _INF
    return t


def _clamp_distance(v, max_range):
    if v != v:
        return max_range
    if v < 0.0:
        return 0.0
    if v > max_range:
        return max_range
    return v


# -----------------------------------------------------------------------------
# Tiny 3x3 linear algebra helpers
# -----------------------------------------------------------------------------

def _identity_3x3():
    return [[1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0]]


def _transpose_3x3(A):
    return [[A[0][0], A[1][0], A[2][0]],
            [A[0][1], A[1][1], A[2][1]],
            [A[0][2], A[1][2], A[2][2]]]


def _matmul_3x3(A, B):
    C = [[0.0, 0.0, 0.0],
         [0.0, 0.0, 0.0],
         [0.0, 0.0, 0.0]]
    for i in range(3):
        for j in range(3):
            C[i][j] = (A[i][0] * B[0][j]
                       + A[i][1] * B[1][j]
                       + A[i][2] * B[2][j])
    return C


def _matvec_3x3(A, v):
    return [A[0][0] * v[0] + A[0][1] * v[1] + A[0][2] * v[2],
            A[1][0] * v[0] + A[1][1] * v[1] + A[1][2] * v[2],
            A[2][0] * v[0] + A[2][1] * v[1] + A[2][2] * v[2]]


def _add_3x3(A, B):
    return [[A[0][0] + B[0][0], A[0][1] + B[0][1], A[0][2] + B[0][2]],
            [A[1][0] + B[1][0], A[1][1] + B[1][1], A[1][2] + B[1][2]],
            [A[2][0] + B[2][0], A[2][1] + B[2][1], A[2][2] + B[2][2]]]


def _sub_3x3(A, B):
    return [[A[0][0] - B[0][0], A[0][1] - B[0][1], A[0][2] - B[0][2]],
            [A[1][0] - B[1][0], A[1][1] - B[1][1], A[1][2] - B[1][2]],
            [A[2][0] - B[2][0], A[2][1] - B[2][1], A[2][2] - B[2][2]]]


def _innovation_covariance(H, P, R_var):
    S = _matmul_3x3(_matmul_3x3(H, P), _transpose_3x3(H))
    S[0][0] += R_var
    S[1][1] += R_var
    S[2][2] += R_var
    return S


def _inverse_3x3(M):
    a, b, c = M[0]
    d, e, f = M[1]
    g, h, i = M[2]
    det = (a * (e * i - f * h)
           - b * (d * i - f * g)
           + c * (d * h - e * g))
    if abs(det) < 1.0e-12:
        return None
    inv_det = 1.0 / det
    return [[(e * i - f * h) * inv_det,
             (c * h - b * i) * inv_det,
             (b * f - c * e) * inv_det],
            [(f * g - d * i) * inv_det,
             (a * i - c * g) * inv_det,
             (c * d - a * f) * inv_det],
            [(d * h - e * g) * inv_det,
             (b * g - a * h) * inv_det,
             (a * e - b * d) * inv_det]]


def _symmetrise_cov(P):
    p01 = 0.5 * (P[0][1] + P[1][0])
    p02 = 0.5 * (P[0][2] + P[2][0])
    p12 = 0.5 * (P[1][2] + P[2][1])
    P[0][1] = p01
    P[1][0] = p01
    P[0][2] = p02
    P[2][0] = p02
    P[1][2] = p12
    P[2][1] = p12
    for i in range(3):
        if P[i][i] < 1.0e-12:
            P[i][i] = 1.0e-12
    return P
