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
        if reading is not None and self.t.slam_correction_gain > 0.0:
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
        segments = _known_wall_segments(self.km, T.planner_cell_size_m)
        if not segments:
            self.corrections_skipped += 1
            return

        z = [_clamp_distance(reading.front, T.sensor_max_range_m),
             _clamp_distance(reading.left, T.sensor_max_range_m),
             _clamp_distance(reading.right, T.sensor_max_range_m)]
        z_pred = _measurement_vector(self._x[0], self._x[1], self._x[2],
                                     segments, T)
        H = _measurement_jacobian(self._x[0], self._x[1], self._x[2],
                                  segments, T)
        residual = [z[i] - z_pred[i] for i in range(3)]

        R_var = max(getattr(T, "slam_measurement_noise", 9.0e-6), 1.0e-12)
        S_gate = _innovation_covariance(H, self._P, R_var)
        gate_sq = getattr(T, "slam_gate_sigma", 3.0)
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
        self.last_correction_mag_m = math.hypot(delta[0], delta[1])


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
