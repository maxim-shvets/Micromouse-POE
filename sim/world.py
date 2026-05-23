"""Simulated world: kinematics, ray-cast sensors, virtual clock.

`SimWorld` holds the maze + robot pose and steps a differential-drive model
forward in time.  `SimSensors`, `SimDrive`, and `SimClock` implement the
abstract interfaces from `interfaces.py` against it, so the same
`algorithm.run` that drives the RP2040 also drives the sim.

Wheel model: acceleration-limited first-order lag.
    accel = clip((cmd - speed) / sim_wheel_tau_s, -a_max, +a_max)
    speed += accel * dt

This captures both small-error settling (the lag term, behaves like the old
first-order model) and large-step rate limiting (the clip), which is closer
to what an N20 + DRV8833 actually does when you slam from 0 to commanded
cruise.  Tune `max_wheel_accel_mps2` to match the bench measurement.
"""

import math
import random

from interfaces import RangeSensors, Drive, Clock, Reading, IMU, IMUReading


_INF = float("inf")


# -----------------------------------------------------------------------------
# Geometry primitives
# -----------------------------------------------------------------------------

def _ray_segment_distance(ox, oy, dx, dy, x1, y1, x2, y2):
    """Distance from ray origin to its intersection with segment, or inf.

    Ray:  (ox, oy) + t*(dx, dy), t >= 0.   Segment: (x1,y1)-(x2,y2).
    Assumes (dx, dy) is unit length so the returned t is in meters.
    """
    sx = x2 - x1
    sy = y2 - y1
    denom = dx * sy - dy * sx
    if abs(denom) < 1e-12:
        return _INF  # parallel
    t = ((x1 - ox) * sy - (y1 - oy) * sx) / denom
    u = ((x1 - ox) * dy - (y1 - oy) * dx) / denom
    if t < 0.0 or u < 0.0 or u > 1.0:
        return _INF
    return t


def _point_segment_distance(px, py, x1, y1, x2, y2):
    vx = x2 - x1
    vy = y2 - y1
    wx = px - x1
    wy = py - y1
    c1 = vx * wx + vy * wy
    if c1 <= 0.0:
        return math.hypot(px - x1, py - y1)
    c2 = vx * vx + vy * vy
    if c2 <= c1:
        return math.hypot(px - x2, py - y2)
    b = c1 / c2
    bx = x1 + b * vx
    by = y1 + b * vy
    return math.hypot(px - bx, py - by)


# -----------------------------------------------------------------------------
# World
# -----------------------------------------------------------------------------

class SimWorld(object):
    """Maze + robot state + virtual clock."""

    def __init__(self, maze, tunables, start_pose=None):
        self.maze = maze
        self.t_ = tunables
        self.walls = maze.wall_segments()

        if start_pose is None:
            cx, cy = maze.cell_center(*maze.start_cell)
            self.x = cx
            self.y = cy
            self.theta = 0.5 * math.pi  # face +y (north) by default
        else:
            self.x, self.y, self.theta = start_pose

        self._cmd_left = 0.0
        self._cmd_right = 0.0
        self._meas_left = 0.0
        self._meas_right = 0.0

        self.t = 0.0
        self.collisions = 0
        self._was_colliding = False
        self.distance_traveled = 0.0
        self.path = [(self.x, self.y)]

    # ---- kinematics ---------------------------------------------------------

    def set_command(self, left_mps, right_mps):
        self._cmd_left = left_mps
        self._cmd_right = right_mps

    def measured(self):
        return (self._meas_left, self._meas_right)

    def tick(self, dt):
        T = self.t_
        # Acceleration-limited first-order lag per wheel.
        self._meas_left = _step_wheel(self._meas_left, self._cmd_left, dt, T)
        self._meas_right = _step_wheel(self._meas_right, self._cmd_right, dt, T)

        v = 0.5 * (self._meas_left + self._meas_right)
        omega = (self._meas_right - self._meas_left) / T.wheel_base_m

        new_x = self.x + v * math.cos(self.theta) * dt
        new_y = self.y + v * math.sin(self.theta) * dt
        new_theta = self.theta + omega * dt

        if self._collides(new_x, new_y):
            if not self._was_colliding:
                self.collisions += 1
                self._was_colliding = True
            # Rotation still permitted in place so the robot can pivot
            # against a wall to look for an opening.
            if not self._collides(self.x, self.y):
                self.theta = new_theta
            # Forward velocity is reality-checked to zero (we're not moving).
            self._meas_left = 0.0
            self._meas_right = 0.0
        else:
            self._was_colliding = False
            self.distance_traveled += math.hypot(new_x - self.x, new_y - self.y)
            self.x = new_x
            self.y = new_y
            self.theta = new_theta

        # Wrap heading into [-pi, pi] so downstream consumers (telemetry,
        # visualizer, fusion midpoint integration) read sensible values
        # regardless of how many revolutions have accumulated.
        if self.theta > math.pi or self.theta < -math.pi:
            self.theta = ((self.theta + math.pi) % (2.0 * math.pi)) - math.pi

        self.t += dt
        self.path.append((self.x, self.y))

    def _collides(self, x, y):
        r = self.t_.chassis_radius_m
        for (x1, y1, x2, y2) in self.walls:
            if _point_segment_distance(x, y, x1, y1, x2, y2) < r:
                return True
        return False

    # ---- sensing ------------------------------------------------------------

    def cast_ray(self, angle_rad):
        T = self.t_
        max_range = T.sensor_max_range_m
        off = T.sensor_forward_offset_m
        ox = self.x + off * math.cos(self.theta)
        oy = self.y + off * math.sin(self.theta)
        a = self.theta + angle_rad
        dx = math.cos(a)
        dy = math.sin(a)
        best = max_range
        for (x1, y1, x2, y2) in self.walls:
            d = _ray_segment_distance(ox, oy, dx, dy, x1, y1, x2, y2)
            if d < best:
                best = d
        return best

    def reached_goal(self):
        gx, gy = self.maze.cell_center(*self.maze.goal_cell)
        return math.hypot(self.x - gx, self.y - gy) < self.maze.cell_size_m * 0.4


def _step_wheel(speed, cmd, dt, T):
    """Advance one wheel by dt with acceleration-limited first-order lag."""
    tau = T.sim_wheel_tau_s
    if tau <= 0.0:
        # Degenerate: clip directly.
        accel = (cmd - speed) / max(dt, 1e-9)
    else:
        accel = (cmd - speed) / tau
    a_max = T.max_wheel_accel_mps2
    if accel > a_max:
        accel = a_max
    elif accel < -a_max:
        accel = -a_max
    return speed + accel * dt


# -----------------------------------------------------------------------------
# Interface adapters
# -----------------------------------------------------------------------------

class SimSensors(RangeSensors):
    """3-ray ToF model: forward, +angle (left), -angle (right)."""

    def __init__(self, world, rng_seed=0xC0FFEE):
        self.world = world
        self._rng = random.Random(rng_seed)

    def read(self):
        W = self.world
        T = W.t_
        side = T.side_sensor_angle_rad
        front = W.cast_ray(0.0)
        left = W.cast_ray(+side)
        right = W.cast_ray(-side)
        sigma = T.sensor_noise_sigma_m
        if sigma > 0.0:
            front += self._rng.gauss(0.0, sigma)
            left += self._rng.gauss(0.0, sigma)
            right += self._rng.gauss(0.0, sigma)
        return Reading(front, left, right, timestamp=W.t)


class SimDrive(Drive):
    def __init__(self, world):
        self.world = world

    def set_wheel_speeds(self, cmd):
        self.world.set_command(cmd.left, cmd.right)

    def read_encoders(self):
        return self.world.measured()


class SimIMU(IMU):
    """Virtual 6-axis IMU derived from the world kinematics state.

    The robot is planar -- accel_z carries gravity, gyro_x and gyro_y are
    pure noise.  The interesting channels are:

        accel_x = dv/dt   (forward accel; spikes when the wheel model
                           rate-limits during a step change)
        accel_y = v * w   (centripetal lateral accel during turns; sign
                           convention: left turn = w > 0 = a_y > 0)
        gyro_z  = w       (yaw rate; what the fusion layer integrates)

    On top of the ground-truth signal we layer:
      - white noise (`imu_noise_accel_mps2`, `imu_noise_gyro_rps`)
      - a constant gyro_z bias (`imu_bias_gyro_z_rps`) so the fusion code
        gets a non-trivial bias-estimation problem to solve

    The IMU must be read exactly once per control tick so its
    differentiator (for accel_x) sees a consistent dt.  `algorithm.run`
    polls it; everything else reads via `controller.imu_reading`.
    """

    GRAVITY_MPS2 = 9.81

    def __init__(self, world, rng_seed=0xC0DECAFE):
        self.world = world
        import random
        self._rng = random.Random(rng_seed)
        self._prev_v = 0.0
        self._prev_t = 0.0

    def read(self):
        W = self.world
        T = W.t_
        meas_l, meas_r = W._meas_left, W._meas_right
        v = 0.5 * (meas_l + meas_r)
        omega = (meas_r - meas_l) / T.wheel_base_m

        now = W.t
        dt = now - self._prev_t
        if dt <= 0.0:
            ax = 0.0
        else:
            ax = (v - self._prev_v) / dt
        ay = v * omega        # centripetal lateral, +y = left turn
        az = self.GRAVITY_MPS2

        wx = 0.0
        wy = 0.0
        wz = omega

        # Sensor model: white noise + constant gyro_z bias.
        sa = T.imu_noise_accel_mps2
        sg = T.imu_noise_gyro_rps
        if sa > 0.0:
            ax += self._rng.gauss(0.0, sa)
            ay += self._rng.gauss(0.0, sa)
            az += self._rng.gauss(0.0, sa)
        if sg > 0.0:
            wx += self._rng.gauss(0.0, sg)
            wy += self._rng.gauss(0.0, sg)
            wz += self._rng.gauss(0.0, sg)
        wz += T.imu_bias_gyro_z_rps

        self._prev_v = v
        self._prev_t = now
        return IMUReading(ax, ay, az, wx, wy, wz, timestamp=now)


class SimClock(Clock):
    """Virtual clock -- `sleep(dt)` advances the world by dt and returns
    immediately, so simulation runs as fast as the host can compute it.

    Substep size matters for collision integrity at speed: the wall check
    runs once per substep, so motion-per-substep must be less than the
    chassis radius or the robot can teleport past a wall undetected.
    Pulled from `Tunables.sim_max_substep_s` when the world carries one;
    falls back to the explicit `max_substep_s` arg otherwise.
    """

    def __init__(self, world, max_substep_s=None):
        self.world = world
        if max_substep_s is None:
            # Prefer the world's tunable; fall back to a safe default.
            t = getattr(world, "t_", None)
            if t is not None and hasattr(t, "sim_max_substep_s"):
                max_substep_s = t.sim_max_substep_s
            else:
                max_substep_s = 0.005
        self.max_substep_s = float(max_substep_s)

    def now(self):
        return self.world.t

    def sleep(self, seconds):
        remaining = seconds
        max_step = self.max_substep_s
        while remaining > 0.0:
            dt = max_step if remaining > max_step else remaining
            self.world.tick(dt)
            remaining -= dt
