"""Hardware-agnostic interfaces.

The algorithm only ever talks to these abstract types.  Sim and real-hardware
implementations live in `sim/` and `hardware/` respectively.  Keeping the
algorithm code free of `time.sleep`, `board`, `busio`, etc. means the exact
same `algorithm.step()` runs in CPython on a laptop and CircuitPython on the
RP2040 with zero edits.

Plain classes (no @dataclass / typing imports) so this file is portable to
CircuitPython, which has only partial support for those modules.
"""


class Reading(object):
    """Three forward-cone distance measurements in meters.

    `inf` (or a large sentinel) means 'no return within sensor range'.
    """

    __slots__ = ("front", "left", "right", "timestamp")

    def __init__(self, front, left, right, timestamp=0.0):
        self.front = float(front)
        self.left = float(left)
        self.right = float(right)
        self.timestamp = float(timestamp)

    def __repr__(self):
        return "Reading(front={:.3f}, left={:.3f}, right={:.3f})".format(
            self.front, self.left, self.right
        )


class WheelSpeeds(object):
    """Commanded linear wheel speeds in meters / second.

    Sign convention: positive = wheel rolling forward.  A pivot-in-place is
    expressed as opposite signs on left/right.
    """

    __slots__ = ("left", "right")

    def __init__(self, left, right):
        self.left = float(left)
        self.right = float(right)

    def __repr__(self):
        return "WheelSpeeds(left={:.3f}, right={:.3f})".format(self.left, self.right)


class RangeSensors(object):
    """ABC for a 3-channel ToF sensor module (front / left / right)."""

    def read(self):
        """Return a `Reading`.  Must be non-blocking-ish (<< loop period)."""
        raise NotImplementedError


class Drive(object):
    """ABC for a differential-drive base with wheel encoders."""

    def set_wheel_speeds(self, cmd):
        """Apply a `WheelSpeeds` setpoint to the inner-loop controller."""
        raise NotImplementedError

    def read_encoders(self):
        """Return (left_speed_mps, right_speed_mps) measured from encoders."""
        raise NotImplementedError

    def stop(self):
        """Cut power.  Default: zero command."""
        self.set_wheel_speeds(WheelSpeeds(0.0, 0.0))


class Clock(object):
    """ABC for a clock + sleeper.  Lets the sim advance virtual time."""

    def now(self):
        raise NotImplementedError

    def sleep(self, seconds):
        raise NotImplementedError


class IMUReading(object):
    """One sample from a 6-axis IMU.

    Convention: right-handed body frame, x forward, y left, z up.
    Accel is m/s^2 including gravity on z (so a flat stationary robot
    reads ~(0, 0, +9.81)).  Gyro is rad/s.

    On a 2D differential-drive robot we mainly care about:
      - gyro_z   : yaw rate, used to correct heading-drift of encoder odometry
      - accel_x  : forward accel, used to detect wheel slip and crashes
      - accel_y  : lateral accel, useful as a centripetal sanity check in turns

    The remaining channels are still recorded -- they cost nothing and give
    later debugging an easier time.
    """

    __slots__ = ("accel_x", "accel_y", "accel_z",
                 "gyro_x", "gyro_y", "gyro_z", "timestamp")

    def __init__(self, accel_x, accel_y, accel_z,
                 gyro_x, gyro_y, gyro_z, timestamp=0.0):
        self.accel_x = float(accel_x)
        self.accel_y = float(accel_y)
        self.accel_z = float(accel_z)
        self.gyro_x = float(gyro_x)
        self.gyro_y = float(gyro_y)
        self.gyro_z = float(gyro_z)
        self.timestamp = float(timestamp)

    def __repr__(self):
        return ("IMUReading(a=({:+.2f}, {:+.2f}, {:+.2f}) m/s^2, "
                "w=({:+.2f}, {:+.2f}, {:+.2f}) rad/s)").format(
            self.accel_x, self.accel_y, self.accel_z,
            self.gyro_x, self.gyro_y, self.gyro_z)


class IMU(object):
    """ABC for a 6-axis IMU.  Wraps an LSM6DS3TR-C on real hardware; the
    sim derives accel + gyro from the world's kinematics state.
    """

    def read(self):
        """Return an `IMUReading`.  Must be non-blocking-ish."""
        raise NotImplementedError
