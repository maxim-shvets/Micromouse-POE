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
