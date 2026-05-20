"""Real-hardware adapter for the Maker Pi RP2040 demo rig.

This file is NOT imported by the simulator -- it depends on CircuitPython
libraries that don't exist on a host CPython install.  Drop the project on
the board (rename to `code.py` or import from there) and run.

Hardware:
  - Maker Pi RP2040
  - 3x VL53L0X ToF sensors behind a TCA9548A I2C multiplexer
      channel 0: front
      channel 1: left
      channel 2: right
  - 2x N20 6V 500 RPM motors with quadrature encoders, via the on-board
    motor driver (DC Motor 1 = LEFT, DC Motor 2 = RIGHT)
  - 4xAA NiMH pack on VMOTOR

Wire pins below to match your build before deploying.
"""

# -----------------------------------------------------------------------------
# Pin map -- EDIT THESE for your build.
# Maker Pi RP2040 default labels.  Confirm against the board silk-screen.
# -----------------------------------------------------------------------------
PIN_I2C_SDA = "GP2"
PIN_I2C_SCL = "GP3"

# Motor driver channels (DRV8833 on the Maker Pi RP2040).
PIN_M1A = "GP8"     # left motor IN1
PIN_M1B = "GP9"     # left motor IN2
PIN_M2A = "GP10"    # right motor IN1
PIN_M2B = "GP11"    # right motor IN2

# Encoder inputs.  Wire the encoder A/B channels here.
PIN_ENC_LA = "GP12"
PIN_ENC_LB = "GP13"
PIN_ENC_RA = "GP14"
PIN_ENC_RB = "GP15"

# TCA9548A channel assignments.
TCA_CHAN_FRONT = 0
TCA_CHAN_LEFT = 1
TCA_CHAN_RIGHT = 2

# Encoder counts per output-shaft revolution (N20 w/ Hall encoder + gearbox).
# Confirm with your specific motor's datasheet; 1:100 N20 with 7ppr Hall ->
# 700 edges/rev counted on one channel, 2800 on quadrature x4.
ENC_COUNTS_PER_REV = 700.0

# Wheel circumference -- recomputed from Tunables in DriverN20.__init__,
# but kept here as a sanity-check default that matches the demo wheels.
WHEEL_CIRCUMFERENCE_M = 0.032 * 3.141592653589793


def _import_circuitpython():
    """Lazy imports so this file is at least syntax-checkable on CPython."""
    import board       # noqa: F401
    import busio       # noqa: F401
    import digitalio   # noqa: F401
    import pwmio       # noqa: F401
    import countio     # noqa: F401
    import time as _time
    import adafruit_tca9548a
    import adafruit_vl53l0x
    return {
        "board": board, "busio": busio, "digitalio": digitalio,
        "pwmio": pwmio, "countio": countio, "time": _time,
        "tca9548a": adafruit_tca9548a, "vl53l0x": adafruit_vl53l0x,
    }


# -----------------------------------------------------------------------------
# Sensors
# -----------------------------------------------------------------------------

class TcaVL53L0X(object):
    """3-channel VL53L0X reader behind a TCA9548A mux."""

    def __init__(self, mods=None):
        if mods is None:
            mods = _import_circuitpython()
        self._mods = mods
        board = mods["board"]
        busio = mods["busio"]
        sda = getattr(board, PIN_I2C_SDA)
        scl = getattr(board, PIN_I2C_SCL)
        i2c = busio.I2C(scl, sda)
        self.mux = mods["tca9548a"].TCA9548A(i2c)
        self.front = mods["vl53l0x"].VL53L0X(self.mux[TCA_CHAN_FRONT])
        self.left = mods["vl53l0x"].VL53L0X(self.mux[TCA_CHAN_LEFT])
        self.right = mods["vl53l0x"].VL53L0X(self.mux[TCA_CHAN_RIGHT])
        # Long-range timing budget (~33 ms).  Trade range vs. loop rate.
        for s in (self.front, self.left, self.right):
            s.measurement_timing_budget = 33000

    def read(self):
        from interfaces import Reading
        # range_mm -> meters.  Cap at 1.2 m (sensor max-reliable).
        def _to_m(mm):
            d = mm / 1000.0
            if d <= 0.0 or d > 1.2:
                return 1.2
            return d
        return Reading(
            front=_to_m(self.front.range),
            left=_to_m(self.left.range),
            right=_to_m(self.right.range),
            timestamp=self._mods["time"].monotonic(),
        )


# -----------------------------------------------------------------------------
# Motor driver + encoders
# -----------------------------------------------------------------------------

class DriverN20(object):
    """Differential drive with two N20 + DRV8833 channels + encoders.

    Inner loop closed by `algorithm.WheelController` -- this class only
    converts commanded m/s to PWM duty given measured wheel speeds.
    """

    PWM_FREQ_HZ = 20000  # above audible range

    def __init__(self, tunables, mods=None):
        if mods is None:
            mods = _import_circuitpython()
        self._mods = mods
        self._t = tunables
        board = mods["board"]
        pwmio = mods["pwmio"]
        countio = mods["countio"]

        def _pwm(pin):
            return pwmio.PWMOut(getattr(board, pin), frequency=self.PWM_FREQ_HZ)

        self._m1a = _pwm(PIN_M1A)
        self._m1b = _pwm(PIN_M1B)
        self._m2a = _pwm(PIN_M2A)
        self._m2b = _pwm(PIN_M2B)

        # Encoder counters -- single-channel rising-edge counts.  For
        # quadrature you'd need two `countio.Counter` and direction inference;
        # single-channel + sign-from-command is good enough for v1.
        self._enc_l = countio.Counter(getattr(board, PIN_ENC_LA))
        self._enc_r = countio.Counter(getattr(board, PIN_ENC_RA))

        from algorithm import WheelController
        self._wc_l = WheelController(tunables.encoder_kp, tunables.encoder_ki)
        self._wc_r = WheelController(tunables.encoder_kp, tunables.encoder_ki)

        # Derived from tunables so a wheel-diameter override propagates.
        self._wheel_circ_m = tunables.wheel_diameter_m * 3.141592653589793
        self._loop_dt = 1.0 / tunables.loop_hz

        self._last_count_l = 0
        self._last_count_r = 0
        self._last_t = mods["time"].monotonic()
        self._last_cmd_sign_l = 1
        self._last_cmd_sign_r = 1
        self._last_meas = (0.0, 0.0)

    def _set_pwm(self, a, b, duty):
        # duty in -1..1.  Drive one pin, hold the other.  16-bit PWM.
        if duty >= 0.0:
            a.duty_cycle = int(min(1.0, duty) * 65535)
            b.duty_cycle = 0
        else:
            a.duty_cycle = 0
            b.duty_cycle = int(min(1.0, -duty) * 65535)

    def _read_speed(self):
        """Compute (left_mps, right_mps) from encoder counts since last call.

        Sign inferred from most recent command sign.
        """
        now = self._mods["time"].monotonic()
        dt = now - self._last_t
        if dt <= 0.0:
            return (0.0, 0.0)
        cl = self._enc_l.count
        cr = self._enc_r.count
        dcl = cl - self._last_count_l
        dcr = cr - self._last_count_r
        self._last_count_l = cl
        self._last_count_r = cr
        self._last_t = now
        rev_l = dcl / ENC_COUNTS_PER_REV
        rev_r = dcr / ENC_COUNTS_PER_REV
        speed_l = self._last_cmd_sign_l * rev_l * self._wheel_circ_m / dt
        speed_r = self._last_cmd_sign_r * rev_r * self._wheel_circ_m / dt
        return (speed_l, speed_r)

    # ---- Drive interface ---------------------------------------------------

    def set_wheel_speeds(self, cmd):
        meas_l, meas_r = self._read_speed()
        # Approximated by the outer-loop period.  Tunables.loop_hz sets it.
        dt = self._loop_dt
        duty_l = self._wc_l.update(cmd.left, meas_l, dt)
        duty_r = self._wc_r.update(cmd.right, meas_r, dt)
        self._last_cmd_sign_l = 1 if cmd.left >= 0 else -1
        self._last_cmd_sign_r = 1 if cmd.right >= 0 else -1
        self._set_pwm(self._m1a, self._m1b, duty_l)
        self._set_pwm(self._m2a, self._m2b, duty_r)
        self._last_meas = (meas_l, meas_r)

    def read_encoders(self):
        return self._last_meas

    def stop(self):
        from interfaces import WheelSpeeds
        self.set_wheel_speeds(WheelSpeeds(0.0, 0.0))


# -----------------------------------------------------------------------------
# Real-time clock
# -----------------------------------------------------------------------------

class MonotonicClock(object):
    def __init__(self, mods=None):
        if mods is None:
            mods = _import_circuitpython()
        self._time = mods["time"]

    def now(self):
        return self._time.monotonic()

    def sleep(self, seconds):
        self._time.sleep(seconds)


# -----------------------------------------------------------------------------
# Top-level entry point.  Drop into `code.py` on the board.
# -----------------------------------------------------------------------------

def main(tunables=None):
    from algorithm import run
    from tunables import Tunables

    if tunables is None:
        # Try to load a saved profile from /tunables.json on the board's
        # filesystem; fall back to defaults if missing.
        try:
            tunables = Tunables.from_json_file("/tunables.json")
        except (OSError, ValueError):
            tunables = Tunables()

    sensors = TcaVL53L0X()
    drive = DriverN20(tunables)
    clock = MonotonicClock()
    run(sensors, drive, clock, tunables)


if __name__ == "__main__":
    main()
