"""Hardware adapter for the XIAO nRF52840 Sense demo build.

Drop this file (and its dependencies) on the board, rename to `code.py`
or import `main` from there.  CircuitPython firmware ships with most of
what we need; the only library you may have to copy to /lib is
`adafruit_lsm6ds`.

Hardware (per the "FAST DEMO CONFIG - XIAO VERSION" spec):

    Controller : Seeed XIAO nRF52840 Sense
    Motor drv  : DRV8833 dual DC motor driver (breakout)
    Sensors    : 3 x VL53L0X (or TOF200C) on TCA9548A I2C mux,
                 + on-board LSM6DS3TR-C 6-axis IMU
    Drive      : 2 x N20 6V 300-500 RPM gearmotor + 32 mm wheels
    Support    : front-mounted small caster
    Power      : 4xAA NiMH (simplest) OR 2S LiPo + 5 V buck for logic
    Protection : 470-1000 uF cap across DRV8833 VM/GND, software PWM
                 duty cap (Tunables.motor_duty_cap), stuck-detection
                 timeout (Tunables.stuck_time_s)

Common ground everywhere -- the IMU + ToF + motor driver must share GND
or the I2C bus will misbehave under motor load.

Pin map (edit these to match your build before flashing).
The XIAO nRF52840 Sense exposes 11 GPIO: D0-D10, all 3.3 V logic.  A few
are duplicated with I2C/SPI/UART; the Sense variant also has an internal
I2C bus (1) wired to the on-board IMU + PDM mic.

CircuitPython-portable: lazy imports, no host-only modules at module
level, so this file is syntactically valid on CPython for unit / lint.
"""


# -----------------------------------------------------------------------------
# Pin map -- EDIT to match your wiring.
# -----------------------------------------------------------------------------

# External I2C bus (exposed at D4=SDA / D5=SCL on XIAO Sense).  Drives the
# TCA9548A multiplexer, which fans out to the three VL53L0X.
PIN_I2C_SDA = "D4"
PIN_I2C_SCL = "D5"

# Motor driver pins -- DRV8833 IN1/IN2/IN3/IN4.  Each motor uses a pair:
# one pin holds at 0 while the other PWMs to drive forward, swap for
# reverse.  The XIAO has hardware PWM on most pins.
PIN_M1A = "D0"   # left motor IN1 (AIN1 on DRV8833)
PIN_M1B = "D1"   # left motor IN2 (AIN2)
PIN_M2A = "D2"   # right motor IN1 (BIN1)
PIN_M2B = "D3"   # right motor IN2 (BIN2)

# Encoder inputs.  Use single-channel hall + sign-from-command for v1
# (quadrature support would need two countio.Counter per wheel and a
# direction-inference state machine).
PIN_ENC_LA = "D6"
PIN_ENC_RA = "D7"

# TCA9548A channel assignments -- keep consistent with the cable harness.
TCA_CHAN_FRONT = 0
TCA_CHAN_LEFT = 1
TCA_CHAN_RIGHT = 2

# Encoder counts per output-shaft revolution.  1:50 N20 + 11 ppr hall
# gives 550 edges/rev counted on a single channel; quadrature x4 = 2200.
# 1:100 with 7 ppr -> 700 / 2800.  Confirm against your specific motor.
ENC_COUNTS_PER_REV = 700.0


# -----------------------------------------------------------------------------
# Lazy CircuitPython import bundle
# -----------------------------------------------------------------------------

def _import_circuitpython():
    """Lazy imports so this file is syntax-checkable on CPython."""
    import board       # noqa: F401
    import busio       # noqa: F401
    import digitalio   # noqa: F401
    import pwmio       # noqa: F401
    import countio     # noqa: F401
    import time as _time
    import adafruit_tca9548a
    import adafruit_vl53l0x
    # On-board IMU on XIAO Sense.  LSM6DS3TR-C is a register-compatible
    # variant of LSM6DS3; adafruit_lsm6ds.lsm6ds3trc is the right driver.
    import adafruit_lsm6ds.lsm6ds3trc as _lsm6
    return {
        "board": board, "busio": busio, "digitalio": digitalio,
        "pwmio": pwmio, "countio": countio, "time": _time,
        "tca9548a": adafruit_tca9548a, "vl53l0x": adafruit_vl53l0x,
        "lsm6": _lsm6,
    }


# -----------------------------------------------------------------------------
# Sensors -- ToF triplet via TCA mux
# -----------------------------------------------------------------------------

class TcaVL53L0X(object):
    """3-channel VL53L0X reader behind a TCA9548A mux.

    Identical conceptually to the RP2040 build.  Reuses the same library
    chain; only the I2C bus pins differ.
    """

    def __init__(self, mods=None):
        if mods is None:
            mods = _import_circuitpython()
        self._mods = mods
        board = mods["board"]
        busio = mods["busio"]
        sda = getattr(board, PIN_I2C_SDA)
        scl = getattr(board, PIN_I2C_SCL)
        self._i2c = busio.I2C(scl, sda)
        self.mux = mods["tca9548a"].TCA9548A(self._i2c)
        self.front = mods["vl53l0x"].VL53L0X(self.mux[TCA_CHAN_FRONT])
        self.left = mods["vl53l0x"].VL53L0X(self.mux[TCA_CHAN_LEFT])
        self.right = mods["vl53l0x"].VL53L0X(self.mux[TCA_CHAN_RIGHT])
        # ~33 ms timing budget = max 30 Hz, plenty for the 50-200 Hz
        # control loop's needs.  Trade range vs. rate by adjusting.
        for s in (self.front, self.left, self.right):
            s.measurement_timing_budget = 33000

    def read(self):
        from interfaces import Reading
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
# IMU -- on-board LSM6DS3TR-C
# -----------------------------------------------------------------------------

class XiaoIMU(object):
    """On-board LSM6DS3TR-C 6-axis IMU.

    The XIAO Sense exposes the IMU on the internal I2C bus -- not the
    external D4/D5 bus we use for the ToF mux.  CircuitPython's `board`
    module on the XIAO Sense provides this as `board.IMU_I2C()` (or a
    dedicated SCL/SDA pair, depending on firmware version).  We try the
    helper first and fall back to manual pin construction.

    Axes (per LSM6DS3 datasheet, board orientation depends on mounting):
        x : forward  (mouse heading at theta=0)
        y : left
        z : up
    Adjust the `axis_remap` constants below if your physical mounting is
    different from the assumed orientation.
    """

    AXIS_REMAP = (
        # (sign, source_index) for (accel_x, accel_y, accel_z,
        #                            gyro_x,  gyro_y,  gyro_z)
        (+1, 0), (+1, 1), (+1, 2),
        (+1, 0), (+1, 1), (+1, 2),
    )

    DEG_TO_RAD = 0.017453292519943295

    def __init__(self, mods=None):
        if mods is None:
            mods = _import_circuitpython()
        self._mods = mods
        # Internal IMU I2C bus on XIAO Sense.  Some firmwares expose this
        # via board.IMU_I2C() helper; older ones need manual construction
        # from explicit pin names (P0_24/P0_25 on nRF52840).
        board = mods["board"]
        busio = mods["busio"]
        try:
            self._i2c = board.IMU_I2C()
        except AttributeError:
            # Manual pin names -- adjust for your board variant if these
            # constants aren't present.
            sda = getattr(board, "IMU_SDA", None) or getattr(board, "P0_25")
            scl = getattr(board, "IMU_SCL", None) or getattr(board, "P0_24")
            self._i2c = busio.I2C(scl, sda)
        self._dev = mods["lsm6"].LSM6DS3TRC(self._i2c)

    def read(self):
        from interfaces import IMUReading
        # accel in m/s^2, gyro in deg/s from the library.
        ax_raw, ay_raw, az_raw = self._dev.acceleration
        gx_raw, gy_raw, gz_raw = self._dev.gyro
        raw_a = (ax_raw, ay_raw, az_raw)
        raw_g = (gx_raw, gy_raw, gz_raw)
        rmap = self.AXIS_REMAP
        ax = rmap[0][0] * raw_a[rmap[0][1]]
        ay = rmap[1][0] * raw_a[rmap[1][1]]
        az = rmap[2][0] * raw_a[rmap[2][1]]
        # Gyro: deg/s -> rad/s, then axis-remap.
        gx = rmap[3][0] * raw_g[rmap[3][1]] * self.DEG_TO_RAD
        gy = rmap[4][0] * raw_g[rmap[4][1]] * self.DEG_TO_RAD
        gz = rmap[5][0] * raw_g[rmap[5][1]] * self.DEG_TO_RAD
        return IMUReading(ax, ay, az, gx, gy, gz,
                          timestamp=self._mods["time"].monotonic())


# -----------------------------------------------------------------------------
# Motor driver -- DRV8833 + N20 + single-channel hall encoders
# -----------------------------------------------------------------------------

class DriverN20(object):
    """Differential drive with two N20 + DRV8833 channels + encoders.

    Inner loop is closed by `algorithm.WheelController` (cmd_mps -> PWM
    duty); this class wraps PWM output + encoder counting + speed
    estimation.  Direction is inferred from the sign of the most recent
    command, which is good enough as long as the wheel speed doesn't
    reverse between encoder samples (true at our 50-200 Hz loop rate).
    """

    PWM_FREQ_HZ = 20000  # above audible range; well within DRV8833 spec

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
        self._enc_l = countio.Counter(getattr(board, PIN_ENC_LA))
        self._enc_r = countio.Counter(getattr(board, PIN_ENC_RA))

        from algorithm import WheelController
        # Apply the software PWM cap from tunables -- protects DRV8833
        # under sustained-stall conditions.
        cap = tunables.motor_duty_cap
        self._wc_l = WheelController(
            tunables.encoder_kp, tunables.encoder_ki,
            duty_min=-cap, duty_max=cap)
        self._wc_r = WheelController(
            tunables.encoder_kp, tunables.encoder_ki,
            duty_min=-cap, duty_max=cap)

        self._wheel_circ_m = tunables.wheel_diameter_m * 3.141592653589793
        self._loop_dt = 1.0 / tunables.loop_hz
        self._last_count_l = 0
        self._last_count_r = 0
        self._last_t = mods["time"].monotonic()
        self._last_cmd_sign_l = 1
        self._last_cmd_sign_r = 1
        self._last_meas = (0.0, 0.0)

    def _set_pwm(self, a, b, duty):
        if duty >= 0.0:
            a.duty_cycle = int(min(1.0, duty) * 65535)
            b.duty_cycle = 0
        else:
            a.duty_cycle = 0
            b.duty_cycle = int(min(1.0, -duty) * 65535)

    def _read_speed(self):
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

    # ---- Drive interface --------------------------------------------------

    def set_wheel_speeds(self, cmd):
        meas_l, meas_r = self._read_speed()
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
# Clock
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
# Top-level entry.  Drop into `code.py` on the board.
# -----------------------------------------------------------------------------

def main(tunables=None):
    from algorithm import run, ReactiveController
    from planner import FloodFillPlanner
    from slam import ScanMatchSlam
    from tunables import Tunables

    if tunables is None:
        try:
            tunables = Tunables.from_json_file("/tunables.json")
        except (OSError, ValueError):
            tunables = Tunables()

    sensors = TcaVL53L0X()
    imu = XiaoIMU()
    drive = DriverN20(tunables)
    clock = MonotonicClock()

    # On hardware we use SLAM as the planner's pose source.  Encoder
    # odometry alone is not enough over a real run.
    # Goal cell is fixed at maze centre; cell size from tunables; start
    # cell is (0, 0) by convention.  Adapt these if your competition
    # maze places the start elsewhere.
    cols = 16
    rows = 16
    goal_cell = (cols // 2, rows // 2)
    planner = FloodFillPlanner(
        cols=cols, rows=rows, goal_cell=goal_cell,
        cell_size_m=tunables.planner_cell_size_m,
        turn_cost=tunables.planner_turn_cost,
        reverse_cost=tunables.planner_reverse_cost,
        unknown_cost=tunables.planner_unknown_cost,
        use_diagonals=tunables.planner_use_diagonals,
        diagonal_strict=tunables.planner_diagonal_strict,
    )
    # Start centred in cell (0, 0), facing N.
    s = tunables.planner_cell_size_m
    estimator = ScanMatchSlam(s / 2.0, s / 2.0, 1.5707963267948966,
                              planner.map, tunables)

    def pose_provider():
        return estimator.pose()
    # Bug #29 fix: planner.observe() uses the smooth pre-correction pose
    # so mm-scale SLAM corrections at cell boundaries don't flip the cell
    # index and poison the known map.
    def observation_pose_provider():
        return estimator.dead_reckoning_pose()

    if tunables.controller_mode == "path":
        from path_controller import PathController
        controller = PathController(
            tunables, planner=planner, pose_provider=pose_provider,
            observation_pose_provider=observation_pose_provider)
    else:
        controller = ReactiveController(
            tunables, planner=planner, pose_provider=pose_provider,
            observation_pose_provider=observation_pose_provider)

    # We bypass the standard `algorithm.run` so we can interleave the
    # estimator's `update` between the controller step and the wheel
    # command.  The structure mirrors `algorithm.run` otherwise.
    dt = 1.0 / tunables.loop_hz
    while True:
        reading = sensors.read()
        encoders = drive.read_encoders()
        imu_r = imu.read()
        # Update estimator from latest encoders + IMU + ToF.
        estimator.update(encoders[0], encoders[1], dt,
                         imu_reading=imu_r, reading=reading)
        # Controller sees the fused pose via pose_provider().
        cmd = controller.step(reading, encoders, dt, imu_reading=imu_r)
        drive.set_wheel_speeds(cmd)
        clock.sleep(dt)


if __name__ == "__main__":
    main()
