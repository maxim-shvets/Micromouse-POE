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

# External I2C bus (D4=SDA / D5=SCL on XIAO Sense).  Shared by all three
# VL53L0X ToF sensors via TCA9548A multiplexer.
PIN_I2C_SDA = "D4"
PIN_I2C_SCL = "D5"

# VL53L0X XSHUT pins -- pulled low to reset/disable individual sensors
# during I2C address assignment at startup.
PIN_TOF_XSHUT_RIGHT  = "D2"
PIN_TOF_XSHUT_MIDDLE = "D3"
PIN_TOF_XSHUT_LEFT   = "D6"

# Motor driver pins -- DRV8833 IN1/IN2/IN3/IN4.  Each motor uses a pair:
# one pin holds at 0 while the other PWMs to drive forward, swap for
# reverse.  The XIAO has hardware PWM on most pins.
PIN_M1A = "D10"  # left motor  IN1 (AIN1 on DRV8833)
PIN_M1B = "D9"   # left motor  IN2 (AIN2)
PIN_M2A = "D8"   # right motor IN3 (BIN1)
PIN_M2B = "D7"   # right motor IN4 (BIN2)

# Set to True to flip a motor's forward direction without rewiring.
# If the robot spins in place instead of going straight, flip one of these.
MOTOR_LEFT_INVERT  = False
MOTOR_RIGHT_INVERT = True

# Encoder inputs.  Use single-channel hall + sign-from-command for v1
# (quadrature support would need two countio.Counter per wheel and a
# direction-inference state machine).
PIN_ENC_RA = "D0"  # right motor encoder
PIN_ENC_LA = "D1"  # left motor encoder

# I2C addresses assigned to each VL53L0X during XSHUT init sequence.
# All three start at the factory default (0x29); we wake them one at a
# time and reprogram each to a unique address before enabling the next.
TOF_ADDR_RIGHT  = 0x2A
TOF_ADDR_MIDDLE = 0x2B
TOF_ADDR_LEFT   = 0x2C

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
    import adafruit_vl53l0x
    # On-board IMU on XIAO Sense.  LSM6DS3TR-C is a register-compatible
    # variant of LSM6DS3; adafruit_lsm6ds.lsm6ds3trc is the right driver.
    import adafruit_lsm6ds.lsm6ds3trc as _lsm6
    return {
        "board": board, "busio": busio, "digitalio": digitalio,
        "pwmio": pwmio, "countio": countio, "time": _time,
        "vl53l0x": adafruit_vl53l0x, "lsm6": _lsm6,
    }


# -----------------------------------------------------------------------------
# Sensors -- ToF triplet via XSHUT address assignment
# -----------------------------------------------------------------------------

class TcaVL53L0X(object):
    """3-channel VL53L0X on a shared I2C bus, addressed via XSHUT pins.

    All three sensors default to address 0x29.  Init sequence:
      1. Pull all XSHUT pins low -- every sensor enters reset (bus clear).
      2. Release one XSHUT, wait for the sensor to boot, reassign its
         address, then move on to the next.
    After init, all three sit permanently on the bus at distinct addresses
    and are read directly with no mux chip required.
    """

    def __init__(self, mods=None):
        if mods is None:
            mods = _import_circuitpython()
        self._mods = mods
        board    = mods["board"]
        busio    = mods["busio"]
        digitalio = mods["digitalio"]
        vl53     = mods["vl53l0x"]
        _time    = mods["time"]

        def _xshut(pin_name):
            io = digitalio.DigitalInOut(getattr(board, pin_name))
            io.direction = digitalio.Direction.OUTPUT
            io.value = False   # low = sensor held in reset
            return io

        # Init I2C first while sensors are still active (pull-ups live),
        # then pull XSHUT lines low to begin the address-assignment sequence.
        # Use bitbangio to bypass CircuitPython 10.x hardware pull-up check.
        sda = getattr(board, PIN_I2C_SDA)
        scl = getattr(board, PIN_I2C_SCL)
        import bitbangio
        self._i2c = bitbangio.I2C(scl, sda)

        xshut_right  = _xshut(PIN_TOF_XSHUT_RIGHT)
        xshut_middle = _xshut(PIN_TOF_XSHUT_MIDDLE)
        xshut_left   = _xshut(PIN_TOF_XSHUT_LEFT)
        _time.sleep(0.01)  # let all sensors enter reset before waking them one by one

        # Wake right, reassign, then leave it running at TOF_ADDR_RIGHT.
        xshut_right.value = True
        _time.sleep(0.01)
        self.right = vl53.VL53L0X(self._i2c)
        self.right.set_address(TOF_ADDR_RIGHT)

        # Wake middle (still at factory 0x29 -- no conflict now).
        xshut_middle.value = True
        _time.sleep(0.01)
        self.middle = vl53.VL53L0X(self._i2c)
        self.middle.set_address(TOF_ADDR_MIDDLE)

        # Wake left.
        xshut_left.value = True
        _time.sleep(0.01)
        self.left = vl53.VL53L0X(self._i2c)
        self.left.set_address(TOF_ADDR_LEFT)

        # ~33 ms timing budget = max 30 Hz, plenty for the control loop.
        for sensor in (self.right, self.middle, self.left):
            sensor.measurement_timing_budget = 33000

        # Load calibration if available; fall back to identity per sensor.
        self._cal = {"right":  (1.0, 0.0),
                     "middle": (1.0, 0.0),
                     "left":   (1.0, 0.0)}
        try:
            import json
            with open("/tof_cal.json") as _f:
                _d = json.load(_f)
            for _k in ("right", "middle", "left"):
                if _k in _d:
                    self._cal[_k] = (_d[_k]["slope"], _d[_k]["offset"])
        except (OSError, ValueError, KeyError):
            pass   # no calibration file -- identity is fine

    def read(self):
        from interfaces import Reading

        def _to_m(raw_mm, slope, offset):
            corrected = slope * raw_mm + offset
            d = corrected / 1000.0
            if d <= 0.0 or d > 1.2:
                return 1.2
            return d

        rs, ro = self._cal["right"]
        ms, mo = self._cal["middle"]
        ls, lo = self._cal["left"]
        return Reading(
            front=_to_m(self.middle.range, ms, mo),
            left=_to_m(self.left.range,   ls, lo),
            right=_to_m(self.right.range,  rs, ro),
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
        board    = mods["board"]
        busio    = mods["busio"]
        digitalio = mods["digitalio"]
        _time    = mods["time"]

        # IMU_PWR must be driven high before the internal I2C bus has pull-ups.
        imu_pwr = digitalio.DigitalInOut(board.IMU_PWR)
        imu_pwr.direction = digitalio.Direction.OUTPUT
        imu_pwr.value = True
        _time.sleep(0.05)  # allow rail to stabilise

        self._i2c = busio.I2C(board.IMU_SCL, board.IMU_SDA)
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
        cap = tunables.motor_duty_cap
        dband = getattr(tunables, "motor_duty_min", 0.0)
        self._wc_l = WheelController(
            tunables.encoder_kp, tunables.encoder_ki,
            duty_min=-cap, duty_max=cap, duty_deadband=dband)
        self._wc_r = WheelController(
            tunables.encoder_kp, tunables.encoder_ki,
            duty_min=-cap, duty_max=cap, duty_deadband=dband)

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
        self._set_pwm(self._m1a, self._m1b, -duty_l if MOTOR_LEFT_INVERT  else duty_l)
        self._set_pwm(self._m2a, self._m2b, -duty_r if MOTOR_RIGHT_INVERT else duty_r)
        self._last_meas = (meas_l, meas_r)

    def read_encoders(self):
        return self._last_meas

    def raw_drive(self, duty_l, duty_r):
        """Drive both motors at a fixed duty (-1.0 to 1.0), bypassing PID.
        Respects MOTOR_*_INVERT.  Use for bench testing only."""
        self._set_pwm(self._m1a, self._m1b,
                      -duty_l if MOTOR_LEFT_INVERT  else duty_l)
        self._set_pwm(self._m2a, self._m2b,
                      -duty_r if MOTOR_RIGHT_INVERT else duty_r)

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
