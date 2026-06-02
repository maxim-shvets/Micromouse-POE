"""BLE-driven self-test suite -- "is everything green and ready?".

Drop-in CircuitPython script.  Copy to the XIAO as `code.py` for the
verification session (swap back to the real hardware/code.py to run).

It runs the component checks in sequence and reports PASS/FAIL for each
over the same BLE link the run script uses, so you drive the whole suite
from a laptop (tools/mouse_selftest.py) or any BLE UART app -- with an
emergency stop at any time.

Commands (newline-terminated, over BLE; also runs `check` automatically
if no BLE library is present):
    check        SAFE sensor suite: i2c, tof, imu, encoders, looprate
                 (no motion)
    motors       motor + encoder pulse test (WHEELS MOVE -- prop up!)
    test / all   check, then motors  (full verification)
    loop <n>     endurance: run the reactive drive loop <n> times
                 (WHEELS MOVE)
    stop / x     abort whatever's running, motors off
    status / ?   report last results
    help

Status lines: "RUN tof", "PASS tof F=0.42 L=0.18 R=0.19",
"FAIL imu az=2.1 (expect ~9.8)", and a final
"GREEN 5/5" or "RED 1 failure(s): imu".

CircuitPython-portable: no f-strings, plain functions.
"""

import time

from hardware.xiao_nrf52840 import (
    TcaVL53L0X, XiaoIMU, DriverN20, MonotonicClock,
    PIN_I2C_SDA, PIN_I2C_SCL)
from hardware.ble_control import BleControl
from tunables import Tunables
from interfaces import WheelSpeeds


# -----------------------------------------------------------------------------
# Individual checks.  Each returns (ok_bool, detail_str).  `ctx` is a dict
# carrying lazily-built hardware + the ble link + clock + tunables.
# -----------------------------------------------------------------------------

def check_i2c(ctx):
    import board
    import busio
    found_ext = []
    found_imu = []
    try:
        ext = busio.I2C(getattr(board, PIN_I2C_SCL), getattr(board, PIN_I2C_SDA))
        while not ext.try_lock():
            pass
        try:
            found_ext = ext.scan()
        finally:
            ext.unlock()
        ext.deinit()
    except Exception as e:  # noqa: BLE001
        return (False, "external bus error: " + str(e))
    try:
        import digitalio
        pwr = getattr(board, "IMU_PWR", None)
        if pwr is not None:
            en = digitalio.DigitalInOut(pwr)
            en.direction = digitalio.Direction.OUTPUT
            en.value = True
            time.sleep(0.05)
        try:
            imu_i2c = board.IMU_I2C()
        except AttributeError:
            scl = getattr(board, "IMU_SCL", None) or getattr(board, "P0_24")
            sda = getattr(board, "IMU_SDA", None) or getattr(board, "P0_25")
            imu_i2c = busio.I2C(scl, sda)
        while not imu_i2c.try_lock():
            pass
        try:
            found_imu = imu_i2c.scan()
        finally:
            imu_i2c.unlock()
    except Exception as e:  # noqa: BLE001
        return (False, "imu bus error: " + str(e))
    has_mux = 0x70 in found_ext
    has_imu = (0x6A in found_imu) or (0x6B in found_imu)
    detail = "mux={} imu={}".format("ok" if has_mux else "MISSING",
                                    "ok" if has_imu else "MISSING")
    return (has_mux and has_imu, detail)


def check_tof(ctx):
    try:
        tof = ctx.get("tof") or TcaVL53L0X()
        ctx["tof"] = tof
    except Exception as e:  # noqa: BLE001
        return (False, "init failed: " + str(e))
    try:
        r = tof.read()
    except Exception as e:  # noqa: BLE001
        return (False, "read failed: " + str(e))
    vals = (r.front, r.left, r.right)
    # Plausible = finite and within (0, max].  All three identical at max
    # often means a channel isn't wired -- warn but don't hard-fail.
    ok = all(v > 0.0 for v in vals)
    return (ok, "F={:.2f} L={:.2f} R={:.2f}".format(*vals))


def check_imu(ctx):
    try:
        imu = ctx.get("imu") or XiaoIMU()
        ctx["imu"] = imu
    except Exception as e:  # noqa: BLE001
        return (False, "init failed: " + str(e))
    try:
        s = imu.read()
    except Exception as e:  # noqa: BLE001
        return (False, "read failed: " + str(e))
    mag = (s.accel_x ** 2 + s.accel_y ** 2 + s.accel_z ** 2) ** 0.5
    # Stationary accel magnitude should be ~ gravity (9.8 m/s^2).
    ok = 7.0 < mag < 12.5
    return (ok, "|a|={:.1f} gz={:.2f}".format(mag, s.gyro_z))


def check_encoders(ctx):
    # Building the drive constructs the countio counters; success = wired.
    try:
        drive = ctx.get("drive") or DriverN20(ctx["tun"])
        ctx["drive"] = drive
    except Exception as e:  # noqa: BLE001
        return (False, "init failed: " + str(e))
    return (True, "counters initialised")


def check_looprate(ctx):
    try:
        tof = ctx.get("tof") or TcaVL53L0X()
        ctx["tof"] = tof
        imu = ctx.get("imu") or XiaoIMU()
        ctx["imu"] = imu
        drive = ctx.get("drive") or DriverN20(ctx["tun"])
        ctx["drive"] = drive
    except Exception as e:  # noqa: BLE001
        return (False, "init failed: " + str(e))
    clock = ctx["clock"]
    n = 0
    t0 = clock.now()
    while clock.now() - t0 < 1.5:
        tof.read()
        imu.read()
        drive.read_encoders()
        n += 1
    hz = n / (clock.now() - t0)
    # Below ~15 Hz the blocking ToF reads dominate -> sensor-I/O work needed.
    return (hz >= 15.0, "{:.0f} Hz sensor-read ceiling".format(hz))


def check_motors(ctx):
    """Pulse each wheel and confirm its OWN encoder moved (not the other).

    WHEELS MOVE -- the caller warns + counts down first.  Abortable by a
    BLE 'stop' between pulses.
    """
    ble = ctx["ble"]
    clock = ctx["clock"]
    tun = ctx["tun"]
    try:
        drive = ctx.get("drive") or DriverN20(tun)
        ctx["drive"] = drive
    except Exception as e:  # noqa: BLE001
        return (False, "init failed: " + str(e))

    dt = 1.0 / tun.loop_hz
    detail = []
    ok = True
    for side, name in ((0, "left"), (1, "right")):
        own = 0.0
        cross = 0.0
        n = 0
        t_end = clock.now() + 1.0
        while clock.now() < t_end:
            for c in ble.poll():
                if c in ("stop", "x", "halt"):
                    drive.stop()
                    return (False, "aborted")
            cmd = WheelSpeeds(0.15, 0.0) if side == 0 else WheelSpeeds(0.0, 0.15)
            drive.set_wheel_speeds(cmd)
            ml, mr = drive.read_encoders()
            own += abs(ml if side == 0 else mr)
            cross += abs(mr if side == 0 else ml)
            n += 1
            clock.sleep(dt)
        drive.stop()
        time.sleep(0.3)
        moved = own / n if n else 0.0
        other = cross / n if n else 0.0
        if moved < 0.02:
            ok = False
            detail.append(name + ":NO-MOTION")
        elif other > moved * 0.5:
            ok = False
            detail.append(name + ":CROSSED")
        else:
            detail.append(name + ":ok({:.2f})".format(moved))
    return (ok, " ".join(detail))


SAFE_CHECKS = (
    ("i2c", check_i2c),
    ("tof", check_tof),
    ("imu", check_imu),
    ("encoders", check_encoders),
    ("looprate", check_looprate),
)


# -----------------------------------------------------------------------------
# Runner
# -----------------------------------------------------------------------------

def run_checks(checks, ctx, ble):
    fails = []
    for name, fn in checks:
        for c in ble.poll():
            if c in ("stop", "x", "halt"):
                ble.send("ABORTED")
                return None
        ble.send("RUN " + name)
        try:
            ok, detail = fn(ctx)
        except Exception as e:  # noqa: BLE001
            ok, detail = False, "exception: " + str(e)
        tag = "PASS" if ok else "FAIL"
        ble.send("{} {} {}".format(tag, name, detail))
        print(tag, name, detail)
        if not ok:
            fails.append(name)
    total = len(checks)
    if not fails:
        ble.send("GREEN {}/{}".format(total, total))
    else:
        ble.send("RED {} failure(s): {}".format(len(fails), ",".join(fails)))
    return fails


def motor_warn_countdown(ble, clock, seconds):
    ble.send("MOTORS: prop the robot up! pulsing in {:.0f}s (send stop to abort)"
             .format(seconds))
    t_end = clock.now() + seconds
    while clock.now() < t_end:
        for c in ble.poll():
            if c in ("stop", "x", "halt"):
                return False
        clock.sleep(0.1)
    return True


def endurance_loop(ctx, ble, n):
    """Run the reactive drive loop (no planner) for ~20 s, `n` times.

    Exercises sensors + control + motors together to shake out intermittent
    faults.  WHEELS MOVE.  Abortable by 'stop'.
    """
    from algorithm import run, ReactiveController
    tun = ctx["tun"]
    clock = ctx["clock"]
    tof = ctx["tof"] = ctx.get("tof") or TcaVL53L0X()
    imu = ctx["imu"] = ctx.get("imu") or XiaoIMU()
    drive = ctx["drive"] = ctx.get("drive") or DriverN20(tun)
    if not motor_warn_countdown(ble, clock, 3.0):
        ble.send("ABORTED")
        return
    dt = 1.0 / tun.loop_hz
    for i in range(n):
        ble.send("LOOP {}/{}".format(i + 1, n))
        ctrl = ReactiveController(tun)
        t_end = clock.now() + 20.0
        aborted = False
        while clock.now() < t_end:
            for c in ble.poll():
                if c in ("stop", "x", "halt"):
                    aborted = True
                    break
            if aborted:
                break
            reading = tof.read()
            enc = drive.read_encoders()
            imu_r = imu.read()
            cmd = ctrl.step(reading, enc, dt, imu_reading=imu_r)
            cmd = ctrl._rate_limit(cmd, dt)
            drive.set_wheel_speeds(cmd)
            clock.sleep(dt)
        drive.stop()
        if aborted:
            ble.send("LOOP aborted")
            return
    ble.send("LOOP done {}/{}".format(n, n))


def main():
    ble = BleControl()
    tun = Tunables()
    clock = MonotonicClock()
    ctx = {"tun": tun, "clock": clock, "ble": ble}
    last = {"fails": None}

    # No BLE library -> just run the safe checks once over USB serial.
    if not ble.available:
        print("BLE unavailable -> running safe checks once.")
        run_checks(SAFE_CHECKS, ctx, ble)
        return

    print("BLE self-test ready. Advertising 'Micromouse'. "
          "Send: check motors test loop<n> stop status help")
    was_connected = False
    while True:
        ble.service()
        nc = ble.connected
        if nc and not was_connected:
            ble.send("SELFTEST READY")
            ble.send("cmds: check motors test loop<n> stop status help")
        was_connected = nc

        for c in ble.poll():
            try:
                if c == "check":
                    last["fails"] = run_checks(SAFE_CHECKS, ctx, ble)
                elif c == "motors":
                    if motor_warn_countdown(ble, clock, 3.0):
                        last["fails"] = run_checks((("motors", check_motors),),
                                                   ctx, ble)
                    else:
                        ble.send("ABORTED")
                elif c in ("test", "all"):
                    fails = run_checks(SAFE_CHECKS, ctx, ble)
                    if fails is not None and motor_warn_countdown(ble, clock, 3.0):
                        mf = run_checks((("motors", check_motors),), ctx, ble)
                        if mf:
                            fails = (fails or []) + mf
                    last["fails"] = fails
                elif c.startswith("loop"):
                    parts = c.split()
                    n = int(parts[1]) if len(parts) > 1 and parts[1].isdigit() else 1
                    endurance_loop(ctx, ble, n)
                elif c in ("stop", "x", "halt"):
                    try:
                        if "drive" in ctx:
                            ctx["drive"].stop()
                    except Exception:  # noqa: BLE001
                        pass
                    ble.send("idle")
                elif c in ("status", "?"):
                    f = last["fails"]
                    if f is None:
                        ble.send("no results yet")
                    elif not f:
                        ble.send("last: GREEN")
                    else:
                        ble.send("last: RED " + ",".join(f))
                elif c == "help":
                    ble.send("cmds: check motors test loop<n> stop status help")
                else:
                    ble.send("? unknown: " + c)
            except Exception as e:  # noqa: BLE001
                try:
                    if "drive" in ctx:
                        ctx["drive"].stop()
                except Exception:  # noqa: BLE001
                    pass
                ble.send("ERROR " + str(e))
                print("ERROR:", e)

        clock.sleep(0.05)


main()
