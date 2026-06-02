"""BLE-driven calibration & tuning station -- onboard-sensor self-tune.

Drop-in CircuitPython script.  Copy to the XIAO as `code.py` for a
calibration session, then `save` the result and swap back to the real
hardware/code.py.

TWO PARTS, both over BLE (drive from tools/mouse_calibrate.py or the
console):

  PART A -- AUTOMATIC (code-led, uses the onboard IMU + ToF + encoders):
    cal gyro       still: gyro-z bias + IMU noise + mounting tilt
                   -> imu_bias_gyro_z_rps, imu_noise_gyro_rps,
                      imu_noise_accel_mps2
    cal wheelbase  spin in place; the GYRO is ground truth for the angle
                   turned, so wheel_base_m = integral(vR - vL) dt / angle
                   -> wheel_base_m            (WHEELS MOVE)
    cal encoder    drive at a wall; the FRONT ToF is ground truth for the
                   distance covered, so it corrects the encoder distance
                   scale -> wheel_diameter_m  (WHEELS MOVE, needs a wall)
    cal            gyro, then (after a warning) wheelbase + encoder

  PART B -- MANUAL (you + your partner, human judgement):
    tune <key>=<value>   set a tunable live (e.g. tune cruise_speed_mps=0.4)
    show                 list tunables that differ from defaults
    drive                reactive drive until a wall (watch front_stop /
                         cruise / wall-centering)             (WHEELS MOVE)
    spin                 rotate in place (watch turn speed)   (WHEELS MOVE)
    straight             drive straight ~0.5 m, report heading drift (trim)
    save                 write the current tunables to /tunables.json
    stop / x             abort, motors off
    status / ?           summary
    help

Each calibrated value is also emitted as a machine-readable
"SET <key>=<value>" line so tools/mouse_calibrate.py can capture them.

SAFETY: motion routines warn + count down and poll `stop` every tick.
Prop the robot up for wheelbase/drive/spin; for `cal encoder` put it on
the floor facing a flat wall ~0.3-0.5 m away.

CircuitPython-portable: no f-strings, plain functions.
"""

import math
import time

from hardware.xiao_nrf52840 import (
    TcaVL53L0X, XiaoIMU, DriverN20, MonotonicClock)
from hardware.ble_control import BleControl
from tunables import Tunables
from interfaces import WheelSpeeds


# Tunables whose value is baked into DriverN20 at construction -> changing
# them needs the drive rebuilt before they take effect.
_DRIVE_KEYS = ("encoder_kp", "encoder_ki", "loop_hz", "motor_duty_cap",
               "wheel_diameter_m")


# -----------------------------------------------------------------------------
# Shared context: lazily-built hardware + the working Tunables.
# -----------------------------------------------------------------------------

def _hw(ctx, key):
    if key in ctx:
        return ctx[key]
    if key == "tof":
        ctx[key] = TcaVL53L0X()
    elif key == "imu":
        ctx[key] = XiaoIMU()
    elif key == "drive":
        ctx[key] = DriverN20(ctx["tun"])
    return ctx[key]


def _rebuild_drive(ctx):
    # Stop + drop the old drive so the new tunables take effect.
    try:
        if "drive" in ctx:
            ctx["drive"].stop()
    except Exception:  # noqa: BLE001
        pass
    ctx.pop("drive", None)


def _set(ctx, key, value):
    """Apply a tunable to the working set and emit a machine + human line."""
    ctx["tun"] = Tunables.from_overrides(
        ["{}={}".format(key, value)], base=ctx["tun"])
    ctx["ble"].send("SET {}={}".format(key, value))
    if key in _DRIVE_KEYS:
        _rebuild_drive(ctx)


# -----------------------------------------------------------------------------
# PART A -- automatic calibration
# -----------------------------------------------------------------------------

def cal_gyro(ctx):
    """Still: gyro-z bias, IMU noise, mounting tilt.  No motion."""
    ble = ctx["ble"]
    clock = ctx["clock"]
    imu = _hw(ctx, "imu")
    ble.send("CAL gyro: hold STILL + flat 3s ...")
    n = 0
    sgz = 0.0
    sgz2 = 0.0
    sam = 0.0
    sam2 = 0.0
    saz = 0.0
    t_end = clock.now() + 3.0
    while clock.now() < t_end:
        s = imu.read()
        gz = s.gyro_z
        am = (s.accel_x ** 2 + s.accel_y ** 2 + s.accel_z ** 2) ** 0.5
        sgz += gz
        sgz2 += gz * gz
        sam += am
        sam2 += am * am
        saz += s.accel_z
        n += 1
        clock.sleep(0.01)
    if n < 5:
        return (False, "no samples")
    bias = sgz / n
    var_g = max(0.0, sgz2 / n - bias * bias)
    noise_g = var_g ** 0.5
    mean_a = sam / n
    var_a = max(0.0, sam2 / n - mean_a * mean_a)
    noise_a = var_a ** 0.5
    # Tilt from vertical: az/|a| = cos(tilt).
    cos_t = (saz / n) / mean_a if mean_a > 0 else 1.0
    cos_t = max(-1.0, min(1.0, cos_t))
    tilt_deg = math.degrees(math.acos(cos_t))
    _set(ctx, "imu_bias_gyro_z_rps", round(bias, 6))
    _set(ctx, "imu_noise_gyro_rps", round(max(noise_g, 1.0e-4), 6))
    _set(ctx, "imu_noise_accel_mps2", round(max(noise_a, 1.0e-3), 4))
    ble.send("CAL gyro: bias={:.4f} rad/s noise={:.4f} |a|={:.2f} tilt={:.1f}deg"
             .format(bias, noise_g, mean_a, tilt_deg))
    if tilt_deg > 6.0:
        ble.send("WARN IMU tilt {:.1f}deg -- check mounting/level".format(tilt_deg))
    return (True, "bias={:.4f}".format(bias))


def cal_wheelbase(ctx):
    """Spin in place; gyro angle is ground truth -> wheel_base_m.

    omega = (vR - vL)/L, so integrate:  L = integral(vR - vL) dt / angle.
    WHEELS MOVE.
    """
    ble = ctx["ble"]
    clock = ctx["clock"]
    tun = ctx["tun"]
    imu = _hw(ctx, "imu")
    drive = _hw(ctx, "drive")
    if not _warn(ble, clock, "wheelbase (spins in place)", 3.0):
        return (False, "aborted")
    dt = 1.0 / tun.loop_hz
    v = tun.turn_speed_mps
    angle = 0.0
    diff = 0.0
    t_end = clock.now() + 4.0
    while clock.now() < t_end and abs(angle) < 6.0 * math.pi:
        for c in ble.poll():
            if c in ("stop", "x", "halt"):
                drive.stop()
                return (False, "aborted")
        drive.set_wheel_speeds(WheelSpeeds(-v, v))   # CCW spin
        ml, mr = drive.read_encoders()
        s = imu.read()
        angle += s.gyro_z * dt
        diff += (mr - ml) * dt
        clock.sleep(dt)
    drive.stop()
    if abs(angle) < math.pi:
        return (False, "spin too small (gyro angle {:.2f} rad)".format(angle))
    wb = diff / angle
    if not (0.02 < wb < 0.30):
        ble.send("CAL wheelbase: implausible {:.4f} m -- check spin/encoders"
                 .format(wb))
        return (False, "implausible {:.4f}".format(wb))
    old = tun.wheel_base_m
    _set(ctx, "wheel_base_m", round(wb, 5))
    ble.send("CAL wheelbase: {:.4f} m (was {:.4f}) over {:.1f} turns"
             .format(wb, old, abs(angle) / (2 * math.pi)))
    return (True, "{:.4f} m".format(wb))


def cal_encoder(ctx):
    """Drive at a wall; front ToF is ground truth for distance -> scale
    wheel_diameter_m.  WHEELS MOVE, needs a flat wall ahead."""
    ble = ctx["ble"]
    clock = ctx["clock"]
    tun = ctx["tun"]
    tof = _hw(ctx, "tof")
    drive = _hw(ctx, "drive")
    start = tof.read().front
    if start > 1.0 or start < (tun.front_stop_m + 0.15):
        return (False, "need a wall ~0.3-0.6 m ahead (saw {:.2f} m)".format(start))
    if not _warn(ble, clock, "encoder (drives forward at the wall)", 3.0):
        return (False, "aborted")
    dt = 1.0 / tun.loop_hz
    v = min(0.15, tun.cruise_speed_mps)
    enc = 0.0
    stop_at = tun.front_stop_m + 0.05
    t_end = clock.now() + 8.0
    front = start
    while clock.now() < t_end:
        for c in ble.poll():
            if c in ("stop", "x", "halt"):
                drive.stop()
                return (False, "aborted")
        front = tof.read().front
        if front <= stop_at or enc >= 0.30:
            break
        drive.set_wheel_speeds(WheelSpeeds(v, v))
        ml, mr = drive.read_encoders()
        enc += 0.5 * (ml + mr) * dt
        clock.sleep(dt)
    drive.stop()
    actual = start - front
    if enc < 0.03 or actual < 0.03:
        return (False, "too little motion (enc={:.3f} tof={:.3f})".format(enc, actual))
    scale = actual / enc
    new_d = tun.wheel_diameter_m * scale
    if not (0.5 < scale < 2.0):
        ble.send("CAL encoder: scale {:.3f} out of range -- check wall/straightness"
                 .format(scale))
        return (False, "scale {:.3f}".format(scale))
    old = tun.wheel_diameter_m
    _set(ctx, "wheel_diameter_m", round(new_d, 5))
    ble.send("CAL encoder: scale {:.3f} -> wheel_diameter {:.4f} m (was {:.4f})"
             .format(scale, new_d, old))
    return (True, "scale {:.3f}".format(scale))


def cal_all(ctx):
    ble = ctx["ble"]
    clock = ctx["clock"]
    ok1, _ = cal_gyro(ctx)
    ble.send("--- motion calibrations next ---")
    ok2 = ok3 = False
    if _warn(ble, clock, "wheelbase + encoder", 3.0):
        ok2, _ = cal_wheelbase(ctx)
        ok3, _ = cal_encoder(ctx)
    n = (1 if ok1 else 0) + (1 if ok2 else 0) + (1 if ok3 else 0)
    ble.send("CAL done {}/3.  Send 'save' to persist, 'show' to review.".format(n))


# -----------------------------------------------------------------------------
# PART B -- manual tuning behaviours (rebuild a fresh controller each run)
# -----------------------------------------------------------------------------

def behaviour_drive(ctx, secs=15.0):
    """Reactive forward-until-wall (no planner).  Watch front_stop / cruise /
    wall-centering and `tune` between runs."""
    from algorithm import ReactiveController
    ble = ctx["ble"]
    clock = ctx["clock"]
    tun = ctx["tun"]
    tof = _hw(ctx, "tof")
    imu = _hw(ctx, "imu")
    drive = _hw(ctx, "drive")
    if not _warn(ble, clock, "drive (reactive, until wall)", 3.0):
        return
    ctrl = ReactiveController(tun)
    dt = 1.0 / tun.loop_hz
    t_end = clock.now() + secs
    while clock.now() < t_end:
        for c in ble.poll():
            if c in ("stop", "x", "halt"):
                drive.stop()
                ble.send("drive stopped")
                return
        reading = tof.read()
        enc = drive.read_encoders()
        cmd = ctrl.step(reading, enc, dt, imu_reading=imu.read())
        cmd = ctrl._rate_limit(cmd, dt)
        drive.set_wheel_speeds(cmd)
        clock.sleep(dt)
    drive.stop()
    ble.send("drive done")


def behaviour_spin(ctx, secs=4.0):
    ble = ctx["ble"]
    clock = ctx["clock"]
    tun = ctx["tun"]
    drive = _hw(ctx, "drive")
    if not _warn(ble, clock, "spin in place", 3.0):
        return
    dt = 1.0 / tun.loop_hz
    v = tun.turn_speed_mps
    t_end = clock.now() + secs
    while clock.now() < t_end:
        for c in ble.poll():
            if c in ("stop", "x", "halt"):
                drive.stop()
                ble.send("spin stopped")
                return
        drive.set_wheel_speeds(WheelSpeeds(-v, v))
        clock.sleep(dt)
    drive.stop()
    ble.send("spin done")


def behaviour_straight(ctx, dist=0.5):
    """Drive straight ~dist m at cruise; report the heading drift the gyro
    saw (a proxy for left/right wheel-scale imbalance)."""
    ble = ctx["ble"]
    clock = ctx["clock"]
    tun = ctx["tun"]
    imu = _hw(ctx, "imu")
    drive = _hw(ctx, "drive")
    if not _warn(ble, clock, "straight {:.1f} m".format(dist), 3.0):
        return
    dt = 1.0 / tun.loop_hz
    v = min(0.25, tun.cruise_speed_mps)
    enc = 0.0
    yaw = 0.0
    t_end = clock.now() + 10.0
    while clock.now() < t_end and enc < dist:
        for c in ble.poll():
            if c in ("stop", "x", "halt"):
                drive.stop()
                ble.send("straight stopped")
                return
        drive.set_wheel_speeds(WheelSpeeds(v, v))
        ml, mr = drive.read_encoders()
        enc += 0.5 * (ml + mr) * dt
        yaw += imu.read().gyro_z * dt
        clock.sleep(dt)
    drive.stop()
    drift = math.degrees(yaw) / enc if enc > 0 else 0.0
    ble.send("straight: {:.2f} m, heading drift {:.1f} deg/m (0 = true)"
             .format(enc, drift))


# -----------------------------------------------------------------------------
# Helpers
# -----------------------------------------------------------------------------

def _warn(ble, clock, what, secs):
    ble.send("MOTION: {} in {:.0f}s (send stop to abort)".format(what, secs))
    t_end = clock.now() + secs
    while clock.now() < t_end:
        for c in ble.poll():
            if c in ("stop", "x", "halt"):
                return False
        clock.sleep(0.1)
    return True


# -----------------------------------------------------------------------------
# Main BLE command loop
# -----------------------------------------------------------------------------

def main():
    ble = BleControl()
    clock = MonotonicClock()
    # Start from a saved profile if present, else defaults.
    try:
        tun = Tunables.from_json_file("/tunables.json")
    except (OSError, ValueError):
        tun = Tunables()
    ctx = {"tun": tun, "clock": clock, "ble": ble}

    if not ble.available:
        print("BLE unavailable -> running 'cal gyro' once.")
        cal_gyro(ctx)
        return

    print("Calibration station ready. Advertising 'Micromouse'. Send: "
          "cal | cal gyro|wheelbase|encoder | tune k=v | show save | "
          "drive spin straight | stop help")
    was = False
    while True:
        ble.service()
        nc = ble.connected
        if nc and not was:
            ble.send("CALIBRATE READY")
            ble.send("cmds: cal | tune k=v | show save | drive spin straight | stop")
        was = nc

        for c in ble.poll():
            try:
                parts = c.split()
                head = parts[0] if parts else ""
                if head == "cal" and len(parts) == 1:
                    cal_all(ctx)
                elif head == "cal" and parts[1] == "gyro":
                    cal_gyro(ctx)
                elif head == "cal" and parts[1] == "wheelbase":
                    cal_wheelbase(ctx)
                elif head == "cal" and parts[1] == "encoder":
                    cal_encoder(ctx)
                elif head == "tune" and len(parts) >= 2 and "=" in parts[1]:
                    k, _, v = parts[1].partition("=")
                    try:
                        _set(ctx, k, v)
                    except (KeyError, ValueError) as e:
                        ble.send("bad tune: " + str(e))
                elif head == "show":
                    d = ctx["tun"].diff()
                    if not d:
                        ble.send("tunables: all defaults")
                    else:
                        for k in sorted(d):
                            ble.send("  {} = {}".format(k, d[k]))
                elif head == "save":
                    ctx["tun"].to_json_file("/tunables.json")
                    ble.send("saved /tunables.json")
                elif head == "drive":
                    behaviour_drive(ctx)
                elif head == "spin":
                    behaviour_spin(ctx)
                elif head == "straight":
                    behaviour_straight(ctx)
                elif head in ("stop", "x", "halt"):
                    try:
                        if "drive" in ctx:
                            ctx["drive"].stop()
                    except Exception:  # noqa: BLE001
                        pass
                    ble.send("idle")
                elif head in ("status", "?"):
                    ble.send("{} tunables differ from default".format(
                        len(ctx["tun"].diff())))
                elif head == "help":
                    ble.send("cmds: cal | tune k=v | show save | "
                             "drive spin straight | stop")
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
