"""FINAL RUN SCRIPT -- drop-in `code.py` for the XIAO nRF52840 Sense mouse.

Copy this file to the CIRCUITPY drive root as `code.py`.  It runs the full
competition flow on real hardware using the same control stack the sim is
validated against:

    EXPLORE  -> drive from the start cell to the centre, mapping walls
                with the planner + SLAM as it goes.
    RETURN   -> drive back to the start cell using the now-known map.
    SPEED    -> a faster run to the centre over the mapped maze.

Run it ONLY after the bring-up tests in hardware/tests/ all pass
(I2C scan, ToF, IMU, motors, encoders, polarity).

START / CONTROL OVER BLUETOOTH (no physical button needed):
    The XIAO's BLE radio advertises as "Micromouse".  Drive it from EITHER:
      - a laptop:  python3 tools/mouse_console.py   (needs `pip install
        bleak`; no phone app -- recommended for dev / tuning / logging), OR
      - a phone:   any BLE UART app (Bluefruit Connect -> UART, etc.)
    Send newline-terminated commands:
        go        run the full flow (explore -> return -> speed)
        explore / return / speed   run one phase
        stop      EMERGENCY STOP (abort, motors off)
        reset     forget the mapped maze
        status    report current cell
    Status is sent back so the controller shows progress.  See
    ble_control.py (robot side) + tools/mouse_console.py (laptop side).
    If the adafruit_ble library is absent, it falls back to a timed
    countdown start and runs the full flow once.

Board filesystem layout expected (copy these to CIRCUITPY root):
    code.py                       <- this file
    interfaces.py algorithm.py planner.py pose_fusion.py slam.py
    tunables.py                   <- the CircuitPython-portable core
    hardware/__init__.py
    hardware/xiao_nrf52840.py     <- the adapter (drivers + pin map)
    hardware/ble_control.py       <- BLE UART control
    lib/  adafruit_tca9548a  adafruit_vl53l0x  adafruit_lsm6ds  adafruit_ble
    tunables.json                 <- optional saved profile (else defaults)

Status LED (on-board RGB, active-low):
    blue  blinking   advertising, waiting for a BLE connection
    blue  solid      phone connected, waiting for a command
    green solid      EXPLORE in progress
    yellow solid     RETURN in progress
    cyan  solid      SPEED run in progress
    green slow blink  DONE (back to ready)
    red   blinking   error / phase aborted (motors stopped)

SAFETY: the motors WILL drive.  On the bench, prop the robot up.  A
phase aborts (motors stop, red LED) if it can't reach its target within
the per-phase time budget, so it never runs away indefinitely.

CircuitPython-portable: no f-strings, plain classes.
"""

import time
import gc
gc.collect()

from hardware.xiao_nrf52840 import (
    TcaVL53L0X, XiaoIMU, DriverN20, MonotonicClock)
from hardware.ble_control import BleControl
from planner import FloodFillPlanner
from slam import ScanMatchSlam
from algorithm import ReactiveController
from tunables import Tunables


# -----------------------------------------------------------------------------
# Run configuration -- edit to match your maze / preferences.
# -----------------------------------------------------------------------------

COLS = 4
ROWS = 4

# Start cell + initial heading (1.5708 rad = facing North / +y).
START_CELL = (0, 0)
START_THETA = 1.5707963267948796

# Phases to run, in order.  Disable any by setting False.
DO_EXPLORE = True
DO_RETURN = True
DO_SPEED = True

# Per-phase time budget (s).  A phase that overruns aborts safely.
EXPLORE_TIME_S = 240.0
RETURN_TIME_S = 120.0
SPEED_TIME_S = 120.0

# Countdown before the first move (s) -- time to place the robot + step back.
START_COUNTDOWN_S = 5.0

# How many consecutive ticks the estimated cell must sit in the target
# region before a phase is declared complete (debounces SLAM jitter).
REACH_DWELL_TICKS = 8

# No-progress watchdog: if the robot moves less than this (m) over
# WATCHDOG_WINDOW_S while commanding motion, abort the phase.
WATCHDOG_MIN_MOVE_M = 0.01
WATCHDOG_WINDOW_S = 30.0

# SPEED-phase tunable overrides layered on top of the base profile.
# Conservative "race-lite" by default -- bump cruise/max toward the race
# profile (2.0 / 5.0) once you trust the hardware.  controller_mode is
# forced to "cell" because the path-tracking controller is still WIP.
SPEED_OVERRIDES = {
    "cruise_speed_mps": 0.55,
    "max_speed_mps": 0.90,
    "max_wheel_accel_mps2": 10.0,
    "max_decel_mps2": 6.0,
    "planner_turn_cost": 0.4,
    "planner_use_diagonals": True,
    "controller_mode": "cell",
}


# -----------------------------------------------------------------------------
# Status LED (on-board RGB on the XIAO nRF52840; active-low).  All calls are
# defensive -- a board without these pins just gets no LED feedback.
# -----------------------------------------------------------------------------

class StatusLed(object):
    def __init__(self):
        self._r = self._g = self._b = None
        try:
            import board
            import digitalio
            for name, attr in (("_r", "LED_RED"),
                               ("_g", "LED_GREEN"),
                               ("_b", "LED_BLUE")):
                pin = getattr(board, attr, None)
                if pin is not None:
                    io = digitalio.DigitalInOut(pin)
                    io.direction = digitalio.Direction.OUTPUT
                    io.value = True   # active-low -> True = off
                    setattr(self, name, io)
        except Exception:  # noqa: BLE001
            pass

    def set(self, r, g, b):
        # active-low: value False = LED on
        if self._r is not None:
            self._r.value = not r
        if self._g is not None:
            self._g.value = not g
        if self._b is not None:
            self._b.value = not b

    def off(self):
        self.set(False, False, False)


# -----------------------------------------------------------------------------
# Build helpers
# -----------------------------------------------------------------------------

def _center_cells():
    gc = COLS // 2 - 1
    gr = ROWS // 2 - 1
    return frozenset([(gc, gr), (gc + 1, gr), (gc, gr + 1), (gc + 1, gr + 1)])


def _goal_cell():
    # A single centre cell for the flood-fill goal.
    return (COLS // 2 - 1, ROWS // 2 - 1)


def make_planner(goal, shared_map, tun):
    """Build a planner for `goal`.  Passes `shared_map` directly so no
    throwaway KnownMap is ever allocated."""
    gc.collect()
    p = FloodFillPlanner(
        cols=COLS, rows=ROWS, goal_cell=goal,
        cell_size_m=tun.planner_cell_size_m,
        turn_cost=tun.planner_turn_cost,
        reverse_cost=tun.planner_reverse_cost,
        unknown_cost=tun.planner_unknown_cost,
        use_diagonals=tun.planner_use_diagonals,
        diagonal_strict=tun.planner_diagonal_strict,
        existing_map=shared_map,
    )
    if shared_map is not None:
        p._dirty = True
        p.replan()
    gc.collect()
    return p


def make_controller(planner, estimator, tun):
    """Fresh controller (clean recovery state machine) for a phase.

    Built WITHOUT estimator= so the run loop owns the single estimator
    update per tick (avoids a double update)."""
    return ReactiveController(
        tun, planner=planner,
        pose_provider=estimator.pose,
        observation_pose_provider=estimator.dead_reckoning_pose)


# -----------------------------------------------------------------------------
# One phase: drive until the estimated cell reaches a target region.
# -----------------------------------------------------------------------------

def run_phase(name, target_cells, controller, planner, estimator,
              sensors, drive, imu, clock, tun, max_time_s, led, led_rgb, ble):
    """Drive until the estimated cell reaches `target_cells`.

    Returns one of: "reached", "timeout", "stuck", "stopped".  A BLE
    "stop" command aborts immediately ("stopped").
    """
    print("PHASE {}: target {} ...".format(name, sorted(target_cells)))
    ble.send(name + " ...")
    dt = 1.0 / tun.loop_hz

    # Warmup: run sensor + estimator updates with motors stopped for 10 ticks
    # so the controller's first live command is based on a settled pose and the
    # wheel speed history starts at zero (prevents initial backward lurch).
    for _ in range(10):
        reading = sensors.read()
        encoders = drive.read_encoders()
        imu_r = imu.read()
        estimator.update(encoders[0], encoders[1], dt,
                         imu_reading=imu_r, reading=reading)
        controller.step(reading, encoders, dt, imu_reading=imu_r)
        clock.sleep(dt)
    drive.stop()

    t0 = clock.now()
    dwell = 0

    # Watchdog state.
    wd_t = t0
    x0, y0, _ = estimator.pose()
    wd_ref = (x0, y0)

    blink = 0
    while True:
        reading = sensors.read()
        encoders = drive.read_encoders()
        imu_r = imu.read()

        # Single estimator update per tick, before the controller reads pose.
        estimator.update(encoders[0], encoders[1], dt,
                         imu_reading=imu_r, reading=reading)
        cmd = controller.step(reading, encoders, dt, imu_reading=imu_r)
        cmd = controller._rate_limit(cmd, dt)
        from interfaces import WheelSpeeds as _WS
        # Prevent backward TRANSLATION (robot centre moving backward) while
        # still allowing in-place pivoting (one wheel negative, one positive).
        mean_v = (cmd.left + cmd.right) * 0.5
        if mean_v < 0.0:
            diff = (cmd.right - cmd.left) * 0.5
            cmd = _WS(-diff, diff)   # pure pivot, zero net translation

        # Front-wall brake: if a wall is very close ahead, zero the forward
        # component and keep only the turning differential so the robot
        # pivots away instead of driving into the wall.
        if reading.front < 0.10:
            diff = (cmd.right - cmd.left) * 0.5
            cmd = _WS(-diff, diff)
        drive.set_wheel_speeds(cmd)

        x, y, _th = estimator.pose()
        cell = planner.pose_to_cell(x, y)

        # BLE control: emergency stop / status while running.
        for c in ble.poll():
            if c in ("stop", "x", "halt"):
                drive.stop()
                print("  STOP (ble)")
                ble.send("STOPPED in " + name)
                return "stopped"
            if c in ("status", "?"):
                ble.send("{} at {}".format(name, cell))

        # Reached?
        if cell in target_cells:
            dwell += 1
            if dwell >= REACH_DWELL_TICKS:
                drive.stop()
                t = clock.now() - t0
                print("  reached {} at t={:.1f}s".format(cell, t))
                ble.send("reached {} t={:.1f}s".format(cell, t))
                return "reached"
        else:
            dwell = 0

        # LED blink so you can see it's alive.
        blink += 1
        if (blink % 25) == 0:
            on = (blink // 25) % 2 == 0
            led.set(led_rgb[0] and on, led_rgb[1] and on, led_rgb[2] and on)

        now = clock.now()
        # Phase timeout.
        if now - t0 > max_time_s:
            drive.stop()
            print("  TIMEOUT after {:.0f}s".format(max_time_s))
            ble.send("TIMEOUT " + name)
            return "timeout"
        # No-progress watchdog.
        if now - wd_t >= WATCHDOG_WINDOW_S:
            moved = ((x - wd_ref[0]) ** 2 + (y - wd_ref[1]) ** 2) ** 0.5
            if moved < WATCHDOG_MIN_MOVE_M:
                drive.stop()
                print("  STUCK (moved {:.3f} m in {:.0f}s)".format(
                    moved, WATCHDOG_WINDOW_S))
                ble.send("STUCK " + name)
                return "stuck"
            wd_t = now
            wd_ref = (x, y)

        clock.sleep(dt)


# -----------------------------------------------------------------------------
# Main competition flow
# -----------------------------------------------------------------------------

_PHASE_RGB = {
    "EXPLORE": (False, True, False),    # green
    "RETURN": (True, True, False),      # yellow
    "SPEED": (False, True, True),       # cyan
}


def main():
    led = StatusLed()
    ble = BleControl()

    # --- Load tunables ---------------------------------------------------
    try:
        base_tun = Tunables.from_json_file("/tunables.json")
    except (OSError, ValueError):
        base_tun = Tunables()

    # --- Hardware --------------------------------------------------------
    sensors = TcaVL53L0X()
    imu = XiaoIMU()
    drive = DriverN20(base_tun)
    clock = MonotonicClock()

    s = base_tun.planner_cell_size_m
    sx = (START_CELL[0] + 0.5) * s
    sy = (START_CELL[1] + 0.5) * s
    center = _center_cells()
    start_set = frozenset([START_CELL])

    # One KnownMap + one SLAM estimator, persisted across commands so the
    # learned maze carries between explore / return / speed.  Held in a
    # dict so the nested helpers can rebuild them ("reset").
    base_planner = make_planner(_goal_cell(), None, base_tun)
    st = {"map": base_planner.map,
          "est": ScanMatchSlam(sx, sy, START_THETA, base_planner.map, base_tun)}

    def reset_pose():
        st["est"] = None          # free old estimator before allocating new one
        gc.collect()
        st["est"] = ScanMatchSlam(sx, sy, START_THETA, st["map"], base_tun)

    def reset_map():
        st["est"] = None
        st["map"] = None
        gc.collect()
        np_ = make_planner(_goal_cell(), None, base_tun)
        st["map"] = np_.map
        reset_pose()

    def speed_tun():
        return Tunables.from_overrides(
            ["{}={}".format(k, v) for k, v in SPEED_OVERRIDES.items()],
            base=base_tun)

    def run_one(name, goal, target, tun, max_t):
        gc.collect()
        planner = make_planner(goal, st["map"], tun)
        gc.collect()
        ctrl = make_controller(planner, st["est"], tun)
        rgb = _PHASE_RGB.get(name, (False, True, False))
        led.set(rgb[0], rgb[1], rgb[2])
        return run_phase(name, target, ctrl, planner, st["est"],
                         sensors, drive, imu, clock, tun, max_t, led, rgb, ble)

    def do_full():
        # Fresh pose origin each full run (robot is placed at start).
        reset_pose()
        if DO_EXPLORE:
            if run_one("EXPLORE", _goal_cell(), center, base_tun,
                       EXPLORE_TIME_S) != "reached":
                return False
        if DO_RETURN:
            if run_one("RETURN", START_CELL, start_set, base_tun,
                       RETURN_TIME_S) != "reached":
                return False
        if DO_SPEED:
            if run_one("SPEED", _goal_cell(), center, speed_tun(),
                       SPEED_TIME_S) != "reached":
                return False
        return True

    def done_blink():
        # green slow blink until the next command (BLE) / forever (no BLE).
        for _ in range(8):
            led.set(False, True, False)
            clock.sleep(0.2)
            led.set(False, False, False)
            clock.sleep(0.2)
            if ble.connected and ble.poll():
                return

    # --- No BLE library -> legacy countdown + a single full run ----------
    if not ble.available:
        print("BLE unavailable -> countdown start.")
        t_end = clock.now() + START_COUNTDOWN_S
        while clock.now() < t_end:
            on = int(clock.now() * 2) % 2 == 0
            led.set(False, False, on)       # blue blink
            clock.sleep(0.1)
        try:
            ok = do_full()
            drive.stop()
            print("RUN COMPLETE." if ok else "RUN ENDED (phase failed).")
        except Exception as e:  # noqa: BLE001
            try:
                drive.stop()
            except Exception:  # noqa: BLE001
                pass
            print("ABORT:", e)
        while True:
            led.set(False, True, False)
            clock.sleep(0.5)

    # --- BLE command loop ------------------------------------------------
    print("BLE ready. Advertising as 'Micromouse'. Send: "
          "go explore return speed stop reset status "
          "sensors enc motors mem")
    was_connected = False
    while True:
        ble.service()
        now_connected = ble.connected
        if now_connected and not was_connected:
            ble.send("Micromouse READY")
            ble.send("cmds: go explore return speed stop reset status")
        was_connected = now_connected

        # Ready indicator: blue solid when a phone is connected, slow blue
        # blink while still advertising / waiting.
        if now_connected:
            led.set(False, False, True)
        else:
            led.set(False, False, int(clock.now() * 2) % 2 == 0)

        for c in ble.poll():
            try:
                if c in ("go", "run", "start"):
                    ble.send("GO")
                    ok = do_full()
                    drive.stop()
                    ble.send("DONE" if ok else "ENDED")
                elif c == "explore":
                    reset_pose()
                    run_one("EXPLORE", _goal_cell(), center, base_tun,
                            EXPLORE_TIME_S)
                    drive.stop()
                elif c == "return":
                    run_one("RETURN", START_CELL, start_set, base_tun,
                            RETURN_TIME_S)
                    drive.stop()
                elif c == "speed":
                    run_one("SPEED", _goal_cell(), center, speed_tun(),
                            SPEED_TIME_S)
                    drive.stop()
                elif c == "reset":
                    reset_map()
                    ble.send("map cleared")
                elif c in ("status", "?"):
                    x, y, _t = st["est"].pose()
                    cc = max(0, min(COLS - 1, int(x / s)))
                    rr = max(0, min(ROWS - 1, int(y / s)))
                    ble.send("READY cell ({}, {})".format(cc, rr))
                elif c == "mem":
                    ble.send("free: {} bytes".format(gc.mem_free()))
                elif c in ("sensors", "tof"):
                    ble.send("taking 10 readings...")
                    readings = []
                    for _ in range(10):
                        r = sensors.read()
                        readings.append(r)
                        clock.sleep(0.05)
                    def avg(vals):
                        return sum(vals) / len(vals)
                    fronts = [r.front for r in readings]
                    lefts  = [r.left  for r in readings]
                    rights = [r.right for r in readings]
                    ble.send("front: {:.3f} m  (min {:.3f} max {:.3f})".format(
                        avg(fronts), min(fronts), max(fronts)))
                    ble.send("left : {:.3f} m  (min {:.3f} max {:.3f})".format(
                        avg(lefts), min(lefts), max(lefts)))
                    ble.send("right: {:.3f} m  (min {:.3f} max {:.3f})".format(
                        avg(rights), min(rights), max(rights)))
                    ble.send("1.2 m = no target in range")
                elif c in ("enc", "encoders"):
                    ble.send("spin wheels by hand for 3s...")
                    l0 = drive._enc_l.count
                    r0 = drive._enc_r.count
                    clock.sleep(3.0)
                    ble.send("L={} R={}".format(
                        drive._enc_l.count - l0,
                        drive._enc_r.count - r0))
                elif c in ("motors", "motortest"):
                    ble.send("motor test -- prop robot up!")
                    clock.sleep(1.0)
                    for pct in (30, 50, 75, 100):
                        ble.send("fwd {}%".format(pct))
                        drive.raw_drive(pct / 100.0, pct / 100.0)
                        clock.sleep(1.5)
                    drive.stop()
                    clock.sleep(0.5)
                    ble.send("reverse 50%")
                    drive.raw_drive(-0.5, -0.5)
                    clock.sleep(1.5)
                    drive.stop()
                    ble.send("motor test done")
                elif c in ("stop", "x", "halt"):
                    drive.stop()
                    ble.send("idle")
                else:
                    ble.send("? unknown: " + c)
            except Exception as e:  # noqa: BLE001
                try:
                    drive.stop()
                except Exception:  # noqa: BLE001
                    pass
                ble.send("ABORT " + str(e))
                print("ABORT:", e)
                # flash red briefly, then return to ready.
                for _ in range(6):
                    led.set(True, False, False)
                    clock.sleep(0.1)
                    led.set(False, False, False)
                    clock.sleep(0.1)

        clock.sleep(0.05)


main()
