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

Board filesystem layout expected (copy these to CIRCUITPY root):
    code.py                       <- this file
    interfaces.py algorithm.py planner.py pose_fusion.py slam.py
    tunables.py                   <- the CircuitPython-portable core
    hardware/__init__.py
    hardware/xiao_nrf52840.py     <- the adapter (drivers + pin map)
    lib/  adafruit_tca9548a  adafruit_vl53l0x  adafruit_lsm6ds
    tunables.json                 <- optional saved profile (else defaults)

Status LED (on-board RGB, active-low):
    blue  blinking   waiting to start (countdown)
    green solid      EXPLORE in progress
    yellow solid     RETURN in progress
    cyan  blinking   SPEED run in progress
    green slow blink  DONE
    red   blinking   error / phase timed out (motors stopped)

SAFETY: the motors WILL drive.  On the bench, prop the robot up.  A
phase aborts (motors stop, red LED) if it can't reach its target within
the per-phase time budget, so it never runs away indefinitely.

CircuitPython-portable: no f-strings, plain classes.
"""

import time

from hardware.xiao_nrf52840 import (
    TcaVL53L0X, XiaoIMU, DriverN20, MonotonicClock)
from planner import FloodFillPlanner
from slam import ScanMatchSlam
from algorithm import ReactiveController
from tunables import Tunables


# -----------------------------------------------------------------------------
# Run configuration -- edit to match your maze / preferences.
# -----------------------------------------------------------------------------

COLS = 16
ROWS = 16

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
WATCHDOG_MIN_MOVE_M = 0.03
WATCHDOG_WINDOW_S = 6.0

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
    """Build a planner for `goal`.  If `shared_map` is given, adopt it so
    the accumulated walls carry across phases."""
    p = FloodFillPlanner(
        cols=COLS, rows=ROWS, goal_cell=goal,
        cell_size_m=tun.planner_cell_size_m,
        turn_cost=tun.planner_turn_cost,
        reverse_cost=tun.planner_reverse_cost,
        unknown_cost=tun.planner_unknown_cost,
        use_diagonals=tun.planner_use_diagonals,
        diagonal_strict=tun.planner_diagonal_strict,
    )
    if shared_map is not None:
        p.map = shared_map
        p._dirty = True
        p.replan()
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
              sensors, drive, imu, clock, tun, max_time_s, led, led_rgb):
    print("PHASE {}: target {} ...".format(name, sorted(target_cells)))
    dt = 1.0 / tun.loop_hz
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
        drive.set_wheel_speeds(cmd)

        x, y, _th = estimator.pose()
        cell = planner.pose_to_cell(x, y)

        # Reached?
        if cell in target_cells:
            dwell += 1
            if dwell >= REACH_DWELL_TICKS:
                drive.stop()
                print("  reached {} at t={:.1f}s".format(
                    cell, clock.now() - t0))
                return True
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
            return False
        # No-progress watchdog.
        if now - wd_t >= WATCHDOG_WINDOW_S:
            moved = ((x - wd_ref[0]) ** 2 + (y - wd_ref[1]) ** 2) ** 0.5
            if moved < WATCHDOG_MIN_MOVE_M:
                drive.stop()
                print("  STUCK (moved {:.3f} m in {:.0f}s)".format(
                    moved, WATCHDOG_WINDOW_S))
                return False
            wd_t = now
            wd_ref = (x, y)

        clock.sleep(dt)


# -----------------------------------------------------------------------------
# Main competition flow
# -----------------------------------------------------------------------------

def main():
    led = StatusLed()

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

    # One SLAM estimator + one KnownMap shared across every phase.
    explore_planner = make_planner(_goal_cell(), None, base_tun)
    shared_map = explore_planner.map
    estimator = ScanMatchSlam(sx, sy, START_THETA, shared_map, base_tun)

    center = _center_cells()
    start_set = frozenset([START_CELL])

    # --- Start countdown -------------------------------------------------
    print("Place the robot at the start.  Starting in {:.0f}s ...".format(
        START_COUNTDOWN_S))
    t_end = clock.now() + START_COUNTDOWN_S
    while clock.now() < t_end:
        on = int(clock.now() * 2) % 2 == 0
        led.set(False, False, on)   # blue blink
        clock.sleep(0.1)

    ok = True
    try:
        # --- EXPLORE -----------------------------------------------------
        if DO_EXPLORE:
            led.set(False, True, False)   # green
            ctrl = make_controller(explore_planner, estimator, base_tun)
            ok = run_phase("EXPLORE", center, ctrl, explore_planner,
                           estimator, sensors, drive, imu, clock, base_tun,
                           EXPLORE_TIME_S, led, (False, True, False))
            if not ok:
                raise RuntimeError("explore failed")

        # --- RETURN ------------------------------------------------------
        if ok and DO_RETURN:
            led.set(True, True, False)    # yellow
            ret_planner = make_planner(START_CELL, shared_map, base_tun)
            ctrl = make_controller(ret_planner, estimator, base_tun)
            ok = run_phase("RETURN", start_set, ctrl, ret_planner,
                           estimator, sensors, drive, imu, clock, base_tun,
                           RETURN_TIME_S, led, (True, True, False))
            if not ok:
                raise RuntimeError("return failed")

        # --- SPEED -------------------------------------------------------
        if ok and DO_SPEED:
            led.set(False, True, True)    # cyan
            speed_tun = Tunables.from_overrides(
                ["{}={}".format(k, v) for k, v in SPEED_OVERRIDES.items()],
                base=base_tun)
            speed_planner = make_planner(_goal_cell(), shared_map, speed_tun)
            ctrl = make_controller(speed_planner, estimator, speed_tun)
            ok = run_phase("SPEED", center, ctrl, speed_planner,
                           estimator, sensors, drive, imu, clock, speed_tun,
                           SPEED_TIME_S, led, (False, True, True))

        # --- Done --------------------------------------------------------
        drive.stop()
        if ok:
            print("RUN COMPLETE.")
            while True:
                on = int(clock.now()) % 2 == 0
                led.set(False, on, False)   # green slow blink
                clock.sleep(0.25)
        else:
            raise RuntimeError("speed run failed")

    except Exception as e:  # noqa: BLE001
        # Any failure -> motors off, red blink, surface the message.
        try:
            drive.stop()
        except Exception:  # noqa: BLE001
            pass
        print("ABORT:", e)
        while True:
            on = int(clock.now() * 3) % 2 == 0
            led.set(on, False, False)       # red blink
            clock.sleep(0.15)


main()
