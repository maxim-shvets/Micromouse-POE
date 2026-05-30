"""Bring-up test 03 -- DRV8833 motors (NO encoders needed).

Drop-in CircuitPython script.  Copy to the XIAO as `code.py`.

!!! PROP THE ROBOT UP so the wheels spin free, or remove the wheels.   !!!
!!! The motors WILL spin.  Keep fingers clear.                         !!!

Runs a sequence on each motor:
    left  forward (short burst)  -> coast
    left  reverse (short burst)  -> coast
    right forward                -> coast
    right reverse                -> coast
    both  forward                -> stop

Watch each wheel:
  - "forward" should roll the wheel in the mouse's forward direction.
  - If a wheel spins backward on "forward", swap that motor's two pins
    in the pin map (PIN_M1A<->PIN_M1B or PIN_M2A<->PIN_M2B), or flip the
    sign in software.
  - If the WRONG wheel moves, swap the (M1*, M2*) pin pairs.

Duty is capped low (0.45) and bursts are short (0.5 s) for safety.
Pin map mirrors hardware/xiao_nrf52840.py.
"""

import time
import board
import pwmio

PIN_M1A = "D0"   # left  IN1 (AIN1)
PIN_M1B = "D1"   # left  IN2 (AIN2)
PIN_M2A = "D2"   # right IN1 (BIN1)
PIN_M2B = "D3"   # right IN2 (BIN2)

PWM_FREQ_HZ = 20000
DUTY = 0.45          # safety cap for bring-up
BURST_S = 0.5


def pwm(pin):
    return pwmio.PWMOut(getattr(board, pin), frequency=PWM_FREQ_HZ)


def drive(a, b, duty):
    """duty in [-1, 1]; sign sets direction via the DRV8833 pin pair."""
    if duty >= 0.0:
        a.duty_cycle = int(min(1.0, duty) * 65535)
        b.duty_cycle = 0
    else:
        a.duty_cycle = 0
        b.duty_cycle = int(min(1.0, -duty) * 65535)


def main():
    print("DRV8833 motor test -- PROP UP THE ROBOT.  Starting in 3 s ...")
    time.sleep(3.0)

    m1a, m1b = pwm(PIN_M1A), pwm(PIN_M1B)
    m2a, m2b = pwm(PIN_M2A), pwm(PIN_M2B)

    def stop_all():
        for p in (m1a, m1b, m2a, m2b):
            p.duty_cycle = 0

    seq = [
        ("LEFT  forward", m1a, m1b, +DUTY),
        ("LEFT  reverse", m1a, m1b, -DUTY),
        ("RIGHT forward", m2a, m2b, +DUTY),
        ("RIGHT reverse", m2a, m2b, -DUTY),
    ]
    try:
        for label, a, b, duty in seq:
            print(" ", label)
            drive(a, b, duty)
            time.sleep(BURST_S)
            drive(a, b, 0.0)
            time.sleep(0.4)

        print("  BOTH forward")
        drive(m1a, m1b, +DUTY)
        drive(m2a, m2b, +DUTY)
        time.sleep(BURST_S)
        stop_all()
        print("done.  motors stopped.")
    finally:
        stop_all()


main()
