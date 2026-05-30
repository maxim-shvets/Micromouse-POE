"""Bring-up test 05 -- drive + measured wheel speed (open loop).

Drop-in CircuitPython script.  Copy to the XIAO as `code.py`.

!!! PROP THE ROBOT UP -- the wheels spin.                              !!!

Ramps each motor through a few duty levels and prints the measured wheel
speed (m/s) derived from the encoder counts.  Use this to:
  - confirm the encoder direction matches the commanded direction,
  - read off a rough duty -> speed curve (useful for seeding the PI gains),
  - sanity-check ENC_COUNTS_PER_REV + wheel diameter give a believable m/s.

Wheel speed = (delta_counts / counts_per_rev) * wheel_circumference / dt.

Pin map + constants mirror hardware/xiao_nrf52840.py.
"""

import time
import math
import board
import pwmio
import countio

PIN_M1A, PIN_M1B = "D0", "D1"   # left
PIN_M2A, PIN_M2B = "D2", "D3"   # right
PIN_ENC_LA, PIN_ENC_RA = "D6", "D7"

PWM_FREQ_HZ = 20000
ENC_COUNTS_PER_REV = 700.0
WHEEL_DIAMETER_M = 0.032
WHEEL_CIRC_M = WHEEL_DIAMETER_M * math.pi

DUTIES = (0.30, 0.45, 0.60, 0.80)
DWELL_S = 1.2


def main():
    print("Drive + measured-speed test -- PROP UP THE ROBOT.  3 s ...")
    time.sleep(3.0)

    def pwm(p):
        return pwmio.PWMOut(getattr(board, p), frequency=PWM_FREQ_HZ)

    m1a, m1b = pwm(PIN_M1A), pwm(PIN_M1B)
    m2a, m2b = pwm(PIN_M2A), pwm(PIN_M2B)
    enc_l = countio.Counter(getattr(board, PIN_ENC_LA))
    enc_r = countio.Counter(getattr(board, PIN_ENC_RA))

    def drive(a, b, duty):
        a.duty_cycle = int(max(0.0, duty) * 65535)
        b.duty_cycle = int(max(0.0, -duty) * 65535)

    def stop():
        for p in (m1a, m1b, m2a, m2b):
            p.duty_cycle = 0

    def measure(enc, dwell):
        enc.reset()
        t0 = time.monotonic()
        time.sleep(dwell)
        dt = time.monotonic() - t0
        counts = enc.count
        rev = counts / ENC_COUNTS_PER_REV
        speed = rev * WHEEL_CIRC_M / dt if dt > 0 else 0.0
        return counts, speed

    try:
        for label, a, b, enc in (
                ("LEFT", m1a, m1b, enc_l),
                ("RIGHT", m2a, m2b, enc_r)):
            print("\n{} motor duty -> speed:".format(label))
            for duty in DUTIES:
                drive(a, b, duty)
                counts, speed = measure(enc, DWELL_S)
                print("  duty {:.2f}  ->  {:5d} counts  {:.3f} m/s".format(
                    duty, counts, speed))
            drive(a, b, 0.0)
            time.sleep(0.4)
        stop()
        print("\ndone.")
    finally:
        stop()


main()
