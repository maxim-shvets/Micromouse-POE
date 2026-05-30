"""Bring-up test 04 -- wheel encoders (countio).

Drop-in CircuitPython script.  Copy to the XIAO as `code.py`.

Two phases:
  1. MANUAL : motors OFF.  Spin each wheel by HAND one full turn and watch
              the counts.  One full wheel revolution should add about
              ENC_COUNTS_PER_REV counts on that channel.  Use this to
              CALIBRATE ENC_COUNTS_PER_REV for your specific motor/encoder.
  2. AUTO   : (optional) drives each motor briefly and prints counts so you
              can confirm the encoder channel matches the motor.  Prop the
              robot up first.  Set RUN_MOTORS = False to skip.

If a channel never changes -> check the encoder wiring (D6 / D7), the
encoder's power (3V3), and that it's a digital/hall output.

Pin map mirrors hardware/xiao_nrf52840.py.
"""

import time
import board
import countio

PIN_ENC_LA = "D6"
PIN_ENC_RA = "D7"
ENC_COUNTS_PER_REV = 700.0   # CALIBRATE THIS in phase 1

# Phase 2 motor confirmation (needs the robot propped up).
RUN_MOTORS = True
PIN_M1A, PIN_M1B = "D0", "D1"
PIN_M2A, PIN_M2B = "D2", "D3"
PWM_FREQ_HZ = 20000
DUTY = 0.45


def main():
    print("Encoder test")
    enc_l = countio.Counter(getattr(board, PIN_ENC_LA))
    enc_r = countio.Counter(getattr(board, PIN_ENC_RA))

    # --- Phase 1: manual spin -------------------------------------------
    print("\nPHASE 1: motors OFF.  Spin each wheel by hand.")
    print("  Watching counts for 15 s -- one wheel turn ~= {:.0f} counts.".format(
        ENC_COUNTS_PER_REV))
    enc_l.reset()
    enc_r.reset()
    t_end = time.monotonic() + 15.0
    last = -1.0
    while time.monotonic() < t_end:
        now = time.monotonic()
        if now - last >= 0.5:
            last = now
            print("  L counts = {:6d}   R counts = {:6d}".format(
                enc_l.count, enc_r.count))
        time.sleep(0.02)

    # --- Phase 2: motor-driven confirmation -----------------------------
    if not RUN_MOTORS:
        print("\nRUN_MOTORS=False -- skipping motor phase.  Done.")
        return

    import pwmio
    print("\nPHASE 2: PROP UP THE ROBOT.  Driving motors in 3 s ...")
    time.sleep(3.0)

    def pwm(p):
        return pwmio.PWMOut(getattr(board, p), frequency=PWM_FREQ_HZ)

    m1a, m1b = pwm(PIN_M1A), pwm(PIN_M1B)
    m2a, m2b = pwm(PIN_M2A), pwm(PIN_M2B)

    def drive(a, b, duty):
        a.duty_cycle = int(max(0.0, duty) * 65535)
        b.duty_cycle = int(max(0.0, -duty) * 65535)

    def stop():
        for p in (m1a, m1b, m2a, m2b):
            p.duty_cycle = 0

    try:
        for label, a, b, el, er in (
                ("LEFT motor", m1a, m1b, enc_l, enc_r),
                ("RIGHT motor", m2a, m2b, enc_r, enc_l)):
            print(" ", label, "forward 1.5 s")
            el.reset()
            er.reset()
            drive(a, b, DUTY)
            time.sleep(1.5)
            drive(a, b, 0.0)
            time.sleep(0.3)
            print("    its channel  = {:6d} counts".format(el.count))
            print("    other channel = {:6d} counts (should be ~0)".format(er.count))
        stop()
        print("done.")
    finally:
        stop()


main()
