"""Bring-up test 07 -- motor + encoder polarity / wiring verification.

Drop-in CircuitPython script.  Copy to the XIAO as `code.py`.

!!! PROP THE ROBOT UP.                                                 !!!

This is the test that catches the four classic wiring mistakes before you
ever run the algorithm:

  1. Left/right swapped   -> commanding LEFT spins the RIGHT wheel.
  2. Motor polarity       -> "forward" spins a wheel backward.
  3. Encoder/motor mismatch-> driving LEFT increments the RIGHT counter.
  4. Encoder sign          -> wheel rolls forward but counts go down.

It drives each motor forward briefly and checks the matching encoder
counted UP.  Then it prints a PASS/FAIL verdict per wheel and, on failure,
the exact fix (which pins to swap in hardware/xiao_nrf52840.py).

The encoder here is single-channel (no direction), so "counts went up"
just means "the wheel turned".  Direction is inferred from the command.
For sign verification, watch the physical wheel during the FORWARD burst.

Pin map mirrors hardware/xiao_nrf52840.py.
"""

import time
import board
import pwmio
import countio

PIN_M1A, PIN_M1B = "D0", "D1"   # LEFT  (per the adapter)
PIN_M2A, PIN_M2B = "D2", "D3"   # RIGHT
PIN_ENC_LA, PIN_ENC_RA = "D6", "D7"
PWM_FREQ_HZ = 20000
DUTY = 0.45
BURST_S = 1.2
MIN_COUNTS = 20   # below this, the wheel effectively didn't move


def main():
    print("Motor/encoder polarity check -- PROP UP THE ROBOT.  3 s ...")
    time.sleep(3.0)

    def pwm(p):
        return pwmio.PWMOut(getattr(board, p), frequency=PWM_FREQ_HZ)

    m1a, m1b = pwm(PIN_M1A), pwm(PIN_M1B)
    m2a, m2b = pwm(PIN_M2A), pwm(PIN_M2B)
    enc_l = countio.Counter(getattr(board, PIN_ENC_LA))
    enc_r = countio.Counter(getattr(board, PIN_ENC_RA))

    def fwd(a, b):
        a.duty_cycle = int(DUTY * 65535)
        b.duty_cycle = 0

    def off(a, b):
        a.duty_cycle = 0
        b.duty_cycle = 0

    def stop():
        off(m1a, m1b)
        off(m2a, m2b)

    results = []
    try:
        for label, a, b, own_enc, other_enc, own_name, other_name in (
                ("LEFT", m1a, m1b, enc_l, enc_r, "left(D6)", "right(D7)"),
                ("RIGHT", m2a, m2b, enc_r, enc_l, "right(D7)", "left(D6)")):
            print("\nDriving {} motor forward {:.1f}s -- WATCH the wheel.".format(
                label, BURST_S))
            enc_l.reset()
            enc_r.reset()
            fwd(a, b)
            time.sleep(BURST_S)
            off(a, b)
            time.sleep(0.4)
            own = own_enc.count
            other = other_enc.count
            print("  {} counter = {:5d}   {} counter = {:5d}".format(
                own_name, own, other_name, other))

            ok = True
            if own < MIN_COUNTS and other >= MIN_COUNTS:
                print("  FAIL: the OTHER encoder moved -> motor/encoder "
                      "channels are crossed.")
                print("        Swap (M1A,M1B)<->(M2A,M2B) OR (ENC_LA<->ENC_RA).")
                ok = False
            elif own < MIN_COUNTS:
                print("  FAIL: this wheel did not turn -> check motor power "
                      "/ wiring / duty.")
                ok = False
            else:
                print("  PASS: {} motor drives {} encoder.".format(label, own_name))
            print("  Did the wheel roll the mouse's FORWARD way?  If not,"
                  " swap this motor's two pins.")
            results.append((label, ok))
        stop()

        print("\n----- verdict -----")
        for label, ok in results:
            print("  {:5s} : {}".format(label, "PASS" if ok else "CHECK WIRING"))
    finally:
        stop()


main()
