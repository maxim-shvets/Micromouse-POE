"""Bring-up test 01 -- ToF triplet (3x VL53L0X via TCA9548A mux).

Drop-in CircuitPython script.  Copy to the XIAO as `code.py`.

Continuously prints the three ranges (front / left / right) in mm.  Wave
your hand in front of each sensor in turn to confirm the right channel
moves -- this also verifies the TCA_CHAN_* mapping matches your harness.

Expected:
    - With nothing in front: ~max range (often 1200-2000 mm or "out of range")
    - Hand at ~10 cm: ~100 mm on that channel only

Needs in /lib:  adafruit_tca9548a, adafruit_vl53l0x
Pin map mirrors hardware/xiao_nrf52840.py.
"""

import time
import board
import busio
import adafruit_tca9548a
import adafruit_vl53l0x

PIN_I2C_SDA = "D4"
PIN_I2C_SCL = "D5"

TCA_CHAN_FRONT = 0
TCA_CHAN_LEFT = 1
TCA_CHAN_RIGHT = 2

# 33 ms timing budget = ~30 Hz.  Lower = faster but noisier/shorter range.
TIMING_BUDGET_US = 33000


def main():
    print("ToF triplet test (front/left/right)")
    sda = getattr(board, PIN_I2C_SDA)
    scl = getattr(board, PIN_I2C_SCL)
    i2c = busio.I2C(scl, sda)
    mux = adafruit_tca9548a.TCA9548A(i2c)

    front = adafruit_vl53l0x.VL53L0X(mux[TCA_CHAN_FRONT])
    left = adafruit_vl53l0x.VL53L0X(mux[TCA_CHAN_LEFT])
    right = adafruit_vl53l0x.VL53L0X(mux[TCA_CHAN_RIGHT])
    for s in (front, left, right):
        s.measurement_timing_budget = TIMING_BUDGET_US
    print("3 sensors initialised.  Ctrl-C to stop.\n")

    while True:
        try:
            f = front.range
            l = left.range
            r = right.range
        except Exception as e:  # noqa: BLE001
            print("read error:", e)
            time.sleep(0.2)
            continue
        # Simple bar so you can see motion at a glance.
        def bar(mm):
            n = min(20, mm // 60)
            return "#" * n
        print("F {:4d}mm {:20s} | L {:4d}mm {:20s} | R {:4d}mm {:20s}".format(
            f, bar(f), l, bar(l), r, bar(r)))
        time.sleep(0.1)


main()
