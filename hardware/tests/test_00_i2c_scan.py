"""Bring-up test 00 -- I2C bus scan.

Drop-in CircuitPython script.  Copy to the XIAO as `code.py` (or run from
the REPL) to scan BOTH I2C buses and list the device addresses found.

What you should see:
    External bus (D4/D5):  0x70   <- TCA9548A mux
    Internal bus (IMU):    0x6A   (or 0x6B)  <- LSM6DS3TR-C IMU

If the mux is missing -> check D4=SDA / D5=SCL wiring + pull-ups + power.
If the IMU is missing -> the XIAO Sense IMU power rail may be off; this
script enables board.IMU_PWR if the pin exists.

Pin map mirrors hardware/xiao_nrf52840.py -- keep in sync if you rewire.
"""

import time
import board
import busio

PIN_I2C_SDA = "D4"
PIN_I2C_SCL = "D5"


def scan(i2c, label):
    print("scanning {} ...".format(label))
    while not i2c.try_lock():
        pass
    try:
        addrs = i2c.scan()
    finally:
        i2c.unlock()
    if addrs:
        for a in addrs:
            print("  found 0x{:02X}".format(a))
    else:
        print("  (no devices)")
    return addrs


def main():
    print("=" * 48)
    print("I2C bus scan")
    print("=" * 48)

    # --- External bus: TCA9548A mux + (behind it) the 3 ToF sensors ------
    try:
        sda = getattr(board, PIN_I2C_SDA)
        scl = getattr(board, PIN_I2C_SCL)
        ext = busio.I2C(scl, sda)
        ext_addrs = scan(ext, "external bus D4/D5 (mux)")
        if 0x70 in ext_addrs:
            print("  OK: TCA9548A mux present at 0x70")
        else:
            print("  WARN: no 0x70 -- check mux wiring / power")
        ext.deinit()
    except Exception as e:  # noqa: BLE001
        print("  ERROR on external bus:", e)

    print()

    # --- Internal bus: on-board LSM6DS3TR-C IMU --------------------------
    # The XIAO Sense gates IMU power behind board.IMU_PWR on some firmware
    # revisions; enable it before scanning.
    try:
        import digitalio
        pwr = getattr(board, "IMU_PWR", None)
        if pwr is not None:
            en = digitalio.DigitalInOut(pwr)
            en.direction = digitalio.Direction.OUTPUT
            en.value = True
            time.sleep(0.05)
            print("enabled IMU_PWR")
    except Exception as e:  # noqa: BLE001
        print("note: could not toggle IMU_PWR:", e)

    try:
        try:
            imu_i2c = board.IMU_I2C()
        except AttributeError:
            scl = getattr(board, "IMU_SCL", None) or getattr(board, "P0_24")
            sda = getattr(board, "IMU_SDA", None) or getattr(board, "P0_25")
            imu_i2c = busio.I2C(scl, sda)
        imu_addrs = scan(imu_i2c, "internal bus (IMU)")
        if 0x6A in imu_addrs or 0x6B in imu_addrs:
            print("  OK: LSM6DS3TR-C IMU present")
        else:
            print("  WARN: no 0x6A/0x6B -- IMU not responding")
    except Exception as e:  # noqa: BLE001
        print("  ERROR on internal bus:", e)

    print()
    print("scan complete.")


main()
