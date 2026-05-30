"""Bring-up test 02 -- on-board LSM6DS3TR-C IMU.

Drop-in CircuitPython script.  Copy to the XIAO as `code.py`.

Prints accel (m/s^2) and gyro (deg/s).  Two phases:
  1. STILL  : hold the board flat + motionless ~3 s.  Reports the gyro-z
              bias (should be a small constant; this is what the fusion
              layer estimates and subtracts) and confirms accel-z ~ +9.8.
  2. LIVE   : continuous stream -- rotate the board and watch gyro_z; tilt
              it and watch accel.

Sanity checks:
  - Flat + still: accel ~ (0, 0, +9.8) m/s^2, |gyro| ~ 0 (+ small bias).
  - Rotate CCW about vertical (z): gyro_z goes positive (right-hand rule).
  If gyro_z sign is backwards vs the mouse's turn direction, flip the
  gyro-z entry in XiaoIMU.AXIS_REMAP.

Needs in /lib:  adafruit_lsm6ds
"""

import time
import board
import busio


def make_imu():
    # Enable IMU power rail if the board gates it.
    try:
        import digitalio
        pwr = getattr(board, "IMU_PWR", None)
        if pwr is not None:
            en = digitalio.DigitalInOut(pwr)
            en.direction = digitalio.Direction.OUTPUT
            en.value = True
            time.sleep(0.05)
    except Exception:  # noqa: BLE001
        pass
    try:
        i2c = board.IMU_I2C()
    except AttributeError:
        scl = getattr(board, "IMU_SCL", None) or getattr(board, "P0_24")
        sda = getattr(board, "IMU_SDA", None) or getattr(board, "P0_25")
        i2c = busio.I2C(scl, sda)
    import adafruit_lsm6ds.lsm6ds3trc as lsm
    return lsm.LSM6DS3TRC(i2c)


def main():
    print("IMU test (LSM6DS3TR-C)")
    dev = make_imu()

    # --- Phase 1: bias estimate while still ------------------------------
    print("\nPHASE 1: hold STILL and flat for 3 seconds ...")
    time.sleep(0.5)
    n = 0
    sgz = 0.0
    saz = 0.0
    t_end = time.monotonic() + 3.0
    while time.monotonic() < t_end:
        _, _, az = dev.acceleration
        _, _, gz = dev.gyro
        saz += az
        sgz += gz
        n += 1
        time.sleep(0.01)
    if n:
        print("  accel_z mean : {:+.2f} m/s^2  (expect ~ +9.8 flat)".format(saz / n))
        print("  gyro_z  bias : {:+.3f} deg/s  (the fusion layer removes this)".format(sgz / n))

    # --- Phase 2: live stream -------------------------------------------
    print("\nPHASE 2: live stream -- rotate/tilt the board.  Ctrl-C to stop.\n")
    while True:
        ax, ay, az = dev.acceleration
        gx, gy, gz = dev.gyro
        print("a=({:+5.1f},{:+5.1f},{:+5.1f}) m/s^2   "
              "g=({:+6.1f},{:+6.1f},{:+6.1f}) deg/s".format(
                  ax, ay, az, gx, gy, gz))
        time.sleep(0.1)


main()
