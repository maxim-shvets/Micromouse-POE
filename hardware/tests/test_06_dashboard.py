"""Bring-up test 06 -- all-sensors live dashboard (NO motors).

Drop-in CircuitPython script.  Copy to the XIAO as `code.py`.

Reads ToF triplet + IMU + encoders together at a fixed rate and prints a
one-line dashboard.  This is the read-side of the real control loop with
the motors left off -- a safe end-to-end sensor check, and it reports the
achieved loop rate (Hz), which tells you the sensor-read budget on the
actual hardware (compare against the projected numbers from the sim's
Path-1 instrumentation).

If the loop rate is much lower than your target loop_hz, the ToF reads are
the bottleneck (each VL53L0X one-shot blocks ~30 ms).  That's the cue to
move to continuous-ranging mode / interrupt reads (the kernel work).

Needs in /lib:  adafruit_tca9548a, adafruit_vl53l0x, adafruit_lsm6ds
Pin map mirrors hardware/xiao_nrf52840.py.
"""

import time
import board
import busio

PIN_I2C_SDA, PIN_I2C_SCL = "D4", "D5"
TCA_FRONT, TCA_LEFT, TCA_RIGHT = 0, 1, 2
PIN_ENC_LA, PIN_ENC_RA = "D6", "D7"


def make_tof():
    import adafruit_tca9548a
    import adafruit_vl53l0x
    i2c = busio.I2C(getattr(board, PIN_I2C_SCL), getattr(board, PIN_I2C_SDA))
    mux = adafruit_tca9548a.TCA9548A(i2c)
    f = adafruit_vl53l0x.VL53L0X(mux[TCA_FRONT])
    l = adafruit_vl53l0x.VL53L0X(mux[TCA_LEFT])
    r = adafruit_vl53l0x.VL53L0X(mux[TCA_RIGHT])
    for s in (f, l, r):
        s.measurement_timing_budget = 33000
    return f, l, r


def make_imu():
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
    print("All-sensors dashboard (no motors).  Initialising ...")
    front, left, right = make_tof()
    imu = make_imu()
    import countio
    enc_l = countio.Counter(getattr(board, PIN_ENC_LA))
    enc_r = countio.Counter(getattr(board, PIN_ENC_RA))
    print("ready.  Ctrl-C to stop.\n")

    n = 0
    t_rate = time.monotonic()
    hz = 0.0
    while True:
        f = front.range
        l = left.range
        r = right.range
        ax, ay, az = imu.acceleration
        gz = imu.gyro[2]
        cl = enc_l.count
        cr = enc_r.count

        n += 1
        if n % 10 == 0:
            now = time.monotonic()
            hz = 10.0 / (now - t_rate) if now > t_rate else 0.0
            t_rate = now

        print("ToF F{:4d} L{:4d} R{:4d}mm | az{:+5.1f} gz{:+6.1f} | "
              "enc {:5d}/{:5d} | {:4.1f} Hz".format(
                  f, l, r, az, gz, cl, cr, hz))
        # No explicit sleep -- run as fast as the sensors allow so the
        # printed Hz reflects the true sensor-read ceiling.


main()
