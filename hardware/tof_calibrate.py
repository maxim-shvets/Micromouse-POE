"""ToF sensor calibration script for XIAO nRF52840.

Copy this file to the CIRCUITPY root as `code.py`, open a serial terminal
(Mu editor, `screen`, `tio`, etc.), and follow the prompts.  Use a ruler or
calipers to place a flat, perpendicular target at each stated distance.

After completing all three sensors the calibration is written to
`/tof_cal.json` on the board.  Swap `code.py` back to the main run script
when done -- xiao_nrf52840.py loads the file automatically on startup.

Calibration model (per sensor):
    corrected_mm = slope * raw_mm + offset

Two sources of VL53L0X inaccuracy are corrected:
    - Offset error  : constant shift from cover-glass reflection, crosstalk,
                      or factory trim mismatch.  Visible as a fixed delta at
                      all distances.
    - Scale error   : proportional stretch/shrink across the range.  Shows up
                      as error that grows with distance.
A two-point linear fit captures both; more calibration distances reduce noise
in the fitted coefficients.

CircuitPython-portable: no f-strings, no walrus, no complex stdlib.
"""

import time
import json

# ---------------------------------------------------------------------------
# Calibration settings -- adjust if needed.
# ---------------------------------------------------------------------------

# Distances (mm) you will measure with a ruler.  Use at least 3 points that
# span the operating range of the maze walls (~30 mm to ~200 mm).
CAL_DISTANCES_MM = [30, 60, 90, 120, 180]

# Readings averaged per distance point.  More = less noise in the fit.
SAMPLES_PER_POINT = 30

# Delay between samples (s).  33 ms timing budget -> 0.04 s is safe.
SAMPLE_DELAY_S = 0.04

# Where to write the result.
CAL_FILE = "/tof_cal.json"

# ---------------------------------------------------------------------------
# Hardware pin constants (must match xiao_nrf52840.py).
# ---------------------------------------------------------------------------

PIN_SCL          = "D5"
PIN_SDA          = "D4"
PIN_XSHUT_RIGHT  = "D2"
PIN_XSHUT_MIDDLE = "D3"
PIN_XSHUT_LEFT   = "D6"

TOF_ADDR_RIGHT   = 0x2A
TOF_ADDR_MIDDLE  = 0x2B
TOF_ADDR_LEFT    = 0x2C

# VL53L0X returns 8190 or 65535 for out-of-range / timeout reads.
TOF_INVALID_MIN  = 8000


# ---------------------------------------------------------------------------
# Math helpers
# ---------------------------------------------------------------------------

def _mean(vals):
    return sum(vals) / len(vals)


def _linear_fit(xs, ys):
    """Least-squares linear fit: y = slope*x + intercept."""
    n   = len(xs)
    sx  = sum(xs)
    sy  = sum(ys)
    sxy = sum(xs[i] * ys[i] for i in range(n))
    sxx = sum(xs[i] * xs[i] for i in range(n))
    denom = n * sxx - sx * sx
    if denom == 0.0:
        return 1.0, 0.0
    slope     = (n * sxy - sx * sy)  / denom
    intercept = (sy - slope * sx)    / n
    return slope, intercept


def _rmse(xs, ys, slope, intercept):
    """Root-mean-square residual of the fit."""
    total = 0.0
    for x, y in zip(xs, ys):
        err = y - (slope * x + intercept)
        total += err * err
    return (total / len(xs)) ** 0.5


# ---------------------------------------------------------------------------
# Sensor helpers
# ---------------------------------------------------------------------------

def _sample(sensor, n, delay):
    """Return average of n valid readings, or None if none are valid."""
    vals = []
    for _ in range(n):
        r = sensor.range
        if 0 < r < TOF_INVALID_MIN:
            vals.append(float(r))
        time.sleep(delay)
    if not vals:
        return None
    return _mean(vals)


def _calibrate_one(label, sensor):
    """Interactively collect data points and return {slope, offset} dict."""
    print("")
    print("=== {} sensor ===".format(label))
    raw_readings = []
    true_mm      = []

    for dist in CAL_DISTANCES_MM:
        while True:
            try:
                input("  Place target at {} mm then press ENTER "
                      "(or type 's' to skip): ".format(dist))
            except Exception:
                # CircuitPython input() raises on Ctrl-C -- treat as skip.
                print("  skipped.")
                break
            avg = _sample(sensor, SAMPLES_PER_POINT, SAMPLE_DELAY_S)
            if avg is None:
                print("  No valid readings -- check target is in range "
                      "(30-1200 mm) and retry.")
                continue
            print("  Raw avg: {:.1f} mm  |  target: {} mm  |  "
                  "error: {:+.1f} mm".format(avg, dist, avg - dist))
            raw_readings.append(avg)
            true_mm.append(float(dist))
            break

    if len(raw_readings) < 2:
        print("  < 2 valid points -- using identity calibration.")
        return {"slope": 1.0, "offset": 0.0}

    slope, offset = _linear_fit(raw_readings, true_mm)
    rmse = _rmse(raw_readings, true_mm, slope, offset)
    print("  Fit  : corrected = {:.5f} * raw + ({:.2f} mm)".format(
        slope, offset))
    print("  RMSE : {:.2f} mm over {} points".format(rmse, len(raw_readings)))
    return {"slope": round(slope, 6), "offset": round(offset, 4)}


# ---------------------------------------------------------------------------
# Hardware init (mirrors xiao_nrf52840.TcaVL53L0X.__init__)
# ---------------------------------------------------------------------------

def _init_sensors():
    import board
    import busio
    import digitalio
    import adafruit_vl53l0x

    def _xshut(pin_name):
        io = digitalio.DigitalInOut(getattr(board, pin_name))
        io.direction = digitalio.Direction.OUTPUT
        io.value = False
        return io

    xs_right  = _xshut(PIN_XSHUT_RIGHT)
    xs_middle = _xshut(PIN_XSHUT_MIDDLE)
    xs_left   = _xshut(PIN_XSHUT_LEFT)

    import bitbangio
    i2c = bitbangio.I2C(getattr(board, PIN_SCL), getattr(board, PIN_SDA))

    xs_right.value = True
    time.sleep(0.01)
    right = adafruit_vl53l0x.VL53L0X(i2c)
    right.set_address(TOF_ADDR_RIGHT)

    xs_middle.value = True
    time.sleep(0.01)
    middle = adafruit_vl53l0x.VL53L0X(i2c)
    middle.set_address(TOF_ADDR_MIDDLE)

    xs_left.value = True
    time.sleep(0.01)
    left = adafruit_vl53l0x.VL53L0X(i2c)
    left.set_address(TOF_ADDR_LEFT)

    for s in (right, middle, left):
        s.measurement_timing_budget = 33000

    return right, middle, left


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    print("")
    print("ToF Sensor Calibration")
    print("======================")
    print("Sensors : RIGHT (D2)  MIDDLE (D3)  LEFT (D6)")
    print("Points  : {} mm".format(CAL_DISTANCES_MM))
    print("Samples : {} per point".format(SAMPLES_PER_POINT))
    print("")
    print("Use a flat target (cardboard, book) held perpendicular to the")
    print("sensor face.  Measure distance from the sensor lens, not the PCB.")
    print("")

    try:
        right, middle, left = _init_sensors()
    except Exception as e:
        print("ERROR initialising sensors:", e)
        return

    cal = {}
    cal["right"]  = _calibrate_one("RIGHT",  right)
    cal["middle"] = _calibrate_one("MIDDLE", middle)
    cal["left"]   = _calibrate_one("LEFT",   left)

    # Write result.
    try:
        with open(CAL_FILE, "w") as f:
            json.dump(cal, f)
        print("")
        print("Saved to", CAL_FILE)
    except Exception as e:
        print("ERROR saving calibration:", e)
        print("Copy this manually into", CAL_FILE, ":")
        print(json.dumps(cal))
        return

    # Print a human-readable summary.
    print("")
    print("Summary")
    print("-------")
    for name in ("right", "middle", "left"):
        c = cal[name]
        print("  {} : slope={} offset={} mm".format(
            name.upper(), c["slope"], c["offset"]))
    print("")
    print("Done. Swap code.py back to the main run script.")


main()
