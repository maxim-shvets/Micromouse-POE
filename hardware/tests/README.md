# Hardware bring-up tests

Standalone **drop-in CircuitPython** scripts to verify each part of the
XIAO nRF52840 Sense micromouse build *before* running the full algorithm.
Each script is self-contained: copy it to the board's `CIRCUITPY` drive as
`code.py` (or paste into the REPL) and read the console over USB serial.

> All pin maps mirror `hardware/xiao_nrf52840.py`. If you rewire, update
> both. The constants are duplicated into each script on purpose so a test
> never depends on the rest of the repo being present on the board.

## Hardware recap

| Part | Connection |
|---|---|
| Seeed XIAO nRF52840 **Sense** | controller (on-board LSM6DS3TR-C IMU) |
| DRV8833 dual motor driver | D0/D1 = left (AIN1/AIN2), D2/D3 = right (BIN1/BIN2) |
| 3× VL53L0X ToF | external I2C **D4=SDA / D5=SCL** → TCA9548A mux ch 0/1/2 = front/left/right |
| TCA9548A I2C mux | external bus, address `0x70` |
| 2× N20 + encoders | encoder channels D6 (left) / D7 (right), single-channel |
| Power | 4×AA NiMH or 2S LiPo + 5 V buck; **common ground everywhere** |

CircuitPython libraries needed in `CIRCUITPY/lib/`:
`adafruit_tca9548a`, `adafruit_vl53l0x`, `adafruit_lsm6ds`
(grab them from the matching-version Adafruit CircuitPython Library Bundle).

## Run them in this order

| # | Script | Verifies | Motors move? |
|---|---|---|---|
| 00 | `test_00_i2c_scan.py` | both I2C buses; finds mux `0x70` + IMU `0x6A` | no |
| 01 | `test_01_tof.py` | 3× VL53L0X through the mux; channel mapping | no |
| 02 | `test_02_imu.py` | LSM6DS3TR-C accel/gyro; gyro-z bias; sign | no |
| 03 | `test_03_motors.py` | DRV8833 drives each motor both directions | **YES** |
| 04 | `test_04_encoders.py` | encoder counting; calibrate counts/rev | yes (phase 2) |
| 05 | `test_05_drive_encoder.py` | duty→measured-m/s curve | **YES** |
| 06 | `test_06_dashboard.py` | all sensors together + achieved loop Hz | no |
| 07 | `test_07_motor_polarity.py` | left/right + polarity + encoder pairing | **YES** |

> **Before any motor test (03, 05, 07 and phase 2 of 04): prop the robot
> up so the wheels spin free, or take the wheels off.** Duty is capped at
> 0.45 and bursts are short, but the wheels *will* turn.

## What "good" looks like

- **00** — `0x70` on the external bus, `0x6A` (or `0x6B`) on the internal
  IMU bus. Missing `0x70` → check D4/D5 + pull-ups + mux power. Missing
  IMU → power rail (`IMU_PWR`) or internal-bus pin names.
- **01** — all three channels read a large number with nothing in front
  and drop to ~100 mm with a hand at 10 cm. Wave per channel to confirm
  front/left/right map to `TCA_CHAN_*`.
- **02** — flat & still: `accel_z ≈ +9.8`, `|gyro| ≈ 0` plus a small
  constant bias (printed). Rotate CCW about vertical → `gyro_z` positive.
  Backwards? Flip the gyro-z entry in `XiaoIMU.AXIS_REMAP`.
- **03** — each named wheel spins the named direction. Wrong wheel → swap
  the `(M1*, M2*)` pin pairs. Backward → swap that motor's two pins.
- **04** — one hand-turn of a wheel ≈ `ENC_COUNTS_PER_REV` counts. Set
  that constant from what you actually measure.
- **05** — speed rises with duty and the m/s values are believable for
  32 mm wheels (e.g. 0.1–0.6 m/s). Seeds the PI gains / `motor_duty_cap`.
- **06** — steady dashboard; note the **Hz**. If it's far below your
  target `loop_hz`, the blocking ToF reads are the ceiling → that's the
  signal to do the sensor-I/O kernel work (continuous ranging + interrupts).
- **07** — `PASS` for both wheels. Any `CHECK WIRING` prints the exact
  pins to swap.

## After bring-up

Once 00–07 all pass, copy the real adapter + algorithm to the board:

```
# on CIRCUITPY:
#   code.py  ->  from hardware.xiao_nrf52840 import main; main()
#   plus: algorithm.py planner.py slam.py pose_fusion.py tunables.py
#         interfaces.py  (the CircuitPython-portable core)
#   optionally /tunables.json  (a saved profile)
```

Calibration values you collected here that feed the algorithm:
- `ENC_COUNTS_PER_REV` (from test 04) → `hardware/xiao_nrf52840.py`
- gyro-z bias sanity (test 02) → fusion converges to this
- duty→speed (test 05) → `encoder_kp` / `encoder_ki` / `motor_duty_cap`
- achieved sensor Hz (test 06) → realistic `loop_hz` ceiling
