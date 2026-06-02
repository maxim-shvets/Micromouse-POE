# Micromouse — setup & test plan

Goal: get every subsystem **green and ready** before a real run, driven
over Bluetooth from a laptop with an emergency stop always available.

Three layers, run in order:

1. **Bench bring-up** (`hardware/tests/`) — one component at a time, eyes on
   the hardware. Do this once per build / after rewiring.
2. **BLE self-test** (`hardware/selftest.py` + `tools/mouse_selftest.py`) —
   the whole suite in one shot, green/red verdict, repeatable, hands-off.
3. **Dry + endurance runs** — the real control loop, short then looped.

---

## 0. Setup (once)

### Laptop
```bash
pip install bleak           # BLE for the Mac console + orchestrator
```
macOS: the first BLE run prompts for Bluetooth permission for your
terminal — allow it (System Settings → Privacy & Security → Bluetooth).
Core Bluetooth hides MAC addresses, so we connect by name ("Micromouse").

### Board (CIRCUITPY drive)
Copy the CircuitPython-portable core + hardware package + libs:
```
CIRCUITPY/
  code.py                      <- whichever script you're running (see below)
  interfaces.py algorithm.py planner.py pose_fusion.py slam.py tunables.py
  hardware/__init__.py
  hardware/xiao_nrf52840.py
  hardware/ble_control.py
  lib/  adafruit_tca9548a  adafruit_vl53l0x  adafruit_lsm6ds  adafruit_ble
  tunables.json                <- optional saved profile
```
`code.py` is whatever you want to run *right now*:
- bring-up: copy a `hardware/tests/test_0X_*.py` → `code.py`
- self-test: copy `hardware/selftest.py` → `code.py`
- real run: copy `hardware/code.py` → `code.py`

### Wiring recap (must match the pin map in `xiao_nrf52840.py`)
| Part | Pins |
|---|---|
| ToF mux (TCA9548A) | I2C D4=SDA / D5=SCL, addr 0x70; VL53L0X on ch 0/1/2 = front/left/right |
| IMU (LSM6DS3TR-C) | internal I2C, addr 0x6A |
| Motors (DRV8833) | D0/D1 = left, D2/D3 = right |
| Encoders | D6 = left, D7 = right |
| Power | common ground everywhere |

> **Motor-test safety:** for anything that drives the wheels, **prop the
> robot up** or remove the wheels. Duty is capped (~0.4) and bursts are
> short, but the wheels turn.

---

## 1. Bench bring-up (per build)

Flash each `hardware/tests/test_0X_*.py` as `code.py`, watch the USB serial.
See `hardware/tests/README.md` for "what good looks like" per script.
Order: `00 i2c → 01 tof → 02 imu → 03 motors → 04 encoders →
05 drive_encoder → 06 dashboard → 07 motor_polarity`.

Done when every script reads sane values and `07` prints PASS for both
wheels. You only need to repeat this after rewiring.

---

## 2. BLE self-test (the central, repeatable check)

Flash `hardware/selftest.py` as `code.py`. Power up → it advertises as
**Micromouse**. From the laptop:

```bash
# Safe sensor suite (no motion): i2c, tof, imu, encoders, looprate
python3 tools/mouse_selftest.py

# Full: sensors + motor pulse test (WHEELS MOVE — prop up first)
python3 tools/mouse_selftest.py --full

# Endurance: run the reactive drive loop 5x to shake out intermittent faults
python3 tools/mouse_selftest.py --loops 5

# Halt the robot on the first failure
python3 tools/mouse_selftest.py --full --stop-on-fail
```

The orchestrator streams each result and prints a summary + a one-line
verdict, and **exits 0 only if everything is green** (1 = red, 2 =
timeout/not-found, 130 = Ctrl-C). That exit code makes it a drop-in
"ready?" gate.

You can also drive the suite by hand from the console
(`python3 tools/mouse_console.py`) and type: `check`, `motors`, `test`,
`loop 3`, `status`, `stop`.

### Pass criteria
| Check | Green when |
|---|---|
| `i2c` | mux `0x70` **and** IMU `0x6A/0x6B` both seen |
| `tof` | all three channels return finite, in-range values |
| `imu` | accel magnitude 7–12.5 m/s² (≈ gravity), gyro responds |
| `encoders` | counters initialise (wiring present) |
| `looprate` | ≥ 15 Hz sensor-read ceiling (else sensor-I/O work needed) |
| `motors` | each wheel moves **its own** encoder, not the other one |

### Emergency stop — always available
- **Ctrl-C** in the orchestrator/console → sends `stop` (motors off) before
  exiting.
- Send **`stop`** (or `x`) any time → aborts the current test/loop, motors off.
- `--stop-on-fail` halts on the first FAIL.
- The board also self-aborts: motor/loop phases poll `stop` every tick, and
  the run script's per-phase timeout + no-progress watchdog stop the motors
  if anything hangs.

---

## 2.5 Calibrate (after self-test is green)

Flash `hardware/calibrate.py` as `code.py` and self-tune the onboard
sensors + odometry before trusting a real run:

```bash
python3 tools/mouse_calibrate.py --save     # auto: gyro bias/noise, wheelbase, wheel scale
```

Then operator-led tuning (speeds, thresholds, centering) via the console.
Full procedure + which tunables each part sets: **hardware/TUNING.md**.
`save` persists `/tunables.json`, which every run script loads at boot.

---

## 3. Dry + endurance runs

1. **Sensor dry run** (wheels still): `tools/mouse_selftest.py` (no `--full`)
   while you wave a hand past each ToF and rotate the board — confirm the
   values move on the right channel.
2. **Single full run**: flash `hardware/code.py`, prop up the robot, connect
   with `tools/mouse_console.py`, send `go`. Watch explore → return → speed
   on the LED + status stream. `stop` if anything looks off.
3. **On-floor run**: place the robot at the maze start, `go`. Keep the
   console open — `stop` is one keystroke away.
4. **Endurance**: `tools/mouse_selftest.py --loops 10` (or repeated `go`s) to
   confirm it survives back-to-back runs without intermittent faults.

---

## Ready-to-go checklist

- [ ] `pip install bleak` on the laptop; Bluetooth permission granted
- [ ] Board files + `lib/` copied; correct script as `code.py`
- [ ] Bench bring-up 00–07 all sane (after any rewiring)
- [ ] `mouse_selftest.py` → **GREEN** (sensors)
- [ ] `mouse_selftest.py --full` → **GREEN** (motors, robot propped up)
- [ ] `looprate` ≥ your target `loop_hz` (or you accept the ceiling)
- [ ] `mouse_selftest.py --loops 5` completes with no faults
- [ ] `hardware/code.py` flashed; `go` does explore→return→speed; `stop` works
- [ ] Console stays connected during runs (emergency stop in reach)

When every box is checked, it's green and ready.
