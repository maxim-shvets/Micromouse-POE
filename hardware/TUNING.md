# Calibration & tuning — two parts

Run a calibration session before relying on a real run. Flash
`hardware/calibrate.py` as `code.py`, power up (advertises as
**Micromouse**), and drive it from the laptop. Two parts:

- **Part A — automatic** (code-led, uses the onboard IMU + ToF + encoders).
  The robot measures and sets these itself.
- **Part B — manual** (you + your partner, human judgement). You run a
  behaviour, watch, adjust a tunable, repeat.

`save` writes the result to `/tunables.json` on the board, which every run
script loads at boot.

---

## Part A — automatic (onboard sensors)

From the laptop:
```bash
python3 tools/mouse_calibrate.py            # full auto sequence -> local tunables.json
python3 tools/mouse_calibrate.py --save     # also persist on the board
python3 tools/mouse_calibrate.py --cmd "cal gyro"     # one routine
```
Or by hand in the console: `cal`, `cal gyro`, `cal wheelbase`, `cal encoder`.

Each routine reports a human line **and** a machine `SET key=value` line, so
the orchestrator captures the values into a local `tunables.json`.

| Routine | Ground truth (onboard) | What it sets | How |
|---|---|---|---|
| `cal gyro` | IMU at rest | `imu_bias_gyro_z_rps`, `imu_noise_gyro_rps`, `imu_noise_accel_mps2` | sit still 3 s; bias = mean gyro-z, noise = stdev; also reports mounting **tilt** from the accel vector |
| `cal wheelbase` | **gyro angle** | `wheel_base_m` | spin in place; since ω = (vᴿ−vᴸ)/L, integrate: L = ∫(vᴿ−vᴸ)dt ÷ (gyro angle). Self-calibrates odometry heading. **wheels move** |
| `cal encoder` | **front ToF** | `wheel_diameter_m` | drive at a wall; the ToF distance closed is truth, so it scales the encoder distance: scale = Δtof ÷ Δencoder. **wheels move, needs a flat wall ~0.3–0.6 m ahead** |

`cal` runs `gyro` (safe), then warns + counts down before the motion ones.
The motion routines poll `stop` every tick; Ctrl-C in the orchestrator sends
`stop`.

Why these are trustworthy: each uses a *different* onboard sensor as the
reference for the thing being calibrated — the gyro pins down the wheelbase,
the ToF pins down the wheel scale, and a still IMU pins down its own
bias/noise. No external jig or measuring tape needed.

> Not auto-calibrated (no clean sensor reference / needs human judgement):
> motor deadband, per-wheel trim, ToF absolute offset, PI gains, speed
> profiles. Those are Part B. `cal wheelbase` reports the heading drift, and
> `straight` (below) measures it, so you can hand-trim.

---

## Part B — manual (operator-led)

Use the console (`python3 tools/mouse_console.py`) and these board commands.
The loop: run a behaviour → watch → `tune` → re-run → `save` when happy.

```
tune <key>=<value>   set a tunable live (e.g. tune cruise_speed_mps=0.4)
show                 list tunables that differ from defaults
drive                reactive drive until a wall (wheels move)
spin                 rotate in place (wheels move)
straight             drive ~0.5 m straight; reports heading drift (deg/m)
save                 write current tunables to /tunables.json
stop / x             abort, motors off
```

What to tune, and what to watch:

| Goal | Behaviour to run | Tunables to adjust |
|---|---|---|
| Stops a safe distance from walls | `drive` | `front_stop_m`, `side_min_m` |
| Comfortable cruise / top speed | `drive`, `straight` | `cruise_speed_mps`, `max_speed_mps`, `max_wheel_accel_mps2`, `max_decel_mps2` |
| Stays centred in a corridor | `drive` | `wall_center_gain`, `wall_center_max_bias`, `steer_gain` |
| Clean in-place turns | `spin` | `turn_speed_mps` |
| Drives straight (no veer) | `straight` (read the drift) | re-run `cal wheelbase`/`cal encoder`; if still veering, the wheels differ mechanically |
| Wheel loop responsive, no buzz | `straight` | `encoder_kp`, `encoder_ki` (these rebuild the drive automatically when changed) |

Changing `encoder_kp/ki`, `loop_hz`, `motor_duty_cap`, or `wheel_diameter_m`
rebuilds the motor driver on the fly so the change takes effect immediately.

When it feels right: `save`, then re-flash `hardware/code.py` and do a dry
run (`go`). The saved `/tunables.json` is picked up automatically.

---

## Where this fits in the test plan

Between the self-test (everything responds) and the dry runs (real
behaviour): see `hardware/TEST_PLAN.md` step 2.5. Order is:

1. bring-up (per build) → 2. self-test GREEN → **2.5 calibrate (this doc)** →
3. dry + endurance runs.
