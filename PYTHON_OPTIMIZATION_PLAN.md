# Python Loop Optimization Plan — XIAO nRF52840 Sense

**Status:** Planning only. No code changes yet. Written 2026-05-23 after Path 1
(CPU slowdown projection) showed the current SLAM EKF tick projects to ~36 ms
on XIAO vs the 5 ms race-mode budget.

## Goal

Bring the per-tick wall-clock time on the XIAO under the budget for each
aggression mode:

| Mode    | Loop Hz | Budget (ms) | Today (projected on XIAO @ 170×) | Gap |
|---------|--------:|------------:|---------------------------------:|----:|
| cautious   | 50  | 20 | ~2-8 (GT/fused), 36 (slam) | slam over by ~16 ms |
| normal     | 50  | 20 | similar | similar |
| aggressive | 100 | 10 | ~3-9 (GT/fused), 36 (slam) | slam over by ~26 ms |
| race       | 200 | 5  | ~5 (GT/fused), 36 (slam)   | slam over by ~31 ms |

**Headline target:** race + SLAM on XIAO with no overruns. That means
getting the SLAM tick from ~36 ms down to ~3-4 ms (8-10× speedup).

The two cost drivers are:

1. **SLAM EKF** (slam.py:_measurement_update): central-difference Jacobian
   does 18 ray-casts per tick (3 channels × 6 perturbations).
2. **Planner Dijkstra** (planner.py:flood_fill_weighted): full re-flood
   over (cell, facing) every time a new wall is observed → 256-cell ×
   4-facing = 1024-node Dijkstra. Projected as 200-465 ms spikes on XIAO.

Path 1 instrumentation (`tester.py` --- perf section, stress.py
`--cpu-slowdown-factor`) lets us measure after each change.

## Strategy: 4 layers of optimization in order of effort/payoff

| Layer | Effort | Payoff | What |
|---|---|---|---|
| **A. Algorithmic** | 1-3 days | 10-50× on hot paths | Analytical Jacobian, incremental Dijkstra |
| **B. Coalesce + sub-sample** | half-day | 2-3× | Replan once per cell, SLAM every N ticks |
| **C. MicroPython native** | 2-3 days | 3-10× on annotated functions | `@viper`/`@native` decorators |
| **D. Runtime switch** | 1 day | 2-3× | CircuitPython → MicroPython on XIAO |

Do them in this order. After A + B, race + SLAM should already fit. C + D
are insurance against not-quite-fitting.

---

## Layer A — Algorithmic fixes

### A1. Analytical SLAM Jacobian

**Current code (slam.py around line 175):** central-difference numerical
Jacobian — for each of 3 ToF channels, perturb each of (x, y, θ) by
eps=1e-4 in both directions and re-raycast → 6 ray-casts per channel × 3
channels = **18 ray-casts per tick**. Each ray-cast is O(walls), maybe 20
walls visible. So ~360 wall-checks per tick just for the Jacobian.

**Proposed:** derive closed-form partial derivatives of the ray-cast
distance with respect to (x, y, θ).

For a ray from sensor origin `(s_x, s_y)` at angle `α + θ` hitting wall
segment `((wx1,wy1),(wx2,wy2))`:

- The intersection point gives distance `d`.
- ∂d/∂x and ∂d/∂y are the components of the wall's NORMAL projected on
  the ray direction (negated). Specifically, if wall normal is `n̂`:
  `∂d/∂x = -n̂_x / cos(angle between ray and normal)`.
- ∂d/∂θ is the perpendicular distance the ray sweeps as θ changes, plus
  the sensor-mount-offset contribution.

These are closed-form trig expressions, ~5 multiplications per partial.
Total: 9 partials per channel × 3 channels = 27 multiplications,
plus the SAME 3 ray-casts to find the hit point (down from 18). About 50
arithmetic ops total instead of 18 ray-casts ≈ 360 wall-checks.

**Projected speedup**: 7-10× on the SLAM update. Drops the tick from ~36 ms
to ~5 ms on XIAO. Within budget for normal/aggressive, just over for race.

**Risk:** the analytical Jacobian must handle the "ray switches walls"
discontinuity. When a perturbation flips the ray to hit a different wall,
the central-difference handles it implicitly (just takes the new distance).
Analytical needs to detect the active wall first, then compute partials
*for that wall*. Same wall as long as perturbation is small (< wall-segment
end clearance), which is the normal case. When the ray is near a wall
corner, fall back to central diff or just clamp the Jacobian magnitude.

**Estimated effort**: 1 day. Dispatch as a Codex spec like the SLAM EKF
refactor: clear API contract (preserve `_measurement_update` signature),
math derivation in the spec, hint about the corner-case fallback.

### A2. Incremental Dijkstra (D*-lite or LPA*)

**Current code (planner.py:replan):** every time `_dirty=True`, runs
`flood_fill_weighted` which is a full O(N log N) Dijkstra over N = 4×cols×rows
states. For 16×16, that's 1024 states. Projected 200-465 ms on XIAO.

**Proposed:** D*-lite or LPA* — incremental shortest path that only
re-expands the subgraph affected by the new wall. Typical re-expansion
size for a single wall change: 10-50 cells, not the full 1024.

**D*-lite specifics:**
- Maintain `g[state]` (best known cost-to-goal) and `rhs[state]`
  (one-step-lookahead estimate from successors).
- On wall change: mark predecessors of the affected cell as inconsistent.
- Re-expand only inconsistent states using priority queue.
- Same Dijkstra-like termination but pruned.

**Projected speedup**: 10-50× on replan-after-observation; the first plan
(start of run) is the same full Dijkstra. Drops 200-465 ms spikes to
20-50 ms spikes. Still over budget for race (5 ms) but only at observation
events; not every tick.

**Estimated effort**: 1-2 days. Dispatch as Codex spec. Standard algorithm
with well-known reference implementations.

**Alternative if D*-lite is too complex**: coalesce replans (run replan
at most once per N ticks, batching all observations between). 5× cheaper
on average but doesn't fix the worst-case spike.

### A3. Cap Dijkstra at lookahead

If A2 is too complex, simpler partial-fix: only re-flood the cells within
N cells of the robot's current position. The far end of the maze will be
stale, but the planner only uses the next 1-2 steps anyway. With N=6,
re-flood region is ~13×13 = 169 cells × 4 facings = 676 states. About
1.5× faster than full re-flood, less aggressive than D*-lite.

**Risk**: when the path needs to backtrack past the lookahead horizon
(rare in micromouse), the planner sees stale data. Add a full-flood
heartbeat every few seconds as backup.

---

## Layer B — Coalesce + sub-sample

### B1. Coalesce planner replans

**Current code (algorithm.py:_planner_step):** replan triggered when
`planner._dirty` is True AND `_planner_t >= planner_replan_period_s`
(default 0.1 s). So replans happen at most every 0.1 s = every 20 ticks at
200 Hz.

**Tweak:** bump `planner_replan_period_s` to 0.2-0.5 s. Walls observed
during the interim are accumulated; replan covers all at once. Same total
Dijkstra cost but distributed across the budget.

**Effort:** trivial — change a default. Zero risk.

### B2. SLAM update every N ticks

**Current code:** SLAM `update()` runs every tick.

**Tweak:** run prediction (cheap, just integration) every tick, but only
run measurement update every N ticks. With N=3 (race), SLAM tick cost drops
from 36 ms to 12 ms average (still one expensive every 3rd tick, but most
ticks are cheap).

**Tunable**: `slam_measurement_period_ticks` (default 1 = current behavior).

**Risk:** correction lag. If the robot's dead-reckoning drifts between
updates, the planner sees a stale corrected pose for N-1 ticks. At race
3-tick = 15 ms, robot moves 30 mm at 2 m/s. Probably OK; the planner's
cell attribution is robust to 30 mm error.

**Effort:** 2 hours. Add tunable + counter in slam.py.

### B3. Reduce SLAM measurement noise model size

The central-difference Jacobian is doing 18 ray-casts because we have 3
channels × 3 state dimensions × 2 (forward/back perturbation). After A1
this drops to 3 ray-casts. Until A1 lands, we could drop to 9 ray-casts
by using ONE-SIDED differences (3 × 3 perturbations) — half the cost, half
the accuracy. Stopgap only.

---

## Layer C — MicroPython native

**Insight from Path 3:** MicroPython on Mac runs the codebase at ~3.8× the
speed of CPython (148 → 559 µs/tick). CircuitPython on XIAO is likely
slower than MicroPython on the same chip by ~1.5-2× because of
CircuitPython's stricter safety guarantees.

MicroPython (NOT CircuitPython) supports two decorators that compile
Python source to native ARM code on the target:

- **`@micropython.native`** — compiles Python bytecode to native machine
  code while keeping Python-level dynamic typing. Typical 2-5× speedup.
- **`@micropython.viper`** — restricted Python subset (no Python objects,
  just int/uint/ptr32). Generates near-C performance. Typical 10-100×
  speedup, but you have to rewrite the function in viper-dialect.

**Hot-path candidates for `@native` (low effort, 2-5× win each):**
- `slam.py:_predict` (matrix arithmetic, called every tick)
- `slam.py:_measurement_update` (after A1 lands)
- `pose_fusion.py:update` (called every tick)
- `algorithm.py:_safe_forward_speed`, `_wall_center_bias` (called every tick)
- `algorithm.py:step` (the pure reactive function)

**Hot-path candidates for `@viper` (high effort, 10-100× win each):**
- The 3×3 matrix multiply helpers in slam.py (`_matmul_3x3`, `_inverse_3x3`)
- The ray-cast routine in slam.py (after A1, this is the only ray-cast cost)
- The planner's BFS / Dijkstra inner loop

**Tradeoff:** must move from CircuitPython to MicroPython on XIAO.
CircuitPython doesn't have these decorators. Costs:

- CircuitPython libs (adafruit_lsm6ds for IMU, adafruit_vl53l0x for ToF)
  may need MicroPython equivalents or hand-port.
- Less hand-holding (CircuitPython has nicer error messages, automatic
  rescue REPL after crashes).
- Slightly different file-on-FS conventions (`code.py` → `main.py`).

**Estimated effort:** 2-3 days of careful annotation + reverify on MicroPython
smoke test. The decorator placement is targeted (10-20 functions), but
each viper function needs a careful rewrite.

**Estimated payoff:** 3-10× on annotated functions. If we annotate the
SLAM matrix ops and ray-cast (post-A1) with `@viper`, we get another 5-10×
on top of A1's 7-10×. Combined: 35-100× total speedup on SLAM tick — from
36 ms to 0.4-1 ms. Comfortable race budget.

---

## Layer D — Runtime switch

Switch the XIAO from CircuitPython to MicroPython.

**Justification:** A measured 3.8× speedup on the same hardware between
the two runtimes (from our Path 3 bench), plus access to `@viper`/`@native`
from Layer C.

**Migration cost:**
- Reflash firmware: install MicroPython for nRF52840 (Adafruit and
  MicroPython.org publish images).
- Re-write `hardware/xiao_nrf52840.py`: replace CircuitPython imports
  (`board`, `busio`, `digitalio`, `pwmio`, `adafruit_lsm6ds`,
  `adafruit_vl53l0x`) with MicroPython equivalents (`machine`,
  driver libs from awesome-micropython).
- Reverify with the MicroPython smoke test (already in repo as
  `micropython_smoke.py`).

**Risk:**
- IMU + ToF driver maturity on MicroPython is lower than on CircuitPython.
  Adafruit invests heavily in CircuitPython libs; MicroPython relies on
  community drivers.
- Possible regressions in REPL / debug experience.

**Effort:** 1 day to reflash + port + smoke-test. Another 1-2 days for
the driver work if no off-the-shelf MicroPython lib exists.

**Order**: do Layers A + B + C first, decide at the end whether D is
needed.

---

## Other smaller wins (apply opportunistically)

- **Pre-compute trig tables**: `math.sin`/`math.cos` are slow in
  CircuitPython. Pre-compute `theta_from_heading` for the 4 cardinals (we
  already do); for non-cardinal headings during arc turns, table-lookup
  if the angle resolution is fixed.

- **Avoid `dict` access in hot paths**: dict lookups are ~10× slower than
  `__slots__` attribute access on CircuitPython. Audit
  `algorithm.py:_planner_step` for any dict reads (should be none — we
  already use Tunables `__slots__`, but double-check).

- **Inline tight loops**: small inner helpers like `wrap_pi` have function
  call overhead that dominates the actual math. Inline manually in the
  hottest call sites.

- **`__slots__` everywhere**: enforce on all CircuitPython-portable
  classes (already done on Tunables, Reading, WheelSpeeds, IMUReading;
  check FloodFillPlanner, ReactiveController, ScanMatchSlam, FusedOdometry).

- **Drop reading.timestamp**: the float adds ~3 ms per call cycle when
  rounded for telemetry. Use integer ticks if we don't need wall time.

---

## Verification plan

After each layer:

1. Run the Path 1 stress sweep:
   ```bash
   python3 stress.py --sim-time 60 --sim-time-per-cell-side 3 \
                     --cpu-slowdown-factor 200
   ```
   This projects to XIAO timing. The output's "perf" table should show
   overrun% dropping.

2. Run the MicroPython smoke test:
   ```bash
   micropython micropython_smoke.py
   ```
   Confirms we haven't broken portability.

3. Run the MicroPython bench:
   ```bash
   micropython micropython_bench.py
   python3 micropython_bench.py
   ```
   The ratio between the two should stay around 3-5×. Layer C should
   IMPROVE the MicroPython number specifically (since `@viper`/`@native`
   only run there).

4. After Layer D, run the actual XIAO with telemetry probes around the
   loop. Cross-check projected timing against measured. Update
   `cpu_slowdown_factor` in `tunables.py` to the measured value.

---

## Recommendation: priority order

If we have **3 days**: A1 + B1 + B2. Brings race + SLAM under budget on
average, with replan spikes still occasional.

If we have **1 week**: A1 + A2 + B1 + B2. Race + SLAM cleanly under budget
including spikes.

If we have **2 weeks**: add C (annotate top 5 functions). Comfortable
margin; cell-size changes / faster motors don't blow the budget.

If we have **3 weeks** and still need more: do D (runtime switch) + remaining
C (viper rewrites). Last resort before going to C++ (see CPP_REFACTOR_PLAN.md).

---

## When NOT to bother

If the demo uses a fully-mapped maze in race mode (Maxim's
explore-then-race pattern), the planner's Dijkstra cost amortizes — the
map doesn't change during the race, so no replans fire. In that scenario:
- B1 (coalesce replans) becomes irrelevant
- A2 (incremental Dijkstra) becomes irrelevant
- Only SLAM cost matters → A1 + maybe B2 + maybe C is enough

This is probably the most realistic POE demo scenario. We'd run the
explore phase at cautious tunables (cheap), then race phase at race
tunables. Hot path: SLAM only.
