# C++ Refactor Plan — Contingency

**Status:** Planning only. No code changes yet. Written 2026-05-23 as a
contingency for the case where the Python optimization plan
(PYTHON_OPTIMIZATION_PLAN.md) doesn't reach the XIAO budget.

## When to invoke this plan

Only if **all** of the following are true after Python optimization:

1. SLAM tick on XIAO still exceeds race budget (5 ms) by > 2× after
   analytical Jacobian + MicroPython native annotations.
2. The micromouse competition class we're entering rewards top speed
   enough to make the rewrite worth it (2-3 month effort).
3. We have hardware to test on (XIAO + sim) before committing.

**Otherwise, prefer the Python optimization route**. C++ rewrite trades
3 weeks of code-change time for ~50× speed; for the POE demo target this is
almost certainly overkill.

## Goals

1. **Same algorithm**: not redesigning the controller, just rewriting it.
2. **Single code base**: the sim AND the hardware both run the same C++
   code; no Python-vs-C++ divergence.
3. **Visual workflow preserved**: matplotlib visualizer keeps working
   (via pybind11 bindings into the C++ core).
4. **CircuitPython adapter retained as legacy fallback**: if the C++
   port has a bug, we can swap back. Don't delete `hardware/xiao_nrf52840.py`.

## Architecture

### Layer model

```
┌─────────────────────────────────────────────────────────────┐
│  Sim app (Python)            │  Hardware app (C++)          │
│  ─ tester.py                 │  ─ main.cpp                  │
│  ─ visualizer (matplotlib)   │  ─ Zephyr / Arduino loop     │
│  ─ telemetry, tuning advisor │                              │
├─────────────────────────────────────────────────────────────┤
│   Python ⇄ C++ bridge (pybind11)                            │
├─────────────────────────────────────────────────────────────┤
│            C++ core library (libmicromouse_core)            │
│  ─ Interfaces (Drive, RangeSensors, IMU, Clock)             │
│  ─ Algorithm (ReactiveController, PathController)           │
│  ─ Planner (FloodFillPlanner, KnownMap)                     │
│  ─ Estimators (FusedOdometry, ScanMatchSlam)                │
│  ─ Tunables                                                 │
├─────────────────────────────────────────────────────────────┤
│ Sim impl (Python, in sim/)   │  Hardware impl (C++)         │
│  ─ SimWorld, SimSensors      │  ─ XIAO drivers (DRV8833,    │
│  ─ SimDrive, SimIMU          │     VL53L0X via I2C+TCA9548A,│
│  ─ SimClock                  │     LSM6DS3TR-C, encoders)   │
└─────────────────────────────────────────────────────────────┘
```

The **C++ core library** is the single source of truth for all control
logic. Both Python (sim) and C++ (hardware) call into it.

### Why pybind11 instead of porting the sim to C++

- Sim does heavy use of matplotlib, numpy, scipy. Porting to C++ would
  require Qt/OpenGL/etc. — large investment for zero performance benefit
  (sim already runs >> realtime on Mac).
- Python is the right tool for the sim's debugging / visualization work.
- C++ is the right tool for the embedded controller, period.
- pybind11 is mature, low-overhead, and lets us call the C++ core directly
  from Python with minimal friction.

### Why NOT MicroPython native or full Python+ctypes

- MicroPython native (planned in PYTHON_OPTIMIZATION_PLAN.md Layer C) is
  the cheap intermediate. **Try it first.**
- Full ctypes is awkward for our object-heavy API.
- pybind11 + a real C++ core is the clean break we want IF Python doesn't
  fit.

## Translation strategy

Port the modules in dependency order. After each module, the sim must
still pass the existing stress sweep using the ported module — golden
output comparison against the Python reference.

| Order | Module        | Lines | Effort | Notes |
|------:|---------------|------:|-------:|-------|
| 1     | interfaces    | 130   | 0.5 day | Pure types: Reading, WheelSpeeds, IMUReading. Define as C++ structs + pybind11 bindings. |
| 2     | tunables      | 300   | 0.5 day | Struct of public fields + JSON load/save via nlohmann::json. CLI override parsing in Python wrapper. |
| 3     | planner       | 500   | 1 day | KnownMap, FloodFillPlanner. Standard data structures. heapq → std::priority_queue. |
| 4     | pose_fusion   | 250   | 0.5 day | FusedOdometry. Linear complementary filter. |
| 5     | slam          | 350   | 1.5 days | ScanMatchSlam EKF. The 3x3 matrix ops become Eigen::Matrix3f. Analytical Jacobian (if not yet done in Python — port the math, not the implementation). |
| 6     | algorithm     | 600   | 1.5 days | ReactiveController + WheelController + PathController + step(). State machines port cleanly. |
| 7     | path_controller | 400 | 1 day | Pure Pursuit. Path generation + curvature computation. |
| 8     | bindings      | 200   | 1 day | pybind11 module exposing all the above to Python. |
| 9     | sim adapter   | 100   | 0.5 day | Modify sim/world.py to call C++ core via bindings. |
| 10    | hw adapter    | 400   | 2 days | xiao_nrf52840.cpp: I2C driver, ToF, IMU, encoders, PWM. Use Zephyr or Arduino-IDE for the build. |
| 11    | main.cpp      | 100   | 0.5 day | Boot, loop, OOM watchdog. |

**Total estimate: 9 days of focused effort.** With verification + iteration:
**2-3 weeks** to a working C++ build that passes the same stress sweep as
Python.

### Order rationale

- Bottom-up so each layer has the dependencies it needs.
- After step 5 (slam), we can already benchmark the C++ SLAM tick on Mac
  vs the Python SLAM tick to validate the speedup hypothesis BEFORE
  committing the rest of the rewrite.

### Per-module verification gate

After porting each module, before moving to the next:

```bash
# Sim must still pass the canonical stress sweep using the ported module
python3 stress.py --sim-time 60 --sim-time-per-cell-side 3

# AND: golden output match between Python and C++ for that module
python3 tools/compare_module.py <module_name> --seed 42 --ticks 100
```

The compare tool runs the SAME inputs through both Python and C++
implementations and asserts the outputs match within tolerance (1e-6 for
deterministic outputs, 1e-3 for floating-point accumulation).

## Build system

**CMake** with multi-target:

```cmake
project(micromouse CXX)

# Core lib (host AND embedded)
add_library(micromouse_core STATIC
    interfaces.cpp planner.cpp algorithm.cpp slam.cpp
    pose_fusion.cpp path_controller.cpp tunables.cpp)

# Host: Python bindings (called from sim)
if (TARGET_HOST)
    find_package(pybind11 REQUIRED)
    pybind11_add_module(micromouse_native bindings.cpp)
    target_link_libraries(micromouse_native PRIVATE micromouse_core)
endif()

# Embedded: XIAO adapter
if (TARGET_XIAO)
    add_executable(micromouse_xiao main.cpp hardware/xiao_nrf52840.cpp)
    target_link_libraries(micromouse_xiao PRIVATE micromouse_core)
    # ... Zephyr or Arduino-CMake plumbing here
endif()
```

Build commands:
```bash
# Host (for sim)
mkdir build-host && cd build-host
cmake .. -DTARGET_HOST=ON -DCMAKE_BUILD_TYPE=Release
make
# Drop build-host/micromouse_native.cpython-3*.so into the project
# so `python3 tester.py` picks it up.

# XIAO (for hardware)
mkdir build-xiao && cd build-xiao
cmake .. -DTARGET_XIAO=ON \
    -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake
make
# Flash build-xiao/micromouse_xiao.{hex,uf2} to the board
```

## Dependencies (C++)

| Lib | Purpose | Why this one |
|---|---|---|
| Eigen 3 | 3×3 matrix ops for SLAM | Header-only, mature, fast, well-known. |
| nlohmann::json | tunables.json load/save | Header-only, idiomatic API. |
| pybind11 | Python bindings | Industry standard for C++/Python interop. |
| Zephyr RTOS *or* Arduino core for nRF52840 | XIAO drivers | Either works. Zephyr is more modern; Arduino has bigger community + simpler build. **Choose Zephyr** — better long-term, official Seeed support. |
| GoogleTest | Unit tests for C++ core | Standard. |

No STL containers from the embedded target — use Eigen::Matrix and plain
arrays. STL on Cortex-M4 with newlib is workable but bloated.

## Testing strategy

### Three layers of tests

1. **Unit tests (C++):** Per-module C++ tests. Mock interfaces; test
   pure logic.

2. **Golden-output tests (Python ↔ C++):** Run a fixed input sequence
   through both implementations, assert outputs match. Catches translation
   bugs.

3. **Stress sweep:** The existing Python stress.py harness, but with the
   pybind11-wrapped C++ core. Same goal/collision criteria as today.

### Migration safety

- Keep Python implementations alongside C++ during migration.
- Add a `core_backend` tunable: `"python"` (today's behavior) or `"cpp"`
  (C++ via pybind11).
- Stress sweep runs both, compares results, alerts on divergence.
- Don't delete Python until C++ has shipped on hardware and is stable.

## Risks

| Risk | Mitigation |
|---|---|
| Floating-point divergence between Python and C++ | Use the same precision (float on embedded, double on host); tolerate small drift in stress comparison (1e-3 m). |
| pybind11 overhead on the Python side | Measure: typical pybind11 call overhead is ~0.5-1 µs. For our 50-200 Hz loop this is negligible (vastly less than the Python loop itself). |
| Zephyr learning curve | If too steep, fall back to Arduino-CMake. Lose some build niceties; gain ecosystem familiarity. |
| Driver maturity for VL53L0X / LSM6DS3TR-C in C++ | Both have C drivers in vendor SDKs (ST has reference drivers). Port to C++ wrapper is mechanical. |
| Stress sweep takes longer to run (because sim still in Python) | C++ core is dramatically faster, so sim sweep should be FASTER, not slower. |
| Loss of REPL / interactive debugging on hardware | Use SEGGER J-Link + GDB. Less convenient than CircuitPython REPL but standard for embedded. |

## Decision matrix: do we need this?

Three SHIP-able outcomes from the Python optimization plan:

- **Outcome A:** Python optimization (Layer A+B+C) gets the SLAM tick under
  budget on XIAO with all modes working. **No C++ rewrite needed.** Most
  likely outcome.

- **Outcome B:** Python optimization gets close but race mode (200 Hz)
  still over-budget on the hardest seeds. **Selective C++ for the SLAM
  module only** (port slam.py → slam.cpp + pybind11 binding, keep the rest
  in Python). 1-week effort, 50× speedup on the one hot module. **Most
  likely "we need some C++" outcome.**

- **Outcome C:** Even targeted C++ on SLAM doesn't help (because the
  planner or controller is also tight). **Full C++ rewrite per this plan.**
  Least likely.

The architecture above supports all three. Even Outcome B fits cleanly —
just port slam.py as the first C++ module and stop. Outcome C is the
extension if needed.

## Calibration: when to start

**Trigger**: Path 1 stress sweep shows post-optimization race mode still
> 10% overrun rate on the XIAO target after Layer A+B+C have shipped.

**Pre-flight checks before committing**:
- Hardware is in hand and behaving normally.
- We have a baseline measurement of CircuitPython vs MicroPython vs
  potential-C++ speed on the actual XIAO (not estimates).
- The competition deadline allows 3 weeks of low-feature-velocity work.

If those check out: dispatch the C++ port via Codex in 3 chunks (interfaces+tunables,
planner+slam, algorithm+path), one per week. Verify each chunk against
golden outputs before continuing.

## What this plan deliberately does NOT do

- **No partial rewrite as exploration.** Either we commit to C++ for a
  module (with bindings + golden-tests) or we leave it in Python. Half-ported
  modules are a maintenance nightmare.

- **No rewriting the sim in C++.** Python sim is fast enough; visualization
  + telemetry + iterating on tunables benefit from being in Python.

- **No abandoning CircuitPython adapter.** Keep it as the
  documented-and-tested fallback for users who don't want to flash a
  custom C++ build.

- **No new algorithm work during the rewrite.** Frozen feature set.
  Behavior delta = 0; the only goal is "same thing, faster."
