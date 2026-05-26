# C Extension Plan — `slam.py` as a Native Module

**Status:** Planning only. Written 2026-05-23 after pure-Python optimization
hit its ceiling (~9 ms projected XIAO tick, 1.8× over 5 ms race budget).
Intended as the **minimum-invasive** path to break that ceiling without
abandoning Python for the whole codebase.

## When to invoke

Trigger if **all** are true after kernel work + Python optimizations:

1. Race-mode tick still over 5 ms budget on actual XIAO hardware
   (measured, not projected).
2. MicroPython `@viper`/`@native` either insufficient or not viable
   (e.g. driver porting cost too high).
3. We have ~1 week of focused engineering time.

If only one module needs to be fast, this is the right answer.  If many
modules need it, fall through to `CPP_REFACTOR_PLAN.md` instead.

## Goal

Replace `slam.py` (the EKF measurement update, the dominant 5-10 ms tick
cost on XIAO) with a native C module that exposes the same Python API.
Everything else stays Python.

Target speedup: **50-100×** on the hot path → drops SLAM tick from
~9 ms to ~100-200 µs on XIAO.  Well under any aggression mode's budget.

## Scope

### Port to C:

| Function | Why | Estimated effort |
|---|---|---|
| `_ray_cast_dda` | Inner loop of the SLAM tick.  Pointer-chasing + branch-heavy, but no Python objects involved.  Trivial to port. | 2 hours |
| `_measurement_vector_with_hits` | Calls `_ray_cast_dda` 3 times; trivial. | 1 hour |
| `_measurement_jacobian_analytical` | Floating-point math, no I/O.  Direct translation. | 2 hours |
| `_predict` | Matrix multiply (3×3) + Jacobian build.  Trivial. | 2 hours |
| `_measurement_update` (Kalman gain + Joseph form) | The 3×3 matrix algebra: F·P·Fᵀ + Q, inv(S), K·y, P-update.  ~80 lines of C. | 4 hours |
| 3×3 matrix helpers | `_matmul_3x3`, `_inverse_3x3`, `_transpose_3x3`, `_add_3x3`, `_sub_3x3`, `_symmetrise_cov`.  Pure arithmetic. | 1 hour |

### Stay in Python:

| Module | Why |
|---|---|
| `ScanMatchSlam` (the outer class) | Owns state across ticks, easier to debug in Python.  Calls into C for the hot work. |
| `FusedOdometry` | Already cheap (~5 µs/tick); not worth porting. |
| `KnownMap` access | The C module reads `walls[c][r][d]` via Python's C-API.  ~10ns per access; not a bottleneck after DDA limits visits to 4-12 cells. |
| Everything else | Out of scope. |

## Architecture: how the Python and C parts fit together

```
┌──────────────────────────────────────────────────────────┐
│  Python: ScanMatchSlam (outer class)                      │
│    - holds self._x (state), self._P (covariance)          │
│    - holds self._fused (FusedOdometry)                    │
│    - holds self.km (KnownMap)                             │
│    - calls _slam_native.measurement_update(...)           │
└──────────────────┬───────────────────────────────────────┘
                   │
                   ▼ (Python → C boundary)
┌──────────────────────────────────────────────────────────┐
│  _slam_native (C extension module)                        │
│    - measurement_update(x, y, theta, P, walls, z, T) →   │
│      returns (new_x, new_y, new_theta, new_P, applied)   │
│    - takes Python tuples / lists in, returns tuples out   │
│    - all hot work inline: DDA + Jacobian + Kalman update  │
└──────────────────────────────────────────────────────────┘
```

**API contract** for the C function (exactly what `ScanMatchSlam._measurement_update` calls):

```python
result = _slam_native.measurement_update(
    x, y, theta,                    # 3 floats — state mean
    P,                              # 9-tuple of floats — covariance row-major
    walls,                          # KnownMap.walls (or flat int array, see below)
    cols, rows, cell_size,
    z_front, z_left, z_right,       # 3 floats — measurements
    sensor_max_range,
    sensor_forward_offset,
    side_sensor_angle,
    R_var,
    gate_sigma_sq,
)
# Returns:
#   (new_x, new_y, new_theta,
#    new_P_as_9tuple,
#    applied_count, last_correction_mag)
```

The outer Python class wraps this:
```python
def _measurement_update(self, reading):
    T = self.t
    z = [_clamp_distance(reading.front, T.sensor_max_range_m),
         _clamp_distance(reading.left,  T.sensor_max_range_m),
         _clamp_distance(reading.right, T.sensor_max_range_m)]
    result = _slam_native.measurement_update(
        self._x[0], self._x[1], self._x[2],
        tuple(self._P[i][j] for i in range(3) for j in range(3)),
        self.km.walls, self.km.cols, self.km.rows, T.planner_cell_size_m,
        z[0], z[1], z[2],
        T.sensor_max_range_m, T.sensor_forward_offset_m,
        T.side_sensor_angle_rad,
        T.slam_measurement_noise,
        T.slam_gate_sigma * T.slam_gate_sigma,
    )
    if result is None:
        self.corrections_skipped += 1
        return
    nx, ny, ntheta, nP, applied, mag = result
    self._x[0], self._x[1], self._x[2] = nx, ny, ntheta
    for i in range(3):
        for j in range(3):
            self._P[i][j] = nP[i * 3 + j]
    self.corrections_applied += applied
    self.last_correction_mag_m = mag
```

This is 30 lines of glue Python.  The fallback (when `_slam_native` is
unavailable) is the existing pure-Python `_measurement_update` —
identical behavior.

## Build targets — three platforms

The C code itself is platform-independent.  The build/distribute story
differs:

### 1. Host (Mac/Linux, for sim)

Standard CPython extension via setuptools + a `setup.py`:

```bash
python3 setup.py build_ext --inplace
# Produces _slam_native.cpython-3*-darwin.so in the project root.
```

Imports work like `import _slam_native`.  Used by the sim (`tester.py`,
`stress.py`).  ~5 minutes to set up.

### 2. MicroPython (XIAO, IF we switch)

MicroPython supports two extension flavors:

- **`mpy` native modules** (`.mpy` files with native code).  Built via
  `mpy-cross --target=ARMv7-M -X emit=native`.  No firmware rebuild.
  Drop the `.mpy` on the board's filesystem.  Recommended.
- **Frozen native modules** (compiled into firmware).  Faster import,
  smaller RAM, but requires building MicroPython firmware from source.

Recommendation: **`.mpy` route**.  Faster iteration; the perf is
identical to frozen for our use case.

```bash
# Build for ARM Cortex-M4 (XIAO nRF52840):
mpy-cross --target=ARMv7M -X emit=native slam_native.c -o _slam_native.mpy
# Copy to board:
mpremote cp _slam_native.mpy :
```

### 3. CircuitPython (XIAO, today)

**CircuitPython doesn't support runtime-loaded native modules.**  The only
extension path is to build a custom CircuitPython firmware with the C
code compiled in.  Workflow:

1. Clone CircuitPython source.
2. Add the C module under `ports/nrf/boards/seeed_xiao_nrf52840_sense/`.
3. Build firmware: `make BOARD=Seeed_XIAO_nRF52840_Sense`.
4. Flash via UF2.

Effort: **2-3 days** for first build (toolchain setup, learning the
CircuitPython build system).  After that: trivial rebuilds.

**Strong recommendation**: if we're going to C extension, **switch to
MicroPython** at the same time.  The native-module experience is much
better, the runtime is faster (3-5× per Path 3 bench), and we get
`@viper`/`@native` decorators for other modules as a bonus.

That said, the C extension *itself* is identical code on both runtimes —
only the build/deploy plumbing differs.

## File layout (proposed)

```
slam.py                    # Python ScanMatchSlam class (unchanged API)
                           # Imports _slam_native if available; else
                           # falls back to pure-Python path.
_slam_native.c             # C implementation (new file, ~400 lines)
_slam_native.h             # Forward declarations (optional)
setup.py                   # Host build (CPython extension)
build_mpy.sh               # XIAO build (mpy-cross invocation)
tests/test_slam_native.py  # Cross-check C vs Python (random poses)
```

## Code structure (`_slam_native.c` outline)

```c
#include "py/obj.h"
#include "py/runtime.h"
#include <math.h>

// --- Inline helpers (static, file-local) -----------------------------------

static inline void matmul3x3(const float A[9], const float B[9], float C[9]) {
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            C[i*3+j] = A[i*3+0]*B[0*3+j]
                     + A[i*3+1]*B[1*3+j]
                     + A[i*3+2]*B[2*3+j];
}

static inline int inv3x3(const float M[9], float Minv[9]) {
    float det = M[0]*(M[4]*M[8] - M[5]*M[7])
              - M[1]*(M[3]*M[8] - M[5]*M[6])
              + M[2]*(M[3]*M[7] - M[4]*M[6]);
    if (fabsf(det) < 1e-12f) return 0;
    float idet = 1.0f / det;
    /* ... cofactor matrix * idet ... */
    return 1;
}

// --- DDA ray cast (the hot inner loop) -------------------------------------

typedef struct { float t; float sx; float sy; } ray_hit_t;

static ray_hit_t ray_cast_dda(
    float ox, float oy, float dx, float dy,
    PyObject *walls, int cols, int rows, float cell_size, float max_range
) {
    // Walk the grid via Amanatides-Woo DDA, checking walls[cx][cy][d] is True
    // for each cell-exit boundary.  Returns hit info.
    // PyObject access: PyList_GET_ITEM(walls, cx) -> column,
    //                  PyList_GET_ITEM(col, cy)  -> row's 4-list,
    //                  PyList_GET_ITEM(row, dir) -> Py_True / Py_False / Py_None
    // Single-thread, no GIL release needed.
    ...
}

// --- Python entry point ----------------------------------------------------

static mp_obj_t measurement_update(size_t n_args, const mp_obj_t *args) {
    // Unpack args, call ray_cast_dda 3x, compute Jacobian + Kalman gain,
    // return new (x, y, theta, P, count, mag) as a tuple.
    ...
}

static MP_DEFINE_CONST_FUN_OBJ_VAR(measurement_update_obj, 16, 16,
                                    measurement_update);

// --- Module table ----------------------------------------------------------

static const mp_rom_map_elem_t slam_native_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR__slam_native) },
    { MP_ROM_QSTR(MP_QSTR_measurement_update), MP_ROM_PTR(&measurement_update_obj) },
};
static MP_DEFINE_CONST_DICT(slam_native_globals, slam_native_globals_table);

const mp_obj_module_t slam_native_user_cmodule = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&slam_native_globals,
};
```

The CPython extension version is similar but uses `PyMethodDef` and
`PyArg_ParseTuple` instead of MicroPython's `mp_obj_t` API.  ~80% code
share; thin platform-specific shim at the bottom.

## API choice — walls as Python lists, or as a flat int array?

The hot path looks up `walls[cx][cy][dir]` ~10× per ray cast × 3 rays =
~30 lookups per tick.  Each PyObject lookup is ~10-50 ns on Cortex-M4 in
MicroPython.  So ~1 µs of overhead per tick.  Tolerable.

If we want to squeeze further: copy walls into a flat `bytes` /
`bytearray` at the start of each measurement update.  4 bits per cell ×
1024 cells = 512 bytes.  Allocation is one Python call (~100 ns); reads
are direct memory access (~1 ns).  Saves the 1 µs of PyObject lookup,
but adds the copy cost.

Net: probably **not worth the extra complexity**.  PyObject lookup is
fast enough.  Revisit if profiling shows it as a bottleneck.

## Verification: cross-check C vs Python

Reuse the existing equivalence test:

```python
# tests/test_slam_native.py
import random, math
from slam import (ScanMatchSlam, _ray_cast_dda,
                  _measurement_jacobian_analytical)
import _slam_native

# 1000 random rays
random.seed(7)
for _ in range(1000):
    ox = random.uniform(0.05, 2.88 - 0.05)
    oy = random.uniform(0.05, 2.88 - 0.05)
    theta = random.uniform(-math.pi, math.pi)
    dx, dy = math.cos(theta), math.sin(theta)
    py_d, py_sx, py_sy = _ray_cast_dda(ox, oy, dx, dy, km, 0.18, 1.2)
    c_d, c_sx, c_sy = _slam_native.ray_cast_dda(ox, oy, dx, dy,
                                                km.walls, 16, 16, 0.18, 1.2)
    assert abs(py_d - c_d) < 1e-6, "ray-cast mismatch"
    assert abs(py_sx - c_sx) < 1e-6
    assert abs(py_sy - c_sy) < 1e-6
```

Plus a full SLAM-update cross-check (Kalman gain → state update) with
fixed seed.  ~50 lines total.

## Risk analysis

| Risk | Mitigation |
|---|---|
| **PyObject access overhead in DDA dominates** the C-side speedup, leaving us with only ~10× instead of 50× | Profile first; if needed, copy walls into a flat bytes buffer once per update.  Both options keep the API surface clean. |
| **Float precision mismatch** between Mac (double) and Cortex-M4 (float) → Kalman gain divergence | Use `float` everywhere on Cortex-M4 (hardware FPU is single-precision).  Cross-check tolerates 1e-6.  Empirically, our analytical Jacobian already agreed with central diff to 1e-8 in double; in float we'd be 1e-5 region.  Within R_var noise levels. |
| **Build system pain** (mpy-cross install, MicroPython toolchain on Mac) | Test on a Pi Pico or other readily-available MicroPython target first to debug build before tackling XIAO. |
| **Stuck on CircuitPython** for hardware compat reasons (driver maturity) | Then we MUST do a custom CircuitPython firmware build (option 3 above).  Add 2-3 days to estimate. |
| **Python fallback drifts out of sync with C** | Cross-check test in CI/pre-commit.  Single source of truth: implement in Python first, port to C verbatim, lock with cross-check. |
| **C bug causes silent wrong-state**, harder to debug than Python | Run C-mode in sim alongside Python-mode for all stress tests; assert outputs match.  Easy to A/B since the wrapper supports both backends. |

## Effort estimate

| Phase | Days |
|---|---|
| Set up CPython build (setup.py, smoke-test on Mac) | 0.5 |
| Port DDA + helpers + ray cast to C | 1 |
| Port Jacobian + Kalman gain to C | 1 |
| Port _measurement_update wrapper, cross-check vs Python | 1 |
| **Subtotal: working C extension on Mac** | **3.5** |
| Set up MicroPython mpy-cross toolchain | 0.5 |
| Build for ARMv7M, test on XIAO | 0.5 |
| Profile + tune | 0.5 |
| **Subtotal: shipped on XIAO** | **5** |
| (If CircuitPython firmware route instead of MicroPython) | +2-3 |

**Total: ~1 work-week** (5 days) for MicroPython, **~8 days** if we
must stay on CircuitPython.

## Decision tree at end of plan

Once we've measured on actual hardware:

```
Measured XIAO race tick > 5 ms?
├─ No → ship pure Python ✓
└─ Yes → MicroPython @native viable?
         ├─ Yes (1-2 day port) → try @native first
         │       ├─ Tick under budget → ship ✓
         │       └─ Still over → C extension (this plan)
         └─ No (CircuitPython lock-in) → C extension via custom
                CircuitPython firmware (this plan + 2-3 days)
```

## What this plan deliberately does NOT do

- **Does not port the whole codebase to C** — that's `CPP_REFACTOR_PLAN.md`.
- **Does not port `algorithm.py`** — controller logic is already
  cheap (~10 µs/tick) and easier to iterate on in Python.
- **Does not port the planner** — replan spikes are addressable via the
  `max_expansions` cap (already in code) or D*-lite without C.
- **Does not drop the Python fallback** — the C extension is opt-in via
  `try: import _slam_native; except ImportError`.  Without the native
  module, the code still runs (just slower).  Critical for
  cross-platform development.

## Open questions

1. **MicroPython vs CircuitPython on XIAO** — separate decision, but
   strongly affects this plan.  Recommend deciding before starting.
2. **Float vs double** on Cortex-M4 — float is faster but limits SLAM
   covariance precision.  Need a single dirty-test run to confirm float
   is sufficient.
3. **Whether to also port the analytical Jacobian or compute it in
   Python with C-returned (sx, sy)** — slight code-org choice.
   Recommend: port everything inside `_measurement_update` to keep the
   Python-side wrapper trivial.
