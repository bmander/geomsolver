# gcs — geometric constraint solver

Stage 0 of [`gcs-solver-program.md`](gcs-solver-program.md): a pure-Python
residual-formulation solver with an interactive drag canvas.  Python ≥ 3.14,
numpy + scipy; C/Cython comes only when profiling says so (Stage 1).

## Setup

```sh
python3 -m venv .venv && .venv/bin/pip install -e '.[dev]'
.venv/bin/pytest            # 29 tests: FD Jacobian checks, solves, determinism
.venv/bin/mypy              # strict
.venv/bin/python -m gcs.bench
.venv/bin/python -m gcs.app                     # desktop sketcher (PySide6)
.venv/bin/python -m gcs.canvas rect_fillets      # minimal matplotlib drag testbed
```

## Desktop app (`python -m gcs.app [sketch.json]`)

PySide6 sketcher: draw **P**oints / **L**ines (polyline, snaps to existing
points to share endpoints) / **C**ircles / **A**rcs (center, start, end);
**S**elect entities (shift = multi) and apply constraints from the toolbar —
Coincident, Distance, Horizontal, Vertical, Parallel, Perpendicular, Angle,
Equal (length or radius), On line, Midpoint, On circle, Tangent (line–circle,
arc–line at a shared endpoint, circle–circle), Radius, Fix.  Drag points and the
solver keeps everything satisfied.  Right panel lists constraints (click to
highlight, double-click to edit a value, Del to remove; red = violated).
File → Examples loads the reference sketches; File → Save/Open is JSON
(`gcs.io`).  Ctrl+Z undo, wheel zoom, right-drag pan, Home = fit,
Solve menu picks dogbox / trf / lm.  Status bar shows params, equations, naive
DOF, convergence and solve time.

## Model

* `gcs.model` — `Param` (one scalar DOF), `Point`, `Line`, `Circle`, `Arc`,
  and `Sketch` (ordered param + constraint lists ⇒ deterministic).
* `gcs.constraints` — each constraint owns its `params`, `residual(v)`, an
  analytic `jacobian(v)` on its local value vector, and a `spec` describing its
  constructor arguments (drives serialization and the UI).  Squared distances,
  no sqrt; tangency carries a chirality flag (`side`).
* `gcs.solve.System` — compiles a sketch to a flat evaluation plan once,
  evaluates `r(z)` / sparse `J(z)` many times, and calls
  `scipy.optimize.least_squares` (`dogbox` default; `trf`, `lm` available).
  This compile-once seam is what becomes the C core in Stage 1.  `Drag` is the
  shared interactive-drag protocol (soft pull toward the cursor, then a
  hard-constraints-only polish), used by both frontends.
* `gcs.fdcheck` — finite-difference verification harness (keep forever).
* `gcs.examples` — rectangle-with-fillets, slotted link, truss (~30 entities),
  under-constrained polygon chain.
* `gcs.io` — JSON save/load; also deletion-by-rebuild (`without`).
* `gcs.app` — PySide6 desktop sketcher (see above).
* `gcs.canvas` — matplotlib click-drag testbed.  Dragging = soft `DragTarget`
  pull + hard-only polish, both compiled once per drag (same in the app).

## Stage 0 exit criteria

| criterion | status |
|---|---|
| rectangle-with-fillets solves | ✅ `examples.rect_fillets`, 0 DOF, |r| ~1e-24 |
| slotted link solves | ✅ `examples.slotted_link` |
| ~30-entity sketch solves | ✅ `examples.truss(8)`: 17 pts + 31 lines, ~5 ms |
| dragging feels alive | ✅ 8–20 ms per mouse-move on the examples |
| Sketch → residuals/Jacobian → solve → writeback with clean seams | ✅ `System` |
| analytic Jacobians verified vs FD | ✅ every constraint, every example |

Benchmark (median, perturbed warm start, `python -m gcs.bench`):

```
sketch           params  res |    trf     |  dogbox   |    lm
rect_fillets         28   26 |   4.1 ms   |   3.7 ms  |   2.4 ms
truss (8 bays)       34   32 |   4.7 ms   |   5.3 ms  |   2.5 ms
polygon_chain        48   36 |  23.8 ms   |   3.5 ms  |   9.2 ms   (under-constrained)
truss_50            202  200 | 119.9 ms   |  54.6 ms  |  36.7 ms   <- Python per-constraint overhead: Stage 1's motivation
```
