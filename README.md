# gcs — geometric constraint solver

Stages 0–2 of [`gcs-solver-program.md`](gcs-solver-program.md): a
residual-formulation solver with vectorized numpy kernels, our own DogLeg/LM,
structural diagnosis (matching / Dulmage–Mendelsohn / pebble game / minimal
conflict sets), and a PySide6 sketcher.  Python ≥ 3.14, numpy + scipy; C/Cython comes only
when profiling says so.

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
* `gcs.kernels` — one **vectorized kernel per constraint type**: `res(V, K)`,
  `jac(V, K)` over `(n, k)` value arrays and `(n, m)` constants, evaluating all
  constraints of a type in one numpy call.  Constant Jacobians are flagged and
  evaluated once at compile time.
* `gcs.constraints` — each constraint type is params + constants + a kernel
  reference, plus a `spec` describing its constructor arguments (drives
  serialization and the UI).  Squared distances, no sqrt; tangency carries a
  chirality flag (`side`).  Scalar `residual/jacobian` are one-row views of the
  kernel (what the FD checker tests).
* `gcs.solve.System` — **compile-to-plan**: groups constraints into per-kernel
  blocks of pure arrays (`gidx`, `consts`, row offsets) and precomputes the
  Jacobian's CSR structure and duplicate-summing scatter map; each evaluation
  refills `data` only.  `update_consts()` pushes a moved drag target / edited
  dimension into the plan without recompiling.  This plan is exactly what a C
  core would consume.  `Drag` is the shared interactive-drag protocol (soft
  pull, then hard-only polish).
* `gcs.newton` — our own **Powell DogLeg** (default) and **Levenberg–Marquardt**.
  Gauss–Newton steps are minimum-norm (least-change under-constrained motion):
  LAPACK `dgelsy` (rank-revealing QR — also reports the Jacobian rank) up to
  120 free params, sparse SuperLU on regularized normal equations above.
  scipy's `least_squares` methods remain as `scipy-*` references.
* `gcs.fdcheck` — finite-difference verification harness (keep forever).
* `gcs.examples` — rectangle-with-fillets, slotted link, truss (~30 entities),
  under-constrained polygon chain.
* `gcs.graph` — Hopcroft–Karp matching, coarse Dulmage–Mendelsohn
  decomposition, bipartite components, the (2,3) pebble game with rigid-component
  detection (Lee–Streinu) — plain integer adjacency lists.
* `gcs.diagnose` — `diagnose(sketch)` → `Diagnosis`: structural rank/DOF,
  over-determined block (redundancy suspects), under-determined parameters,
  per-component DOF, rigid clusters + redundant distances, violated
  constraints, minimal conflict set (grow-then-shrink filter), per-entity state
  `well|under|over|conflict` — "which parameters can move" comes from the
  Jacobian null space (sharper than the generous DM under-block), and a
  numeric-rank cross-check logs theorem-type dependencies structural analysis
  can't see (Stage 4 corpus).
* `gcs.io` — JSON save/load; also deletion-by-rebuild (`without`).
* `gcs.app` — PySide6 desktop sketcher (see above).
* `gcs.canvas` — matplotlib click-drag testbed.  Dragging = soft `DragTarget`
  pull + hard-only polish, both compiled once per drag (same in the app).

## Stage 2 status

| criterion | status |
|---|---|
| DOF bookkeeping per component | ✅ `Diagnosis.components` |
| Hopcroft–Karp + Dulmage–Mendelsohn → over / well / under | ✅ `gcs.graph`, `Diagnosis.over`, `.under_params` |
| (2,3) pebble game: rigid clusters, redundant distances | ✅ `gcs.graph.pebble_game`, Henneberg/Laman property tests |
| minimal conflict sets (deletion filter) | ✅ `minimal_conflict_set` — e.g. exactly the two contradicting widths |
| structural-vs-numeric residue logged for Stage 4 | ✅ `Diagnosis.warnings` (`polygon_chain`'s EqualLength cycle is the first case) |
| every failed solve → actionable diagnosis; trustworthy entity colouring | ✅ app status bar / list / colours / Diagnose dialog |

## Stage 1 status

| criterion | status |
|---|---|
| compile-to-plan boundary (flat arrays, no Python objects in the loop) | ✅ `System.blocks` + precomputed CSR/scatter |
| sparse Jacobian assembly, triplet→CSR | ✅ structure once, data per eval |
| own LM + DogLeg (default) | ✅ `gcs.newton` |
| under-constrained = min-norm GN step | ✅ `dgelsy` / regularized SuperLU |
| rank-revealing QR at the solution | ✅ `SolveResult.rank`, `System.rank()` (already caught a redundant EqualLength cycle in `polygon_chain`) |
| >10× scipy on the 30-entity sketch | ⚠ ~7.5× vs the Stage-0 scipy path (0.7 ms vs 5.3 ms, quiet machine); the rest is DogLeg's Python bookkeeping — C territory |
| 60 fps drag on a 200-entity sketch | ✅ 180-entity fully constrained ~110–160 fps, 300-entity ~50–65 fps, 1200-entity floating ~75 fps (loaded machine) |
| flat `slvs`-style C API, GIL release | ⏳ deferred to the C port — the plan arrays are the API |

## Stage 0 exit criteria

| criterion | status |
|---|---|
| rectangle-with-fillets solves | ✅ `examples.rect_fillets`, 0 DOF, |r| ~1e-24 |
| slotted link solves | ✅ `examples.slotted_link` |
| ~30-entity sketch solves | ✅ `examples.truss(8)`: 17 pts + 31 lines, ~5 ms |
| dragging feels alive | ✅ 8–20 ms per mouse-move on the examples |
| Sketch → residuals/Jacobian → solve → writeback with clean seams | ✅ `System` |
| analytic Jacobians verified vs FD | ✅ every constraint, every example |

Benchmark (`python -m gcs.bench`, compiled solve from a perturbed warm start;
measured on a heavily loaded machine — absolute numbers ~2.5× pessimistic):

```
sketch         free  res |  dogleg   |    lm     | scipy-dogbox | scipy-trf | scipy-lm | compile
rect_fillets     26   26 |  2.8 ms   |  5.5 ms   |   7.2 ms     |  6.3 ms   |  3.5 ms  | 1.1 ms
truss            32   32 |  1.9 ms   |  5.7 ms   |   8.2 ms     |  8.0 ms   |  3.2 ms  | 0.9 ms
polygon_chain    46   36 |  3.3 ms   |  4.3 ms   |   7.9 ms     | 71.5 ms   | 17.9 ms  | 0.9 ms
truss_50        200  200 | 15.1 ms   | 47.0 ms   |      —       |    —      |    —     | 4.3 ms
truss_100       400  400 | 25.5 ms   | 58.1 ms   |      —       |    —      |    —     | 5.0 ms
(Stage 0, same truss: 5.3 ms scipy-dogbox on a quiet machine; Stage 1 dogleg: 0.7 ms.)
```
