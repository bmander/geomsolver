# gcs — geometric constraint solver

Stages 0–5 of [`gcs-solver-program.md`](gcs-solver-program.md): a
residual-formulation solver, structural diagnosis (matching / Dulmage–Mendelsohn /
pebble game / minimal conflict sets), Fudos–Hoffmann-style decomposition into cached
solve plans, the witness configuration method (theorem-type dependencies, animated
remaining DOFs), dragging robustness and solution management (sticky chirality, plan
drag with continuation, order-type guards, homotopy enumeration of alternatives).

It exists twice, deliberately:

| | numerics | object model, graphs, decomposition | front end |
|---|---|---|---|
| **reference** | numpy/scipy (`gcs.kernels`, `gcs.newton`) | Python (`src/gcs/`) | none — a library |
| **web** | C compiled to WebAssembly (`csrc/`) | TypeScript (`web/src/core/`) | HTML5 canvas (`web/index.html`) |

There is one interactive front end, the web one.  The Python side is the reference
implementation the port is checked against, and has no UI of its own.

The two are held together by [`tests/test_ccore.py`](tests/test_ccore.py), which
runs the C library against the Python implementation (kernels, the compiled plan's
residuals / Jacobian / CSR structure, the dense linear algebra, both solvers), and by
[`web/src/test/core.test.ts`](web/src/test/core.test.ts), which asserts the same
properties of the port that the Python suite asserts of the reference.

## Setup

```sh
python3 -m venv .venv && .venv/bin/pip install -e '.[dev]'
make                        # the C core as a native shared library, for the tests below
.venv/bin/pytest            # FD Jacobian checks, solves, diagnosis, decomposition, C-vs-Python
.venv/bin/mypy              # strict
.venv/bin/python -m gcs.bench
```

## Web build

```sh
source ~/emsdk/emsdk_env.sh   # emscripten
make wasm                     # csrc/ -> web/src/wasm/gcs.{js,wasm}   (~60 kB)
cd web && npm install
npm test                      # build + the TypeScript suite (node --test)
npm run bench
npm run serve                 # http://localhost:8123/
```

`make wasm` produces an ES module; the app is otherwise static files, so any static host
will do.  The two build outputs (`web/src/wasm/gcs.js`, `gcs.wasm` — 70 kB together) are
checked in so the web app can be built, tested and served without emscripten; rerun
`make wasm` after touching `csrc/`.  See [The C core](#the-c-core-csrc) and
[Web app](#web-app) below.

## The C core (`csrc/`)

The flat, binding-agnostic API the program's Stage 1 calls for.  The caller owns the
object model and compiles a sketch down to an evaluation plan — arrays of
`(kernel id, parameter indices, constants)` — and this library owns every number that
touches the drag loop.

* `kernels.c` — one vectorized residual/Jacobian kernel per constraint type, evaluated
  for a whole block of same-typed constraints per call.  The kernel ids are the
  contract with the front end (`gcs.kernels` and `web/src/core/kernels.ts` mirror them).
* `system.c` — the compiled plan: blocks, the Jacobian's CSR structure and its
  duplicate-summing scatter map computed once, residual / dense-Jacobian / CSR-data
  evaluation, per-constraint errors, numerical rank.
* `linalg.c` — Householder QR with column pivoting (the rank convention
  `|R_ii| > rcond·|R_00|`), the complete orthogonal decomposition behind the
  minimum-norm least-squares step (LAPACK `dgelsy`'s algorithm), a Golub–Reinsch SVD
  and an LU solve.
* `sparse.c` — `JᵀJ` assembled from the fixed CSR structure, ordered by reverse
  Cuthill–McKee and factored by an up-looking `LDLᵀ`, for sketches past the dense limit.
* `newton.c` — Powell's DogLeg (default) and Levenberg–Marquardt over either path.  The
  Gauss–Newton step is the *minimum-norm* least-squares solution, so under-constrained
  sketches — the normal case while editing — move as little as possible.

`make` builds `build/libgcs.dylib` for the Python-side comparison tests; `make wasm`
builds the browser module.  No LAPACK, no BLAS, no allocation in the inner loops.

## Web app

`web/` is the same program with the numerics in WebAssembly and everything else in
TypeScript: `web/src/core/` is a direct port of `src/gcs/` (model, constraints,
graph algorithms, diagnosis, constraint graph, decomposition, witness analysis,
homotopy, JSON I/O, examples), and `web/src/app/` is the sketcher —

* **S**elect / **P**oint / **L**ine (polyline, snapping to existing points) /
  **R**ectangle (`Sketch.rectangle` — four lines round shared corners with *three*
  perpendiculars, since the fourth follows and would over-constrain it) / **C**ircle /
  **A**rc (centre, start, end) / **3**-point arc (two ends, then a point the arc passes
  through — `Sketch.arc_through`, which builds the circumcircle and picks the sweep
  containing that point), wheel zoom, right-drag pan; Escape steps back one stage at a
  time — stop a DOF animation, drop the points a tool has collected, leave the tool;
* a measurement readout in the canvas's lower right whenever exactly two entities are
  selected: their distance from `distance_between` in the model (so the readout and any
  constraint you then apply agree on what "distance" means), plus Δx/Δy for two points and
  the angle for two lines;
* select by clicking (shift = multi) or by dragging a box over empty canvas — window
  selection, so an entity comes along only if all of it is inside (a line's two
  endpoints, a circle's whole extent, every point of an arc's sweep), previewed live
  while you drag and shift-extendable;
* the constraint toolbar (Coincident, Distance, Horizontal, Vertical, Parallel,
  Perpendicular, On line, Midpoint, On circle, Angle, Equal, Tangent, Radius, Symmetric,
  Fix, and **G** to mark lines/circles/arcs as construction geometry — drawn dashed, still
  constraining, persisted with the document).  Distance with two lines selected dimensions
  the gap between them, signed so it keeps the side you drew.  It dimensions the gap
  only — pair it with Parallel unless the rest of the sketch already forces the two lines
  parallel, which is usually the case and is why bundling the parallelism into it made
  sketches quietly redundant.  A point and a line selected together give `PointLineDistance`:
  the point's perpendicular offset from the line, also signed, and measured to the *infinite*
  line so the foot may fall past the end of the segment.  Two circles or arcs give
  `AnnularDistance`: the radial thickness of the ring between them.  None of the three creates
  the alignment it dimensions — pair them with Parallel, or Coincident on the centres, when
  nothing else in the sketch already implies it.
  Equal takes an equality *set*: n selected lines or n circles/arcs become n−1
  constraints against the first, added as one edit — one solve, one undo step;
* entity colouring by constraint state, dashed halos and labels on the culprits of a
  conflict, a banner naming the minimal conflict set, and a constraint list that marks
  redundant (`≈`) and culprit (`✗`) rows;
* drag a point with the cached decomposition plan (falling back to pull-and-polish) — the
  cursor offers a drag only where the diagnosis says something is actually free, so a fully
  constrained sketch says so rather than moving nothing; or
  drag a circle's or arc's edge to resize it — a soft `Radius` pull with the same
  polish, so a dimensioned or fixed radius does not follow while an `EqualRadius` chain
  resizes together (a relation, not a dimension); the
  Diagnose report (structural + witness), DOF animation, branch flipping and homotopy
  enumeration of alternative roots;
* JSON save/load, undo, and the case library from `examples.ts`.

Measured in the browser (Chrome, Apple Silicon, median ms) — the WebAssembly core is
faster than the numpy/scipy reference on every axis, because the per-constraint Python
overhead is gone:

| case | free | compile | dogleg | plan replay | drag frame | py drag frame |
|---|---|---|---|---|---|---|
| rect_fillets | 26 | 0.05 | 0.22 | 0.82 | 0.19 | — |
| truss(50), 300 entities | 200 | 0.11 | 1.12 | 0.68 | 0.18 | 3.9 |
| truss(200), 1200 entities | 800 | 0.43 | 5.27 | 2.13 | 0.80 | 9.1 |

One thing worth knowing: `diagnose` runs after every edit, and a
dense SVD of a 1000-entity sketch costs more than everything else put together, so the
numeric rank / null-space cross-check is skipped above `NUMERIC_MAX` free parameters
(300) and the diagnosis says so.  The full witness analysis is still available on
demand from the Diagnose button.  The Python reference has the same guard, so the two
stay in step.

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
* `gcs.cgraph` — the constraint graph for decomposition: point elements
  (coincident points contracted), line elements, ground (fixed points + x-axis),
  valency-1 edges (point–point distance, point–line signed distance), direction
  relations (all angle-type constraints as a weighted union-find), known radii
  via `Radius`/fixed/`EqualRadius` chains, virtual radius lines for arc-endpoint
  tangency, passive lines dropped, unsupported constraints listed.
* `gcs.decompose` — **cluster merging → plan → replay**: pair/triple merges are
  accepted when the shared points/lines/directions determine the relative rigid
  transforms (rank of the merge Jacobian at generic witness poses, self-motions
  of degenerate clusters accounted for — F–H's triangle rule is the common
  case, parallels/perpendiculars need no special-casing); the merge sequence is
  the plan.  Replay: leaves from live dimension values, PPP triangle merges by
  circle–circle intersection with a sketch-guided **chirality flag**
  (orientation sign), other merges by a tiny min-norm Newton, unfixed roots
  placed by Procrustes (least change), verify with the compiled `System`,
  numeric fallback otherwise.  `PlanSolver` compiles once per topology.
  **3b**: when pair/triple merging stalls, a *core* — the smallest subset of
  clusters that is rigid as a whole (greedy growth by generic-rank deficiency,
  size-capped) — is merged as one numeric step and tree merging resumes, so
  only minimal non-tree-decomposable subsystems ever reach the numeric solver
  (K₃,₃ and Henneberg-II frameworks decompose fully).
* `gcs.witness` — **witness configuration method** (Michelucci & Foufou):
  `make_witness` (every value a constraint *declares* as a dimension is
  jittered — `spec` kinds `length`/`angle` — and re-solved from the current
  geometry, or incidences alone from a perturbed start), `analyze` → rank
  (pivoted QR cross-checked against the SVD that also yields the null space,
  relative tolerance), **dependent constraints** with the constraints that
  imply them (one batched least-squares fit for all of them; theorem-type
  flagged when the structural analysis had them matched), and the **null space
  as motions** (rigid-body generators built from the model's own parameter
  identity, internal DOFs localised).  `diagnose(..., witness=True)` attaches
  it and reuses its rank/null space; the Diagnose report shows it (cached
  per diagnosis) and View → Animate remaining DOF (Ctrl+M) plays the modes.
* `gcs.homotopy` — **homotopy continuation** on a merge system (a core or a
  triangle) in (c, s, tx, ty) form: linear rows fixed, total-degree start
  system on the quadratic rows with the γ-trick, complex predictor–corrector
  tracking, real endpoints polished and deduplicated → the construction's
  real roots; `apply_alternative` puts the sketch on one (replays stay there).
* `gcs.io` — JSON save/load (incl. recorded branches); deletion-by-rebuild (`without`).

## Stage 5 status

| criterion | status |
|---|---|
| chirality tracking: persisted per-construction roots, preferred on re-solve, "flip" per cluster | ✅ `Step.branch`/`Plan.branches()` keyed stably, saved in JSON (`Sketch.branches`), sticky replay; Ctrl+F flips triangle roots / tangency sides |
| continuation-style dragging | ✅ `PlanDrag`/`Drag` subdivide far cursor jumps (≤ 5 % of extent per increment) |
| order-type guards | ✅ numeric drag watches the plan's triangle orientations; retries with smaller steps, records/flags unavoidable flips |
| homotopy continuation for enumeration on small cores | ✅ `gcs.homotopy.enumerate_step` (total-degree, γ-trick; K₃,₃ core → 4 real realizations); `Edit → Alternative solutions…` |
| torture suite: recorded drag trajectories, zero solution jumps; branches survive save/load | ✅ `tests/test_drag.py` (floating truss, sliding rect, pinned apex never jumps, guard flags a forced crossing, continuity under far drags, JSON round-trip of branches) |

## Stage 4 status

| criterion | status |
|---|---|
| witness construction (generic dimensions, incidence structure kept) | ✅ `make_witness`; the user's sketch is used directly when it already satisfies its constraints |
| rank-revealing analysis: dependent constraints incl. theorem-induced | ✅ `polygon_chain`'s EqualLength cycle and concurrent altitudes diagnosed with "implied by …" |
| null space → which DOFs remain, what they look like, animated | ✅ `WitnessReport.motions`, "Animate DOF" in the app |
| numerical-rank care: scaling, relative tolerance, QR vs SVD cross-check | ✅ (a disagreement is reported as a warning) |
| on demand for full diagnosis; automatically on Stage-3 cores | ✅ on demand (Diagnose dialog / `witness=True`); Stage-3 merge decisions already use generic-rank tests at witness poses |
| Stage-2 residue correctly diagnosed | ✅ |

## Stage 3a status

| criterion | status |
|---|---|
| bottom-up cluster merging (F–H), each merge a rigid placement from shared elements | ✅ `decompose.decompose` (generalised: generic-rank determination) |
| ruler-and-compass closed forms with chirality flags | ✅ PPP triangle merge (circle–circle) with orientation flag; other merges numeric (tiny) — closed-form library to grow |
| numeric fallback for non-constructible / unsupported | ✅ `PlanSolver.solve` verifies and falls back |
| decomposition once per topology; drags/edits replay the cached plan | ✅ `PlanSolver` + live dimension values (`refresh_consts`) |
| regression suite runs both paths and diffs | ✅ `tests/test_decompose.py::test_plan_and_numeric_agree` |
| 1000-entity mostly-tree-decomposable sketch in low ms from the cached plan | ⚠ 300-entity truss replays in ~4 ms, 1500-entity in ~22 ms in Python (≈ the vectorized numeric solve, 5 / 14 ms): per-merge Python overhead is the limit — the plan is flat data a C executor would consume |
| non-tree-decomposable cores isolated into minimal numeric subsystems (Owen / DR-planning objective) | ✅ `find_core` in `decompose` — greedy minimal rigid subset, one numeric step, tree merging resumes; K₃,₃ + all random Laman frameworks decompose fully (an SPQR-tree split proper is not implemented — the rank-based core search plays that role) |

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
| >10× scipy on the 30-entity sketch | ✅ 0.13 ms in the C core vs 2.9 ms for `scipy-dogbox` on `truss(8)` (~22×); the numpy path is ~7.5× and the remainder was DogLeg's Python bookkeeping |
| 60 fps drag on a 200-entity sketch | ✅ C core: 0.18 ms/frame at 300 entities, 0.80 ms at 1200 (numpy path: 3.9 ms and 9.1 ms) |
| flat `slvs`-style C API | ✅ [`csrc/gcs.h`](csrc/gcs.h) — `gcs_system_new` / `gcs_system_solve` / `gcs_system_residuals` …, consumed from WebAssembly and (in the tests) from ctypes |

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
