# Implementation status

Stages 0–5 of [`gcs-solver-program.md`](../gcs-solver-program.md): a residual-formulation solver,
structural diagnosis (matching / Dulmage–Mendelsohn / pebble game / minimal conflict sets),
Fudos–Hoffmann-style decomposition into cached solve plans, the witness configuration method
(theorem-type dependencies, animated remaining DOFs), dragging robustness and solution management
(sticky chirality, plan drag with continuation, order-type guards, homotopy enumeration of
alternatives).

The solver exists **once**:

| | where | what |
|---|---|---|
| **core** | [`rust/gcs-core/`](../rust/gcs-core/) | the whole engine — model, kernels, solvers, linear algebra, diagnosis, decomposition, witness analysis, homotopy, document I/O.  No dependencies. |
| **ABI** | [`rust/gcs-ffi/`](../rust/gcs-ffi/) | one flat C ABI, built twice: a native `cdylib` and a self-contained `wasm32-unknown-unknown` module |
| **TypeScript** | [`web/src/core/`](../web/src/core/) | a thin binding over WebAssembly — proxies over handles, buffers for hot-path numbers, JSON for ragged results |
| **app** | [`web/src/app/`](../web/src/app/) | an HTML5-canvas sketcher, the only front end |

The binding contains no algorithm.  It generates its constraint classes from the core's own
registry, so a new constraint type appears in the browser with nothing to change on that side.
The one place two implementations are still compared is
[`rust/gcs-core/tests/linalg.rs`](../rust/gcs-core/tests/linalg.rs), which checks our QR /
complete-orthogonal / SVD / LU against `nalgebra` — on purpose, because there is no LAPACK
anywhere in the project.  The library has no dependencies; its *tests* have one reference
implementation, and it is a dev-dependency so nothing it brings links into the cdylib or the wasm.

## The core (`rust/gcs-core/`)

* `model.rs`, `constraints.rs` — `Param` (one scalar DOF), `Point` / `Line` / `Circle` / `Arc`,
  and `Sketch` (ordered param and constraint lists ⇒ deterministic).  A constraint is
  `(kind, args)` where the args follow the type's `spec` — (attribute, kind) pairs that drive
  JSON I/O, the constraint list, value editing, the toolbar applier, duplicate detection and the
  witness's dimension jitter.  Identity is an integer everywhere (a Param is its index, an entity
  is `(kind, index)`, a constraint is a monotonic id), which is what lets the bindings intern
  proxies and keep `is` / `===` meaning what they always did.
* `expr.rs` — **dimension expressions**: a `Length`/`Angle` argument may be text — `w = 80`
  names its value, `h = w / 2` and `sin(h * 10)` read names — parsed by a small recursive-descent
  parser (`+ - * / ^`, parentheses, `pi`, the usual functions; trigonometry in degrees, like
  every angle a person reads here).  The names make a graph over the document's dimensions and
  `evaluate` is a topological (Kahn) walk of it, earliest in the document first among the ready
  ones: every value is written into its argument (radians for an angle) and the report lists
  each expression with its name, what it reads, and its error — defined twice, not defined, on a
  cycle, not a number.  One that cannot be computed keeps its last number, so the solver always
  has a constant; the report says what is wrong.  Documents save `{"expr", "value"}`; the
  callout shows `h=40` or `=0.342`, the constraint list `h = w / 2 = 40`.
* `kernels.rs` — one **vectorized residual/Jacobian kernel per constraint type**, evaluated for a
  whole block of same-typed constraints per call.  Registration order **is** the kernel id.
  Squared distances, no sqrt; a determinant for parallel; a wrapped atan2 gap for the directed
  angle; signed distance minus the radius for tangency, with a chirality flag fixed at
  construction.
* `system.rs` — **compile-to-plan**: constraints grouped into per-kernel blocks of flat arrays,
  the Jacobian's CSR structure and duplicate-summing scatter map computed once; each evaluation
  refills `data` only.  `update_consts` pushes a moved drag target or an edited dimension into the
  plan without recompiling.
* `newton.rs` — our own **Powell DogLeg** (default) and **Levenberg–Marquardt**.  Gauss–Newton
  steps are minimum-norm, so under-constrained sketches — the normal case while editing — move as
  little as possible.  Dense path up to 120 free params; regularized sparse normal equations above.
* `linalg.rs` — Householder QR with column pivoting, the complete orthogonal decomposition
  behind the minimum-norm least-squares step (LAPACK `dgelsy`'s algorithm), a Golub–Reinsch SVD
  and an LU solve.  A rank is decided by a `Tol`: relative (`|R_ii| > rcond·|R_00|`,
  `σ_i > rcond·σ_0`) for a matrix in unknown units, absolute for the dimensionless
  `system::Conditioned` Jacobian that every rank and null space in the core is judged on.
* `sparse.rs` — `JᵀJ` assembled from the fixed CSR structure, ordered by reverse Cuthill–McKee and
  factored by an up-looking `LDLᵀ`, for sketches past the dense limit.
* `graph.rs` — Hopcroft–Karp matching, coarse Dulmage–Mendelsohn decomposition, bipartite
  components, and the (2,3) pebble game with rigid-component detection (Lee–Streinu) — plain
  integer adjacency lists.
* `diagnose.rs` — `diagnose(sketch)` → `Diagnosis`: structural rank/DOF, the over-determined block
  (redundancy suspects), under-determined parameters, per-component DOF, rigid clusters and
  redundant distances, violated constraints, the minimal conflict set (grow-then-shrink filter),
  per-entity state `well|under|over|conflict`.  "Which parameters can move" comes from the
  Jacobian null space (sharper than the generous DM under-block), and a numeric-rank cross-check
  logs theorem-type dependencies structural analysis cannot see.  Both are judged on the
  conditioned Jacobian (rows over `extent^(degree−1)`, columns in world length) at the one
  absolute `RANK_TOL`, so the verdict on a figure is the figure's alone and not the drawing's
  size or another figure's dimensions.  First-order motions the matching cannot account for are
  settle-tested (step along the null direction, re-solve): one that walks back is a double root
  — a tangency at its own contact — and is `shaky`, not DOF.
* `cgraph.rs` — the constraint graph for decomposition: point elements (coincident points
  contracted), line elements, ground (fixed points + x-axis), valency-1 edges (point–point
  distance, point–line signed distance), direction relations (all angle-type constraints as a
  weighted union-find), known radii via `Radius`/fixed/`EqualRadius`/`AnnularDistance` chains,
  virtual radius lines for arc-endpoint tangency, passive lines dropped, unsupported constraints
  listed.
* `decompose.rs` — **cluster merging → plan → replay**: pair/triple merges are accepted when the
  shared points/lines/directions determine the relative rigid transforms (rank of the merge
  Jacobian at generic witness poses, self-motions of degenerate clusters accounted for — F–H's
  triangle rule is the common case, parallels and perpendiculars need no special-casing); the
  merge sequence is the plan.  Replay: leaves from live dimension values, PPP triangle merges by
  circle–circle intersection with a sketch-guided **chirality flag**, other merges by a tiny
  min-norm Newton, unfixed roots placed by Procrustes (least change), verified against the
  compiled `System` with a numeric fallback.  **Stage 3b**: when pair/triple merging stalls, a
  *core* — the smallest subset of clusters that is rigid as a whole — is merged as one numeric
  step and tree merging resumes, so only minimal non-tree-decomposable subsystems ever reach the
  numeric solver (K₃,₃ and Henneberg-II frameworks decompose fully).
* `witness.rs` — the **witness configuration method** (Michelucci & Foufou): `make_witness`
  (every value a constraint *declares* as a dimension is jittered and re-solved from the current
  geometry, or incidences alone from a perturbed start), `analyze` → rank (pivoted QR
  cross-checked against the SVD that also yields the null space), **dependent constraints** with
  what implies them, and the **null space as motions** (rigid-body generators built from the
  model's own parameter identity, internal DOFs localised).
* `solve.rs` — `Drag` (soft pull, then a hard-only polish; continuation increments and order-type
  guards) and `RadiusDrag`, its scalar counterpart.
* `homotopy.rs`, `complex.rs` — **homotopy continuation** on a merge system in (c, s, tx, ty)
  form: linear rows fixed, a total-degree start system on the quadratic rows with the γ-trick,
  complex predictor–corrector tracking, real endpoints polished and deduplicated → the
  construction's real roots.
* `io.rs`, `json.rs` — JSON save/load (including recorded branches), deletion-by-rebuild
  (`without`), `describe`, and a small dependency-free JSON reader/writer.
* `report.rs` — the JSON views of diagnosis, witness reports, plans and constraint graphs that
  keep the bindings thin, plus the constraint-type registry they generate their classes from.
* `examples.rs` — rectangle-with-fillets, slotted link, truss, under-constrained polygon chain,
  K₃,₃, random Laman frameworks, and the conflict/redundancy cases the app's library shows.

## The binding

A `Sketch` is a handle; `Param`, `Point`, `Line`, `Circle`, `Arc` and the constraint classes are
interned proxies over `(handle, index)`; hot-path numbers (residuals, Jacobians, drag frames)
cross as raw buffers; everything ragged crosses as one JSON document.  The constraint classes
themselves are generated at load time from `gcs_registry_json`, which is why
`web/src/core/constraints.ts` knows no type by name.

```ts
const sk = new Sketch();
const p = sk.point(0, 0), q = sk.point(12, 0);
sk.add(new C.Distance(p, q, 10));
solve(sk);                     // p -> (1, 0), q -> (11, 0): least change
```

## Web app


`web/src/core/` is the TypeScript binding; `web/src/app/` is the sketcher —

* **S**elect / **P**oint / **L**ine (polyline, snapping to existing points) /
  **R**ectangle (`Sketch.rectangle` — four lines round shared corners with *three*
  perpendiculars, since the fourth follows and would over-constrain it) / **C**ircle /
  **A**rc (centre, start, end) / **3**-point arc (two ends, then a point the arc passes
  through — `Sketch.arcThrough`, which builds the circumcircle and picks the sweep
  containing that point), wheel zoom, right-drag pan; Escape steps back one stage at a
  time — stop a DOF animation, drop the points a tool has collected, leave the tool;
* **File ▸ Trace image…** puts a picture behind the drawing and the **Image** tool (`u`)
  places it: drag it to move it, drag a corner to size *and* turn it at once, `[` and `]`
  fade it.  Under every other tool it is inert, so the drawing is made straight through it.
  It is view state and not document state — not saved, not exported, not solved, not undone;
* every dimension's number is editable as text (double-click its callout or its row): a
  number, or an expression — `w = 80` names it, `h = w / 2` uses it — evaluated by the core
  in dependency order; a row whose expression cannot be computed is marked `ƒ` with why, and
  Solution ▸ Diagnose lists every expression in evaluation order;
* a measurement readout in the canvas's lower right whenever exactly two entities are
  selected: their distance from `distanceBetween` in the model (so the readout and any
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

Measured in the browser (median ms) — `npm run bench`, whose native half
(`cargo run --release -p gcs-core --bin bench`) measures the same things on the same code:

| case | free | compile | dogleg | plan replay | drag frame |
|---|---|---|---|---|---|
| rect_fillets | 26 | 0.02 | 0.33 | 0.26 | 0.21 |
| truss(50), 300 entities | 200 | 0.11 | 0.60 | 0.87 | 0.30 |
| truss(200), 1200 entities | 800 | 0.27 | 1.77 | 2.98 | 1.21 |

For scale, the pure-Python prototype this replaced solved `truss(100)` in 25.5 ms and dragged a
1200-entity truss at 9.1 ms/frame; the same work is now 0.61 ms and 1.6 ms.

One thing worth knowing: `diagnose` runs after every edit, and a dense SVD of a 1000-entity sketch
costs more than everything else put together, so the numeric rank / null-space cross-check is
skipped above `NUMERIC_MAX` free parameters (300) and the diagnosis says so.  The full witness
analysis is still available on demand from the Diagnose button.

## Stage 5 status

| criterion | status |
|---|---|
| chirality tracking: persisted per-construction roots, preferred on re-solve, "flip" per cluster | ✅ `Step.branch` / `Plan::branches` keyed stably, saved in JSON (`Sketch.branches`), sticky replay; the app's `Flip branch` flips triangle roots and tangency sides |
| continuation-style dragging | ✅ `PlanDrag`/`Drag` subdivide far cursor jumps (≤ 5 % of extent per increment) |
| order-type guards | ✅ numeric drag watches the plan's triangle orientations; retries with smaller steps, records/flags unavoidable flips |
| homotopy continuation for enumeration on small cores | ✅ `homotopy::enumerate_step` (total-degree, γ-trick; the K₃,₃ core enumerates its real realizations in ~3 s of 256 tracked paths); the app's `Alternatives…` button |
| torture suite: recorded drag trajectories, zero solution jumps; branches survive save/load | ✅ `rust/gcs-core/tests/drag.rs` (floating truss, sliding rect, pinned apex never jumps, guard flags a forced crossing, continuity under far drags, JSON round-trip of branches) |

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
| regression suite runs both paths and diffs | ✅ `rust/gcs-core/tests/decompose.rs::the_plan_and_the_numeric_path_agree` |
| 1000-entity mostly-tree-decomposable sketch in low ms from the cached plan | ✅ a 300-entity truss replays in 0.21 ms and a 600-entity one in 0.39 ms; the plan executes entirely in the core |
| non-tree-decomposable cores isolated into minimal numeric subsystems (Owen / DR-planning objective) | ✅ `find_core` in `decompose` — greedy minimal rigid subset, one numeric step, tree merging resumes; K₃,₃ + all random Laman frameworks decompose fully (an SPQR-tree split proper is not implemented — the rank-based core search plays that role) |

## Stage 2 status

| criterion | status |
|---|---|
| DOF bookkeeping per component | ✅ `Diagnosis.components` |
| Hopcroft–Karp + Dulmage–Mendelsohn → over / well / under | ✅ `graph.rs`, `Diagnosis.over`, `.under_params` |
| (2,3) pebble game: rigid clusters, redundant distances | ✅ `graph::pebble_game`, Henneberg/Laman property tests |
| minimal conflict sets (deletion filter) | ✅ `diagnose::minimal_conflict_set` — e.g. exactly the two contradicting widths |
| structural-vs-numeric residue logged for Stage 4 | ✅ `Diagnosis.warnings` (`polygon_chain`'s EqualLength cycle is the first case) |
| every failed solve → actionable diagnosis; trustworthy entity colouring | ✅ app status bar / list / colours / Diagnose dialog |

## Stage 1 status

| criterion | status |
|---|---|
| compile-to-plan boundary (flat arrays, no object model in the loop) | ✅ `System.blocks` + precomputed CSR/scatter |
| sparse Jacobian assembly, triplet→CSR | ✅ structure once, data per eval |
| own LM + DogLeg (default) | ✅ `newton.rs` |
| under-constrained = min-norm GN step | ✅ our complete orthogonal decomposition / regularized `LDLᵀ` |
| rank-revealing QR at the solution | ✅ `SolveResult.rank`, `System.rank()` (already caught a redundant EqualLength cycle in `polygon_chain`) |
| >10× scipy on the 30-entity sketch | ✅ 0.15 ms on `truss(8)` vs 2.9 ms for `scipy-dogbox` in the Stage-0 prototype (~20×) |
| 60 fps drag on a 200-entity sketch | ✅ 0.3 ms/frame at 300 entities, 1.6 ms at 1200 |
| flat `slvs`-style C API | ✅ [`rust/gcs-ffi/src/lib.rs`](../rust/gcs-ffi/src/lib.rs) — `gcs_system_new` / `gcs_system_solve` / `gcs_system_residuals` …, consumed from WebAssembly and, as the native `cdylib`, from anything else that speaks C |

## Stage 0 exit criteria

| criterion | status |
|---|---|
| rectangle-with-fillets solves | ✅ `examples.rect_fillets`, 0 DOF, |r| ~1e-24 |
| slotted link solves | ✅ `examples.slotted_link` |
| ~30-entity sketch solves | ✅ `examples.truss(8)`: 17 pts + 31 lines, ~5 ms |
| dragging feels alive | ✅ 8–20 ms per mouse-move on the examples |
| Sketch → residuals/Jacobian → solve → writeback with clean seams | ✅ `System` |
| analytic Jacobians verified vs FD | ✅ every constraint, every example |

Benchmark (`make bench`, compiled solve from a perturbed warm start):

```
sketch         free  res |  dogleg  |    lm    | compile
rect_fillets     26   26 |  0.12 ms |  0.22 ms |  0.04 ms
slotted_link     14   14 |  0.04 ms |  0.07 ms |  0.03 ms
truss            32   32 |  0.15 ms |  0.31 ms |  0.03 ms
polygon_chain    46   36 |  0.30 ms |  0.51 ms |  0.03 ms
truss_50        200  200 |  0.34 ms |  0.56 ms |  0.09 ms
truss_100       400  400 |  0.61 ms |  1.17 ms |  0.16 ms
```

## Tests

| suite | what it covers |
|---|---|
| `cargo test --manifest-path rust/Cargo.toml` | the engine: FD Jacobian checks on every constraint type, both solvers, the graph algorithms against the reference cases, diagnosis, decomposition and replay, witness analysis, homotopy, the drag torture suite, document I/O; `tests/linalg.rs` — our QR / SVD / LU against `nalgebra`; `gcs-ffi/tests/` — the ABI's panic boundary, which only the native target can check |
| `cd web && npm test` | the TypeScript binding reaching all of it, plus the ABI surface check |
