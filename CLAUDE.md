# geomsolver

Geometric constraint solver built in stages per `gcs-solver-program.md` (read it first).
Currently: **Stage 5 done**, in **one** implementation —

* **core** (`rust/gcs-core/`): the whole engine in Rust, no dependencies.  Model and constraints
  (`model.rs`, `constraints.rs`), vectorized kernels and the compile-to-plan `System`
  (`kernels.rs`, `system.rs`), our own DogLeg/LM plus the dense and sparse linear algebra
  (`newton.rs`, `linalg.rs`, `sparse.rs`), structural diagnosis (`graph.rs`, `diagnose.rs`),
  decomposition into cached solve plans (`cgraph.rs`, `decompose.rs`), witness analysis
  (`witness.rs`), drag/solution management (`solve.rs`, `homotopy.rs`), dimension callouts
  (`callout.rs`), dimension expressions (`expr.rs`), document I/O (`io.rs`, `json.rs`) and the
  reference sketches (`examples.rs`).
* **ABI** (`rust/gcs-ffi/`): one flat C ABI over the core, built twice — a native `cdylib` for
  Python and a self-contained `wasm32-unknown-unknown` module for the browser.
* **bindings**: `src/gcs/` (Python, `ctypes`) and `web/src/core/` (TypeScript, WebAssembly).
  Both are *thin*: proxies over handles, buffers for hot-path numbers, JSON for ragged results.
  Neither contains an algorithm.
* **app** (`web/src/app/`): an HTML5-canvas sketcher, the only front end.

Commands:
`make` (native `build/libgcs.dylib`), `make wasm` (`web/src/wasm/gcs.wasm`),
`make test` (cargo + pytest + mypy + the web suite), `cargo test --manifest-path rust/Cargo.toml`,
`.venv/bin/pytest`, `.venv/bin/mypy` (strict, must stay clean), `cd web && npm test`,
`make bench`, `cd web && npm run serve`.

Conventions:
- **The core owns every algorithm.**  A change to the model, a constraint type, diagnosis,
  decomposition or the solvers lands in `rust/gcs-core/` with a Rust test in
  `rust/gcs-core/tests/`.  A binding changes only when the *surface* changes.  If you find
  yourself writing geometry or numerics in Python or TypeScript, it belongs in Rust instead.
- Every new constraint type = a vectorized kernel in `kernels.rs` (added to `KERNELS`; the
  registration order **is** the kernel id) declaring its `degree` — the power of length its
  residual carries, 1 for a signed distance and 2 for a squared one — a `CKind` variant in
  `constraints.rs` declaring its `spec` (constructor args as (attr, kind) pairs), `params()`,
  `consts()` and `default_arg()`, and a row in `rust/gcs-core/tests/jacobians.rs` (FD check,
  spec round-trip).  Both bindings generate their classes from `report::registry_json`, so
  neither needs touching.
- A type whose spec carries a `Length` or an `Angle` is a *dimension*, and dimensions are drawn.
  `callout.rs` matches `CKind` exhaustively in two places, so adding any type stops the build
  there: give it a `Pen` arm (its drafting figure) and a `frame` arm (the `Frame` its placement
  is written in), or list it in `undrawn!`.  `every_dimension_is_drawn` then checks the arm you
  wrote actually produces a figure.
- "Solved" is `System::max_relative_residual <= 1e-6`: each row's residual over its own units
  (`extent^degree`).  Never one absolute threshold for the whole system — half the kernels are
  linear in length and half quadratic, so one threshold is wrong for one of the halves.
- An argument the core reads off the geometry (a tangency's side or sense) declares
  `CKind::infers_arg`; the registry publishes a null default for it so a binding leaves it
  omitted and the core fills it in.  A binding that substitutes a constant picks the branch.
- Mutating a constraint's constants (drag target, edited dimension) must be followed by
  `System::update_consts` / `refresh_consts` on any compiled system, or a recompile.
- `System` is the compile-once / evaluate-many seam; keep the object model out of the hot loop.
  The bindings own a handle — call `dispose()` when you drop one (the drags, the plan solver and
  diagnosis all do).
- Diagnosis is structural (matching/DM); numeric rank cross-check is the only thing that sees
  theorem-type dependencies — that residue is Stage 4's corpus, keep logging it
  (`Diagnosis.warnings`).  It is skipped above `NUMERIC_MAX` (300) free parameters because it
  runs after every edit.  A consistent dependency is `over` ("remove one") only when a
  dimension takes part in it — editing that dimension is the next conflict; one among pure
  relations is a theorem that nothing can break, so its wholly-implied constraints are
  `implied`: noted, never painted as an error.
- Deletion, copy and paste are one rebuild walk (`io::graft`): every surviving entity is
  renumbered into the destination and every reference follows, and a constraint comes along
  exactly when all its entities did.  `without` keeps what is not deleted, `copy` keeps the
  selection (so a clipboard is an ordinary sketch document), `paste` grafts one sketch onto
  another at an offset.  A new thing that travels with a constraint or an entity — a flag, a
  placement — belongs in `graft`, or the three will disagree.
- Decomposition maps constraints onto F–H elements in `cgraph::build`; a new constraint type is
  either an edge (PP/PL), a direction relation, or `unsupported` (numeric residual).  Merge
  decisions use generic-rank at witness poses; chirality of PPP merges is the triangle
  orientation sign from the current sketch.
- Replays are warm-started on the current geometry (leaves re-derived each frame), so the root a
  sketch is on is "nearest the identity"; alternatives are applied by writing geometry, not by
  caching transforms.
- Slow tests are gated by `GCS_SLOW=1` (Python) and `#[ignore]` (cargo).
- Benchmark on a quiet machine (`uptime`); this box often has a JVM indexer at 300% CPU.
- Dimension callouts (`callout.rs`) are geometry, so the whole figure — extension lines, heads,
  radial leaders, angular arcs, the label's box and the hit test — is laid out in the core and
  the front end only strokes what it is handed.  Sizes are screen-constant through `unit`, the
  world length of one screen pixel — as is the pick tolerance, so a front end never converts.
  Where a callout sits is a *placement*: two numbers in a frame that follows the geometry,
  automatic until someone drags it and then `Sketch.placements` document state, saved by index
  into the constraint list (ids are not stable across a load).  The number a dimension states
  comes from `io::dimension_text`, so the drawing and the constraint list cannot print it
  differently.
- A dimension's number may be an *expression* (`expr.rs`): `Arg::Expr { text, value }` in a
  `Length`/`Angle` slot, where `value` is what the kernels read (arg units — radians for an
  angle) and `text` is what a person wrote, in the units they read (degrees).  `w = 80` names
  its value, `h = w / 2` reads one; the names make a graph over the document's dimensions and
  `expr::evaluate` is a Kahn walk of it (earliest in the document first among the ready ones),
  writing every value and reporting, per expression, its name, deps and error — a name defined
  twice, one nothing defines, a cycle, a non-number.  An expression that cannot be computed
  keeps its last number, so the solver always has a constant.  Trigonometry is in degrees.
  `expr::set_dimension` is the one write path for text (a bare number becomes `Arg::Num`, with
  the angle conversion — the app converts nothing); `Sketch::add` and `io::from_json` evaluate;
  `set_num` on an expression drops it.  Documents save `{"expr", "value"}` and accept a bare
  string; the bindings' records keep the number in `args` and put the text under `exprs`, and
  their proxies `sync()` before handing out a value, since an edit elsewhere can move it.
- A drag is an operation on the dragged point's *part* of the document (`io::Part`): what is
  reached from it through shared points and constraints, stopping at fixed entities.  `PlanDrag`
  builds its plan, systems and numeric fallback on that part alone and writes each frame back, so
  a drag costs the figure, never the document, and separate figures cost it nothing; anything a
  drag exchanges by point index (guards, flips, branch keys) crosses through the part's maps.
- A drag made `PlanDrag::on` the document's own `PlanSolver` — the app passes `view.plan()`,
  cached per topology and pinned for the gesture — starts with one pass over the residuals
  (`PlanSolver::ensure_solved`) and runs on the document itself; `PlanDrag::new` is the
  self-contained form, which builds a plan over the part.  The plan comes back with every
  `move_to`/`guard_triangles`/`branches` (`None` for a drag of its own), and `part` is `None`
  exactly while the drag moves the document directly.
- Within the part, a frame costs the *region*, not the figure: `decompose::Wave` moves the plan's
  roots as rigid bodies — the ones holding the dragged point, growing by shared elements only
  while the cursor is out of reach — with every element shared with the rest held as an anchor
  and every direction class the rest carries pinning a rotation.  Pull (cursor row) then polish
  (anchors only, min-norm from the pulled pose) on the tiny merge system, rotations about each
  body's centroid scaled by `TURN_COST` × its gyration radius so least-norm is least motion and a
  free body slides rather than spins.  The wave keeps its bodies' and anchors' poses and never
  re-reads what it moved, so nothing compounds over a long gesture; it starts from a solved
  configuration (`PlanDrag::new` solves first) and hands over to the numeric `Drag` only for
  unsupported constraints, a region past `WAVE_MAX`, or a body solve that will not converge.
- In `decompose`, a direction class is a *relation*, not an *adjacency*: merge candidates, the
  worklist refresh and the core frontier come from shared elements (`neighbours`), and the class
  counts toward a candidate's rank once it is on the table (`pair_rel`, `relation_bound`).
  `Horizontal`/`Vertical` put every levelled line in one class, so counting it as adjacency made
  every cluster a neighbour of every other.  `relation_bound` must stay an upper bound on the
  merge rank (an under-count loses a determined merge to the numeric fallback): validate a change
  by forcing the factorisation on every call and asserting `rank <= bound` across the cases.
- `solve::Drag` is the one point-drag implementation (pull + polish), `RadiusDrag` its scalar
  counterpart for circle/arc radii (a `Radius` with `soft` set — its residual is already
  r − target, so no kernel of its own); the front end only translates coordinates.
- The ABI is a panic boundary: every entry point runs inside `guard`, so a core panic becomes
  `gcs_last_error()` and a neutral return.  That needs `panic = "unwind"` in the release profile.
  `wasm32-unknown-unknown` aborts whatever the profile says, so untrusted input (a document) is
  bounds-checked in the core as well.
- Determinism: ordered containers only (`Vec`, `BTreeMap`/`BTreeSet`), never `HashMap` iteration
  in the solve path.  Every random draw comes from the seeded `rng::Rng`.
- The trust-region loop is `newton::dogleg` over a `TrustRegion`; a new thing to minimise
  implements the trait rather than copying the loop.
- Nothing in the project is auto-formatted: there is no `rustfmt.toml`, and `cargo fmt` would
  reformat every file.  Match the surrounding style by hand (100 columns).
- No LAPACK/BLAS: the QR, complete-orthogonal, SVD and LDLᵀ routines are ours, and
  `tests/test_linalg.py` checks them against numpy — the one place two implementations are still
  compared, on purpose.
