# geomsolver

Geometric constraint solver built in stages per `gcs-solver-program.md` (read it first).
Currently: **Stage 5 done**, in **one** implementation —

* **core** (`rust/gcs-core/`): the whole engine in Rust, no dependencies.  Model and constraints
  (`model.rs`, `constraints.rs`), vectorized kernels and the compile-to-plan `System`
  (`kernels.rs`, `system.rs`), our own DogLeg/LM plus the dense and sparse linear algebra
  (`newton.rs`, `linalg.rs`, `sparse.rs`), structural diagnosis (`graph.rs`, `diagnose.rs`),
  decomposition into cached solve plans (`cgraph.rs`, `decompose.rs`), witness analysis
  (`witness.rs`), drag/solution management (`solve.rs`, `homotopy.rs`), dimension callouts
  (`callout.rs`), dimension expressions (`expr.rs`), parametric curves (`curve.rs`), the
  **Solvent** language the document is written in (`syntax.rs`, `flatten.rs`, `program.rs`,
  `edit.rs`, `tape.rs`), JSON export (`io.rs`, `json.rs`) and the reference sketches
  (`examples.rs`, `gear.sv`).
* **ABI** (`rust/gcs-ffi/`): one flat C ABI over the core, built twice — a native `cdylib` for
  Python and a self-contained `wasm32-unknown-unknown` module for the browser.
* **bindings**: `src/gcs/` (Python, `ctypes`) and `web/src/core/` (TypeScript, WebAssembly).
  Both are *thin*: proxies over handles, buffers for hot-path numbers, JSON for ragged results.
  Neither contains an algorithm — and neither re-derives a number the report already carries
  (a motion's `movingParams` is the core's reading of its own velocities, not a threshold each
  binding picks), since two copies of a rule are two rules the moment one of them is edited.
* **app** (`web/src/app/`): an HTML5-canvas sketcher, the only front end.  Two halves, each
  a handful of modules rather than one slab.  The *view* is the canvas: `view.ts` holds the
  state — the camera, selection, tool, plan, diagnosis — and the modules beside it take that
  view as their first argument and do the work (`paint`, `gesture`, `tools`, `dimension`,
  `edit`), with `camera` the one that holds where the drawing sits on the canvas;
  `SketchView` keeps a one-line delegator for each verb the shell calls, so a caller holds one
  object.  The *shell* is the page around it: `shell.ts` (the elements, the view, the focused
  constraint, and where the core is started), `commands` (the constraints bar), `dialogs` (what
  the menus open), `lists` (the constraints window, the banner and the status line), `dimbox`
  (a dimension's number, edited on the drawing), `program` (the source, beside the drawing it
  makes) over `editor` (the code box it is typed in, which knows nothing of Solvent),
  `ui` (dialogs and bar widgets), and `main.ts`, which is only wiring.  `index.html` is structure and `app.css` is the whole of the styling.

Commands:
`make` (native `build/libgcs.dylib`), `make wasm` (`web/src/wasm/gcs.wasm`),
`make test` (cargo + pytest + mypy + the web suite), `cargo test --manifest-path rust/Cargo.toml`,
`.venv/bin/pytest`, `.venv/bin/mypy` (strict, must stay clean), `cd web && npm test`,
`make bench`, `cd web && npm run serve`.

`npm run build` is `tsc` (module per file, which is what `node --test` and the test suite run
against) then esbuild, which rolls `dist/app/main.js` and everything it reaches into one
`dist/app/bundle.js` — what the page loads, so splitting a module further costs the browser
nothing.  The bundle stands in for the entry module, so it is written *beside* it: `core/wasm.ts`
finds the core at `../wasm/gcs.wasm` relative to `import.meta.url`, which under bundling is the
bundle's own URL, and a bundle written anywhere else would look for it somewhere else.  The
node-only fallback imports in `wasm.ts` are left external; a browser never evaluates them.

Conventions:
- A constraint may own *unknowns* of its own: a `SpecKind::Param` slot in its `spec`, allocated
  by `Sketch::add` and moved by the solver like any other parameter.  The slot holds a seed
  number on the way in — which is what a document stores, what `graft` copies, and what
  `constraints::seed_param` supplies when a caller omits it (the `Param` counterpart of
  `infers_arg`) — and an index into `Sketch::params` once added.  It is not a value anyone
  states: `describe` leaves it out, both bindings publish it read-only, and `same_constraint`
  ignores it, since two contacts of the same point on the same curve say the same thing however
  far apart their seeds started.  `Sketch::remove` retires an orphaned one to `fixed` (a free
  parameter no equation mentions is a DOF the sketch does not have); the rebuild walk reclaims
  the slot outright.
- **The core owns every algorithm.**  A change to the model, a constraint type, diagnosis,
  decomposition or the solvers lands in `rust/gcs-core/` with a Rust test in
  `rust/gcs-core/tests/`.  A binding changes only when the *surface* changes.  If you find
  yourself writing geometry or numerics in Python or TypeScript, it belongs in Rust instead.
- A new *entity* kind stops the build in the exhaustive `match e.kind` arms — `model.rs`
  (`entity_params`, `children`, `count`, `bounds`, `distance_between`, `point_to_drawn`),
  `io::graft`'s remap and the FFI's `ent`/`kind_id`.  Give it an arm in each; `primitives()`
  and `topology_key` are where it joins the document.
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
- A parametric curve (`curve.rs`) is one that is *linear in its control points*, `C(t) = Σ Bᵢ(t) Pᵢ`
  — every B-spline, so every Bézier.  It has no usable implicit form, so a contact with one
  carries its own curve parameter as a `Param` slot and says `p − C(t) = 0`: two residuals, one
  new unknown, the net one equation the contact is worth.  A contact kernel needs the basis
  values and their first two t-derivatives and nothing else about the curve, which is the whole
  extension point — a second curve family is a second basis, not a second constraint family.
  A line against a curve is a tangency; a *circle* against one is a curvature constraint —
  `SplineCurvature` makes it the curve's osculating circle, which is the circle a draughtsman
  would call the radius there.  It is written as "the centre is the centre of curvature", which
  says touching, tangent and equally-bent all at once and needs no `side` to infer.  Dividing by
  the turning rather than multiplying by it is load-bearing: multiplied through, every row would
  vanish as `C'` did and the solver would satisfy the constraint by bunching the control points
  until the parameterisation collapsed, which it promptly does given the freedom.
  Control points are ordinary `Point`s, so they drag, snap and constrain with the tools that
  already exist, the same trick as an arc being a centre and two real points.  Local support is
  what keeps the plan's fixed-width blocks: only `DEGREE + 1` control points are non-zero at any
  t, so a contact addresses one *span*, whichever span t is in.  The span is derived from t, not
  stored, and `Sketch::topology_key` carries it — a contact walking past a knot is a recompile,
  the same event as any other topology change.
- A control polygon is edited by three operations, and they are not the same shape of thing.
  *Inserting* a control point is `curve::insert_control` — Boehm's knot insertion, so C(t) is
  identical afterwards and every contact keeps both its parameter and its place on the drawing;
  `DEGREE - 1` neighbours move, keeping their identity, and if one of them is constrained the
  next solve honours that instead, which is a stronger thing than "keep the shape".  *Deleting*
  one shortens the curve rather than destroying it: `Sketch::min_children` is the general rule
  (an entity survives while enough children do — for a line or an arc that is all of them), and
  `curve::knots_without` gives up one interior knot per lost control point, so deletion is very
  nearly the inverse of insertion.  *Interpolating* is `Sketch::spline_through_held`: chord-length
  parameters, averaged knots and one collocation solve give a control polygon whose curve passes
  through the given places.  A place that came from empty space is construction input and leaves
  nothing behind — the same bargain `arc_through` strikes with its third click; a place that came
  from a Point is *held*, by a `PointOnSpline` whose parameter is **pinned** at the value the fit
  chose.  The pin is what makes the answer determinate: a contact with a free parameter says only
  "the curve meets this point somewhere along its length", so without it a curve through m points
  keeps m degrees of freedom and could slide along itself, and a fit to fully constrained points
  would come out under-constrained.  The fit knows where along, so that is knowledge and not an
  unknown.  A pin travels in the seed: `Arg::Seed { value, pinned }` is what a `Param` slot holds on the
  way in, and `Sketch::add` consumes both halves at the one seam that turns a number into a
  Param — so a document, a paste, a rebuild and a constructor all carry it without knowing pins
  exist.  `clamp_contacts` leaves a pinned parameter alone: somebody said where along, and the
  solver is not to argue.
- A curve parameter is bounded (`t0 <= t <= t1`) and a least-squares problem cannot say so: left
  alone the solver puts a tangency on the phantom polynomial past the end of the drawn curve.
  `System::solve` says it instead — clamp, compare `curve::contact_spans` against the spans it was
  compiled from, rebuild itself when one moved — so *every* caller gets it: the one-shot `solve`,
  the plan solver's fallback and a front end that compiled a system for itself alike.  A clamped
  parameter is *pinned* for the retry: free, the next solve walks straight back off the end.
  `SolveOpts::rehome` turns it off for the one caller owning a *pair* of systems that must stay
  in step — `PullPolish`, which re-homes both together, lifting its drag target out to rebuild.
  All of it is behind an empty span map, so a sketch with no curves pays nothing.
- A block's columns and its constants are ONE compile-time choice: which span of a spline a
  contact sits on.  `System::new` makes it once and passes it to both `params_on` and
  `consts_on`, and remembers it — so `refresh_consts` skips curve contacts outright (their knots
  are document data no solve moves) rather than re-deriving a span that may since have walked.
- `Param::scale` is the world length one unit of a parameter is worth — 1 for a coordinate or a
  radius, the curve's mean speed |C'| for a curve parameter, which `System::new` reads off the
  curve itself so it is a fact about the compile and cannot go stale.  `System` gathers it into
  `col_scale` and solves in `z = x * col_scale`, so the trust region and the minimum-norm step
  measure motion in world units.  It is not a nicety: unscaled, a tangency that converges in nine
  iterations at one size stalls at ten times the size, because the t column is wrong by a factor
  of the curve's length.  Systems where every scale is 1 take the untouched path.
- "Solved" is `System::max_relative_residual <= 1e-6`: each row's residual over its own units
  (`extent^degree`).  Never one absolute threshold for the whole system — half the kernels are
  linear in length and half quadratic, so one threshold is wrong for one of the halves.
- Its Jacobian twin: a rank or a null space is judged on `System::conditioned` — the hard rows
  over `extent^(degree−1)` (a degree-`d` residual's derivative with respect to a length carries
  one power fewer), columns already in world length — against one absolute, dimensionless
  `system::RANK_TOL`.  Never a raw `J`, and never relative to `σ₀`: `σ₀` belongs to whichever
  row is largest, and that row may be a dimension in another figure entirely, so a relative
  rule lets an unrelated part of the drawing decide whether two constraints on a circle are
  dependent.  `Conditioned` is the only matrix the *diagnosis* will judge, and its methods take
  only an absolute tolerance, so the comparison cannot be written the other way — the `Tol`-taking
  factorisations are `pub(crate)` for the same reason.  (`decompose::deficiency` and `homotopy`
  keep their own relative rank on their own synthetic matrices, which are neither the sketch's
  Jacobian nor in the sketch's units.)  `jacobian_dense` stays the raw `∂r/∂z` for the solvers
  and the finite-difference checks.
- A tangency whose contact is a point the drawing already holds to the curve is stated *at*
  that point, or it is a double root: `PointOnCircle(p, C)` + `TangentLineCircle(L, C)` with p
  an endpoint of L say "tangent at p" with a Jacobian that is rank-deficient at *every*
  solution (the contact "swims" along the line to first order, blocked at second), which no
  rank tolerance can read correctly.  `TangentLineCircleAt(line, circle, at)` — the
  `tangent_arc_line` kernel reused, the radius perpendicular to the line at the named endpoint
  — is regular, and `commands::cTangent` states it whenever the picked line has an end already
  on the circle, exactly as it already did for an arc's own endpoints.  The on-circle stays the
  user's constraint; the pair is the statement.  For the degenerate pairs that still arise (an
  old document, a tangency applied before the endpoint was snapped on), the diagnosis and the
  witness settle-test the surplus motions: step along a null direction, let the solver settle,
  and a motion that walks back (a double root has no solution out there) is `shaky` — counted
  out of the DOF, reported as "blocked at second order", never painted as under- or
  over-constrained.  `numeric_rank` already includes them, so nothing downstream adds `shaky`
  back.  Every guard is inside `witness::screen`, because a caller that writes one itself will
  write it differently: the sketch must be at a solution (or the settle measures the solve, not
  the geometry), it must contain a tangency at all (the only thing a double root is made of —
  which is what keeps the solves off the ordinary theorem-type dependency, `polygon_chain`
  paying eight solves an edit for a freedom that is perfectly real), the removals are capped at
  what the matching cannot account for (so a settle that lands short can never eat a genuine
  DOF), and there are never more than `SCREEN_MAX` of them.  A candidate cannot be recognised
  more cheaply than by trying it: reached exactly, a double root's singular value is as small as
  a real freedom's, so there is nothing in the spectrum to sort on.
- `Horizontal`/`Vertical` level a line; `HorizontalPoints`/`VerticalPoints` level a *pair of
  points*, which is the same statement about the segment between them and needs no line drawn
  there.  They reuse the line kernels unchanged — those four columns were always two points'
  coordinates — and `cgraph` gives them a `virtual_line` in the ground x-axis's direction class,
  the same trick arc-endpoint tangency uses, so a levelled pair decomposes rather than falling to
  the numeric residue.
- `same_constraint` is "says exactly the same thing"; `same_relation` is the same *without* the
  numbers — same type, same entities, same flags.  A repeated *relation* is refused by the app
  (`edit::applyConstraints`): it says nothing the sketch does not already say and adds equations
  without adding rank, which the structural check cannot see.  A **dimension is never deduped by
  the UI** — not a run on a pair that already has a length, and not the same number stated twice.
  Whether a second number is redundant or a contradiction is the solve and the diagnosis's
  reading, and it comes back as `over` naming both, which is something the drawing can show and
  the user can act on; a button that guessed would instead decide it silently, and would have to
  decide from the type alone that the run somebody asked for is an edit of the length they had.
  So `gcs_constraint_stating` is a question the core answers and the front end no longer asks:
  `commands::dimension` states, `commands::editDimension` opens one already on the drawing (the
  constraint list's and the callout's double-click), and that is the whole of the distinction.
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
- **The document is the Solvent source (`gear.sv`, `syntax.rs`, `program.rs`, `edit.rs`).**  The
  drawing is what elaborating it produces, and drawing is a way to edit it.  So *every* edit is a
  **splice** — a few characters replaced in the text somebody wrote — and **never a reprint**: a
  reprint would flatten a hand-written `component` into the entities it elaborates to, throw away
  every comment and reflow every line, on the first drag.  A gear written as thirty instances of
  one tooth stays written that way while its hundred and twenty points are dragged.
  `edit.rs` is the whole of it: `commit_seeds` (a solve), `reconcile` (a gesture), `remove`,
  `set_dimension`, `add_point`/`add_entity`/`add_relation`, and `mint` for a name.
  Three edit classes, and **the core says which — the front end never guesses**: `Structural`
  (statements added or removed), `Numeric` (only numbers a solve may move, so a compiled plan
  survives — which is what keeps editing a dimension instant), `None`.
  Writeback is one lexical rule: *a seed is writable iff it is written with `=` and not `==`, is
  a literal and not an expression, and is reached by exactly one instance path.*
  `reconcile` and `retext` **apply themselves to the `Elaborated` and do not rebuild the
  drawing** — nothing about the drawing changed, the source is only catching up, and rebuilding
  would invalidate every proxy a half-finished tool is holding.  `retext` re-parses (same
  statements, same ids); `adopt` extends the map onto the statements just written.
  `reconcile` reads only the *append*: entities and constraints are appended to their vectors, so
  past the map's high-water mark is new and anything the map names that is gone was removed.  A
  mutation that **renumbers** is not something it can follow, which is why deletion is
  `edit::remove` and not `io::without`.
- Deletion, copy and paste are one rebuild walk (`io::graft`): every surviving entity is
  renumbered into the destination and every reference follows, and a constraint comes along
  exactly when all its entities did.  `without` keeps what is not deleted, `copy` keeps the
  selection (so a clipboard is an ordinary sketch document), `paste` grafts one sketch onto
  another at an offset.  A new thing that travels with a constraint or an entity — a flag, a
  placement — belongs in `graft`, or the three will disagree.  JSON is now the *export* format,
  derived rather than canonical; `io::dumps` is still what the Rust tests, the Python binding and
  the benchmarks compare against.
- A statement expanded by `flatten` **keeps the id of the statement it came from**: a `cycle` of
  thirty makes thirty things from one line, and the line is what a span points at, a caret lands
  on and a splice edits.  What tells the thirty apart is the `path` every `Site` carries.  Minting
  an id per copy made each look like a statement of its own, so every consumer that turns an id
  back into source found nothing there — and the multiplicity a fresh id hid is exactly what
  `commit_seeds` needs to see.  `Program::stmts` walks into block bodies for the same reason;
  whether a statement is one the *root* may splice on its own is a different question, asked
  against `root().body` (`edit::in_root`).
- Decomposition maps constraints onto F–H elements in `cgraph::build`; a new constraint type is
  either an edge (PP/PL), a direction relation, or `unsupported` (numeric residual).  Merge
  decisions use generic-rank at witness poses; chirality of PPP merges is the triangle
  orientation sign from the current sketch.
- Replays are warm-started on the current geometry (leaves re-derived each frame), so the root a
  sketch is on is "nearest the identity"; alternatives are applied by writing geometry, not by
  caching transforms.
- Slow tests are gated by `GCS_SLOW=1` (Python) and `#[ignore]` (cargo).
- Benchmark on a quiet machine (`uptime`); this box often has a JVM indexer at 300% CPU.
- The front end is two layers and they are kept orthogonal.  *Geometry* is the core's, and it
  is asked in world coordinates: what a click picks is `model::pick` (which measures what is
  *drawn* — a line's segment, an arc's sweep, the curve itself — as against `point_to`, which
  measures the idealised entity a dimension means, an infinite line and a whole circle), a
  callout is laid out and hit-tested by `callout.rs`, a curve is tessellated by `curve.rs`.
  *Linear algebra* is the front end's, and the whole of it is `app/camera.ts`: a similarity —
  uniform scale, translation, and the flip that comes of the canvas putting y downwards — so it
  carries lengths and angles faithfully, which is exactly what lets every geometric question be
  asked out where the geometry is.  A tolerance therefore travels as a world length
  (`PICK_PX * unit`, the same `unit` the callouts are sized through) and never as pixels.
  Nothing outside `camera.ts` multiplies by `scale` or writes a minus sign in front of a y, and
  nothing in `app/` measures a distance to an entity itself.
- Dimension callouts (`callout.rs`) are geometry, so the whole figure — extension lines, heads,
  radial leaders, angular arcs, the label's box and the hit test — is laid out in the core and
  the front end only strokes what it is handed.  Sizes are screen-constant through `unit`, the
  world length of one screen pixel — as is the pick tolerance, so a front end never converts.
  Where a callout sits is a *placement*: two numbers in a frame that follows the geometry,
  automatic until someone drags it and then `Sketch.placements` document state, **saved on the
  statement it qualifies** — `at (t, r)` after the dimension in Solvent, `"place"` inside the
  constraint object in JSON.  Never by position in a list and never by entity index (Solvent
  §13.1): both follow the position rather than the thing, and both fail silently — a callout
  reappears on another dimension, a recorded root choice goes inert while the document still
  carries it.  `io::from_json` still *reads* the old position-keyed `placements` table, so an
  older document loads; it never writes one.  The number a dimension states
  comes from `io::dimension_text`, so the drawing and the constraint list cannot print it
  differently.  A callout is painted *over* the geometry, so `callout::pick` also owns what
  outranks it: a point within the same tolerance beats the figure's lines — a radius runs its
  leader out of the centre it measures from, and the one point a circle has has to stay
  clickable once it is dimensioned — but not the number's own box, which is filled solid, and
  picking through a thing the drawing covers up would be a lie about the drawing.  The rule is
  the core's because the figure is, so a front end asks once and every front end agrees.
- One straight dimension figure, `Pen::linear`, draws them all: it measures along a *given*
  direction, puts a head where each point falls on that line and runs an extension line out to
  each point from wherever it is.  `Pen::aligned` is the case where the direction is the pair's
  own (a length: both extension lines come out the same); a run or a rise passes a page axis
  instead, and everything else follows.
- A dimension between two points is three dimensions — `Distance`, `HorizontalDistance`,
  `VerticalDistance` — and which one is *stated by where the number is put*.
  `callout::pair_dimension` picks the nearest of the three lines a dimension line could lie
  along (the pair's own, the page's x, the page's y) to the direction from the middle of the
  pair out to the placement.  Nearest, so the borders are the bisectors and there is no
  threshold; a tie goes to the length.  It lives beside the frames because it has to agree
  with the figure that then gets drawn.  The front end asks (`gcs_dimension_pair_kind` → a
  registry index) and swaps the constraint for another as the pointer moves — see
  `SketchView.startDimension`, which is also the seam where a dimension is written at all: it
  is stated at once, at what it measures, and its number is edited on the drawing where it
  will be read.  Nothing reaches the undo stack until the number is accepted, and Escape takes
  the constraint back out.  *Nothing is solved while it is being carried* either — `afterEdit`
  skips the solve while `liveDim.placing`, so stating it, swapping it for another kind and
  moving it about leave the geometry alone; `placeDimension` (the click that plants it, and
  the release that ends a later drag of it) is the one solve, and the editor stays open past
  it because where it sits and what it says are settled separately.
  The run and the rise are signed from the first point to the second
  (`(qx - px) - d`, a constant Jacobian — the best-conditioned row there is, and still so with
  the two points one above the other, which is the pose someone reaches for a run in).  So
  they do not commute, and a front end orders the pair to make the number read positive.  In
  `cgraph` they are `unsupported` on purpose: the cluster vocabulary has no element for the
  line they are really measured from.
- A dimension's number may be an *expression* (`expr.rs`): `Arg::Expr { text, value }` in a
  `Length`/`Angle` slot, where `value` is what the kernels read (arg units — radians for an
  angle) and `text` is what a person wrote, in the units they read (degrees).  `w = 80` names
  its value, `h = w / 2` reads one; the names make a graph over the document's dimensions and
  `expr::evaluate` is a Kahn walk of it (earliest in the document first among the ready ones),
  writing every value and reporting, per expression, its name, deps and error — a name defined
  twice, one nothing defines, a cycle, a non-number.  An expression that cannot be computed
  keeps its last number, so the solver always has a constant.  Trigonometry is in degrees.
  A number may be written as a mixed fraction — `3 1/2` is three and a half, the way a drawing
  writes it — which the tokenizer folds into one `Num`.  The space is what tells the readings
  apart, so `31/2` and a bare `1/2` are still divisions, and the fraction itself is written
  tight.  `expr::literal` deliberately does *not* claim it, so it is kept as text with the value
  it came to: `expr::notation` says that text is a number written a particular way rather than a
  computation, so `arg_text` — the constraint list — prints it as written where a *formula* there
  prints the text and what it came to (`h = w * 2 = 80`).  A **callout carries the expression**:
  `io::dimension_text` draws every written dimension as written, since `h = w / 2` and `3 1/8`
  each tell a reader what 40 and 3.125 do not, and what a dimension came to is the one thing a
  reader can measure off the drawing.
  `expr::set_dimension` is the one write path for text (a bare number becomes `Arg::Num`, with
  the angle conversion — the app converts nothing); `Sketch::add` and `io::from_json` evaluate;
  `Sketch::set_constraint_num` is the write path for a number, and re-evaluates when it dropped
  an expression.  Documents save `{"expr", "value"}` and accept a bare
  string; the bindings' records keep the number in `args` and put the text under `exprs`, and
  their proxies `sync()` before handing out a value, since an edit elsewhere can move it.
- A name **nothing defines is a free variable** (`expr::Free`): an unknown of the sketch rather
  than an error, so the dimensions reading it are tied to each other and what they come to is
  left to the solver — one degree of freedom where two stated numbers would have been none.
  `expr::evaluate` owns them: it allocates one Param per free name into `Sketch::free_vars`,
  retires it to `fixed` when the last reader stops reading it — keeping the *slot*, so reading the
  name again reuses the unknown rather than leaking a new one and moving the parameter count that
  `topology_key` ends with — and rewrites every binding (`Constraint::free`, at most one, which is
  why it is on the constraint and not the argument) from scratch each run, so a document, a paste
  and a rebuild carry only the text and the number and let the next evaluation work the rest out
  again.  It runs on every edit that can touch one: `Sketch::add`, `remove`, `set_constraint_num`,
  `set_dimension`, `from_json`, `report::exprs_json`.  A caller adding a *whole document* one
  constraint at a time uses `Sketch::add_quiet` and evaluates once at the end (`io::graft`,
  `io::from_json`): per-add evaluation is quadratic in the expression count, and would make a
  dimension whose definition has not arrived yet briefly a free variable.
  The tie is **affine in one free name** — `a`, `a / 2`, `2 * a + 5` — because `value = m*a + c`
  is the whole of what a fixed-width block can carry: `expr::eval` works in `Aff`, so ordinary
  evaluation is the `free: None` case and "a free name may only be scaled and offset" falls out
  of the arithmetic rather than being checked for.  `a * a`, `sin(a)` and two free names in one
  dimension are errors, and an erroring dimension keeps its last number like any other.
  Every type carrying a `Length` or an `Angle` therefore needs a *second* kernel, its free twin
  — the same rows and one more column, the unknown where the constant was, with (m, c) as the
  constants — declared in `CKind::free_kernel`, which matches `CKind` exhaustively so a new
  dimension type stops the build there; `every_dimension_can_be_written_free` checks the shape.
  Nothing else in the solve path is per-type: `params_on` appends the free column (it is always
  last), `consts_on` returns `[m, c]`, `kernel_id` picks the twin.  A fresh free variable seeds
  from the number the dimension already stated, or — when it states none — from the geometry, by
  Newton on that one row (`expr::settle`), which asks the kernel and no table.  A free dimension
  is `unsupported` in `cgraph` (the cluster vocabulary has no element for a relation *between*
  dimensions), is never jittered by the witness (it states no number to make generic), is part of
  `topology_key` (which unknown it names is a column), and joins `io::Part`'s walk (two dimensions
  sharing an unknown move together, which is as real a tie as a shared point).  `expr::sync_free`
  brings the numbers they *show* back into step with the unknown, from every seam that writes
  parameters without going through the others — `Sketch::set_x`, `io::Part::write_back` and the
  wave's direct writes — since a solve moves the unknown and a stale callout is a wrong drawing.
- The page is the drawing *and the source it is written as*: the program panel is a permanent
  second child of `<main>`, because the source is not a remark about the drawing — it is what the
  drawing **is**.  Everything else the shell has to *say* is still said beside what is picked, and
  there is no other sidebar.  A component selected names itself in the status line (`describeEntity`) and
  brings up one floating window listing the constraints that reach it, and only those.  The
  window is open on a `subject` — the selection while there is one, and otherwise whatever it
  was last opened on, so focusing a constraint (which empties the selection) does not pull the
  window out from under the pointer that clicked the row.  Nothing infers where a pick came
  from: `openPanel` is how the two things that pick without selecting say so — a callout
  clicked on the drawing, the banner's culprits — and `closePanel`, wired to `onSelect`, is a
  press that hit nothing.
- **The colouring is the parser's own scan** (`syntax::highlight`), not a second lexer in
  TypeScript: told to keep the comments and to say what each token turned out to be, so what a
  colour says a word is and what the parser makes of it cannot disagree — and a regex highlighter
  would part company on the first thing the language learned (`==` against `=`, a mixed fraction,
  a block comment, a new constraint's name, which `CKind::from_name` supplies here for free).
  `Tint` names the classes and the stylesheet says what they look like; the front end writes one
  element per run with the gaps between them plain, so it describes no whitespace and parses
  nothing.  A function of the *text* and not of an `Elaborated`, since the program being looked at
  is usually the one half-typed.
- Offsets cross the ABI in **UTF-8 bytes** and index a **UTF-16 string** on the other side, and
  `gear.sv` has an em dash in its second line — so this is not a corner case but the ordinary one.
  `core/program.ts::Offsets` is the conversion and `Document.adopt` is the **one seam** every
  report crosses: the diagnostics, the source map and the coloured runs are all string indices by
  the time anyone sees them, so no consumer holds the wrong unit and no two of them disagree.  A
  wholly-ASCII program builds nothing and answers in a comparison; otherwise it is a binary search,
  since a source map's offsets do not arrive in order.  Nothing sends an offset back the other way.
- The box it is typed in is `app/editor.ts` — a `CodeEditor`, which knows nothing of Solvent and is
  handed the text and a function saying which runs of it are what.  It is a `<textarea>` over a
  `<pre>` of the same text, because a textarea cannot colour its contents and a `contenteditable`
  gets the caret, undo and the platform's keys wrong.  Everything hard about that is one sentence:
  **the two layers must put every character in the same place**, or the caret sits somewhere other
  than the text it is in front of — and sharing the CSS is necessary and not sufficient, because
  the box is one run per line where the copy is one element per colour.  Four rules, each of which
  has already been the bug: the line height is a whole number of pixels; kerning and ligatures are
  off; a run may change the colour and **never the face** (an italic or a bold is a different font,
  and the box has no spans to match it); and the copy is **translated** to follow the box, never
  scrolled — the box carries scrollbars and the copy does not, so its scroll *range* is a
  scrollbar shorter and an assignment clamps at the bottom of a long file, which is precisely a
  caret that is fine at the top and a line out at the end.  None of it is visible to a unit test,
  so `npm run overlay` (`web/tools/overlay.mjs`) drives headless Chrome against the *real*
  `CodeEditor` and checks the three things that can break: the metrics agree, the colouring moves
  no glyph off where the plain text puts it, and the copy follows the box to both ends.  It is not
  in `npm test` because it needs Chrome, and `make test` must not.
- `SketchView` holds a `Document` (`core/program.ts`), not a `Sketch`: `view.sketch` is a getter
  for what the source came to and `view.source` is the document.  Undo is program text, so it
  restores what somebody wrote, comments and all.  A selection crosses a re-elaboration **by
  name** (`Document.nameOf` / `Document.entity`) — a proxy is interned per `Sketch` and dies with
  it; a name is what the source calls the thing.  `swap` is the one seam that replaces the
  drawing, and it disposes the outgoing document.
  The source catches up at exactly two seams and **never per frame**: `syncSeeds` at the end of a
  drag (`gesture::endGesture`, guaranteed numeric) and `syncSource` at the end of `afterEdit`.
  The panel is wired to `onProgram`, never to `onDragFrame`.
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
- A **curve family is written in the document**, not in the core.  `curve involute(c: circle,
  phase: Angle)(u) = ( xexpr, yexpr )` is a `CurveDef`: two `expr.rs` expressions over the
  geometry the curve is drawn from, compiled to a `tape.rs` tape and differentiated forward in
  `u` *and* in every coordinate they read.  `∂C/∂u` is which way a contact may slide; `∂C/∂θ` is
  how the curve moves when its geometry does, and without it a point solves once and falls off
  the moment the circle is dragged.  An involute, a cycloid and a spiral are then library code,
  not three entity kinds.
  The variable table is the parameter, then every scalar the arguments contribute **in
  `entity_params` order** (`EntKind::scalar_names`), then the numbers the instance was given —
  which is also `params_on`'s column order, so a tape's gradient *is* a row of the Jacobian.
  `EntKind::Curve` is the one kind whose children need not be points, and the one that must be
  built and grafted **last**, since its arguments may be of any other kind.
  A curve's kernel belongs to its **definition**, not its type: two families read different
  numbers of coordinates and cannot share a fixed-width block.  So `CKind::kernel()` panics for
  `PointOnCurve`, `kernel_id_in(sk)` returns `N_KERNELS + def`, `System` owns a table of the
  static kernels plus one per family, and the registry publishes `kernel: -1` — which the
  bindings' exhaustive tests key on rather than on a name.  The tapes ride in the constraint's
  `consts`, so no kernel signature learns about curves and `KERNELS` stays `'static`.
- A family's body may be a **locus** instead of a formula: `curve name(…)(u) = trace p where
  { … }` (spec §6.5.1, `locus.rs`) — the curve is wherever the block's constraints put `p`,
  which is how a person states an involute ("the end of a taut string as it unwinds") without
  ever deriving it.  The block is lowered once, at family compile time
  (`program::compile_trace`), through a scratch sketch and `Constraint::params_on` — the real
  column mapping, never a second copy — into rows of the static kernels over one variable table
  `[u, θ, values, q, w]`: `q` the block's own coordinates, `w` a dimension written over `u` and
  the geometry, computed by a tape and read by the dimension's **free twin** kernel with
  `(m, c)` the unit conversion, so no new derivative code exists anywhere.  Evaluating `C(u)`
  is a small damped Newton solve of the rows and its derivatives are the implicit function
  theorem at the solution — `∂C/∂u` and `∂C/∂θ` from one factorisation of the inner Jacobian —
  which is what keeps a contact on the curve when the geometry it is written over is dragged.
  The whole compiled block encodes to flat `f64` and rides in the contact's consts exactly as
  the tapes do (`locus::eval_flat` is the one evaluator: kernel, tessellation and tests all run
  the flat form), so `System` only picks `trace_kernel` over `curve_kernel` per definition and
  the bindings are untouched.  Chirality — which way the string unwinds — is a *branch*, and no
  regular residual can state one (a residual zero at exactly one direction has a vanishing
  gradient there).  A block states its way onto a branch by three instruments, in order of
  strength (spec §6.5.1): a **signed constraint** where the vocabulary has one
  (`point_line_distance` is signed, so "taut" written against the radius line makes the winding
  algebraic in the sign of the roll); an **orientation predicate** — `ccw(a, b, x)` in a block
  contributes no residual and selects the solution component, its third point one the block
  places, enforced only at the **home** (`trace p from (expr) where …` — evaluation's anchor,
  chosen where the predicates read unambiguously) by reflect-and-resolve, with deterministic
  restarts (fixed-seed `rng::Rng`) when there are no seeds to start from; and a **seed** for
  what neither can say.  Continuity carries the branch from the home everywhere else —
  evaluation is one warm-started march, the same walk the polyline sweep does, and a block with
  predicates never trusts a direct solve at the target (it could land in a forbidden
  component).  A seed is a *place*, named geometrically where it can be: `at c bearing (u +
  phase)` is the point at the edge of the circle, `at t` is where another point starts
  (`program::at_seed` lowers both to the tapes the coordinate spelling would be), and
  `at (xexpr, yexpr)` remains for a place with no name.
  A block must be square — as many rows as inner coordinates — or elaboration refuses it.
  `tests/trace.rs` holds the taut-string involute checked against the closed form it never
  states (with seeds wrong by 3× on purpose), and a gear run on traced flanks.
- A curve is *geometry*, so like a dimension callout it is laid out in the core and the front end
  only strokes what it is handed: `curve::tessellate` refines to `FLATNESS_PX` screen pixels
  through `unit` (the world length of one screen pixel), and `curve::closest` is the pick test
  and the seed for a fresh contact, so the two agree about where "on the curve" is.  No binding
  evaluates a basis function, and none writes the degree down: `report::registry_json` publishes
  `curve.minCtrl`, so a front end's tool and its messages cannot drift from what
  `Sketch::spline_with` will accept.
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
