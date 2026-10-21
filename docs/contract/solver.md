# The solver

Part of the [contract](../../CLAUDE.md) the code is held to. Read it when you touch `kernels.rs`,
`system.rs`, `solve.rs`, `diagnose*`, `decompose.rs`, `cgraph.rs`. An issue number (#n) or a spec
version ([0.n], see the [language changelog](../solvent-changelog.md)) says where a rule came from.

- **A recorded root choice is one record of one triangle** (`decompose::branch_record`).  `ccw(a, b,
  c)` ("c left of a→b") is the same fact as `ccw(a, c, b)` with the sign turned, so a record is
  **canonical**: point indices ascending, the sorting permutation's parity folded into the sign
  (else `apply_gauge` and `Step::stated` write different keys).  Every writer goes through
  `branch_record` — the elaborator, the plan's `branches`/`apply_branches`, `graft`,
  `Part::branches_out` — and `io::from_json` **re-records** what it reads, so older documents
  migrate on load.  `tests/decompose.rs` and `tests/order.rs` are the gates.
- **A stated root choice is enforced; a recorded one is not** (§9.6, #157, [0.160]).
  `Sketch::branches` mixes what the document states with what the plan records (a non-sticky
  replay reads the seeds' turn and writes it back), so the elaborator writes each agreed
  `ccw`/`cw` twice (`Gauges::choose_all`): into `branches`, which the plan replays, and into
  **`Sketch::orientations`**, the statements alone, canonical the same way, saved (JSON
  `"orientations"`, only where there is one) and grafted beside `branches`.  Only `orientations`
  is enforced, so no drawing whose cache disagrees with its seeds changes.  It is held as a bound
  is, by the same machinery (`solve/orientations.rs` over `bounds::carry`, the Newton crossing
  `across` is): `orientation_reading` is the verdict — the triangle's least height (twice its
  signed area over its longest side), signed by its turn, holds only beyond `bound_tol`, so three
  points the drawing puts on one line turn neither way and break it; `cross_bounds` carries the
  point of a broken one on the seeds (the last of the three, in the key's order, with a coordinate
  the solve moves; its twins and lifts with it) to its mirror in the line through the other two,
  or half that line's length onto the stated side, and `program::elaborate` elaborates again from
  there (`crossed_seeds`); `System::steer` steers a broken one after the bounds (a `Choice`, one
  budget of `STEER_ROUNDS`), from the start and then the solution, kept only where it solves and
  holds; still broken, `success` false, status **`ORIENTATION_BROKEN`** (7, "a stated orientation
  does not hold"; a broken bound is said first), `Diagnosis::orientations_violated` (keys; report
  `orientationsViolated`), the CLI's `orientation broken: ccw(a, b, x)`
  (`report::orientation_text`).  `PlanSolver::solve` replays a stated choice as stated over its
  cache, falls back to the numeric solve where its root breaks one, keeps no `ORIENTATION_BROKEN`
  result, and writes the stated sign back to `branches`.  The block pass's root search
  (`blocks::Search`) reads bounds only.  A drag (`retry` off), a drawing stating none and one whose
  seeds keep theirs solve to the same bits; a traced body's are the locus's at its anchor
  (`locus`), and its drawn instance stands where they hold.  The component classifier still
  applies them itself to the roots its probes find (`boundary::search`: a probe steers nothing).
  `tests/orientation.rs` is the gate.

- A constraint may own *unknowns* of its own: a `SpecKind::Param` slot in its `spec`, allocated
  by `Sketch::add` and moved by the solver like any other parameter.  The slot holds a seed
  number on the way in (what a document stores, `graft` copies, and `constraints::seed_param`
  supplies when omitted — the `Param` counterpart of `infers_arg`) and an index into
  `Sketch::params` once added.  It is not a stated value: `describe` leaves it out, both bindings
  publish it read-only, `same_constraint` ignores it.  `Sketch::remove` retires an orphaned one to
  `fixed` (a parameter no equation mentions is no DOF); the rebuild walk reclaims the slot.
  **A contact's place may be shared** (#70 part 2): `t == s` over a declared unknown (`param s:
  Angle hint(318)`, or an unbound formal) is `syntax::Arg::Tie` seeded by the declaration
  (flattener; a `hint(t:)` beside it is refused in `assemble`), then
  `Arg::Shared`, which `Sketch::add` turns into one `Param` per name (`Sketch::shared`, a
  `SharedPlace`: the unknown and its curve) for every contact on that curve (E040 otherwise, and
  for a name a dimension also reads: `Fault::Place`). `remove` retires it with its last owner;
  `graft` and `lift` read `Sketch::owned_arg`, the JSON `"shared"`. `tests/shared_contact.rs`.
  **A point at a fraction of a line's run or an arc's turn** (#248, 0.82): `m coincident(t == 0.25)
  g` is `PointAlongLine` / `PointAlongArc` (twins in space `PointAlongLine3` / `PointAlongArc3`,
  `reading::in_space` carrying the place), chosen in `infix_op` only where `t` is written (a pin or
  a seed, `relations::settle`'s `sel`), so plain `coincident` pays no column; kernels written once
  over `Num` (`kernels::arc_bearings`, which `taylor`'s arc length reads too). `CKind::place_on` /
  `Constraint::place` are the one reading of a contact's place on any carrier (spline, curve, run):
  `contact_carrier`, `param_scale`, `System::new`'s scale; `fraction_slots` / `fraction_contact`
  the runs' own; `seed_param` (the point's foot, its bearing), `contact_speed` (the run's length).
  A line's runs on past its ends (as `PointOnLine`); an arc's is clamped to `[0, 1]`
  (`clamp_contacts`; `System::on_arcs` turns the rehome on) and a pin outside is E040 (`constraints::on_the_arc`, asked by `validate` and before
  `seed_omitted`); a circle with `t` is E040. A shared fraction may span lines and arcs
  (`shared_on_one_curve`), never a curve's parameter. A traced body takes a line's fraction pinned
  (`compile_trace`: a pinned `Param` slot a scratch Param on a `w` tape column; the flattener
  keeps a pin's text in symbolic mode,
  `settle_arg`). `tests/fractions.rs` is the gate (with the lofts probes' stations).

- `Param::scale` is the world length one unit of a parameter is worth — 1 for a coordinate or
  radius, the curve's mean speed |C'| for a curve parameter (read by `System::new`).  `System`
  solves in `z = x * col_scale`, so trust region and minimum-norm step measure world units;
  unscaled, tangencies stall as a drawing grows.  All-1 systems take the untouched path.
  The sparse step is `(JᵀJ + εI) p = −g`, ε of the largest diagonal (`newton::normal_step`), which
  holds back a direction whose eigenvalue is near ε, so a DogLeg converging on one creeps to its
  iteration limit (the spiral bevel's polish: a hundred iterations at 0.86 each).  A document's
  solve refines that step toward the Gauss–Newton step (`newton::refine`, iterated Tikhonov: the
  error is `ε p`, each correction one solve with the factorisation in hand) while ε holds it back
  more than `REFINE_TOL` of `‖g‖`; a step ε barely touches is left to the bit, nothing is added
  along the null space, so the least change stands, and a drag (`SolveOpts::document` false) is
  never refined.
- **A dense column is factored last** (#440, `sparse::order`).  `Ata` orders JᵀJ by reverse
  Cuthill–McKee, which on a column touching most of the others (one unknown every instance
  reads: a free margin, a pitch) puts all it touches in one level and interleaves every instance
  with every other, a factor close to dense and a solve cubic in the instances.  A column of more
  than `max(16, 10·√n)` neighbours (AMD's rule) is left out of the search and appended last in
  index order, so it costs one row of L.  With no such column the order is RCM's on A as it
  stands, and nothing in the library has one.  `tests/many_instances.rs` is the gate: the factor
  of the plates with the margin free within 2× the fixed plates', at 100 and 200, and no case of
  the library with a dense column.
- "Solved" is `System::max_relative_residual <= 1e-6`: each row's residual over its own units,
  its kernel's power of **its own size** (#492, `system::Sizes`, `System::row_size`): the longest
  side of the box round the points it reads, or its largest radius, at least a tenth of the
  largest number it reads (`system::OFF_ORIGIN`: rounding is in proportion to that, and the
  fixtures solve to 1e-12) and a unit, at most `max(1, extent)`, and the larger of its size where
  the system was compiled and where it is judged.  A point standing more than `system::FAR_OUT`
  (100) times the size of the rest a row reads outside their box is left out of the row's size
  (`x` far out on a run, `(x − s) × (e − s)`, is judged at the run's size); a row reading no
  point and no radius is judged at the extent.  Never one absolute threshold — kernels are of
  degree 1 and 2 — and **never the drawing's extent alone**, which one far-out point stretches:
  seeded 1e6 mm out, a contact a micrometre off its 20 mm circle read as 5e-14, and a belt of
  equal circles held to meet, which has no solution, as solved with its claim a theorem.  **The
  iteration steps on the extent**: `residuals_into` and `compute_csr` divide every row by
  `row_scale` (`max(1, extent)^degree`), so the dogleg weighs an `angle` row and a `distance` row
  alike (issue #43), and `System::judged` multiplies a row back to its own size.  A test comparing
  against a kernel multiplies by `row_scale` to get the raw residual.  A failed document solve
  whose every row holds over the extent and not over its own size stopped where the extent read
  what it left as solved (a gradient of `1e-16 / extent` after one step); it is solved on with
  `row_scale` at the rows' own sizes (`System::scale_rows`, `System::at_own_sizes`), kept where
  that settles nearer to holding, no more than `RESIZE` times as wide (a point run off to
  infinity reads as holding at any measure), and put back over the extent either way.  The
  violated rows (`constraint_errors`), a claim's test and a witness block's read the same sizes;
  the rank keeps the extent (below).  Corpus: no solve's outcome moves (`turbine_blade`'s areas,
  in the 16th digit: a failed station's retry kept by the new measure); `maxResidual` is it.  `tests/far_seed.rs` is the gate, `tests/size_measure.rs` the measurement.
  **A rate's rows carry the degree of what they compare** (#365, `contact_rate::Word::degree`, by
  the source's `Form`): a carried point's rate across or along a line a velocity, degree 1, a
  length a radian of the roll, so the rows stay regular where the point stands still (on the axis
  of a screw of no pitch); a traced curve's tangent and a meshing's path of contact the line's
  direction against the unit rate, cosines, degree 0, as their own kernels were; a curve's in its
  view `(C′ × (B − A))/‖B − A‖`, degree 1; a rate's distance, degree 1; a direction against a
  surface's normal (`normal`, `plane`, `in_plane`) cosines, degree 0; a section's circle and a
  curve's centre of curvature, lengths.  A rate never owns an unknown: its slot is its contact's, and its
  other columns are that contact's and its source's.
  `System::stationary` tells apart two non-solutions: at a stationary point the residual is what
  the constraints cannot agree on (a conflict); elsewhere the solver merely stopped, the
  diagnosis solves a scratch copy, and a consistent drawing is `State::Unsolved` — no conflict
  set, no culprits, the unsatisfied rows listed.
- Its Jacobian twin: rank and null space are judged on `System::conditioned` — hard rows over
  `extent^(degree−1)`, columns in world length — against one absolute, dimensionless
  `system::RANK_TOL`.  Never a raw `J`, never relative to `σ₀` (the largest row may be in another
  figure).  `Conditioned` is the only matrix the *diagnosis* judges, taking only an absolute
  tolerance; the `Tol`-taking factorisations are `pub(crate)`.  (`decompose::deficiency` and
  `homotopy` keep a relative rank on their own synthetic matrices.)  `jacobian_dense` stays the
  raw `∂r/∂z` for solvers and finite-difference checks.
- A tangency whose contact the drawing already holds to the curve is stated *at* that point, or it
  is a double root: `PointOnCircle(p, C)` + `TangentLineCircle(L, C)` with p an endpoint of L is
  rank-deficient at *every* solution.  `TangentLineCircleAt(line, circle, at)` (the
  `tangent_arc_line` kernel) is regular.  **Which At-form a contact means is one table**,
  `constraints::tangency_at` (#5: an arc's own end first, `TangentArcLine`; then a line's against a
  rim, `TangentLineCircleAt`; two lines `parallel`; two round things none; `at:` always an end of
  the operand written first), matched over every `Run` a kind given ends must name
  (`EntKind::ends`), so a new way to run stops the build there.  A chain's threaded joint asks it,
  and `commands::cTangent` through the registry's `tangencyAt` (`C.tangencyAt`) once it finds where
  the two already meet.  For degenerate pairs that still arise, the diagnosis and the witness
  settle-test the surplus motions: step along a null direction, settle, and a motion that walks back
  is `shaky` — out of the DOF, "blocked at second order", never painted under- or over-constrained.
  `numeric_rank` already includes them; nothing adds `shaky` back.  Every guard is inside
  `witness::screen`: at a solution, a tangency present, removals capped at what the matching cannot
  account for, at most `SCREEN_MAX`.  A double root's singular value is as small as a real
  freedom's, so a candidate is recognised only by trying it.
  **The tangent at a point of any path** (#251): `g tangent(at: m) l`, the parser reading `at:`
  as a point unless it is `start`/`end`/`p1`/`p2` — of a spline or a curve the contact's **rate**
  there held along the line, `CKind::Rate` (#365, the one kernel every rate of a contact runs,
  `contact_rate`: `spline_tangent_line`'s direction row over `C′` from the spline's or the curve's
  frame), of a circle or arc `TangentCircleAt` (the
  `tangent_arc_line` kernel over `[at, centre, line]`), of a line `parallel`. The spline's and
  curve's own `t` is **the contact's**: `Sketch::add` hands it the place of the contact holding
  `m` on `g` (`Constraint::tangent_at`/`holds_on`, `Sketch::holding`), whichever comes first
  (`share_place` re-points an earlier tangent), whatever it arrived carrying, so a graft, JSON and
  a lifted program (which never writes its `t`) share it again; `remove` keeps a place free while
  any constraint owns it. None holding it is E040 (`relations::tangents_at`). A place is a list of
  unknowns (`tangent_at` returns its slots, `holds_on`/`holding` its Params): one along a curve,
  three on an envelope in space (`[u, v, t]`, #283, `analytic.md`). A line and a spline
  or curve touch whichever is written first (`settle` puts the curve first). A **`path`** formal
  (`Ty::Path`) is an entity formal judged by kind at elaboration: what the call gave it must be a
  line, arc, circle, spline or curve (`Expansion::paths`, `relations::paths`, E103 at the
  argument). `tests/tangent_at.rs` and `tests/curve_contact.rs` are the gates.
- **A tangency at a contact is no tangency along a line of no length** (#479).  The
  `tangent_arc_line` kernel is `(p − c)·(b − a)`, the line's ends' difference unnormalised, so it
  holds for every contact once the ends meet: a run tangent at both ends (`engine.parts.Span`, the
  conrod's flanks) has a root with both ends at the crossing of its two rounds, isolated and of
  full rank, which no rank, diagnosis or witness can tell from the tangent.  So the line a row
  reads only by its direction is named (`Constraint::direction_line`: `TangentArcLine`,
  `TangentLineCircleAt`, `TangentCircleAt`), and `solve::collapsed` lists every acting one whose
  line is no longer than `COLLAPSE_TOL` (the solve's own relative tolerance) of the extent.  A
  document solve ending there fails, status `COLLAPSED`, never reported solved; the plan solver
  falls back to the numeric solve where its root collapses one and keeps no `COLLAPSED` result;
  `Diagnosis::collapsed` names them (report `collapsed`, the CLI's `degenerate`).  Nothing searches
  for the tangent from there: the seeds choose it.  A drag (`retry` off) is not judged.
  `tests/collapsed_tangent.rs` is the gate.
- `Horizontal`/`Vertical` level a line; a `Level` along a view's own axis levels a *pair of
  points* with the same line kernels, and `cgraph` gives it a `virtual_line` in the ground
  x-axis's direction class, so a levelled pair decomposes rather than falling to the residue.

- A **`claim`** (Solvent §9.7) is *judged, never solved for*: **no** `System` compiles a row for
  it, and `cgraph`, `graft::Part` and the witness's jitter skip it, so it never moves geometry,
  welds drag parts, or paints Over or Conflict.  `Constraint::acts()` is the named half, asked by
  `hard_constraints`/`hard_ids`; seams needing a row index spell it out inline, as `soft` is.
  Anything learning what determines the drawing (`known_radii`, entity colouring, conflict
  candidates, `duplicated`) must go through `acts()`.  The diagnosis reads it at the *end* of
  `diagnose_with` via `System::conditioned_with`, stacking its rows onto the compiled system:
  *theorem*, *violated*, or *consuming* (holds only by the pose).  Never judge it by compiling a
  second `System` — a compile calls `locus::forget`.  A claim may not own a `Param` slot nor bind
  a free variable (`CKind::claimable`; E040 at elaboration; document readers drop the flag;
  `expr::evaluate` refuses the binding).  The flag travels through `graft`, JSON (`"claim"`,
  written only when set) and both bindings.  `peaucellier.sv` is the case, `tests/claim.rs` the
  gate.  **A `claim { … }` block** (#460, [0.151]) is one claim over witnesses of its own, the
  drawing having held them (language.md): after the plain claims, `diagnose::witnessed` solves
  each block's witnesses by its rows (and the witnesses' intrinsic rows) on a copy holding every
  other number — a compile of its own, the one `locus::forget` a block costs — and tests each row
  at its own size no larger than the drawing's extent from *before* that solve (#492), so a
  witness run off to infinity cannot loosen its own test.  None holds: *violated*, the
  witnesses left as they were.  Else they are written back and the block is a *theorem* iff
  `rank [J_m 0; G_p G_w] = rank J_m + rank G_w`
  (`System::conditioned_over`, a column per witness unknown after the free ones) — every motion
  the drawing allows is followed by one of the witnesses; with no witness it is the lone claim's
  test — and *consuming* otherwise.  Every row of the block is filed under the one verdict.
  `io::describe` prefixes `claim `; `callout.rs` draws a claimed dimension
  **parenthesised** — the whole label (`(R50)`, never `R(50)`), wrapped *before* measuring.
  **A verdict is shown where the claim is written, quietly**: a wash behind the statement
  (`app/program.ts::marks`, `.claim-proved` / `.claim-refuted` / `.claim-independent`), no banner.
  *proved* (entailed), *refuted* (this drawing is a counterexample), *independent* (true here, not
  implied) — never "inconsistent".  `CodeEditor` carries a *set* of overlapping marks; a mark
  tints and **never changes the face** (background/box-shadow/outline, not border/padding).  Run
  `npm run overlay` (headless Chrome, the real `CodeEditor`) when you touch `editor.ts`;
  `make test` cannot.
- A **bound** (Solvent §9.6, 0.60, #109) *chooses the root*: `distance(>= d)`, `distance(<= d)`,
  `distance(in: (a, b))` and `p inside P` / `p outside P` (point, plane: the ordinate along `n`
  from `P.origin`, `<=`/`>=` 0, lowered by `relations::side_of_plane`; `Written::side`).
  Parsed as `OpArg::Bound { cmp: syntax::Cmp, hi }` before the number's `OpArg::Dim`, which every
  number reader keeps reading; `RelationForm::bound`, `ResolvedRelation::bound`, the canonical
  form's `bound`; checked by `relations::bound_of` (`CKind::boundable`: ordinate, distances; not a
  claim; E040). `Constraint::bound` compiles **no row**: `Constraint::states_rows()` (false for a
  claim or a bound) is what every seam that compiles, counts or welds rows asks (`System::new`,
  `n_residuals`, `cgraph`, `witness`, `graft::Part`), and `acts()` is `!soft && states_rows()`;
  callouts skip it, the witness drops it, `topology_key` marks it `~`. `Constraint::reading` is the number that
  would make the row hold (secant along the number); `solve::bound_reading` (with `bound_tol`) is
  the one verdict — holds, edge, aim — the steering, the elaborator and the diagnosis share.
  **It chooses the seed's side first**: `program::elaborate` reads the bounds on a pass's seeds
  (`solve::cross_bounds`: the measured point, the last point operand with a coordinate the solve
  moves, carried by Newton steps on the reading to its mirror, `Bound::aim`, its twins —
  `Sketch::twin_group` — and lifts with it: `across`) and elaborates again with those
  numbers written over the point's seed right after `entities::places` (`crossed_seeds`; held from
  `settle_deferred`), so `axes_along`, the views folded through it and every seed read through
  them stand as if the source had seeded it so — patching placement in place does not reproduce
  `views::place`. Then `System::solve` ends in `steer`: with `retry`, a bound still broken is
  crossed from the start pose (accumulating) and then the solution, and solved again, kept only if
  it holds; else `success` false, status `BOUND_BROKEN` (named by the diagnosis, the CLI's `bound
  broken`). `PlanSolver::solve` falls back to it where the plan's root breaks a bound. A drag
  (`retry` off) and a drawing whose seeds keep their bounds solve to the same bits. `Diagnosis::bounds_violated`/`bounds_active`, report `boundsViolated`/
  `boundsActive`; JSON `"bound": {cmp, hi}`; `graft`, `lift`, `describe` (`syntax::bounded`).
  A free name in one is E040 (`expr::Fault::ClaimFree`) unless a row of the drawing reads it too:
  `expr::evaluate` asks the bounds last, binding `Constraint::free` without minting (0.64). A
  `ccw` the document states is steered alike (#157: *a stated root choice is enforced*, above).
  `tests/bounds.rs` is the gate.
  **A bound that steers nothing** (#345 S3, [0.115]): `CKind::steers` is `boundable` less
  `RegularWithin`, a flank's least area factor read at a point a search finds — no point of the
  drawing to carry across its edge, no root to choose.  `solve::bounds::broken`, so `cross_bounds`
  and `steer`'s crossings and its block search, and `blocks::Search`'s bounds read only bounds
  that steer; the verdict reads every bound (`bounds_hold`), so one broken is still `BOUND_BROKEN`.
  Steering it would restart every block for nothing (a broken regularity cost the Formate pair two
  minutes a solve).
- `same_constraint` is "says exactly the same thing"; `same_relation` is the same *without* the
  numbers.  A repeated *relation* is refused by the app (`edit::applyConstraints`): equations
  without rank.  A **dimension is never deduped by the UI**: redundant or contradictory is the
  diagnosis's reading (`over` naming both).  `gcs_constraint_stating` is the core's question;
  `commands::dimension` states, `commands::editDimension` opens an existing one (double-click).
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
  (`Diagnosis.warnings`).  Above `NUMERIC_MAX` (300) free parameters the whole SVD is not
  taken, since it runs after every edit: the rank is read **part by part** (#88,
  `diagnose/parts.rs`, `Diagnosis::by_parts`) over `System::block_order` — over part first,
  square blocks in solve order, under part last — each factored alone (pivoted QR where it is
  only asked for a null space, `linalg::null_full_row_rank` for the under part, SVD otherwise),
  every part's null vectors carried into later parts by minimum-norm solves and checked against
  the rows a part could not make up: exact, not a bound (`tests/numeric_parts.rs` holds it to the
  whole SVD over the library; the large cases in the slow tier).  The second-order screen runs
  on its null space as on the whole's.  The V-twin (925) reads in ~0.2 s.  A consistent dependency is `over` ("remove one") only when a
  dimension takes part in it — editing that dimension is the next conflict; one among pure
  relations is a theorem that nothing can break, so its wholly-implied constraints are
  `implied`: noted, never painted as an error.

- Decomposition maps constraints onto F–H elements in `cgraph::build`; a new constraint type is
  either an edge (PP/PL), a direction relation, or `unsupported` (numeric residual).  Merge
  decisions use generic-rank at witness poses; chirality of PPP merges is the triangle
  orientation sign from the current sketch.
- Replays are warm-started on the current geometry (leaves re-derived each frame), so the root a
  sketch is on is "nearest the identity"; alternatives are applied by writing geometry, not by
  caching transforms.

- A drag is an operation on the dragged point's *part* of the document (`graft::Part`): what is
  reached through shared points and constraints, stopping at fixed entities.  `PlanDrag` builds its
  plan, systems and numeric fallback on that part alone and writes each frame back, so a drag
  costs the figure, never the document; anything exchanged by point index (guards, flips, branch
  keys) crosses through the part's maps.
- A drag made `PlanDrag::on` the document's own `PlanSolver` (`view.plan()`, cached per topology
  and pinned for the gesture) starts with one pass over the residuals
  (`PlanSolver::ensure_solved`) and runs on the document itself; `PlanDrag::new` builds a plan
  over the part.  The plan comes back with every `move_to`/`guard_triangles`/`branches` (`None`
  for a drag of its own), and `part` is `None` exactly while the drag moves the document directly.
- Within the part, a frame costs the *region*: `decompose::Wave` moves the plan's roots as rigid
  bodies — those holding the dragged point, growing by shared elements only while the cursor is
  out of reach — with every element shared with the rest an anchor and every direction class the
  rest carries pinning a rotation.  Pull (cursor row) then polish (anchors only, min-norm from the
  pulled pose) on the tiny merge system, rotations about each body's centroid scaled by
  `TURN_COST` × its gyration radius so a free body slides rather than spins.  The wave keeps its
  poses and never re-reads what it moved, so nothing compounds; it starts solved (`PlanDrag::new`
  solves first) and hands over to the numeric `Drag` only for unsupported constraints, a region
  past `WAVE_MAX`, or a body solve that will not converge.
- In `decompose`, a direction class is a *relation*, not an *adjacency*: merge candidates, the
  worklist refresh and the core frontier come from shared elements (`neighbours`), and the class
  counts toward rank once on the table (`pair_rel`, `relation_bound`) — as adjacency, one
  `Horizontal` class made every cluster a neighbour of every other.  `relation_bound` must stay an
  upper bound on the merge rank: validate a change by forcing the factorisation on every call and
  asserting `rank <= bound` across the cases.

- `solve::Drag` is the one point-drag implementation (pull + polish), `RadiusDrag` its scalar
  counterpart for circle/arc radii (a `Radius` with `soft` set — its residual is already
  r − target, so no kernel of its own); the front end only translates coordinates.
  `solve::TurnDrag` turns a body whole (a shift-drag): no soft row pulls a plane's attitude, so
  every free number the body carries is **put** where the rigid turn of its start puts it — a
  point's place, a frame's `o`, an axis's `d` and the foot `a` of the turned line — and the hard
  rows polished alone (re-homing their contacts), the turn taken in steps of at most
  `TURN_STEP`, over the part the body can move (`graft::Part`, as a drag's numeric fallback is),
  written back once a move.  It turns in space when a
  frame, an axis or a point in space is in the body (about the eye's up and right, `turn_seen`),
  else in the plane its first entity is drawn in (`sweep`, the angle the pointer swept about the
  pivot); the pivot is the middle of what the body holds, else of all it carries (a lone plane's
  origin).  `tests/turn.rs` is the gate.

- **Block-triangular solve** ([docs/block-triangular-solve-plan.md](../block-triangular-solve-plan.md)):
  `System::block_order` (memoised per compile) is the DM-matched square part in strongly
  connected blocks (`graph::blocks`: Tarjan, lowest row first), over/under parts beside it.
  `System::subset` evaluates through the whole system's helpers, so its numbers match to the bit,
  and writes only the free columns its rows read (`Subset::reads`, `System::apply_cols`): every
  parameter it reads is held, free among them or derived from bases among them, so the bits are
  `apply_z`'s, and a block of four rows costs four columns, not a document's every one (#459);
  `BlockTr` minimises one block with `newton::dogleg`, compiling no `System`.
  `SolveOpts::blocks`: `Rescue` (default) runs the pass and a whole-system DogLeg polish after a
  failed DogLeg, before LM, only with `retry` and ≥2 blocks, kept only if it solves; `First` is
  for measurement only (it changes bits and roots). `BLOCK_XTOL` 1e-15, accepted at
  `acceptance_tol`. The spiral bevel relies on it (`hypoid_layout.rs` holds rough starts).
- **Words choose roots; the block pass searches for them** (#217, F8): a block is accepted when
  its rows meet `acceptance_tol` **and every bound it decides holds** (`solve::blocks::Search`: a
  bound whose columns are all solved so far, some the block's own); where not, it is started again —
  each broken bound's point carried across it (`across`) and the block mirrored across the plane
  that point crossed (an offset's other hand), from its solution and its start, then `RESTARTS`
  starts with its points scattered about what the blocks before it placed, at their size, and its
  axes turned (a fixed `rng::Rng` stream per start) — each a DogLeg, else followed along the
  Newton homotopy `r(z) = (1 − t) r(start)` (`follow_block`, `HOMOTOPY_STEPS`); a drawing stating
  no bound has no root to search for. Constants read off the pose (`CKind::reads_pose`) are
  refreshed before each block (`refresh_consts`) and put back for the polish. A
  steered solve whose crossings leave a bound broken runs the pass from its start (not under
  `BlockMode::Off`). **A drawing is judged at its own size**: a failed document solve ending more
  than `RESIZE` times the extent it was compiled at is compiled there and solved again from its
  start (`System::resized`), so an unseeded start about the origin at unit size reaches a drawing
  hundreds of millimetres across. **Constants are read again where a solve ends**
  (`solve_unsteered`, `REFRAMES`): a row in space taken across a line (`parallel`, a point on a
  line, a plane over a line's hidden axis) is measured along directions across the line as
  compiled, and a line turned square to them leaves a root of the frame and none of the drawing's;
  so a succeeding document solve with such a row refreshes and, where its rows no longer hold,
  solves on.
  **A chart is gauged where a failed solve ends** (`System::recharted`, `Sketch::charts_moved`):
  one chosen where an unseeded point started can lie in the tangent plane the solve ends at;
  changed, the system is compiled again and solved on, kept only where it succeeds. (A point on
  a meshing needs no recovery of its own: it is seeded where its block comes up, below.) An angle's
  bound is held to the relative tolerance itself, not the length one read as radians
  (`bound_reading`): an offset's side about a far apex is a turn of 1e-4.
  `Parallel3` frames across its second operand where that is held and the first not. **A skew
  distance is a magnitude** (`|s| − d`, `line_line3`, consts `[d, pinned]`): its `sign` slot is no
  longer read off the seed (0, inferred), so both sides are roots and a bound chooses; ±1 pins.
  `tests/words_alone.rs` is the gate (each change's small case), the hypoid its large one.
- **A drawing holding contacts on envelopes solves block by block** (#283, #304): a document
  solve whose sketch holds a `PointOnEnvelope` or `PointOnMeshing` (`blocks::holds_contacts`)
  runs the pass first (`First`, not `Rescue`), since a whole-system minimisation reaches a root
  on another sheet of an envelope that every word allows.  Each block's contacts are seeded at
  its start (`seed_contacts`: the place and the lift sought again with the flanks where the
  blocks before it left them, `seed_param`/`meshing_seed`), but only where the compiled rows are
  more than `SEEDED_OFF` off, so a converged place keeps its sheet; a bound the start alone
  breaks is crossed before the block is solved (`Search::start_within`, its own parameters
  moved), and a solved block whose frame moved is refreshed and solved on (`REFRAMES`).  That
  is the whole of it: no recovery seeds a contact again after a failed solve.  A line along a
  meshing's common normal or path at a point (#345) owns nothing and is not seeded: it reads its
  contact's unknowns as that contact's block leaves them.  Gates:
  `spiral_bevel/contact.sv` solves from its seeds (`meshing::the_mean_points_bearing_…`), and a
  point held to an envelope whose cylinder is placed by relations away from its hints lands where
  it is drawn, at a roll within the bounds, not on the far side or the inner sheet the hinted
  cylinder's seed leads to (`held_envelopes::an_envelope_contact_starts_where_…`).
- **A block is torn at its declared unknowns** (#283, `blocks::torn`): a block finding a free
  number with a declared seed (`param w: Length hint(…)`, an unbound formal so seeded) from a
  contact on an envelope is solved as a designer would: the number held, the rest of the block
  in its own block-triangular order (`tearing`: one or two statements left over whose rows
  number the unknowns, the rest square and perfectly matched, the widest inner block smallest),
  swept until still (`TEAR_SWEEPS`), and the rows left over brought to zero by damped Newton
  over the numbers alone, by differences (`TEAR_DIFF`, `TEAR_STEPS`, `TEAR_HALVINGS`).  At most
  `TEAR_MAX` (2) numbers; tried first in such a block, after a failed whole solve elsewhere, and
  `z` is left as it was where it fails.  The tearing is a fact of the compile's structure, worked
  out once (`System::tearings`; a steered solve passes the blocks again): each choice is matched
  alone and compared by its widest part (`graph::widest_block`), only the one chosen ordered.
  **Where no statements can be left over for all of them, a block finding them from a contact
  is torn at each one alone** (#345 S4, `torn_at`), the later column first and the other only
  where it does not come out, the number not held found inside with the geometry.  A lag stated
  of a meshing's contact is ten rows a tearing can leave over only whole, so two ease
  curvatures, one read off that lag and one off the angle of the contact's path (one row), have
  no tearing together; the structure allows holding either, and which comes out is the search's:
  `spiral_bevel/meshing_problem.sv` comes out torn at its lengthwise ease (the later), its
  profile ease found inside from the lag's contact, from seeds on either side of what it comes to
  (`meshing_problem::the_pair_solves_to_its_ease_from_its_seeds`); solved whole, that block's
  Newton wanders off from seeds a few percent from its root.  A block that fails whole elsewhere
  is torn only at all its numbers, as before.
  A seed of a place (`seed_param`, `meshing_seed`) is a grid search, and its last `SEEDS_KEPT`
  answers are remembered against their whole question (`infer::Recent`): the elaborator's passes
  and a steered solve ask the same ones again.  The spiral bevel's face mill is the case: its pressure
  shift and cutter radius are found by balancing undercut margins (`hypoid_machine.rs`).
- **A pass through every block is polished in its own frames** (`blocks_then_polish`, #283): the
  constants it read last stand for the whole-system DogLeg that follows, since a point it held on
  a line turned since compiling is off that line by the turn in the compiled ones; they are put
  back where the pass broke or the polish fails, so a pass not kept leaves the system as it was.
- **A bound reads its number as solved**: `Constraint::reading` of a constraint whose number is
  a free variable reads the number where it stands, not the seed, so `distance(>= R)` with `R`
  found is decided at the found `R`.
- **Settled** ([docs/iteration-limit-rescue-plan.md](../iteration-limit-rescue-plan.md)):
  `SolveResult::settled` is `success && status != 4`; a DogLeg accepted on its iteration limit
  can be 1e-3 of the extent off. With `retry` and `Rescue`, `System::block_rescue` tries the block
  pass on an unsettled DogLeg: a failure from the start, kept if it succeeds; a limit stop from
  the stop, kept if it converges (status 0), else from the start, kept if it settles, else the
  first pass that settled, else the stop bit for bit. Drags (`retry` off) never see it.
  **Only a settled solve is an answer** (#449): a recovery of a failed document solve
  (`RECOVERIES`, `System::recompiled`) is kept only where it settles, since a recompile changes
  the yardstick — a point run off towards infinity (two equal circles' parallel tangents held to
  meet) widens the extent `resized` compiles at until a contact 0.13 mm off its 20 mm circle
  reads as 3e-8, and was accepted on the iteration limit.  `SolveResult::outcome` is the one
  wording — *solved*, *stopped at the iteration limit*, *did not solve* — that `solventc`
  prints and the report's JSON carries (`settled`, `outcome`); its exit code and its exports
  read `settled`; the ABI's result is nine numbers, `settled` last (`write_result`, the
  binding's `RESULT_LEN`).  The diagnosis cannot tell from a pose whether a solve settled there,
  so the caller says (`DiagnoseOptions::settled`, true by default; `solventc` and the page pass
  the solve's): where not, no claim is judged on the pose (no list holds it), the conflict
  search runs even where every row reads as holding, and a drawing it finds no conflict in is
  `Unsolved`.  The diagnosis's own scratch solve (`stationary` above) counts only where it
  settles.  `tests/unsettled.rs` is the gate.  A pose *already* far out when the system is
  compiled settled under the old yardstick — every row over an extent that point stretched —
  and so passed every guard here; rows are now judged at their own sizes ("Solved" above,
  #492).
- **A drawing over a declared parameter is followed, not solved once** (#391 S4): continuation
  (`follow.rs`) marches the drawing's own solve along the parameter from its interval's start, one
  branch carried by the Jacobian's orientation and a bounded corrector step, and stops at a
  singular instant, saying whether the motion turns back there or two branches cross. The
  diagnosis counts per parameter the freedom it drives (`Diagnosis::parameters`). The contract —
  step control, what is a branch jump, how a singular instant is reported — is
  [parameters.md](parameters.md)'s.
