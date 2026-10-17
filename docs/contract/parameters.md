# Parameters: numbers that run

Part of the [contract](../../CLAUDE.md) the code is held to. Read it when you touch
`model/running.rs`, a `param … over`, a face family's running number, a traced body's formals, an
interval of the roll a reading states, `follow.rs` (continuation), `trace(m over t)` and what is
held to it (`followed.rs`), a drive (`motion/law.rs`, `MotionDef::Driven`,
`tape::eval_interval`), a face swept over a declared parameter or a valued frame's block
(`flatten/valued.rs`, #398 B), or a frame along a path (`motion/guide.rs`, `MotionDef::Guided`,
#398 C). The
direction is [#391](https://github.com/bmander/geomsolver/issues/391): geometry over a
parameter, trajectories rather than snapshots; this file holds what is built of it. An issue
number (#n) or a spec version ([0.n], see the [language changelog](../solvent-changelog.md)) says
where a rule came from.

- **One type for a number that runs** (#391 stage 1): `model::Running { name, dim, from, to }`,
  a parameter and the interval it runs over. Its ends are in the units a person reads,
  **degrees for an angle**, as every named number of the core is (`HeldNumber`, `Declared`).
  `Running::radians` is the one conversion: radians for an angle, as written otherwise. It is
  the `to_radians` each builder applied before the type stood between the readers and the text,
  so a kernel reads the bit it read before. `from < to` is checked where the interval is written
  (the flattener, the builders); a reader takes the ends as they stand. `tests/parameters.rs` is
  the gate, and the corpus's own tests hold that no solved value moved.
- **A declared parameter** (#391 S1, [0.139]): `param t: Angle over (0deg, 360deg)` parses to
  `ParamDecl` with `Input::over` (`syntax::Over`: the two ends' text, the word's span),
  `P::param_over`. It says what it is (`Length`, `Angle` or `Scalar`; none, or `Int`, is a syntax
  error), takes no `:=` and no `hint(…)` (syntax errors: it is neither a value nor an unknown),
  and stands where a `param` may (the parser refuses one in a component or a block; a module's is
  E040, `parameter_axis`). The flattener (`Walk::parameter_axis`, after `work_out`, so an end may
  read any number the body defines) works each end out as a constant of the declared type, finite
  — reading an unknown of the drawing, or the wrong dimension, is E103 — and the interval
  increasing (E103), and records it once: `Expansion::parameters` → `Sketch::parameters`, by
  name. **It is no unknown**: it makes no free value, no `Declared`, no column. `syntax::print`
  writes it as declared and `lift::unknowns` declares a sketch's again (ends as numbers in the
  document's units); `graft` carries none, as it carries no `declared`.
- **A declared parameter is read by a reading over the roll** (#391 S2, [0.140]), and since S4 by
  the document's relations (below): `over t` with no interval (`syntax::RunsOver::declared`) names
  it, and `Walk::declared_interval` writes its ends into the reading's `from`/`to` before they are
  settled — a parameter of another type than `Angle`, or a name the document declares none by (a
  component's body sees no parameter), is E103 at the name. Through S3 a dimension reading one was
  E103 and the relation refused (`Walk::runs`, `values::runs`, `settle_dim`, `Checks::refused`);
  that refusal now stands only where the document's relations are not — a component's body
  (`out_of_reach` says it first) — so no free variable is ever made of its name.
- **The three uses read it** (#391 S1): every number the language already ran over is a
  `Running`, with the meaning it had:
  - **a face family's running number** — `SolidDef::Family::over`, its name the absolute key the
    station is held under (`#f12.s`, `Sketch::held`), its dimension the formal's (the
    `HeldNumber`'s), its ends `over s in (a, b)`. `SolidDef::legs` reads it; `HeldNumber` stays
    the fixed parameter the station is solved at.
  - **a traced body's formals** — in space, `TracedE::over`, one per formal that runs (one for a
    curve, two for a surface), each its name and dimension in the component (`TracedE::domain`
    packs the ends for a contact's constants; a spline drawn over points in space runs `t`, a
    `Scalar`); in the plane, `CurveE::over`, the definition's swept formal (`u`, a `Scalar` over
    `(0, 1)`, for a free curve), which `curve_domain` reads unless a trim says otherwise.
  - **the shared roll** — the one number every joint turns by (#383), an `Angle`: no document
    need declare it. A reading over an interval of it holds a `Running` named as its `over` names
    the parameter (S2: `over t in (a, b)` is `t`; `Running::ROLL`, `roll`, where nothing names
    one — a reading built in Rust, `Running::roll`): an envelope in the plane (`CurveE::over`), an
    envelope in space (`EnvelopeE::roll`), a meshing (`MeshingE::roll`), a swept solid
    (`Roll::Stated`); each kernel reads `radians()`, so the name moves no value. A claim's
    sweep (`model::Sweep::over`) is a `Running` under the claim's own name, a frame's roll or a
    number of the drawing.
- **Two operations on what moves** (#391 S2, [0.140]): every reading of a frame's motion is
  evaluation at an instant or a union over an interval, each written once.
  - **`x at τ`** (`P::instant`: `at`, then `(under: m, seen_from: g)` where the frames are named,
    then the roll as a dimension's text to the statement's end, its `hint(…)`, `class` or a
    placement — `instant_ahead` tells `at (t, r)`, two numbers, from it). After a `coincident`'s
    right operand (`relation`, and `chain` for the one-joint chain a lone relation is) its
    arguments join the relation's as `OpArg::Named` and `OpArg::Dim`, the `CarriedPoint` they
    always were; `under:` on the `coincident` itself is refused beside it. As a definition's value,
    `n := x at τ` is a `Decl` with `carried` set (`P::read_at`), and what it makes depends on what
    `x` is, which the parser cannot see: `flatten::carried::ways` writes it out all three ways
    — a frame carried (`frame`), a solid placed (`solid`, `Sweep::Placed`), a point held (`point`,
    `X := point` and `X coincident x at τ`) — each statement walked with its `Scope::reading`
    tagged, and `Walk::settle_readings`, once every name is known (before `face_sets` and at the
    head of `resolve`), keeps the one the source's declaration asks for: a solid's, a point's, or
    otherwise (a datum the library draws after the document, anything not found) the frame's; a
    source that is itself a reading waits for that one. The gate is `tests/carried_frame.rs` and
    `tests/frames.rs`, their spellings changed and nothing else, and `tests/parameters.rs`
    (`a_point_read_at_an_instant_is_held_where_its_frame_carries_it`).
  - **`over t in (a, b)`** after the operand that runs (`P::runs_over`, `syntax::RunsOver`):
    `sweep(tool over t in (a, b), under:, seen_from:)` is `Sweep::Swept` (`P::sweep_decl`),
    `sweep(tool, through: body, …)` `Sweep::Rolled`, `envelope(tool over …)` and `meshing(f1, f2
    over …)` an `AngularSpan` with `over` set (`AngularSpan::running`, the name the model's
    `Running` takes). `sweep(Comp(…).f over s in (a, b))` is the face family (`P::family`, told by
    the instance written in place) and `trace(C(…).p over u in (a, b))` the traced curve
    (`curve_spec` in the brackets). The printer writes every one so (`print::instant_text`,
    `sweep_text`, `over_text`; a canonical `CarriedPoint` at a stated roll `operator_text`'s ` at`).
  - **Reparametrisation**: a sweep and an envelope are unions, read off the path alone; a meshing,
    a rate and a reading at an instant read the roll itself. Nothing in the core depends on the
    name of the parameter a reading runs over.
  - **Retired, each refused naming its replacement**: `solid(tool, at: θ)`
    (`parser::declarations::AT_RETIRED`), `solid(tool, under: m, from:, to:)` and `through:`
    (`SWEPT_RETIRED`, `ROLLED_RETIRED`; over a solid that rides, `build_solid`, E080), `carried(at:
    τ)`, `coincident(at: τ)`, an envelope's and a meshing's `from:`/`to:`, `solid(… over …)` and a
    curve written bare (`k := leg.toe over …`). The corpus is written in the new spellings and no
    solved value moved.
- **A joint's value** (#391 S3, [0.141]): a jointed frame stands at an instant of the roll,
  `fix(value == τ) f` beside its ratio and advance (`constraints::JOINT_VALUE`; the parser keeps
  the slot's text as `Arg::Dim`, so the flattener settles it as a dimension and it may read a face
  family's running number, read again at every station, or a declared unknown).  Its joint is
  turned `ratio · τ` and risen `advance · τ / 360°` from where it is drawn — linear in `τ`, ratio
  and advance its slopes — and a frame stating none stands at rest, `0deg`
  (`motions::value_of`).  The value is no number of the frame's and states no row:
  `program::motions::values` reads it off the statement into `Resolver::values`, after the
  memberships and before any reading is built, and `motions::holding` hands the `fix` the rest of
  its slots.  An Angle, checked there once (E103, as is a declared parameter read in it);
  refused: a value of anything but a jointed frame (E080), one given twice (E040), a frame with a
  value riding on another, and one a frame rides on (E080, naming `n := base in f`).  What its
  `in` block draws stands where the value puts it (#398 B, below); through [0.144] anything drawn
  there was refused.
  - **`n := x in f`, drawn in a jointed frame** (`P::drawn_in`, a `Decl::carried` whose args are
    `in: f`; the printer's `print::frame_drawn_in`; the label `syntax::DRAWN_IN`): `x` read at `f`'s
    value, the very statements `x at(under: f) τ` writes out (`flatten::carried::ways`, settled as
    an instant is), so it makes a frame, a solid or a point as `x` is.  The elaborator reads `in:`
    as `under:` with the frame's value appended as the stated roll (`relations::carried_motion`),
    and a solid's `Sweep::Placed { at: Arg::Word(DRAWN_IN) }` the same
    (`solids::build::under_motion`).  `x` is carried from where it is drawn whatever it rides on (no
    `under_riding` refusal), and what it makes rides on nothing, as a reading's product rides on its
    observer.  `p := point in P` stays a membership: the parser takes `in` after a reference that is
    no kind word.
  - **The station words are frames whose value is written over `s`** (`std.sv`): `Helix`'s screw
    `fix(value == 360deg * turns * s)`, `n := base in screw`, `axial := n in stand` (a quarter
    turn's value); `OnHelix`'s `start := base in tilt` and `n := start in screw`; `Twisted`'s
    `t := x in turn`, the turn's value `twist * s`.  Their readings at an instant are retired, and
    their rows are the readings' row for row, so `tests/carried_frame.rs` (the stations against
    their closed forms to `EXACT`) and `tests/carried_family.rs` (the two springs one tube on one
    screw) pass unchanged.  `tests/parameters.rs` is the gate for the rest
    (`what_is_drawn_in_a_jointed_frame_stands_where_its_value_puts_it`,
    `a_joints_value_may_be_an_unknown_the_drawing_solves_for`,
    `a_joints_value_prints_back_and_refuses_what_it_cannot_mean`).
  - **Decided here** (the epic's open questions 2 and 4): **one clock** — every joint still runs
    on the one shared roll, a value is an instant of it, and what is drawn in `f`, read under `f` at
    `τ`, is the joint at `value + τ`; **rest** is each frame at its value, the shared roll at 0, so
    `Sketch::world_point` is the drawing as before and reads no joint.
  - **Not built in stage 3**: a value written over a declared parameter (built in stage 4,
    below), a value any but linear in its instant (built in stage 5, below), a frame with a value
    riding on another, and a declaration drawn in a valued frame's own block (built by #398 B,
    below).
- **Relations over a parameter** (#391 S4, [0.142]): a relation reading a declared parameter holds
  for every value of it, and the drawing is followed along it.
  - **Read as a held number.** `Walk::parameter_axis` puts the parameter into the root's `vals` as
    a free value and into `Held::names` at its interval's start, so a dimension's text keeps it by
    name (`of_vals_in`) and `program::elaborate` holds it (`Sketch::hold`, `Sketch::held[t]`, a
    fixed param `~t`) exactly as a face family's running number is held: no unknown, no column,
    `expr::evaluate` reading it at whatever value the param stands at.  **The drawing stands at
    the interval's start** (rest, the epic's question 2 for a declared parameter), so a document
    reading one solves and draws as one instant of itself.  A value reading one is E103
    (`work_out`, `running`: "a value is worked out once"), and a component's body sees none.  A
    joint's value may read one (`motions::values` reads `Sketch::held`), so what is drawn in the
    frame is a trajectory (`tests/parameters.rs`,
    `a_joints_value_may_run_over_a_declared_parameter`, against the motion's closed form).
  - **The default quantifier** (the epic's question 3, decided): no word.  Every relation of the
    drawing holds at every value of every parameter, the ones reading none included — a relation
    between geometry that does not move is the same row at every instant, and one between a moving
    entity and a static one holds throughout, which is what a linkage's ground is.  A relation
    holding *at* one instant (`at t0`) is not built.
  - **Continuation** (`follow.rs`, the epic's question 6, decided): `follow::follow(sk, "t")` solves
    the drawing at the start and marches to the end, each station the drawing's own solve started
    where the secant through the two before carries it (`follow::settle`, the one station solve,
    which a face family's stations share, as they share `follow::short`'s rank check). A step is
    kept when it solves, when the solve moved the predicted station no more than `TURN` (0.1) of the
    step's own motion in world length (`Param::scale`, over the unknowns; plus a floor of `1e-9` of
    the extent), and when the **orientation** of the conditioned Jacobian (the sign of its
    determinant by `linalg::lu_factor`, where it is square and at most `ORIENTED` wide) is the one
    it had — an assembly mode keeps it, so a flip is a branch jump or a singular instant crossed.
    Otherwise the step is halved; kept easily (`EASY`) it grows by `GROW` up to `COARSEST` (1/128)
    of the interval. **A singular instant** is where no step is kept at `FINEST` (1e-7) of the
    interval — the solve fails or the orientation flips — or a kept station is short of rank;
    `beyond` reads a solve `PROBE` (1e-4) of the interval past it: none stands there, `Kind::Limit`
    (the motion turns back: a crank at its dead point), one does, `Kind::Branch` (two branches
    cross: a four-bar at its change point). The march stops at the last station that stands and
    records the instant (`Followed::singular`). A drawing short of rank at the start is refused
    (`not_held`): its freedom is a free function of the parameter. A drawing no relation of which
    reads the parameter (`follow::readers`, the constraints whose `Arg::Expr` text reads its name)
    is refused too. `Followed::put` stands a drawing at any instant reached, solved from the
    stations either side, on the branch carried. `follow::followed` remembers a march on the sketch
    (`Caches::followed`) against every parameter, every constraint's numbers and the interval.
  - **The ledger per parameter** (`Diagnosis::parameters`, `diagnose::Driven`): each declared
    parameter's readers and the freedom they drive — the rank their rows add to the rest's, on the
    conditioned Jacobian, `None` past the dense limit.  `summary` says `1 DOF over t` and, where
    freedom is left, `each DOF left a free function of t`; `diagnosis_json` carries `parameters`
    (name, readers, dof) but not the march, which is a solve a station: `report::parameter_text`
    says how far the drawing is followed and where it stopped, and `solventc` prints it with the
    readers (`driven by t: …`).
  - **`trace(m over t)`**: a point of the drawing itself as `t` runs (§6.5).  The parser takes
    `trace(REF over IDENT)` with no `in` (an empty domain), the flattener fills the parameter's
    interval (E103 where the name is no declared parameter, or in a component), and `resolve`
    gives a `CurveOf` with no instance where `owner_of` finds none and the name is a parameter.
    `program::curves::followed` builds a `CurveE` over the point (`args[0]`) and the interval or a
    stretch of it (E103 outside), its definition `CurveBody::Followed` (one per parameter,
    `followed:t`); `Sketch::followed_points` stands a copy of the drawing at each instant asked
    (`curve_sweep`, `curve_point`), clamped to what the march reached, so the curve ends at a
    singular instant, and `curve_polyline`'s key reads every parameter.  It is a view's curve: a
    point in space is E060 (`entities::drawn_in_planes`, once memberships are known).  A relation
    holds to it (#398 A, below); it cuts nothing (`generated::tool_of`), no envelope is read of
    it and no marched body starts on it.  `print` writes `trace(m over t)`;
    `lift_curve` writes the point over its stretch.
  - **Gate**: `tests/continuation.rs` — the four-bar's rocker pin and coupler midpoint at every
    station against the closed form (`a_four_bars_coupler_curve_is_its_closed_form`, 1e-8); the
    slider-crank's slider through both dead centres; a Grashof four-bar 1e-4 mm from its change
    point held on its branch through the near-fold at 180°; a non-Grashof crank stopped at its
    closed-form limit (`Kind::Limit`, 1e-3°) and a change-point four-bar at its fold
    (`Kind::Branch`, 180°); a drawing not held, refused; the coupler curve as `trace(m over t)`
    against the closed form, closed over the whole turn and ending at the limit; printing and the
    refusals; the ledger (a four-bar's 1 DOF over `t`, a five-bar driven twice 2, the free function
    left). The case library's documents over a parameter are the series
    `rust/examples/mechanisms/`, each held to its closed form by `tests/mechanism_examples.rs`: a
    four-bar round a whole turn, a slider-crank through its dead centres, a triple rocker stopped
    at its limit and a four-bar stopped at its change point.
  - **Not built in stage 4**: a relation holding at an instant (`at t0`), several parameters
    followed together (each is followed alone, the others at their starts), a trace of a point in
    space (a traced body in space is a component's, §6.5), a trace held to by a relation (built
    by #398 A, below), a sweep
    of a section over a declared parameter (`sweep(sec over s)`, built by #398 B, below),
    continuation past a singular
    instant onto a chosen branch, and the app's timeline (stage 6).
- **Relations held to a followed trace** (#398 A, [0.144]): `trace(m over t)` is a curve the
  solve works through — a point on it at an instant, a line tangent to it, a circle osculating it,
  a claim over it — and a number of the drawing that does not run with `t` solves through it.
  - **Lowered to a locus** (`followed::lower`): the point, its derivatives in `t` and its
    sensitivities to the drawing's unknowns are what a component's trace gives (`locus.rs`: the
    implicit function theorem at the place, `C''` and `C'''` off the rows' Taylor forms), so the
    followed drawing's moving part is lowered to one and held to by the curve contacts every trace
    has (`CurveBody::Followed(Some(locus))`, a definition of its own per curve, `followed:t#i`;
    `CurveBody::locus` is what the kernels, the constants and `validate` ask).  **What moves**:
    the rows reading `t` (`follow::readers`) and every row joined to them through an unknown that
    is no free value — not a row held to a followed point, nor a soft one, a claim or a bound —
    their unknowns the locus's `q`, each an entity's own scalar (`CurveE::pose`, read off the
    sheet).  **Its columns** are the free values those rows read (`CurveE::unknowns`, a declared
    unknown or an instance's unbound number, `leg.h`): a free value joins no rows, since it holds
    a number rather than moving it.  Its values are the fixed numbers they read (a fixed point's
    coordinates, a held number), as they stand (`CurveE::values`).  A row reading `t` is its kind's free twin over a tape of its text, as a
    component trace's dimension over its formal is.  The traced point is no column of the curve
    (`Sketch::entity_params` leaves a followed curve's argument out).
  - **One solve** (the issue's order question, decided): the locus is anchored where the drawing
    stands, at the instant `t` is held (`Sketch::curve_home`), its pose the drawing's own, so a
    solve holding a relation to the trace solves the drawing there and the trace from there
    together; the trace at `τ` is the locus's march from the pose, remembered per contact and
    resumed (`locus::eval_at`), and no row reads `Caches::followed`.  A march of the drawing
    (`follow::follow`) solves the held relation at every station as a row like any other,
    anchored at that station.  `Sketch::curve_point` of a lowered curve reads the locus too, so a
    solve asking where the curve closes (`clamp_contacts`) marches nothing; its polyline is still
    the march's, which agrees with it on the branch carried and ends at a singular instant.
    `follow::followed` answers no march from inside its own (`MARCHING`).
  - **Stated late** (`program::elaborate`'s `held_to_followed`): a relation naming a followed
    point is stated after every other row, once `expr::evaluate` has bound the unknowns their
    numbers read; the point is lowered then, and the numbers worked out again.  **Refused**
    (E061, and `validate` wherever the curve is not lowered): what moves leaving free a number it
    places (a coordinate solved through the trace is declared as a number instead, `param l:
    Length`), stating more rows than it places, a row with no static kernel (a curve contact, a
    derivative), a number no point, circle or arc owns, a turned copy's, a traced point that does
    not move, nothing reading `t`, and more than `tape::MAX_VARS` outer numbers or the locus's own
    bounds.  A graft carries a lowered curve unlowered (`io::graft`): its lowering is the whole
    drawing's.
  - **Gate**: the corpus rewritten onto `param t`, to the same numbers — `stride.sv`,
    `osculating_stride.sv` and `dwell.sv` (`tests/generation_examples.rs`; the rod and the offset
    unchanged at every station of the march), `peaucellier.sv` (`tests/claim.rs`, the claim a
    theorem) and `jansen.sv` (`tests/jansen.rs`, the frame's height and the rod against the leg
    built from circle intersections; `tests/examples_sv.rs`, DOF 0).  `tests/followed_held.rs`: a
    four-bar's rocker solved so its coupler curve passes a point, against the closed form (1e-8);
    the march holding it at every station; the point, tangent and curvature contacts' Jacobians
    against differences of their residuals; printing; the refusals.  The drag that stepped
    Jansen's leg went with its free crank (#398 L).
- **General drives** (#391 S5, [0.143]): a joint's value that reads a declared parameter
  *drives* the frame by it — the value is a law, any expression of the parameter, and the frame
  stands at its joint turned `ratio · law(θ)` and risen `advance · law(θ) / 360°` at the roll
  `θ`.
  - **One clock**: a reading of the frame's motion over the roll reads the roll as that parameter,
    so the joint stands at the law's value of the roll rather than at the roll — a value of `t`
    alone is the frame undriven.  Stage 3's rule for a value that reads none stands: the joint
    runs with the roll, and the value is only where `x in f` puts what is drawn.
  - **The node** (`program::motions::motions`): where a frame's value reads a declared parameter
    (`runs`, the declared parameter its text reads) a `MotionDef::Driven { of, law, over }` is minted
    beside the joint's own node; `motions::carried` hands every reading of the frame's motion the
    drive (`driven`), and `x in f` the joint itself (`undriven`: `relations::carried_motion` and
    `solids::build::under_motion`), read at the value worked out where the drawing stands
    (`value_now`, `t` at the instant held) — so a row reads no law, and what is drawn in the frame
    is followed along `t` as stage 4 follows it.  A value reading two parameters is E103
    (`motions::values`); a drive over a parameter that is no `Angle` is read over the roll by
    nothing (`reading_of`, E080: the roll is an angle) and followed all the same.
  - **The law** (`motion::Law`): the value's text compiled once over the held numbers
    (`tape::Tape::compile_wide`, every name but the parameter where the drawing stands when the
    motion is read, as `Family::read` reads its axes; a name no held number is is refused) and
    read three ways — its value at a roll (`eval_num` over `f64`), its value and rate there off a
    Taylor jet (`eval_num` over `taylor::Jet`; nothing is differenced), and an enclosure of both
    over a stretch
    of the roll (`tape::eval_interval`, the fourth walker: forward duals over
    `interval::Interval`, failing closed for `tan`, the inverse trigonometry, `exp`/`ln`/`log`,
    the roundings, `atan2`, a power not a whole number, and a division, a root or a length's
    derivative reaching zero).  Angles in radians a radian; the tape's own trigonometry stays in
    degrees.
  - **In `Family`** (`motion::Step::{Rotation, Translation}::law`, the joint's step with its law,
    `MotionDef::Driven` read by `Family::read`): `at` stands the joint at the law's value and
    carries its rate by the chain rule (`driven`), `pose_at` the value alone, `bounds` encloses
    the joint over the law's enclosure of the interval (`MotionBounds::rotation`/`advanced` over
    it), and `inverse_point_speed_bound_over` scales each driven step's speed by the bound on the
    law's rate and its reach by the bound on its value — so the roll search through a body
    (`solid::roll`) and a swept field's minimum stay conservative under a drive.  A drive is no
    constant twist: `screw`, `turning`, `sliding`, `in_plane`, `in_frame` and `instantaneous`
    refuse one, as does `NormalVelocity` (`motion::contact`) — what is built of a driven sweep is
    the field's.  `solid::reads` keys a drive by its text and every held number.
  - **Refused** where a row would read it: `envelope::held::motion` (a carried point at a roll, a
    rate, a meshing; a relation held to an envelope under one, E061 where the solve is told it
    cannot hold to it), said first for a carried point by `relations::carried_motion` (E080,
    naming `x in f`), both in `motion::unread_drive`'s words; `program::generated::program_signed` (a profile in the plane is
    generated under joints at a constant rate).  A graft carries a drive with its joint
    (`io::graft`), its law reading the parameter by name.
  - **A constant twist by what is stated** (`expr::Ast::affine_in`): a face family's carried rows
    are its motion's sweep (`solid::family::stated_carry`) only where the roll they state is affine
    in the running number as written; through [0.142] three samples (the ends and the middle)
    decided it, which `900deg · s + 30deg · sin(360deg · s)` passes.  A single joint at a monotone
    law sweeps the same surface at another pace (a sweep reads the path alone), and the B-rep still
    reads such walls off the stations as their screw's (`tests/carried_family.rs`,
    `a_law_reparametrises_its_screws_sweep`).
  - **Gate**: `tests/drives.rs` — a radial cam: the follower's roller followed along `t` against
    its law at every station (1e-9); the follower's drive seen from the cam against the closed
    form, its pose and its rate against a difference of it (1e-9, 1e-6); `MotionBounds` holding
    every sampled pose of every 15° stretch and the speed bound every sampled speed; the ball
    swept along the law as the cam sees it, its field's distance against the closed form's at
    points on, inside and outside the groove (1e-6); a tool placed at a roll and drawn in the
    frame; a roll through a blank found never short of the closed form's and within three steps
    of it; a law three ways (`a_laws_jet_and_enclosure_agree_with_its_value`, over every function
    the walker carries); the refusals.  `tests/expr.rs`'s
    `affinity_is_read_off_how_a_number_is_written`.
  - **Not built in stage 5** (the first two built by #398 B and C, below): the guide-following
    station words as frames driven along a path —
    a guide's frame (`std.On`, `std.Minimal`, `std.Frenet`) is no joint at any value, its pose read
    off the guide's geometry and its derivatives (and for `On`/`Minimal`, integrated), so it
    needs a motion of its own kind in `Family` (the guide enclosed over a stretch for its bounds)
    and a sweep over a declared parameter that is no angle (`sweep(sec over s)`, left unbuilt by
    stage 4); a relation the solve holds reading a drive (a law in a kernel's constants); a
    profile in the plane generated under a drive; a slide's value written as a length (it is an
    instant of the roll: `advance: 360mm` makes its degrees millimetres); and the app's timeline
    (stage 6).
- **Faces swept over a declared parameter** (#398 B, [0.145]): one parameter axis for the face
  families as for the drawing.
  - **A face family reads its parameter as declared**: `sweep(Station(…).f over s)`, `s` a `param
    s: … over (a, b)` of the document (`Walk::family_parameter`; a component's body sees none,
    E103, so a family stands in the document), its interval the family's.  The formal that runs
    is the one the call gives the parameter by name (`Tube(hose, std.x, r: r, s: s2).f over s2`:
    the argument is taken off the call, the formal left unbound as before) or else the one named
    as it, and it is of the parameter's kind (E103); the call gives it nothing else (E103).  The
    flattener records the formal by the instance's key (`Held::formals`), and everything after it
    is the family it was: the drawn station at the middle, `#f12.s` held, no solved value moved.
    `over s in (a, b)` in a family's brackets is refused naming the new spelling (`P::family`),
    and the printer writes `over s`.  A formal given a text kept over a family's running number
    answers for its name before the document's parameter of that name does (`settle_text`): the
    eased twist's `s: 3 * s^2 - 2 * s^3` reads `#f12.s` though the document declares `s`.
  - **A face of the drawing itself**: `sweep(sec over s)`, the tool a face and no `under:` or
    `seen_from:`, is `SolidDef::Family` over the parameter itself (`build::drawn_family`): its
    `over` the declared `Running`, its station every entity of the drawing, the part its face
    reaches what each station solves (`solid::family::leg_part`, the prefix empty for a name
    under no instance), drawn where the parameter stands — the interval's start — and stepped from
    there (`step`: the run down from the start is empty).  `Walk::declared_interval` writes a
    non-angle parameter's ends for a sweep without refusing it; the elaborator refuses a solid
    swept over one (E103: the roll is an angle).  A drive over a parameter that is no angle is
    left out of the motions read after the solve (`program::solid_diagnostics`): no reading over
    the roll reads it, and one that would is refused where it is made (`reading_of`).
  - **Drawn in a frame with a value** (`flatten::valued`): a body's statements whose `in`-block
    path names a frame the body gives a value (`fix(value == …) f`, matched by its text) are
    lowered before the walk.  Each plane such a declaration or instance is drawn in (the innermost
    frame of its path but the valued one) gets one twin per body, `#in<offset> := P in f`, walked
    as a reading at an instant is; the declaration is drawn in the twin and rides on the valued
    frame no more; and every reference to the plane (or a member of it) in a statement of the
    block — a declaration's children and membership, an instance's arguments, a relation's
    operands and arguments — reads the twin (`syntax::Relation::within`, stamped by the parser,
    keeps the blocks around a relation).  The twin's rows are `x in f`'s, row for row: its value
    read again at every station.  Refused: a declaration in two valued frames, one drawn in the
    valued frame and no plane of it (E080).  A frame declared in the block is left to
    `motions::values`'s refusal.  `std.Twisted` stays a point drawn in `n` (`t := x in turn`):
    drawn in the block it would stand in the twin, and `mapped` reads its targets in one plane.
  - **Gate**: `tests/swept_over_parameters.rs` — the epic's helix as written (its value in the
    `fix` stage 3 settled on) against the screw's closed form at stations of the march along `s`
    to `EXACT`, against Pappus off its exact B-rep, every wall the surface its first rim sweeps,
    and corner for corner the thread `std.Helix`'s station sweeps; the block's relations holding
    the drawing at rest at every station; the refusals.  `tests/face_family.rs`: a station drawn
    in the document and given the parameter sweeps the reducer's 5120 mm³, and a family's
    refusals.  The corpus is written over declared parameters (`lofts/`, `paths/`, the loose
    families, `turbine_blade.sv`; `lofts/thread.sv`'s thread is the epic's helix;
    `twist_drill/drill.sv`'s roll is a declared `t`) and every test of it passes unchanged in its
    numbers.
- **Frames along a path** (#398 C, [0.145], `motion/guide.rs`): a frame whose pose is read off its
  guide at the path's parameter — a motion of its own kind.
  - **Stated as a joint held along a path**: `g coincident f.axis`, `g` an arc, a spline, a curve
    or a curve traced in space (`motions::motions`, `Held::guide`; a line is a turn's axis), with
    `fix(ratio == ρ, advance == 0mm) f`, `ρ` 0 or 1 (E080 otherwise, and for an advance), makes
    `MotionDef::Guided { guide, frame, frenet: ρ == 1 }`; the statement states no row
    (`Resolver::guides`, skipped by `elaborate::constraints`) and the joint's axis is held where it
    stands.  `std.Along(f, g, up: u, home: h)` (ρ 0) and `std.AlongFrenet(f, g, home: h)` (ρ 1)
    draw the frame at its home square to the path and state the joint.
  - **Read after the solve** (`motion::Guide`, `Step::Guided`): the path read four ways (`Path`: a
    line's and an arc's fraction of the run exactly; a spline, in a plane lifted or over points in
    space through `Sketch::spline_traced`, by its basis with three derivatives; a traced curve by
    `traced::at`, its third derivative a difference of its exact second — what a Frenet frame's
    rate alone reads); its home the parameter nearest the frame's origin (refused past 1e-6 of
    it: drawn off its path); `M(u) x = R(u)(x − c(h)) + c(u)`, `R(u) = F(u) F(h)ᵀ`.  ρ 0: the
    rotation-minimising frame integrated by double reflection over `GRID` steps laid from the
    home, any `u` one step from its nearest node, `ω = T × T′`; ρ 1: the Frenet frame read
    directly, `ω = T × T′ + τ T`.  `at` carries the rate (`envelope::Motion::of_parts`), `pose_at`
    the pose; an inverse is the step undone (`back`).  No constant twist: `screw`, `turning`,
    `sliding`, `in_plane`, `in_frame`, `instantaneous` and `NormalVelocity` refuse it.
  - **Enclosed over a stretch** (`Guide::stretch`): samples a step `w` apart, every point within
    `|C′|·w/2` of one and every turn within `|ω|·w/2`, `|C′|` and `|C″|` bounded over the whole
    path for certain — a line's and an arc's exactly, a polynomial spline's off its hodographs'
    control points — and `|ω| ≤ |C″| / min|C′|`; `MotionBounds` (`bounds`) and the speed bound
    (`inverse_point_speed_bound_over`) read it.  It fails closed for a rational spline, a traced
    curve, and the path's own frame (no bound on the torsion).
  - **What reads it**: a face drawn in the frame and swept as a declared parameter runs
    (`solid::family`, `Along`): at each station its samples and plane, and the exact walls' node
    numbers, carried by `M(s) · C_home · C(s)⁻¹` — the frame's chart as the drawing has it and as
    the part stands there — so a frame the station moves and one it does not are read alike.
    Refused: a relation the solve holds reading it (`envelope::held::motion`, E080), a profile in
    the plane generated under it; a graft carries none (`io::graft`).  Today's station words
    (`std.On`, `std.Minimal`, `std.Frenet`) still solve a plane at every station; the corpus is
    not moved onto the frames.
  - **Gate**: `tests/guided_frames.rs` — a rectangle in `std.Along`'s frame swept over `s`
    against the same rectangle in `std.On`'s plane, corner for corner and by volume to 1e-9, along
    the planar spline (home-free) and the cubic in space (homed where `On`'s family is); the
    ribbon in `std.AlongFrenet`'s frame against `std.Frenet`'s the same way; the frame along a
    traced helix against the helix's rotation-minimising frame in closed form to 1e-9 and its rate
    against a difference of its pose; `MotionBounds` holding every sampled pose of sixteen
    stretches forward and back and the speed bound every sampled speed; the refusals.
