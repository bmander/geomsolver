# Parameters: numbers that run

Part of the [contract](../../CLAUDE.md) the code is held to. Read it when you touch
`model/running.rs`, a `param … over`, a face family's running number, a traced body's formals, or
an interval of the roll a reading states. The direction is [#391](https://github.com/bmander/geomsolver/issues/391):
geometry over a parameter, trajectories rather than snapshots; this file holds what is built
of it. An issue number (#n) or a spec version ([0.n], see the
[language changelog](../solvent-changelog.md)) says where a rule came from.

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
- **A declared parameter is read by a reading over the roll, and by nothing else yet** (#391 S2,
  [0.140]): `over t` with no interval (`syntax::RunsOver::declared`) names it, and
  `Walk::declared_interval` writes its ends into the reading's `from`/`to` before they are settled
  — a parameter of another type than `Angle`, or a name the document declares none by (a
  component's body sees no parameter), is E103 at the name. Reading one in a dimension's text is
  still E103 (`Walk::runs`, `values::runs`) and the relation is refused as for a name nothing
  declares (`settle_dim`, `Checks::refused`), so no free variable is ever made of its name: a
  relation that holds for every value of a parameter is stage 4's.
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
  value riding on another, and one something rides on (E080) — a declaration in its block would
  stand where it is drawn, not where the value puts it, so the refusal names `n := base in f`.
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
  - **Not built in stage 3**: a value written over a declared parameter (a relation holding for
    every value of one is stage 4's), a value any but linear in its instant (stage 5), a frame
    with a value riding on another, and a declaration drawn in a valued frame's own block.
