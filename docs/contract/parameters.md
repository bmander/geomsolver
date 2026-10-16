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
- **An instant is not yet a parameter's own**: the roll `x at τ` reads is a number of the shared
  roll as written, and a family's station value keeps its `HeldNumber`; a joint's value as a
  function of a parameter is stage 3's.
