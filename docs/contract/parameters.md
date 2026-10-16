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
- **Nothing reads a declared parameter yet**: reading one in a dimension's text is E103
  (`Walk::runs`, `values::runs`) and the relation is refused as for a name nothing declares
  (`settle_dim`, `Checks::refused`), so no free variable is ever made of its name. Evaluation
  (`x at τ`) and sweep (`over t`) are stage 2's; until then a parameter is declared, checked and
  represented, and that is all.
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
  - **the shared roll** — `Running::ROLL` (`roll`, an `Angle`), the one number every joint turns
    by (#383): no document declares it. A reading over an interval of it holds `Running::roll`:
    an envelope in the plane (`CurveE::over`), an envelope in space (`EnvelopeE::roll`), a meshing
    (`MeshingE::roll`), a swept solid (`Roll::Stated`); each kernel reads `radians()`. A claim's
    sweep (`model::Sweep::over`) is a `Running` under the claim's own name, a frame's roll or a
    number of the drawing.
- **An instant is not yet a parameter's**: `solid(…, at: θ)` (`SolidDef::Placed::at`), a carried
  point's roll slot (`t == τ`, `at: τ`) and a family's station value keep their own forms until
  stage 2 lowers every reading at a roll to `at` and `over`.
