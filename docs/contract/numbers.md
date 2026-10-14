# Numbers: expressions, units and unknowns

Part of the [contract](../../CLAUDE.md) the code is held to. Read it when you touch `expr.rs`,
`units.rs`, `param`, or a dimension's text. An issue number (#n) or a spec version ([0.n], see the
[language changelog](../solvent-changelog.md)) says where a rule came from.

- A dimension's number may be an *expression* (`expr.rs`): `Arg::Expr { text, value }` in a
  `Length`/`Angle` slot, `value` in arg units (radians) for the kernels, `text` as written
  (degrees).  An expression defines no name ([0.41]): the source names a number (`w := 80`)
  and the flattener writes it in, and a name a text still reads is a free variable (below).
  `expr::evaluate` works the document's expressions out in document order, reporting per
  expression what it reads and its error.  One that cannot be computed keeps its last number.
  Trigonometry is in degrees.
  A mixed fraction — `3 1/2` — is one `Num` from the tokenizer (`31/2` and `1/2` are divisions).
  `expr::literal` does *not* claim it; `expr::notation` marks it notation, so `arg_text` prints it
  as written where a *formula* prints text and value (`h = w * 2 = 80`).  A **callout carries the
  expression** (`io::dimension_text`), a `param` included: the flattener settles `distance(w)` to
  `100`, so the spelling is read back at the argument's span into `Constraint::written`
  (`program::relations::written`), presentation only, dropped by any write of a number — at the
  root and in a same-file component body (`design.module`).  A module's body and a block copy draw
  the number; `program::relations::repeated` marks later copies drawing the same label
  (`Constraint::repeated`), left out of the full layout, drawn when requested by id.
  `expr::set_dimension` is the one write path for text (a bare number becomes `Arg::Num`, angle
  converted — the app converts nothing); `Sketch::add` and `io::from_json` evaluate;
  `Sketch::set_constraint_num` writes a number and re-evaluates when it dropped an expression.
  Documents save `{"expr", "value"}` and accept a bare string; binding records keep the number in
  `args`, the text under `exprs`, and proxies `sync()` before handing out a value.
- **Every number has a dimension, and it is checked** (`units.rs`, Solvent §3.3).  Two bases,
  **length** and **angle**, with *rational* exponents (`sqrt` halves one).  `*` and `/` derive,
  `+` and `-` demand agreement, `^` takes a whole power on a dimensioned base, and the result is
  checked against its slot (`SpecKind::dim()`).  `Aff` carries the dimension beside the value.
  **The asymmetry between `Dim::fits` and `Dim::agree` is the design.**  A *context* (a slot, a
  function's argument) may take a bare number; two *operands* may not mix: `90 / N + ivp` is an
  error, `90deg / N + ivp` the answer.  A **name** is worth a number (`w = 80` in a Length slot
  does not make `w` a length); a literal's unit and a formal's declared `Ty` *do* travel, so the
  formal catches `x := w + phi` after `flatten::settle` substitutes a parameter away.
  A literal's unit is converted **to the document's own** by the tokenizer (`expr::parse_in`
  takes `Units`): `unit mm` names it, and without one a suffix is refused.  **Feet-and-inches is
  one literal** — `1' 6 3/16"` — so there is **no string literal** (`"` is the inch mark, a `Str`
  argument and a raw branch key are bare words).  `pi` is dimensionless and `tau`/`turn` a
  **turn**, so `tau == 2 * pi * 1rad`; radians are written `* 1rad`, not `* 180 / pi`.  Storing
  the unit costs the solve nothing, and `io::paste` converts between documents' units
  (`Sketch::rescale`, written out by kind: a `Param` cannot say it is a length).
- **A number is named one way, and an unknown is declared** (issue #77; Solvent §3.4, §5, §6.3,
  [0.41]).  `w := 60` is a value; `param w := 60` (`ParamDecl::input`, a modifier the parser reads
  in `statements.rs::input`) marks one of the document's **inputs**, the only lines
  `examples::with_params` rewrites; `param beta: Angle hint(30deg)` with no value is an
  **unknown**: `flatten::values::params` puts `free(name, ty)` in `vals` (as `bind` does for an
  unbound formal) and records a `model::Declared` (dimension, seed) in `Expansion::unknowns` →
  `Sketch::declared`, which `expr::evaluate` reads (the seed, skipping `settle`; the declared
  dimension: a text is worked out over `expr::with_declared`, here and in the flattener's `value_read`, so
  `values::of_vals`'s `(-1 * m + 0.2mm)` — the constant in the whole's unit — reads alike
  everywhere; `check_dim`).  An unbound
  `param` states its type (`Length`/`Angle`/`Scalar`), takes the one keyless `hint(E)`; a seed on
  a bound one, an `Int` unknown and a module's unbound `param` are E040; `param` is refused by the
  parser inside a component (formals are its inputs) or block.  A call seeds a formal it leaves
  unbound with `InstVal::Hint` (`Wing(f, beta: hint(15deg))`), and a hint written over that
  unknown reads it at its seed ([0.90], `settle_seeds`'s provisional values; zero where it has
  none).  **A name nothing declares is
  E101** — in a dimension (`settle_text`), a pin (`settle_arg`), a dotted read nothing made
  (`resolve`, unless it is an instance's unbound formal) — and the relation is not emitted
  (`Checks::refused`), so it never reaches the expression graph as an unknown; W111 is gone.  A
  dotted name read in a body is made absolute under `Scope::instance_prefix`.  `t == s` over an
  unknown is `Arg::Tie` seeded by the declaration (a `hint(t:)` beside it is E040 in `assemble`).
  `commit_seeds` writes each declared unknown's solved value into its `hint(…)`
  (`edit::unknown_seeds`); `to_program` declares a sketch's unknowns (`lift::unknowns`);
  `edit::set_dimension` over a dimension reading a bound root `param` writes the param's line; a
  drawing's `dimension m.w` is the first top-level dimension written `w` (`render::written_as`).
  `tests/names.rs` is the gate.
- At the **sketch level** (JSON documents, the bindings) an expression reads names and defines none:
  every name it reads is a **free variable** (`expr::Free`) — the form a declared unknown takes once
  flattened.  It is an unknown of the sketch tying the dimensions that read it.  `expr::evaluate`
  (document order; no definitions, so no dependency graph) allocates one Param per free name into
  `Sketch::free_vars`, retires it to `fixed` when unread — keeping the *slot*, so the parameter
  count `topology_key` ends with does not move — and rewrites every binding (`Constraint::free`, at
  most one) from scratch each run.  It runs on `Sketch::add`, `remove`, `set_constraint_num`,
  `set_dimension`, `from_json`, `report::exprs_json`; a whole document uses `Sketch::add_quiet` and
  evaluates once (`io::graft`, `io::from_json`), since per-add evaluation is quadratic. The tie is
  **affine in one free name** — `a`, `a / 2`, `2 * a + 5`, `value = m*a + c`, all a fixed-width
  block carries; `expr::eval` works in `Aff`, so `a * a`, `sin(a)` and two free names are errors by
  arithmetic, and an erroring dimension keeps its last number. Every `Length`/`Angle` type needs a
  *free twin* kernel (one more column, (m, c) as constants) in `CKind::free_kernel`, exhaustive;
  `every_dimension_can_be_written_free` checks it.  `params_on` appends the free column (always
  last), `consts_on` returns `[m, c]` turned by `side:`, `along:` or `sense:` (a skew distance's by
  its pinned `sign`, with the pin beside them), `kernel_id` picks the twin.  A fresh one seeds from the stated number, else by
  Newton on its row (`expr::settle`).  A free dimension is `unsupported` in `cgraph`, never jittered
  by the witness, part of `topology_key`, and joins `io::Part`'s walk.  `expr::sync_free` updates
  the numbers shown from every seam writing parameters directly — `Sketch::set_x`,
  `io::Part::write_back`, the wave's writes.
