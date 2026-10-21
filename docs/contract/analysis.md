# Analysis

Part of the [contract](../../CLAUDE.md) the code is held to. Read it when you touch
`interior.rs`, `KernelKey::Interior`, or `sparse::Ata::least_pivot`; or `model/scenario.rs`,
`syntax/scenario.rs`, `syntax/parser/scenario.rs`, `program/scenarios.rs`, or write a `scenario`;
or `model/material.rs`, a `material(…)` group (`flatten::values::material`), a solid's `of`, or a
material's JSON (`io::material_json`) or report (`report::materials_json`).
The epic these rules serve is #510 (design intent as structural performance), whose
prerequisites and stages (P1–P8, S0–S4) are named there; this file holds the rules landed so
far. The gate is `rust/gcs-core/tests/interior.rs` for the interior (P5) and
`rust/gcs-core/tests/scenarios.rs` for scenarios (P4) and `rust/gcs-core/tests/materials.rs` for
materials (P3), unless a rule names another.

- **An analysis output is a row, and its interior is a kernel** (#510 P5). A requirement whose
  value comes from `K(x)u = f`, `x` the drawing's numbers and `u` the displacements, is a kernel
  built at its block's widths (`KernelKey::Interior { n_par, n_const }`, `interior::kernel`) as a
  spline's length is (`integral.rs`): one row reading every number the structure is made of. It
  is not a boundary of `solve/through.rs`. A boundary's inner unknowns are parameters of the one
  `System`, and its inner solve is `newton::square` on a dense LU, its derivative a column at a
  time. A structure's displacements are thousands of numbers the drawing does not hold, its
  solve is linear, and it needs one derivative for many columns. As a row it is taken as every
  other row is, by the flat solve, the decomposition, the diagnosis, a claim and a free twin,
  and no boundary has to be named from Rust.

- **The value is one factorisation; the gradient is the adjoint** (#510 P5). `Structure::solve`
  assembles `K(x)`, factors it once and solves for `u`. `Structure::gradient` solves `Kᵀλ = ∂J/∂u`
  on the factor already in hand and returns `dJ/dx = ∂J/∂x − λᵀ(∂K/∂x·u − ∂f/∂x)`: one more
  solve, whatever number of columns the row reads. The gradient only reads the `State`
  (`Ata::resolve`), so it cannot factor again
  (`the_gradient_is_one_more_solve_whatever_the_columns`, to over 2,000 columns). `∂K/∂x·u` is
  never formed. Each element differentiates its own mutual energy `λₑᵀkₑ(xₑ)uₑ` in its own columns
  (`Bar::mutual`).

- **The stiffness is the Gram matrix of strain rows** (#510 P5). An element's stiffness is
  `Σ sᵣsᵣᵀ` over rows of its own degrees of freedom. A pin-jointed bar has one,
  `√(EA/L)·(−ĝ, ĝ)`. So `K = SᵀS`, ordered and factored by the normal equations the solver
  already has (`sparse::Ata`). A support holds a degree of freedom at zero by leaving its column
  out of `S`. A frame's or a continuum's element joins as more rows (a Cholesky factor of its
  constitutive matrix times its strain operator). A factorisation fit for 3D meshes is P1's.

- **A structure is its columns and its constants** (#510 P5). The columns are every node's
  coordinates (`dim` = 2 or 3 each), then every bar's area (`Structure::columns`), then the
  unknown where one sets the output. A held number is a column the system drops, as for any
  row, so a requirement can drive a node's place or a member's size. The constants are
  `[dim, nodes, bars]`, what the row states (`J₀`, or `(m, c)` with an unknown), every bar's
  `(a, b, E)`, then every degree of freedom's `(held, load, weight)`, written and read back
  exactly (`Structure::consts`, `Structure::read`, `a_structure_is_its_constants`). Whether an
  unknown sets the output is read off the row's width, as a spline length's is. The output is `J = Σ weight · u`, its weights a direction's
  cosines, so it is a displacement and the row is of **degree 1**. The residual is `J − J₀`, or
  `J − (m·a + c)` with an unknown.

- **A structure with no displacement to give is a failure, never a number** (#510 P5).
  `Failure::Free`: the supports leave a motion that costs no energy. The factor has an exactly
  zero pivot, or its least pivot as a fraction of its diagonal (`Ata::least_pivot`, recorded
  while the factor is made) is below
  `interior::SINGULAR` (1e-12, a few thousand times the precision). A node no bar reaches is
  one, and so is a node held only along a line of bars. `Failure::Collapsed` is a bar of no
  length, and `Failure::Unstiff` is a bar with `E·A ≤ 0`. The kernel reads every failure as NaN,
  in the residual and the Jacobian, which the step rejects and shrinks its trust region for
  (`a_structure_with_no_displacement_to_give_says_why`). Naming the free motion in the solver's
  words is P7's.

- **Held to closed forms and to its differences** (#510 P5). The apex of two bars sinks
  `P L³ / (2 E A h²)` to 1e-12, and its rates in the rise and in each area are the closed form's
  (`a_two_bar_truss_sinks_as_its_closed_form`). A bar in space stretches `P L / (E A)`
  (`a_bar_in_space_stretches_by_pl_over_ea`). Newton on the row alone finds the area that sinks
  the apex half a millimetre, to the closed form's (`a_requirement_drives_the_area`). The
  adjoint gradient agrees with central differences of the value. That holds for braced,
  redundant structures in the plane and in space, stated and free, and for two instances in one
  block (`tests/jacobians.rs`, `a_linear_interior_agrees_with_finite_differences`).

- **What P5 leaves to later stages.** No `CKind`, word or statement compiles to the kernel yet.
  Materials are P3 (below), and nothing reads one yet; scenarios and their requirement words are parsed and elaborated (P4,
  below) but compile no row; S0/S1 give the row its constraint type, with its callout or an
  `undrawn!` entry. Every call reads the structure from
  its constants and orders `K`'s pattern again (`Ata::new`'s symbolic analysis), though neither
  moves with `x`. The system also calls a kernel's residual and Jacobian separately, so a step
  that asks for both factors `K` twice. Holding the symbolic analysis per row (a hit on every
  step) and the factor between the two calls (a hit only at one `x`) are two caches, to be
  measured first and apart, as `solve/through.rs` measured its own. The element interface
  (strain rows and their derivatives, degrees of freedom apart from coordinates, several section
  columns) is S0's, shaped against the beam that needs it.

- **A scenario, parsed** (#510 P4, §9.11 [0.164]). `scenario NAME [: BASE] { … }` is
  `syntax::StmtKind::Scenario` (`syntax::Scenario`, its lines `Item`/`ItemKind`: a support, a load
  or a `RequirementText`), read by `parser/scenario.rs` alone. Its body is lines, not statements,
  each told by its first word (`Hold::parse`, `LoadKind::parse`, `claim`, `Output::parse` before
  `(`), so the words are contextual and only `scenario` is reserved (`words::OPENERS`). It stands
  at the top of a document or in its `preview` (`unannotated_chain_or_one` asks
  `at_document_top`, as a `param` does: anywhere else is a syntax error). A declaration in it is a
  syntax error ("adds no geometry"), and so is a load vector of two components. `print`'s
  `write_stmt` writes it back as written (`Scenario::write`), and `highlight` tints its words
  inside a scenario alone, as the parser reads them (`scenario_word`).

- **Read as the drawing's names** (#510 P4). The flattener writes the scope's numbers into every
  number (`Scenario::numbers_mut`, `substitute_in`) and rescopes every reference (`refs_mut` in
  `resolve::rewrite`): a scenario is a block like `when`, no prefix and no `PathStep`.
  `program::scenarios` reads each in `finish`, once every solid is built, into a
  `model::Scenario` (gcs-solve, pure data). Each target is read by `Resolver::operand`:
  `Applied::Ent` (a point, an edge, a face, a solid, `material`; `B.body` its solid) or
  `Applied::Face` (a face path, `FaceRef`), anything else **E040**, an unresolved name **E101**.
  A pressure is on a face alone (**E040**); `along:` names an axis or a line (**E040**). Each
  number is read by `flatten::value_read` against `Sketch::declared`, held to
  `LoadKind::dim`/`Output::dim` (**E103**) and refused where it reads an unknown (**E040**);
  `mode`'s count is a whole number from 1 (**E103**). Whether a face path names faces is not
  judged here: the analysis reads the solid's boundary after the solve, as a region reading a
  face does. Values are in the units the arithmetic is done in (`units.rs`).

- **Scoping, decided** (#510 P4). A scenario's name is its own namespace (**E001** twice),
  colliding with no geometry. `scenario B: A` begins with `A`'s supports and loads (copied, with
  the lines they were written on), then its own; requirements are never inherited; `A` is
  written above `B` (**E101**), so there is no cycle. A load combination is written out, no
  construct. A scenario adds no geometry of its own: a load patch is drawn in the body.
  `Elaborated::scenarios` holds each as a `program::Stated` beside the `Line` (statement, index)
  of every support, load and requirement, so a later report says it where it is written and
  survives `retext`.

- **A scenario solves nothing** (#510 P4). It states no row and makes no entity, parameter or
  unknown: the sketch, its JSON and its solve are the document's with every scenario deleted
  (`a_scenario_adds_nothing_to_the_drawing`). A requirement's grade (`Requirement::grade` over
  its `expr::Cmp`: `Claim`, `Driving`, `Bound`) is recorded, not acted on, but the rule S1–S3
  hold is kept now: one scenario's requirements drive dimensions, a driving row in a second
  **E106** until S4's optimiser. An edit removing what a scenario names never takes the scenario
  with it (`edit::remove::mentions`): the name left dangling is said (**E101**).

- **A material is a model value** (#510 P3, §6.9 [0.165]): `model::Material`, a name and the
  properties stated of it (`model::Property`: `E`, `nu`, `rho`, `yield`).  `Sketch::materials`
  holds the document's materials by absolute name (`steel`, `p.al` inside an instance), and
  `SolidE::material` names the one a solid is made of.  Nothing reads either yet.  The solve,
  the diagnosis and every evaluated solid are unchanged by a material.  `tests/materials.rs` is
  the gate.
- **A material is held in SI** (`Units::to_si`): pascals, and kilograms per cubic metre.  The
  same text states the same material in any document unit, inch or millimetre
  (`a_material_is_the_same_in_any_unit`).  The stored value is the value worked out
  (`7850kg / 1m^3` in millimetres comes back a hair under 7850).  Only a report rounds,
  `Property::text` to twelve figures.
- **`steel := material(E: …)` is a group with a material head** (`syntax::GroupDecl::material`).
  Its members are worked out as any group's, so `steel.E` reads as a number with its dimension.
  A material is handed to a component as a `group` formal.  `flatten::values::material` reads the
  members once they are worked out and checks each with `Property::read`.  A material whose
  properties all pass is stated in `Expansion::materials`.  One that fails any check is still
  a name a solid's `of` may give, but nothing more is said of that solid.  Every complaint is
  E103 at the property, or E001 for a property given twice:
  - a label that is no property;
  - a value of another dimension;
  - a bare number for a dimensioned property;
  - a value out of range;
  - a value that reads an unknown of the drawing;
  - a material stating nothing (E103 at the name).

  **A bare number is refused, not read in the document's unit.**  In a millimetre document the
  unit of pressure is a kilogram per millimetre-second², which no one means by `200000`.
  `Property::admits` holds the ranges: `E`, `rho` and `yield` are positive, and `nu` lies in
  (-1, 0.5).  The JSON reader holds the same ranges.
- **`of` names a material** (`syntax::Decl::material`, a trailer of `solid(…)`, reserved like
  `in`).  The flattener resolves it and judges it in `flatten::resolve::rewrite`, where it
  knows whether the name resolved.  `build_solid` only attaches a material that was stated.
  Each outcome:
  - a name that resolves to something that is no material is E103 (`` `rim` is no material``);
  - a material that was refused says nothing more;
  - a name of nothing is E101 alone, and one read through a name already said to be nothing
    says nothing.

  A group read whole is "handed on" for the W206 lint only inside an argument of a call
  (`program::drift`), so `of m` reads its group formal.

  The material belongs to the solid the document named.  When `Sketch::as_body` turns a cut
  solid into a body, the body keeps the material and the sweep moved to its stock carries none.
  A body's layers below its top carry none either.
- **A material is document state.**  `io::to_json` writes `materials`, but only when there is
  one: `[{name, E?, nu?, rho?, yield?}]` in SI.  `io::from_json` reads it back through
  `io::material_from_json`, which refuses a property no material has and a value no property
  takes.  The JSON holds no solids, so a solid's `of` is the source's to carry.  A rebuild
  (`graft`) copies the materials, and a paste brings those the document lacks.  `to_program`
  lifts each one as `material(…)`, written by `Property::source`, under a name a program can
  declare.  The printer writes `material(…)` and `of` back as written.
- **The surface:** `report::materials_json` gives each material's record with its `text` and the
  `solids` made of it.  `gcs_materials_json` exposes it over the ABI and `mesh.ts`'s `materials`
  exposes it to the page.  `report::material_text` is the CLI's line, `material steel: E 200GPa,
  …, of disc`.  `gcs-ffi/tests/materials.rs` checks the ABI and its JSON round trip.

- **Within the solver.** `interior.rs` is in `gcs-solve` and knows no Solvent.
