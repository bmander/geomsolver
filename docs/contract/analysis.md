# Analysis

Part of the [contract](../../CLAUDE.md) the code is held to. Read it when you touch
`interior.rs`, `KernelKey::Interior`, or `sparse::Ata::least_pivot`; or `model/scenario.rs`,
`syntax/scenario.rs`, `syntax/parser/scenario.rs`, `program/scenarios.rs`, or write a `scenario`;
or `model/material.rs`, a `material(…)` group (`flatten::values::material`), a solid's `of`, or a
material's JSON (`io::material_json`) or report (`report::materials_json`); or `eigen.rs`,
`sparse::Sym`, `sparse::Ldl` or `sparse::Pencil`; or a case of `rust/examples/analysis/` or
`tests/analysis_corpus*`.
The epic these rules serve is #510 (design intent as structural performance), whose
prerequisites and stages (P1–P8, S0–S4) are named there; this file holds the rules landed so
far. The gate is `rust/gcs-core/tests/interior.rs` for the interior (P5),
`rust/gcs-core/tests/scenarios.rs` for scenarios (P4), `rust/gcs-core/tests/materials.rs` for
materials (P3), `rust/gcs-core/tests/linalg.rs` for the eigensolver (P2) and
`rust/gcs-core/tests/analysis_corpus.rs` for the verification corpus (P8), unless a rule names
another.

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

- **A symmetric matrix is given by its entries** (#510 P2; `sparse::Sym`). `from_triplets` takes
  every entry of the matrix, both triangles, and sums the ones named twice, as an assembly adds
  element matrices into a global one; nothing is mirrored, so an entry given on one side only is
  not symmetric and is not a matrix the factor answers for.

- **One sparse factor** (#510 P2; `sparse::Ldl`). The up-looking LDLᵀ the normal equations use
  (`sparse::Ata`, its order RCM with any dense column last) is one `Factor` over a `Sym`, JᵀJ
  among them; nothing has a copy of it. Without pivoting: an SPD matrix factors stably, an
  indefinite one when no pivot is zero (`None` when one is), and **its negative pivots are its
  negative eigenvalues** (Sylvester), so `A − μB`'s count the pencil's eigenvalues below μ when B
  is positive definite. **A pencil is analysed once** (`sparse::Pencil`): `L − μR` on the union of
  the two patterns, ordered and its elimination tree built at construction, so each shift and each
  count is a numeric factor and no more. `sparse_ldl_solves_and_counts_negative_eigenvalues` holds
  the solve and the count against the reference on membranes shifted into their spectrum.

- **One iteration for every pencil `L x = λ R x`** (#510 P2; `eigen.rs`'s `solve`): shift-invert
  Lanczos on `T = (L − σR)⁻¹R`, self-adjoint in `xᵀBy` with B the side that is positive definite,
  its largest θ = 1/(λ − σ) the λ just above σ. Full reorthogonalization, thick restart (the
  wanted Ritz vectors kept), one factor for the whole solve. The tolerances are named once
  (`BREAKDOWN`, `FLOOR`, `GAP`, `SHIFT`), never written at a use.

- **The lowest pairs of `A x = λ B x`** (#510 P2; `eigen::lowest`; A symmetric, B positive
  definite, else `Refusal::NotDefinite`): `L = A`, `R = B`, the inner product R's. **The shift is
  found, not given**: `SHIFT` of the least diagonal ratio `A_ii/B_ii` below zero (a Rayleigh
  quotient, so no eigenvalue is above it), moved down until `A − σB` factors with no negative
  pivot, so nothing is below it and the nearest are the lowest; a stiffness free in a rigid-body
  motion has its mode at zero, found like any other.
  `lowest_modes_agree_with_the_dense_reference`,
  `lowest_modes_of_a_large_membrane_hold_their_properties`.

- **Linear buckling is the same iteration** (#510 P2; `eigen::buckling`). `(K + λ K_G) φ = 0` is
  `L = B = K`, `R = −K_G`, σ = 0, so `T = K⁻¹(−K_G)` and θ = 1/λ: K must be positive definite (a
  supported structure; a rigid-body motion is `NotDefinite`), no shift is needed, the largest θ
  are the lowest positive load factors, and a load that buckles nothing (a tension) has none. `L −
  μR = K + μK_G`'s negative pivots count the positive factors below μ.
  `buckling_finds_eulers_loads` holds a pinned and a cantilevered column to Euler's loads and to
  the reference; `buckling_refuses_what_has_no_load`.

- **No pair is missed** (#510 P2; rings and patterns give repeated pairs). A Krylov space holds
  one direction of each eigenspace, so after the iteration converges Sylvester's count is taken
  `GAP` above the highest value found; where the pencil has more below it than were found, the
  iteration runs again B-orthogonal to every pair found and finds the copies. The count answers,
  not the iteration. `a_free_ring_has_its_rigid_mode_and_every_pair`: the rigid turn at zero, and
  every mode twice, `4(k/m) sin²(πj/N)`, the tenth value cutting a pair.

- **What a solve returns is held to its properties** (#510 P2), not just to the reference: values
  ascending, vectors B-orthonormal (a mode `φᵀMφ = 1`, a buckling shape `φᵀKφ = 1`) and
  A-orthogonal with λ on the diagonal, each residual the pair's backward error `‖Lφ − λRφ‖ /
  ((‖L‖₁ + |λ|‖R‖₁)‖φ‖)` computed from the matrices (meaningful at zero, for a rigid mode), and
  Sylvester's count just below the top value equal to the number found below it
  (`assert_pencil_pairs`).

- **Deterministic** (#510 P2). The start and any fresh direction are draws of the seeded
  `rng::Rng` (`Opts::seed`), so a pencil has one answer. A pair converges when its Ritz residual
  is below `tol` (1e-10) of its θ, or of `FLOOR` of the largest for a θ near zero; out of
  restarts, or a count the deflated iteration cannot meet, is `Refusal::NoConvergence`, never an
  answer.

- **Dense pairs** (#510 P2; `linalg::sym_eigen`) are `sym_eigenvalues`'s Jacobi rotations,
  accumulated: the eigenvalues unchanged to the bit, and the vectors the iteration's Ritz vectors
  are made from.

- **Within the solver.** `interior.rs` and `eigen.rs` are in `gcs-solve` and know no Solvent.

- **The verification corpus is documents** (#510 P8). Each analytic case every analysis is held
  to is a Solvent document in `rust/examples/analysis/`: a solid made of a material, and a
  `scenario` whose claims bracket the case's answer. The README's table is the list of cases,
  their references and the stage that first answers each. The corpus claims its answers and
  never drives by them: a driving row or a bound in a case fails the gate. Each document states
  its own tolerance in its claims. `Case` (the test's table) holds, for each, the solid it is
  about, how it is read, the function that works out its answers (`Case::answers`) and how finely
  the oracle meshes it. Every file in the directory is a case, and the README's table reads each
  as `Case` does (`the_corpus_is_its_files`). The documents join the library's own suites
  through `fixtures::examples`.

- **A claim is its own drawing's answer** (#510 P8; `each_case_claims_its_own_answer`). The
  reference is worked out from the solved drawing, not from the document's text: its size from
  the exact solid's extent and its points, its material from the solid's `of` (SI, read back
  into the document's units), and its loads from the scenario. Each bracket the document claims
  must then be the case's factors on that reference. That is to the last bit for a bracket
  written as an expression (`0.99 * bending`, `bending := P * L^3 / (3 * steel.E * I)`), and to
  the written figures for one written as a number (a Rayleigh–Lamb root, a NAFEMS value). A
  benchmark's dimensions are SI constants beside its reference (`closed::LE1_ELLIPSES`), compared
  through the document's units. No claim may be left over.
  So a document whose geometry, material or load drifts from what its claims assume fails here,
  before any analysis reads it. The cantilever's bracket holds Euler–Bernoulli's tip deflection
  (a frame's answer) and Timoshenko's (a solid's, 0.2% more) both
  (`the_cantilevers_bracket_holds_both_beams`).

- **The closed forms are held to their equations** (#510 P8; `analysis_corpus/closed.rs`), so a
  wrong reference cannot pass a right analysis:
  - the cantilever's deflection solves the beam equation with its moment `P (L − x)`, and its
    root `β₁L` solves `1 + cos x cosh x = 0`;
  - Euler's load closes the buckled column at its free head (`kL = π/2`);
  - Lamé's stresses are in equilibrium, carry the pressure at the bore and nothing outside, and
    are compatible with one displacement, whose von Mises stress is greatest at the bore;
  - Kirsch's field is in equilibrium, free at the hole, the pull far off, and three times it at
    the hole's edge square to the pull; Heywood's finite width tends to it;
  - the plate's frequency is a root of the flexural Rayleigh–Lamb equation. It falls short of
    Kirchhoff's by the square of the thinness and becomes a Rayleigh wave when thick. A double
    cancels the equation's two terms to `(kh)⁵`, so it is checked no thinner than `h/a = 0.01`.
    Its first six modes come as (1, 1), a pair, (2, 2) and a pair.

  The plate's supports are diaphragms: each edge face is held along the edge and across the
  plate, and free square to it. Under them the modes are exact in three dimensions at any
  thickness. A NAFEMS reference is the publication's number, with its source named. The closed
  forms' root finding is their own bisection, not the core's: a reference owes nothing to what
  it checks.

- **CalculiX is the oracle** (#510 P8; `analysis_corpus/ccx.rs`, `make test-ccx`, the feature
  `gcs-core/ccx`, with `ccx` and `gmsh` on the path). It is in the tests alone, as OCCT is for
  `brep/`. Each case's solid is built by this kernel and written as STEP (`brep::step::write`).
  gmsh meshes it in C3D10, or meshes the face it sweeps in CPS6 for a case read in the plane.
  Every midside node is checked to sit on its edge, so the element order is Abaqus's. A boundary
  facet belongs to the face of the solid whose surface is nearest its middle, and every node of
  the facet must lie on that surface to ten times the STEP file's tolerance (1 µm), read by this
  kernel's own `Surface::inverse`. A face path (`beam.root`) names the facets by the face's own
  name (`Exact::named`).
  - Supports hold every node of a face, or the node nearest a point, along the world axis
    `along:` names.
  - A force on a face is a uniform traction. A pressure is integrated over each quadratic facet
    and pushes into the material.
  - The deck is in SI, its mesh written once and included by each step's deck, each asking
    only for the results its brackets read. A plane deck turns the drawing's z into CalculiX's y.
    It drops a point held across the thickness (the solid reading's), refuses any other hold
    across it, and refuses a solid whose caps are not square to y (`swept_across_y`).
  - A deflection at a point is interpolated by the shape functions of the element that holds
    it. A factor of safety is the yield over the greatest nodal von Mises stress. A mode and a
    buckling factor are read from the `.dat`. A mass is the B-rep's volume times the density.

  The cases are independent and run side by side, one thread and one directory each. Every
  claim the corpus states is kept; the test prints each answer beside its reference. That
  agreement is the evidence that each document says what its reference says: the supports, the
  units, the load's sign, and the axes, the front being x and z. It is also what S0–S3's own
  answers will be put beside.

- **How a case is read is the corpus's, not yet the language's** (#510 P8). `Reading::Plane`
  reads a case in plane stress over the face its solid sweeps, its depth the thickness; the
  oracle reads it so only for a solid swept across y. A hold across the thickness (`pinned m along:
  std.y`) is the solid reading's, taking out what the symmetry cuts leave free. How a document
  asks to be read in the plane is S2's to decide. Until it does, FV32 read as a solid bends out
  of its plane far below the in-plane modes it claims.

- **What P8 leaves.**
  - **The frame cases** (a cantilever, Euler's column, NAFEMS FV2) wait on how S0 writes a
    section. The beam's closed forms are already `closed.rs`'s.
  - **NAFEMS LE10's target** is a component of stress at a point, which no output word reads
    (an open decision of the requirement words).
  - **NAFEMS FV52's supports** are published two ways, the lower edges or the middle plane's.
    The case waits for the publication in hand, and `plate_modes.sv` is the same physics with
    supports under which the answer is exact.
  - **The oracle lays no body load**, holds only along the world's axes, reads no compliance and
    meshes one primitive's faces by name. Each is refused by name, where a later case needs it.

