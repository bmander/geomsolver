# Analysis

Part of the [contract](../../CLAUDE.md) the code is held to. Read it when you touch
`interior.rs`, `KernelKey::Interior`, or `sparse::Ata::least_pivot`; or `model/scenario.rs`,
`syntax/scenario.rs`, `syntax/parser/scenario.rs`, `program/scenarios.rs`, or write a `scenario`;
or `model/material.rs`, a `material(…)` group (`flatten::values::material`), a solid's `of`, or a
material's JSON (`io::material_json`) or report (`report::materials_json`); or `eigen.rs`,
`sparse::Sym`, `sparse::Ldl` or `sparse::Pencil`; or `delaunay/refine/cells.rs`,
`delaunay/refine/volume.rs` or `delaunay/tetra.rs`.
The epic these rules serve is #510 (design intent as structural performance), whose
prerequisites and stages (P1–P8, S0–S4) are named there; this file holds the rules landed so
far. The gate is `rust/gcs-core/tests/interior.rs` for the interior (P5),
`rust/gcs-core/tests/scenarios.rs` for scenarios (P4), `rust/gcs-core/tests/materials.rs` for
materials (P3), `rust/gcs-core/tests/linalg.rs` for the eigensolver (P2) and
`rust/gcs-core/tests/volume_mesh.rs` for the volume mesh (P6), unless a rule names another.

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

- **A volume is the surface's refinement carried inside** (#510 P6; `delaunay::refine::volume`,
  `Progressive::with_cells`, `refine/cells.rs`). Once the surface is done, as
  [fields.md](fields.md) has it, the cells are refined on the same regular triangulation. A cell
  is a tetrahedron whose orthocentre the domain holds, so the volume's boundary is the restricted
  facets, to the vertex. That stage is `Stage::Filling`, stepped as the others are. Facets are
  refined first throughout, so the surface's criteria hold to the end. A filling refinement
  takes its last build's surface, never an earlier one, since the cells are that build's.
  Rounds of judging every cell (`FILL_ROUNDS`) end at the first round that inserts nothing. A
  surface left not manifold is a refusal, never a volume. `Cells::check` refuses criteria no
  refinement meets: a radius–edge bound under 1, no size, a dihedral bound of 30° or more.

- **A cell is judged by its radius–edge ratio, its size and its least dihedral angle**
  (`Cells`; `tetra::radius_edge`, `circumsphere`, `least_dihedral`). Each kind of bad cell is
  refined in its own way:
  - **Too long for its shortest edge, or too big:** the cell's orthocentre goes in. It is at
    least its ball's radius from every vertex, so refinement ends.
  - **Its point in a restricted facet's surface ball:** the facet is refined instead
    (`Bad::forced`, Mesh_3's rule), and the cell is judged again.
  - **A sliver** (a fair ratio, its least dihedral under `Cells::dihedral`): a point from its
    picking region (Li and Teng). That is `CANDIDATES` seeded draws in `PICK` of its radius about
    the orthocentre, and the best goes in, by the least dihedral angle of the tetrahedra it would
    make, among the draws that reach no surface ball and that the domain holds. The domain is
    asked last, once a draw has passed every other test.
  - **Its point in a protecting ball, or a ball under the margin:** the cell is left and
    counted (`CellReport`), and never judged again while it stands (`Filling::left`, by its
    corners). No cell shrinks a ball. Trying the picking region for a centre in a
    ball was measured and refused: the cube's bad cells went from 19 to 3,711.

  On a smooth domain every cell meets radius–edge 2 and a 12° dihedral
  (`a_ball_is_filled_with_fair_cells_and_curved_elements_on_its_sphere`, the torus, the
  ellipsoid). With features, every cell left bad has a protecting ball's centre for a corner
  (`Volume::protected`; the cube, 19 of 2,639 cells, least dihedral 7.8°; the cylinder, 5 of
  3,624).

- **The quadratic element is TET10, its nodes numbered as CalculiX numbers them**
  (`tetra::Tet10`, `EDGES`). The corners come first, then the edges 0–1, 1–2, 2–0, 0–3, 1–3,
  2–3. An edge node is its edge's middle, except on the boundary (`refine/volume.rs`):
  - **On a feature curve** (two protecting balls next to each other on one curve), the node is
    at the curve's arc-length middle between them. `protect` gives each ball's neighbours and
    those points (`Protection::halfway`), where the stations are known.
  - **On any other boundary edge,** the node is where the domain's boundary crosses the line
    through the middle along the facets' mean normal, within a quarter of the edge. It is found
    as a surface centre is, by Newton on readings or by bisection, to `Criteria::bisection`. An
    edge whose line meets no boundary stays straight, and is counted.

  Every node is shared by every element on its edge, so the mesh conforms. On the ball, the
  ellipsoid and the torus, every boundary node is on the surface to 1e-8. The quadratic
  elements hold the volume about 600 to 2,800 times better than the linear cells: the ball 1.6e-2
  against 2.6e-5, the torus 1.3e-2 against 9.5e-6, the cylinder 5.1e-3 against 1.8e-6. That is
  the chord's error against the parabola's.

- **No element is inverted, and the proof is its Bernstein coefficients** (`Tet10::
  jacobian_bernstein`, Johnen, Remacle and Geuzaine). A TET10's Jacobian determinant is a cubic
  in the barycentric coordinates. Its twenty cubic Bernstein coefficients bound it below
  everywhere in the element, and the four at the corners are its values there. They are formed
  exactly from the columns' corner values, and their mean over 120 is the element's volume
  (`Tet10::volume`). An element whose least coefficient is under `LEAST_JACOBIAN` (0.2) of its
  straight element's `6V` (`jacobian_ratio`) has its curved edges put back straight, and its
  neighbours are judged again, until none fails. A straight element is its positive linear cell.
  `a_tet10s_jacobian_is_bounded_by_its_bernstein_coefficients` holds the corners, the bound on a
  lattice, the volume's closed form and a folded element.

- **A volume is held to what a complex is** (`conforming`): every cell positive, every face on
  one cell or two, the faces on one exactly the boundary's outward triangles, the boundary
  closed and manifold, and the Euler characteristic V − E + F − T the solid's: 1 for a ball, 0
  for a solid torus. The cube's linear and quadratic volumes are exactly 1, to 1e-9, with its
  corners as vertices. Filling is deterministic: two runs give the same mesh to the bit, and
  the slivers' draws come from the seeded `rng::Rng` (`cells::SEED`). The boundary is the
  restricted facets `Refiner::extract` gives, renumbered, so the surface mesh and the volume's
  boundary are one list. `conforming` also holds the volume the boundary encloses to the cells'.

- **What P6 leaves to S3.** No element stiffness reads a `Volume` yet. A protecting ball's
  neighbours keep cells no refinement reaches. Mesh_3 removes those slivers by perturbation and
  exudation, which need a vertex moved or reweighted, and this triangulation removes none. The
  cells have one size, with no grading toward a feature or a load. The volume is built in the
  terminal and the tests only, not in the page's mesh worker. A `MaterialField` is a `Domain`
  like any other, but no gate fills one yet.
