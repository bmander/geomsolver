# Analysis

Part of the [contract](../../CLAUDE.md) the code is held to. Read it when you touch
`interior.rs`, `KernelKey::Interior`, or `sparse::Ata::least_pivot`; or `model/scenario.rs`,
`syntax/scenario.rs`, `syntax/parser/scenario.rs`, `program/scenarios.rs`, or write a `scenario`;
or `model/material.rs`, a `material(…)` group (`flatten::values::material`), a solid's `of`, or a
material's JSON (`io::material_json`) or report (`report::materials_json`); or `eigen.rs`,
`sparse::Sym`, `sparse::Ldl` or `sparse::Pencil`; or `diagnose/analysis.rs`, `Structure::free`,
`Structure::reach`, `BodyFaces::whole` or a scenario's report (`report::scenarios_json`); or
`delaunay/refine/cells.rs`, `delaunay/refine/volume.rs` or `delaunay/tetra.rs`; or a case of
`rust/examples/analysis/` or `tests/analysis_corpus*`; or `interior::Beam`, `model/beam.rs`,
`syntax/beam.rs`, `program/beams.rs`, `beams.rs` or a scenario's answers
(`report::scenario_answers`); or `CKind::Deflection`, `beams::Row`, `interior::Spread`,
`interior::Sizing`, `model::Over`, `Sketch::scenarios` or `program::scenarios::drive`; or
`interior::Tri6`, `interior::Laid`, `interior::Moved` (`interior/plane.rs`, `interior/moved.rs`),
`delaunay/plane.rs`, `prisms.rs`, a scenario's `plane stress`/`plane strain` line
(`model::PlaneReading`), `Sketch::meshed` or `DeflectionRow`; or `interior::Tet10`,
`interior::Borne`, `interior::Stack` (`interior/solid.rs`), `bodies.rs`, `beams::Read::errors`,
`cholesky::Cholesky::factor_ldl` or `eigen::UNBUCKLED`; or `deformed.rs`, `overview/scenario.rs`,
a reader's `read_pictured` or `report::scenarios_pictured_json` (a scenario drawn).
The epic these rules serve is #510 (design intent as structural performance), whose
prerequisites and stages (P1–P8, S0–S4) are named there; this file holds the rules landed so
far. The gate is `rust/gcs-core/tests/interior.rs` for the interior (P5),
`rust/gcs-core/tests/scenarios.rs` for scenarios (P4), `rust/gcs-core/tests/materials.rs` for
materials (P3), `rust/gcs-core/tests/linalg.rs` for the eigensolver (P2),
`rust/gcs-core/tests/analysis_words.rs` for the diagnosis words (P7),
`rust/gcs-core/tests/volume_mesh.rs` for the volume mesh (P6),
`rust/gcs-core/tests/analysis_corpus.rs` for the verification corpus (P8) and
`rust/gcs-core/tests/beams.rs` for frames (S0), `rust/gcs-core/tests/driven.rs` for a
frame's driving row (S1), `rust/gcs-core/tests/prisms.rs` for solids read in their plane and
a region's driving row (S2, its mesher `rust/gcs-core/tests/plane_mesh.rs`), and
`rust/gcs-core/tests/bodies.rs` for solids read in three dimensions, a solid's driving row and the
errors estimated beside every answer (S3), and `rust/gcs-core/tests/scenario_view.rs` for a
scenario drawn, unless a rule names another.

- **An analysis output is a row, and its interior is a kernel** (#510 P5). A requirement whose
  value comes from `K(x)u = f`, `x` the drawing's numbers and `u` the displacements, is a kernel
  built at its block's widths (`KernelKey::Interior { n_par, n_const }`, `interior::kernel`) as a
  spline's length is (`integral.rs`): one row reading every number the structure is made of. A
  structure's displacements are thousands of numbers the drawing does not hold, its solve is
  linear, and it needs one derivative for many columns, which the adjoint gives. As a row it is
  taken as every other row is, by the flat solve, the decomposition, the diagnosis, a claim and
  a free twin.

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
  already has (`sparse::Ata`); a plane structure's `K` is the same rows summed into each element's
  matrix and factored by P1's supernodal Cholesky under minimum degree (S2, below). A support
  holds a degree of freedom at zero by leaving its column out of `S`. A frame's or a continuum's element joins as more rows (a Cholesky factor of its
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
  (`a_structure_with_no_displacement_to_give_says_why`). `Structure::free` names the free motion
  (P7, below).

- **Held to closed forms and to its differences** (#510 P5). The apex of two bars sinks
  `P L³ / (2 E A h²)` to 1e-12, and its rates in the rise and in each area are the closed form's
  (`a_two_bar_truss_sinks_as_its_closed_form`). A bar in space stretches `P L / (E A)`
  (`a_bar_in_space_stretches_by_pl_over_ea`). Newton on the row alone finds the area that sinks
  the apex half a millimetre, to the closed form's (`a_requirement_drives_the_area`). The
  adjoint gradient agrees with central differences of the value. That holds for braced,
  redundant structures in the plane and in space, stated and free, and for two instances in one
  block (`tests/jacobians.rs`, `a_linear_interior_agrees_with_finite_differences`).

- **What P5 leaves to later stages.** ~~No `CKind` compiles to the kernel~~: S1's
  `CKind::Deflection` does (below), in `undrawn!`.  S0 reads the structure directly, for claims
  (below). Every call reads the structure from
  its constants and orders `K`'s pattern again (`Ata::new`'s symbolic analysis), though neither
  moves with `x`. The system also calls a kernel's residual and Jacobian separately, so a step
  that asks for both factors `K` twice. Holding the symbolic analysis per row (a hit on every
  step) and the factor between the two calls (a hit only at one `x`) are two caches, to be
  measured first and apart, as `solve/through.rs` measured its own. The element interface is
  S0's (below).

- **The beam is the interior's second element** (#510 S0; `interior::Beam`). An Euler–Bernoulli
  beam rigid at both ends, of modulus `e`, shear modulus `g` and density `rho`, its section's z
  a constant direction (the normal of the plane it is drawn in). Its local axes come from its
  run `Δ` and z: e1 along it, e2 = z × e1, e3 = e1 × e2 (`Beam::axes`); z along it is
  `Failure::Collapsed`. **Degrees of freedom are numbered apart from coordinates**: a node has
  `dim` coordinates (columns) and, once there is a beam, `Structure::per` degrees of freedom —
  its moves, then its turns (one about z in the plane, three in space); `Structure::dof` maps a
  node's component in space to its degree of freedom, `None` where the plane has none. **A
  section is four columns** after the bars' areas, in `SECTION`'s order: `A`, `Iy`, `Iz`, `J`.
  `fixed`, `load` and `weight` are per degree of freedom, as for bars.

- **A beam's stiffness is six strain rows** (#510 S0; `Beam::local`), each over its twelve
  degrees of freedom in its own axes at a unit section, then turned into the world's
  (`beam_rows`): the stretch `√(E/L)(u_b − u_a)·e1`; in each plane it bends, two rows
  `√(E/L)(2φ_a + φ_b)` and `√(3E/L)φ_b`, `φ` an end's turn against the chord — a Cholesky factor
  of `E I/L·[4 2; 2 4]`; and the twist `√(G/L)(θ_b − θ_a)·e1`. Each group scales by the root of
  its column, so the element's stiffness is linear in its section. In the plane only the stretch
  and the bending about z are rows (`IN_PLANE`, read once in `beam_local`). So the beam joins
  `solve` and `free`'s mechanism by the one row builder (`rows`), as P7 asked; `stiffness` (a
  `Sym`, for a pencil) sums the same rows into each beam's twelve-by-twelve matrix in its axes,
  turned as the mass and the geometric stiffness are (`beams_into`). A node's components are one
  table (`components`: which of the six in space the structure has), so `per`, `dof` and its
  inverse read it alone. `free` holds a held turn as the body's turn, and where there are beams the body is
  no longer the nodes' places alone: a twist about a line of nodes turns them.

- **A beam's gradient is its mutual energy's, carried in its axes** (#510 S0;
  `Structure::gradient`). Each group's mutual energy at a unit section, `Σ (r·λ)(r·u)`, is
  formed in `kernels::Dual<3>` over `Δ` (the axes and the length carry their rates), so the
  gradient in the nodes' coordinates is exact and the section's column reads the group's energy
  itself. It agrees with central differences of the value over random braced frames in the plane
  and in space, stated and free (`jacobians.rs`, `a_frames_interior_agrees_with_finite_differences`;
  a frame's normal equations square a stiffer conditioning than a truss's, so its difference is
  taken over 1e-4). The constants carry beams (`HEAD` is `[dim, nodes, bars, beams]`, each beam
  `(a, b, E, G, ρ, z)`), written and read back exactly (`a_frame_is_its_constants`). No word
  compiles to the row yet: that is S1's.

- **A cubic is the beam's exact deflection under end loads** (#510 S0). A cantilever's tip
  sinks `P L³/3EIz` and turns `P L²/2EIz` in its plane, moves `P L³/3EIy` across it, twists `T
  L/GJ` and stretches `P L/EA`, in one element or several, in the plane and in space, to 1e-10
  (`a_cantilever_bends_twists_and_stretches_as_its_closed_forms`). **Its modes and its buckling
  are the cubic's to approximate**: the consistent mass (`Structure::mass`: `ρAL/6·[2 1; 1 2]`
  along it, `ρ(Iy + Iz)L/6·[2 1; 1 2]` in its twist, `ρAL/420·[156 22L 54 −13L; …]` in each
  bending plane, no rotary inertia, a bar none) and the geometric stiffness
  (`Structure::geometric`, each beam's pull `N`: `N/30L·[36 3L −36 3L; …]` in each bending
  plane, `N(Iy + Iz)/(AL)` in its twist), read by `eigen::lowest` and `eigen::buckling`. Cut into
  sixteen, a cantilever's first two frequencies are within 1e-6 and its third 1e-5 of `βᵢ²/(2πL²)
  √(EI/ρA)`, and its first load factor within 1e-6 of Euler's
  (`a_cantilevers_modes_and_buckling_are_its_closed_forms`). `Structure::end_actions` gives each
  beam's `k_e u_e` in its own axes, the far end's pull at `[6]`; a load spread along it is the
  caller's to take out.

- **A beam is a statement of the language** (#510 S0; `arm := beam(span, post, A: …, Iy: …, Iz:
  …, J: …[, Sy: …, Sz: …]) of steel`, `syntax::BeamDecl`, `StmtKind::Beam`). `beam` is a word
  only as a definition's call, as `material` is, and `of` the trailer a solid's is. The lines
  come first, unlabelled; then the section's numbers, each labelled (syntax errors otherwise, and
  for a beam on no line). It flows as a scenario does: the flattener names it under its scope's
  prefix, writes the scope's numbers into its section (`numbers_mut`) and rescopes its lines and
  material (`refs_mut`); `program::beams` reads each, once the geometry is built, into a
  `model::Beam` on `Sketch::beams` (gcs-solve, pure data): its lines by index, its `Section` in
  the **document's units** (as every length is; the material stays in SI), its material by name.
  Refused, each where it is written: a target that is no line (**E040**) or a line standing in
  space (**E040**: its section's z is the plane's normal); a name of nothing (**E101**); a label
  no section has (**E103**), twice (**E001**); a number of the wrong power of length, at zero or
  less (**E103**), or reading an unknown of the drawing that is no seeded length or plain number
  (**E040**; S1 reads a section over the drawing's unknowns, below); a
  section leaving out any of `A`, `Iy`, `Iz`, `J` (**E103**); `of` naming no material
  (**E103**, judged in `flatten::resolve::rewrite` as a solid's is). A beam that fails any check
  is left out whole: a frame missing a member is another frame. `Sy` and `Sz`, the elastic
  section moduli, are optional: only a factor of safety reads them. **A beam adds nothing to the
  drawing** (`a_beam_is_its_lines_section_and_material`): no entity, unknown or row, and the
  sketch's JSON is the document's without it, so it is carried by the Solvent source alone, as
  a solid's `of` is. The printer writes it back as written; an edit removing a line it names
  never takes the beam with it (E101 is said).

- **A scenario names a beam as the beam it is** (#510 S0; `Applied::Beam`, its index on
  `Sketch::beams`). `Applied::entity` is `None` for one, and `Applied::solid` names a solid's.
  A beam is what a body load (`accelerate arm …`), a force spread along it (`load arm f along:
  d`) and an output (`mass(arm)`, `mode(arm, 1)`) are of; a support on one is **E040** ("hold its
  ends"), and so is a pressure. `Stated::heads` keeps each requirement's output as written,
  `deflection(tip, along: down)`, for the report.

- **The drawing is the mesh** (#510 S0; `beams.rs`, `Mesh`). Every line a beam names is a
  member. Members join where their ends meet, or where an end lies on another member, by place
  (`JOINED`, 1e-6 of the frame's extent): a **joint**. A point the scenario names (a support, a
  load, an output's) lying on a member between its ends cuts it there into a joint of its own
  (`a_point_on_a_member_cuts_it`). Between joints a member is cut into `PIECES` (8) cubic
  elements, its section's z the normal of the plane its line is drawn in. A **frame** is the
  members joined to one another (`a_frame_joins_where_its_members_meet`: an L, rigid at its
  corner, and a tee); a requirement reads the frame of what it names — a beam, a member's line, a
  point at a joint — and one naming two frames has no answer. Nothing else is meshed, and the
  frame stands where the solved drawing puts it.

- **A frame is read under the scenario where it falls on it** (#510 S0; `Analysed::new`). A
  support holds every joint it names a point of (a point; a line's every node): its moves, or
  along `along:` its move along that one world axis (a direction off the world's axes is no
  answer yet), and where it is `fixed` its turns. A force at a point is a load at its joint; a
  force on a beam or a member's line is spread evenly along its members, and `accelerate` is
  `ρ A a` a length along them; each spread load is its consistent load at the elements' ends
  (`interior::fixed_end`), exact for the cubic. The material is read back into the document's units
  (`Units::from_si`): `E`, `G = E / 2(1 + nu)` (a frame twists, so `nu` is needed), `rho` where
  a mass is read, `yield` for a safety. A material that does not state what is read of it, a
  beam made of none, a frame the supports leave free (`Structure::free`, named in the solver's
  words and said once as the scenario's word: `` scenario `loose` leaves `arm` free to turn about
  x through (0, 0, 0), … ``) are reasons, never numbers (`what_has_no_answer_says_why`).

- **What a frame answers** (#510 S0; `beams::read`, `answer`), in the document's units:
  - `deflection(X[, along: d])`: a joint's move along `d` (signed), or its length; of a beam or
    a line, the worst over its nodes;
  - `compliance(B)`: `fᵀu`, the work the scenario's loads do on the frame;
  - `mass(B)`: `ρ A L` over the members `B` names;
  - `mode(B, n)`: the frame's `n`th frequency, `√λₙ / 2π` of `eigen::lowest(K, M)`;
  - `buckling(B)`: the least factor on the loads, `eigen::buckling(K, K_G)`, `K_G` from each
    element's pull at its middle; none where nothing is pressed;
  - `safety(B)`: the least yield over the greatest normal stress along the members `B` names,
    `|N|/A + |My|/Sy + |Mz|/Sz` at the section's farthest fibres. Each is a quadratic along an
    element (its near end's actions less the spread load's, and the spread load), so the
    greatest of their sizes is the greatest of the eight signed sums, each at an end or its
    vertex (`stress`): exact. Euler–Bernoulli's: no shear, and the twist's is not read.

  A requirement about no frame (a solid's) has no answer of this kind (`None`). **Every frame
  case of the corpus answers its reference** (`every_frame_case_answers_its_reference`): to 1e-9
  where the cubic is exact — a load at a joint or spread evenly, the stress they give, a mass —
  and to 1e-5 for a mode or a buckling load; and keeps every claim it states.

- **A claim is judged; nothing is solved for** (#510 S0; `beams::keeps`,
  `report::scenario_answers`). A value keeps a requirement on its side of `<=`/`>=`, or within
  1e-9 of what it states for `==`. A claim is said `kept` or `refuted`; a driving row or a bound
  is said to hold or not, and that nothing drives it yet (S1, S4). `solventc` prints each under
  the scenario's words, `` scenario `tip_load`: deflection(end, along: down) is 6.4mm: claimed
  >= 6.336mm, kept `` (a frequency in hertz, a mass in kilograms, a compliance in joules, a
  factor plain, six figures: `beams::value_text`), and `--json` gives them as each scenario's
  `answers` (`report::scenarios_json`: the requirement, its grade, its value, whether it holds,
  the sentence). A refuted claim is a report, not an error: the document still exits 0
  (`gcs-cli/tests/cli.rs`, `a_frames_answers_are_said`). The words of a scenario's solids
  (`scenario_words`) leave out the supports that hold a frame, and a scenario of frames alone
  holds no body of its own (`diagnose::analysis::scenario_read`).
  **The page says them too**, from the same JSON (FFI `gcs_elab_scenarios_json`,
  `Document.scenarios`; in the page `gcs_elab_scenarios_pictured_json`, read in a worker — "A
  scenario drawn", below): read again after every solve (`SketchView.afterEdit`,
  `view.scenarios`), counted in the footer's `#scenarios` line — what holds, what fails, what has
  no answer, each word — with every sentence as its tooltip, red and standing while one fails,
  and listed in the Diagnosis report.  A document whose requirements fail never looks solved and
  silent (`core.test.ts`, `a document says what its load scenarios answer`).

- **What S0 leaves.**
  - ~~**No row.**~~ S1 compiles a driving deflection to the beam's row (below).
  - **A beam's line is drawn in a plane**, whose normal is its section's z; a line in space,
    and a section turned about its member (an `up:`), are not read yet.
  - **A support holds along the world's axes**, and `fixed` holds every turn: a support holding
    one turn and not another (a simply supported beam's ends in space, free to bend and held
    from twisting) has no spelling, so such a beam is free to twist about its own line, and said
    so.
  - **A factor of safety leaves out the twist's shear**, and stress is read on Euler–Bernoulli's
    terms (no shear stress). The failure criterion is still the open decision's.
  - **NAFEMS FV2** (the pin-ended double cross) waits on its published values in hand, and on a
    pin: every joint is rigid.
  - **No load at a moment**: a scenario's load is a force.
  - **What is worked out once is per read**: a frame is analysed once per scenario, its `K`
    assembled once, its modes solved once for the most any requirement asks, its buckling and
    its end actions once (`Analysed`'s cells); but nothing is kept between reads, and a pencil
    factors `K` again beside the static solve's normal equations.

- **A frame's deflection, driven, is a row** (#510 S1; `CKind::Deflection`, `beams::Row`,
  `program::scenarios::drive`, §9.11 [0.168]). `deflection(p, along: d) == v` in a scenario, `p` a
  joint of a frame (`beams::of_frame`), is compiled by the elaborator, once every scenario is read,
  into a constraint of the drawing: `[at: Point, along: Direction, scenario: Int, d: Length]`, the
  scenario's supports and loads held for it on `Sketch::scenarios` (only a scenario that drives is
  there), its site the requirement's line, and `Stated::driven` its id. It is the interior's
  kernel (`KernelKey::Interior`, built at its frame's widths; one row, degree 1, `d·u − v`), so
  the flat solve, the decomposition and the diagnosis take it as any row: the sized cantilever
  leaves no freedom (`a_requirement_sizes_a_section`). Any other driving requirement — a solid's,
  or a frame's mode, mass, buckling, compliance or safety, or a deflection with no `along:` —
  compiles nothing, and the report says nothing drives it yet. The kind has no free twin (a
  scenario's number reads no unknown) and is in `undrawn!`: a requirement's figure is the deformed
  frame, the drawing's to say.

- **The row's frame is meshed at one element between joints** (`Mesh::new(…, 1)`): every node is a
  joint, so every node is a point of the drawing — a member's end, or a point the scenario names
  lying on one — and its three columns are that point's lift (`Constraint::lifted_points`, minted at
  the add). A joint is a point by identity first, then by place (`Mesh::joint`, `at_point`), so
  members sharing a point join wherever a step puts it. One element is exact here: under loads at
  the joints and loads spread evenly, the cubic's nodal moves are the beam's, which is all a static
  deflection reads, and the claims' reading at `PIECES` agrees to rounding. The topology is read
  where the drawing stands when the row is compiled and kept through that solve: within it the nodes
  move, and the members never change. The weights, and a load's `along:`, are read as constants of
  the row, so a direction the solve moves (an axis whose direction is not held, a line whose ends or
  plane are not) is refused (**E040**, `Sketch::direction_held`), as is a frame the supports leave
  free (`Row::check` in `validate`, the solver's words).

- **A section read over unknowns** (#510 S1; `model::Over`, `interior::Sizing`). A section's number
  may read the drawing's declared unknowns (`param h: Length hint(40mm)`, `Iz: b * h^3 / 12`): it is
  worked out at their seeds and checked for its dimension and sign (`program::beams::number`), and
  kept as its text, the unknowns it reads and its tape over them. Any function of them, not only one affine in one: each
  is a tape (`tape.rs`) over the row's unknowns, a stated number a constant tape (`Tape::compile` of
  the number). The row's columns are the joints' lifts, then those unknowns (at most
  `tape::MAX_VARS`), bound by the expression pass when a row reads them (`expr::evaluate`, from
  their seeds, as an envelope's set's are). `Section::read` reads a number at the drawing — a bound
  unknown's value, else its seed — and the claims' analysis reads the section so too. An unknown
  that is an angle (a tape reads no angle as the drawing holds one) or has no seed is **E040** where
  the section reads it, and one no row binds — no driving row on its frame and no dimension — is
  **E040** too (`program::beams::sized`): nothing determines it. The kernel solves the structure at
  the sections the tapes give, and carries each section's rate to the unknowns by the tape's
  gradient (`interior::jac`); the constants carry the tapes after the structure (`HEAD`'s `sized`,
  −1 where the sections are columns of their own, so P5's and S0's rows are unchanged).
  `Structure::read_sized` refuses constants that are not whole, which the kernel reads as NaN.

- **A spread load moves with its members** (#510 S1; `interior::Spread`, `Structure::forces`). A
  force spread evenly along beams is its whole over their length, `q = F / Σ L`, the same force
  however long they grow; a body load is `q = ρ a A`, growing with the area. Each beam's share is
  its consistent load (`interior::fixed_end`, one formula for what `Structure::forces` lays at its
  ends and what a reading of its end actions takes out), `q L/2` at each end and `± L/12 (Δ × q)`
  its end moments in the world's axes, exact for the
  cubic (`a_spread_load_is_its_consistent_load`: `q L⁴/8EI` and `q L³/6EI` in one beam or
  several). So `f` is `f(x)` and the gradient is `λᵀ(∂f/∂x − ∂K/∂x u)`: `λᵀfₑ = q · wₑ`, `wₑ`
  formed in `Dual<3>` over the beam's run (`spread_into`). The frames' analysis states its loads
  so (`Framed::new`), and reads the compliance as `fᵀu` of them. The adjoint gradient agrees with
  central differences over random braced frames with both kinds spread, and over frames whose
  sections are read over two unknowns (`jacobians.rs`,
  `a_frames_interior_agrees_with_finite_differences`,
  `a_sized_frames_interior_agrees_with_finite_differences`); the constraint agrees as the drawing
  states it, sized, placed, loaded by its weight and spread, at the solution and knocked off it
  (`a_frames_deflection_agrees_with_finite_differences`).

- **Held to closed forms** (`tests/driven.rs`): a depth sized so the tip sinks `δ`,
  `(4 P L³ / (E b δ))^(1/3)`, for three sags; a length placed, `(3 E I δ / P)^(1/3)`; a depth
  under its own weight, `√(1.5 ρ g L⁴ / (E δ))`, and under a force spread evenly, `(1.5 P L³ / (E
  b δ))^(1/3)`, and a length so, `(8 E I δ / P)^(1/3)`; an L of two members sized by its arm's end,
  the root of one cubic — each to 1e-9. The report says `driven == 4mm, held`, or `not reached`.
  **The library's driving cases** are `sized_cantilever.sv` (a depth), `reach_cantilever.sv` (a
  length), `shelf_bracket.sv` (an L under a load spread along its arm) and `portal_frame.sv` (a
  tube's diameter, non-affine, under a load at a point cutting a member and the frame's weight):
  each leaves no freedom, its deflection read at `PIECES` is what its row states, and its claims
  are kept (`every_driving_case_holds_what_it_drives`); the reach and the bracket are their
  closed forms (`the_librarys_reach_and_bracket_are_their_closed_forms`), and each row agrees
  with its differences (`a_frames_deflection_agrees_with_finite_differences`).

- **The row is the source's alone** (`CKind::of_source`): it reads beams and a scenario, which no
  document JSON, rebuild walk (`graft`) or flat program (`to_program`) holds, so none of them
  writes the row; the source states it again when it is elaborated (`the_row_is_the_sources_alone`).

- **What S1 leaves.**
  - **One output drives**: a deflection of a joint along a direction. A deflection with no
    `along:` (a length, not linear in `u`), a beam's worst (a maximum), a compliance (`f(x)ᵀu`,
    its own `∂J/∂x`), a mass, a mode (`φᵀ(K' − λM')φ`, P2's open derivative) and a safety (a
    maximum: S2's aggregation) are claims yet.
  - **A drag and a rebuild drop the row**: `graft` copies neither beams nor scenarios, so a part a
    gesture moves, a copy and a paste carry no frame row. Carrying them means remapping a beam's
    lines and a scenario's targets as every entity reference is remapped.
  - **The topology is the compile's**: a point the scenario names cut into a member by place, at
    the drawing the row is compiled at; a point held on a member that a step moves off it bends
    the member at it rather than leaving it. Remeshing as the drawing moves (the epic's first
    difficulty) is not needed while the joints are points.
  - **The two caches P5 named** are still not kept: each call reads the frame from the sketch
    once per compile and factors `K` once for the residual and again for the Jacobian.

- **A solid is read in its plane where a scenario says so** (#510 S2; `plane stress B`, `plane
  strain B`, §9.11 [0.170]; `syntax::ItemKind::Reading`, `model::PlaneReading`, `InPlane`). The
  open decision of how a document asks to be read in the plane, decided: a line of the scenario,
  since how a body is read is the analysis's, beside its supports and loads, and the corpus's own
  oracle reads one document either way. `plane` at a line's head (the parser's `scenario_item`),
  `stress` or `strain` after it, and a target, contextual words tinted as the parser reads them
  (`highlight::scenario_word`) and printed back as written. Elaborated (`program::scenarios`,
  `Reading::reading`): the target is a solid (**E040** otherwise) whose definition is a prism of a
  face swept square to its plane with no draft (`SolidDef::Prism { draft: None }`, **E040**
  otherwise — a turned, cut or drafted body has no one face it is read over); a solid read twice
  in one scenario is **E001**; an extended scenario begins with its base's readings, and its own
  replaces its base's for a solid (`the_reading_is_a_line_of_the_scenario`). A reading adds
  nothing to the drawing. `Scenario::plane_of` says how a solid is read.

- **The plane element is the interior's third** (#510 S2; `interior::Tri6`, `interior/plane.rs`).
  A quadratic triangle, its nodes numbered as CalculiX numbers a CPS6 (corners counter-clockwise,
  then the middles of the sides 0–1, 1–2, 2–0), isoparametric: a side whose middle node is off its
  chord is curved. Its stiffness is the Gram matrix of strain rows as every element's is: at each
  of the degree-two rule's three points, `√(w·det J·t)·Cᵀ B`, `C` the Cholesky factor of `D` —
  plane stress `E/(1 − ν²)·[1, ν, (1 − ν)/2]`, plane strain `E/((1 + ν)(1 − 2ν))·[1 − ν, ν, (1 −
  2ν)/2]` — nine rows, its rank exactly, so no spurious motion. `det J ≤ 0` anywhere is
  `Failure::Collapsed`, `E ≤ 0` or `ν` outside (−1, ½) `Unstiff`. Its consistent mass `ρ t ∫ NᵀN`
  and area `∫ det J` take Dunavant's degree-four rule (exact for a curved element's area). Its
  stresses at its nodes are `D` times the strain read there (`Tri6::stresses`), and its von Mises
  stress reads `σz = 0` in plane stress and `ν(σx + σy)` in plane strain (`Tri6::von_mises`). Its
  gradient is its mutual energy's in `Dual<12>` over its twelve coordinates
  (`Structure::gradient`), as the beam's is over its run. A node of a plane structure moves along
  x and y and turns not at all (`Structure::components`); bars, beams and plane elements are not
  mixed (the constants are refused).

- **A plane structure is factored under minimum degree** (#510 S2; `interior::Factor`). A mesh's
  thousands of unknowns, which a band order fills as `n^{3/2}`: its `K` is assembled from each
  element's twelve-by-twelve `Σ rrᵀ` and factored by `cholesky::Cholesky` under AMD; a pivot lost
  to rounding (`NotPositive`) is `Failure::Free`, which the supports name (below), never the
  factor. The adjoint reads the same factor (`Factor::resolve`). On a region of 6,000 unknowns the
  factor took the residual from 219 ms to 31 ms.

- **A load laid on the plane moves with it** (#510 S2; `interior::Laid`). A pressure `p` on sides
  of depth `t`, `−p t ∫ N n ds` (each side `[first, middle, last]` run with the material on its
  left, the outward normal times `ds` the tangent turned clockwise: linear in the places); a force
  spread evenly along sides (`F ∫ N ds / Σ L`) or over elements (`F ∫ N dA / Σ A`), the same force
  however the region grows; a body load `b t ∫ N dA`. Each node's share is laid by
  one function per kind of share (`side_load`, `element_load`), over a plain number for the loads
  and over `Dual` for `λᵀ∂f/∂x`, an even load's whole also reading `−(Σ λᵀfₑ)/S² ∂S`. The patch
  test holds to 1e-9 (a sheet of knocked-about elements pulled: `σx = σ` at every node of every
  element, its stretch `σ w/E'` and narrowing `−ν'σh/E'` in either reading,
  `a_sheet_pulled_stresses_uniformly_as_the_patch_test_asks`); the element is rigid in three
  motions and only them, weighs `ρ t A`, and each load lays its whole
  (`a_plane_element_is_rigid_in_three_motions_and_weighs_its_area`); the gradient agrees with
  central differences over sheets with curved elements under every load at once, through the
  kernel so the constants read back (`a_plane_interior_agrees_with_finite_differences`, all in
  `tests/interior.rs`).

- **A region is meshed by Delaunay refinement** (#510 S2; `delaunay::plane::mesh`). Its loops of
  sides (each a curve on `[0, 1]`), the outer turned counter-clockwise and the holes clockwise so
  the material is on every segment's left. A curved side is first cut where a piece would turn
  more than `Criteria::turn`, and every boundary point lies on its curve. The triangulation
  (Bowyer–Watson on the exact `orient2` and `in_circle`, `delaunay::predicates`) conforms: a
  segment a vertex encroaches (inside its diametral circle) is split at its parameter's middle, on
  the curve. Then Ruppert: a triangle of the region whose circumradius passes `Criteria::ratio` of
  its shortest side (√2: every angle over 20.7°) or `Criteria::size` gets its circumcentre, unless
  that encroaches a segment, which is split instead; a small feature grades its neighbours to its
  size. Past `Criteria::most` points it is refused (a corner too sharp for refinement to end), and
  so is a loop that does not close. Each element's middle nodes are a segment's curve point at its
  parameters' middle, else the chord's middle; every boundary node knows its side and parameter
  (`Mesh::on`), each side its segments in order (`Mesh::sides`). Parameters a side must be cut at,
  and points inside, are nodes (`Side::cuts`, `Region::points`). Held to (`tests/plane_mesh.rs`):
  every angle over 20.7°, conforming (every inner side on two elements, the boundary exactly the
  segments, each with an element on its left), every element valid, the areas the region's to
  1e-7 (a parabola through three points 1.4° apart on a circle), the hole's nodes on it, graded
  ten to one, the same run to run, named points nodes.

- **A region read in its plane** (#510 S2; `prisms::read`, `Region`, `Plated`). The face's loops
  walked into sides (`Bound`: a line, an arc, a circle, a spline's or a curve's span), meshed at
  `TURN` (π/72) and `SIZE` (1/24 of the region's extent); the points the scenario names in the
  region are nodes. What a line names falls on the region: a face path's faces (`BodyFaces::whole`)
  whose patch holds a side's quarter points at the prism's middle are those sides, and a cap (a
  plane square to the face's normal) the whole; the line a side was swept from, that side; the
  solid or its face, the whole; a point within the region, its node. A support holds its nodes
  along the plane's axes — `along:` one of them, off them no answer yet, across the thickness
  nothing; a force at a point is a nodal load, on sides `Laid::Along`, on the whole `Laid::Over`;
  a pressure on sides `Laid::Pressure` at the prism's depth; `accelerate` `Laid::Body` of the
  material's `rho`. The material is read back into the document's units: `E`, `nu` (said where
  missing: a reading in the plane reads Poisson's ratio), `rho`, `yield`.

- **What a region answers** (#510 S2; `prisms::answer`): `deflection` a node's move in the plane
  along `d` (or its length), a side's or the solid's worst; `compliance` `fᵀu`; `mass` `ρ t Σ A`;
  `mode` `eigen::lowest(K, M)`; `safety` the yield over the greatest von Mises stress at the nodes
  `B` names, each node's stress the average of its elements' there. **No answer, and why**: a
  region its supports leave free, named in the solver's words with every node held across the
  plane (`analysis::freedoms` in space: `` scenario `pull` leaves `plate` free to slide along z
  ``, said once as the scenario's word); `buckling` (a plate buckles out of its plane, which it
  has no move for); a load or a pressure across the plane; a `safety` where the solid has a sharp
  re-entrant edge (`analysis::reentrant`'s text in the answer itself, not as a word as well). A
  solid read in its plane is no body of the scenario's own: its supports are the region's
  (`Read::holds`, merged with a frame's by `Read::merge` in `scenario_read`). **Held to**
  (`tests/prisms.rs`): every plane case of the corpus answers its reference — Kirsch's Kt and
  LE1's stress at D within 1% (0.62% and 0.13%), FV32's six frequencies within 0.1% (0.03%) — and
  keeps its claims; Lamé's cylinder, exact in plane stress with its ends open, answers its bore's
  move within 1e-4 and its von Mises within 0.5% in plane stress, and plane strain's closed forms
  in plane strain; a pulled sheet answers its move, its work, its factor of safety, its mass and
  an inner point's move to 1e-9; every no-answer says why. `solventc` says them as a frame's
  (`gcs-cli/tests/cli.rs`, `a_plane_reading_is_said`).

- **A region's deflection, driven, is a row whose mesh moves with the drawing** (#510 S2;
  `prisms::Row`, `interior::Moved`, `DeflectionRow::Plane`). `deflection(p, along: d) == v`, `p` a
  point of a region the scenario reads in its plane (`prisms::of_region`, tried after a frame's
  joint), compiles to `CKind::Deflection` as a frame's does, the interior's kernel at the region's
  widths. The epic's first difficulty, remeshing, met by keeping the topology: the region is
  meshed **once**, where the drawing stands when the row is compiled, and held on the sketch
  (`Sketch::meshed`, by the scenario's index, the point and the direction, shared by `Arc`; made
  before the row is validated, so the validation reads it) so that every compile of the row
  through a solve — and every residual after it — reads one mesh. Within it the nodes are
  functions of the row's columns, each region point's two numbers in the plane and each arc's or
  circle's radius (`Moved`, its constants after the structure's, `HEAD`'s `moved`): a node at a
  point of the drawing (a side's start, a point the scenario names) is that point; a node along a
  side is the side's point at the parameter it was meshed at (a line's `a + s(b − a)`, an arc's at
  the fraction `s` of its turn, a circle's at `2πs`); the nodes inside follow the boundary
  harmonically, `L_II δ_I = −L_IB δ_B` over the cotangent Laplacian of the compile's mesh (each
  element cut into its four straight triangles), which leaves a plane mesh where it was to rounding
  and is positive definite with the boundary held. The gradient is the structure's adjoint carried
  through: the inner nodes' onto the boundary's, `−L_IBᵀ L_II⁻¹ g_I` (one solve per axis on the
  Laplacian's factor), then each boundary node's onto its columns (`Dual<7>` over an arc's). It
  agrees with central differences on the library's plate and on a plate in plane strain with a
  round corner and a circular hole under a pressure on the arc, a force along a side and its
  weight, at the solution and knocked off it, to 1e-5 of the row's largest rate
  (`tests/jacobians.rs`, `a_regions_deflection_agrees_with_finite_differences`). Refused where
  written (**E040**): a region bounded by a spline or a curve (not moved yet), drawn in a plane the
  solve turns, under a direction the solve moves, or left free (`Row::check`).

- **What the report says of a driven requirement is its row's** (#510 S2;
  `report::scenario_answers`). The row's value at the solved drawing (its residual plus what it
  states) is the requirement's value, so `driven == 0.5mm, held` judges what the solve held. A
  claim of the same output reads the solved drawing meshed afresh, which agrees to the mesh's
  discretisation, not to rounding: the library's plate reads its sag so to 1e-3. A frame's row
  and its reading agree to rounding either way.

- **The library's region cases** each show a capability, and each solves to no freedom, answers
  every requirement, keeps its claims and holds what it drives, with nothing said against it
  (`tests/prisms.rs`, `every_region_case_of_the_library_says_what_it_claims`):
  `tuning_fork.sv` (modes in the plane: the fork rocking on its stem, its tines ringing a little
  below a built-in tine's), `rounded_bracket.sv` (a round making a factor of safety answerable;
  drawn sharp, it has none and says to round it), `pressure_pipe.sv` (plane strain, to Lamé's
  closed form), `gravity_wall.sv` (plane strain under its weight and a pressure, its mass the
  trapezoid's), and the driving `sized_plate.sv` (below) and `drilled_plate.sv` (a hole's radius
  driven, its circle's nodes moving with it: 19.23mm).

- **The library's driving region** is `sized_plate.sv`: a steel plate built in along its root,
  loaded along its end, its depth an unknown its sides' dimensions read, driven so its end's lower
  corner sinks 0.5mm — 76.75mm, within 0.5% of Timoshenko's beam with its shear counted for each
  of three sags, leaving no freedom (`tests/prisms.rs`, `a_requirement_sizes_a_plate`).

- **What S2 leaves.**
  - **Stress drives nothing yet.** A factor of safety is read at the nodes, a maximum: driving it
    needs its aggregation (a KS function or a p-norm, with its error reported) and its gradient,
    and the edge diagnosis already says where it has no answer. Mass, compliance and modes are
    claims too.
  - **The mesh is the compile's**, made where the drawing stands when it is elaborated: a region
    whose seeds are far from its solution (a circle seeded at radius 1 that the solve grows ninefold)
    inverts elements as its nodes follow, and the row reads NaN. No remeshing when the motion
    distorts the elements. ~~No error estimate beside an output~~: S3 estimates one beside every
    output (below).
  - **One prism, one face**: a body cut or united, or drafted, is not read in its plane; nor is a
    region bounded by a spline or a curve driven (its nodes would read the spline's basis). A
    support along a direction off the plane's axes has no answer.
  - **Nothing cached between reads**: each read meshes afresh, and each call of the row reads its
    constants, factors its Laplacian and orders `K` again (P5's two caches); the AMD analysis is
    redone per call though the pattern is the compile's.
  - **A frame and a region are not read together** in one structure.

- **The solid element is the interior's fourth** (#510 S3; `interior::Tet10`,
  `interior/solid.rs`). A quadratic tetrahedron, its nodes numbered as CalculiX numbers a C3D10
  and the volume mesh a TET10 (`delaunay::tetra::EDGES`: the corners, then the middles of the edges
  0–1, 1–2, 2–0, 0–3, 1–3, 2–3), isoparametric: an edge whose middle node is off its chord is
  curved. Its stiffness is the Gram matrix of strain rows as every element's is: at each of the
  degree-two rule's four points, `√(w·det J)·Lᵀ B`, `L` the Cholesky factor of the isotropic `D`
  (Lamé's `λ + 2μ` on the stretches' diagonal, `λ` off it, `μ` on the engineering shears') —
  twenty-four rows, its rank exactly (thirty moves less six rigid ones), so no spurious motion.
  `det J ≤ 0` at a point is `Failure::Collapsed`, `E ≤ 0` or `ν` outside (−1, ½) `Unstiff`. Its
  consistent mass `ρ ∫ NᵀN` and volume `∫ det J` take Walkington's fourteen-point degree-five rule,
  every weight positive (exact for a curved element's volume, whose `det J` is a cubic). Its
  stresses at its nodes are `D` times the strain read there (`Tet10::stresses`), its von Mises
  stress the six components' (`Tet10::von_mises`), and its geometric stiffness under the stress
  its moves make is `∫ ∇Nₐᵀ σ ∇N_b dV` along each axis alike (`Tet10::geometric`, the same rule),
  what a linear buckling reads (`Structure::stressed`). Its gradient is its mutual energy's in
  `Dual<30>` over its thirty coordinates, as the plane element's is over its twelve. A node of a
  solid structure moves along x, y and z and turns not at all (`Structure::components`); solid
  elements are mixed with no bar, beam or plane element (the constants are refused), and a solid
  structure is assembled whole and factored under minimum degree as a plane one is
  (`interior::Factor::Cholesky`). **Held to** (`tests/interior.rs`): the patch test in space to
  1e-9 — a block of knocked-about elements pulled, `σx = σ` at every node of every element and
  every other component nothing, its stretch `σw/E` and narrowings `−νσh/E`, `−νσd/E`
  (`a_block_pulled_stresses_uniformly_as_the_patch_test_asks`); exactly six motions costing
  nothing, the rigid ones; a mass of `ρV` along each axis; the volume of an element with its
  middle nodes knocked about the volume mesh's own Bernstein reading of it (P6, an independent
  formula for the same cubic); each load laying its whole; a pull stiffening a move across it by
  `σx V`; mirrored, `Collapsed` (`a_solid_element_is_rigid_in_six_motions_and_weighs_its_volume`);
  and the gradient against central differences over curved elements under every load at once,
  through the kernel so the constants read back (`a_solid_interior_agrees_with_finite_differences`).

- **A load borne by a solid moves with it** (#510 S3; `interior::Borne`): a pressure `p` on
  facets, `−p ∫ N n dA` (each facet a quadratic triangle `[a, b, c, ab, bc, ca]` turned so `(b − a) ×
  (c − a)` points out of the material; `n dA` the cross of the facet's two tangents, of degree two,
  so the degree-four rule is exact); a force spread evenly over facets (`F ∫ N dA / Σ A`) or through
  elements (`F ∫ N dV / Σ V`), the same force however the solid grows; a body load `b ∫ N dV`. One
  function per kind of share (`facet_load`, `element_load`) lays the load and, over `Dual`, its
  `λᵀ∂f/∂x`, an even load's whole also reading `−(Σ λᵀfₑ)/S² ∂S`, as the plane's do. The row's
  constants carry them after the plane loads (`HEAD` is twelve: `tets`, `borne`, and `stacked`
  below), each `[kind, count, a, b, c]` then its facets or elements; a solid element's are its ten
  nodes, its modulus, ratio and density.

- **A solid is read in three dimensions where a requirement names it and nothing else reads it**
  (#510 S3; `bodies::read`, `scenario_read`). A solid made of a material, named by a requirement —
  itself, a face path past it, a point in it, a line its wall was swept from — and read in no plane,
  is read as the solid it is, after frames and regions (`Read::merge`: a requirement a frame or a
  region answers is theirs). A solid of no material is no structure, and is read by nothing: the
  scenario's words hold it as a body of its own, as P7 does. A solid read so is no body of the
  scenario's own: its supports are its own (`Read::holds`), and its stress unbounded at an edge is
  said in its `safety`'s answer, not as a word as well (`Read::bodies` names it to
  `scenario_read`).

- **A prism is its face's mesh stacked** (#510 S3; `bodies::Mesh::stacked`). Its face is meshed in
  quadratic triangles (`prisms::Region::mesh_sized`, its boundary nodes on the drawing's curves)
  no larger than S2's `SIZE` of its extent nor `BREADTH` (⅓) of its breadth (`Region::breadth`,
  `2A/P`: a strip's width, a disc's radius), so a slender face has elements enough across it to
  bend — the corpus's cantilever, 50mm deep and a metre long, read its tip's sag 1.1% stiff at the
  extent's size alone and reads it 0.13% stiff at this. Its depth is cut at the ordinates of the
  points the scenario names in it, then into layers no deeper than `LAYER` (2) of the face's size,
  and `LAYERS` (2) at least: one prism of tetrahedra through a plate reads its bending stiff (the
  corpus's plate, its sixth mode 1.05% high in one layer, 0.28% in two). Each triangle of a layer is
  a prism of three tetrahedra, cut by the order of its corners' nodes, so two prisms sharing a
  wall cut it on one diagonal and the mesh conforms; every middle node is a node of the face — a
  corner at a middle level, or a side's middle at its corners' level or between — so a side
  curved in the face is curved through the depth, and a point the scenario names is a node (its
  place in the face a node of the face's mesh, its ordinate a level). Each boundary facet is on a
  cap (near or far, by its level) or a side (by the face's boundary segment its corners are on).

- **Any other solid is its exact boundary refined and filled** (#510 S3; `bodies::Mesh::filled`,
  P6's `delaunay::refine::volume`). A solid that is no prism — turned, united, cut — is meshed over
  its domain the body's signed distance (`BodyFaces::signed_distance`, of the drawing's own B-rep
  with its rounds made), each sharp edge of its boundary (where two faces meet at a corner,
  `BodyFaces::convex`) protected as a feature curve read in `CURVE` (64) pieces, its cells held
  to a radius–edge ratio of 2, a least dihedral of 12° and a size of `FILLED` (1/16) of its
  extent or `BREADTH` of its breadth (`3V/A`, from the B-rep's exact properties), whichever is less:
  at the extent's size alone a bar 10mm thick and its reading twice as coarse, cells of 10mm, lost
  its corners' cells and read its volume 1.2e-4 short. Each element is a TET10 whose boundary nodes
  lie on the surface and the curves (P6); each facet of its boundary is on the face its middle is
  nearest. Its corners are protecting balls' centres, so vertices: a point at a corner is a node.
  A point anywhere else names the node nearest it, within an element's size (`Mesh::near`).

- **What a line names on a solid** (#510 S3; `Filled::on`): a point its node; the solid, or the face
  a prism is swept from, the whole; a line its prism's wall was swept from, that wall's facets; a
  face path the facets on the faces it names — on a prism a cap (a plane square to the face's
  normal, out of the near side or the far) or the sides whose quarter points lie on the face's patch
  at the prism's middle, on a filled body the faces themselves. **A body names its parts' faces from
  itself**: `bar.tail.right` is the face `right` of the block `tail` united into `bar`, as a face
  path names any operand's (`FacePath::matches`), and a support or a load written on a part itself
  (`tail.right`) is about the body, so it is said, never read as nothing: `` `tail` is a part of
  `bar`: a line about it names it from the body, `bar.tail.…` ``. A support holds its nodes along
  the world's axes (`along:` one of them; along any other direction no answer yet), `fixed` holding
  what `pinned` does, a solid element's node having no turn; a force at a point is a load at its
  node, on facets `Borne::Across`, on the whole `Borne::Through`; a pressure on facets
  `Borne::Pressure`, on anything else no answer; `accelerate` `Borne::Body` of the material's
  `rho`. The material is read back into the document's units: `E`, `nu` (said where missing: a
  reading in three dimensions reads Poisson's ratio), `rho`, `yield`.

- **What a solid answers** (#510 S3; `bodies::answer`): `deflection` a node's move along `d` (or its
  length), a face's or the solid's worst; `compliance` `fᵀu`; `mass` `ρ ΣV`; `mode`
  `eigen::lowest(K, M)`; `buckling` the least factor `eigen::buckling(K, K_G)` reads, `K_G` of the
  stress the scenario's loads make; `safety` the yield over the greatest von Mises stress at the
  nodes it names, each node's the average of its elements'. Each is worked out once and only where
  asked: a motion the supports leave free is named from them before anything is factored
  (`Filled::free`, `analysis::freedoms` over every held degree of freedom — a mesh of solid
  elements is one body, so a motion no element strains is rigid), and the static solve is made
  only where an output reads it, so a modal scenario factors no stiffness it does not read.
  **No answer, and why**: a solid its supports leave free (said once as the scenario's word, ``
  scenario `pull` leaves `plate` free to slide along z ``); a material missing what is read; a
  support off the world's axes; a pressure on no face; a line naming a part of a body; a buckling
  under loads that press nothing; a `safety` where the solid has a sharp re-entrant edge
  (`analysis::reentrant`'s text in the answer). **Held to** (`tests/bodies.rs`): every solid case
  of the corpus answers its reference within 1% and keeps every claim, with nothing said against
  it (`every_solid_case_answers_its_reference`): the cantilever's sag −0.13% and its first mode
  +0.27%, Euler's column +0.11%, Lamé's bore +0.003% and its stress −0.10%, the plate's six modes
  +0.06% to +0.28%; a bar pulled answers its move, its narrowing, its work, its factor of safety,
  its mass and an inner point's move to 1e-9, its errors nothing (`a_bar_pulled_answers_to_rounding`);
  the same bar made of two blocks united, filled, to 1e-6 (its nodes on its flat faces read to the
  refinement's bisection), and a part named for the body said
  (`a_body_of_two_blocks_answers_to_rounding`); a turned rod pulled moves `σL/E` within 2e-3 — the
  nodes nearest the points holding its base across are near its axis, not on it, and hold a little
  of its narrowing (0.14%) — and weighs `ρπr²L` within the error beside it
  (`a_turned_rod_answers_its_closed_forms`); every no-answer says why
  (`what_has_no_answer_in_three_dimensions_says_why`). `solventc` says them as a frame's
  (`gcs-cli/tests/cli.rs`, `a_solid_reading_is_said`).

- **Every output has its error estimated beside it** (#510 S3; `beams::Read::errors`,
  `Read::estimated`). A scenario's readings — a frame's (`beams::read_at`), a region's
  (`prisms::read_at`), a solid's (`bodies::read_at`) — are made again twice as coarse, once and
  only where something is answered (`diagnose::analysis::scenario_read`: a frame at half its
  `PIECES`, a region or a solid meshed at twice its elements' size and its curves' pieces' turn),
  and beside each value is the change from that reading: nothing where the two agree to rounding (`ROUNDING`, 1e-12 of the
  value: a straight solid's mass, a frame's joint under loads the cubic is exact for), none where
  the coarser has no value. Wherever the discretisation converges the coarser reading is at least
  twice as wrong, and the change bounds the error of the reading it is beside — where the reference
  is exact in three dimensions the corpus holds it to that (Lamé's, the plate's Rayleigh–Lamb
  roots), or both within 1e-4 of the value (Lamé's bore, within 3e-5, where the arcs' pieces need
  not converge monotonically); on the plate it is six to nine times the error. It is no bound on
  what the element leaves out (a beam's shear, a stress at a point a load is laid on), and no
  goal-oriented estimate: a second reading, at about a quarter of the first's cost. A report says
  it beside the value, to two figures (`beams::error_text`: `` deflection(end, along: down) is
  6.39152mm ± 0.026mm: claimed >= 6.336mm, kept ``), and `--json` gives each answer's `error`
  (`report::ScenarioAnswer::error`). A driven requirement's value is its row's, its error the two
  readings' change.

- **A solid's deflection, driven, is a row stacked over its face's** (#510 S3; `bodies::row`,
  `bodies::of_body`, `interior::Stack`, `DeflectionRow::Meshed`). `deflection(p, along: d) == v`,
  `p` a point of a prism read in three dimensions (tried after a frame's joint and a region's
  point), compiles to `CKind::Deflection` as a region's does, and its row is a region's
  (`prisms::Row`): the solid meshed once where the drawing stands when the row is compiled, held on
  `Sketch::meshed` with the regions', its structure under the scenario with the deflection's
  weights (the direction's world components) on its point's node. Its face's nodes move with the
  drawing as a region's do (`prisms::moved_with`, the one reading of how), and every node of the
  solid stands over its face's node at its ordinate along the plane's normal (`Stack`: `o + p₀u +
  p₁v + h n`), so it moves as that node does; a gradient in the solid's places is folded onto the
  face's along its two axes (`Stack::fold`) and then carried as a region's is. The constants carry
  the stack after the face's moves (`HEAD`'s `stacked`: how many nodes the face has). Refused where
  a region's is (**E040**): a direction the solve moves, a plane it turns, a side it does not move,
  a solid left free. A driving deflection of a solid that is no prism compiles nothing yet: its
  nodes are no functions of the drawing, and the report says nothing drives it. **Held to**: its
  gradient against central differences on the library's plank, at the solution and knocked off
  it, and on a plate with a round corner and a hole under a pressure on the arc, a force across it
  and its weight, knocked off it, to 1e-5 of the row's largest rate (`tests/jacobians.rs`,
  `a_solids_deflection_agrees_with_finite_differences`) — taken over 1e-2, since a solid's
  stiffness is good to fewer figures than a region's: over 1e-4 its rounding is a thousandth of
  the rate, over 1e-2 its truncation under a millionth.

- **The library's driving solid** is `sized_plank.sv`: a steel plank drawn in plan, built in at its
  root and loaded at its end across its own plane — so it bends out of the face it is swept from,
  which no reading in that plane has a move for — its width an unknown its sides' dimensions read,
  driven so its end's corner sinks 0.9mm under 50N: 29.71mm, between a beam's `4PL³/(Et³δ)`
  (30mm, free to curl across its width) and a plate's `(1 − ν²)` of it (27.3mm), the plank three
  times as wide as it is thick and held at its root. It solves to no freedom, its row holds what it
  states, and the claims' reading of the solved drawing agrees with the row within its error
  (`a_requirement_sizes_a_plank`).

- **The library's solid cases** each show a capability of the reading in three dimensions, and
  each solves to no freedom, answers every requirement with its error beside it, keeps its claims
  and holds what it drives, with nothing said against it (`tests/bodies.rs`,
  `every_solid_case_of_the_library_says_what_it_claims`): the driving `sized_plank.sv` (above);
  `pressed_panel.sv` (a pressure on a cap, across the plane: a clamped square panel's middle sinks
  within 0.3% of Timoshenko's `0.00126 q a⁴ / D`); `flat_bar_strut.sv` (a solid's buckling, across
  its thickness: 2.070, within 0.7% of Euler's); `cantilever_plate.sv` (modes out of the plane: it
  bends at 46.6Hz, twists at 199Hz above Saint-Venant's, bends again at 290.6Hz); `turned_shaft.sv`
  (a solid of revolution, filled: its shoulder's round gives its factor of safety an answer, a Kt
  of 1.59, where a chamfer says to round it); and `pin_lug.sv` (a body cut by another solid, its
  pull spread over the cutter's face named from the body, `lug.pin.bore`). The panel's sag and the
  bar's factor are held to their closed forms within 1%.

- **What S3 leaves.**
  - ~~**Not in the page.**~~ The page reads every scenario in the mesh worker, and draws the one
    chosen (below, "A scenario drawn").
  - **No Hadamard derivative, and a filled body is not driven.** A prism's nodes are functions of
    the drawing; a filled body's are the refinement's, so its driving deflection compiles nothing.
    The boundary form `∫_Γ (V·n) w dΓ` over the faces' velocities is what would drive one.
  - **The error is a second reading's**, not the goal-oriented (dual-weighted residual) estimate
    the adjoint would give with the derivative: it costs a coarser mesh, and promises a bound only
    where the reading converges monotonically.
  - **Stress, modes, buckling, compliance and mass drive nothing yet**, as S2 left them.
  - **A filled body's point is the node nearest it**, so a support at a point off a vertex holds a
    little of what it should not (the rod's 0.14%). A degenerate pair of a solid's modes (the
    plate's two) is found and not yet named with its ring (P7's word).
  - **Speed.** In layers as deep as its elements are wide Lamé's tube is 53,754 unknowns, factored
    in 9.4 s; at `LAYER` 2 it is read in 3.3 s, the plate in 5.1 s (its modes), the column in 2.2 s
    (its buckling), the cantilever in 1.2 s, each read once at its own size. The
    Cholesky's update is formed four columns at a time (`cholesky::BLOCK`) and its dense panels a
    column at a time, and each read meshes afresh, twice (P5's two caches are still not kept).

- **A scenario drawn: where its lines fall** (#510; `overview::scenario::marks`,
  `report::scenario_marks_json`, FFI `gcs_elab_scenario_marks_json`, `Document.scenarioMarks`).
  Every line of a scenario — support, load, requirement, reading in the plane — is a **mark**, in
  the order written, read off the solved drawing with no analysis made: its anchors; at each the
  direction it acts along (a force's; a pressure's into its face, against the face's outward
  normal; the one direction a support holds where it says `along:`), and on a face that normal;
  the faces it tints, as triangles; its size among the scenario's loads of its kind; its words in
  the core's (`600N along down` is said `600N`, a pressure `100kPa`, `pinned along z`:
  `scenario::quantity`, SI with a prefix); where its words go; whether it is a requirement read at
  a place (`probe`, a deflection — one of a whole body has no place); and the span of the line it
  is written on. A point is drawn at itself, a line, an edge or a beam's lines at `ALONG` places
  evenly along them, a solid at its middle, faces a path names at up to `OVER` places spread over
  them (farthest-point, over each triangle cut finer than a sixth of the faces' reach). **A face
  is read off the mesh the workspace already draws** (`EvaluatedSolid::mesh` at the object's own
  scale, its groups named `lug.root`, `pin.bore`: `names_group` takes the path with or without the
  solid's own name first), so a mark on a face builds no exact boundary on the page's thread.
  **A mark adds nothing to the drawing** (`a_mark_adds_nothing_to_the_drawing`); every line of
  every scenario of the library is one, its span the line it is written on
  (`every_line_of_a_scenario_is_a_mark`), and a pressure points into its face
  (`a_pressure_points_into_its_face`). Gate: `tests/scenario_view.rs`.

- **A scenario drawn: how the drawing moves under it** (#510; `deformed::Picture`,
  `diagnose::analysis::scenario_pictured`, each reader's `read_pictured`). **Every body any line
  of the scenario falls on is read**, a requirement's or not — a frame a support or a load names a
  joint or a member of (`beams::read_pictured`), each region it reads in its plane
  (`prisms::read_pictured`), each solid read in three dimensions a line falls on
  (`bodies::read_pictured`, `solid_of`) — so a scenario of supports and loads alone is drawn.
  **What is drawn deformed is the mesh that was solved**, never the display's: a frame a tube of
  `SIDES` about each element at `PIECES`, sampled at `SAMPLES` places on the cubic it bends as (its
  ends' moves in a line along it; across it, in each plane, Hermite's cubic in its ends' moves
  and turns — a turn about e3 the slope along e2, about e2 the slope along e3's negative), every
  vertex of a ring moving as its centre; a region its quadratic triangles cut in four on both caps
  at its middle ± half its depth, walls along its sides, the in-plane move the same at each level;
  a solid its tetrahedra's boundary facets cut in four. Each body is its vertices at rest and
  their moves, triangles and the mesh's edges. **It is the answers' reading**: each body is
  analysed once for its answers and its picture (`read_pictured` shares the readers' `Analysed`),
  at the readers' own size, so a deflection of a point is the move of the vertices drawn nearest
  it, to rounding (`the_picture_moves_each_point_by_its_answer`, over the library); the coarser
  reading the error is estimated against draws nothing. A cantilever's picture is its cubic,
  inside its elements as at its nodes (`a_cantilevers_picture_is_its_cubic`). A body that cannot be
  read is not drawn, and why is said in the core's words (`Picture::words`,
  `a_frame_left_free_draws_no_deflection_and_says_why`); a scenario that loads nothing moves
  nothing. **The factor it is drawn at** (`deformed::factor`): the largest move as `SHOWN` (a
  tenth) of the bodies' extent, rounded down to 1, 2 or 5 of a decade, never below true scale;
  a host draws `rest + s·moved` for any `s`.

- **A scenario drawn: over the ABI and in the page** (#510; `report::scenarios_pictured_json`,
  FFI `gcs_elab_scenarios_pictured_json`, `Document.scenariosPictured`, `app/scenario-view.ts`).
  `{ "scenarios", "picture" }`: `scenarios_json`'s answers word for word
  (`the_answers_beside_a_picture_are_the_answers`), each answer now with its requirement's
  `index`, and the picture of the scenario asked for — flat arrays, places and moves to seven
  figures — or `null`. **The page reads it in the mesh worker** (a `scenario` job, its own
  instance), never on its own thread: a solid read in three dimensions is seconds. The job is
  keyed by the text, the pose and the scenario drawn, as the claims' is; the answers already said
  stand until the new ones come, and the picture goes at once (it is of a pose the edit may have
  left). With no worker (node) the reading is made at once, so the tests see what the page would
  (`app.test.ts`, `choosing a scenario writes nothing, and draws its marks and how the drawing
  moves`). The marks are read on the page with every edit (`gcs_elab_scenario_marks_json`), the
  names with no analysis (`gcs_elab_scenario_names_json`); `core.test.ts`, `a document says where
  its scenarios fall and how the drawing moves under one`. The page's chooser and card are
  [app.md](app.md)'s.

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
  its `expr::Cmp`: `Claim`, `Driving`, `Bound`) is recorded — and, since S1, a frame's driving
  deflection compiled into a row (above), the one exception to adding nothing — but the rule S1–S3
  hold is kept now: one scenario's requirements drive dimensions, a driving row in a second
  **E106** until S4's optimiser. An edit removing what a scenario names never takes the scenario
  with it (`edit::remove::mentions`): the name left dangling is said (**E101**).

- **A material is a model value** (#510 P3, §6.9 [0.165]): `model::Material`, a name and the
  properties stated of it (`model::Property`: `E`, `nu`, `rho`, `yield`).  `Sketch::materials`
  holds the document's materials by absolute name (`steel`, `p.al` inside an instance), and
  `SolidE::material` names the one a solid is made of, `model::Beam::material` a beam's (S0).
  A frame's analysis reads a beam's (S0, below); nothing reads a solid's yet.  The solve, the
  diagnosis and every evaluated solid are unchanged by a material.  `tests/materials.rs` is
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

- **One sparse factor for a symmetric matrix given by its entries** (#510 P2, S3; `sparse::Ldl`).
  Since S3 it is P1's supernodal factor as `LDLᵀ` (`cholesky::Cholesky::factor_ldl`) under
  approximate minimum degree, the analysis `factor` reads, so a mesh's pencils fill as its
  stiffness does; the up-looking LDLᵀ under RCM (`sparse::Factor`) is the normal equations' alone
  (`sparse::Ata`). Without pivoting: an SPD matrix factors stably, an indefinite one when no pivot
  is zero (`None` when one is), and **its negative pivots are its negative eigenvalues**
  (Sylvester), so `A − μB`'s count the pencil's eigenvalues below μ when B is positive definite.
  **A pencil is analysed once** (`sparse::Pencil`): `L − μR` on the union of the two patterns, its
  lower triangles and its analysis made at construction, so each shift and each count is a numeric
  factor and no more. `sparse_ldl_solves_and_counts_negative_eigenvalues` holds the solve and the
  count against the reference on membranes shifted into their spectrum. On the corpus's plate (24,840
  unknowns) a factor of its pencil went from 2.27 s to 0.98 s, and on Lamé's tube (53,754) from
  22.5 s to 9.5 s, the Cholesky's own time.

- **One iteration for every pencil `L x = λ R x`** (#510 P2; `eigen.rs`'s `solve`): shift-invert
  Lanczos on `T = (L − σR)⁻¹R`, self-adjoint in `xᵀBy` with B the side that is positive definite,
  its largest θ = 1/(λ − σ) the λ just above σ. Full reorthogonalization, thick restart (the
  wanted Ritz vectors kept), one factor for the whole solve. The tolerances are named once
  (`BREAKDOWN`, `FLOOR`, `GAP`, `SHIFT`, and S3's `UNBUCKLED`), never written at a use.

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
  the reference; `buckling_refuses_what_has_no_load`. **A tension buckles nothing, counted** (#510
  S3): a solid pulled has a geometric stiffness whose null space is thousands wide, every θ of the
  iteration about zero, where it would converge on none; so Sylvester's count is asked before it
  iterates — no negative pivot of `K + μK_G` at `μ = UNBUCKLED` (10⁶, a factor past any load a
  scenario reads) is no factor below it, and none is returned, one factor more for every buckling
  (`what_has_no_answer_in_three_dimensions_says_why`, a bar pulled).

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

- **What an analysis cannot answer is said in the solver's words** (#510 P7;
  `diagnose/analysis.rs`, in `gcs-solve`, knowing no Solvent). Four words, each read off what is
  already in hand — supports, a body's boundary, the pairs a solve returned, a row's value and
  gradient — and none off a mesh: the motions supports leave free, unbounded stress at an edge, a
  degenerate modal pair, and a requirement out of reach. A word is a report, never a row: it
  changes no count and no answer. One word ends work early: a requirement out of reach where a
  solve stopped is not retried and not searched for a conflict (below), since neither could meet
  it.

- **A rigid-body motion is named from the supports, never from a factor** (`analysis::freedoms`).
  P1's Cholesky refuses only a pivot lost to rounding and P5's interior a least pivot under
  `SINGULAR`, and no pivot ratio tells a free body from a slender held one, so the motion is read
  off the supports. Each `Held` support holds its points along one direction or every one, and a
  `fixed` support (`turn`) the body's turn too. The free twists `(v, ω)` are the null space of
  `e·(v + ω×(p − c)) = 0` over every held point and direction, `c` the supports' middle and ω's
  columns scaled by the supports' size so every column is a velocity (`RIGID`, the diagnosis's
  rank cut `system::RANK_TOL`). The slides are the free twists that do not turn. Each turn is about its axis through the
  point of it nearest `c`, its velocity there along the axis the pitch: zero for a turn, else a
  screw. World axes are preferred where the freedoms hold them (`SNAP`), so a body free about z
  says `turn about z through (0, 0, 0)`. A twist that moves no point of the body (`body`: bars all
  on one line, turning about it) moves nothing and is no freedom. Every freedom moves no held point
  along a held direction, and `6 − rank` remain under generic holds, screws among them
  (`the_freedoms_are_what_the_supports_leave`,
  `a_rigid_body_motion_is_named_from_the_supports`).

- **A structure's free motion is named** (`Structure::free`, P5's `Failure::Free` said). The
  rigid motions are the supports': each held degree of freedom holds its node along its axis, and
  the nodes are the body. A mechanism is what is left of the motions no bar stretches (each bar's
  strain row, `ĝ·(u_b − u_a) = 0`) once the rigid motions' fields are taken out. The rows are the
  ones `solve` factors, unweighted: one numbering (`Structure::equations`) and one strain-row
  builder (`Structure::strain`) serve both, so an element that joins the interior joins the
  mechanism too. Its nodes are
  those it moves. It is looked for by a dense null space up to `diagnose::NUMERIC_MAX` free
  degrees of freedom, and is `None` past it. `Free::text` says both: `free to turn about (-3, 0);
  a mechanism: nodes 1 and 2 move with no bar stretched`
  (`a_structure_names_what_leaves_it_free`).

- **Stress at a sharp re-entrant edge is unbounded, and is said rather than computed**
  (`analysis::reentrant`). A body's edges are read off the boundary the drawing finishes, its
  rounds made (`BodyFaces::whole`, where `BodyFaces::read` is the stratum a round reads). A body
  nothing reads the faces of is that stratum, already remembered (`faces::cached`). Any other is
  remembered in `Caches::whole`, over the drawing's own B-rep (`Sketch::exact_solid`), so every
  scenario, report and export over one body reads one build. A
  concave corner where the material's angle (`BodyFaces::angle`) passes a half turn (`STRAIGHT`)
  is one, once per pair of faces at its sharpest. Stress grows toward it as `r^(λ−1)`, λ
  Williams' least exponent for a traction-free wedge in its opening mode (`analysis::williams`:
  the least positive root of `sin(λα) + λ sin α = 0`; ½ at a crack, 0.5445 at three quarters of
  a turn, 1 at a half turn or less). So a stress requirement there has no answer a finer mesh
  converges to. A rounded edge's faces meet tangent, and it is no corner
  (`williams_exponent_is_the_wedges_least_root`).

- **A degenerate modal pair is named, and the ring that makes it one** (`analysis::repeated`,
  `Repeated::wave`). The eigensolver says which of its values are one (`Pairs::runs`: consecutive
  values within its `GAP` of the first of each run), and a run of more than one is repeated. A run at rest (`AT_REST` of the
  largest) is a rigid motion's, the supports' word, and is left out. An eigenvalue of two vectors
  has no derivative there (`φᵀ(L' − λR')φ` is no longer one number), so a requirement reads the
  run whole. Given one step of a ring of `order` as a map of the freedoms and the inner product
  the pair is orthonormal in, the step acts on the pair as `A = ΦᵀBTΦ`. A ring's pair turns by
  `2πk/order` under it, and `k` is its wave. A step that keeps each mode (or a reflection) gives
  none: degenerate, but not the ring's (`a_degenerate_pair_is_named_with_its_ring`, the free ring
  of 120: pairs 1–4 at waves 1–4, the cut tenth value no pair).

- **A requirement out of reach of what drives it** (`analysis::reach`; `Structure::reach` for the
  interior's row). A row that holds to what the solver accepts (`SolveOpts::acceptance_tol`, at the
  size of the larger of its value and what it states) is `Reached`. One that does not
  is `Held` where no column it reads is free. It is `Extreme` where its change over the free
  columns, each moved by its own scale, is below `STATIONARY` (1e-3) of its gap to what it
  states: closing the gap along its rate would move them across a thousand times their scale. A
  solve that stopped there stopped at the extreme of what the row can be, short of what it states:
  the least it reaches where it states less, else the most. Otherwise it `Moves`, and there is no
  verdict. Against the gap and not the value: a solve reaches an extreme only to within its step
  (the drilled plate's hole driven toward nothing stops 0.08mm short, its row's rate a millionth of
  its value), while a row a hair from what it states may move slowly and still reach it.
  The interior moves a coordinate by the structure's extent and an area by itself. Driving two
  bars' rise to half their least sink, a damped Newton stops at `h = √2 w`, and the verdict is
  the closed form's `3√3 P w / (4EA)` (`a_requirement_out_of_reach_says_so`).

- **A requirement out of reach is the verdict on a failed solve** (`System::out_of_reach`,
  `System::reach`). A driven deflection (`CKind::Deflection`) that does not hold where a solve
  stopped is read by `interior::reach_of` — its output, what it states and the rate the solve
  steps on — over the parameters it reads that the solve moves, each across the drawing's extent
  in its own units (`Param::scale`). Where at least one fails and every one that fails is `Held`
  or `Extreme`, the failure is the requirement's: any other row that does not hold is what the
  pull toward it made (a hole's radius driven toward nothing reads its circle's row over a size
  that went with it). The solve then runs none of its `RETRIES` (each starts again from the same
  seeds toward the same extreme, every step an analysis factored), and the diagnosis — where the
  drawing stands, or where its scratch solve stopped when that is no stationary point, as on a
  page that puts a failed solve's pose back — files the rows as the conflict
  (`Diagnosis::out_of_reach`, each with `Reach::text`; report `outOfReach`, `[id, words]`) and
  searches for no other. The summary says `CONFLICT — a requirement out of reach of what drives
  it: change what it states, or what drives it`, `solventc` prints each row with its words in
  place of a `conflict:` line, and the page's banner says them. The drilled plate loaded ten
  times over is the gate (`a_requirement_out_of_reach_is_the_verdict`): its least, with the hole
  driven toward nothing, within 0.5% of the plain plate's sink, and the same verdict at both
  poses; on the page it is answered in a few seconds where the cascade and the search took half a
  minute. A hole driven *larger* than its mesh, made where it was compiled, can follow is not
  out of reach — its row cannot be read past where the mesh folds, and it is still searched.

- **A scenario's words, at the solved drawing** (`analysis::scenario_words`). Its supports'
  points come from what each names: a point; a line's ends; a solid's box (a solid spans space);
  or every face a path names, by its edges' ends and quarters. A support that holds a frame of
  beams is the frame's (S0, below); the rest are held as one body, named by the solids the
  scenario names:
  `scenario `lift` leaves `plate` free to turn about x through (0, 0, 0), …`. Every `safety`
  requirement on a solid with a re-entrant edge is said to have no answer. A support that cannot
  be read is said (`` `plate.nowhere` names no face of `plate` ``), never taken as holding
  nothing (`a_scenario_says_what_leaves_it_free_and_where_stress_is_unbounded`,
  `a_support_on_no_face_is_said`). `solventc` prints each word under the document's result once
  it is solved, and `--json` gives them as `scenarios` (`report::scenarios_json`,
  `gcs-cli/tests/cli.rs`'s `a_scenarios_words_are_said`). They are words, not errors: the
  document still exits 0.

- **What P7 leaves to later stages.** No word yet reads an analysis's own results: the degenerate
  pair is said of pairs a caller hands it, the out-of-reach verdict of rows a caller hands it and
  of a failed solve's driven deflections (above), and a frame's
  modes are S0's, but no repeated pair of theirs is named yet, and no scenario compiles a
  driving row until S1. The ring a pair is the ring's is
  given as a map of the freedoms; reading it off a drawing's `ring` is the modal stage's. The
  free motion of a scenario's solids holds their supports as one body; a frame's is its own
  (S0). A pinned face is read as its points held, which holds a turn as a clamp does.

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

- **What P6 leaves to S3.** ~~No element stiffness reads a `Volume` yet~~: S3's solid that is no
  prism is filled so and read (`bodies::Mesh::filled`, below). A protecting ball's
  neighbours keep cells no refinement reaches. Mesh_3 removes those slivers by perturbation and
  exudation, which need a vertex moved or reweighted, and this triangulation removes none. The
  cells have one size, with no grading toward a feature or a load. The volume is built in the
  terminal and the tests only, not in the page's mesh worker. A `MaterialField` is a `Domain`
  like any other, but no gate fills one yet.

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

- **How a case is read is its document's** (#510 P8, decided in S2). `Reading::Plane` reads a
  case in plane stress over the face its solid sweeps, its depth the thickness; the oracle reads
  it so only for a solid swept across y. A hold across the thickness (`pinned m along: std.y`) is
  the solid reading's, taking out what the symmetry cuts leave free, and holds nothing in the
  plane. Since S2 a document says how it is read (`plane stress plate`, above), and every case
  the table reads in the plane states it in each scenario, no other case stating it
  (`a_plane_case_states_its_reading`). Read as a solid, FV32 bends out of its plane far below the
  in-plane modes it claims.

- **What P8 leaves.**
  - **The frame cases** are S0's (above): the cantilever, Euler's column and a beam built in at
    both ends, as frames (`cantilever_frame.sv`, `column_frame.sv`, `clamped_frame.sv`), read
    `Reading::Frame` off the beam's line, section and material. CalculiX reads no frame: a
    frame's answers are the core's own, held to the closed forms (`tests/beams.rs`). NAFEMS FV2
    waits.
  - **NAFEMS LE10's target** is a component of stress at a point, which no output word reads
    (an open decision of the requirement words).
  - **NAFEMS FV52's supports** are published two ways, the lower edges or the middle plane's.
    The case waits for the publication in hand, and `plate_modes.sv` is the same physics with
    supports under which the answer is exact.
  - **The oracle lays no body load**, holds only along the world's axes, reads no compliance and
    meshes one primitive's faces by name. Each is refused by name, where a later case needs it.
