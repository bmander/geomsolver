# Analysis

Part of the [contract](../../CLAUDE.md) the code is held to. Read it when you touch
`interior.rs`, `KernelKey::Interior`, `sparse::Ata::least_pivot`, `eigen.rs`, `sparse::Sym`,
`sparse::Ldl` or `sparse::Pencil`. The epic these rules serve is #510 (design intent as
structural performance), whose prerequisites and stages (P1–P8, S0–S4) are named there; this
file holds the rules landed so far. Each section names its gate: a rule that names none is held
there.

## An output as a row (P5)

The gate is `rust/gcs-core/tests/interior.rs`.

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
  Materials, scenarios and the requirement words are P3, P4 and S0, and S0/S1 give the row its
  constraint type, with its callout or an `undrawn!` entry. Every call reads the structure from
  its constants and orders `K`'s pattern again (`Ata::new`'s symbolic analysis), though neither
  moves with `x`. The system also calls a kernel's residual and Jacobian separately, so a step
  that asks for both factors `K` twice. Holding the symbolic analysis per row (a hit on every
  step) and the factor between the two calls (a hit only at one `x`) are two caches, to be
  measured first and apart, as `solve/through.rs` measured its own. The element interface
  (strain rows and their derivatives, degrees of freedom apart from coordinates, several section
  columns) is S0's, shaped against the beam that needs it.

- **Within the solver.** `interior.rs` is in `gcs-solve` and knows no Solvent.

## The eigensolver (P2)

The gate is `rust/gcs-core/tests/linalg.rs`.

- **A symmetric matrix is given by its entries** (`sparse::Sym`). `from_triplets` takes every entry
  of the matrix, both triangles, and sums the ones named twice, as an assembly adds element matrices
  into a global one; nothing is mirrored, so an entry given on one side only is not symmetric and is
  not a matrix the factor answers for.
- **One sparse factor** (`sparse::Ldl`). The up-looking LDLᵀ the normal equations use
  (`sparse::Ata`, its order RCM with any dense column last) is one `Factor` over a `Sym`, JᵀJ among
  them; nothing has a copy of it. Without pivoting: an SPD matrix factors stably, an indefinite one
  when no pivot is zero (`None` when one is), and **its negative pivots are its negative
  eigenvalues** (Sylvester), so `A − μB`'s count the pencil's eigenvalues below μ when B is positive
  definite. **A pencil is analysed once** (`sparse::Pencil`): `L − μR` on the union of the two
  patterns, ordered and its elimination tree built at construction, so each shift and each count is
  a numeric factor and no more. `sparse_ldl_solves_and_counts_negative_eigenvalues` holds the solve
  and the count against the reference on membranes shifted into their spectrum.
- **One iteration for every pencil `L x = λ R x`** (`eigen.rs`'s `solve`): shift-invert Lanczos on
  `T = (L − σR)⁻¹R`, self-adjoint in `xᵀBy` with B the side that is positive definite, its largest θ
  = 1/(λ − σ) the λ just above σ. Full reorthogonalization, thick restart (the wanted Ritz vectors
  kept), one factor for the whole solve. The tolerances are named once (`BREAKDOWN`, `FLOOR`, `GAP`,
  `SHIFT`), never written at a use.
- **The lowest pairs of `A x = λ B x`** (`eigen::lowest`; A symmetric, B positive definite, else
  `Refusal::NotDefinite`): `L = A`, `R = B`, the inner product R's. **The shift is found, not
  given**: `SHIFT` of the least diagonal ratio `A_ii/B_ii` below zero (a Rayleigh quotient, so no
  eigenvalue is above it), moved down until `A − σB` factors with no negative pivot, so nothing is
  below it and the nearest are the lowest; a stiffness free in a rigid-body motion has its mode at
  zero, found like any other. `lowest_modes_agree_with_the_dense_reference`,
  `lowest_modes_of_a_large_membrane_hold_their_properties`.
- **Linear buckling is the same iteration** (`eigen::buckling`). `(K + λ K_G) φ = 0` is `L = B = K`,
  `R = −K_G`, σ = 0, so `T = K⁻¹(−K_G)` and θ = 1/λ: K must be positive definite (a supported
  structure; a rigid-body motion is `NotDefinite`), no shift is needed, the largest θ are the lowest
  positive load factors, and a load that buckles nothing (a tension) has none. `L − μR = K + μK_G`'s
  negative pivots count the positive factors below μ. `buckling_finds_eulers_loads` holds a pinned
  and a cantilevered column to Euler's loads and to the reference;
  `buckling_refuses_what_has_no_load`.
- **No pair is missed** (rings and patterns give repeated pairs). A Krylov space holds one direction
  of each eigenspace, so after the iteration converges Sylvester's count is taken `GAP` above the
  highest value found; where the pencil has more below it than were found, the iteration runs again
  B-orthogonal to every pair found and finds the copies. The count answers, not the iteration.
  `a_free_ring_has_its_rigid_mode_and_every_pair`: the rigid turn at zero, and every mode twice,
  `4(k/m) sin²(πj/N)`, the tenth value cutting a pair.
- **What a solve returns is held to its properties**, not just to the reference: values ascending,
  vectors B-orthonormal (a mode `φᵀMφ = 1`, a buckling shape `φᵀKφ = 1`) and A-orthogonal with λ on
  the diagonal, each residual the pair's backward error `‖Lφ − λRφ‖ / ((‖L‖₁ + |λ|‖R‖₁)‖φ‖)`
  computed from the matrices (meaningful at zero, for a rigid mode), and Sylvester's count just
  below the top value equal to the number found below it (`assert_pencil_pairs`).
- **Deterministic.** The start and any fresh direction are draws of the seeded `rng::Rng`
  (`Opts::seed`), so a pencil has one answer. A pair converges when its Ritz residual is below `tol`
  (1e-10) of its θ, or of `FLOOR` of the largest for a θ near zero; out of restarts, or a count the
  deflated iteration cannot meet, is `Refusal::NoConvergence`, never an answer.
- **Dense pairs** (`linalg::sym_eigen`) are `sym_eigenvalues`'s Jacobi rotations, accumulated: the
  eigenvalues unchanged to the bit, and the vectors the iteration's Ritz vectors are made from.
