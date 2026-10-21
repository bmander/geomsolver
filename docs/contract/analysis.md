# Structural analysis

Part of the [contract](../../CLAUDE.md) the code is held to. Read it when you touch `eigen.rs`,
`sparse::Sym`, `sparse::Ldl` or `sparse::Pencil`. It is the contract of #510 (design intent as structural
performance): what lands for the epic's prerequisites and stages is ruled here. An issue number
(#n) says where a rule came from; a tag (P2, S0, …) is the item of #510 that landed it. The gate
is `rust/gcs-core/tests/linalg.rs` unless a rule names another.

## The eigensolver (P2)

- **A symmetric matrix is given by its entries** (`sparse::Sym`). `from_triplets` takes every
  entry of the matrix, both triangles, and sums the ones named twice, as an assembly adds element
  matrices into a global one; nothing is mirrored, so an entry given on one side only is not
  symmetric and is not a matrix the factor answers for.
- **One sparse factor** (`sparse::Ldl`). The up-looking LDLᵀ the normal equations use
  (`sparse::Ata`, its order RCM with any dense column last) is one `Factor` over a `Sym`, JᵀJ
  among them; nothing has a copy of it. Without pivoting: an SPD matrix
  factors stably, an indefinite one when no pivot is zero (`None` when one is), and **its
  negative pivots are its negative eigenvalues** (Sylvester), so `A − μB`'s count the pencil's
  eigenvalues below μ when B is positive definite. **A pencil is analysed once**
  (`sparse::Pencil`): `L − μR` on the union of the two patterns, ordered and its elimination tree
  built at construction, so each shift and each count is a numeric factor and no more.
  `sparse_ldl_solves_and_counts_negative_eigenvalues` holds the solve and the count against the
  reference on membranes shifted into their spectrum.
- **One iteration for every pencil `L x = λ R x`** (`eigen.rs`'s `solve`): shift-invert Lanczos
  on `T = (L − σR)⁻¹R`, self-adjoint in `xᵀBy` with B the side that is positive definite, its
  largest θ = 1/(λ − σ) the λ just above σ. Full reorthogonalization, thick restart (the wanted
  Ritz vectors kept), one factor for the whole solve. The tolerances are named once (`BREAKDOWN`,
  `FLOOR`, `GAP`, `SHIFT`), never written at a use.
- **The lowest pairs of `A x = λ B x`** (`eigen::lowest`; A symmetric, B positive definite, else
  `Refusal::NotDefinite`): `L = A`, `R = B`, the inner product R's. **The shift is found, not
  given**: `SHIFT` of the least diagonal ratio `A_ii/B_ii` below zero (a Rayleigh quotient, so no
  eigenvalue is above it), moved down until `A − σB` factors with no negative pivot, so nothing is below it and the
  nearest are the lowest; a stiffness free in a rigid-body motion has its mode at zero, found like
  any other. `lowest_modes_agree_with_the_dense_reference`,
  `lowest_modes_of_a_large_membrane_hold_their_properties`.
- **Linear buckling is the same iteration** (`eigen::buckling`). `(K + λ K_G) φ = 0` is
  `L = B = K`, `R = −K_G`, σ = 0, so `T = K⁻¹(−K_G)` and θ = 1/λ: K must be positive definite (a
  supported structure; a rigid-body motion is `NotDefinite`), no shift is needed, the largest θ
  are the lowest positive load factors, and a load that buckles nothing (a tension) has none.
  `L − μR = K + μK_G`'s negative pivots count the positive factors below μ. `buckling_finds_eulers_loads`
  holds a pinned and a cantilevered column to Euler's loads and to the reference;
  `buckling_refuses_what_has_no_load`.
- **No pair is missed** (rings and patterns give repeated pairs). A Krylov space holds one
  direction of each eigenspace, so after the iteration converges Sylvester's count is taken
  `GAP` above the highest value found; where the pencil has more below it than were found, the
  iteration runs again B-orthogonal to every pair found and finds the copies. The count answers,
  not the iteration. `a_free_ring_has_its_rigid_mode_and_every_pair`: the rigid turn at zero, and
  every mode twice, `4(k/m) sin²(πj/N)`, the tenth value cutting a pair.
- **What a solve returns is held to its properties**, not just to the reference: values
  ascending, vectors B-orthonormal (a mode `φᵀMφ = 1`, a buckling shape `φᵀKφ = 1`) and
  A-orthogonal with λ on the diagonal, each residual the pair's backward error
  `‖Lφ − λRφ‖ / ((‖L‖₁ + |λ|‖R‖₁)‖φ‖)` computed from the matrices (meaningful at zero, for a
  rigid mode), and Sylvester's count just below the top value equal to the number found below
  it (`assert_pencil_pairs`).
- **Deterministic.** The start and any fresh direction are draws of the seeded `rng::Rng`
  (`Opts::seed`), so a pencil has one answer. A pair converges when its Ritz residual is below
  `tol` (1e-10) of its θ, or of `FLOOR` of the largest for a θ near zero; out of restarts, or
  a count the deflated iteration cannot meet, is `Refusal::NoConvergence`, never an answer.
- **Dense pairs** (`linalg::sym_eigen`) are `sym_eigenvalues`'s Jacobi rotations, accumulated:
  the eigenvalues unchanged to the bit, and the vectors the iteration's Ritz vectors are made
  from.
