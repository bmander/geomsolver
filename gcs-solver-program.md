# Building a Geometric Constraint Solver: A Staged Program
## Python front-end, C core — from Newton's method to the research frontier

The strategy mirrors how the field itself evolved: start with the pure-numerical approach that SolveSpace and PlaneGCS use (which gets you a *working* sketcher fast), then layer on the graph-theoretic machinery (diagnosis, decomposition, witness analysis) that separates D-Cubed DCM from everything in open source. Each stage produces a usable solver; each subsequent stage is the answer to a concrete failure you'll have personally experienced in the previous one. That failure-driven ordering matters — you'll understand *why* decomposition exists after you've watched DogLeg grind on a 400-entity sketch, in a way you can't from reading Owen's paper cold.

A note on "C extension": the architecture below is binding-agnostic. Straight CPython C API is maximal control and maximal boilerplate; **Cython** is the pragmatic middle for this project (you can move code across the Python/C boundary incrementally, which fits the staged plan perfectly); **pybind11** if you'd rather the core be C++ (which gives you Eigen, a real asset here). Given the routelab pattern, everything here also ports 1:1 to Rust/PyO3 with nalgebra — the stages don't change, only the FFI. I'd honestly suggest deciding at Stage 1, not now.

---

## Stage 0 — Pure-Python prototype: the residual formulation

**Goal:** a working solver for real sketches in ~500 lines, zero C, so all subsequent design decisions are informed by a running system.

**The core model, which persists through every stage:**

- A sketch is a flat parameter vector `x ∈ R^n` (point coords, line params, circle center+radius…).
- Each constraint contributes one or more **residual functions** `r_i(x)` that are zero when satisfied: coincidence of two points is 2 residuals; distance is `‖p−q‖² − d²` (use the squared form — no sqrt, smooth everywhere); tangency line↔circle is signed distance minus radius; parallel is a 2×2 determinant; angle via dot product.
- Solving = finding `x` minimizing `‖r(x)‖²`, warm-started from the current sketch positions.

**Implementation:** primitives and constraints as small Python classes; each constraint knows its residuals and its parameter indices. Solve with `scipy.optimize.least_squares` (`method='lm'` and `'dogbox'` — try both). Start with scipy's numerical Jacobian; then hand-derive analytic Jacobians per constraint type and verify against finite differences (write that verification harness now — it stays useful forever).

**Also build now:** a dead-simple interactive canvas (matplotlib event handling is enough, or a tiny web canvas) with click-drag of points. Dragging = add a temporary "point at cursor" soft constraint, re-solve every mouse-move from the warm start. This is your permanent testbed; every later stage is judged by how dragging *feels*.

**Exit criteria:** solves a rectangle-with-fillets sketch, a slotted link, and a ~30-entity sketch; dragging feels alive; you have a `Sketch → residuals/Jacobian → solve → writeback` pipeline with clean seams.

**Read:** SolveSpace's technology page (solvespace.com/tech.pl) — it describes almost exactly this system, minus the symbolic layer; Light & Gossard (1982) for the original variational formulation.

---

## Stage 1 — The C core: sparse Newton done properly

**Goal:** move the hot path into C and replace scipy with your own solver, because everything after this stage requires owning the numerics.

**What moves to C:** the residual/Jacobian evaluation loop and the solve iteration. The Python layer keeps the object model and compiles a sketch down to a flat **evaluation plan**: arrays of `(constraint_type, param_indices, constants)` records the C core iterates over. This compile-to-plan boundary is the single most important architectural decision — it's what lets Python stay expressive while C stays branch-predictable, and it's the same pattern as routelab's Python-orchestration/Rust-kernel split.

**The numerics to implement:**

1. **Sparse Jacobian assembly** in triplet→CSR form. Sketch Jacobians are extremely sparse (each constraint touches 2–8 parameters).
2. **Levenberg–Marquardt** with your own damping schedule, and **Powell's DogLeg** as the default (matching PlaneGCS's choice — it's more robust for this problem class).
3. **Under-constrained handling:** this is the normal case during editing, not the exception. Solve the Gauss–Newton step in minimum-norm least-squares sense (sparse QR — SuiteSparseQR if C, Eigen's SPQR if C++ — or LSQR iteratively). Minimum-norm update = least-change behavior = the geometry moves as little as possible, which *is* the UX users expect.
4. **Rank-revealing QR** on the Jacobian at the solution — you need the rank machinery now anyway, and it becomes the workhorse of Stages 2 and 4.
5. Release the GIL during solves. Batch/parallel solving pays off in Stage 6.

**Exit criteria:** beats scipy by >10× on the 30-entity sketch; 60fps dragging on a 200-entity sketch; a `slvs`-style flat C API (`gcs_compile`, `gcs_solve`, `gcs_drag`) so the core is usable from anywhere, not just Python.

**Read:** Nocedal & Wright ch. 10 (least-squares methods); the PlaneGCS source (`src/Mod/Sketcher/App/planegcs/` in FreeCAD) — note what it does *not* do, structurally, since that's your roadmap.

---

## Stage 2 — Diagnosis: structural constraint analysis

**Goal:** stop reporting "solver failed to converge" and start reporting "these 3 constraints conflict; this arc has 2 remaining DOF." This is where you pass SolveSpace/PlaneGCS in user experience.

**The algorithms (all graph-side — implement in Python first, port hot ones later):**

1. **DOF bookkeeping:** per-primitive DOF, per-constraint valency, running sums per connected component. Trivial and immediately useful.
2. **Maximum bipartite matching** (Hopcroft–Karp) on the equations↔parameters graph, then **Dulmage–Mendelsohn decomposition** to canonically split the system into over-, well-, and under-constrained parts. The over-determined part gives you *which* equations are structurally redundant; the under-determined part tells you *which* parameters are free (drive "show remaining DOF" UI).
3. **The (2,3) pebble game** (Jacobs–Hendrickson; Lee–Streinu generalization) on the point-distance subgraph: detects rigid clusters and redundant distance constraints combinatorially, O(n²), and finding rigid components feeds directly into Stage 3. It's also a genuinely fun ~200-line algorithm.
4. **Minimal conflict sets:** when over-constrained, greedily grow/shrink to a minimal infeasible subset (deletion-filter algorithm) so the user gets "remove one of these 3," not a list of 40 suspects.

**Important honesty point baked into the design:** all of this is *structural* — it cannot see dependencies that follow from geometric theorems (three concurrent perpendiculars, Pappus configurations). Log the cases where structural analysis says "fine" but the Jacobian rank says otherwise; that residue is Stage 4's motivation and your test corpus for it.

**Exit criteria:** every failed solve produces an actionable diagnosis; UI colors entities by constraint state (the FreeCAD-style green/orange/red, but trustworthy).

**Read:** Pothen & Fan (1990) on computing DM decompositions; Jacobs & Hendrickson (1997); Zou et al. arXiv:2202.13795 §Detector for the taxonomy.

---

## Stage 3 — Decomposition: the graph-constructive core

**Goal:** the DCM move. Stop solving sketches monolithically; decompose into small rigid subsystems, solve those (closed-form where possible), recombine. This is the largest single stage and the one with no serious open-source precedent — it's the gap in the world.

**Two sub-phases:**

**3a. Fudos–Hoffmann cluster merging (bottom-up).** Seed clusters from pairs of primitives joined by a constraint; repeatedly find three clusters pairwise sharing geometry and merge them by solving a triangle-like placement problem (each merge is a rigid-body transform computed from 3 shared elements). Each terminal subproblem is small — most have **ruler-and-compass closed forms** (point from two distances = circle intersection, etc.). Build a library of these constructions with explicit **chirality flags** (each has 2 roots; record which was chosen — this becomes Stage 5's solution-management substrate). Numeric fallback (your Stage 1 core) for non-constructible clusters. This handles the "tree-decomposable" class, which covers a large majority of real engineering sketches.

**3b. Owen's triconnected split (top-down) + DR-planning ideas.** Split the constraint graph at articulation pairs (SPQR-tree decomposition — Hopcroft–Tarjan; OGDF has a reference implementation worth studying) so that non-tree-decomposable cores are isolated into minimal subsystems before the numeric solver ever sees them. The Hoffmann–Lomonosov–Sitharam framing gives you the objective function: *minimize the size of the largest subsystem sent to the numeric solver*, because cost is exponential in exactly that.

**Architecture:** decomposition runs in Python (it's graph manipulation, done once per topology change); it emits a **solve plan** — a DAG of cluster-solve and cluster-merge steps — that the C core executes. Topology changes are rare; dragging re-executes the cached plan with new numbers. This is why DCM drags large sketches effortlessly: dragging never re-analyzes the graph.

**Exit criteria:** a 1,000-entity mostly-tree-decomposable sketch solves in low milliseconds from the cached plan; the monolithic path is now the fallback, not the norm; solver regression suite runs both paths and diffs.

**Read:** Owen (1991, DOI 10.1145/112515.112573); Bouma/Fudos/Hoffmann (CAD 27(6), 1995); Fudos & Hoffmann (ACM TOG 16(2), 1997) — read all three, in that order; Hoffmann–Lomonosov–Sitharam JSC 2001 parts I & II.

---

## Stage 4 — The witness configuration method

**Goal:** catch the dependencies Stage 2 can't — the non-structural, theorem-induced ones. This is the marker of a *modern* solver per the post-2005 literature, and it's surprisingly cheap given Stage 1's rank machinery.

**Method (Michelucci & Foufou):** construct a **witness** — a configuration sharing the sketch's incidence structure but with generic dimensions (perturb the user's sketch, or re-solve with randomized dimension values; the user's own sketch is often already an adequate witness since it satisfies the incidences by construction). Evaluate the Jacobian there and analyze it with rank-revealing QR/SVD: rank deficiency in rows = dependent constraints (including theorem-induced ones); the null space of `J` = the infinitesimal motions = exactly which DOFs remain and what they look like geometrically (animate them in the UI — genuinely great UX for "why isn't this fully constrained").

**Integration:** run witness analysis (a) on demand for full diagnosis, (b) automatically on the small non-decomposable cores from Stage 3, where degeneracy concentrates. Numerical rank needs care — scale parameters, use a relative tolerance tied to sketch extent, and cross-check QR against SVD on disagreement.

**Exit criteria:** the logged Stage-2 residue (structurally-fine-but-rank-deficient cases) is now correctly diagnosed; you can display remaining-DOF motions as animations.

**Read:** Michelucci & Foufou, CAD 38(4), 2006; the follow-up "Interrogating witnesses" (Information & Computation, 2012); Thierry et al. 2011.

---

## Stage 5 — Dragging robustness and solution management

**Goal:** the qualities users can feel but can't name: no solution jumping, continuity under edits, deliberate root selection. This is where commercial solvers earn their licensing fees.

**The work:**

1. **Chirality tracking:** every closed-form construction from Stage 3a recorded a root choice. Persist those flags in the document; on re-solve, prefer the recorded branch; expose "flip solution" per cluster (the tangent-inside/outside toggle every CAD user knows).
2. **Continuation-style dragging:** for large drags, step the target in increments and re-solve at each (SolveSpace's documented mitigation), so the solution point tracks its homotopy branch instead of teleporting across it.
3. **Order-type guards:** detect when a Newton step would flip the sign of an oriented-area/order-type invariant in a cluster and either damp the step or flag the flip. (Preserving all order types is NP-hard in general — this is a heuristic layer, and that's fine; it's what everyone does.)
4. **Homotopy continuation for enumeration** on the small non-decomposable cores only: coefficient-parameter homotopy from a solved generic instance lets you enumerate nearby real roots and offer them as alternatives. Small systems only — this is the academically honest version of "we can show you the other solutions."

**Exit criteria:** a torture suite of dragging scenarios (recorded pointer trajectories on nasty sketches) runs with zero solution jumps; branch choices survive save/load.

**Read:** Essert-Villard/Schreck/Dufourd (2000) on sketch-guided root selection; Sitharam et al. ACM TOG 2006 on solution-space navigation; Durand & Hoffmann (2000) on homotopy for GCS.

---

## Stage 6 — The frontier: solver-as-oracle, auto-constrain, and the benchmark

**Goal:** the state of the art in 2026 isn't a fancier Newton — it's the solver's *new role* as the verifier inside ML loops, plus filling the field's embarrassing benchmarking gap.

**6a. Batch oracle API.** Expose exactly what the Autodesk alignment work (Casey et al., arXiv:2504.13178) needed and had to build privately: given (primitives, candidate constraint set), return `{fully_constrained | under | over | unsolvable}` + remaining-DOF count + a stability score (how far the geometry moved when constraints were applied). Make it GIL-free, batchable, and fast (thousands of evaluations/sec — Stages 1–4 make this nearly free). **No fast open solver-oracle exists; this alone would get the project used by every group doing RL/preference-tuning on constraint generation.**

**6b. Auto-constrain.** Classical version first: snap-detection of near-horizontal/vertical/tangent/equal within tolerance, filtered through Stage 2/4 analysis so proposals never over-constrain — this is beautification à la the classical literature. Then optionally the learned version: fine-tune a small constraint-inference model on SketchGraphs (15M Onshape sketches with ground-truth constraint graphs) using your 6a oracle as the reward, replicating the Casey et al. loop (93% fully-constrained via RLOO vs 34% SFT) entirely in the open. That replication is a publishable result by itself.

**6c. The benchmark suite.** The field has no shared benchmark — no Maros–Mészáros equivalent. You are unusually well-positioned to fix a standards gap (this is, after all, the GTFS move): a versioned corpus of sketches in a documented JSON schema — solvable/over/under/theorem-degenerate cases, dragging trajectories with expected-continuity assertions, timing categories — plus a harness with adapters for PlaneGCS, SolveSpace, and yours. Seed it from SketchGraphs extractions plus hand-built degenerate cases from the literature.

**Read:** Casey et al. ICCV 2025 (arXiv:2504.13178); Seff et al. SketchGraphs (arXiv:2007.08506); Karadeniz et al. DAVINCI (arXiv:2410.22857) for the constraint-inference task framing.

---

## Cross-cutting infrastructure (start on day one)

**Testing:** the finite-difference Jacobian checker (Stage 0); property-based tests — generate random Laman graphs via Henneberg construction, assign random lengths, assert the solver finds a realization and the pebble game agrees it's rigid; golden-file regression on the growing sketch corpus; fuzz the compile-to-plan boundary.

**Determinism:** same sketch + same edit ⇒ bit-identical result. Ordered iteration everywhere, seeded perturbations, no address-dependent hashing in the plan compiler. Non-determinism in a constraint solver destroys user trust and makes every bug unreproducible.

**Corpus:** harvest real sketches early — the SketchGraphs pipeline, FreeCAD `.FCStd` files, your own Onshape documents exported via their REST API. Synthetic tests lie; real sketches are pathological in ways you won't invent.

**Profiling discipline:** per-stage flamegraphs of the drag loop, because the whole architecture is justified by drag latency and it's easy to quietly regress.

## Reading spine (in order of need)

Zou et al., *A review on geometric constraint solving*, arXiv:2202.13795 (orientation, read first) → SolveSpace tech notes + PlaneGCS source (Stage 0–1) → Pothen & Fan 1990, Jacobs & Hendrickson 1997 (Stage 2) → Owen 1991; Bouma et al. 1995; Fudos & Hoffmann 1997; Hoffmann–Lomonosov–Sitharam 2001 (Stage 3) → Michelucci & Foufou 2006 (Stage 4) → Sitharam et al. TOG 2006; Durand & Hoffmann 2000 (Stage 5) → Casey et al. 2025; Seff et al. 2020 (Stage 6). The Sitharam/St. John/Sidman *Handbook of Geometric Constraint Systems Principles* (2018) is the desk reference throughout.
