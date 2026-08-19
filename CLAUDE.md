# geomsolver

Geometric constraint solver built in stages per `gcs-solver-program.md` (read it first).
Currently: **Stage 5 done** — vectorized numpy kernels (`gcs.kernels`), compile-to-plan `System`, own DogLeg/LM
(`gcs.newton`), structural diagnosis (`gcs.graph`, `gcs.diagnose`), decomposition into cached solve plans
(`gcs.cgraph`, `gcs.decompose`), witness analysis (`gcs.witness`), drag/solution management (`PlanDrag`,
`Drag` guards, `gcs.homotopy`); scipy kept only as `scipy-*` reference methods. Python ≥ 3.14 in `.venv/`.

Commands: `.venv/bin/pytest` (Qt tests run offscreen), `.venv/bin/mypy` (strict, must stay clean),
`.venv/bin/python -m gcs.bench`, `.venv/bin/python -m gcs.app` (PySide6 sketcher), `.venv/bin/python -m gcs.canvas <example>`.

Conventions:
- Every new constraint type = a vectorized kernel in `gcs.kernels` (added to `KERNELS`), a class in
  `gcs.constraints` declaring `kernel`, `params`, `consts()` and `spec` (constructor args as (attr, kind) pairs —
  drives JSON I/O, the app's constraint list, value editing and the toolbar applier), and a row in
  `tests/test_jacobians.py::all_constraints` (FD check, spec round-trip, vectorized-vs-scalar consistency).
- Mutating a constraint's constants (drag target, edited dimension) must be followed by `System.update_consts(c)`
  on any compiled system, or a recompile.
- Diagnosis is structural (matching/DM); numeric rank cross-check is the only thing that sees theorem-type
  dependencies — that residue is Stage 4's corpus, keep logging it (`Diagnosis.warnings`).
- Decomposition maps constraints onto F–H elements in `gcs.cgraph.build`; a new constraint type is either an
  edge (PP/PL), a direction relation, or `unsupported` (numeric residual). Merge decisions use generic-rank at
  witness poses; chirality of PPP merges is the triangle orientation sign from the current sketch.
- Replays are warm-started on the current geometry (leaves re-derived each frame), so the root a sketch is on
  is "nearest the identity"; alternatives are applied by writing geometry, not by caching transforms.
- Slow tests are gated by `GCS_SLOW=1`.
- Benchmark on a quiet machine (`uptime`); this box often has a JVM indexer at 300% CPU.
- `gcs.solve.Drag` is the one drag implementation (pull + polish); frontends only translate coordinates.
- Determinism: ordered lists only, no set/dict-order-dependent iteration in the solve path.
- `System` is the compile-once / evaluate-many seam; keep Python object model out of the hot loop.
- Go to C/Cython only when profiling shows the bottleneck (user's call).
