# geomsolver

Geometric constraint solver built in stages per `gcs-solver-program.md` (read it first).
Currently: **Stage 5 done**, in two implementations that must stay in step —

* **reference** (`src/gcs/`): numpy/scipy kernels, compile-to-plan `System`, own DogLeg/LM
  (`gcs.newton`), structural diagnosis (`gcs.graph`, `gcs.diagnose`), decomposition into cached
  solve plans (`gcs.cgraph`, `gcs.decompose`), witness analysis (`gcs.witness`), drag/solution
  management (`PlanDrag`, `Drag` guards, `gcs.homotopy`).  A library with no UI — the web app is
  the only front end.  scipy is kept only as `scipy-*` reference methods.  Python ≥ 3.14 in `.venv/`.
* **web** (`csrc/` + `web/`): the numerics in C compiled to WebAssembly, everything above the
  numerics ported to TypeScript in `web/src/core/`, and an HTML5-canvas sketcher in
  `web/src/app/`.  `web/src/core/*.ts` is a file-for-file port of `src/gcs/*.py`.

Commands:
`.venv/bin/pytest` (`tests/test_ccore.py` needs `make` first), `.venv/bin/mypy` (strict, must
stay clean), `.venv/bin/python -m gcs.bench`, `make` (native `build/libgcs.dylib`),
`make wasm` (needs `source ~/emsdk/emsdk_env.sh`), `cd web && npm test` (tsc + `node --test`),
`npm run bench`, `npm run serve`.

Conventions:
- **Both implementations, or neither.**  A change to the model, a constraint type, diagnosis,
  decomposition or the solvers lands in Python *and* in `web/src/core/` (and `csrc/` when it is
  numerics), with the matching test in `tests/` and `web/src/test/core.test.ts`.  Divergence is
  the one failure mode this layout has; `tests/test_ccore.py` exists to catch it for the C part.
  UI-only concerns (the case-library dropdown, colours, dialogs) live in `web/src/app/` alone.
- Every new constraint type = a vectorized kernel in `gcs.kernels` (added to `KERNELS`) **and** in
  `csrc/kernels.c` (same registration order — the ids are the ABI, mirrored in
  `web/src/core/kernels.ts`), a class in `gcs.constraints` and `web/src/core/constraints.ts`
  declaring `kernel`/`kernelId`, `params`, `consts()` and `spec` (constructor args as
  (attr, kind) pairs — drives JSON I/O, the constraint list, value editing and the toolbar
  applier), and a row in `tests/test_jacobians.py::all_constraints` (FD check, spec round-trip,
  vectorized-vs-scalar consistency).
- Mutating a constraint's constants (drag target, edited dimension) must be followed by
  `System.update_consts(c)` / `updateConsts(c)` on any compiled system, or a recompile.
- `System` is the compile-once / evaluate-many seam; keep the object model out of the hot loop.
  In the web build `System` owns a C handle — call `dispose()` when you drop one (the drag,
  plan solver and diagnosis all do).
- Diagnosis is structural (matching/DM); numeric rank cross-check is the only thing that sees
  theorem-type dependencies — that residue is Stage 4's corpus, keep logging it
  (`Diagnosis.warnings`).  It is skipped above `NUMERIC_MAX` (300) free parameters because it
  runs after every edit; both implementations use the same threshold.
- Decomposition maps constraints onto F–H elements in `gcs.cgraph.build`; a new constraint type is
  either an edge (PP/PL), a direction relation, or `unsupported` (numeric residual).  Merge
  decisions use generic-rank at witness poses; chirality of PPP merges is the triangle
  orientation sign from the current sketch.
- Replays are warm-started on the current geometry (leaves re-derived each frame), so the root a
  sketch is on is "nearest the identity"; alternatives are applied by writing geometry, not by
  caching transforms.
- Slow tests are gated by `GCS_SLOW=1`.
- Benchmark on a quiet machine (`uptime`); this box often has a JVM indexer at 300% CPU.
- `gcs.solve.Drag` / `web/src/core/system.ts` `Drag` is the one drag implementation (pull +
  polish); the front end only translates coordinates.
- Determinism: ordered lists only, no set/dict-order-dependent iteration in the solve path.  The
  TypeScript port uses a seeded `Rng` (never `Math.random`) for the same reason.
- No LAPACK/BLAS in `csrc/` — the QR, complete-orthogonal, SVD and LDLᵀ routines are ours, and
  `tests/test_ccore.py` checks them against numpy.
