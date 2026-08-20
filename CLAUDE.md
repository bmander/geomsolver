# geomsolver

Geometric constraint solver built in stages per `gcs-solver-program.md` (read it first).
Currently: **Stage 5 done**, in **one** implementation —

* **core** (`rust/gcs-core/`): the whole engine in Rust, no dependencies.  Model and constraints
  (`model.rs`, `constraints.rs`), vectorized kernels and the compile-to-plan `System`
  (`kernels.rs`, `system.rs`), our own DogLeg/LM plus the dense and sparse linear algebra
  (`newton.rs`, `linalg.rs`, `sparse.rs`), structural diagnosis (`graph.rs`, `diagnose.rs`),
  decomposition into cached solve plans (`cgraph.rs`, `decompose.rs`), witness analysis
  (`witness.rs`), drag/solution management (`solve.rs`, `homotopy.rs`), document I/O (`io.rs`,
  `json.rs`) and the reference sketches (`examples.rs`).
* **ABI** (`rust/gcs-ffi/`): one flat C ABI over the core, built twice — a native `cdylib` for
  Python and a self-contained `wasm32-unknown-unknown` module for the browser.
* **bindings**: `src/gcs/` (Python, `ctypes`) and `web/src/core/` (TypeScript, WebAssembly).
  Both are *thin*: proxies over handles, buffers for hot-path numbers, JSON for ragged results.
  Neither contains an algorithm.
* **app** (`web/src/app/`): an HTML5-canvas sketcher, the only front end.

Commands:
`make` (native `build/libgcs.dylib`), `make wasm` (`web/src/wasm/gcs.wasm`),
`make test` (cargo + pytest + mypy + the web suite), `cargo test --manifest-path rust/Cargo.toml`,
`.venv/bin/pytest`, `.venv/bin/mypy` (strict, must stay clean), `cd web && npm test`,
`make bench`, `cd web && npm run serve`.

Conventions:
- **The core owns every algorithm.**  A change to the model, a constraint type, diagnosis,
  decomposition or the solvers lands in `rust/gcs-core/` with a Rust test in
  `rust/gcs-core/tests/`.  A binding changes only when the *surface* changes.  If you find
  yourself writing geometry or numerics in Python or TypeScript, it belongs in Rust instead.
- Every new constraint type = a vectorized kernel in `kernels.rs` (added to `KERNELS`; the
  registration order **is** the kernel id), a `CKind` variant in `constraints.rs` declaring its
  `spec` (constructor args as (attr, kind) pairs), `params()`, `consts()` and `default_arg()`,
  and a row in `rust/gcs-core/tests/jacobians.rs` (FD check, spec round-trip).  Both bindings
  generate their classes from `report::registry_json`, so neither needs touching.
- Mutating a constraint's constants (drag target, edited dimension) must be followed by
  `System::update_consts` / `refresh_consts` on any compiled system, or a recompile.
- `System` is the compile-once / evaluate-many seam; keep the object model out of the hot loop.
  The bindings own a handle — call `dispose()` when you drop one (the drags, the plan solver and
  diagnosis all do).
- Diagnosis is structural (matching/DM); numeric rank cross-check is the only thing that sees
  theorem-type dependencies — that residue is Stage 4's corpus, keep logging it
  (`Diagnosis.warnings`).  It is skipped above `NUMERIC_MAX` (300) free parameters because it
  runs after every edit.
- Decomposition maps constraints onto F–H elements in `cgraph::build`; a new constraint type is
  either an edge (PP/PL), a direction relation, or `unsupported` (numeric residual).  Merge
  decisions use generic-rank at witness poses; chirality of PPP merges is the triangle
  orientation sign from the current sketch.
- Replays are warm-started on the current geometry (leaves re-derived each frame), so the root a
  sketch is on is "nearest the identity"; alternatives are applied by writing geometry, not by
  caching transforms.
- Slow tests are gated by `GCS_SLOW=1` (Python) and `#[ignore]` (cargo).
- Benchmark on a quiet machine (`uptime`); this box often has a JVM indexer at 300% CPU.
- `solve::Drag` is the one point-drag implementation (pull + polish), `RadiusDrag` its scalar
  counterpart for circle/arc radii (a `Radius` with `soft` set — its residual is already
  r − target, so no kernel of its own); the front end only translates coordinates.
- Determinism: ordered containers only (`Vec`, `BTreeMap`/`BTreeSet`), never `HashMap` iteration
  in the solve path.  Every random draw comes from the seeded `rng::Rng`.
- No LAPACK/BLAS: the QR, complete-orthogonal, SVD and LDLᵀ routines are ours, and
  `tests/test_linalg.py` checks them against numpy — the one place two implementations are still
  compared, on purpose.
