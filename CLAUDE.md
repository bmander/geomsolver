# geomsolver

A geometric constraint solver, and **Solvent**, the language a drawing in it is written as.

**Start here.** Asked to *draw* something — a sketch, constraints, why a figure will not solve —
read [`docs/solvent-primer.md`](docs/solvent-primer.md) first: the language as the implementation
accepts it. `solvent-spec.md` is the normative specification but specifies constructs that do not
parse yet (`hint` as a statement, `path`), so write from the primer.  Asked to work on the
*solver* — kernels, diagnosis, decomposition, the bindings, the app — this file and the topic
files it lists are the contract, and `gcs-solver-program.md` is the staged program it is built to.

## Layout

* **core**: the whole engine in Rust, no dependencies, as two crates.
  * **solver** (`rust/gcs-solve/`): model, constraints, kernels, solvers, diagnosis,
    decomposition, and what they evaluate: curves, solids, motions, the B-rep kernel.  It knows
    no Solvent, and the compiler holds it: nothing in it can name the crate built on it.
  * **language** (`rust/gcs-core/`): the Solvent language and what is built on it, over
    `gcs-solve`, whose modules it re-exports by name (`gcs_core::model` is `gcs_solve::model`).
* **ABI** (`rust/gcs-ffi/`): one flat C ABI over the core, built as a native `cdylib` and as a
  `wasm32-unknown-unknown` module.
* **binding** (`web/src/core/`): thin TypeScript proxies over the wasm core. No algorithm.
* **CLI** (`rust/gcs-cli/`): `solventc`, which parses, elaborates, solves, diagnoses, reports and
  exports. It invents no wording: every line it prints is the core's.
* **app** (`web/src/app/`): the 3D-workspace sketcher, the only front end.

The full map, and what each layer may and may not hold, is
[docs/contract/architecture.md](docs/contract/architecture.md).

## Commands

`make` (native `build/libgcs.dylib`), `make solventc` (`build/solventc`),
`make wasm` (`web/src/wasm/gcs.wasm`),
`make test` (both released artefacts, cargo and the web suite),
`cargo test --manifest-path rust/Cargo.toml` (**never `--release`**: the suite runs under
`[profile.test]` — optimised, no LTO, no debuginfo; `rust/Cargo.toml` says why; one file of the
core suite is filtered as `cargo test chain::`),
`cd web && npm test`, `make bench` (the native `bench` binary and `npm run bench`, read side by
side), `cd web && npm run serve`.

`npm run build` is `tsc` (module per file, what `node --test` runs against) then esbuild, which
rolls `dist/app/main.js` into one `dist/app/bundle.js`, what the page loads.  The bundle is written
*beside* the entry module: `core/wasm.ts` finds the core at `../wasm/gcs.wasm` relative to
`import.meta.url`, which under bundling is the bundle's URL.  The node-only fallback imports in
`wasm.ts` are left external.  **three.js is the project's one runtime dependency**, bundled (not
from a CDN) so the app opens from a file.

## The contract, by subsystem

This file is the index; the rules themselves are in `docs/contract/`. **Before changing an area,
read its file** — each paragraph there is a rule the code and its tests hold, and most name the
gate test that checks it. When a change adds or alters a rule, edit the topic file (not this
one), and add a row here only for a new topic. Language changes are also recorded in the
[changelog](docs/solvent-changelog.md) under their spec version.

| file | covers | read when |
|---|---|---|
| [architecture.md](docs/contract/architecture.md) | the core, the ABI, the TypeScript binding, the CLI and the app, and what each may hold | you need the map: which layer a change belongs in |
| [language.md](docs/contract/language.md) | `:=`, `hint(…)` and writeback, anonymous children, modules and imports, relation words, predicates applied, components, a closed instance's binding and its mirrored placement, interfaces, `ring` and the instances it places, `when`, `rate` and `initially`, recursion, styles and classes | you touch `syntax/`, `flatten/`, `program/`, `modules.rs`, `style.rs`, or write `.sv` |
| [relations.md](docs/contract/relations.md) | how a relation is written and settled into a `CKind`, chains and threading, corner rounds as joints, sides and selectors, ordinates and `level`, statement identity under expansion | you touch `constraints::infix_op`/`prefix_op`, `Written`, chains, `program/rounds.rs`, `model/rounds.rs`, or add a relation word |
| [numbers.md](docs/contract/numbers.md) | dimension expressions, units and their checking, declared unknowns, free variables | you touch `expr.rs`, `units.rs`, `param`, or a dimension's text |
| [space.md](docs/contract/space.md) | points in space, axes, planes and their origins, `in` membership, twins, `project`, stacks | you touch `planes.rs`, `views.rs`, `reading.rs`, axes or planes |
| [sets.md](docs/contract/sets.md) | sets and their uses, derivatives along a set, regions and region solids, faces as sets, distances from a set and offsets, a plane tangent to a set | you touch `flatten/sets.rs`, `solid/region.rs`, `solid/faces.rs`, or a set in `std.sv` |
| [rounds.md](docs/contract/rounds.md) | rounds and chamfers as words of `std.sv`, the lowering table, the conrod, draft | you touch `solid/lowering.rs`, `brep/fillet.rs`, `brep/corners.rs`, or `std.Rounded` |
| [curves.md](docs/contract/curves.md) | splines (rational too), control-polygon edits, curve parameters, curves as points of components, loci and traces, variational (free) curves, rates along a running formal, the method of lines, the rate of a contact (one kernel for every `at:` statement), the ellipse | you touch `curve.rs`, `locus.rs`, `tape.rs`, `variational.rs`, `extremal.rs`, `ode.rs`, `rates.rs`, `lines.rs`, `contact_rate.rs` |
| [frames.md](docs/contract/frames.md) | frames that move: a frame riding on the frames of its blocks, `in` nested in `in`, carriers, readings at a roll that infer their carrier, what a reading makes riding on its observer, a point or a frame read at a roll owned or stated, joints about a frame's axes, the station words | you touch `program/motions.rs`, `program/planes.rs`'s paths, `Sketch::carriers`, a reading's `under:`, `flatten/carried.rs`, `flatten/valued.rs`, `motion/guide.rs` or `envelope/screw.rs` |
| [parameters.md](docs/contract/parameters.md) | numbers that run (#391): `model::Running`, a declared `param t: Angle over (a, b)`, and the face family's running number, a traced body's formals and the roll's intervals as uses of it; the two readings of what moves, `x at τ` and `over t in (a, b)` (`sweep`, `trace`, an envelope's and a meshing's); a joint's value, `fix(value == τ) f`, and `n := x in f`, drawn where it puts `x`; relations over a parameter, continuation and singular instants, the ledger per parameter, `trace(m over t)`; a joint driven by a law of a parameter (a cam), its jet and its interval bounds; faces swept over a declared parameter, a valued frame's block, frames along a path (#398); a drawing stood at an instant of its march, the timeline's core (#398 L); a frame the drawing moves read as a motion, a trace in space (#398 F) | you touch `model/running.rs`, a `param … over`, `sweep(… over s)`, `flatten/valued.rs`, `motion/guide.rs`, `followed.rs`, a joint's `value`, `x in f`, `SolidDef::Family::over`, `TracedE::over`, `CurveE::over`, a reading's roll interval, `flatten/carried.rs`, `P::instant`, `follow.rs`, `CurveBody::Followed`, `motion/law.rs`, `MotionDef::Driven`, `tape::eval_interval`, `follow::stand`/`Scrub` or `motion/track.rs` |
| [analysis.md](docs/contract/analysis.md) | structural analysis as design intent (#510): a scenario drawn — where its lines fall and how the drawing moves under it, the mesh that was solved, one reading with its answers, read in a worker; an output of a linear solve `K(x)u = f` as a row, its interior a kernel and not a boundary, the adjoint gradient on the factor in hand, the stiffness as strain rows' Gram matrix, a structure's columns and constants, a structure with no displacement to give; load scenarios, `scenario { }` parsed and elaborated into `model::Scenario` — supports, loads, requirements and their grades, targets through the selectors, units, extension and scoping, nothing solved yet; materials as model values in SI, a solid's `of`, their JSON and report; the sparse symmetric factor and its inertia, the lowest modes of a pencil by shift-invert Lanczos, linear buckling, no repeated pair missed; the diagnosis words for analysis (#510 P7) — a rigid-body motion named from the supports, a structure's mechanism, unbounded stress at a re-entrant edge, a degenerate modal pair and its ring, a requirement out of reach; a volume mesh fit for analysis — cells refined to a radius–edge ratio, a size and a least dihedral angle, slivers by their picking region, TET10 nodes on the surface and the feature curves, no element inverted by its Bernstein coefficients; the verification corpus (#510 P8), analytic cases and NAFEMS benchmarks as documents whose claims are their own closed forms, CalculiX as the oracle; frames as claims (#510 S0) — the Euler–Bernoulli beam as the interior's second element, its rows, gradient, mass and geometric stiffness, `beam(…) of m` as a statement, the drawing as the mesh, a scenario's requirements on a frame answered and judged; a frame's driving deflection as a row (#510 S1) — its frame meshed at its joints, its columns the joints' lifts and the unknowns a section is read over by a tape, a spread load's `∂f/∂x`, the row the source's alone; solids read in their plane (#510 S2) — `plane stress`/`plane strain` as a scenario's line, the quadratic triangle as the interior's third element, loads laid on the plane, a region meshed by Ruppert's refinement with its nodes on the drawing's curves, a plane structure factored under minimum degree, what a region answers and why not, a region's driving deflection as a row whose compiled mesh moves with the drawing; solids read in three dimensions (#510 S3) — the quadratic tetrahedron as the interior's fourth element, loads a solid bears, a prism as its face's mesh stacked through its depth, any other solid its exact boundary refined and filled, the six outputs of a solid, a solid's driving deflection as a row stacked over its face's, the error estimated beside every output, the eigensolver's factor as P1's supernodal LDLᵀ, a tension's buckling counted; what a body is made of, the drawing's: `mass(m) B` and `volume(v) B`, stated as a row or claimed | you touch `interior.rs`, `KernelKey::Interior` or `sparse::Ata::least_pivot`; `model/scenario.rs`, `syntax/scenario.rs`, `parser/scenario.rs`, `program/scenarios.rs`, or write a `scenario`; `model/material.rs`, a `material(…)` group or a solid's `of`; `eigen.rs`, `sparse::Sym`, `sparse::Ldl` or `sparse::Pencil`; `diagnose/analysis.rs`, `Structure::free` or `reach`, `BodyFaces::whole`; `delaunay/refine/cells.rs`, `delaunay/refine/volume.rs` or `delaunay/tetra.rs`; `rust/examples/analysis/` or `tests/analysis_corpus*`; `interior::Beam`, `model/beam.rs`, `syntax/beam.rs`, `program/beams.rs`, `beams.rs` or `report::scenario_answers`; `CKind::Deflection`, `beams::Row`, `interior::Spread`/`Sizing`, `model::Over` or `Sketch::scenarios`; `interior/plane.rs`, `interior/moved.rs`, `delaunay/plane.rs`, `prisms.rs`, a scenario's `plane stress` line or `Sketch::meshed`; `interior/solid.rs`, `interior::Stack`, `bodies.rs`, `beams::Read::errors`, `cholesky::Cholesky::factor_ldl` or `eigen::UNBUCKLED`; `deformed.rs`, `overview/scenario.rs`, a reader's `read_pictured` or `report::scenarios_pictured_json`; `interior::Reads`, `CKind::Mass`/`Volume`, `model::Measured`, `measured.rs`, `program/measures.rs`, a reader's `measure`; `pcg.rs`, `Structure::solve_by` or `BodyFaces::reading` |
| [solver.md](docs/contract/solver.md) | constraints owning unknowns, scaling, "solved", rank, tangencies, claims, bounds, diagnosis, decomposition, drags, the block-triangular solve and root search | you touch `kernels.rs`, `system.rs`, `solve.rs`, `diagnose*`, `decompose.rs`, `cgraph.rs`, `sparse.rs`, `cholesky.rs`, `amd.rs` |
| [solids.md](docs/contract/solids.md) | the body rule, faces and their names, a refused solid said once, sheets, solid claims, glTF, meshes, solids of one binding evaluated once and placed, a mirrored instance's solids reflected, the spiral bevel | you touch `solid.rs`, `csg.rs`, `mesh.rs`, `hidden.rs`, `gltf.rs`, `model/bindings.rs`, `program/refused.rs`, or `hypoid.sv` |
| [generating.md](docs/contract/generating.md) | named motions, planar generation, solid placement, swept solids and their admission, export tolerance, sectors, screws, the Wankel | you touch `motion.rs`, `generate.rs`, `solid/admission.rs`, `brep/sweep`, or the native export |
| [analytic.md](docs/contract/analytic.md) | surface references, envelopes, interval bounds, shell topology, and the post-solve B-rep as Rust API (patches, seams, corners, edges, face boundaries) | you touch `envelope/`, `patch.rs`, `seam.rs`, `interval.rs`, `topology.rs` |
| [fields.md](docs/contract/fields.md) | continuous motion solids, field meshing in the page, material fields, an operand built exactly read by its own boundary, removed tracks | you touch `solid/field/`, `delaunay/`, `crease.rs`, or the mesh worker |
| [brep.md](docs/contract/brep.md) | the clean-room kernel, its phases and rungs, a B-rep reflected, OCCT as oracle | you touch `brep/` |
| [app.md](docs/contract/app.md) | the document as source, splicing edits, graft, solid gestures, the workspace, the glass box, callouts, the program panel and editor, the names under its caret, drags in the page, the timeline | you touch `edit.rs`, `io.rs`, `overview/`, `callout.rs`, `syntax/symbols.rs`, or `web/src/app/` |

## Rules for every change

- **Model and drawing are separate files.** `.sv` is geometry, constraints, hints and assertions
  only; presentation belongs in `.svd` ([docs/solvent-drawing.md](docs/solvent-drawing.md)).
  Never write `class`, `style`, `view`, `section`, `dimensions` or callout placements in model
  source (the full rule: [language.md](docs/contract/language.md)).
- **The core owns every algorithm.**  A change to the model, a constraint type, diagnosis,
  decomposition or the solvers lands in `rust/gcs-solve/` (the language's in `rust/gcs-core/`)
  with a Rust test in `rust/gcs-core/tests/`, the suite of both crates — **a new file there is
  listed in `tests/main.rs`** (one binary, `autotests = false`; `every_file_is_a_module` fails
  an unlisted file).  A binding changes only when the *surface* changes.  Geometry or numerics
  in TypeScript belong in Rust instead.
- A new *entity* kind stops the build in the exhaustive `match e.kind` arms — `model.rs`
  (`entity_params`, `own_params`, `own_length_params`, `children`, `min_children`, `count`,
  `bounds`, `distance_between`, `point_to_drawn`, `class_of`, `set_class`, `spatial`), `graft`'s
  remap, `overview::drawable`, `svg::entity`, `program`'s build and `set_class`,
  `model::kind_initial` and the FFI's `ent`/`kind_id`.  Give it an arm in each; `primitives()`
  and `topology_key` are where it joins the document.  A kind **evaluated rather than drawn**
  (`Face`, `Solid`) answers `spatial()` and owns no parameter.
- Every new constraint type = a vectorized kernel in `kernels.rs` (added to `KERNELS`; the
  registration order **is** the kernel id) declaring its `degree` — the power of length its
  residual carries, 1 for a signed distance and 2 for a squared one — a `CKind` variant in
  `constraints.rs` declaring its `spec` (constructor args as (attr, kind) pairs), `params()`,
  `consts()` and `default_arg()`, and a row in `rust/gcs-core/tests/jacobians.rs` (FD check,
  spec round-trip).  Both bindings generate their classes from `report::registry_json`.
- A type whose spec carries a `Length` or an `Angle` is a *dimension*, and dimensions are drawn.
  `callout.rs` matches `CKind` exhaustively in two places: give a new type a `Pen` arm (its
  drafting figure) and a `frame` arm (the `Frame` its placement is written in), or list it in
  `undrawn!`.  `every_dimension_is_drawn` checks the arm produces a figure.
- **Every case in the library is a Solvent document**: a `.sv` file in `rust/examples/` whose
  builder elaborates the text (`examples::document`).  `with_params` gives the document's own
  named numbers (`w := 100`, `distance(a := 30)`) the caller's values — no second copy in Rust.
  A start off the solution is `jitter`, a function of the sketch.  **Seeds must track
  parameters** (`pythagoras.sv` places its square from `la`/`lb`).  `tests/examples_sv.rs` holds
  each document to what the library advertises.
- **What is not a drawing is not a case** (`fixtures.rs`).  `laman` and `henneberg_edges` make
  random graphs for property tests; they stay in the core, in their own module, off the case
  library and the app's menu.  A case belongs in Solvent when its numbers are things a person
  would state, and stays a generator when a program measured them.
- Slow tests are gated by `#[ignore]` (cargo). The **slow tier** — ten seconds to minutes, still
  passing on every landing (gear whole-member verifications, a native tooth space) — is
  `#[cfg_attr(not(feature = "slow"), ignore = "slow tier, …")]`, run by `make test-slow`
  (`OCCT=1` for the native one); `make test` leaves it out.
- **Measure compile time and test time separately**
  ([`docs/build-performance.md`](docs/build-performance.md)). `make test` builds both release
  artefacts and runs every integration suite and doc test. Release: incremental ThinLTO, 16
  codegen units; native test profile: opt-level 2, debug assertions, no debuginfo. The core suite
  is one binary (`gcs-core/tests/main.rs`); libraries and binaries have `test = false`, so an
  in-source unit test needs that target's harness enabled again. The TypeScript cache lives in
  `web/dist`, so removing the output removes it.
  Unpacked debuginfo in `target/debug/deps` makes macOS launches slow: keep `debug = 0`;
  `cargo clean --profile dev` clears an existing accumulation.
- Benchmark on a quiet machine (`uptime`; a JVM indexer often runs here).  Native:
  `rust/gcs-core/src/bin/bench.rs` (`cargo run --release -p gcs-core --bin bench`); wasm: `npm run
  bench`; `make bench` runs both.  Wall-clock medians only — no harness dependency.
- The ABI is a panic boundary: every entry point runs inside `guard`, so a core panic becomes
  `gcs_last_error()` and a neutral return.  That needs `panic = "unwind"` in the release profile.
  `wasm32-unknown-unknown` aborts whatever the profile says, so untrusted input (a document) is
  bounds-checked in the core as well.
- Determinism: ordered containers only (`Vec`, `BTreeMap`/`BTreeSet`), never `HashMap` iteration
  in the solve path.  Every random draw comes from the seeded `rng::Rng`.
- The trust-region loop is `newton::dogleg` over a `TrustRegion`; a new thing to minimise
  implements the trait rather than copying the loop.
- Nothing in the project is auto-formatted: there is no `rustfmt.toml`, and `cargo fmt` would
  reformat every file.  Match the surrounding style by hand (100 columns).
- No LAPACK/BLAS: the QR, complete-orthogonal, SVD, LDLᵀ and sparse Cholesky routines are ours,
  and `rust/gcs-core/tests/linalg.rs` checks them against `nalgebra`, on purpose.  **The library has
  no dependencies; its tests have one reference implementation**, a `[dev-dependencies]` entry so
  nothing links into the cdylib or wasm.  Each test also states the property (`A ≈ QR`,
  `NᵀN ≈ I`, minimum-norm orthogonal to the null space): agreement is evidence, the property the
  contract.

