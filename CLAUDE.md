# geomsolver

A geometric constraint solver, and **Solvent**, the language a drawing in it is written as.

**Start here.** Asked to *draw* something — a sketch, constraints, why a figure will not solve —
read [`docs/solvent-primer.md`](docs/solvent-primer.md) first: the language as the implementation
accepts it. `solvent-spec.md` is the normative specification but specifies constructs that do not
parse yet (`hint` as a statement, `path`), so write from the primer.  Asked to work on the
*solver* — kernels, diagnosis, decomposition, the bindings, the app — this file and the topic
files it lists are the contract, and `gcs-solver-program.md` is the staged program it is built to.

## Layout

* **core** (`rust/gcs-core/`): the whole engine in Rust, no dependencies — model, constraints,
  kernels, solvers, diagnosis, decomposition, the Solvent language, solids, the B-rep kernel.
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
| [language.md](docs/contract/language.md) | `:=`, `hint(…)` and writeback, anonymous children, modules and imports, relation words, predicates applied, components, interfaces, `ring`, `when`, `rate` and `initially`, recursion, styles and classes | you touch `syntax/`, `flatten/`, `program/`, `modules.rs`, `style.rs`, or write `.sv` |
| [relations.md](docs/contract/relations.md) | how a relation is written and settled into a `CKind`, chains and threading, sides and selectors, ordinates and `level`, statement identity under expansion | you touch `constraints::infix_op`/`prefix_op`, `Written`, chains, or add a relation word |
| [numbers.md](docs/contract/numbers.md) | dimension expressions, units and their checking, declared unknowns, free variables | you touch `expr.rs`, `units.rs`, `param`, or a dimension's text |
| [space.md](docs/contract/space.md) | points in space, axes, planes and their origins, `in` membership, twins, `project`, stacks | you touch `planes.rs`, `views.rs`, `reading.rs`, axes or planes |
| [sets.md](docs/contract/sets.md) | sets and their uses, derivatives along a set, regions and region solids, faces as sets, distances from a set and offsets, a plane tangent to a set | you touch `flatten/sets.rs`, `solid/region.rs`, `solid/faces.rs`, or a set in `std.sv` |
| [rounds.md](docs/contract/rounds.md) | rounds and chamfers as words of `std.sv`, the lowering table, the conrod, draft | you touch `solid/lowering.rs`, `brep/fillet.rs`, `brep/corners.rs`, or `std.Rounded` |
| [curves.md](docs/contract/curves.md) | splines (rational too), control-polygon edits, curve parameters, curves as points of components, loci and traces, variational (free) curves, rates along a running formal, the method of lines, the rate of a contact (one kernel for every `at:` statement), the ellipse | you touch `curve.rs`, `locus.rs`, `tape.rs`, `variational.rs`, `extremal.rs`, `ode.rs`, `rates.rs`, `lines.rs`, `contact_rate.rs` |
| [frames.md](docs/contract/frames.md) | frames that move: a frame riding on the frames of its blocks, `in` nested in `in`, carriers, readings at a roll that infer their carrier, what a reading makes riding on its observer | you touch `program/motions.rs`, `program/planes.rs`'s paths, `Sketch::carriers`, or a reading's `under:` |
| [solver.md](docs/contract/solver.md) | constraints owning unknowns, scaling, "solved", rank, tangencies, claims, bounds, diagnosis, decomposition, drags, the block-triangular solve and root search | you touch `kernels.rs`, `system.rs`, `solve.rs`, `diagnose*`, `decompose.rs`, `cgraph.rs` |
| [solids.md](docs/contract/solids.md) | the body rule, faces and their names, sheets, solid claims, glTF, meshes, the spiral bevel | you touch `solid.rs`, `csg.rs`, `mesh.rs`, `hidden.rs`, `gltf.rs`, or `hypoid.sv` |
| [generating.md](docs/contract/generating.md) | named motions, planar generation, solid placement, swept solids and their admission, export tolerance, sectors, screws, the Wankel | you touch `motion.rs`, `generate.rs`, `solid/admission.rs`, `brep/sweep`, or the native export |
| [analytic.md](docs/contract/analytic.md) | surface references, envelopes, interval bounds, shell topology, and the post-solve B-rep as Rust API (patches, seams, corners, edges, face boundaries) | you touch `envelope/`, `patch.rs`, `seam.rs`, `interval.rs`, `topology.rs` |
| [fields.md](docs/contract/fields.md) | continuous motion solids, field meshing in the page, material fields, removed tracks | you touch `solid/field/`, `delaunay/`, `crease.rs`, or the mesh worker |
| [brep.md](docs/contract/brep.md) | the clean-room kernel, its phases and rungs, OCCT as oracle | you touch `brep/` |
| [app.md](docs/contract/app.md) | the document as source, splicing edits, graft, solid gestures, the workspace, the glass box, callouts, the program panel and editor, drags in the page | you touch `edit.rs`, `io.rs`, `overview/`, `callout.rs`, or `web/src/app/` |

## Rules for every change

- **Model and drawing are separate files.** `.sv` is geometry, constraints, hints and assertions
  only; presentation belongs in `.svd` ([docs/solvent-drawing.md](docs/solvent-drawing.md)).
  Never write `class`, `style`, `view`, `section`, `dimensions` or callout placements in model
  source (the full rule: [language.md](docs/contract/language.md)).
- **The core owns every algorithm.**  A change to the model, a constraint type, diagnosis,
  decomposition or the solvers lands in `rust/gcs-core/` with a Rust test in
  `rust/gcs-core/tests/` — **a new file there is listed in `tests/main.rs`** (one binary,
  `autotests = false`; `every_file_is_a_module` fails an unlisted file).  A binding changes only
  when the *surface* changes.  Geometry or numerics in TypeScript belong in Rust instead.
- A new *entity* kind stops the build in the exhaustive `match e.kind` arms — `model.rs`
  (`entity_params`, `own_params`, `own_length_params`, `children`, `min_children`, `count`,
  `bounds`, `distance_between`, `point_to_drawn`, `class_of`, `set_class`, `spatial`), `io::graft`'s
  remap, `overview::drawable`, `svg::entity`, `program`'s build and `set_class`,
  `syntax::kind_initial` and the FFI's `ent`/`kind_id`.  Give it an arm in each; `primitives()`
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
- No LAPACK/BLAS: the QR, complete-orthogonal, SVD and LDLᵀ routines are ours, and
  `rust/gcs-core/tests/linalg.rs` checks them against `nalgebra`, on purpose.  **The library has
  no dependencies; its tests have one reference implementation**, a `[dev-dependencies]` entry so
  nothing links into the cdylib or wasm.  Each test also states the property (`A ≈ QR`,
  `NᵀN ≈ I`, minimum-norm orthogonal to the null space): agreement is evidence, the property the
  contract.

