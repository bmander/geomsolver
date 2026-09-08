# Solid module review

This pass pauses meshing algorithm changes after checkpoint `4dd656a`. It extracts
existing responsibilities into private modules while retaining the compiler and
`solid` public entry points. It does not change the language or promote the experimental
marcher into the runtime.

## Mechanical extractions

| Former file | Before | After | Extracted responsibilities |
| --- | ---: | ---: | --- |
| `src/program/solids.rs` | 1,508 | 325 | `claims.rs`: relations and placement; `faces.rs`: boundary lowering and validation; `build.rs`: solid construction |
| `src/solid.rs` | 1,271 | 445 | `profile.rs`: solved profiles and provenance; `primitive.rs`: faceted prisms/revolutions; `document.rs`: dependencies, cache reads and CSG resolution |
| `tests/functional_solids/front.rs` | 591 | 15 | `field.rs`: queries, projection, sizing and seeds; `mesh.rs`: front topology and growth; `geometry.rs`: shared geometry; `cases.rs`: fixtures and acceptance checks |

Paths are relative to `rust/gcs-core`. Feature-probe tests also move from
`front/features.rs` into `front/cases.rs`, leaving discovery itself in `features.rs`.
The largest new module is 527 lines. Helpers shared by these modules are visible only
within their parent; `material_sources` becomes private to document resolution.
The workbench still compiles into the existing single core integration-test binary.

A token comparison against the checkpoint matches all 100 moved function definitions,
ignoring whitespace/comments and the extra `super::` qualification required by a move.
This is a mechanical audit, supplemented by the existing tests and native/WASM builds;
it is not proof of numerical correctness. At the refactor checkpoint, the cube,
tetrahedron and apex acceptance failures were explicitly ignored and unresolved.
Subsequent algorithm results are recorded in [the meshing workbench notes](implicit-meshing-methods.md).

Validation: `make test` passes after building release native and WASM artifacts:
1,066 core tests pass (four existing ignored cases), 247 web tests pass, and the
CLI, native FFI and doctest suites pass. The focused `generic_front` run also
passes its ten enabled cases with the same three ignored acceptance cases.

## Model and expansion follow-up

The next mechanical pass starts from `3bee942` and splits the two largest core source
files, keeping `model::…`, `Sketch` methods and `flatten::expand` / `expand_component`
as the entry points. All child modules are private.

| Former file | Before | After | Responsibilities |
| --- | ---: | ---: | --- |
| `src/model.rs` | 2,704 | 230 | Sketch storage, presentation and public re-exports |
| `src/flatten.rs` | 2,192 | 322 | Expansion state, limits, entry points and diagnostics |

`model/` separates entity metadata (`entities.rs`), spatial records and evaluated-solid
access (`spatial.rs`), geometry/constraint construction (`construction.rs`), parameter
ownership and scaling (`parameters.rs`), topology (`topology.rs`), curve sampling and
its cache (`curves.rs`), solved geometry and bounds (`geometry.rs`), and measurement
and picking (`measure.rs`). These modules range from 209 to 453 lines.

`flatten/` separates scalar definitions and substitution (`values.rs`), source-order
expansion and seed settling (`expand.rs`), argument and module binding (`bindings.rs`),
and final alias/privacy resolution (`resolve.rs`). These modules range from 374 to
563 lines. The public expansion result and lexical scope remain owned by the parent;
helpers needed across phases have parent-only visibility. Scalar-definition bookkeeping
is private to `values.rs`; curve scratch storage is private to `model/curves.rs`.

A token audit compares all 202 function definitions, 37 structs and nine enums against
the pre-refactor sources. Their signatures and bodies match after excluding comments,
whitespace and the internal visibility changes needed by the extraction. The audit
retains literal text, numeric constants, field order and arithmetic order. It supplements
the existing regression suite; it does not establish numerical correctness.

Validation: `make test` passes with release native and WASM artifacts built, 1,082 core
tests passing (three existing ignored), 247 web tests, ten CLI tests, four native FFI
tests and four doctests. The core run includes the separately checkpointed retained-vertex
meshing experiment; no marcher algorithm changes are part of these mechanical splits.

No entity metadata generator or shared field evaluator is introduced in this pass.
The substantial architectural benefit is that parameter contracts, expression substitution,
and final privacy enforcement each now have a distinct home. The opportunities below
remain separate changes requiring their own behavioral validation.

## Architectural opportunities

1. **Give discovery failures a common result type before runtime integration.**
   The workbench currently mixes `Option` projection failures, assertions for query
   exhaustion, and an open frontier after growth. The runtime partition extractor has
   its own explicit `BoundaryError`/`BoundaryStage` contract. A future discovery result
   should distinguish exhausted work, unresolved field bounds, failed projection and
   incomplete topology, retaining diagnostics for each. This would let a caller choose
   whether to refine or stop without confusing a partial candidate with an accepted
   boundary. Keep whole-boundary validation separate from candidate generation; the
   front's sampled error checks do not satisfy the partition extractor's coverage contract.
   The new field/front split gives this work a focused boundary.

2. **Share field composition only where its contracts actually agree.**
   `SpatialField` and `MaterialField` repeat immutable Boolean/transform nodes, depth
   checks and support traversal. A private composition helper could centralize this if
   another field leaf or operation is added. Their evaluators have different jobs:
   spatial queries memoize node identity and a complete point box; material queries
   additionally key the separation band, reflect it through subtraction, retain sweep
   evidence, and share bounded pose caches. Keep those policies explicit and preserve
   the static-source/completed-sweep type distinction and separate depth limits.
   The common support arithmetic already lives in `solid/field.rs`. Introducing a
   generic traversal solely to remove the remaining short matches is not justified yet.

3. **Keep ownership metadata distinct from dependency order.**
   The model split now gathers `own_params`, `own_length_params`, `entity_params`
   and vector/scaling operations in `model/parameters.rs`. Their exhaustive matches
   encode different contracts: ownership, length scaling, and Jacobian order including
   child parameters. A shared metadata table would need to express those distinctions
   before replacing the matches. Similarly, the repeated entity-kind matches in
   construction, serialization and topology serve different purposes. Consolidating
   them into one generic visitor would couple independent changes without yet removing
   a demonstrated source of inconsistency.

4. **Keep expansion phases explicit before changing their representation.**
   `flatten/expand.rs` preserves source order and carries lexical scopes; `bindings.rs`
   binds arguments; `values.rs` preserves dimensions and authored expression text;
   `resolve.rs` checks aliases and private-member access before emitting flat IR.
   This makes a future typed distinction between unresolved and resolved statements
   tractable. Such a change should prove preservation of diagnostics, anonymous/cyclic
   names and privacy rules; it is more than a mechanical refactor. Reference rewriting
   and seed-text rescoping already use the same lookup policy, so a second resolver or
   an untyped catch-all visitor would be a regression.

## Deliberate separation

- The marcher's vector normalization uses chained `hypot` without coordinate snapping;
  `plane::unit` uses a different norm, rejection threshold and component cleanup. Merging
  these apparent duplicates would change numerical behavior.
- The independent Python geometry verifiers must retain their own arithmetic and
  reconstruction. Sharing the generator's implementation would remove independent
  evidence, rather than useful duplication.
- Existing faceted CSG and the experimental F-rep marcher have different acceptance
  contracts. Moving files does not make either an interchangeable backend for the other.
