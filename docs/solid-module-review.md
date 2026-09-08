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
it is not proof of numerical correctness. The cube, tetrahedron and apex acceptance
failures remain explicitly ignored and unresolved.

Validation: `make test` passes after building release native and WASM artifacts:
1,066 core tests pass (four existing ignored cases), 247 web tests pass, and the
CLI, native FFI and doctest suites pass. The focused `generic_front` run also
passes its ten enabled cases with the same three ignored acceptance cases.

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

3. **Continue splitting by ownership where changes require it.**
   `model.rs` (2,704 lines) is the next useful mechanical candidate: entity definitions
   and metadata, construction, and solved geometry access have distinct roles.
   `flatten.rs` (2,192 lines) mixes scope binding, expansion and scalar substitution.
   Preserve canonical parameter order and diagnostics when extracting these. In
   particular, `own_params`, `own_length_params` and `entity_params` are different
   contracts: ownership, length scaling, and Jacobian order including child parameters.
   A shared metadata table would need to express those distinctions before replacing
   their exhaustive matches. File length alone is not a reason to introduce one.

## Deliberate separation

- The marcher's vector normalization uses chained `hypot` without coordinate snapping;
  `plane::unit` uses a different norm, rejection threshold and component cleanup. Merging
  these apparent duplicates would change numerical behavior.
- The independent Python geometry verifiers must retain their own arithmetic and
  reconstruction. Sharing the generator's implementation would remove independent
  evidence, rather than useful duplication.
- Existing faceted CSG and the experimental F-rep marcher have different acceptance
  contracts. Moving files does not make either an interchangeable backend for the other.
