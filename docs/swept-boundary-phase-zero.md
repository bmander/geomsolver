# Phase 0: field evidence and acceptance

Implementation record for [the architectural plan](swept-boundary-architecture-plan.md).
The baseline is commit `48b915692987a22af6500d240f0c3e7dda0278dc`, on a clean tree.
Work is on `swept-boundary-phase-zero`.

## Reproducible baseline

The source snapshot, five-case Solvent/STL exports, original whole-turn test, two-phase seed
perturbation test, and negative-control results are preserved in
`/private/tmp/swept-boundary-phase0-48b9156/`. `manifest.json` records SHA-256 hashes and options.
The source archive's SHA-256 is
`59663b8206ad2fda346bc70636ba4b0099597a77d79a6ad5a9fea341e7412b07`.
The source is also recoverable directly from the baseline commit; the temporary artifacts
are local evidence and are not required to compile or run the tests.

Baseline options: sagitta 0.02, spacing 0.5, vertex tolerance 0.005, initial probe 0.04,
least probe 0.01, near/far query budgets 4000/1000, cached poses 4096.

| Baseline case | Evidence before Phase 0 |
|---|---|
| Turning prism | Closed; old centroid gate reported no failures |
| Turned lens | Closed; old centroid gate reported no failures |
| Turned box | Closed; old centroid gate reported no failures |
| Tumbling cylinder | 6,713 triangles; seven open loops; volume sum 9.3057 |
| Sliding dumbbell | 35,370 triangles; two open loops, both size four; volume sum 9.5277 |
| Whole-turn box | Test passed with 230 triangles in the old `thin` category; volume 20.1557 |
| Perturbed cylinder | Both 1e-12 seed perturbations already differed, first at overlap clipping |

An open mesh's volume sum is diagnostic only. The baseline gate's success is historical
behavior, not a claim of correctness under the new contract.

## Contract changes

`Sign::Near` now carries a field-value bound rather than a spatial radius. A strictly signed
enclosure stays strictly signed, however small its magnitude. Value convergence never supplies
a boundary witness. The meaning of `deep_sign` remains one-sided exclusion: an insufficient
field margin does not prove that the point is near a boundary.

`BoundaryBracket` records actual material/exterior points and their strict interval enclosures.
Projection results carry these witnesses and derive their spatial radius using outward-rounded
coordinate arithmetic. Projection normalizes its direction. Ambiguous midpoint values cause
additional bracket refinement or an explicit unresolved result that retains the last bracket;
they never replace its width with a residual. Same-sign search results describe sampled search
points, not an exhaustive proof that there is no intervening feature.

The triangle report retains strict centroid-side brackets, failing/unknown outcomes and their
areas. Slivers use the same strict evidence requirement as other triangles. Thin material can
resolve at a shorter probe; exhausting the probe ladder does not prove thinness or license a
triangle. The old `thin` field is now named `unresolved`. Completing this report establishes
its point checks only; the spatial audit below is independently required for acceptance.

`candidate` and `candidate_from` expose the final mesh, open rims, topology result and triangle
report without asserting acceptance. The certificate observer runs on open candidates too.
`construct` and `construct_from` require acceptance. `validate` also accepts an independently
proposed mesh, but checks the actual mesh and field rather than trusting a supplied report.
The accepted `SweptBoundary` exposes immutable geometry and evidence through accessors.

## Spatial acceptance

The acceptance audit validates the existing mesh; it does not modify or contour it.

1. Topology establishes one connected closed oriented manifold, including vertex links.
2. Strict centroid-side checks must all resolve.
3. Each triangle is partitioned into dyadic subtriangles. Each accepted subtriangle has strict
   spatial witnesses and an outward-rounded bound from every point in its enclosing box to
   the witness segment. Their maximum bounds the mesh-to-material-boundary distance.
4. A complete partition of the material's construction-derived support is retained. Strictly
   signed cells contain no boundary. Every other terminal cell has a conservative distance
   bound to an actual barycentric point of the mesh. This proves reverse coverage, including
   components that the candidate omitted. A caller-supplied crop is not accepted as support.

Both directions must meet the stated tolerance. Spatial indexing accelerates witness search;
only interval bounds decide acceptance. Each audit has explicit depth and cell budgets, and
exhaustion refuses acceptance. Defaults use the existing probe distance as the spatial tolerance,
100,000 total visited cells/subtriangles, and depth 24. This is a bounded validation baseline,
not a claim of production-speed gear validation.

The accepted scope is the binary64 field snapshot and mesh. The audit does not establish
geometric embedding, topology equivalence to all features smaller than the tolerance, source
solve accuracy, or the accuracy/topology of a later float32 STL encoding. Those remain separate
gates in the subsequent phases. Multiple material shells are an explicit limitation of this
single-shell acceptance API; they are not deleted to make a candidate pass.

## Validation

The four initial negative controls were run before changing the library and all failed:
small union residual, `A - A` projection, buried facets/slivers, and a zero-set sliver.
Their old/new logs are preserved with the baseline. Additional controls cover actual thin
material, ambiguous bisection, a complete accepted cube, an unsupported triangle interior that
passes centroid checks, omitted disconnected material, topology, and spatial-budget exhaustion.

Commands use the optimized test profile, never `--release`:

```sh
cargo test --manifest-path rust/Cargo.toml -p gcs-core --test core sweep_mesh::evidence
cargo test --manifest-path rust/Cargo.toml -p gcs-core --test core
```

The new `sweep_mesh::status` tests actively assert the observed refusal categories and check
that complete candidate reports are delivered even when topology fails. Thirteen historical
success/repair assertions have been explicitly deferred with reasons, preserving their bodies
as future targets. They are not silently treated as passing. The normal suite now tests the
Phase-0 refusal contract; a green result does not complete those geometry milestones.

Final full core run: **1,223 passed, 0 failed, 93 ignored**, in 116.89 seconds.
This includes all 12 evidence controls and the active candidate-refusal regressions.
The ignored loop-drawing instrument also passed separately (6.65 seconds); its SVG and
rendered PNG are preserved under `after/`. `core-final.log` records the full run.

## Current candidate matrix

Fresh default-option candidate reports, including both cylinder perturbation phases:

| Case | Triangles | Open loops | Reversed checks | Unresolved checks | Acceptance barrier |
|---|---:|---:|---:|---:|---|
| Turning prism | 1,065 | 3 | 25 | 0 | Open topology |
| Turned lens | 2,818 | 0 | 23 | 27 | Non-manifold vertex |
| Turned box | 1,366 | 0 | 0 | 17 | Unresolved surface checks |
| Tumbling cylinder | 6,129 | 10 | 350 | 252 | Open topology and unsupported surface |
| Sliding dumbbell | — | — | — | — | Earlier `ReversedNormal` during construction |
| Whole-turn box | 1,374 | 0 | 74 | 0 | Reversed surface checks; topology reports genus 10 |
| Cylinder, perturbation 0 | 6,821 | 13 | 469 | 363 | Invalid edge use and unsupported surface |
| Cylinder, perturbation 1 | 6,385 | 15 | 373 | 318 | Open topology and unsupported surface |

The cylinder's total triangle area is 28.203018; reversed area is 1.298876 and unresolved area
is 1.205055. Counts describe centroid-side checks, not a measure of missing boundary area.
Its whole-surface and reverse-coverage acceptance gates remain unmet.

All four newly produced five-case STLs differ from the baseline; the dumbbell produces no new
final-candidate STL. These are intentional consequences of changing unsupported labels and
projection positions. The replacement surface construction of later phases has not been
implemented, and this work claims no improvement in mesh quality or perturbation stability.
The fresh files and `status.tsv` are in the baseline directory's `after/` subdirectory, with
hashes in `manifest.json`.

Reproduce the matrix without overwriting the checked-in example exports:

```sh
SOLVENT_EXPORT=/private/tmp/swept-boundary-phase-zero-check \
cargo test --manifest-path rust/Cargo.toml -p gcs-core --test core \
  sweep_mesh::status::phase_zero_status -- --ignored --nocapture
```
