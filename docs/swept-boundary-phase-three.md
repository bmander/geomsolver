# Phase 3 progress: reconnect tracing to source charts

Implementation after `f09fa19`. This is the source-connection milestone of Phase 3,
not completion of its cylinder exit gate. Ordinary tracing now retains evaluable source
coordinates and local contact-branch/event observations. The ordinary candidate still uses
the legacy cap, overlap and stitching construction. Phase 2's explicit atlas remains test
support; it has not been installed as a cylinder recognizer.

## Source connection

`solid::sweep_source` exposes snapshot-local face, charted-edge and Boolean-crease identities.
Every traced characteristic point carries its original native coordinates, recorded when the
station equation or edge evaluator produced it. Station roots also retain their local branch
index. These observations survive source clipping, chaining, reversal, rotation of a closed
curve, simplification and both row-major carried sheets and column-major variable-motion sheets.

A chained joint retains every consumer's original observation. The old chain can displace one
consumer onto another's point; retaining both does not establish coincidence. The provenance
API can reevaluate every observation and report its residual to the current patch vertices.
Moving a public patch vertex does not change the immutable source snapshot or its coordinates.
No metadata is reconstructed with a closest-point lookup.

Each emitted sheet shares an owned `SweepContacts` snapshot with the other sheets from that
tracing call. Face/edge indices are identities within that snapshot, not persistent names across
model edits or source reordering. The snapshot owns the solved source geometry and motion and
remains available after the sketch or original contacts object is dropped.

Candidate chart evaluators cover:

- Source face carriers at either roll endpoint, in native `(u,v)`.
- A smooth-face station root, in `(u,roll)`, using the original station equation.
- A stationary station, in `(v,roll)`, checking the normal-velocity equation at each query.
- A charted edge or Boolean crease carried through `(t,roll)`.

Edge evaluation retains both incident faces' own coordinates and positions. Boolean crease
evaluation uses the existing crease solver and may refuse. Neither an endpoint carrier nor
an edge sweep asserts material visibility; source trims and global visibility must still be
arranged. Station branch indices are local equation roots, not certified continuation through
a fold or periodic seam. Invalid coordinates, missing branches and failed contact evaluations
return errors rather than extrapolating a polygonal strip.

Fold endpoints and poles retain their coordinates with no regular chart continuation. The
tracer also records numerical continuation limits, edge normal-velocity band crossings and
edge eligibility limits. These are candidate event observations, not isolated topological trim
vertices. In particular, an edge fan currently ends at a normal-velocity tolerance threshold;
that is not an exact shared boundary with its adjacent face contact sheet.

Legacy faceted caps and planar grazing-region unions explicitly have no native provenance.
They must be regenerated from source domains when their responsibilities move into the new
constructor. Labeling their interpolated vertices with guessed native coordinates would
reintroduce the same architectural problem.

## Independent validation controls

`SweptBoundaryOptions` now independently exposes `field_value_tolerance` and
`minimum_probe_distance`. Omitting them preserves the old defaults derived from
`vertex_tolerance`. Construction spacing, vertex tolerance and geometric trimming tolerances
are unaffected by either explicit override. Changing field precision can still affect a legacy
construction's field decisions; it is not a promise of identical output under changed options.
An optional `spatial_audit` supplies the requested spatial tolerance and budgets to ordinary
`construct`, so its acceptance step need not use the fixed default audit budget.

The Phase 2 calibration is expressible without changing constructor tolerances:

```rust
let options = SweptBoundaryOptions {
    field_value_tolerance: Some(0.000005),
    minimum_probe_distance: Some(0.00002),
    spatial_audit: Some(AuditOptions {
        tolerance: 0.04, max_cells: 800000, max_depth: 36, box_budget: 1000,
    }),
    ..Default::default()
};
```

These settings do not fix the legacy candidate's unsupported geometry. No acceptance condition
has been weakened, and the ordinary cylinder success test remains deferred.

## Validation

The focused `sweep_mesh::source_charts` tests exercise original geometry reevaluation across
cylinder, sphere, box and Boolean-lens sources; both sheet layouts; intermediate native
parameters and roll; invalid requests; and detection of edits to public patch vertices.
Cylinder observations at spacings 0.35, 0.5 and 0.7 reproduce source positions with maximum
residual about `2.0003e-9` model units. This is a numeric provenance diagnostic, not a spatial
surface certificate or a claim that the old chained geometry is correct.

The nine cylinder sheets' geometry fingerprints are derived from
`seeds-and-label-witnesses.txt` in the unmodified Phase 1a archive, SHA-256
`914b94111651a4efcaa3249614fd2731e3c87ef9c24ba8abd509af1461e7fb7c`.
They include vertices, normals, triangles, columns, times, closed flags and array counts;
new provenance is excluded. This checks that retaining metadata does not silently change
construction geometry.

The calibration regression runs the unchanged strict centroid checks on the Phase 2 mesh.
The defaults leave 24 triangles unresolved. Changing either new control alone resolves none
of those 24; changing both resolves all 24, with the original mesh and construction settings.
This repeats the discriminating calibration control, not the complete Phase 2 spatial audit.

Run:

```
cargo test --manifest-path rust/Cargo.toml -p gcs-core --test core sweep_mesh::source_charts -- --nocapture
cargo test --manifest-path rust/Cargo.toml -p gcs-core --test core
```

Validation results:

- Full core suite: **1,251 passed, zero failed, 100 ignored** (149.18 seconds).
- Focused source-chart suite: **5 passed**, including the frozen tracing fingerprints and
  separated calibration controls.
- `cargo check --manifest-path rust/Cargo.toml -p gcs-cli --features occt,manifold`: passed.
- The ordinary `sweep_mesh::status::phase_zero_status` exporter completed. All seven candidate
  STLs and `status.tsv` are byte-identical to the Phase 1a archive. This includes the prism,
  lens, box, cylinder, whole-turn box and both cylinder perturbations. The dumbbell retains
  its earlier construction refusal. This is the current diagnostic successor to the historical
  five-case exporter; it does not misrepresent refused candidates as accepted exports.
- `git diff --check`: passed.

The featureless CLI compile also attempted here fails at the pre-existing unguarded
`cad::mesh_sweep::VERBOSITY` references in `main.rs:178-179`; `mesh_sweep` requires both CAD
features. The same references and feature guards are present in `f09fa19`. No CLI code was
changed by this milestone.

## Next construction milestone

Build the global visibility arrangement from these source charts and the solved motion:

1. Resolve candidate events and contact branch continuations, including periodic seams,
   folds and edge/face handoffs. Preserve unresolved event searches explicitly.
2. Intersect endpoint carriers and swept supports, split their native domains at the
   resulting trim curves, and determine exposure against the complete sweep. Trace
   continuity and a local normal-velocity root alone cannot determine global exposure.
3. Assign each supported common trim curve one identity and parameter sequence, retain its
   coordinates on all consumers, and derive adequate directional refinement requests.
4. Feed the arranged domains into the existing shared tessellator. Reproduce the Phase 2
   cylinder through ordinary construction and check actual tracing-seed perturbations.
5. Complete the cylinder gate: interval acceptance in both distance directions, topology,
   intersections, independent section/volume checks, and encoded float32 export validation.

The source connection removes the need to infer geometry from triangles. It does not remove
the need for an arrangement solver. That distinction is the remaining architectural boundary;
copying the Phase 2 cylinder's visibility formulas into production would bypass it.
