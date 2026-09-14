# Phase 3 progress: source charts and trim construction

Subphase 3a, source connection, was committed as `092f2cf` after `f09fa19`. Subphase 3b,
trim construction, was committed as `9bf5cd9` and adds native-chart intersection candidates, isolated edge-contact roots and strict
hiding witnesses. Neither milestone completes the cylinder exit gate. Ordinary tracing retains
evaluable source coordinates and local contact-branch/event observations. The ordinary candidate
still uses the legacy cap, overlap and stitching construction. Phase 2's explicit atlas remains test
support; it has not been installed as a cylinder recognizer.

**Latest implementation:** [3c, carried-rim correspondence](swept-boundary-phase-three-c.md),
establishes the common carrier analytically for supported circular edges and coordinate-axis
rotations, retains interval-regular native maps, and supplies checked numerical inverses.
The original two-band case is resolved. Domain union is the next subphase, 3d.

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

## Source-connection validation (`092f2cf`)

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

## Trim-construction milestone

`solid::swept_boundary::arrangement` operates on the same solved source/motion snapshot.
It does not yet alter `candidate` or `construct`.

### Transverse trim candidates

`Pair` intersects two evaluable source charts within explicit native-coordinate boxes. One
of their four parameters is the curve coordinate; the shared bounded DogLeg adapter solves
three original position-equality equations for the remaining parameters. Each result retains
both native parameter pairs, source identities, roll values and separately evaluated positions.
It does not average positions or infer a source from a nearby mesh vertex.

Every root, including an exact initial seed, must pass the shared solver's rank check and a
sampled surface-normal transversality check. Coincident carriers, tangent supports, missing
station branches and out-of-domain trials cannot establish a trim curve. `trace` adaptively
samples midpoint chord error and native parameter steps on both consumers. Its completed and
unresolved spans partition the requested interval; budget exhaustion or failed solves cannot
create a chord across a missing span. This is numerical intersection and sampling evidence,
not a proof of branch uniqueness, continuous coverage, global exposure or spatial error.
The caller currently supplies chart pairs, parameter boxes, seeds and a continuation coordinate.

The cylinder's endpoint-wall intersection is constructed this way without its visibility
formula in production code. A test-only analytic section judges the result. With sagitta
`0.001`, tracing native `v` from `0.05` to `0.45` needs 65 root solves for 32 segments, with
maximum sampled midpoint error `0.000891` or less. Both original consumers agree within
`1e-11`. Perturbed seeds, exchanged consumers and half-roll angles 29.9, 30 and 30.1 degrees
produce the same intersection within the stated test tolerance.

### Isolated edge-contact events

`arrangement::edge::isolate` uses outward-rounded source-surface and motion bounds for the
original equation `G(t,roll) = n dot velocity = 0` on a declared incident face of a source edge.
Strict opposite endpoint signs and a derivative excluding zero isolate one edge-coordinate
root for **every** roll in the requested roll interval. The whole source interval must also
have a nonzero normal: a pole's vacuous zero is not a contact root. Interval Newton contraction
then encloses that same root. It retains the sign evidence, derivative, original domain, roll
interval, root enclosure and work count. Reaching the requested parameter width is reported
separately from having isolated the root; zero refinement budget keeps the wider enclosure.
The width comparison is outward rounded too.

This currently supports fixed-u/fixed-v edges of revolved faces. Other chart families, absent
brackets, possible singularities and nonmonotone boxes refuse explicitly. It is an isolator
within a supplied box, not a complete root cover or event discovery algorithm. In the cylinder
regression all four edge/incident-face combinations isolate the root at `t=0.5` across the
entire roll domain to width below `1e-12`. A root on the `t=0/1` seam cannot be strictly
bracketed inside one such box and remains refused; a periodic handoff must establish the alias
and outgoing branch before joining it. These roots are contact evidence, not edge eligibility,
trim adjacency or global visibility evidence.

### Global hiding witnesses

`arrangement::hiding` queries the existing continuous minimizer over the complete sweep roll
interval, then independently bounds the source field at its witness pose. Only a strictly
negative upper bound licenses a `Hidden` witness for the **entire supplied spatial box**.
The result retains the minimum enclosure, status and query count; confirming the witness
costs one additional source-bound query beyond the minimizer's count. A hidden center cannot
license discarding a box containing an exterior point. The cylinder test distinguishes local
endpoint contact from coverage by an intermediate pose.

No witness means unresolved, including when the budget runs out. It is not evidence of
exposure. Nor does a box witness classify a whole source-parameter cell unless that cell's
image has separately been enclosed in the box. Source-solve error transfer is still outside
the existing interval geometry contract.

### Barrier exposed: coincident carried supports

Different parameter bands of a carried cylinder rim can overlap over a two-dimensional
surface. They are not merely two transverse surfaces with an awkward intersection seed.
For the nominal fixture a rim point satisfies `(x-3)^2 + y^2 + z^2 = 2`; tumbling about the x axis
preserves this spherical carrier. Distinct generating edge parameters and roll values can
therefore reach the same surface region. The new counterexample gives the intersection solver
such an exact coincident point and requires refusal instead of manufacturing a trim curve.

The arrangement needs an explicit treatment of **coincident carriers and their covered
domains**, preserving each consumer's coordinates and multiplicity. Numerical rank deficiency
only identifies an unresolved case; it does not prove carrier equivalence. Establish an
analytic source/motion relation or a certified chart correspondence before arranging a union
on one carrier. Then split at the domains' own boundaries, classify coverage and retain the
chosen consumers with their maps. Proximity welding, dropping nearby triangles, or hard-coding
the Phase 2 cylinder atlas would evade this obligation.

The next focused construction experiment should arrange the two overlapping rim bands from
this counterexample before attempting the full atlas. Derive a carrier correspondence from
the source curve and motion, split at its folds and periodic seams, and retain each band's
covered parameter domain. The output must contain the union boundary and maps back to every
covering consumer. Test reversed consumer order and altered sampling seeds against the same
analytic rim geometry; an incomplete correspondence must remain unresolved. Only then combine
this result with the transverse endpoint trims and contact events. This gives the next step a
specific geometric output and refusal control, rather than another mesh-repair heuristic.

### Trim-construction validation

The six `sweep_mesh::arrangement` tests cover source incidence, the independent endpoint-wall
section, seed/consumer/roll perturbations, coincident-support refusal, partial trace coverage,
work limits, uniform edge-root isolation, seam refusal, and whole-box hiding versus unresolved
searches. Run:

```
cargo test --manifest-path rust/Cargo.toml -p gcs-core --test core sweep_mesh::arrangement -- --nocapture
cargo test --manifest-path rust/Cargo.toml -p gcs-core --test core
```

Final validation results:

- Focused arrangement suite: **6 passed**, zero failed.
- Full core suite: **1,257 passed, zero failed, 100 ignored** (157.04 seconds).
- `cargo check --manifest-path rust/Cargo.toml -p gcs-cli --features occt,manifold`: passed.
- The ignored `sweep_mesh::status::phase_zero_status` exporter passed (102.78 seconds),
  writing diagnostics to `/private/tmp/swept-boundary-phase3-trim-exports`. All seven STLs and
  `status.tsv` are byte-identical to `exports/` in the unchanged Phase 1a replay archive;
  its SHA-256 remains `914b94111651a4efcaa3249614fd2731e3c87ef9c24ba8abd509af1461e7fb7c`.
  The dumbbell retains the same construction refusal. These are diagnostic candidates,
  not newly accepted exports.
- `git diff --check` and whitespace checks on the three new Rust files: passed.

The full suite includes the source-chart regressions and shared-domain tests. The cylinder
production acceptance gate is still deferred; passing these tests does not complete Phase 3.

## Remaining subphases

The [Phase 3 subphase plan](swept-boundary-phase-three-plan.md) now owns the detailed remaining
sequence and completion criteria. This document remains the implementation and measurement
record. The source-connection and trim-construction milestones above are **3a and 3b**, complete.
[3c is also implemented](swept-boundary-phase-three-c.md), establishing a checked correspondence
for the two carried-rim bands within the documented geometric support.

**Next is 3d:** construct their domain union with all consumer maps. Shared carrier identity
does not by itself construct that union or prove visibility.

Subphases **3e–3h** discover the complete event cover, connect the trim network, classify
visibility and assemble an oriented atlas compatible with the shared tessellator. **3i–3k**
integrate ordinary construction, pass the cylinder's geometric gates and validate the actual
encoded float32 mesh. **3l–3m** extend the same mechanism to the broader tool/motion matrix and
multiple-shell or topology-changing cases. Hypoid CLI/Boolean integration remains Phase 4.

The ordinary cylinder is still the Phase 1a candidate (6033 triangles, 13 open loops;
5473 centroid brackets, 324 failed and 236 unresolved). The implemented utilities do not improve
that mesh until the arrangement owns its domains and is connected to production construction.
