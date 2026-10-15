# Analytic face boundaries

A spatial face boundary is an ordered loop of finite edges on an exact analytic support, made
in Rust of its parts:

```rust
let working = SpatialFaceBoundary::new("working".into(), Support::Envelope(flank.clone()),
    vec![toe, tip, heel, join], tolerance)?;
let transition = SpatialFaceBoundary::new("transition".into(), Support::Envelope(fillet.clone()),
    vec![round_toe, join_on_fillet, round_heel, root], tolerance)?;
```

Each `Use` is a finite `edge::SpatialEdge`, the identity the caller gives it, which way the loop
walks it, and — where the edge's seam is a junction whose chart is the other face's — this face's
`u` at the shared vertex. The support is a trimmed patch of an envelope (`Support::Envelope`) or a
revolved surface (`Support::Surface`). Here the working face traverses `join` in reverse and the
transition traverses it forward; both keep the same edge and corner identities.

This is Rust API, not Solvent (#364). Through spec 0.119 the document declared these objects —
`patch`, `seam`, `vertex`, `edge(…, along:)` and `face(…, on:)` — and a `::named` reader read each
after the solve. They added no unknown, no solve row read them and nothing exported from them, so
the language dropped them; the drawing names only the surfaces and envelopes they are made over.

The spiral bevel's checks (`gcs-core/tests/envelope/paired.rs`) make this pattern of its parts
(#212): each member is built at one tooth space, its faces named by the crown faces that made them
(`BodyFaces`), and each of its four tooth sides' working and transition faces, with their seams,
corners and edges, is made where the built member says those faces meet, of the analytic envelopes
the faces' names say (`faces::face_source`; `SpatialFaceBoundary::new` and its peers) — eight
supported faces. The rim's toe and heel contours read those face boundaries, including their
correct support parameters, rather than addressing edge names separately.

## Identity and ownership contracts

Identity is the caller's. A corner is one corner because the caller gives the edges meeting there
one corner identity, never because two positions coincide; an edge is shared by two faces because
both are given the same edge. Nothing welds coordinates or invents closure edges.

`SpatialFaceBoundary::new` requires at least two uses. The loop must close on the edges' own
corner identities in the written order, each use's end the next use's start, the last returning to
the first. Holes are not supported. Every use's ends are then checked on this particular support.
The resulting snapshot is immutable; a change to the drawing requires its parts made again.

A `FaceE` — what a document's `face(…)` makes — is a planar profile only, its `plane` the one its
edges' points are drawn in. No spatial support can masquerade as a profile on the page, because
none can be written.

## Checked geometric samples

`sample_boundary(edge_slot, fraction, max_iterations)` follows the directed use in
the loop. It delegates curve evaluation to the finite edge, preserves that edge's xyz,
then checks incidence on the face. The returned parameters belong to this support:

- A generating junction's `u` is the second face's endpoint chart where the use says so.
- An envelope sample satisfies the envelope equation and material conditions.
- A surface sample uses its finite projector and bounded surface parameters.

The returned incidence error is the distance between the shared edge position and the
support position. The face uses the explicit incidence tolerance; a coarser junction
coincidence tolerance does not grant finer support membership. Reverse fractions are
validated before subtraction, so a tiny negative input cannot round into an endpoint.

## Verification and remaining assembly work

`rust/gcs-core/tests/seams/vertices/faces.rs` builds two faces over the envelopes of a revolution's
two tangent sides, their edges cut by spheres, from a small Solvent drawing that names only the
surfaces, envelopes and spheres. It supplies independent circular-section equations for boundary
samples and a common junction whose two face charts use `u=1` and `u=0`. Tests cover reversed
loops, exact shared endpoints, chart conversion, stale snapshots, the wrong support and open loops.

All eight gear faces are exercised in each of the nine tooth-count/size configurations:
1,440 endpoint/interior boundary samples agree with the independent generating equations
in the correct support chart. Every loop closes at exactly shared corner positions;
each flank/fillet join has opposed edge uses.

These boundary samples do not certify a disk interior. A deliberate counterexample uses two
distinct edges tracing the same arc in opposite directions: its loop has valid boundary
incidence and no area. The boundary accepts the incidence, and the tests keep that distinction
explicit.

The completed gear solids still require analytic interior selection, tip/root and
toe/heel/back face assembly, tooth indexing with shared identities, closed-shell
integration, source-error transfer, global interference checks and tolerance-controlled
export. The [nominal regularity proof](interval-geometry.md) and
[closed-shell topology checker](shell-topology.md) address different parts of that work. An
automatic derivation of a built solid's seams, corners and edges from its shared face adjacency
(`face_source` with `BodyFaces`) is the follow-up the declarations stood in for.
