# Analytic face boundaries

A spatial face now uses the existing `face` declaration with an explicit support:

```sv
face working(toe, tip, heel, join, on: flank)
face transition(round_toe, join, round_heel, root, on: fillet)
```

Every boundary operand is a named finite `edge`. `on:` names the exact surface,
envelope or material patch that contains those edges. The ordered loop determines
the directed uses. Here the working face traverses `join` in reverse and the transition
traverses it forward; both keep the same edge and corner identities.

The spiral-bevel source packages this pattern as `ToothSideFaces`, called once for each
of its four tooth sides. It declares eight supported faces. The rim's toe and heel
contours now read those face boundaries, including their correct support parameters,
rather than addressing edge names separately.

## Declaration and ownership contracts

`FaceSupport` distinguishes `Plane(Option<plane>)` from `Surface(entity)`. A spatial
support cannot masquerade as a profile on the page. All existing planar readers request
`FaceE::plane()` and fail for spatial faces; planar sweep construction also refuses them.
The support is an ordinary dependency, reached as `face.on`. Copy/delete and private
component members follow the same rules as other entity references.

Planar profiles are built before primitive solids. Spatial faces are built after
surfaces, envelopes, material patches, seams, vertices and edges. Their provisional
names cannot address a planar profile's index while they await construction. Copying
uses the same dependency order. Forward declarations, typed component formals and
canonical printing preserve the support and ordered edge list.

The boundary reader requires at least two distinct edge identities. Each edge's seam
must explicitly name the face's exact support as one of its operands; a separately
declared coincident support is insufficient. Edges must close in the written order,
with each vertex visited once before returning to the start. The reader does not weld
coordinates or invent closure edges. For a two-edge loop, both directions can close;
the first edge's declared direction chooses the traversal. Spatial holes, repeated
periodic edge uses and `-> close` are currently refused.

## Checked geometric samples

`spatial_face::SpatialFaceBoundary::named` reads the support and its finite edge
snapshots, taking canonical vertex witnesses indexed by spatial vertex ID and explicit
`EdgeTolerance`. Only required witnesses are read. It checks the loop and every endpoint
against this particular support. The resulting snapshot is immutable; source changes
require a new read.

`sample_boundary(edge_slot, fraction, max_iterations)` follows the directed use in
the loop. It delegates curve evaluation to the finite edge, preserves that edge's xyz,
then checks incidence on the face. The returned parameters belong to this support:

- A generating junction converts `u` into the second face's endpoint chart when needed.
- An envelope sample satisfies the envelope equation and material conditions.
- A surface sample uses its finite projector and bounded surface parameters.

The returned incidence error is the distance between the shared edge position and the
support position. The face uses the explicit incidence tolerance; a coarser junction
coincidence tolerance does not grant finer support membership. Reverse fractions are
validated before subtraction, so a tiny negative input cannot round into an endpoint.

The browser exposes `Document.faces()`, `spatialFaceLoop` and `faceBoundarySample`.
The host supplies witnesses and marshals buffers; it does not implement surface geometry.

## Verification and remaining assembly work

The elementary fixture is shared by the Rust and browser tests:
`rust/gcs-core/tests/fixtures/spatial_faces.sv`. It supplies independent circular-section
equations for boundary samples and a common junction whose two face charts use `u=1`
and `u=0`. Tests cover reversed loops, exact shared endpoints, chart conversion,
copy/paste/delete, support paths, privacy, forward construction, stale snapshots,
invalid witnesses and refusal of spatial faces as planar sweep profiles.

All eight gear faces are exercised in each of the nine tooth-count/size configurations:
1,440 endpoint/interior boundary samples agree with the independent generating equations
in the correct support chart. Every loop closes at exactly shared corner positions;
each flank/fillet join has opposed edge uses.

These declarations and boundary samples do not certify a disk interior. A deliberate
counterexample uses two distinct named edges tracing the same arc in opposite directions:
its loop has valid boundary incidence and no area. The boundary reader accepts the
incidence, and the tests keep that distinction explicit. It cannot be exported as a
valid spatial solid through the current planar sweep path.

The completed gear solids still require analytic interior selection, tip/root and
toe/heel/back face assembly, tooth indexing with shared identities, closed-shell
integration, source-error transfer, global interference checks and tolerance-controlled
export. The [nominal regularity proof](interval-geometry.md) and
[closed-shell topology checker](shell-topology.md) address different parts of that work.
