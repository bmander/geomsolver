# Analytic surfaces, envelopes, seams and faces

Part of the [contract](../../CLAUDE.md) the code is held to. Read it when you touch `envelope/`,
`patch.rs`, `seam.rs`, `interval.rs`, `topology.rs`. An issue number (#n) or a spec version ([0.n],
see the [language changelog](../solvent-changelog.md)) says where a rule came from.

**Analytic surface references:** `flank := surface(crown, edge: rack.outer)` names a
line/arc/circle patch of an unmodified revolution: spatial, parameter-free, not 2D.
`solid::RevolvedSurface::named` snapshots it; re-read after model changes. `u` follows the edge,
`v` the revolution, so tangent orientation is not an outward normal. Browser `core/surface.ts`
(position, `du`, `dv`: nine doubles). `RevolvedSurface::projector` / `SurfaceProjector::named`:
`project` gives an oriented meridian residual, normal, support parameters, a bounded patch point
and incidence error. The residual is no global distance and cannot certify finite-patch
membership; use incidence error (`surfaceProjection`, `gcs_surface_project`). `line_on_sphere`
returns every finite root. `from:` / `to:` Angles restrict the `v` chart without renumbering or
reversing normals; read `RevolvedSurface::domain` / `GeneratedEnvelope::domain` (`surfaceDomain`
/ `envelopeDomain`), never assume [0,1]. Spans and roll bounds share `AngularSpan`; neither is a
hint or unknown.

**Named envelopes:** `flank := envelope(source, under: generating, from: -35deg, to: 35deg)`: a
zero-normal-velocity locus over a finite increasing roll interval (ABI kind 11).
`GeneratedEnvelope` intersects it with two section equations via the shared DogLeg loop. A nonzero
trial residual is off the envelope; a local root certifies no global regularity or material side.
`TrustRegion::restrict_step` (default no-op) limits steps to the box; all equations and rank are
still checked. Browser `core/envelope.ts` (10 doubles). `intersect_boundaries` checks two
patches' finite incidence and refuses roots on undeclared continuations.

**Trimmed spatial patches:** `flank := patch(source, inside: tip, outside: root)`; repeat the label
per clipping solid. `patch::TrimmedPatch` has explicit axis, trim and residual tolerances; clips
unmodified full revolutions (line/arc/circle profiles, holes). `solid::RevolvedRegion` is analytic
meridian membership apart from the CSG kernel. Mind full-circle seams, arc shared vertices,
vanishing axis edges; never turn a sphere's construction diameter into a wall. The final root must
satisfy trim and envelope. `core/patch.ts`; ABI kind 12. No branch, face loop or closed solid yet.

**Whole-interval geometry bounds:** `interval::Interval`: private finite endpoints,
outward-rounded arithmetic; zero-straddling division, overflow and invalid domains fail closed.
`sin_cos` is Taylor with remainders on [-8,8], not libm. `generating_profile_bounds`
(`generatingProfileBounds`) bounds profile and du; `generating_profile_jet_bounds` also duu. The
circular-crown verifier and its rational checker establish nominal differential regularity, not
source-error transfer, injectivity or interference (`docs/interval-geometry.md`).

**Geometry validation contracts:** `docs/geometry-validation.md`; the spiral-bevel
[roadmap](../spiral-bevel-roadmap.md). `interval::minimum::enclose` refines with an enclosure
oracle; exhausted budgets keep bounds, never guess membership. `Family::bounds` / `MotionBounds`
enclose mathematical poses and point boxes (coefficients private), not float evaluation or
source error; past [-8,8] angles use `Interval::sin_cos_periodic`. `solid::PlanarField` /
`RevolvedField`: material is closure({f<0}); Boolean fields are one-Lipschitz, not signed
distances, and zero alone is no boundary (A minus A is the negative control). See
`docs/continuous-volumes.md`; `indexed_full_crown_union_exposes_neighboring_gear_overcut` keeps a
rejected candidate. `SpatialField` memo keys include node identity and the whole box; shared DAGs
must not expand exponentially or conflate transformed coordinates; spatial depth is bounded apart
from planar. `solid::SweptField` owns a one-Lipschitz source, a motion and a roll domain; its
`SweepEvaluator` has a capped pose cache. Evidence observers cannot define geometry or prune; do
not reintroduce a callback-based oracle. `SpatialField::read` takes prisms
(`depth:`/`from:`/`to:`/`through:`), full revolutions, bodies and placements; `SweptField::read`
binds a solid and motion. `solid::ExtrudedField` is the prism leaf (interval Gram–Schmidt frame);
`through:` spans `support_bounds` (unknown refuses). Profiles go through `PlanarField::from_loops`
(`solid/field/profile.rs`, exact, no tessellation): `ordered` joins within the larger of roundoff
and axis tolerance, `turning` checks one turn and convexity; convex loops are half-planes and
sectors, others `Node::Profile` (ray-parity sign only when the box clears every wall, else
`[-d, d]`). Partial revolutions and lofts refuse.

`SolveOpts::acceptance_tol` controls hard-row success and the DogLeg-to-LM retry; `tol` controls
stopping; defaults keep interactive behavior. Analytic callers request accuracy instead of
switching optimizers, and still check geometric errors. `GeneratedEnvelope::evaluate` returns
trials; `at` requires the envelope equation. `EnvelopePatch` owns one source with its material
conditions; do not reintroduce parallel untrimmed and trimmed copies of a generating face.
Snapshot domains are private, returned by value; source edits require new snapshots.

**Shared generating seams:** `join := seam(flank, transition)` takes two envelopes or patches of
envelopes sharing one actual profile vertex, a source revolution and a named motion.
`seam::EnvelopeSeam` snapshots them after solving, checks coincidence and tangent-plane agreement
with explicit `SeamTolerance`, and intersects their angular domains; first-source u is fixed at the
junction. `SeamIntersectionOptions` exposes only [v,roll]; `intersect` solves both envelope
equations and one section through the DogLeg adapter (one normal-velocity tolerance for both
faces), and the final root checks both envelopes and material trims. Not a nearest-point weld or
an arbitrary surface intersection. Local singular intersections are refused. Browser
`core/seam.ts`; ABI kind 13. Contacts carry the first face's normal and generating velocity, not
the seam's curve derivative.

**Envelope/boundary seams:** `tip_edge := seam(flank_region, tip.wall)` names an envelope (or its
patch) cut by a finite analytic surface. `seam::BoundarySeam` reads one `EnvelopePatch` and one
`SurfaceProjector`, keeping the full [u,v,roll] domain; it solves envelope, boundary support and
section equations in that order, then checks finite incidence and trims. A separate type from
`EnvelopeSeam`: three unknowns, no junction tangency tolerances. Browser `boundarySeamDomain` /
`boundarySeamSample` in `core/seam.ts`, kind 13. Reading a seam asserts no regular branch or
oriented face loop.

**Shared spatial vertices:** `corner := vertex(tip_edge,toe_edge)` meets two boundary seams on one
exact named generating face; a generating junction may also meet a boundary seam on either face.
Coincident declarations keep distinct identity. `BoundaryVertex` owns one face and two boundaries
(three unknowns); `JunctionVertex` one junction and one boundary (two), checking finite incidence
on the face the boundary seam names — a coarser seam tolerance cannot transfer finer boundary
membership. `position` is xyz only, no artificial corner normal. Local solves check rank, not
global uniqueness. Ordinary formals/privacy/copy; ABI kind 14, `core/vertex.ts`.

**Finite spatial edges:** `extent := edge(seam,from: a,to: b,along: axis)` names a finite axial
slicing interval between spatial vertices. `edge::SpatialEdge::named` rechecks endpoint witnesses
and maps junction endpoint u into the correct incident-face chart. At a junction a boundary on the
other incident face shares the corner through boundary identity; never infer this from proximity
or require duplicate corners. `sample` solves via seam APIs, checks local rank/material/incidence
inside, and reuses stored endpoints checked by their full vertex constraints. Never re-solve a
known endpoint with a potentially tangent slicing equation. Fraction is linear projection along
the line, not arc length; local samples are no global branch certificate. No planar coordinates or
glyph; ABI kind 15, `core/edge.ts`.

**Checked shell topology:** `topology::ClosedShell` owns explicit edge endpoint identities and
ordered directed uses in connected faces with boundary loops; construction checks closure, two
opposed uses per edge, one circular link per vertex and face connectivity (periodic self-uses and
annular faces supported). Separate from planar `FaceE`; no coordinates or tolerance weld.
`from_triangles` uses index identity. `mesh::stl_topology` verifies the encoded float32 coordinates
(exact equals, signed zero normalized). Neither establishes incidence, non-self-intersection,
outward orientation or deviation error. See `docs/shell-topology.md` before connecting faces.

**Spatial face boundaries:** `working := face(toe,tip,heel,join,on: region)` binds an ordered
finite-edge loop to an exact named surface/envelope/patch. `FaceSupport` distinguishes inherited
planes from spatial supports; `FaceE::plane()` refuses the latter, never treating them as page
profiles. Spatial faces build/copy after edges, `on:` an ordinary dependency.
`spatial_face::SpatialFaceBoundary` validates shared vertex identity and maps junction parameters
to this face's chart; samples keep edge xyz and check this support's incidence. No holes or
repeated vertices/edges yet; solid assembly still needs an interior/embedding certificate. Browser
`spatial-face.ts`. See `docs/analytic-face-boundaries.md`; a closed boundary walk is not a valid
disk.
