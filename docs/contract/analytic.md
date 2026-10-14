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

**Envelopes the solve sees (#283):** every envelope in space is built with the drawing — after the
motions, before any relation (`program::envelopes::held_envelopes`) — carrying what the solve reads
of its source (`model::Held`): a revolution's side read off the declarations (`Held::Revolution`,
its `about:` line or axis and its line, arc or circle edge; the side itself bound after the solids
into `EnvelopeE::surface`), or a set's (`Held::Set`: the hidden point `{envelope}#probe` the
flattener applied it to, held; its `SetShape` read off the one relation the body states there by
`set_shape` — `Angle3` seen from a line's start a cone, `PointLine3` a cylinder, an `Ordinate`
along a line from a point a plane (`SetKind::Plane`, `SetShape::origin` that point, the number
signed toward the probe, #326) — never a row; an
unknown its number reads is bound for the contacts, `expr::bind_free` and `evaluate`). One whose
motion or roll is unreadable is left to the late pass, its name dropped so no relation holds to a
wrong index; one whose source is unreadable is `Held::Not(why)`, evaluated after the solve as before.
`envelope::held` is the evaluator: the moving point at a place `[u, v, t]` over `kernels::Num` in
second-order Taylor arithmetic (`H2`), the source a revolution of a meridian element (`Source`:
line, cone, cylinder, round) or a plane (`Source::Plane`, columns `[a, b, o]`: the point `o + x·d`
moved `u` and `v` along `across(d, ref)` and `d ×` it — no turn, both lengths; the seeds' grid
samples `v` as `u` there, `held::turn_spans`, and reaches from `o`, `held::reach`) and the motion the steps `motion::Family` reads (rotations, screws,
translations, relatives; a planar turn and a measured number refused), so a residual is exact and
a Jacobian exact by forward differentiation (`kernels::Dual`, eight columns a pass). `Parts::of`
reads the columns (lifts, an axis's place and direction, a free number) and constants (`Layout`,
`HEADER`, `STEP`); a set's turn starts at `SetShape::reference`, read once — before the solve
places the axis, so a reference within 30° of the axis as solved gives way to the first world
axis the axis does not nearly run along (`held::across`, #283), or which way rounding tips it
moves every place a quarter turn. The kinds
`PointOnEnvelope` (four rows, three owned), `EnvelopeTangentAt` (one), `EnvelopeNormalAt` and
`EnvelopePlaneAt` (two, each its own `Contact`) and `EnvelopeCurvatureAt` (three: Meusnier,
`x + (n − (n·m)m)/κₙ`, `κₙ` from `−dn·dX` along two directions of the place square to `∇g`) are
built at their widths (`KernelKey::Envelope`); the four with `at:` share the contact's place
(`solver.md`, #251). Seeds: `held::seed`, a coarse search
over the roll interval then `newton::dogleg` on the four rows (`Seeding`), remembered across a
contact's three slots. A set's
envelope has no chart after the solve (`GeneratedEnvelope::named` refuses it; `program::solid_diagnostics`
skips it); an envelope's `surface` and `motion` are read by name (`resolve::follow`), the surface
off the declaration until bound. A plane's envelope under a slide along its normal relative to a turn is the member's involute
helicoid: `tests/phillips.rs` holds it to the helicoid's own equations, and Phillips' pair
(`spiral_bevel/phillips.sv`) to a point contact on the line of action with no lag, a pair whose
racks slide at different speeds to the lag that closes their gap (`δ = (σ/σ' − 1)φ`), and the pair
misaligned after its flanks are made — the gear half a millimetre further off, or its shaft
turned half a degree — to a constant lag (−0.307° and 0.0066° of the gear: Phillips' claim, which
holds). `tests/held_envelopes.rs` is the gate — the frame against
`GeneratedEnvelope` to 1e-12, held points against the snapshot and `motion::Family`, the normal,
tangent and osculating circle against closed forms and a three-point section fit under a skew
screw mesh, the mean-point lean (`spiral_bevel/mean_point.sv`) — with
`jacobians::envelope_contacts_agree_with_finite_differences`.

**The meshing of two flanks (#283 step 3, [0.94]):** `contact := meshing(a, b, under: mesh, from:,
to:)` is `MeshingE` (ABI kind 17): two `Flank`s — `Envelope(i)`, a held envelope, or `Formed(Held)`,
a source read as an envelope's is (a set applied to its hidden point `{contact}#first#probe` /
`#second#probe`, `set_shape` through `program::envelopes::Reader`; a revolution's side) — the
motion and the phase, built with the held envelopes (`held_meshings`), refused (E080) where a flank
or the motion is unreadable or the motion is not relative. A flank may also be `Traced(i)` (#326,
[0.107]): a surface traced in space, `meshing::Side::Traced` over `traced::flank_consts` (`[0,
traced::FLANK, domain×4, contact constants…]`), its own columns the body's `θ`
(`FlankParts::Traced`), its place `[u, s, t]` with `t` held as a formed flank's; `traced::seen`
reads its point, `C_u` and the unit normal `C_u × C_s` over any `Num` — the frame's exact
derivatives linearised into the numbers' own (`Num::moves` false for a plain number skips them),
all a `Dual` pass reads — and `traced::nearest` seeds its place. `envelope::meshing` is the
evaluator: the first flank turned with the motion's observer at the phase `φ`, the second with its source at
`φ + δ` (`held::frames` at two rolls), each flank `held::local` over its own constants (a formed one
`Layout::read_any`, no step, its pose the identity); rows the two points' offset, the pinion's
normal against two directions square to the gear's (each times the lever, the contact's distance
from the motion's first axis point, so a length), and each flank's own row — its meshing equation,
or a formed flank's roll times the lever — seven at a phase, three more for `PointOnMeshing`'s point
(lead its lift; columns `[lead, φ, δ, place₁, place₂, flank₁, flank₂, motion]`, constants
`[lead, n₁, n₂, steps, flank₁, flank₂, steps]`; `KernelKey::Meshing`, exact by `Dual`). Its
Jacobian is taken **a block at a time** (#302): each flank's `local` against its own place and
columns, the two poses against the phase, the lag and the mesh's axes (`Mesh::flank_cols`,
`pose_cols`, no column in two), each differentiated over its own columns only, then the rows over
the blocks' values and derivatives by the chain rule (`assemble`, `write`) — so a flank is
differentiated in the passes its own columns need, not in every pass over the contact's (`bench
meshing`); the residual reads the same `assemble`, nothing packed. The phase and the lag are degrees, the places as an envelope contact's. `Meshing::at` solves a phase by
`newton::dogleg` with minimum-norm steps, a rank lost (`Sample::line`) a line contact; `trace`
walks out from the anchor (the first `PointOnMeshing`'s solved place), predicting by the last two
and halving a step the solve cannot take (`reach`); `seed` places a point's place (`held::seed` or
a coarse grid on a formed flank, then the phase solved), remembered by `infer::meshing_place`.
`report::meshing_text` / `meshings_json` are the CLI's line and `--json`. What it certifies is
local: no tooth pair, flank side, trimmed face or first touch elsewhere. `tests/meshing.rs` is the
gate — a conjugate pair's lag zero and line contact, a pair cut at another ratio lagging `(q − 1)φ`
exactly, a larger cutter at a modified roll touching at a point on the common generator, a cutter
turned about the normal at M lagging by the turn that closes the gap between the members (found
from the cone's equation and the poses alone), a stated lag solving its phase, `contact.sv` — with
`jacobians::meshing_contacts_agree_with_finite_differences`; and `tests/phillips.rs` holds
`spiral_bevel/phillips_pure.sv`'s traced flanks to the rack's envelopes (each a point of the
envelope at the element's phase, to 1e-9), to the closed form's point contact along `L` with no
lag, and their meshing's Jacobian to finite differences.

**A point a motion carries (#326, [0.104]):** `Reading::Carried` is `X − Pose(τ)·x`, the pose
`held::frames`' last at `τ` degrees — any motion, its twist constant or not (`Reading::twist`
false). `held::Step::Turn` (code 4, nine columns: a centre's lift, its view's `u` and `v`) is a turn
about a point of a view as it reads in space; only `Moving::of_any` reads one.

**A motion's screw (#326, [0.103]):** `envelope::screw` reads a motion as its twist, `v(x) = v₀ +
ω × x` a radian, constant in the fixed frame for a turn, a screw or a slide about a fixed line
(`own`: `ω = ratio·d`, `v₀ = a × ω + d·advance/τ`) and one relative to another (the source's less
the observer's) — `constant` refuses anything else. `Reading::Reciprocal` is one row of degree 2,
`(B − A)·v(A)`; `Reading::Axis` five of degree 1: the axis's direction against the two directions
square to `ω̂` (`square_to`: the world axis `ω̂` leans least toward, its part off) times the lever
(the motion's first axis as drawn), its place against `ω × v₀/|ω|²` across them, and the pitch
against `ω·v₀/|ω|²` (`axis_of`; `None`, so NaN rows, where `ω` is zero). Written once over
`kernels::Num`, the Jacobian by `held::block_jac`. `motion::Family::instantaneous` is the same
read off a snapshot (`Isa`: point, direction, pitch, twist); `report::screws_text` is the CLI's
line. `tests/phillips.rs` holds the screw to Ball's closed form on the common perpendicular — the
foot `E ω₂·ω/|ω|²` from the pinion's shaft, the pitch that point's speed along `ω` — at three shaft
angles, no offset giving zero pitch through the apex, and `L reciprocal mesh` to the members'
equal speeds along `L` at every point of it.

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
`[-d, d]`). Partial revolutions and face families refuse.

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
