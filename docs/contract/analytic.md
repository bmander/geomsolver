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
moves every place a quarter turn. `PointOnEnvelope` (four rows, three owned) is built at its widths
(`KernelKey::Envelope`); what is read at a point it holds is the envelope's frame, a `Rate` of the
contact (#365, `contact_rate`, curves.md's "The rate of a contact"): `held::frame` gives the point,
`∂X/∂u` to read a normal across, the unit normal, and two directions of the place square to `∇g`
with `−dn·dX` along them, so a line in its tangent plane (`in_plane`, one row), along its normal
(`normal`, two), a plane along it (`plane`, two) and a section's circle (`curvature`, three:
Meusnier, `x + (n − (n·m)m)/κₙ`) share the contact's place (`solver.md`, #251). Seeds: `held::seed`, a coarse search
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
[0.109]): a surface traced in space, `meshing::Side::Traced` over `traced::flank_consts` (`[0,
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

**The common normal and the path of contact at a point (#345 S1, [0.112], #365):** both are the
meshing contact's frame, read by a `Rate` of it (`contact_rate`, `meshing::frame`): `contact
perpendicular(at: q) l` its `normal` — the line square to the gear's tangent plane at the contact
(`along`, `n₂ × along`), two rows, degree 0 — and the path of contact the contact point's rate along
the phase, `rate(q over t) parallel l` (`contact tangent(at: q) l` the same statement), held
across a line, along one or measured along one, a length a radian of the phase, degree 1.  Columns
the lead (a line's two lifted ends or an axis's direction), then the point's contact's eight
unknowns — the rate sharing the phase, `holds_on` handing the rest as columns — then the meshing's
columns at no lead (`Parts::consts(sk, 0)`). The path is **the contact
point's derivative along the phase in the fixed frame**, `∂x/∂φ + ∂x/∂z · dz/dφ` with `J_z dz/dφ =
−J_φ` over the seven rows' Jacobian in the lag and the places (`meshing::path_of`, `touching`,
the elimination `eliminate`): the implicit function theorem, the rows read once over `Fwd<T, 8>`, a
number carrying its derivatives in the eight unknowns, each itself a `T`; a line held along it is
read against two directions square to the line (`screw::square_to`). Its derivative in every column is the rows' second derivatives, exact:
`meshing::frame` takes it once a block as the point's Jacobian is, each flank's `Seen` over `Fwd<Dual, 3>` in its own place and
the poses over `Fwd<Dual, 2>` in the phase and the lag, each differentiated over its own columns
only (`derive`), then `assemble` and `path_of` over `Fwd<Dual, 8>` by the chain rule. It is not the
bearing: `contact.sv`'s pair and step 3's turned cutter stand still on the gear's flank (the
bearing a point), and their path is the gear's turn carrying the contact. At a line contact `J_z`
is singular and the rows are not finite. `tests/meshing.rs` is the gate — on `contact.sv` at 0° and
10° the point on the turned common generator where the cutting motion's relative velocity is square
to the cones' normal (`closed_contact`, apart from the meshing), the normal that normal and the path
that point's derivative over the phase; on the turned cutter the path a central difference of the
traced positions; a free point's phase solved from a stated path angle and the traced path at that
angle there; printing, sharing the place either way round and refusing an unheld point — with
`jacobians::meshing_contacts_agree_with_finite_differences` holding the point and every rate over
a formed, a round and two generated flanks.

**Ease-off (#345 S2, [0.114]):** `meshing(…, ease: M, along: l, profile: κₚ, lengthwise: κₗ)` is
`MeshingE::ease` (`model::Ease`: the point, the line, each curvature a `TermNumber` read off the
declaration's text by `program::envelopes::ease`, an unknown it reads bound there —
`expr::bind_free`, and kept bound by `expr::evaluate` while a contact holds to the meshing). The
syntax is `AngularSpan::ease` (`EaseSpec`), parsed with the meshing, printed as written, its two
names rescoped as a declaration's references are and counted by `edit::remove`. In the solve it is
`envelope::meshing::Ease`: `EASE` constants after the mesh's steps (each curvature's `c`, `m` and
whether `x` is a column) and columns after the mesh's axes (`M`'s lift, the line's ends', the
unknowns, `EaseParts` over `held`'s columns and `held::number`), read with the first flank
(`Mesh::flank_cols`), so a flank's block carries them and the poses' do not. `eased` reads the flank
at second order (`held::local`'s `dn`) and returns its point moved `d` along the normal, `d(X) = ½
κₗ ((X − M)·a)² + ½ κₚ |(X − M) − ((X − M)·a)a|²` in the flank's member as drawn; the normal of the
moved surface across two directions of the place along the flank (`u` and `v` for a formed flank,
`held::along_envelope` for a generated one), each `dX + (∇d·dX) n + d·dn`, turned to the flank's
side; and the first of those for `∂X/∂u`. Its own row is the flank's: an eased envelope is the
envelope's point moved, at the envelope's place. The relief runs along `∂u × ∂v` of the source as
`held` reads it. A traced flank is refused (E080 where the meshing is built, and `Mesh::read`).
`tests/ease.rs` is the gate — the conjugate pair of `tests/meshing.rs` eased about M touches there
with no lag where it is drawn and at a point at every phase, and its lag is the turn of the gear
that closes the gap between the gear's cone and the eased flank worked out apart from the meshing
(`tests/meshing.rs`'s envelope point, moved along its normal by the parabola, and its `closing`),
the contact where they close; the lags at −15° and 15°, stated of two points with both curvatures
unknown, solve both back; printing and refusals — with
`jacobians::meshing_contacts_agree_with_finite_differences` holding the point, the normal and the
path over an eased generated and an eased formed flank, a curvature an unknown in each.

**A flank regular within its blank (#345 S3, [0.115]):** `flank regular(>= m, by: tool) within
blank` is `CKind::RegularWithin` (`KernelKey::Regular`, one row of degree 0): the least **area
factor** `A = (g_t − α g_u − β g_v)/g_t` (`∂X/∂t = α ∂X/∂u + β ∂X/∂v`, least squares) of a held
envelope over the tool points on `tool`'s face whose contact in the roll interval lies in `blank`,
less `m`. `A` is what `solid::admission`'s E3 samples by central differences (`area_factor`, its
`least_area_factor`): 1 rolling without sliding, nothing at a fold. `envelope::regular` is the
evaluator. `Parts::of` reads a `Held::Set` cone or cylinder (others refused). `tool` and `blank`
are `SolidDef::Region`s whose terms are read as `solid::field::region::Term` reads them, in
closed form over `kernels::Num`: a ball, a cylinder, a cone (its generator nearest, or the apex
past a quarter turn), a half-space along a line or an axis, each grown. The tool's are read at the
source's point as drawn (`held::source_at`) and the blank's at the generated point. The tool's
term that is the source (same shape, same axis line, same number, `same`) is the face and is no
condition; one grown off it is refused (the backlash's offset face is not the source's envelope).
A round of the face (`RegionShape::Measured` over a two-term set, one the source's grown by `r`,
`Shape::Round`) is the core's other term at the source point's foot on the grown one, which ends
the face at its tangent line exactly. Another face's round is read as its core's terms less `r`,
short of the true distance only about that face's own edge. The roll's ends, and a line source's
`[0, 1]`, are conditions too. Columns are the envelope's (`held::Parts`, lead 0), then each
point, axis and unknown of the terms once (`Builder::col`; an unknown by `held::number`). The
constants are `Problem::write`'s. **The worst point is searched for**: the face's stretch of the
meridian scanned (`face`), a grid of it × turns × rolls (`GRID`, the equation at each roll's pose
read once, `held::posed`) whose sign changes give contacts, each kept where every condition holds;
the best four followed by a shrinking 7 × 7 grid in `(u, v)`, the roll on the envelope at each;
then Newton on the KKT conditions (`kkt`: the Lagrangian's gradient in the place by `Dual<3>`
over `read`, the envelope's equation and up to two held conditions, the place's Jacobian by
differences), a multiplier on the wrong side or a free condition broken trying fewer held
(`polish`). The Jacobian is the Lagrangian's in the columns at the worst point, `∂A + λ ∂g + Σ μ
∂c`, the place and the multipliers standing (the envelope theorem), exact by `Dual<PASS>` passes.
`worst` remembers each problem's last answers by its constants. The same columns give the answer
again, nothing found included (a residual's Jacobian asks where the residual did, and a drawing not
yet solved has none). Columns within `NEAR` start from the last place and its held conditions
(`warm`), anything else searches again (`cold`, ~0.1 s). A least found, not a bound proved. A
fold between grid points the shrink never reaches is missed, as a sampled check misses one.
`tests/regular.rs` is the gate. On `spiral_bevel/formate.sv`'s conjugate pinion (the inner cone's
envelope under the mesh, `pair.gear_tooth.crown` within `pair.pinion_blank.material`), the least
is the admission's from below: 0.722584 against 0.722589 sampled at 12.5°, 0.135043 against
0.135059 at 8°, each on the inner round's tangent line. Its zero, solved, is 7.497°: a tenth of a
degree above it the pinion is admitted and the bound holds, a tenth below the admission refuses it
for a fold and the solve reports the bound broken. A margin of 0.3 solves the pressure shift to
8.7824°, read back and sampled there. `jacobians::regularity_agrees_with_finite_differences` holds
the Jacobian, the cone's half a column, at the solution and knocked off it. `hypoid.sv`'s
`Balanced` is unchanged: it balances two members' osculating radii at two sections, a choice of
settings, not a feasibility bound.

**The pair stated as its meshing problem (#345 S4):** `spiral_bevel/meshing_problem.sv` is
`formate.sv`'s pair with the gear's concave flank eased: the meshing reads the face mill's inner
cone eased about the mean contact T as the first flank, under `motion(pinion_turn, relative_to:
gear_turn)`, and the pinion's envelope of the cone as the second. The pressure angle, the path's
angle at T, the lag at the start of the phase and no undercut are stated; the two curvatures are
the unknowns (the block torn at fewer, `solver.md`). It adds no word: the gear's eased flank is
cut by a blade the document draws (`EasedBlade`), an arc in the normal section bent from the
cone's generator by the parabola at 2 mm and turned about an axis parallel to the mill's through
the centre of the circle through T and the cone's point 2 mm along the trace moved by the
parabola (divided by the cosine of the blade's lean, the circle's plane being square to the
axis) — a surface of revolution matching the eased cone to second order at T. Gates:
`tests/meshing_problem.rs` — the ease from the document's seeds and from seeds on either side
(κₚ = −0.018005, κₗ = −0.0028544 a millimetre); the contact at phase 0 at T, the cone's normal there
30° from the plane square to the mill's axis, the path 70° from the heading by differences of the
traced contact, the lag −0.03° at −7.5°, a point contact over the phase, the least area factor
(0.486) the sweep's admission's from below; the blade held within 0.1 µm of the eased cone
(worked out from the cone alone) along the traced bearing, 2 µm within 1.25 mm and 4 mm of T,
20 µm at the tooth's corners (it measures 0.02, 1.5 and 13 µm). The tool reading the two
exported STLs (`gcs-cli/tests/pair_check.rs`,
`the_exported_meshing_problem_pair_runs_as_its_meshing`, ignored): the convex side's clearance
gained over the phase is the meshing's lag times the pinion's lever within 1 µm, the members
nearest within 1 mm of the meshing's contact (the next pair's, a pitch on, at the phase's ends),
the concave side constant within the files' sag, no overlap.

**A point a motion carries (#326, [0.106]):** `Reading::Carried` is `X − Pose(τ)·x`, the pose
`held::carried` at `τ` degrees — any motion. `held::Step::Turn` (code 4, nine columns: a centre's lift, its view's `u` and `v`) is a turn
about a point of a view as it reads in space; only `Moving::of_any` reads one.

**A motion's twist, read by rates (#326, #365, [0.130]):** a carried point's rate,
`rate(X over t)` of `X coincident(under: m, t == τ) x`, is `∂(Pose(τ)·x)/∂τ` a radian
(`held::carried`, the pose's derivative in the roll carried by its `H2`), every other column held —
the twist at `X`, any motion's, its twist constant or not.  `Reading::{Rate, RateSquare,
RateAlong}`: the rate's two components across the line's direction (`square_to`), its component
along it, and that less a distance — `(m, c)` in the constants, a free one's unknown the last
column — each degree 1, a velocity, so a point on the axis of a screw of no pitch, standing still,
holds them regularly.  Columns the line's lifted ends (or an axis's place and direction), the
carried point's base point's lift and the roll, then the motion's.  The standard library says the
twist's two words with them: `l reciprocal m` (a point of `l` carried at roll 0, its rate square to
`l`: `d · v(p)`, the same at every point of `l`) and `Screw(isa, m, pitch)` (two points of `isa`
carried, their rates along it, the first's speed along it the pitch — Ball's pitch times `ω` along
`isa`, the advance a radian of the roll).  `motion::Family::instantaneous` reads Ball's screw off a
snapshot (`Isa`: point, direction, pitch, twist).  `tests/phillips.rs` holds `std.Screw` to Ball's
closed form on the common perpendicular — the foot `E ω₂·ω/|ω|²` from the pinion's shaft, the pitch
that point's speed along the axis — at three shaft angles, no offset giving zero pitch through the
apex, and `L reciprocal mesh` to the members' equal speeds along `L` at every point of it;
`tests/rate_of_contact.rs` a rolling circle's carried point's rate to the epicycloid's derivative.

**The post-solve B-rep is Rust API, not Solvent** (#364). Patches, seams, corners, edges and face
boundaries add no unknown, no solve row reads them and nothing exports from them, so the language
has no word for them (`patch`, `seam`, `vertex`, `edge(…, along:)` and `face(…, on:)` were retired
in spec 0.120) and the ABI and the browser bind none. Each is built of its parts — the surfaces,
envelopes and solids a drawing names, read with `RevolvedSurface::named`,
`GeneratedEnvelope::named`, `SurfaceProjector::named` and `RevolvedRegion::read` — through its
`::new` (or `TrimmedPatch::of_envelope`/`of_surface`), as `tests/envelope/paired.rs` builds the
spiral bevel's off `faces::face_source`. Identity is the caller's: two corners are one only where
the caller gives them one identity, never because they coincide. Every type is an immutable
snapshot: a change to the drawing needs the part made again. `tests/patches.rs` and
`tests/seams/**` hold the rules below over the constructors.

**Trimmed spatial patches:** `patch::TrimmedPatch::of_envelope(name, envelope, inside, outside)` and
`of_surface` keep a source's points within every `inside` region and outside every `outside` one,
each a `solid::RevolvedRegion`, with explicit trim and residual tolerances; regions read unmodified
full revolutions (line/arc/circle profiles, holes) with an explicit axis tolerance.
`RevolvedRegion` is analytic meridian membership apart from the CSG kernel. Mind full-circle
seams, arc shared vertices, vanishing axis edges; never turn a sphere's construction diameter into
a wall. The final root must satisfy trim and envelope; a trial may cross a trim. No branch, face
loop or closed solid yet.

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

**Shared generating seams:** `seam::EnvelopeSeam::new(name, [first, second], ends, tolerance)`
takes two patches of envelopes and each source edge's `u` at the vertex they share, which the
caller knows from the profile (`paired.rs` reads it off the cutter's meridian). It checks
coincidence and tangent-plane agreement there with explicit `SeamTolerance` — ends that are no
shared vertex are refused, never repaired by proximity — and intersects their angular domains;
first-source u is fixed at the junction. A patch of a surface is refused. `SeamIntersectionOptions` exposes only [v,roll]; `intersect` solves both envelope
equations and one section through the DogLeg adapter (one normal-velocity tolerance for both
faces), and the final root checks both envelopes and material trims. Not a nearest-point weld or
an arbitrary surface intersection. Local singular intersections are refused. Contacts carry the
first face's normal and generating velocity, not the seam's curve derivative.

**Envelope/boundary seams:** `seam::BoundarySeam::new(name, face, boundary)` is a patch of an
envelope cut by a finite analytic surface. It holds one `EnvelopePatch` and one
`SurfaceProjector`, keeping the full [u,v,roll] domain; it solves envelope, boundary support and
section equations in that order, then checks finite incidence and trims. A separate type from
`EnvelopeSeam`: three unknowns, no junction tangency tolerances. Two surfaces meeting are a
`SurfaceSeam::new(name, first, second)`. A seam asserts no regular branch or oriented face loop.

**Shared spatial vertices:** `vertex::BoundaryVertex::new(name, face, [a, b])` is a generating
face meeting two finite surfaces (three unknowns), owning one copy of the face;
`JunctionVertex::new(name, junction, boundary, face)` a generating junction meeting one boundary
on its face 0 or 1 (two unknowns), checking finite incidence on that face — a coarser seam
tolerance cannot transfer finer boundary membership. `position` is xyz only, no artificial corner
normal. Local solves check rank, not global uniqueness.

**Finite spatial edges:** `edge::SpatialEdge::new(name, source, vertices, endpoints, direction,
tolerance)` is a finite axial slicing interval of a seam between two corner identities. Each
endpoint is given in the seam's own chart — a junction corner read on its second face takes that
face's `u` at the shared vertex, which the caller puts in — and is checked on the seam. At a
junction a boundary on the other incident face shares the corner through the caller's identity;
never infer this from proximity or require duplicate corners. `sample` solves via seam APIs, checks local rank/material/incidence
inside, and reuses stored endpoints checked by their full vertex constraints. Never re-solve a
known endpoint with a potentially tangent slicing equation. Fraction is linear projection along
the direction, not arc length; local samples are no global branch certificate.

**Checked shell topology:** `topology::ClosedShell` owns explicit edge endpoint identities and
ordered directed uses in connected faces with boundary loops; construction checks closure, two
opposed uses per edge, one circular link per vertex and face connectivity (periodic self-uses and
annular faces supported). Separate from planar `FaceE`; no coordinates or tolerance weld.
`from_triangles` uses index identity. `mesh::stl_topology` verifies the encoded float32 coordinates
(exact equals, signed zero normalized). Neither establishes incidence, non-self-intersection,
outward orientation or deviation error. See `docs/shell-topology.md` before connecting faces.

**Spatial face boundaries:** `spatial_face::SpatialFaceBoundary::new(name, support, uses,
tolerance)` binds an ordered loop of edge uses to an analytic support (`Support::Envelope` of a
patch, or `Support::Surface`). The loop closes on the edges' own corner identities, in order; a
use whose seam's chart is the other face's of a junction carries this face's `u` there
(`Use::endpoint_u`), and samples keep edge xyz and check this support's incidence. A `FaceE` is a
planar profile only, its `plane` the one it inherits. No holes yet; solid assembly still needs an
interior/embedding certificate. See `docs/analytic-face-boundaries.md`; a closed boundary walk is
not a valid disk.
