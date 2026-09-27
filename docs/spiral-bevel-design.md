# Parametric spiral bevel pair: design and implementation record

The deliverable is a new matched pair whose tooth counts and dimensions are Solvent
parameters. Solid design is separate from manufacturing: no cutter selection, toolpath,
machine-axis access, feeds, or process allowances belong in the pair's interface.
A generating surface is a mathematical construction; it does not prescribe a physical cutter.

**Current direction:** [the subproject roadmap](spiral-bevel-roadmap.md) evaluates continuous
generating-volume subtraction as the primary solid representation. Further manual cap/face
assembly is paused while that route is assessed. The implementation history below remains
evidence for analytic geometry and verification; it does not establish a finished solid.

## Current implementation

`rust/gcs-core/src/envelope.rs` evaluates smooth surface envelopes from surface derivatives
and composed rigid motions, including the derivatives of moving inverse frames. Its
intersection solver uses the existing DogLeg implementation, explicit parameter scales,
physical residual tolerances, finite search bounds, and a final regularity check. Invalid,
singular, and unconverged intersections are errors. Equal parameter bounds hold a parameter
exactly and remove it from the numerical unknowns; all residuals still have to hold, and the
Jacobian must have full rank over the remaining unknowns. This is needed when following an
exact generating-edge endpoint: leaving that coordinate free under an equality and strict
bounds can reject Newton steps because of roundoff. The bounded adapter also projects trial
steps into the domain so a root on an edge can be reached without freezing the other
parameters. Residual and rank
checks still reject an excluded root or an underdetermined stationary family.

Solvent exposes this through `envelope flank(source, under: generating, from: -35deg,
to: 35deg)`. The descriptor binds a named surface and motion over an angular domain;
`GeneratedEnvelope` snapshots their solved state and evaluates the defining equation or
solves a local section intersection. It does not yet specify material trims or assemble a
closed solid. The matched pair now declares its generating surfaces, relative motion,
crown-center placement and implicit envelopes in the model source.

`solid::RevolvedSurface` now reads exact conical and toroidal patches from the named
line/arc/circle boundaries of an existing Solvent revolution. Positions and first derivatives
come from the solved source geometry, independently of faceting. It returns snapshots;
the caller must re-read after a model change. Solvent now names these patches with
`surface flank(crown, edge: rack.outer)`. A `surface` is an evaluated spatial reference:
it owns no solver coordinates and can be private, tagged, or passed through a component
formal. It follows its source geometry across edits; copy/paste and dependency deletion
carry that relationship. The browser's `surfaceSample` API reads positions and exact
derivatives through the same core evaluator. This supplies generating surfaces to the
envelope evaluator, but does not yet add a generated gear solid to the model.

`blank/cone.sv` and `blank/sphere.sv` define the conical and spherical boundary components,
drawn in each member's axial view by `blank/member.sv`. The layout declares spherical toe and
heel at `R_mean ± face_width/2`, with the current `face_width = 0.2 R_mean`. Each member has
tip, root and back cones at signed pitch-normal offsets `+m_n`, `-1.25 m_n` and `-4 m_n`. A
meridian's generator coordinate spans `[0.5 R_mean, 1.5 R_mean]`; the separately declared
spheres set the actual face ends. These carriers are construction geometry, not stock or
machining tools. Their offsets, angles and radii constrain the geometry; which side of the
generator a meridian stands on is a directed angle from the rib toward the axis, not a sign.

The numerical rim generator reads its tip and back cones and toe/heel distances from these
source surfaces. Its previous hardcoded addendum, back-depth and face-end formulas are now
only in the independent verifier. `SurfaceProjector` supplies signed meridian equations and
finite-patch incidence checks. `GeneratedEnvelope::intersect_boundaries` uses two such named
patches as section equations, then checks their actual finite incidence errors. A negative
control restricts a spherical boundary to an unrelated narrow sweep: the same sphere equation
still vanishes, but the intersection is correctly refused as outside the declared patch.
An analytic line/sphere section returns all roots on a finite line meridian and constructs
the rim's back section. No triangle mesh enters these equations.

The independent numerical reference is Bibel, Reddy and Kumar,
[NASA CR-191009 (1994)](https://ntrs.nasa.gov/api/citations/19940028394/downloads/19940028394.pdf).
The tests reconstruct all four generating motions and conical surfaces from the printed
settings and transformations (pp. 4–6 and 25–33). They compare the first point of each flank
with the independently published coordinates (pp. 9–11), within 1e-7 inch, and test 8 × 6
points on every flank against the separately stated scalar meshing equation. The latter
equation is not used to generate the tested points. The coordinate tolerance does not claim
that the many printed decimal places establish that accuracy: the original program uses
fixed iteration counts and some single-precision expressions.

These settings are a regression fixture, not a recipe for arbitrary tooth ratios. The
reference also assumes a full fillet for its finite-element model; that assumption is not
being adopted as a verified root construction.

## Parameter contract

The proposed component inputs are integer `pinion_teeth` and `gear_teeth`, a length scale
(`mean_module`), `face_width`, `shaft_angle`, `pressure_angle`, `spiral_angle`, and parameters
for tooth thickness, clearance and the root transition. Two integer tooth counts specify
the ratio exactly; an arbitrary real-valued ratio must not be rounded silently.

`mean_module` means transverse module at the mean pitch circle. It must be named explicitly
because mean/outer and normal/transverse module are different conventions. For the initial
90° intersecting-axis construction,

```
delta_p = atan2(pinion_teeth, gear_teeth)
delta_g = pi/2 - delta_p
R_mean  = mean_module * hypot(pinion_teeth, gear_teeth) / 2
r_p     = R_mean * sin(delta_p)
r_g     = R_mean * sin(delta_g)
```

Changing tooth counts changes the pitch cones, indexing and relative motion. Changing size
scales the generating geometry and its trim surfaces. Search hints and tessellation are
not geometric design parameters. Invalid parameter combinations must fail visibly; the
implementation must not quietly shorten a tooth to hide undercut or interference.

### Offset shafts: the true hypoid

`configuration.sv` states `shaft_angle` (90°) and `offset`, the common perpendicular E between
the shafts. The gear's cone is the intersecting-axis one above. The pinion's is solved
(`pitch/pinion.sv`): its apex lies on the gear's pitch plane, its axis is square to the gear's
and E from it, and its pitch radius at the mean point is the equal normal pitch's,

```
r_p cos(psi + eps) = pinion_teeth * m_n / 2,    m_n = mean_module * cos(psi)
```

where `eps`, the angle the pinion's generator turns from the gear's in the pitch plane, is
solved rather than stated. That is the condition under which the relative velocity of the
crown and the pinion at the mean point lies along the tooth trace. The pinion's pitch angle
comes out below `delta_p` (24.36° against 26.57° at the configured 25 mm), and its spiral angle
is `psi + eps`.

Conjugacy still runs through the common crown, but the relative angular velocities are no
longer parallel, so the argument above becomes the envelope argument alone: each member is the
envelope of one crown section, and a crown normal meeting both envelope equations at a point
makes the members touch there. What must hold exactly is timing: the crown turns once per
`N_c = 2 R_mean / mean_module` of its pitches, the gear `N_c / gear_teeth` and the pinion
`N_c / pinion_teeth` times per crown turn, and each is indexed by its own tooth. The pinion's
own cone would roll at `1 / sin(gamma_p)`, which is not that ratio off the bevel, so
`generation.sv` measures the pinion's roll off the gear's triangle, `R_mean / (pinion_teeth *
mean_module / 2)`; `tests/hypoid_layout.rs` holds both ratios to `N_c / N` within 1e-12 at five
designs. At E = 0 the pinion is the intersecting-axis one and every bevel number is unchanged.
The configured pair is E = 25 mm with a 12.5° pressure shift and a 25° spiral, inside the
generating-sweep class for both members
([the layout plan](spiral-bevel-layout-plan.md#the-true-hypoid) records the grid).

## Common-crown construction under investigation

[Chang, Huston and Coy, NASA TM-101449 (1989)](https://ntrs.nasa.gov/api/citations/19890007877/downloads/19890007877.pdf)
derives bevel tooth surfaces as envelopes of a rotating crown surface. The following
shared-crown construction and conjugacy proof are our derivation using that framework;
they are not the machine settings of CR-191009.

Put the common apex at the origin, the common pitch generator along x, and the crown
axis along z. With pitch angles `delta_p` and `delta_g`, the two shaft unit vectors are

```
a_p = (cos(delta_p), 0,  sin(delta_p))
a_g = (cos(delta_g), 0, -sin(delta_g))
```

Use crown rotation `t` as the shared motion parameter. The shaft angular velocities are
`a_p/sin(delta_p)` and `-a_g/sin(delta_g)`. Subtracting the crown's angular velocity gives
`cot(delta_p) x_axis` and `-cot(delta_g) x_axis`. Consequently the two relative velocities
at every spatial point are proportional. A crown normal satisfying the envelope equation
for one member satisfies it for the other:

```
n · ((omega_p - omega_c) × p) = 0
n · ((omega_g - omega_c) × p) = 0
=> n · ((omega_p - omega_g) × p) = 0
```

In column-vector notation the generating family in each member's frame is
`M_i(t) S(u,v) = B_i(t)^-1 C(t) S(u,v)`, where

```
C(t)   = Rz(t)
B_p(t) = Ry(pi/2 - delta_p) Rz( t/sin(delta_p))
B_g(t) = Ry(pi/2 + delta_g) Rz(-t/sin(delta_g))
```

The experimental crown flank is a circular trace with a straight inclined normal section:
`S(h,theta) = (cx + (rc + h*k) cos(theta), cy + (rc + h*k) sin(theta), h)`.
Choose its center so that the trace passes through the mean pitch point at the desired
spiral angle; `k = ±tan(pressure_angle)`. The two crown flanks have opposite slopes and
appropriate angular tooth-thickness offsets. `tests/envelope/crown.rs` exercises this
construction across tooth ratios and sizes, checking common positions, common normals,
and the normal component of the actual shaft-relative velocity. It remains a **local
flank experiment**. Equal contact position and normal plus zero normal-relative velocity
do not prove global freedom from interference or establish a usable closed tooth.

An initial deeper-section experiment failed for the 12:36 pair at 85% mean cone distance,
0.45 mean module below the pinion pitch surface. An independent analytic construction
now identifies a cusp on that local branch: for mean module 0.2, its crown height is
approximately 0.13587436 and its polar angle 17.13147846°. The requested section's polar
angle is 16.83667090°, below that branch's minimum. The surface tangent vanishes at the
cusp. Fixing crown height and spherical radius allows the generic solver to trace both
sides of it, confirming that the failure was not a search-bound issue. This is a local
branch diagnosis, not an exhaustive global envelope certificate. The common-crown pair
tests cover a band of ±0.15 mean module, not a complete root-to-tip tooth.

## Declarative root construction

`rust/examples/spiral_bevel/crown/section.sv` defines `RackSection` using ordinary lines,
circular arcs, tangencies, flank angles and depths from its two pitch points
(`crown/rounding.sv` states the tip roundings every crown section shares). Its construction
points stay private. Revolving its public profile produces the reference geometry, and
`crown.svd` draws its preview. The source solves with zero degrees of freedom after its seeds
are disturbed. The preview places the pitch line at 0.8 of the 24:48 pair's mean cone
distance; in the layout (`crown/thickness.sv`) the pitch points follow from the tooth traces.

The envelope tests read the actual solved `outer`, `outer_round` and `tip` patches through
`RevolvedSurface`. They verify the conical and toroidal equations and their derivatives,
then generate a pinion flank-to-root transition for a 24:48, mean-module-2 fixture.
At 90%, 100% and 110% of mean cone distance, the generated transition joins both its flank
and root cone with matching positions and normals; the root has the declared depth of 2.
Sampling the transition finds no collapsed segment. This is evidence for the tested
patches, not yet proof of whole-pair root clearance or an admissible parameter domain.

This construction follows the conical/toroidal generating-surface framework described in
[Teixeira Alves et al., Mechanics & Industry 13 (2012), pp. 325–335](https://www.mechanics-industry.org/articles/meca/pdf/2012/05/mi110099.pdf),
§2.1.1. The reference surface is a design construction; no physical cutter or machining
route is selected. In particular, the generated fillet is not an arbitrary circular blend
inserted after generating the tooth flank.

The matching step distinguishes generating normals from material normals. Simply using the same
oriented crown patch for both members establishes the velocity equation, but gives coincident
normals, not the opposing outward normals required for contact. `crown/mate.sv` (with
`crown/mate_section.sv`) implements the circular reference section by pairing the pinion's
outer flank with a mate section drawn on that same flank line one tooth's width outward, and
the pinion's inner flank with one a tooth's width inward. Each mate section has its tip
pointing the other way along the cutter's axis, which reverses the pressure slope; it walks its
edges the other way round, which restores the same geometric cone with opposite boundary
orientation. The two axes share one spatial center, and the mate's flanks lie on the tooth's by
construction, so the two sections cannot be given different pressure angles. Tests read the
actual source geometry for 24:48, 32:32 and 28:49 pairs at mean modules 0.2, 2 and 25.4. At
three face stations and three active-flank heights they verify common positions, **opposite**
oriented normals, equal generating roll, and zero normal component of the actual shaft-relative
velocity. Both sides of both members in the 24:48 fixture also join their root cones
tangentially through the declared toroidal envelope.

The source gives the reference centerline a circular trace radius of `0.8 R_mean`, with a
35-degree spiral at the mean pitch point in the design the paired checks read (the configured
pair's is 25 degrees). Its two flank radii are the distances from that trace center to the
points at `±90°/hypot(z_p,z_g)` on the mean pitch circle. Their difference determines reference
width. Thus the generated pinion space and mating gear tooth each occupy exactly half their
angular pitch at the mean pitch circle; the tests check this independently in the member
frames. This is a **zero-backlash nominal geometry** convention. It does not assert half-pitch
thickness elsewhere across the face or choose an allowance for manufacture.

The reference dimensions are uniform depth: normal module is `mean_module*cos(spiral_angle)`,
root depth is 1.25 normal modules, and the tip rounding's radius is 0.3 normal modules. A base
depth of two normal modules extends each active reference flank beyond the future tooth-tip
trim. For the 24:48 pair the checks read, tests trim at one normal module of addendum and
sample 11 spherical face stations from 0.9 to 1.1 mean cone distance. On all four root-to-tip
contours, polar angle increases from root to tip without a sampled reversal. This does not by
itself prove surface regularity everywhere or global non-interference.

The later [continuous-volume investigation](continuous-volumes.md#indexed-closure-counterexample)
finds that simply sweeping and indexing these full closed construction solids does not
preserve the intended gear material. An inactive wall of a neighboring `gear_outer` section
causes a verified overcut. This does not invalidate the local shared-flank equations above;
it prevents treating the reference solids' arbitrary closure as a finished generating system.

## Closed-rim verification workbench

`tests/envelope/paired/boundary.rs` now assembles both complete 24:48 rims from those
generated contours. The pinion material tooth spans its inner flank and the next indexed
outer flank; the gear tooth spans its two mating reference flanks. Tip and root connections
are arcs on their declared cones. Spherical toe and heel faces lie at 0.9 and 1.1 mean cone
distance. A back cone four normal modules below the pitch cone closes the rim. These are
explicit nominal blank choices, not a hub, shaft interface or manufacturing specification.

The workbench refuses reversed angular contours and crossed tip/root connections. Each rim
must construct a checked, connected, oriented shell with one circular face fan per vertex,
two opposed uses per edge and genus one. Edge pairing and Euler count alone do not establish
manifold closure; see [Checked oriented shells](shell-topology.md).
At 4, 8 and 16 divisions per generating patch/face interval,
the pinion volumes are 9116.6328, 9131.5731 and 9134.7678 mm³; gear volumes are 20298.3711,
20294.2049 and 20292.6164 mm³. Successive volume differences decrease for both members.
The finest meshes pass `checked_stl`, which tests float32 triangle representability, and
`stl_topology`, which reconstructs a closed genus-one shell from the actual encoded positions.
Volume convergence is not a surface-deviation bound, and topology alone does not establish
geometric non-self-intersection.

To reproduce the experimental assembled-position STL files:

```sh
mkdir -p /private/tmp/solvent-bevel-pair
SOLVENT_GEAR_OUTPUT=/private/tmp/solvent-bevel-pair cargo test \
  --manifest-path rust/Cargo.toml -p gcs-core --test core \
  envelope::paired::boundary::generated_rims -- --nocapture
```

This Rust test workbench supplies the motion, trims, indexing and mesh assembly. Only the
reference geometry is currently expressed in Solvent. It is a verification stage, not the
final language interface or an approved export for manufacture. The checks below strengthen
the assembled-pair evidence; regularity between surface samples, a global interference bound
and a tolerance-controlled surface export remain to be established.

## Independent assembled-pair checks

`tests/envelope/paired/analytic.rs` evaluates the circular-crown characteristic by closed-form
geometry, independently of the numerical envelope intersection solver. At fixed generating
height `h` and meridian radius `r`, intersect the generating circle with a sphere of radius
`rho` about the apex. For reference center `(cx,cy)`, distance `c = hypot(cx,cy)`,

```
cos(theta - atan2(cy,cx)) = (rho² - h² - c² - r²) / (2 c r)
```

The lower circle-intersection branch is the declared local branch. With its translated
position `P` and normal `n`, solve the characteristic roll directly:

```
A = nz Px - h nx
B = nz Py - h ny
A sin(t) + B cos(t) = 0
```

Choose `t` modulo pi in `(-pi/2,pi/2)`. The tests compare this construction with numerical
envelopes at 21 parameters on all four cones and four toroidal transitions at three face
stations, including patch endpoints. Positions agree within 1e-7 mm and oriented normals
within 1e-8. This independent derivation is specific to the circular references; it is not
a second gear-specific implementation in the core.

`paired/boundary/interference.rs` uses that evaluator to invert the contour at an azimuth
and classify nominal material directly, without interpolating a tooth surface or using mesh
triangles. It checks the numerical source contours against this independent boundary first.
Across nine spherical face stations and 33 phases in one tooth period, 4,105,728 outer-boundary
samples from both rims remain outside the exact mating contour. The smallest reported signed
meridional clearance is approximately 4.9e-10 mm, numerically indistinguishable from contact.
The classification permits 2e-8 mm numerical error. A separate interpolated-section check
at 16, 32 and 64 subdivisions also finds no penetrating samples. These are **sampled
interference checks**, not a bound on clearance between stations, phases or boundary samples.

Contact coverage is checked separately from clearance. Intersect the declared active-height
ranges of the two mating flanks and evaluate their contact-roll endpoints at five face
stations. Conservative algebraic bounds over each entire height interval keep the generating
circle/sphere intersection transverse and keep `A` away from zero. Therefore the chosen roll
is continuous across the interval. Periodic copies of these contact windows overlap and cover
the full crown indexing period, `2 pi / hypot(24,48) = 0.1170802455` radians, on **both** sides
of the teeth. At the mean face the two windows are approximately `[-0.052536, 0.100114]` and
`[-0.104075, 0.047778]` radians. At 25 rolls per window the verifier checks opposing material
normals, common positions on both nominal boundaries, and the correct opposite tooth-index
shifts in the actual assembly. A ±0.001-radian phase perturbation supplies a negative control:
one displaced probe enters material while the other moves outside.

Continuous coverage by these contact windows does not establish global freedom from
interference. It also does not establish load distribution, elastic transmission error or
robustness to mounting errors; those have not been included in this nominal geometry study.

## Language direction

Keep gear knowledge in components. The reusable language capabilities needed are:

1. **Implemented:** named analytic surfaces from existing solids provide the initial generating
   geometry: `surface flank(crown, rack.outer)` and `surface root(crown, rack.outer_round)`
   refer to the cone and torus. General spatial
   surfaces traced by a component member over two formals can extend this later, coherently
   with existing computed and constrained curves; the gear need not await or bypass that
   design with a separate formula language.
2. **Implemented:** `motion crown(about: crown_axis)` and
   `motion generating(crown, relative_to: blank)` express constant-ratio rotational families
   about solved lines. The matched-reference component declares crown, pinion and gear
   motions, with ratios `1`, `1 / sin(pinion_angle)` and `-1 / sin(gear_angle)`. The envelope
   verifier now evaluates these named relationships, including exact frame derivatives.
   The crown center is now source geometry: offset front/back planes and signed ordinate
   constraints place the reference axes at `(center_x, center_y, 0)`. The verifier no longer
   translates source patches. A fixed rotation only expresses results in its independent
   initial local shaft coordinate convention.
3. **Implemented:** `envelope flank(source, under: generating, from: -35deg, to: 35deg)`
   names the zero-normal-velocity locus over a finite motion domain. The pair component
   declares its flank, root-transition and root-cone reference envelopes. Numerical section
   intersections read these declarations. Crown surfaces now declare their source semicircles
   with `from:` / `to:` angles: 180–360 degrees for the pinion's front-axis references and
   0–180 degrees for the gear's reversed-axis references. Both describe the intended half
   of the crown trace in world space. Source parameters and tangent orientations remain
   unchanged. The verifier reads these domains and initializes each local solve at the span
   midpoint; its hardcoded half-circle bounds and 55-degree seeds have been removed.
   This is a geometric branch restriction, not a proof that every retained envelope point
   is globally regular. Additional analysis section equations still come from the verifier.
   The named evaluator owns solved surface and motion
   snapshots, and does not approximate the surface with triangles.
4. **Material regions implemented; topology in progress:** tip, root, back, toe and heel
   boundary surfaces are declared in the source. Core intersections use their analytic
   equations and check finite incidence. The ordinary `ToothRegion` component now declares
   `patch bounded(source, inside: limits.tip, inside: limits.heel, outside: limits.root,
   outside: limits.toe)` for all eleven generated patches, over each member's design group.
   Operands are the boundary components' closed carrier solids, reached through their
   public surface references. These conditions intersect and include
   their boundaries. Rim samples on flanks and root transitions must satisfy the named
   patch as well as the envelope equation. The remaining work verifies the retained branches,
   defines oriented boundary loops, and assembles a closed solid with stable face names.
5. **Shared generating boundaries implemented:** eight `seam` declarations identify the
   flank/fillet and fillet/root junctions through their common generating-profile vertices.
   The core checks that both operands share a revolution and motion, and that their solved
   source junction is coincident and tangent. Intersections read both envelopes and material
   trims. The rim builder now reads these named curves for its joined vertices. All eight
   seams are compared with independent closed-form characteristics at toe, mean face and
   heel for all nine tooth-count/size cases. Seams still need oriented uses in face loops;
   they do not assert that a collection of patches closes a solid.
6. **Named finite boundary intersections implemented:** twenty additional seams declare
   the four flank/addendum edges and the sixteen toe/heel edges of the flanks and fillets.
   They use the same `seam` declaration with a generated face first and boundary surface
   second. `BoundarySeam` retains the original envelope chart and material trims, solves
   against the finite boundary through the common intersection adapter, and rejects
   support-continuation roots. The rim verifier reads its tip vertices from these names.
   Across all nine size/ratio cases, 108 tip intersections are compared with independent
   scalar bracketing on closed-form characteristics; 432 toe/heel samples are compared
   with independent spherical sections. Oriented boundary uses and solid assembly remain
   separate work; these declarations do not yet close a tooth.
7. **Checked shell topology implemented in the core:** oriented face loops use explicit
   edge and vertex identities. The constructor checks loop closure, paired orientation,
   vertex links and connectedness; it supports periodic edge uses and faces with holes.
   The generated rims now retain this checked type before triangulated export. The encoded
   STL is checked again after float32 conversion. Binding analytic face regions and finite
   named seams to the same topology remains the next language/geometry step.
8. **Shared spatial corners implemented:** twenty-four `vertex` declarations identify
   tip, flank/fillet and fillet/root corners at toe and heel. Two boundary seams must share
   the exact generating-face identity; a junction corner's boundary must belong to one
   of its faces. The snapshots check all defining equations, finite boundaries and material
   trims through the existing solvers. The rim reuses these named endpoints. All 216 corners
   across nine size/ratio cases are compared with independent tip bracketing or closed-form
   generating characteristics. Local roots still need selected finite edge branches and
   oriented analytic face loops before they can define the final solid.
9. **Finite edge extents implemented:** twenty-eight `edge` declarations bound the seven
   seam roles on each of four tooth sides, sharing the twenty-four named corners. Each
   uses its member's shaft axis for spatial slices. Endpoint witnesses are checked and
   converted into the correct incident-face chart. The rim reads the finite toe/heel
   edges, and interior rows use the same axial coordinate. Across all nine size/ratio
   cases, 1,260 endpoint/interior samples agree with independent characteristics, axial
   slices and cone/sphere boundaries. This establishes local sliced geometry; global
   branch uniqueness, continuity and regularity still need proof before face assembly.
10. **Whole-domain crown branch bounds implemented:** outward-rounded interval arithmetic
    covers the complete meridian and toe-to-heel rectangles for all eight flank/fillet
    references in all nine configurations. Every leaf proves transverse circle/sphere
    intersection, the selected source semicircle, a nonzero generating normal and one
    continuous roll root within the declared limits. Independent rational checkers audit
    arithmetic, every reported margin and exact domain coverage. These are bounds for
    the nominal circular-crown equations, not yet source-solve error transfer.
11. **Nominal generated-surface regularity established:** second-derivative profile bounds
    support an analytic area factor for the complete crown-to-member surface map. Its
    interval enclosure excludes zero on all 72 flank/fillet domains, using 435 leaf cells,
    including the fillet/root endpoints. The rational checker verifies each enclosure and
    consistent positive orientation; independent numerical differentiation checks the
    derivation. The earlier 12:36 deep-section cusp reverses the factor's sign and cannot
    pass this gate. This proves local differential regularity of the nominal map, while
    source error transfer, global injectivity/interference and final analytic face assembly
    remain unfinished. See [Whole-interval geometry bounds](interval-geometry.md).
12. **Supported analytic face boundaries implemented:** the source uses `ToothSideFaces`
    to declare the eight working/transition faces with the existing `face` construct and
    explicit `on:` support. Their ordered finite edges share corners and traverse every
    flank/fillet join in opposite directions. Face samples map the support chart and retain
    the shared edge position, with independent checks at 1,440 boundary points across the
    nine configurations. The rim's end contours read these faces. Planar and spatial
    supports are explicit alternatives, so these faces cannot be mistaken for planar
    sweep profiles. Interior/embedding checks and remaining cap/indexed face assembly are
    still needed. See [Analytic face boundaries](analytic-face-boundaries.md).

The material classifier reads full revolutions with straight or circular profile edges and
holes. It uses analytic cylindrical-meridian crossings and finite curve distances, with
explicit axis and boundary tolerances. A sphere's axis diameter disappears on revolution
and is not a material wall. Sphere/torus equations, holes, reversed and horizontal axes,
ray tangencies and shared vertices test the classifier independently. The pair's sphere
and cone material signs and distances are checked against separate equations at all nine
tooth-count/size cases. Partial revolutions and other solid kinds are refused as analytic
clipping operands. The existing profile topology validator still uses facets; this is not
a global analytic validity or floating-point error certificate for arbitrary profiles.

The motion-language integration is checked against the independent shaft transformation at
three roll angles for 24:48, 32:32 and 28:49 pairs, including point positions, transformed
vectors and exact velocities. The existing envelope, rim closure, contact coverage and
sampled interference tests run through the named motions and envelopes. Elementary envelope
tests verify trial residuals, characteristic intersections, boundary roots, invalid domains,
component aliases, and dependency copying/deletion. Regression coverage includes angular source
spans, inherited envelope domains, finite conical caps, trimmed tooth-corner intersections,
shared generating seams, browser bindings and the CLI/ABI. The default source solves with DOF 0
and no diagnostics; these software checks do not remove the geometric gates below.

The verifier requests an iteration target of `1e-16` and hard-row acceptance of `1e-12`;
the ordinary solver owns
the DogLeg-to-LM retry. It no longer hard-codes LM to work around the interactive success
threshold. Seam position, normal and material checks retain their separate geometric
tolerances. The validation contracts and cleanup are recorded in
[Geometry validation and refactoring](geometry-validation.md).

The existing model/drawing split, component privacy and construction semantics apply to
these objects. A public `pair.pinion` and `pair.gear` can hide the crown, local frames,
characteristics and trimming scaffolding. Visualization belongs in the drawing.

Surface references, constant-ratio motion families, implicit envelopes, material trims
and shared seams, vertices and finite edges are specified in §6.13–6.19. Surface-bounded-solid spelling is not
normative yet. Adding an opaque gear builtin or
an external coordinate importer would bypass the language question rather than resolve it.

## Remaining gates

The end-to-end exported geometry accuracy target is **0.001 inch (0.0254 mm)**, selected
by the user for machining with approximately 0.002 inch repeatability. This is the total
geometric error budget, including fitting and export; it does not specify backlash or
replace engagement checks. Earlier 0.001 mm component checks remain tighter evidence.

- Specify a full admissible parameter domain and the tooth thickness/backlash convention.
- Establish both flanks, root transition, tooth tip, toe and heel boundaries and material
  sides; check regularity and trim the envelope's invalid branches.
- Validate indexing, assembled phase, contact continuity over a tooth period, and global
  non-interference. Add geometric relief only as an explicit design modification.
- Expose the general surface/motion/envelope construction in Solvent and express the pair
  as ordinary components.
- Connect bounded analytic surfaces to closed solid evaluation and tolerance-controlled
  export. Verify convergence, orientation, manifold closure and export fidelity.

Until these gates are met, the current code is an independently checked envelope foundation,
not a finished spiral bevel gear model or a production-qualified gear pair.
