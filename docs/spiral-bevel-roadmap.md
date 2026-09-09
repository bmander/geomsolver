# Spiral bevel gears: subprojects and representation decision

## Current critical path

The goal is one fully specified, independently verifiable, parametric matched pair from
a documented generating system, expressed through Solvent and exported at stated accuracy.
Manufacturing planning remains separate. **The active task is now Solvent integration.**
The user explicitly reprioritized it after the inspection exports. Further mesh-distance,
UV coverage and surface-attachment proofs are parked; the existing checks remain evidence
with their original limitations. They should resume for a concrete acceptance failure,
not displace frontend/backend integration.

The integration path is:

1. Use an ordinary, editable Solvent project for the configured generating geometry.
   `rust/examples/spiral_bevel/pair.sv` imports `configuration.sv` and instantiates
   `MatchedReferences`; the export adapter no longer rewrites the component's source text.
   Each new workflow saves its actual linked sources under `solvent/` and checks them with
   the normal `solventc` executable.
2. Express the blank, generating removal and tooth indexing with general solid operations
   in components. Carry those definitions across a backend interface instead of recognizing
   gear names or importing precomputed coordinates. Reuse the existing motion and material
   snapshot implementations.
3. Connect native OCCT construction and STEP/STL output to that interface, replacing the
   test-only section-export adapter. Check a small generic model before rebuilding both gears.
4. Run the existing pair and alternate-configuration checks through the public path, then
   address the remaining acceptance gaps below.

The first native path now exists: `solventc --step` evaluates a solved analytic solid DAG
through a C ABI bridge to OCCT's C++ library. The CLI's optional `occt` feature links
the native libraries directly; generation needs no Python process. Ordinary extrusions, revolutions, holes and Boolean bodies,
including component through-cutters, use that path. `blank.sv` expresses the finite
bevel rim through spherical and conical boundary components. Its STEP agrees with
the experimental pinion blank under both directed native Boolean differences (no
remaining solids); volumes agree within 5e-9 mm³. This is a construction comparison,
not a new geometric error certificate. Motion placement now supports indexing through
`solid indexed(tool, under: indexing, at: i * 360deg / teeth)` in ordinary components.
Native builds export both STEP and STL from that same solid, constructing once when both
formats are requested. Encoded STL shell checks reject cracks, pinches and float32 collapse;
all native faces must be meshed. Native STL uses millimetres and a 0.01 mm linear tessellation
setting, not a complete error certificate. The six-hole example passes this native path;
the legacy mesh backend's triangulation defect remains recorded separately.
The continuous generating removal declaration is connected to material evaluation; its native
boundary construction is the next integration task. Along-guide lofts and generating-motion sweeps remain unsupported
by this native host. No union of sampled tool poses substitutes for a continuous sweep.

The native indexed-pattern STL has 2,096 triangles and passes exact encoded shell checks;
the legacy path produced 72,027 triangles with unpaired edges. The declarative bevel blank
exports both formats in about 0.53 s on the current host, with 7,344 STL triangles and STEP
volume 12038.990933011 mm³. Independent encoded-edge and volume checks pass. These are
small integration checks, not completed tooth generation or a machining accuracy claim.
The experimental `tooth_space.py` still takes tooth-specific solved grids and constructs
its own closure surfaces. Moving those grids into the public compiler unchanged would
preserve that bypass; the generating source and motion must determine the native boundary.

The source-to-material bridge now exists: `SpatialField::read` accepts ordinary full
revolutions with convex line/arc profile loops, holes, Boolean bodies and motion placements.
`SweptField::read` takes that solid and a named motion from the solved sketch. The rounded
crown and bevel blank need no name-specific coordinate adapter on this path. Source tests
compare the crown against independent analytic membership and a sphere sweep against a
closed-form torus. Concave profile loops, prisms and partial revolutions are still refused.
These numerical conversion checks do not certify source-solve error.
The public declaration is now `solid removal(tool, under: generating, from: -30deg, to: 30deg)`.
Its component parameters, copy/delete graph and continuous material evaluation are connected.
`MaterialField::read` handles the full static/swept/placed Boolean graph without pose sampling.
Tests cover finite end caps and indexing one swept cutter through an ordinary component.
Boundary export still refuses these graphs explicitly; native sweep-boundary construction
remains ahead, so this is not yet public gear solid export.


`rust/examples/spiral_bevel/gears.sv` now instantiates both complete material bodies through
`MatchedPair`. `GeneratedMember` subtracts a continuous cutter sweep at each tooth index;
its tooth count and module come from the ordinary configuration module. The gear cutter is
built by the owning reference component, which passes its private datums explicitly to
`ComplementarySpace`. Each active rounded flank is closed with ordinary lines, revolved,
and intersected with the neighboring crown. The finite radial closure lies beyond the
reachable blank under every apex-centered generating rotation. No private member access
rule or gear-specific compiler operation is added.
The source cutter is compared with the independently assembled active-flank fields at
thousands of points in the reachable domain. Both complete source bodies match the separate
member evaluator at 48 retained/removed probes across the 24:48, module 2 mm and 28:49,
module 1.5 mm configurations, including distant tooth indices and blank ends. These are
finite regression checks, not complete engagement or a supported-parameter-domain proof.
Analytic profile ordering now follows shared source vertices instead of reconstructing
arc junctions from trigonometry; this preserves topology in the presence of solve residuals
without welding or replacing the analytic curves.

The native cutter alone exports to `build/exports/solvent-gear-space.step` and `.stl`:
STEP volume 1164.899457706113 mm³, 4,962 triangles, independently checked STEP validity
and exact encoded STL edge pairing. This is the stationary generating cutter, not a gear.
The integration exposed and fixed solid references capturing motion indices before the
motion graph's dependency ordering was finalized. It also required the regularized CSG
identity `A - (A - B) = A intersect B` for shared immutable field nodes; the raw field
expression retained zeros on discarded boundaries and obstructed sweep separation.


The next boundary-construction stage is `solid::SweepContacts`: it reads the declared
sweep, gathers the generating solid's revolved faces through Boolean and fixed-placement
nodes, and retains the complete source material classifier. Shared faces are collected
once per fixed pose; axis diameters contribute no surface. For a meridian station under
an instantaneous rigid motion, `RevolvedSurface::contacts` solves the contact equation
`A cos(theta) + B sin(theta) + C = 0` directly. It retains both regular branches, respects
source angular spans, and reports stationary or near-double-root events explicitly.
These are smooth-face candidates, not a finished swept solid: source-hidden portions,
sharp-edge sweeps, endpoint caps, singular events and global self-intersections still
need to be resolved before closed solid assembly.

This follows the contact-set/B-rep decomposition described by
[Adsul, Machchhar and Sohoni](https://arxiv.org/abs/1404.0119), with their
[sharp-feature extension](https://arxiv.org/abs/1405.7457) relevant to the cutter's Boolean
edges. The direct trigonometric reduction here uses the existing analytic revolutions
and exact first-order rigid-motion evaluator; it does not require a gear-specific
meshing equation or a union of sampled tool poses.
The source-derived candidates match 120 independent characteristic points on both flanks
and fillets of both members, including the gear's phase-shifted neighboring cutter.
Generic tests cover an orbiting sphere, compound rotation and translation, fixed source
placements, periodic branch identity, angular restrictions and explicit degeneracies.
The native C++ bridge now interpolates regular candidate charts into B-spline faces.
Its integration tests read the actual `gears.sv` material definitions, calculate contacts
in Rust and fit sixteen flank/fillet charts in OCCT. The observed withheld-point error
is about 0.00013 mm on meridian [0.05,0.95], roll [-0.3,-0.2] rad; tangent planes are also
compared. An independent orbiting-sphere/torus check exercises refinement. These are
partial regular charts, not full-domain or export-accuracy certificates. A rectangular
chart near zero roll encountered a ring with no contacts; the regression retains that
case instead of interpolating across the missing domain. The bridge primitive is used
by tests only until source trimming, chart events and closed-solid assembly are connected.
Public continuous-sweep STEP/STL export remains explicitly unsupported.

An alternate contact chart now holds the source `(u,v)` parameters and solves for
motion time. `Family::normal_velocity` reduces a rotation viewed from another fixed-axis
rotation to a sinusoid in the source angle, including offset axes, signed rates and phase.
`SweepContacts::at_source` enumerates isolated roots over the declared roll interval and
checks each against the original differentiated motion. Constant-contact and near-double
roots remain explicit degeneracies; nested relative motions are unsupported by this
temporal reduction. Root enumeration has a finite budget and never returns a truncated set.
This complements the original chart; it does not guarantee coverage or surface regularity.

Both charts now reproduce the same 120 independently calculated crown characteristics,
including the neighboring gear cutter. Tests compare the temporal coefficients and roots
against direct matrix differentiation with offset/tilted axes, negative and zero rates,
multiple windings, endpoint roots and observer phase changes. A native fitted face over
source `(u,v)` = `[0.65,0.8] x [0.7,0.8]` crosses the pinion inner-flank join where the
original `(u,time)` chart folds: both old branches are covered, with observed withheld
position error about 0.0000016 mm and matching tangent planes. This resolves that local
coordinate failure without claiming the full swept boundary is constructed or trimmed.

The native bridge also reads the stationary cutter's actual trimmed edge/face incidence,
including edges created by Booleans rather than declared profile edges alone. It returns
edge positions, unit tangents and both outward incident-face normals, with measured
curve/face discrepancies. Periodic seams and collapsed pole edges stay explicit; tangent
queries at collapsed edges fail. A shared native session/exception header separates the
topology reader from solid construction/export. Tests reuse the CLI's recipe builder:
cube and cylindrical-hole cases check Boolean-created edges and material orientation,
sphere/torus cases check poles and seams, and 84 samples on the actual cutters' 10/18
edges agree with Solvent's independent source material field. These are source-boundary
inputs for sharp-edge sweeping and trimming, not a completed swept-boundary arrangement.

The boundary reader now measures signed dihedral using oriented face boundaries, so a
convex cube edge, concave blind-hole floor edge and smooth periodic seam stay distinct.
`envelope::edge_contact` implements the local normal-cone condition from the
[sharp-feature sweep framework](https://arxiv.org/abs/1405.7457): for a convex edge, a
nonnegative combination of incident outward normals must be perpendicular to velocity.
It returns that candidate normal and moving position. Smooth and concave edges supply
no regular sharp-edge face; collapsed cones, inconsistent incidence and tangent velocity
on convex edges return explicit errors. This is an interior-time candidate test, with
endpoint caps and global visibility still separate.

Independent box/diagonal-translation and thin-wedge checks cover orientation, scale,
inactive motion and degeneracies. A native rotating cube edge generates the known
cylindrical envelope, with observed withheld fit error about 0.00000073 mm. On the actual
pinion/gear cutters, 17/37 sampled sharp contacts pass the local test; 106 source-material
checks at nearby times verify that strict interior-cone samples are locally exposed.
An explicit rotating-box negative control retains a normal-cone candidate that another
pose covers, so candidate eligibility cannot stand in for swept-material classification.
Those samples do not certify full-interval visibility. Native candidate-domain coverage,
global trimming and closed-solid assembly remain unfinished.

`MaterialEvaluator::probe` now checks candidate point boxes against the entire material
graph and declared motion intervals. Strict field margins certify interior/exterior balls;
otherwise it checks outward-rounded offsets on both sides of the supplied direction.
Opposite material signs bracket a boundary within the requested distance. They do not
establish a unique crossing, the surface normal, topology or a whole-face error bound.
Field zeros and exhausted or ambiguous searches remain unresolved.
Generic sphere/torus checks cover finite caps swallowed by extended motion, Boolean-cut
orientation, uncertain input boxes, phantom field zeros and budget exhaustion. At one
station on every eligible native cutter edge and three roll times, the actual source pair
produces 14 outward brackets and four covered candidates (all four on the gear). Every
query covers the full declared sweep; a separate static-source evaluation confirms each
covering witness. The 18 candidate checks take about two seconds together, including
source solving and native cutter construction. Their 0.01 mm offset is a local probe
distance, not a certificate of exported gear accuracy. Tracing the boundaries between
covered and exposed regions and constructing their native trim curves remain ahead.

The bridge now delegates intersection-curve construction to OCCT's face Splitter.
It keeps every source-face fragment and its original supporting-surface chart, with
explicit native trim membership and diagnostics. Fitting and trimming live together in
`surfaces.cpp`; tests share the same native session/recipe constructor as the CLI.
Small checks cover a known parabolic split, a closed spherical trim with a holed face,
and material probes on the fragments of a planar-cut swept-sphere chart. For both gear
members, a regular source-parameter contact chart crossing the toe sphere is now split
by that sphere constructed from the declared solid. Independent sphere-field checks at
169 withheld points per member agree with fragment membership; observed local fit error
is about 0.0000023 mm. The eight native surface tests finish in about 0.42 s, excluding
compilation. This supplies native trim construction for provided intersecting faces;
automatic chart coverage, intersections separating globally covered sweep regions,
consistent final face selection and closed-solid assembly remain unfinished.

Automatic smooth contact-domain discovery now starts from every source patch's
entire `(u,v,time)` box. Revolved-surface position, tangent and normal enclosures feed
the interval version of the existing relative-rotation contact equation. A cell is
discarded only when its complete equation enclosure excludes zero. Opposite endpoint
signs over the entire free-parameter box, a nonzero derivative in the dependent parameter,
and a nonzero source normal establish a unique contact root. The search can solve for
motion time or source revolution angle without hand-selected flank ranges. Source poles, chart
events and exhausted regions stay explicitly unresolved in the returned partition.
This establishes a contact graph, not regularity or exposure of its mapped surface.

The search refines free parameters after proving monotonicity and rotates work among
source faces. A single fixed-axis rotation's time-independent equation now produces angular
charts. Their derivative includes both position and normal changes, with shifted axes
retained in the product rule. Charts may overlap across internal subdivision planes;
their dependent intervals contain the corresponding partition interval. Full revolutions
also allow local angular charts through their coordinate seam, in `[-0.25,1.25]`;
partial revolutions, restricted spans and motion endpoints keep their declared limits.
These charts evaluate unwrapped angles directly, without assuming exact periodic equality
for binary64 TAU. The original source-domain partition is unchanged. If extension loses
the derivative bound, subdivision can refine that coordinate instead of indefinitely
refining only the free coordinates. `SweepContacts::at_chart` evaluates either chart
through the ordinary analytic root solver and refuses missing/ambiguous roots or non-chart
cells. Poles, endpoint events and unresolved chart regions remain explicit; coincident
seam geometry still needs reconciliation during final B-rep assembly.

The sphere domain/fitting test takes about 0.1 s and performs eight angular fits,
including seam charts, against the independent torus equation. It retains 524 cells after
1,047 evaluations at depth 10; sampled regular contacts at both angular endpoints have
charts, while poles remain unresolved. At a 30,000-cell limit per member, the current
pair produces 845 time/82 angular charts for the pinion and 222 time/186 angular charts for
the gear, including 18/15 seam charts, with bounded search/audit times about 5.2/2.1 s.
Every source face receives work and pending domains remain represented; those budgets do
not finish the atlas. Sixteen selected gear charts, including four seam charts on each
member, pass native fitting checks at withheld points. Full event/seam coverage, overlap reconciliation, material
selection and closed native assembly remain necessary before public continuous-sweep export.

Interval trigonometry now evaluates its Taylor polynomial at the box midpoint and bounds
the remaining displacement by angle addition. This avoids the old dependency blow-up
far from zero while retaining outward rounding and explicit Taylor remainders. A separate
exact-rational checker passes 1,841 arithmetic/trigonometric records, including sampled
point checks of the new whole-box bounds. This is numerical infrastructure validation,
not a completed swept-boundary or final machining-error certificate.

Finite-motion endpoint candidates now come directly from the declared native cutter.
`Session::sweep_caps` constructs its static recipe once, places complete copies at the
declared start/end parameters, and retains every native face, trim and orientation at
both endpoints. The pose-to-millimetre matrix is shared with ordinary placed-solid
construction. This accepts static prisms as well as revolutions and Boolean cutters;
it performs no sampled union or fitted reconstruction of endpoint faces.

A separate native face query uses each face's finite UV trim bounds, classifies holes
and outside regions explicitly, and returns the face's oriented normal. The existing
supporting-surface query keeps its original parameterization. Tests cover a drilled box,
shifted relative-rotation gear cutters, and a finite sphere sweep in millimetres and
inches. The sphere's independent distance-to-circular-arc calculation agrees with eight
covered/eight exposed endpoint samples per unit configuration; construction plus those
checks takes about 4 ms. For the default pair, 184 native endpoint/source field and normal
checks pass. Whole-motion probes at 26 selected endpoint samples report four covered/six
outward brackets for the pinion and thirteen covered/three outward brackets for the gear,
with no unresolved results among those samples. These are point/local-neighborhood checks,
not whole-face visibility. Endpoint contact-curve trimming, global face selection and
closed assembly remain required; public continuous-sweep export still refuses incomplete
boundary construction.

The acceptance work remains:

1. **Reproduce the reference pair:** one command from recorded source and tooth-count/size
   parameters to both members, assembly placement and a report. Use the existing 24:48 pair
   as the baseline; experimental parameter overrides do not establish a supported domain.
2. **Verify engagement:** contact, material sides and interference across a tooth period,
   with explicit coverage and wrong-phase controls. Sampled success is a baseline, not
   evidence about every intervening phase.
3. **Verify actual export accuracy:** use one end-to-end budget for source solve, fitting,
   Boolean/export conversion and tessellation. The user-selected target is **0.001 inch
   (0.0254 mm)** for the complete exported geometry. Mesher settings and separate face
   bounds do not establish that combined accuracy. Earlier 0.001 mm checks remain useful
   tighter component checks; they are not the required end-to-end target.
4. **Complete parametric Solvent integration:** ordinary components and general solid
   operations produce both public solid outputs; multiple configurations pass the same
   checks and unsupported configurations receive useful diagnostics.

The next milestone is public parametric Solvent solid construction and export. Independent verification
means reproducible checks against independently implemented documented generating equations,
with stated tolerances and visible unresolved results. Formal certification of every CAD
operation is not automatically a prerequisite. Further analytical/spline attachment proofs
and general F-rep meshing are parked unless a concrete gear acceptance failure requires them.
The existing proof results below are retained evidence, not the current task ordering.

`experiments/cad-backend/build_pair.py` connects the experimental source exporters, native
construction, STEP/STL export, encoded mesh audit and sampled engagement checks. It records
source/version provenance, both wrong-phase controls, and remaining acceptance gaps. This
test-adapter workflow is a bridge to public integration, not the finished language interface.
The first fresh workflow run reproduces both inspection meshes and passes its sampled CAD
checks and wrong-phase controls. A separate nominal engagement refinement now reaches
65 phases and nine face-width stations with no sampled interference, including back-cone
boundaries. These checks remain sampled; the full-period contact and whole-geometry
acceptance questions must retain their stated coverage limits.
The finer STL exports now also have complete triangle-to-support distance bounds below
0.019985 mm, covering interiors as well as encoded vertices. Finite trimmed-face membership,
reverse coverage and transfer to nominal generating geometry remain separate from this
support-distance result; it is not yet the complete 0.0254 mm export-accuracy gate.
Finite rectangular-domain checks now transfer the forward bounds to actual finite
CAD faces for all 178,793 triangles on the 360 spline faces. Exact identity with
the earlier reference coefficients is checked. The remaining 81 analytical faces
(18,505 triangles), reverse coverage, and the prior nominal-reference checks' own
source/material limitations remain outside that result.

## Backend and accumulated evidence

**Current backend decision:** evaluate a conventional CAD kernel before further investment
in general F-rep boundary recovery. The user identified established industrial gear-to-CAD
workflows as a likely reusable backend for Solvent's declarative frontend. The
[Open CASCADE tooth-space trial](../experiments/cad-backend/README.md) now fits existing
generated flank/fillet references, constructs candidate solids, subtracts them from exact
blanks, and checks STEP round trips. The F-rep remains a material definition and verification
asset; completing its general-purpose mesher is no longer a prerequisite for this alternative
export path. The historical extraction gates below remain applicable to that meshing work.
Neither backend path has yet delivered the verified complete matched pair.

The CAD trial now produces complete 24-cut pinion and 48-cut gear rim candidates, each
passing kernel validity and STEP round-trip checks. Forty probes derived from the STEP
surfaces agree with the continuous indexed material definition and pass the independent
rational whole-roll audit; 480/960 rotated CAD probe classifications also pass. Both
inspection STLs pass independent exact encoded topology and embedding checks. The actual
STEP pair now also reports no volumetric overlap at five assembly phases, while a deliberate
0.001-radian gear phase error produces a detected 0.71536 mm³ overlap. All 250 analytical
contact positions agree with both STEP boundaries within 0.001 mm, and 138 interior contact
locations have complementary material on opposite sides. These are sampled CAD checks;
continuous interference and complete surface-error/material coverage remain acceptance work,
followed by public Solvent solid integration. These successful candidate exports do not
certify production accuracy or complete the matched-pair goal.

The [closure-support audit](../experiments/cad-backend/SUPPORTS.md) now gives exact rational
whole-parameter distance bounds for all twelve tooth-space closure B-splines. Across 3,072
polynomial patches the largest bound to a nominal sphere/cone is 8.2e-6 mm, below the
0.001 mm target. This closes the closure support-distance question for the extracted
binary64 surfaces, not finite trimmed coverage, later Boolean/export error propagation,
or generated flank/fillet accuracy. Those limits remain part of complete surface acceptance.

The [generated-fillet audit](../experiments/cad-backend/FILLETS.md) additionally establishes
whole-parameter correspondence within 0.001 mm for all four fillet support surfaces against
the ideal common-crown references with enclosed solved coefficients. Exact cover checks
accept every original Bézier rectangle; no cells remain unresolved. Quadratic Taylor bounds
reduced each complete audit to 556–576 cell checks (81–86 seconds in concurrent runs).
The [working-flank audit](../experiments/cad-backend/FLANKS.md) now likewise bounds all four
complete flank supports within 0.001 mm. A locally unique tip root and implicit derivatives
through third order allow the same correspondence checker to cover the moving trim.
The pinion sides each need 768 cell checks (about 86 seconds); the gear sides each need
256 (about 30 seconds). All complete covers pass and an interior coefficient distortion is
rejected. Source/assembly error, finite trimmed coverage, STEP-reader conversion, subsequent
indexed operations and global material/mating acceptance remain. These completed nominal
surface gates do not yet establish a finished production pair.

The [finite-face and indexed-surface audit](../experiments/cad-backend/INDEXED-SURFACES.md)
now verifies that all twenty tooth-space faces and all 360 indexed B-spline faces cover
their complete bounded parameter rectangles. Exact coefficient comparison bounds every
indexed support against an exact rotation of its original support within 2.25e-11 mm.
Combined nominal accuracy remains below 0.001 mm on those indexed faces. Analytical blank
faces and their trims, separate 3D edge consistency, source/reader error and global
material/mating checks remain; this does not close the full-solid acceptance gate.

The [analytical-face audit](../experiments/cad-backend/ANALYTICAL-FACES.md) additionally
bounds all 81 remaining sphere/cone faces against their nominal blank supports within
1.10e-11 mm. All 1,188 parameter curves contribute finite slab bounds, including six
polynomial extensions beyond imported knot endpoints. The audit takes approximately
0.85 seconds. With the earlier indexed B-splines, all 441 faces have nominal support
distance bounds below 0.001 mm. Separate 3D edge consistency, analytical trim topology,
source/reader error and global material/mating coverage still remain.

The [shared-edge and vertex audit](../experiments/cad-backend/EDGES.md) closes the next
local boundary-consistency gates: all 2,634 edge/face incidences pass whole-interval
correspondence within 2e-6 mm, and all edge endpoints agree with their 876 named vertices
within 1.56e-8 mm. Every one of the 441 face wires closes through its vertex identities;
each vertex has one connected link cycle. Conservative nominal-face/edge/vertex distance
chains stay below 0.001 mm. Geometric trim simplicity and embedding, source/reader error,
global material coverage and continuous mating remain separate; the pair is not yet
production-accepted.

The [analytical trim-curve audit](../experiments/cad-backend/TRIM-CURVES.md) now proves
individual UV injectivity for all 1,188 analytical trim curves and strict UV separation
for all 106,004 non-neighboring pairs in their wires. It also records 614 small nonzero
corner gaps without snapping them. Adjacent-curve separation, corner treatment and
periodic chart identification remain necessary before claiming simple embedded trims.

The subsequent [closed-representative audit](../experiments/cad-backend/TRIM-LOOPS.md)
constructs and proves simple closed UV loops for all 81 analytical faces, with at most
6.78e-11 mm boundary displacement on their surfaces. Seventy-five representative face
regions also have injective surface charts. Six periodic seam faces require explicit
identification, and shared 3D topology realization and global embedding remain open.
This is a bounded representative of the tolerant STEP data, not an assertion that its
raw parameter curves close exactly or that every reader uses the same representative.

The [periodic-seam audit](../experiments/cad-backend/TRIM-SEAMS.md) now gives all six seam
faces embedded annular representatives with exact full-revolution identification and at
most 1.01e-10 mm boundary displacement. All 81 analytical faces therefore have individually
embedded representatives. Shared 3D topology realization, inter-face embedding, global
swept material, continuous mating and source/reader accuracy transfer still remain.

The [whole-spline embedding audit](../experiments/cad-backend/SPLINE-EMBEDDING.md) now
proves injectivity and regularity of all 360 indexed polynomial spline faces on their
complete finite rectangles. A single strongly monotone projected map per face excludes
global self-intersections as well as local folds. Together with the analytical-face
representatives, all 441 faces have individual embedding evidence. Inter-face embedding
and a consistent shared 3D realization remain unproved.

The [spline-pair separation audit](../experiments/cad-backend/SPLINE-SEPARATION.md) now
proves all 35,532 nonincident spline-face pairs disjoint, with a conservative minimum
separation bound of 0.00364 mm. The full check takes 35 seconds and only 48 pairs need
subdivision. The 288 incident spline pairs remain explicitly unverified, as do pairs
involving analytical faces and the common shared 3D boundary realization.

The [spline-join audit](../experiments/cad-backend/SPLINE-JOINS.md) now constructs one
consistent representative per spline face and proves all 288 intended joins as globally
injective combined rectangles. Maximum face movement is 7.76e-10 mm; all 35,532 nonincident
separation bounds survive the change. Each member's complete spline region is embedded
with consistent shared curves. Analytical blank attachment and the closed 3D boundary,
global material, continuous mating and source/reader accuracy remain open.

The deliverable remains one fully specified, parametric matched pair from a documented
generating system, independently verifiable and exportable at a stated geometric accuracy.
Manufacturing planning remains separate. The earlier functional-solid investigation below
provides the material definition and independent verification assets. Explicit fields,
continuous sweeps and a first distance-bounded extractor exist; the current export path uses
the CAD kernel. The [implicit meshing comparison](implicit-meshing-methods.md) records possible
future extraction work, outside the current critical path.

## Solid definition

Let B be the nominal blank, C a closed generating solid, and M(t) its rigid motion in
the member's coordinates over a declared finite interval I. Define removed material as

```
W = union over t in I of M(t) C
G = regularized difference(B, W)
```

Tooth indexing adds the rotated copies of W. Each member must use its own appropriate
generating volumes and relative motion. The current gear reference uses separate complementary
sections for its two mating flanks; subtracting one arbitrarily chosen crown solid from both
blanks would not preserve the documented construction.

For continuous fields negative in material, the corresponding membership expressions are

```
w(x) = min over t in I of c(inverse(M(t)) x)
g(x) = max(b(x), -w(x))
```

These expressions determine signs. Boolean combinations need not remain signed distance
functions; field residuals alone do not bound distance to the output surface. Regularized
solid semantics must also remove isolated sheets/points from degenerate Boolean results.
Neither a sign expression nor regularization guarantees a manifold boundary.

The boundary becomes a derived representation of the material set. Overlaps of the moving
generator contribute once, and internal portions of local envelopes do not become exposed
faces. Start/end motion positions contribute where exposed. This addresses assembly and
branch selection at the definition level; numerical evaluation and extraction still have
to preserve that definition.

At a smooth exposed boundary attained at an interior roll value, stationarity of the field
along the generating motion yields the same normal-relative-velocity equation used by the
existing envelope implementation. That local equation is necessary, not sufficient: another
roll value may put the same point inside the swept volume. This gives the current analytic
tooth work a role as boundary witnesses and independent differential checks.

## Earlier representation work breakdown (not the current execution order)

| Subproject | Deliverable and completion evidence | Existing assets and missing work |
| --- | --- | --- |
| **1. Functional solid contract** | A composable material-set evaluator for analytic primitives, rigid transforms and regularized Booleans. Point and box queries distinguish established inside/outside from unresolved bounds. Known primitive and Boolean cases verify semantics independently of a mesh. | `PlanarField` and `RevolvedField` bound explicit generating sections; `SpatialField` adds static spatial Booleans and fixed poses. `MaterialField` now composes completed swept operands, owning the blank and indexed cuts with per-sweep uncertainty and box-aware memoization. `RevolvedRegion` remains an independent floating-point comparison. Source-error transfer and editable solid-graph integration remain; preserve provenance and units. |
| **2. Continuous swept volumes** | A generating solid under a finite named motion, with conservative bounds over the complete motion interval. Known sweeps and adversarial multiple-minimum cases verify that uncovered time intervals cannot be silently discarded. | `SweptField` now owns a typed spatial field, named-motion snapshot and finite domain. Its evaluator uses interval poses/speed bounds and `interval::minimum::enclose`, retains unresolved status, caps its pose cache and exposes evidence observations. Pair witnesses pass, but an indexed full-crown closure produces a verified overcut. Local envelopes cannot establish global membership. |
| **3. Boundary extraction and export** | Derive an adaptive mesh from the functional solid with a stated spatial error budget, sharp-feature handling and checked output topology. Detect unresolved/thin features rather than silently erasing them. | A first core extractor derives finite support, covers all spatial cells and checks both boundary-distance directions before accepting one closed shell. An independent rational sphere audit verifies its spatial certificate; a swept-sphere torus and a hidden disconnected feature exercise the runtime. Fine gear extraction, embedding/topology preservation and sharp-feature handling remain. No manually supplied seam network defines the material. |
| **4. Documented matched-pair geometry** | Specify both blanks, each generating volume, indexing, finite roll extent, thickness/backlash convention and supported parameters. Establish that the active exposed boundaries have the intended conjugate geometry and identify undercut/interference failures. | Reuse the common-crown derivation, declarative rounded sections, NASA regression and nine configurations. Check material sides, generator closures and roll endpoints explicitly; existing local-envelope success is insufficient. |
| **5. Solvent integration** | Express the whole construction using ordinary solids, components, named motions, relations and parameters. Both members are usable `solid` outputs through the existing application/export interface. | Extend the existing solid evaluator and syntax deliberately. A volume swept under rigid motion generalizes the current profile sweep but has different inputs; settle semantics before grammar. Keep display and export accuracy outside design parameters. |
| **6. Independent verification** | A reproducible report for the final pair covering source accuracy, global swept membership, mating contact/interference, exported surface deviation and encoded topology. Verification distinguishes proved bounds from samples. | Preserve independent reference equations and rational interval audits. Positive interval covers now support independent whole-roll material-sign checks at selected member points. Source-solve error transfer and whole-space/contact/export checks remain. Mesh topology and nominal surface regularity each prove only their own contracts. |

Projects 1 and 2 define the representation; project 4 supplies its first demanding real
input and can be investigated alongside them. Project 3 consumes the bounded evaluator.
Project 5 integrates those contracts into Solvent. Project 6 supplies acceptance evidence
throughout and audits the final result. This is a work breakdown, not a request to develop
six general-purpose systems before producing a gear.

The first [spherical-contour experiment](implicit-meshing-methods.md#spherical-contour-and-chart-experiment)
now traces complete mean-radius contours of both members, using the full indexed material
field to correct each vertex. The default 2 mm candidate target produces 96 pinion edges
and 192 gear edges in approximately 4 and 16 seconds of tracing. Its
[candidate shell extension](implicit-meshing-methods.md#candidate-shell-experiment) now
spans the face width and source-derived end/back boundaries with adaptive conforming
triangles. At the same sampled target, the complete pinion candidate has 2,234 triangles
and takes 25 seconds; the gear has 4,556 triangles and takes 97 seconds. Runtime and
independent encoded-STL checks establish closed genus-one mesh topology. The surface chart
still assumes unique polar crossings and an annular material boundary. A subsequent
[exact STL audit](stl-embedding-verification.md) establishes embedding of these encoded
meshes, but complete material coverage, whole-surface error and sharp-feature acceptance
remain unproved. The full shell uses a structured chart, not an advancing-front march;
its regular, rough tooth triangulation prompted the
[general advancing-front direction](implicit-meshing-methods.md#next-method-general-curvature-adaptive-advancing-front).
The user explicitly requires geometry-independent discovery/adaptation, with no supplied
tooth-tip curves or gear-specific meshing rules. One marcher must traverse the existing
complete-member F-rep, with no separate analytic back/end meshes or prescribed annular
connectivity. The decomposition in the chart baseline must not carry into that mesher.
These results do not accept a final gear solid.

The immediate iteration gate is now [small-case discovery](implicit-meshing-methods.md#small-case-discovery-workbench-and-latency-target):
complete sphere and cube extraction under 100 ms each, with compilation measured
separately. The test-only general front closes a 414-triangle sphere in about 56 ms.
Local retriangulation now closes the 1,898-triangle torus (about 227 ms), with an
independent exact embedding audit. Cube sharp transitions remain an explicit failing
acceptance case. Keep high-precision gear runs out of this loop until the primitive method is
efficient and robust. Sphere timing alone does not establish that milestone.
An isolated field-only feature probe now recovers local cube edges/corners (including
rotated cases) and a curved Boolean crease while rejecting smooth-sphere controls.
Automatic feature acquisition now crosses right-angle and thin wedges from an ordinary
incoming patch, including rotated cases. Whole-cube growth still exhausts its query
budget. Bounded seed search handles centered and translated thin
tetrahedra. After the [mechanical refactor](solid-module-review.md), direct field-gradient
sampling resolves the local apex/projection failure, including rotated/scaled and tenfold
thinner cases. A shared quality policy now permits angles forced by recovered corner
branches while rejecting avoidable slivers, passing rotated/scaled acute-corner controls.
Shrinking a probe past a distant fitted intersection now recovers nearer creases and
advances the tetrahedron front from 8 to 40 triangles. Interval separation checks for
nearby distinct faces then reach 75 triangles, 11 open edges and about 1.41 s. An exact
triangle-pair audit finds no improper intersections in that partial candidate after
binary32 rounding. Subsequent shared-face prediction and interior-orientation checks
produce 77 triangles with 11 open edges. That candidate passes source-coordinate pair
checks but acquires three intersections after binary32 rounding; it is unaccepted.
Complete tetrahedron closure still fails. The cube and tetrahedron remain
unaccepted; local feature recovery does not establish the small-solid discovery milestone.

The subsequent [isolated external-mesher comparison](../experiments/implicit-mesh/README.md)
tests Fidget, Manifold and libfive against the same independent small-solid checks. None
has passed the full fixture set, and none is integrated with the continuous swept evaluator.
A libfive/Manifold/CGAL sequence now produces a passing rotated thin tetrahedron, including
exact encoded-STL embedding and sampled source checks. The
[repair investigation](../experiments/implicit-mesh/CGAL.md) records failures on other sharp,
thin and disconnected inputs; one native repair reports success while discarding most of
the source body. This is an unresolved general boundary-extraction dependency, not completion
of the gear or selection of a production backend.

## First experiment and decision gate

The [continuous-volume experiment](continuous-volumes.md) records the initial pinion
probe, the generic interval refiner and motion bounds. Explicit analytic fields remove
the assumed material-evaluation error band. The default gear's two complementary generating
sections now pass the same selected whole-roll checks when combined; the independent
rational checker audits attained witnesses for both members. A subsequent indexed check
finds a definite gear overcut from a neighboring generator's inactive outer wall. The full
closed reference sections are therefore a rejected final generating-volume construction;
their closures must be resolved before proceeding to boundary extraction. A replacement
candidate intersects the two one-sided active boundaries after shifting the neighboring
section by the crown pitch. Its selected whole-roll/indexed boundary and interior checks pass. Complete-member workbench
queries now include finite blanks and every interval-evaluated body index; whole-space
coverage and the remaining acceptance evidence are still required. Source-error transfer, whole-tooth
coverage and the remaining acceptance evidence below are not yet complete.

Implement one continuous tooth-space subtraction for the default pinion, bounded by its
existing blank, using the solved rounded generating section and named relative motion.
First establish material membership, before building a new mesher or extending syntax.
The closed generator's inactive faces and the selected roll endpoints must be checked:
neither may introduce an unintended active tooth boundary. Extending the interval must
either leave the intended region unchanged with evidence, or reveal that the original
finite interval was part of the shape definition.

Acceptance evidence for that experiment:

1. Independently known ball translations/rotations and multi-pass sweeps exercise endpoint
   minima, competing minima, overlaps and thin missed-between-sample regions.
2. Existing analytic flank and fillet witnesses agree with the volume boundary where they
   are exposed; points on an envelope branch covered at another roll are rejected.
3. Classification across the entire declared roll interval has explicit uncertainty and
   refinement controls. Exhausting work returns unresolved, never guessed material.
4. Runtime and refinement behavior are measured at tooth-scale spatial tolerances, with
   source numerical error distinguished from sweep evaluation error.

If that gate succeeds, extend the same representation to all indexed spaces and both
members, then boundary extraction. The experiment does not replace the full-pair goal.
Pause further cap/seam/face assembly as the primary solid construction route while this
decision is assessed. Preserve existing analytic and topology work as verification assets;
the current surface-seam additions remain unfinished and must not be presented as released.

## Research basis

The continuous implicit sweep formulation and its boundary-tracking use are described by
[Sellán, Aigerman and Jacobson, *Swept Volumes via Spacetime Numerical Continuation* (2021)](https://www.dgp.toronto.edu/projects/swept-volumes/swept-volumes-low-res.pdf),
equation 1 and section 4.3. Their continuation method is a useful acceleration reference;
the discussion acknowledges the possibility of missing global minima. We cannot treat
its empirical robustness as the independent accuracy certificate required here.

The gear generating system remains the common-crown construction documented in
[the design record](spiral-bevel-design.md), based on
[Chang, Huston and Coy, NASA TM-101449 (1989)](https://ntrs.nasa.gov/api/citations/19890007877/downloads/19890007877.pdf).
The proposed volume formulation must be checked against that system; it does not change
the pair into an arbitrary helical groove or specify a physical machining process.
