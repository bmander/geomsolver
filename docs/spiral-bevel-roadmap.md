# Spiral bevel gears: subprojects and representation decision

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

The deliverable remains one fully specified, parametric matched pair from a documented
generating system, independently verifiable and exportable at a stated geometric accuracy.
Manufacturing planning remains separate. This document redirects the next investigation:
evaluate a solid defined by continuous generating volumes before extending manual assembly
of analytic faces. Explicit fields, continuous sweeps and a first distance-bounded extractor
now exist; a completed functional gear export does not. The
[implicit meshing comparison](implicit-meshing-methods.md) directs the next extraction work
toward documented adaptive and continuation methods, with independent acceptance evidence.

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

## Separately reviewable subprojects

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
