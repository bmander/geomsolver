# The native export without OCCT: plan

Move what the native export takes from Open CASCADE into Rust, in the core, so that the
fabrication-grade path — an exact B-rep, a STEP master, an STL meshed from it within a stated
tolerance — needs no C++ kernel, runs wherever the core runs (the browser included), and is ours
to reason about. **Not a general CAD kernel**: the B-rep, geometry, Booleans and writers the
admitted class uses first (phases 0–5), then — a rung at a time, each decided on its own — every
static solid the language defines (the scope ladder), accepting what we choose to accept and
refusing the rest by name, the way `admission::admit_body` already does. Continuous sweeps outside
the generating class stay on the field mesher. OCCT stays, as a
**test-only oracle** in the slow tier — the role `nalgebra` plays for the linear algebra.

## Why

- **The browser.** The core is wasm; OCCT is not. Today a fabrication export needs a native
  `solventc` built with `OCCT=1`; the app can offer only the field mesh. With the kernel in the
  core, `File ▸ Export STEP` and a toleranced STL are the same code in both places.
- **The contracts.** The mesher's deflection is a control and not a bound (hence `mesh_sag` and
  meshing again); `BRepCheck_Analyzer` is unreliable in parallel; `UnifySameDomain` widens
  tolerances on shared vertices; the read-back needs `ShapeFix` to be orientable. Each is a
  workaround we wrote around behaviour we cannot change. Our own mesher can refine to a measured
  sag; our own B-rep can be built valid instead of checked valid.
- **The floor phase 6 named.** What is left of the export's time is OCCT's, serial or nearly: the
  sector's split (2.0–2.7 s, a third of it `GeomLib_CheckCurveOnSurface`'s particle swarm, which no
  option turns off), the union (3.8 s for the gear: sewing 48 sectors, unifying, checking) and
  `BRepMesh`. A pattern built with shared topology is never sewn.
- **The rules.** The core has no dependencies; the C++ toolchain and OCCT are the project's
  heaviest build requirement, and exceptions from it are caught at an ABI we maintain.

**What it costs.** OCCT's maturity on inputs nobody foresaw, and an independent second opinion:
today the kernel's read-back checks our STEP against a reader we did not write. The oracle keeps
that second opinion where it matters, in the slow tier, against the same files.

**A caution from this repo's history.** Two tracks that built geometry ourselves were removed:
the certified general swept boundary (2026-09-25) and the candidate construction — contact covers,
meridian charts, face splitting, endpoint caps (2026-09-26). Both tried to construct the *swept
boundary* itself. This plan does not: the sheets stay what they are (fitted to traced contacts,
verified by the meter), and what moves is ordinary face-level B-rep work on them. The meter, the
field agreement and the pair check remain independent gates on every stage.

## What the export uses today

The backend is about 2,800 lines of C++ (`rust/gcs-cli/backend/`) behind 47 C ABI calls, used by
3,200 lines of Rust in `gcs-cli/src/cad/`. By what it does:

| capability | OCCT | used for | difficulty |
|---|---|---|---|
| B-rep topology | `TopoDS`, `BRep_Tool`, `TopExp`, `BRep_Builder` | everything | moderate |
| surfaces, curves | `Geom_*` (plane, cone, sphere, cylinder, torus, B-spline) | faces, edges | easy |
| primitives | `BRepPrimAPI_MakePrism`/`MakeRevol`, `BRepBuilderAPI_Make*` | recipes, the blank | easy–moderate |
| fitting | `GeomAPI_PointsToBSplineSurface` | the sheets, the sector sides | moderate |
| projection | `GeomAPI_ProjectPointOnSurf`, `Extrema_*` | fit checks, feet, meter | easy |
| classification | `BRepClass3d_SolidClassifier`, `BRepClass_FaceClassifier`, `IntCurvesFace` | cells | easy (ray parity exists) |
| mass properties | `BRepGProp` | volumes, the pattern's flux | moderate |
| validation | `BRepCheck_Analyzer` | every construction | easy for our own invariants |
| copy, transform, sewing, unify | `BRepBuilderAPI_Copy`/`Transform`/`Sewing`, `ShapeUpgrade_UnifySameDomain` | the pattern, the union | mostly disappears |
| sections | `BRepAlgoAPI_Section` | the cutters' meridian profiles | easy (the section is 2D) |
| **Booleans** | `BRepAlgoAPI_Splitter`/`Fuse`/`Common`/`Cut`, `BOPAlgo` | the split, the sheets' fuse, the cutters, static solids | **hard** |
| meshing | `BRepMesh_IncrementalMesh`, `Poly_Triangulation` | the STL, the sag | moderate |
| STEP | `STEPControl_Writer`/`Reader`, `StepData_*` | the master, `--verify-step full` | moderate (the parser exists) |

**What the files contain.** The configured members at 10 µm have only three kinds of surface:

| | faces | B-spline | conical | spherical | edges | edge curves (B-spline / circle / line) | pcurves | STEP |
|---|---|---|---|---|---|---|---|---|
| pinion | 147 | 72 | 73 | 2 | 439 | 1,014 / 100 / 203 | 878 | 12.8 MB |
| gear | 291 | 144 | 145 | 2 | 871 | 2,021 / 197 / 395 | 1,742 | 27.4 MB |

No plane, cylinder or torus reaches a gear file: the blank is cones and end spheres, every one a
surface of revolution about the member's axis, and everything else is a fitted sheet.

**What the core already has.** B-spline curves (`curve.rs`, Boehm insertion); exact predicates
and a regular Delaunay triangulation (`delaunay/`, 3D); the seams (`seam.rs`: envelope ∩
envelope, envelope ∩ analytic boundary), vertices and edges; `topology::ClosedShell` (identity
without coordinates); analytic projection and the meter (`solid::accuracy`); contact tracing and
the sheets' layout (`solid::contact_trace`); the sector's boundary (`solid::sector`); interval
arithmetic; our own dense linear algebra. And in the CLI: the STEP parser and verifier
(`step_check`), ray-parity cell classification, the flux volume of a pattern.

## Design

- **Where.** `gcs-core/src/brep/` — the core owns every algorithm, and only the core reaches the
  wasm. No dependency is added. The CLI's `cad::native` keeps its `Session` shape, so the export
  code chooses a kernel at one seam (`SOLVENT_KERNEL=occt|rust` while both exist).
- **The B-rep.** Arenas of vertices, edges, loops, faces, shells, solids with typed indices, never
  pointers (`BTreeMap`/`Vec` only: determinism). A face is a surface, an outer loop and holes, and
  a sense; an edge is a 3D curve with a parameter interval, a pcurve on each face that uses it,
  and a **measured** tolerance — the largest gap between its 3D curve and its pcurves' images,
  computed when the edge is made and asserted under the export's bar, never widened to make a
  check pass. Identity is shared by construction: the pattern's copies reference one seam edge,
  so a sector's neighbour is not found by proximity (the rule `topology::ClosedShell` states).
- **Geometry.** `enum Surface { Plane, Cone, Sphere, Cylinder, Torus, Revolution(profile),
  BSpline }` and `enum Curve { Line, Circle, BSpline }`, each evaluating position and first two
  derivatives; analytic surfaces keep their closed-form inverse (point → (u, v)).
- **Tolerance model.** Two tolerances, never one. The **export's bar** (`solid::export::Tolerance`,
  as today: 10 µm) is what the finished file is held to, read by the meter. The **construction
  tolerance** (OCCT's is its Boolean fuzzy value, 1e-5 mm) is what the kernel merges within while
  it builds: vertices closer than it are one vertex, an edge shorter than it is none. An edge far
  shorter than the bar but longer than the construction tolerance is an ordinary edge — the 10 µm
  pinion has one of 0.5 µm (phase 0) — and rung 1's rule refusing an edge shorter than the bar
  gives way to this. Intersection curves are traced to a fraction of the bar and fitted to
  B-splines within another fraction, as before.
- **Robustness.** Parameter-domain arrangements (a face split by its intersection curves) use
  exact 2D orientation (added beside `delaunay::predicates`) on polyline approximations whose
  distance to the true curves is bounded; the refinement of a curve near a crossing is driven by
  the predicate's doubt, not by a fixed count.
- **The oracle.** A slow-tier harness builds each stage both ways and compares face and edge
  counts, faces by surface kind, the STEP read back by OCCT and the meter. Volumes agree to 1e-9
  where both kernels' surfaces are analytic; on B-spline and swept-spline faces OCCT's own
  integration is off by 4e-8 to 2e-6 (rung 2), so there the gate is this kernel's volume against
  closed forms where they exist, the meter, and OCCT's volume only to ~1e-5, as a gross check.
  OCCT stays a CLI feature for that harness only, until the last stage retires it from the export.

## Revisions after rungs 1–2 and phase 0 (2026-09-30)

The ladder's first two rungs were climbed before the phases, and phase 0 measured the export. What
they taught, and where the plan now says it:

1. **Phase 1 is smaller, and has a risk the plan did not size: the mesher's speed.** The B-rep, the
   STEP writer, the mesher and the volume exist (rungs 1–2); phase 1 is now B-spline surfaces, the
   converter from OCCT's shape and per-edge tolerances. But the mesher's Delaunay insertion scans
   every triangle for each point it adds, and the gear's STL is 776,000 triangles (16,000 a
   sector, turned into the rest). Point location comes first (Phase 1).
2. **Two tolerances.** A 0.5 µm edge in a correct 10 µm pinion means the B-rep merges within a
   construction tolerance and measures its edges against it, and holds the file to the bar
   separately (Design: tolerance model). The same model is what the crown cutters rung 1 refused
   needed.
3. **OCCT is no 1e-9 volume oracle on spline faces** (4e-8 to 2e-6 off where this kernel meets the
   closed forms to 1e-9). The gates of phases 1, 3 and 4 read closed forms and the meter there,
   and OCCT's volume only as a gross check (Design: the oracle).
4. **The split (phase 3) is less risky than written.** Its intersections are three transversal kinds, never
   under 17.8°: a sheet against the blank's cones and spheres (the contour in the sheet's domain
   the phase already proposes) and two or three sheet × sheet curves. No general B-spline ×
   B-spline intersection is needed for the gear (Phase 3).
5. **The cutters' own sections (phase 4) are required, not a nicety.** Every near-tangent pair the export
   meets (tori at 0°) is in the cutters' construction, so the meridian-plane route must handle
   tangent circles exactly; it also removes the cutters' plane sections, about 11 s of thread time
   on the gear (Phase 4; see the revision below on the gear's cutter, which is not in one meridian
   plane).
6. **The faceted kernel drifts from the exact one.** Rung 2 found three places they disagreed
   (splines lofted as chords, twisted lofts 26% light, loft pairing). Until rung 3 retires it, the
   oracle compares the two on every root at a bar tied to the faceting rather than a flat 0.5%
   (Rung 3).
7. **The sizing was far too high.** Rungs 1–2 came to about 5,900 lines (4,500 of source) against
   the 13–25k estimated for them (Sizing).

## Revisions after phase 2 (2026-09-30)

Phases 1 and 2 read what the kernel builds into the core and work on that; starting phase 3 found
two things that change the order.

1. **The gear's cutter is not one meridian plane's.** Phase 3 (as first written) took the gear's
   cutters for 2D Booleans of their profiles in a common meridian plane. The space cutter is the outer
   crown bounded by its neighbour, and the neighbour is the inner crown turned a crown pitch about
   the crown's axis (`crown/space.sv`), not the cutter's own: two solids of revolution about
   different lines, so a meridian half-plane of one cuts the other along curves past lines and
   arcs. That is also why the gear's cutter is sectioned at every station (636 `section` calls)
   where the pinion's, a solid of revolution about its own axis, is sectioned once and turned. The
   gear's sections come from the core's analytic field of the cutter instead (Phase 4).
2. **The split before what it splits.** Built by the core while the kernel still splits, the blank,
   the sheets and the cutters would each have to be handed to the kernel as its shapes — an
   importer and adapters, all thrown away once the split is the core's. The split is therefore
   phase 3, taking the kernel's blank, sheets and sides through phase 1's converter as phases 1 and
   2 did, and the producers phase 4, each replacing the kernel's with nothing handed back. The
   phases below are renumbered; references to them throughout follow.

## Revisions after phase 5 (2026-10-01)

Taking phase 5 and reading rung 3's code found three things.

1. **The same file everywhere needs the core's own mathematics.** A STEP or STL byte-identical
   between `solventc` and the browser's worker is not reached by sharing the code: the platform's
   `sin`, `atan2`, `exp` and `hypot` differ in their last bit between a native libm and the wasm
   build's, and a fitted sheet amplifies a bit into a different file. The core therefore computes
   them itself (`fmath`, the musl/FreeBSD algorithms, reached through the `Det` trait as `x.dsin()`
   and the rest), on every target and in the solver too. That moved bits across the whole solve —
   the tests recording bits were re-recorded once, and two comparisons that had passed by the
   platform's luck were made honest (two readings alike to rounding, not to the bit; a drawing's
   places matched as a set, where a sort had let `15.000000000000002` read as two points swapped).
   Our `atan2` is the correctly rounded one where the platform's was an ulp off. A test that records
   bits records `fmath`'s.
2. **Rung 3 has a seam already, and it is smaller for it.** An evaluated solid need not be the facet
   term's boundary: a swept solid's is its field mesh handed in as one polyhedral primitive
   (`EvaluatedSolid::from_surface`), and every reader — boundary, edges and hidden lines,
   containment, the mesh, glTF, the claims — reads it as it reads any other. The exact solid enters
   the same way: the B-rep built from the CAD recipe, meshed at the policy's sagitta with each
   triangle carrying its face's document path (`body.bore.wall`, so surviving faces, provenance and
   round features read as before), and its volume read off the B-rep exactly. Hidden lines and
   sections classified against the surfaces themselves are then a step of its own after the rung,
   not inside it. (Silhouettes proved to be inside it: drawn from an irregular mesh's seams they
   zigzag, so they are traced on the surfaces — see the rung's record.)
3. **The B-rep is built once per geometry, the mesh once per zoom.** The B-rep does not depend on
   the pixel length; the facet term is evaluated again at each one. The B-rep is cached against the
   solid's reads alone and only its tessellation per policy. **And the facet kernel stays** as the
   evaluation of whatever this kernel refuses (rung 1's designs that touch themselves) — not
   retired once nothing reads it — and as a drag's reading if the B-rep's Booleans measure too slow
   for a frame, which is the first thing rung 3 measures.

## Phases

Each phase leaves the export working, gated by the oracle, and is taken in the order that retires
the most OCCT for the least risk. Phase 0 measures before any code is committed to.

### Phase 0 — measure

- Count, for each export stage, the ABI calls made and the time in each (the stage trace already
  carries processor time), for both members at both bars and for the static goldens
  (`solid_flange`, `solid_pulley`, `solid_tray`, `solid_elbow`, `solid_indexed_pattern`,
  `solid_loft`).
- For every Boolean the export performs, record the operand face kinds and the pairs that
  actually intersect (sheet × cone, sheet × sphere, sheet × sheet, sheet × side, analytic ×
  analytic): this is the intersection vocabulary phase 3 (the split) must cover, measured rather than guessed.
- Record the intersection curves' lengths, the minimum angle between the surfaces along them
  (near-tangent intersections are where marching fails), and the smallest face and edge.
- For the ladder: every solid in the corpus by form and profile kind (which rung it needs), and
  every Boolean with coincident or tangent operand faces, by kind (a flush bore, a boss in `union`
  with its stock, a mate, coaxial equal radii).
- **Exit:** a table per stage and per Boolean, here; a go/no-go on the split (phase 3) and on each rung, with
  the estimated size of each.

#### Phase 0 — done (2026-09-30)

Measured by a probe in the native backend (`backend/probe.hpp`): with `SOLVENT_ABI_TRACE` naming a
file, every entry point's time and thread, every Boolean's face pairs read from OCCT's own
interference table (`BOPDS_InterfFF`: the pairs that met, each curve's length, the least angle
between the two surfaces along it, OCCT's tangency flag), the exported shape's smallest face and
edge, and each export stage marked when it completes on the same clock. `tools/abi_trace.py`
turns a log into the tables below. Both members at the gross bar and at 10 µm, and the static
goldens (`solid_flange`, `solid_pulley`, `solid_tray`, `solid_indexed_pattern`; `solid_elbow`
and `solid_loft` are lofts, which OCCT never built and this kernel does since rung 2).

**Where the time goes** (10 µm; a stage's native seconds summed over its threads):

| stage | pinion, done at | pinion native | gear, done at | gear native | what |
|---|---|---|---|---|---|
| blank, reach, admission, sheets, fit, clearance | 3.9 s | 2.9 s | 4.4 s | 17.4 s | the cutters' meridian sections (`section`, 44 and 636 calls: 0.6 s and 10.8 s), point-in-solid, 80–190k `face_normal` |
| **split** | 14.2 s | 8.8 s + 2.0 s (tools' fuse) | 13.7 s | 6.3 s + 2.4 s | the sector split by its sheets — half the export |
| classify | 14.4 s | 0.2 s | 14.0 s | 0.2 s | ray parity |
| fuse, mesh | 17.5 s | 4.9 s | 17.4 s | 5.3 s | the pattern, its check, the sector's mesh |
| STEP | 16.3 s | 1.1 s | 20.0 s | 3.9 s | OCCT's writer |

The statics take 0.1–0.2 s each, a third of it the STL.

**What the Booleans meet.** The sector split (and the fuse of its tools) is the whole of phase
4's vocabulary, and it is small and well conditioned:

| surfaces meeting in the split | pinion curves | gear curves | least angle | shortest curve |
|---|---|---|---|---|
| B-spline sheet × cone (the blank's cones) | 20–25 | 20 | 25.5° / 27.0° | 2 µm (pinion, 10 µm) / 0.29 mm |
| B-spline × sphere (the end spheres) | 8–11 | 8 | 37.5° / 59.5° | 60 µm / 4.5 mm |
| B-spline × B-spline (removal × relief) | 2 | 3 | 17.8° / 25.7° | 22 mm / 4 mm |

No pair in the split is tangent or under 17.8°; OCCT flagged no tangent faces anywhere. The
near-tangent pairs the exports do meet (torus × torus and plane × torus at 0°, cone × plane at 7°)
are all in the *cutters'* construction — the crown bounded by its turned neighbour (`body bound`),
and the plane sections that read a cutter's meridian — which phase 4 does with no 3D intersection
(a cutter of revolution in its meridian plane, the gear's from its analytic field), and which rung 1
refuses in 3D. The other Booleans are the clearance commons (plane × cone,
plane × sphere: the sector's half-planes), the seam's ring pieces (cone × plane) and the recipe's
body cuts, which met nothing.

**The smallest pieces.** Exported, the pinion has 147 faces (73 cones, 2 spheres, 72 B-splines)
and the gear 291 (145, 2, 144); the smallest faces are 0.30 and 0.38 mm². The pinion's shortest
edge at 10 µm is **0.5 µm** (8 µm at the gross bar), the gear's 13 µm — edges shorter than the
export's own tolerance, which OCCT keeps (its split runs at a fuzzy 1e-5 mm) and rung 1's rule
would refuse.

**Go, with one finding.** The split (phase 3) is a go: its intersections are three transversal kinds, each a
scalar contour in the sheet's domain against a surface of revolution about the member's axis
(cone, sphere) or a well-conditioned sheet pair, a few dozen curves per member. The finding is the
tolerance model: a construction tolerance (OCCT's 1e-5 mm) apart from the export's bar, so that
a 0.5 µm edge is kept and merged within the construction tolerance rather than refused against the
bar. The same model is what the crown cutters needed in rung 1, and phase 4's cutter sections avoid the
3D near-tangencies altogether. Phases 1–3 stand as planned: the pattern, the blank (a meridian
region turned once — 0.05 s now), the sheets' fits (0.6–0.8 s) and the STEP writer (1–4 s) are
each small. Rung 3 needs nothing from these numbers.

### Phase 1 — the B-rep, the STEP writer and the mesher, fed by OCCT

What rungs 1–2 built stands (`brep/`: topology, geometry, the STEP writer checked by `step_check`,
the mesher to a measured sag, volume by Green's theorem). What the gear adds:

- **B-spline surfaces**: `Surface::BSpline` (tensor, non-rational, as the fitted sheets are), with
  evaluation, its inverse by Newton from a grid, and its signed distance; and B-spline pcurves,
  since OCCT's edges on a sheet carry arbitrary 2D curves in its parameters.
- **The converter** from OCCT's finished shape into ours, through the `brep_summary`-style ABI
  extended to surfaces' data (B-spline poles and knots), edge curves and pcurves.
- **Per-edge tolerances** in the B-rep: each edge's measured gap between its 3D curve and its
  pcurves' images, checked against the construction tolerance (Design), not one tolerance for all.
- **The mesher's point location**: an insertion that walks to the triangle holding the point
  instead of scanning every triangle, so a sector's 16,000 triangles (and the whole gear's
  776,000, meshed whole as a check) take the time they should; measured against `BRepMesh`.
  The sector's mesh turned into its copies as today (`sector_stl`'s seam pairing becomes
  identity, since the copies share edges).
- **Gate:** for both members at both bars, our STEP of OCCT's shape verifies (`step_check`) and
  reads back in OCCT as a valid solid with the same faces and a volume within ~1e-5 (OCCT's
  integration of sheets being no finer); our volume against OCCT's to its own integration's width;
  our STL passes the shell checks and the meter within the tolerance; the pair check unchanged
  within 0.1 µm. Time the file stages against phase 6's.

#### Phase 1 — done (2026-09-30)

`SOLVENT_WRITER=rust` hands the solid the native export built to the core and writes its STEP and
STL by our writer and mesher (the solid built whole, its union made before its files). The hand-over
is `backend/dump.cpp` (`solvent_cad_brep_json`) read by `brep::json`; `SOLVENT_BREP_DUMP=PATH`
writes it for the core's tool (`brep_json_debug`). What it took, and what it found:

- **B-spline surfaces and pcurves.** `Surface::BSpline` over `nurbs::Net` (tensor, non-rational;
  degrees to 25, since OCCT's pcurves reach 11); `Pcurve::Curve`, a kernel's 2D curve read at the
  edge's own parameter and written to STEP exactly (its poles).
- **Reading OCCT's shape.** Each face dumped forward and a reversed one's loops turned by the
  reader (a seam's pcurve is picked by the edge's orientation, which a reversed face turns once
  more); left-handed placements made right-handed (a revolution's `u` negated with its pcurves);
  and the uses **ordered by vertex and parameters**, since `BRepTools_WireExplorer` misreads a
  closed edge's orientation (a sphere band's two circles came out running the same way).
- **Measured edge tolerances** (`Edge::tol`): the pinion's worst is 0.26 µm, the gear's 0.006 µm.
  `check` honours them, so it no longer proves that pcurves meet their edges; a reader holds them
  to a bar itself.
- **The volume closes each loop in the face's parameters.** OCCT's loops have gaps of about
  1e-7 in a sheet's chart, and `G` over a chart whose `u` spans a few hundredths is large, so the
  pinion came out 3e-6 light and moved 0.4 mm³ with a 100 mm shift of the origin. Joined straight
  across each gap, it is translation-invariant to 2e-9, and the whole pinion is 24 sectors to 2e-8.
  A B-spline face's `G` is tabled exactly per span (Gauss in `u`, Chebyshev interpolation in `v`),
  and the faces are integrated side by side: 11–15 s became 0.4–0.6 s.
- **The mesher is a neighbour-array CDT**: points located by walking, Bowyer–Watson cavities, the
  loops recovered by Sloan's flips, inside by flood fill, refinement by inserting a triangle's
  middle and legalizing about it, the faces side by side. The whole pinion went from 21–31 s and
  200k triangles to under a second; the spheres' 9,500-point boundaries had cost 49 s of it.
- **A facet must face the way its surface does.** Refined for sag alone, the pinion's mesh failed
  the pair check (backlash 0.009 mm): where a sheet's metric changes fast across a triangle, one
  wound the right way in the scaled parameters comes out turned over in space while near the
  surface (two a sheet, 0.01–0.06 mm², 24 teeth alike), and the check sorts flanks by facet
  normals. Refinement now also tests each facet against the surface's outward normal at its corners
  and middle (signed, past `angular`), a sliver getting a point off its longest side rather than
  in its middle (which only makes two more), and `Mesh::turned` counts what is left thicker than a
  hundredth of the bar; the export refuses any. A face's points nearer than 1e-9 of the solid are
  welded (a stretch of a chart where several parameters are one point).

**Gate** (both members, at the gross bar and at `--tolerance`, `SOLVENT_STEP_VERIFY=full`):

| | read back by OCCT, our measure | OCCT's measure | STL sag / bar | triangles (ours / OCCT) | field agreement |
|---|---|---|---|---|---|
| pinion, gross | 1.2e-6 | 3.5e-5 | 9.999 / 10 µm | 139k / 143k | 0 disagree |
| pinion, 10 µm | 2.5e-6 | 1.8e-6 | 4.998 / 5 µm | 133k / 776k | 0 disagree |
| gear, gross | 1.9e-7 | 3.3e-7 | 9.991 / 10 µm | 216k / 197k | 0 disagree |
| gear, 10 µm | 2.1e-6 | 1.8e-6 | 4.998 / 5 µm | 286k / 352k | 0 disagree |

Every edge is within 0.26 µm (pinion) and 0.008 µm (gear) of its faces. The meter, at 10 µm: the
pinion's STEP within 1.37 µm of every exact face (OCCT's 2.75), the gear's 0.69 µm (1.44); the STLs
within 5.74 and 5.53 µm (OCCT's 3.94 and 6.16). The pair check passes on our STLs:
backlash 0.0413 and 0.0456 mm against OCCT's 0.0418 and 0.0479 (0.05 designed), the members' least
distance 0.0211 mm against 0.0214 mm. The plan's "unchanged within 0.1 µm" was never a bar an STL
can meet at a 5 µm sag; the two files differ by what their sags allow. Our pinion's axis reads
7e-5° off and the offset 0.24 µm long, since each tooth is meshed on its own where OCCT's mesh is
one sector turned: phase 2's shared pattern gives that back.

**Time** (a machine at load 50, so ratios rather than totals). Our STEP is written in 0.5–1.0 s
where OCCT's took 2.8–8.4 s; the conversion is 0.5–1.2 s; our mesh 0.55–1.3 s at 10 µm in one
pass, where OCCT's pinion needed a second meshing (5.7 s) for 776k triangles. Full verification
(reading our STEP back through OCCT) is 5–13 s, and is the slow tier's.

OCCT is no volume oracle here. Its STEP of the pinion, read back by OCCT and measured by us,
moves 1.1e-6, and ours moves 1.2e-6: that is OCCT's reader repairing what it reads. Writing a
tighter uncertainty makes it worse (3.3e-5 at 1e-7 mm), because the reader then repairs every
edge. Its own volume of the pinion is 9e-6 off its own sectors' sum. So full verification reads
our STEP back through OCCT, measures that reading with `props` to 1e-5, and holds OCCT's own
volume only to 1e-4.

### Phase 2 — the pattern, built with shared topology

- The sector's material turned by the indexing motion into its copies, each copy's boundary faces
  on the source surfaces with pcurves shifted, neighbouring copies meeting on one shared seam edge
  — the construction `solvent_cad_pattern` does by sewing, done by identity.
- Validation of our own invariants: every edge used twice with opposite senses, each loop closed
  in its face's domain, faces' senses consistent (`topology::ClosedShell` over the result).
- **Gate:** as phase 1, with the pattern ours (the sector still OCCT's); the union's 3.8 s gone.

#### Phase 2 — done (2026-09-30)

`SOLVENT_WRITER=rust` on a body built as one sector reads the kernel's *sector* and turns it into the
whole in the core (`brep::pattern`); the kernel's union, its unification and its check are never
made. By identity, as planned:

- **The sides** are the one pair of faces (not of revolution about the axis) one of which, turned a
  pitch either way, lies on the other; the far side's vertices and edges are matched once to the near
  side's turned (to 1.6e-14 mm on the pinion: the kernel makes the far side the near one turned).
- **Faces of revolution about the axis keep the sector's surface** in every copy, their pcurves moved
  along `u` by the turn (the sign read off a point). Each is first homed within half a turn of the
  sector's middle, since a kernel keeps each face's parameters in a period of its own choosing; a
  piece continuing one across the last copy is read a period on.
- **Pieces of one surface meeting across a side are one face.** On the gear pair every side edge is
  between two pieces of one blank face, so every junction disappears: the pinion's three lands are
  piece 13 of copy k joined to piece 0 of copy k + 1, and the cone and two spheres meeting both sides
  close into rings, each keeping its last junction as its seam (one edge used twice, its pcurves a
  period apart — not an iso line, which our mesher and writer do not need). 147 and 291 faces, as
  OCCT's unified unions; more edges (555 and 1155 against 415 and 871), since the junction vertices
  stay on the arcs they split.
- **Sheets are cut to their faces** (`Net::segment`: exact knot insertion to the face's parameter box,
  a hundredth about it), so the STEP carries what OCCT's did: 148,595 entities for the 10 µm pinion
  (OCCT 149,767), 13 MB.
- **The mesh is the sector's turned** (`Built::mesh`, `mesh::mesh_with`): the sector's faces but its
  sides meshed once, the far side sampled as the near side turned, the copies sharing their seam
  points by index. The pair check reads the transverse moments equal within 6.5e-9 and 1.0e-8, the
  shaft angle 90.00000° and the offset 25.00000 mm, as from OCCT's sector STL (phase 1, meshing each
  copy alone: 4e-6, 0.24 µm off). Meshing takes 0.09–0.18 s.

**Gate** (both members, both bars, full verification): OCCT reads our STEP back valid with the same
faces, within 1.2e-6 to 2.5e-6 by the core's measure; the STLs within their bars with no turned
facets; the field agreement 0 disagreeing; the meter at 10 µm, STEPs 1.37 and 0.69 µm, STLs 5.13 and
5.65 µm; the pair check passes (backlash 0.0413 and 0.0457 mm). The pattern's volume is the sector's
times the copies to 2e-8, and a torus and a cone triangle revolved through a pitch pattern into their
Pappus volumes (`a_sector_patterned_is_its_whole_revolution`).

**Time** (load 35, interleaved, light verification as exported): pinion 6.2 / 6.7 s (OCCT 5.7 / 11.0),
gear 8.6 / 10.6 s (OCCT 10.5 / 12.1), gross / 10 µm. From the cells to the files the gear at 10 µm
takes 2.9 s against OCCT's 4.8 (its union alone 4.1 s; the core's turn 0.25 s); what is left there is
our STEP's writing and parsed check.

### Phase 3 — the sector's split

The phase with the most code, though phase 0 made it smaller: the split meets only a sheet
against the blank's cones and spheres and two or three sheet × sheet curves, all transversal
(17.8° and up), a few dozen curves a member. The sector is the blank between two fitted sides,
split by its sheets into cells, one kept (phase 6 of the speed plan: five cells, one material).
The kernel still builds what the split takes — the blank, the sheets and the two sides — and the
core reads them through phase 1's converter (`backend/dump.cpp`, `brep::json`), so nothing is
handed back to the kernel; the cell kept goes to phase 2's pattern.

- **Sheet × blank face.** Every blank face is a surface of revolution about the member's axis:
  a curve f(r, z) = 0 in the meridian plane. A sheet point S(u, v) is on it where
  f(r(S), z(S)) = 0 — a **scalar contour in the sheet's own parameter domain**: seeded by a grid
  sign scan (with interval enclosures to prove no small component is missed), traced by
  continuation, refined by Newton, fitted as a pcurve in the sheet and a B-spline in space, with
  the blank face's pcurve read off by the closed-form inverse. No surface–surface machinery.
- **Sheet × sheet and sheet × side** (the removal against the relief; each sheet against the two
  sides): true B-spline/B-spline intersections, two or three per member and at 17.8° or more
  (phase 0). Subdivision of the two control nets to seed, marching on the two surface
  equations to trace (the DogLeg loop, as the seams already do), both pcurves kept.
- **The arrangement.** Each face's domain split by its curves (exact 2D predicates), the pieces
  assembled into cells with shared edges, each cell classified by ray parity (as today) and the
  material cell kept by the field (`MaterialEvaluator::probe`, as today).
- **Refusals.** A tangential or near-tangential intersection (the angle under a few degrees:
  the split meets none under 17.8°), a curve that leaves its face's domain unexpectedly, an arrangement whose pieces do not
  close: each refused with a witness, the export falling back to OCCT's split while it exists.
- **Gate:** both members at both bars split by the core: the kept cell's faces by kind as OCCT's,
  its volume as OCCT's cell's by the core's measure (1e-9 where both are read by it), the field
  agreement and the meter pass, the STEP reads back in OCCT, the pair check passes with its readings
  within the meshes' sag of OCCT's (phase 1: an STL cannot carry a 0.1 µm bar at a 5 µm sag); the
  48-design harness (`generating_harness.rs`) refuses nothing OCCT split. Time it.

#### Phase 3 — done (2026-10-01)

`brep::boolean::split(solid, sheets, tol)` cuts a solid by open faces into its closed cells: every
face split where the other's faces meet it (`arrange`), the sheets' pieces inside the solid kept both
ways round, and the cells assembled radially — at each edge the pieces meeting there are taken in turn
about it, each two neighbours bounding one cell by the sides facing the gap between them, and a
class holding a solid face turned round is the outside. Under `SOLVENT_WRITER=rust` the sector is
split by the core (`SOLVENT_SPLIT=occt` keeps the kernel's split), its cells classified by interior
samples the core measures (`brep::query::interior`: a jittered grid and points stepped in along the
boundary's normals, judged against a mesh whose sag was measured, so a sliver has samples too) and the
field's probes, and the material cell patterned (phase 2). The split meets the blank's cones and
spheres in closed form or traced, and the sheets' and sides' B-splines traced; what was learned:

- **A traced curve is fitted, not read by differences.** Its derivative by central differences of
  projected points carried their noise into a face's flux (0.131 mm³ on one gear face): each open
  trace is now a cubic B-spline (`ssi::fitted`) through every eighth traced point and the seeds on it
  (the faces' crossings, its vertices, placed by a local search along the trace), its gaps graded
  2:1 and split where a traced point or the trace's middle of a step lies past a quarter of the
  tolerance — to an eighth of a step and no further, below which the traced curve's own noise is
  what is measured.
- **B-spline feet** (`net_inverse`): a warm start from each thread's last foot on the sheet, else
  Newton from the nearest poles' Greville points — full Newton where the squared distance's Hessian
  is positive definite, Gauss–Newton otherwise, with backtracking and an active set at the domain's
  edges — and the grid only past that (5,013 grid searches a split fell to 2). Edges and rays are
  tested against a face only within its box (`curve_surface_within`).
- **A hole goes in the outer cycle about the ground beside it**, never one that shares an edge with
  it: a probe a hair off the hole is nearer it than two walks' samples of one edge agree. The gear's
  131 mm³ cell lost its cut there. Nodes of a face's pieces are one vertex within 1e-3 of the
  parameters (a fitted curve ends within its fit of the vertex; only a seam's copies are farther).
- **Volumes by Green's theorem integrate from the face's middle and across its narrower parameter**
  (a pole's degenerate edge counted where the line integral runs in `u`): on a ring turned nine
  radians round, `G` from zero was π R³ times the turn.

**Gate** (both members, both bars, full verification): the split gives OCCT's cells, five each, faces
by kind equal; the kept cells within 3e-8 of OCCT's (pinion 610.1428851 against 610.1428495 mm³), the
partition summing to the wedge exactly; OCCT reads our STEP back valid within 2.6e-6 by the core's
measure; the field agreement 0 disagreeing; the meter at 10 µm, STEPs 0.42 and 1.10 µm, STLs 5.19 and
5.77 µm; the pair check passes (backlash 0.0407 and 0.0467 mm). The split takes 2–3 s a member (sides
0.3 s, sheets side by side). Recorded: OCCT's own volume of a STEP it reads is good to ~1e-5, and
the plan's 1e-9 volume bar holds between our split's cells and OCCT's only to the curves' fit (~3e-8);
a pcurve on a spline face is written as cubic Bézier pieces within a tenth of `FIT` of its foot
(`step::bezier_pcurve`: a chordal pcurve within the file's tolerance had trimmed each sheet face 1e-5
of its flux off), and the mesher cuts a B-spline edge at its knots only a bar apart, a sliver thinner
than a tenth of the bar counted as no turned facet.

### Phase 4 — the blank, the sheets and the cutters

What the split takes and the contacts read, built by the core, each producer replacing the
kernel's one at a time with the split already ours (phase 3), so none is handed to the kernel.

- **The blank** as its meridian region turned once: the region's Booleans are 2D Booleans of
  lines and arcs (exact), the revolution a face per profile edge (`Surface::Cone`, `Sphere`,
  `Plane`, `Cylinder`, `Torus` by the edge's relation to the axis); or the recipe built by rung 1,
  which already builds the blank (0.42 s against OCCT's 0.88).
- **The cutters' sections.** A cutter of revolution about its own axis (the pinion's) is its own
  meridian profile: a 2D Boolean of its revolutions' profiles, no plane section at all. The gear's
  is not: its space cutter is the outer crown bounded by its neighbour, and the neighbour is the
  inner crown turned about the *crown* axis (`crown/space.sv`, `crown_neighbor`), not about the
  cutter's own, so a meridian half-plane of the cutter cuts the neighbour obliquely — along
  curves of degree past a line or an arc, not along its profile. Its sections come from the core's
  analytic material field of the cutter instead (`MaterialField::read`: Booleans of placed
  revolutions, exact, with no 3D intersection), contoured in each half-plane: each boundary piece
  belongs to one operand's surface, its points placed by Newton on that operand and the corners
  where the deciding operand changes found as the creases are. The tangent tori phase 0 found are
  two operands whose fields both vanish along a curve; in a half-plane that is a corner like any
  other. This replaces the cutters' plane sections (636 `section` calls on the gear, 11 s of
  thread time) and the clearance's Booleans (a field reading over the blank's support).
- **Sheet fitting** (`fit.rs`'s chord-length and centripetal grids) and the sector sides' fits, by our
  interpolation; projection and feet by Newton on the B-spline.
- **Gate:** the blank's volume and faces equal OCCT's (1e-9: all analytic); every cutter section's
  loops agree with OCCT's at every station (points within 1e-6 mm, the same corners); every sheet
  fits its withheld contacts as OCCT's did (the fit report the same to the bar); the exports
  unchanged by the oracle.

#### Phase 4 — done (2026-10-01)

Every producer the sector takes is the core's under `SOLVENT_WRITER=rust`:

- **The blank** is its meridian region turned once from the half-plane opposite the sector
  (`brep::recipe::meridian`): each revolution's profile — a ball's half-disc — read into that
  half-plane's (radius, axial) coordinates and combined by **planar Booleans of lines and arcs**
  (`brep::planar`: edges split where they meet in closed form, pieces kept by where their middles
  stand against the other region — an exact ray test, since a polygon of the arcs sagged 0.09 mm —
  coincident stretches by their sense, a profile's edges walked end to end first since a recipe lists
  them in any order). Both members' blanks equal OCCT's in volume to the last printed digit and in
  faces by kind; the core had built them by 3D Booleans with the end spheres revolved about their own
  diameters, each split at its own seam.
- **The cutters' sections** (`brep::section`): a solid of revolutions about lines parallel to its
  first's, cut by a half-plane through that line, is a planar region whose steps are each one
  revolution's meridian — plain where it turns about the half-plane's line, and carried in by a
  `planar::Chart` where it turns about another: radius `r² = s² + 2βs + δ²` about the other line, its
  axial coordinate the same, monotone in `s` on the side a meridian is seen on. So the gear's cutter
  (the outer crown bounded by its neighbour, the inner crown turned about the gear's axis, which is
  parallel to the cutter's) sections exactly, with no plane section and no field contour; every step
  is tagged with its face, stable across stations, and read exactly (`Sectioned::at`: point and
  outward normal). Against OCCT at all 318 stations a gear export asks: the same pieces and convex
  corners, points within 2.8e-7 mm of OCCT's own polyline samples, corners within 7.5e-9 mm; the
  pinion's within 3e-14.
- **The sheets and the side** are this kernel's cubic interpolation of their grids
  (`nurbs::interpolate_net`: each direction's parameters averaged over the other, rows then poles):
  by chord length its averaged knots overshoot the pinion's unevenly spaced rows into a fold, so a
  sheet at the gross bars is fitted by centripetal parameters (3.3 µm from its withheld contacts where
  OCCT's chord-length fit was 15.5), and held to a tolerance by both, the better kept, as before.
  Feet and grids are read on the core's B-spline (`Surface::foot_from`, `inverse`).

**Gate** (both members, both bars, full verification, the core's producers throughout): the files and
checks as phase 3's — OCCT's read-back within 2.5e-6, the meter's STEPs 0.42 and 1.10 µm and STLs
5.19 and 5.77 µm, the pair check passing (backlash 0.0407 and 0.0467 mm), the field agreement clean.

### Phase 5 — the export without OCCT

- The native export's default kernel is ours; `SOLVENT_KERNEL=occt` builds the old way for the
  oracle. `solventc --step/--stl` for the admitted class needs no `OCCT=1` build.
- The FFI and the app: `gcs_solid_step`, `File ▸ Export STEP`, the toleranced STL from the app,
  run in the mesh worker (it is seconds, not milliseconds).
- **Gate:** the goldens' native files byte-identical between the CLI and the wasm build; the
  slow tier's oracle harness green; the web suite.

#### Phase 5 — done (2026-10-01)

- **One export, every host.** `brep::export::{exact,step,stl}` builds, writes and checks a solid —
  from its recipe, or an admitted swept body by `brep::sweep` — and both `solventc` and the app call
  it: `gcs_solid_exact` in the FFI, `core/mesh.ts::exact`, `app/export-worker.ts` (its own core, off
  the drawing thread), `File ▸ Export solid (STEP)` and the toleranced STL. This kernel is the
  default whatever the binary was built with; OCCT answers `--kernel occt` and is the oracle.
- **The same bytes.** A file is one file only if the arithmetic is: the platform's `sin`, `atan2`,
  `exp` and `hypot` differ by an ulp between macOS's libm and the wasm build's, and a fitted sheet
  turns an ulp into another file. The core computes them itself (`fmath`; see the revisions after
  phase 5). Native and wasm agree byte for byte on the static goldens (flange, pulley, tray, loft,
  indexed pattern, pierced sphere, the V-twin cylinder: 14 files) and on the pair's STEP and STL at
  10 µm. The bits `fmath` moved in the solver were re-recorded where a test records bits.
- **A sweep placed once** is no sector: where a sector's premises fail (`sector::premises`: the
  placements not turns of one, no side through the gaps, a side the field reads as removed) the body
  is built whole (`sector::whole`: the blank split by every placed sheet, its cells judged, the one
  material cell kept), as the native host did, and says why. The swept torus at 0.1 µm is the case.
- **Full verification reads our file back by OCCT** wherever the binary has it (`cad::read_back`):
  a valid solid of as many faces, measured by the core within 1e-5 (the torus 3.7e-7). A reader's
  rational pcurve (a conic on a plane) is read as the `Pcurve::Inverse` of its edge, from its ends in
  homogeneous coordinates, so the core builds no rational curve.

- **The bridge is gone.** Phases 3 and 4 proved each core producer inside the native construction
  (`SOLVENT_WRITER=rust`'s core split, sheets and sections, shapes the core held under the kernel's
  handles, and their dump and check switches). Once `brep::sweep` built the whole body, that hybrid
  only duplicated it, and it was removed: the OCCT path is OCCT's again, the oracle, and
  `SOLVENT_WRITER=rust` means phases 1–2's reading of the kernel's shape into our writer and mesher.
- **The generating harness** (`tests/generating_harness.rs`, the fixtures, the controls and the
  48-design sweep, each a `solventc` run located by its stage trace) found six things, each fixed
  where it was: the core export marked no `Written` stage, so a file written read as a refusal;
  a cutter face meeting one meridian section twice — the neighbour of a gear's tooth space, a
  revolution about another line, cut in two by the half-plane, which OCCT's Boolean had split into
  two faces — is now told apart by its order from a canonical start of the loop
  (`cutter::OCCURRENCE`); the whole construction turns its blank's seams opposite the cuts, as the
  sector does; a trace running off a sheet's patch ended only when its signed distance left the bar,
  which past the edge runs on along the tangent extension — it went back and forth there a hundred
  thousand times — and now ends by the distance to the patch itself (`Surface::off_patch`); a curve
  of more than 200 points is interpolated in its band (`nurbs::BANDED`, Gaussian elimination without
  pivoting in a totally positive matrix), the dense solve being cubic; and a gross sheet whose
  centripetal fit leaves its contacts is fitted by chord length. The pair's files changed with the
  traces (the same faces, edges and cells; fewer poles, the curves stopping at the patches) and
  measure as before: STEPs 0.424 and 1.099 µm, STLs 5.11 and 5.74 µm, the pair check passing.
  The sweep exports 28 designs (OCCT's path 26 on 2026-09-23); 10 are refused by the class and 10 by
  the construction or the kernel at named stages, none timed out.

**Gate:** the native files byte-identical between the CLI and the wasm build (above); the slow tier
with OCCT green — the B-rep oracle, the native sectors and surfaces, the pinion and the gear at
10 µm read back by OCCT and measured within it, the swept torus, the core's slow tier (1376); the
web suite (255). The pair exports in 8.3 and 9.7 s natively and 22 s a file in node's wasm, one core.

## The scope ladder: every solid Solvent defines

Phases 1–5 retire OCCT from the gear. Each rung below widens the kernel to more of the language,
and each is a decision of its own, taken on phase 0's measurements and the previous rung's
experience. Until a rung lands, what it covers stays where it is today: OCCT where the build has
it, the core's facet kernel (`--stl-backend mesh`) or field mesher otherwise, and a STEP refused
by name without OCCT.

The language has seven solid forms (`SolidDef`: prism, through, revolve, loft, placed, swept,
body), and the OCCT path already refuses part of what it allows:

| solid | OCCT path today | rung |
|---|---|---|
| prism, through, revolve (full or partial), placed, body — line/arc/circle profiles | built | 1 |
| the same with a **spline** profile | refused (`solid::cad`: "CAD profiles currently require lines, arcs or circles") | 2 |
| the same with a **traced or formula curve** (an involute, `std.Ellipse`) | refused | 2 |
| **loft** along a guide (`solid_loft`) | refused (`solid::cad`: "does not yet support along-guide lofts") | 2 |
| a swept cut of the generating class | built, on our sheets | phases 1–5 |
| **any other continuous sweep**: swept stock (`swept_tumble`, `swept_spring`, `swept_torus`), a sweep `union`ed or `bound`, nested sweeps, tools outside rows T1–E4 | refused; field mesh only | outside the ladder |

A kernel that climbs rungs 1 and 2 covers more than OCCT covers here today — OCCT could build
splines and lofts; nothing ever handed them to it.

### Rung 1 (phase 6) — static solids of line, arc and circle profiles

- Prisms, partial revolutions (planar end caps), placements, and bodies over planes, cylinders,
  cones, spheres and tori: analytic × analytic intersections in closed form where they have one
  (plane × quadric; coaxial quadrics as circles), marched with phase 3's tracer where they do not
  (a torus against a cylinder is a quartic), through the phase 3 arrangement.
- **Coincident faces**, which the gear avoids and Solvent documents write routinely: a flush bore,
  a boss in `union` with its stock (the shared face counted once), parts mated `against` each other,
  coaxial cylinders of one radius. Coincidence is decided exactly where the geometry says it
  (two planes, two coaxial quadrics of equal radius: the same surface to the tolerance) and never
  by an intersection that happens to come out tangent; coincident pieces are merged, their
  material sides compared, and the arrangement built on the one surface. Phase 0 counts how often
  the corpus does this, by kind.
- Degenerate results — a sliver below the tolerance, an edge shorter than it, surfaces touching
  without crossing — are refused with a witness, never tolerated into the file.
- **Gate:** every static golden and every solid of the corpus with an OCCT recipe: volume and
  faces by kind equal OCCT's, the STEP read back by OCCT, the STL's shells and its sag within the
  bar; the core's facet volume (`tests/solid.rs`'s arithmetic) agrees to its own faceting.

#### Rung 1 — done (2026-09-30)

Begun ahead of phases 1–5, since its solids need no fitted sheets and every piece it builds is
one the gear needs too. In `gcs-core/src/brep/`:

- **`geom`**: frames and rigid motions; plane, cylinder, cone, sphere and torus parameterised as
  OCCT and STEP do, with closed-form inverses, signed distances and gradients; lines, circles,
  ellipses and traced curves.
- **`topo`**: vertices, edges, faces, loops of oriented uses with pcurves, degenerate poles, and a
  check of every promise (loops closed in space and in parameters, pcurves on their surfaces,
  loops turning with their face's sense, every edge used twice, once each way).
- **`build`**: a profile of lines, arcs and circles swept or turned, full or partial, seams and
  poles built whole. **`props`**: the volume as each face's flux turned into a line integral
  round its loops (Green's theorem), nothing meshed. **`recipe`**: the CAD recipe's nodes.
- **`query`**: a curve's roots on a surface; a point placed in, on or out of a face and a solid.
- **`ssi`**: two surfaces' intersection in closed form (two planes; any two surfaces of
  revolution about one axis, by their meridians; a plane with a cylinder or a sphere; parallel
  cylinders), the same surface, or traced — marched along `∇a × ∇b` from where either face's
  boundary crosses the other, read anywhere by pulling a chord onto both surfaces. Surfaces
  meeting at under a degree are refused by name.
- **`boolean`**: edges split where they cross the other's faces, faces intersected pairwise and
  the curves kept where they lie in both, faces on one surface splitting each other, every face
  split in its own parameters, pieces placed by a point inside them, the kept ones assembled on
  shared edges. Tangent touches split nothing.

**Where it stands.** Closed forms (`tests/brep.rs`): every primitive, Booleans of boxes in general
position, pockets, bores, flush and blind faces, perpendicular rods and a pierced ball, to 1e-8 or
better. The oracle (`gcs-cli tests/brep_oracle.rs`, every node of every corpus object's recipe
built both ways): **176 nodes agree with OCCT** — every V-twin part, the throttle, the flange,
the pulley, the tray, the pierced sphere, the spiral bevel's blank and tooth relief — volumes to
1e-9 (traced intersections about 1e-11), face counts equal except where OCCT splits a cap along a
line it only touches (three grub-screw bores tangent to their hubs). **Three crown bodies are
refused by name**: their neighbouring fillet tori meet at 0.05° and 0.87°. The census
(`tests/brep_census.rs`) found the corpus's shared faces all planar but one coaxial cylinder.

**Files (2026-09-30).** `brep::mesh` meshes a boundary within a *measured* sag (every edge
sampled once in space, every face the constrained Delaunay triangulation of its loops in its
scaled parameters, refined where the surface stands off a triangle by more than the bar);
`brep::step` writes AP214 with every edge's pcurves (a seam's as a seam curve) and exact
surfaces and curves, traced ones as B-splines within the tolerance. `solventc --kernel rust`
(and any export of a build without OCCT) uses them for static solids, the STEP checked by the
native export's own parser (`step_check`, against `Solid::of` the B-rep), the STL's shells
checked. Every corpus body the kernel builds exports this way; the V-twin plate builds in 0.5 s
and meshes at 5 µm in 0.8 s. **OCCT's reading is not the gate:** with pcurves most files read
back exact to 1e-13, but OCCT integrates the volume of the spiral bevel's blank 2e-4 off (in two
to three minutes) where the accuracy meter finds every face of the same file on its exact
surface. Two designs are refused because they pinch — the V-twin disc and flywheel, whose
grub-screw holes end tangent to the hub bore — and OCCT's own export of them fails too.

**The gate, as it closed.** Of the corpus's 41 objects, 29 have a CAD recipe (the rest are
swept, or lofts — rung 2); every one of their 176 recipe nodes the kernel builds agrees with
OCCT's to 1e-9 in volume, with the same faces by kind, and every object with the core's faceted
kernel to its faceting. Every body the kernel builds exports: its STEP parsed back against it,
read back by OCCT as a valid solid with its faces (slow tier), its STL closed within its bar.
What it refuses, it refuses by name, and each is a design that touches itself:
- **Three crown bodies** (the spiral bevel's gear-space cutters, `crown/space.sv` and
  `crown/relief.sv`): both crowns' fillet tori are tangent to one shared plane and their tangent
  circles cross in it, so the tori touch each other there — their intersection has a singular
  point. Lowering the limit only traced through it into an invalid boundary (0.05°, and the
  relief's 0.87° falls to 0.34° at half a degree). The refusal names the angle and the place.
  The gear needs these cutters (phase 4), so the answer belongs there: measured vertex and edge
  tolerances, merging within them as OCCT's Booleans do within 1e-5 mm.
- **The V-twin disc and flywheel pinch** where their grub-screw holes end tangent to the hub
  bore; OCCT's own export of both fails as well.
- **Faces by kind** differ only where OCCT splits a cap along a line it only touches (the crank
  disc's and flywheel's grub-screw caps): one plane more on OCCT's side.

**Speed** (release, STEP and STL at 10 µm, best of two, each including the ~0.3 s the document
takes to elaborate and solve): pulley 0.68 s against OCCT's 0.36 s, flange 0.80 / 0.54, tray
0.40 / 0.38, indexed pattern 0.44 / 0.33, V-twin plate 1.68 / 0.73, cylinder 0.62 / 0.31,
throttle 0.68 / 0.54, piston 0.40 / 0.32, the spiral bevel's blank 0.42 / 0.88, the pierced
sphere 0.80 (OCCT's export of it fails its STEP check). Most of ours is the mesher (the plate:
0.5 s built, 0.8 s meshed): its Delaunay insertion scans every triangle, and is the first place
to look when speed matters.

### Rung 2 (phase 7) — spline, curve and loft profiles

- A spline edge extruded is a B-spline surface and revolved a surface of revolution, both exact.
  A traced or formula curve (`curve.rs`'s tapes, `locus.rs`'s traces) is approximated by a
  B-spline within a share of the export tolerance, its error measured against the exact curve at
  the fit — how every CAD system carries an involute. A loft along a line or arc is a ruled or
  skinned B-spline surface between its end sections.
- Their Booleans need **general B-spline intersection**: subdivision of both nets (with interval
  enclosures) until every component is found — small closed loops included — then marching, and
  tangential contact detected and refused rather than traced through. This generalises phase 3
  from the handful of well-conditioned pairs the gear has to whatever a document writes, and is
  where most of this rung's cost and risk is.
- **Gate:** new fixtures with closed forms (a spline prism's volume by Green's theorem, an
  involute tooth's area against the involute's integral, a loft's volume by the prismoidal
  formula), each Boolean against OCCT given the same B-splines, and the meter reading the exact
  curve.

#### Rung 2 — done (2026-09-30)

The gate as written, with one substitution: a swept spline is written as STEP's exact surfaces of
linear extrusion and revolution over the exact B-spline (as OCCT builds them), not as a B-spline
surface, and only a loft's blend is fitted.

- **Spline profiles.** A face admits a clamped spline (its ends its first and last poles). The
  kernel has `Curve::BSpline` (`brep::nurbs`: Cox–de Boor with two derivatives on the stack, degree
  ≤ 9), `Surface::Extrusion` (`C(u) + v z`) and `Surface::Revolution` (`C(v)` turned by `u`). Their
  inverse is the curve's nearest point in the section through the point and their signed distance
  the distance to the tangent line at that foot, which runs on past the curve's ends, so
  `query::curve_surface` keeps a root only where the surface itself is. Green's-theorem volumes
  split at knots. A plane square to a revolution's axis, a coaxial cylinder, cone, sphere or torus
  meets it in circles where the meridians cross (`ssi::spline_cross`); two spline meridians are
  traced. Two faces on one swept surface (a copied or overlapping prism) are one surface.
- **Traced and formula curves in faces** (`f := face(k from p to q)`, spec 0.28): the stretch of a
  curve between two points held on it by their contacts, minted as a hidden curve whose interval is
  the contacts' parameters (`CurveE::trim`, `Sketch::curve_domain`). The faceted kernel walks it to
  the sheet's flatness (`curve_polyline_within`); the CAD recipe carries the cubic interpolating it
  at uniform fractions of its parameter, fitted within `cad::FIT_MM` (1e-4 mm) and the error it
  measured (`"fit"`). A lone edge closed by `-> close` is a loop with its chord.
- **Lofts.** Along a line with no end section a prism, along an arc a revolution; with an end
  section each edge pair is joined at the same fraction of its own parameter, blended while the
  guide carries it: a plane or a cone or cylinder where one exists, otherwise `Surface::Blend`
  (`carry_v((1 − v) A(u) + v B(u))`, exact, periodic for a whole-circle pair) with its rails the
  blend's iso-curves (`Curve::Iso`) where the guide turns. STEP writes a blend as the tensor B-spline
  fitted within `step::FIT` (1e-6 mm), its rails as fitted cubics; `step_check` compares those numbers.
  The faceted kernel now pairs a spline or a stretch by its parameter (it had lofted one as its
  chord), and stations a straight loft where a side twists, `n = ⌈h / 4·flatness⌉` for a side whose
  fourth corner stands `h` off the plane of the other three (it had taken one station, and a
  square-to-diamond loft came out 26% light).
- **The gate.** Closed forms: a two-span spline prism by Green's theorem, a vase by Pappus, the same
  cut level with its caps and halved through its axis, a rod on the spline side (disk ∩ region by
  Green), two overlapping prisms of one profile, a spline plate lofted to half size (7hA/12), a
  twisted and an arc-guided two-section loft (Simpson on the sections' areas and first moments),
  `solid_spline` (cam 3366.807105 mm³, vase 14282.577801 mm³), `solid_tooth` (the involute's
  integral, within the fit), `solid_bend` (1560π) and `solid_twist` (12373.333 mm³). Against OCCT
  given the same B-splines: the cam to 1e-15; OCCT integrates swept splines to ~2e-6 (on the tooth
  fitted within 1e-7 mm this kernel meets the closed form to 1e-9, OCCT is 4e-8 off), and the oracle
  states that bar. Every loft is now written as STEP, read back by OCCT in the slow tier and meshed
  (the oracle had skipped nodes OCCT does not build). **The meter** reads a body with no swept cut
  through this kernel's exact B-rep (`accuracy::Exact`) and its material field, which takes a spline
  or a stretch as chords within `CHORD_SLACK` of the profile's reach: an offset of the cam's spline
  side reads as itself to 1e-14 analytically and within the slack by the field.
- **Found on the way.** A cubic S-bend's middle lies on its chord, so a midpoint test sees a
  straight line: the mesher's edge sampling and `curve::tessellate` test the quarter points too
  (the vase had meshed as one chord, its faceted volume 4% light). The field's convex construction
  intersected its half-planes in a chain past the field's depth limit; it is a balanced tree now.
- Still open, and not rung 2's: a blend's intersections are traced (no closed form), and a
  stretch's STEP is its fit, not the curve.

### Rung 3 (phase 8) — the app reads the exact solid

- An evaluated static solid is the B-rep's (see the revisions after phase 5): built from its CAD
  recipe once per geometry, meshed at the policy's sagitta into one polyhedral primitive whose
  facets carry their faces' document paths, its volume the B-rep's own. Views, sections, hidden
  lines, the solid claims, `dimensions(…)` and the glass box read it through the readers that exist.
- The facet kernel answers what this kernel refuses, named, and a drag if measurement says so.
- A view's silhouettes on curved faces are the surfaces' own, not the mesh's seams.
- After the rung, as its own step: hidden lines and sections classified against the surfaces, and
  the claims measured on them with no sagitta caveat.
- **Gate:** every sheet's SVG, report and glTF against today's within the faceting (then
  replaced as the new record), `tests/derived.rs` and `tests/sheet.rs`, and the drag benchmarks
  (`make bench`) no slower.

#### Rung 3 — done (2026-10-01)

- **The evaluated solid is the exact one.** `EvaluatedSolid::evaluate` asks `Sketch::exact_solid`
  (`solid::Exact`, cached against `solid::reads` alone, so a zoom re-meshes and never rebuilds)
  and hands its mesh in as one polyhedral `Prim` (`from_brep`, `Prim::exact`): every reader —
  boundary, edges, containment, views, sections, claims, `dimensions(…)`, the glass box, glTF and
  STL — reads it unchanged. The volume is the B-rep's. The facet term answers what the exact path
  refuses or cannot mesh (the crown cutters' touching tori; an empty solid), and
  `SOLVENT_SOLIDS=facets` throughout, for comparison.
- **Names.** The recipe's faces are named by the paths the facet term gives them as the B-rep is
  built (`solid::exact`): a leaf's `solid.face`, a placed copy's renamed through its operand paths,
  a body's its operands' (a boolean never renames). The B-rep's prism caps were named the wrong way
  round (`near` is the cap toward the viewer, along the normal), and a straight sweep's caps are
  `start` and `end`. An edge between two faces is named by the face whose facets lead, as in the
  facet term (a prism's and a sweep's caps: `Exact::leading`), so a drawing names its lines alike.
- **Where and how finely.** The recipe is built about the solid's own origin (`cad::shifted`, a
  profile's or a revolution's datum moved within its plane or along its axis to the point nearest
  it), with a Boolean tolerance no finer than the coordinates' rounding where it was read: a part
  solved at 1e9 mm read 0.025 mm³ light until both. The mesh's sag is the policy's but never over
  `BREP_RELATIVE_SAG` (5e-4) of the solid, a circle's or an ellipse's chord keeps its turn however
  short (`brep::mesh`; a fitted curve's short chord still turns as it likes), and a turn is at
  least 64 steps (`BREP_ANGULAR`) — the facet term's rules, so a solid a micron across is meshed
  as finely, for its size, as one a metre across.
- **Silhouettes from the surfaces** (`EvaluatedSolid::silhouettes`): each curved facet carries its
  surface's outward normal at its corners, and a view draws the zero set of n·eye across them, each
  point put back on its surface — a cylinder's two generators, not a zigzag of an irregular mesh's
  seams. Drawn from seams, the V-twin cylinder's sheet took 44 s and 6.4 MB; traced, 0.69 s and
  110 KB, and the picture is the facet term's.

**Gate.** Over the corpus, 321 static solids evaluate exactly and 11 fall back (the spiral bevel's
crown cutters). Against the facet term at a 0.05 pixel length every surviving face set agrees but
one: a 0.018 mm² sliver of the V-twin piston's shank side the faceted wall leaves showing, which
the exact union rightly does not; every volume agrees within the faceting (worst 0.4%, a ball).
The sheets' SVGs read the same (the cylinder and the flange compared by eye); `tests/derived.rs`,
`tests/sheet.rs`, the core suite (1369), the CLI and FFI suites pass, eight tests that compared a
box's volume bit for bit now compare it to rounding, and the index's containment test compares
against the evaluated solid's own term. Exact evaluation of the whole corpus takes 6.4 s against
the facet term's 7.1 s (the V-twin plate 0.29 s against 0.61). The native benchmark is unchanged
(summed medians 64.3 against 64.8 ms; no case outside run-to-run noise).

### Outside the ladder — continuous sweeps in general

A swept stock such as `swept_tumble` is a torus carried through a compound motion; its boundary
is the envelope of a surface family, with folds, several contacts per tool point and
self-intersections. Only the generating class has a construction, and it is ours already (sheets
fitted to traced contacts); OCCT never built one. The certified general swept boundary was the
attempt at the rest and was removed. These solids stay on the field mesher — an STL of the
material field, gated by the field agreement — and their STEP stays refused by name. A B-rep for
them is a research plan of its own, not a rung of this one.

## Sizing (revised 2026-09-30, from what was built and measured)

The first guess was high. It put rungs 1–2 at 13–25k lines of Rust on top of 3–5k for the B-rep and
geometry; the B-rep, geometry, Booleans, mesher, STEP writer, lofts and both rungs came to about
5,900 lines (4,500 of source, 1,400 of tests and the oracle), because a traced intersection (a
chord pulled onto both surfaces) served where general B-spline intersection was budgeted.

What remains, on the same evidence: phase 1 1–2k (the B-spline surface and pcurves, the converter,
per-edge tolerances, point location in the mesher); phase 2 about 1k; phase 3 2–4k (the contour tracer against surfaces of revolution, two or three
sheet × sheet curves, the arrangement over B-spline faces, the construction-tolerance merging);
phase 4 1–2k (the blank, the cutters' sections from the field, the fits); phase 5 under 1k; rung 3
3–5k plus the migration of every sheet's record. Phase 3, the split, is still the largest and the
riskiest, but it is no longer half of everything.

## Verification

Every phase: the full and slow suites, the web suite, the corpus and non-native goldens
byte-identical; the native exports compared with OCCT's by the oracle (faces by kind, the
read-back, the volume to 1e-9 on analytic faces and to OCCT's integration's width on spline ones),
closed forms where they exist, the meter at 10 µm, the field agreement, the pair check. Timings with instructions
retired, on a quiet machine, against phase 6 of `docs/native-speed-plan.md` (pinion 6.1 and
9.8 s, gear 8.7 and 10.2 s).
