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
- **Tolerance model.** One `solid::export::Tolerance` states the bars, as today. Intersection
  curves are traced to a fraction of it and fitted to B-splines within another fraction; the meter
  reads the finished file, as today.
- **Robustness.** Parameter-domain arrangements (a face split by its intersection curves) use
  exact 2D orientation (added beside `delaunay::predicates`) on polyline approximations whose
  distance to the true curves is bounded; the refinement of a curve near a crossing is driven by
  the predicate's doubt, not by a fixed count.
- **The oracle.** A slow-tier harness builds each stage both ways and compares: volume to 1e-9,
  face and edge counts, faces by surface kind, the STEP read back by OCCT, the meter. OCCT stays a
  CLI feature for that harness only, until the last stage retires it from the export.

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
  analytic): this is the intersection vocabulary phase 4 must cover, measured rather than guessed.
- Record the intersection curves' lengths, the minimum angle between the surfaces along them
  (near-tangent intersections are where marching fails), and the smallest face and edge.
- For the ladder: every solid in the corpus by form and profile kind (which rung it needs), and
  every Boolean with coincident or tangent operand faces, by kind (a flush bore, a boss `on` its
  stock, a mate, coaxial equal radii).
- **Exit:** a table per stage and per Boolean, here; a go/no-go on phase 4 and on each rung, with
  the estimated size of each.

### Phase 1 — the B-rep, the STEP writer and the mesher, fed by OCCT

- `brep/`: topology, geometry, evaluation; a converter from OCCT's finished shape (through the
  existing `brep_summary`-style ABI, extended to curves, pcurves and B-spline data) into ours.
- The STEP writer (AP214 `MANIFOLD_SOLID_BREP`/`ADVANCED_FACE`), checked by our parser and by
  OCCT's read-back.
- The mesher: constrained Delaunay in each face's parameter domain (2D, exact predicates), edges
  discretised once and shared by both faces, refined until each triangle's **measured** sag is
  within the bar — the bound OCCT's deflection never was; the sector's mesh turned into its copies
  as today (`sector_stl`'s seam pairing becomes identity, since the copies share edges).
- Volume by the divergence theorem over trimmed faces (Gauss quadrature in each face's domain).
- **Gate:** for both members at both bars, our STEP of OCCT's shape verifies and reads back in
  OCCT with the same volume (1e-9) and faces; our STL passes the shell checks and the meter within
  the tolerance; the pair check unchanged within 0.1 µm. Time the file stages against phase 6's.

### Phase 2 — the pattern, built with shared topology

- The sector's material turned by the indexing motion into its copies, each copy's boundary faces
  on the source surfaces with pcurves shifted, neighbouring copies meeting on one shared seam edge
  — the construction `solvent_cad_pattern` does by sewing, done by identity.
- Validation of our own invariants: every edge used twice with opposite senses, each loop closed
  in its face's domain, faces' senses consistent (`topology::ClosedShell` over the result).
- **Gate:** as phase 1, with the pattern ours (the sector still OCCT's); the union's 3.8 s gone.

### Phase 3 — the blank and the sheets

- The blank as its meridian region turned once: the region's Booleans are 2D Booleans of lines
  and arcs (easy, exact), the revolution a face per profile edge (`Surface::Cone`, `Sphere`,
  `Plane`, `Cylinder`, `Torus` by the edge's relation to the axis).
- The cutters' meridian profiles: a revolved cutter's section is its own profile; the gear's
  cutters (a crown bounded by its turned neighbour) are 2D Booleans of their profiles in the
  common meridian plane where the neighbour's turn allows, otherwise refused by name.
- Sheet fitting (`fit.rs`'s chord-length and centripetal grids) and the sector sides' fits, by our
  least squares; projection and feet by Newton on the B-spline.
- **Gate:** the blank's volume and faces equal OCCT's (1e-9); every sheet fits its withheld
  contacts as OCCT's did (the fit report the same to the bar); the exports unchanged by the oracle.

### Phase 4 — the sector's split

The hard phase. The sector is the blank between two fitted sides, split by its sheets into cells,
one kept (phase 6 of the speed plan: five cells, one material).

- **Sheet × blank face.** Every blank face is a surface of revolution about the member's axis:
  a curve f(r, z) = 0 in the meridian plane. A sheet point S(u, v) is on it where
  f(r(S), z(S)) = 0 — a **scalar contour in the sheet's own parameter domain**: seeded by a grid
  sign scan (with interval enclosures to prove no small component is missed), traced by
  continuation, refined by Newton, fitted as a pcurve in the sheet and a B-spline in space, with
  the blank face's pcurve read off by the closed-form inverse. No surface–surface machinery.
- **Sheet × sheet and sheet × side** (the removal against the relief; each sheet against the two
  sides): true B-spline/B-spline intersections, a handful per sector and well conditioned where
  phase 0 says they are. Subdivision of the two control nets to seed, marching on the two surface
  equations to trace (the DogLeg loop, as the seams already do), both pcurves kept.
- **The arrangement.** Each face's domain split by its curves (exact 2D predicates), the pieces
  assembled into cells with shared edges, each cell classified by ray parity (as today) and the
  material cell kept by the field (`MaterialEvaluator::probe`, as today).
- **Refusals.** A tangential or near-tangential intersection (the angle below a bar phase 0
  sets), a curve that leaves its face's domain unexpectedly, an arrangement whose pieces do not
  close: each refused with a witness, the export falling back to OCCT while it exists.
- **Gate:** both members at both bars built without OCCT: faces and volume equal OCCT's (volume
  1e-9 relative; face counts and kinds equal), the field agreement and the meter pass, the STEP
  reads back in OCCT, the pair check within 0.1 µm; the 48-design harness
  (`generating_harness.rs`) refuses nothing OCCT built. Time it.

### Phase 5 — the export without OCCT

- The native export's default kernel is ours; `SOLVENT_KERNEL=occt` builds the old way for the
  oracle. `solventc --step/--stl` for the admitted class needs no `OCCT=1` build.
- The FFI and the app: `gcs_solid_step`, `File ▸ Export STEP`, the toleranced STL from the app,
  run in the mesh worker (it is seconds, not milliseconds).
- **Gate:** the goldens' native files byte-identical between the CLI and the wasm build; the
  slow tier's oracle harness green; the web suite.

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
| **any other continuous sweep**: swept stock (`swept_tumble`, `swept_spring`, `swept_torus`), a sweep `on` or `bound`, nested sweeps, tools outside rows T1–E4 | refused; field mesh only | outside the ladder |

A kernel that climbs rungs 1 and 2 covers more than OCCT covers here today — OCCT could build
splines and lofts; nothing ever handed them to it.

### Rung 1 (phase 6) — static solids of line, arc and circle profiles

- Prisms, partial revolutions (planar end caps), placements, and bodies over planes, cylinders,
  cones, spheres and tori: analytic × analytic intersections in closed form where they have one
  (plane × quadric; coaxial quadrics as circles), marched with phase 4's tracer where they do not
  (a torus against a cylinder is a quartic), through the phase 4 arrangement.
- **Coincident faces**, which the gear avoids and Solvent documents write routinely: a flush bore,
  a boss standing `on` its stock (the shared face counted once), parts mated `against` each other,
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
  The gear (phase 4) needs these cutters, so the answer belongs there: measured vertex and edge
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
  tangential contact detected and refused rather than traced through. This generalises phase 4
  from the handful of well-conditioned pairs the gear has to whatever a document writes, and is
  where most of this rung's cost and risk is.
- **Gate:** new fixtures with closed forms (a spline prism's volume by Green's theorem, an
  involute tooth's area against the involute's integral, a loft's volume by the prismoidal
  formula), each Boolean against OCCT given the same B-splines, and the meter reading the exact
  curve.

#### Rung 2 — progress (2026-09-30)

- **Lofts along a line** (`brep::build::loft`): with no end section a sweep is a prism along a line
  or a revolution about an arc's axis; between two sections each start edge is joined to the end
  edge written in its place by the ruled surface between them, which for parallel lines is a plane
  and for coaxial circles and arcs in step a cone or cylinder. A twisted ruled face, and a loft
  between two sections along an arc, are refused by name. `solid_loft` is the prismoidal
  formula's 5120 mm³ exactly; OCCT builds no lofts, so the oracle takes the faceted kernel there.
- **Spline profiles.** A face admits a clamped spline edge (its ends are its first and last poles:
  `topology::edge_ends`); the faceted kernel walks its tessellation and the CAD recipe carries it
  as a `bspline` edge (degree, full knot vector, poles). The kernel has `Curve::BSpline`
  (`brep::nurbs`: Cox–de Boor with two derivatives, on the stack, degree ≤ 9) and two swept
  surfaces, written as OCCT and STEP write them rather than as B-spline surfaces:
  `Surface::Extrusion` (`C(u) + v z`, `SURFACE_OF_LINEAR_EXTRUSION`) and `Surface::Revolution`
  (`C(v)` turned by `u`, `SURFACE_OF_REVOLUTION`), exact and compared with OCCT's faces by kind.
  Their inverse is the curve's nearest point in the section through the point, and their signed
  distance the distance to the tangent line at that foot — which runs on past the curve's ends,
  so `query::curve_surface` keeps a root on a swept surface only where the surface itself is.
  Green's-theorem volumes split at knots, where a spline's third derivative steps; a pcurve on a
  plane is the spline's own poles carried into the plane's parameters.
- **An S-bend hides from a midpoint test**: a cubic is symmetric about its inflection, so a stretch
  centred on one has its middle exactly on its chord. The mesher's edge sampling now starts from
  every knot span in degree + 1 pieces and also tests the quarter points, and `curve::tessellate`
  (which draws a spline on the sheet and feeds the faceted kernel) tests the quarter points too —
  before which the vase below meshed as a single chord and its faceted volume was 4% small.
- `solid_spline` (a cam plate with a spline lobe, bored; a vase with a spline wall) is exact:
  3366.807105 mm³ and 14282.577801 mm³, area by Green's theorem and Pappus. OCCT agrees on the
  cam to 1e-15 and on the vase to 3.8e-7, which is OCCT's integration of a swept spline (asked for
  1e-9), and a `through` prism over a spline face is sized by each kernel's own box (OCCT's from
  its poles); the oracle states both bars. `tests/brep.rs` holds a two-span spline prism and a
  vase to their closed forms, with Booleans traced across the new surfaces: a slab level with the
  prism's caps, a rod standing on the spline edge (disk ∩ region by Green's theorem), the vase cut
  at half height and halved through its axis; meshes closed within the bar. The mesher takes
  0.7 s for the vase where OCCT takes 0.5 s.
- Still to do: a traced or formula curve in a face (it has no end points of its own to walk
  through — a language question first), splines in lofts and twisted ruled faces, lofts between
  two sections along an arc, closed-form SSI for swept surfaces (a plane square to a revolution's
  axis is traced today), and two faces on one swept surface (`ssi::same` reads them as different).

### Rung 3 (phase 8) — the app reads the exact solid

- Everything that asks the faceted kernel (`csg.rs`, `mesh.rs`, `hidden.rs`) asks the B-rep
  instead: views and sections with exact silhouettes and hidden lines, volumes and `dimensions(…)`
  read exactly, the solid claims (`clear`, `fits`, `inside`) measured on exact surfaces with no
  sagitta caveat, the glass box meshed at the screen's resolution from the one B-rep.
- The facet kernel is retired once nothing reads it — or kept as the fast reading for a drag,
  decided by measurement: an exact evaluation per frame may cost more than the drawing can afford.
- **Gate:** every sheet's SVG, report and glTF against today's within the faceting (then
  replaced as the new record), `tests/derived.rs` and `tests/sheet.rs`, and the drag benchmarks
  (`make bench`) no slower.

### Outside the ladder — continuous sweeps in general

A swept stock such as `swept_tumble` is a torus carried through a compound motion; its boundary
is the envelope of a surface family, with folds, several contacts per tool point and
self-intersections. Only the generating class has a construction, and it is ours already (sheets
fitted to traced contacts); OCCT never built one. The certified general swept boundary was the
attempt at the rest and was removed. These solids stay on the field mesher — an STL of the
material field, gated by the field agreement — and their STEP stays refused by name. A B-rep for
them is a research plan of its own, not a rung of this one.

## Sizing (to be replaced by phase 0's numbers)

A first guess, in lines of Rust: the B-rep and geometry 3–5k; the STEP writer 1–2k; the mesher
2–3k; mass properties, projection and classification 1–2k; fitting 1k; the pattern 1k; the blank
and the cutters 1–2k; **the split 5–10k** — about 15–25k in all, a few months, with phase 4 half
of the risk. Phases 1–3 are useful on their own: they remove the union, the mesher's workaround
and the writer's time, and put a verified B-rep in the core even if phase 4 stops.

The ladder: rung 1 another 5–10k (analytic intersections, coincident faces, degeneracy refusals),
rung 2 another 8–15k (general B-spline intersection and its hardening), rung 3 3–5k plus the
migration of every sheet's record — 30–50k in all to cover every static solid the language
defines, and months of hardening past the gear. Each rung is decided on its own.

## Verification

Every phase: the full and slow suites, the web suite, the corpus and non-native goldens
byte-identical; the native exports compared with OCCT's by the oracle (volume, faces by kind, the
read-back), the meter at 10 µm, the field agreement, the pair check. Timings with instructions
retired, on a quiet machine, against phase 6 of `docs/native-speed-plan.md` (pinion 6.1 and
9.8 s, gear 8.7 and 10.2 s).
