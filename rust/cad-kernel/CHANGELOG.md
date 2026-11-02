# Changelog

What changes in each release of `cad-kernel`. Versions follow [semver](https://semver.org/):
until 1.0 a minor release may break the public API and a patch release never does.

Write each change under **Unreleased** as it lands. `scripts/release-cad-kernel.sh prepare X.Y.Z`
(in geomsolver) turns that section into the release's, and the release's notes are that section.

## Unreleased

- **The surface arrangement** (#735). A cell complex (`complex::Complex`): faces each parting two
  cells, an edge bounding any number of faces in order round it (tangent faces by their curvature),
  touches along edges and at points held; `select` the boundary between kept cells and the rest,
  each wedge of a self-touch its own edge, and where it touches said (`Touch`).
- Any number of solids arranged at once (`boolean::arrangement`, `arrangement_within`), every
  meeting laid, touches as edges, coincident faces one face naming each operand; each cell's
  membership in each (`members`); a Boolean of many solids in one pass (`selected`).
- The Booleans (`combined`, `boolean`) and `split` are selections over the complex, with the same
  results; a union touching itself along an edge now comes back a valid boundary (an edge for each
  wedge) rather than one `check` refuses.
- Where two surfaces meet as a topological answer (`meeting::meet`): branches crossing or touching,
  singular points, coincidence, complete by boundary seeds, critical points and the proved search;
  each pair of surfaces' closed-form meeting computed once in an arrangement (`meeting::Shared`).
- Tolerant topology in one place (`tolerant`): the pool of vertices, each keeping its tolerance
  (`topo::Vertex::tol`, **a new field**), and every rule by which two features merge.
- A search's warm start (`util::warm`) is forgotten at every arrangement and parallel item and kept
  across narration: a split is the same bits with `SOLVENT_BREP_DEBUG` set or not, on any thread.
- Arrangements hold degeneracies they refused: a thin piece's side read along its curves where its
  outline is coarser than it is wide; a meeting off a surface's patch (a hyperboloid's rulings past
  its heights) not laid on it; a traced meeting along an edge either face has taken for that edge;
  where two meetings cross on a face with no closed form for it, the crossing found by search
  (`meeting::crossings`) and polished onto the three surfaces; a piece shorter than a vertex's reach
  collapsed; a cell too thin to mesh read inside at points of its faces stepped in
  (`Complex::inside_points`).

- A face of revolution's contacts under any rigid motion, read at each instant (`swept::instant`):
  its chart over roll, ring height and branch (`ring_chart`), and the branch locus where a ring's
  two roots meet (`branch_locus`).
- A polyhedral tool leaning to the axis turned part of a turn closes into its swept solid
  (`swept::closed::closed`), by the arrangement.
- A plane meets a surface of revolution whose meridian is a conic symmetric about its axis (a
  hyperboloid, a cone, an ellipsoid as a rational quadratic) in closed form (`ssi::intersect`): an
  ellipse, a hyperbola's branches or two lines, held to the meridian's heights, where it was traced.
- A Boolean drops a face piece with no inside (a sliver cut off along an edge a surface touches)
  rather than refusing; whether what is left closes is the result's check.
- A convex polyhedral tool turned a whole turn, however it leans to the axis, closes into its
  solid of revolution (`swept::closed::closed`): its slices' radial spans as a profile of
  hyperbola arcs (rational quadratics) and lines, cut where what reaches farthest or nearest
  changes, turned once with no Boolean.
- A plane through a cone's apex leaning to its axis by its half-angle meets it in the generator it
  touches along, in closed form (`ssi::intersect`), where it was traced and refused as nearly
  touching.
- The closed swept solid of a polyhedral tool under a slide, or under a turn when its edges run
  along the axis or square to it (`swept::closed::closed`, `slid`): the candidate
  planes split a box holding the sweep, each cell kept by membership, the kept cells united;
  anything else refused by what it is (`Unbuilt`).
- Whether a point is in a swept solid (`swept::member`): the tool's B-rep meshed, its signed
  distance by the generalised winding number (`Solid`), branch and bound over the roll with the
  trajectory's speed bound (`Swept::member`), unsettled near the boundary rather than guessed.
- A point's, line's or circle's cut under a planar motion (`swept::planar_cut`: a program of turns,
  slides and relatives, `PlanarStep`, its cut and derivative along the roll in closed form) and the
  extruded cutter's sheet built from it (`swept::extruded::sheet`).
- An indexed body's sector (`swept::sector`): its indexing, the side through the gaps its cuts
  leave, the slicings a recipe suggests, the cells a sector is split into and the blank turned to
  its seam.
- The cubic B-spline basis, its knots, Piegl and Tiller's interpolation and a sampled stretch's fit
  (`cubic`); the planar classes' builds (`swept::planar::InnerEnvelope::prism`,
  `swept::region::profiles`, `swept::region::lofted`) and the generating class's traced sheet
  (`swept::trace::traced_sheet`).
- The generating class's rows asked of one placement (`swept::admission`): M2, E1–E4 and T1–T3 for
  a tool under a relative rotation (`check`), T2, E1 and S1–S4 under a constant twist
  (`check_screw`), sampled (`Sampling`), returning what they found (`Found`, `Capped`) and the
  blank's reads (`Reads`); `constant_twist::Row` gains the generating class's rows.
- The planar classes' readings: a closed curve's inner envelope under a planar motion
  (`swept::planar`, its curve a `Section` and its material a `Region`, read in a `View`) and the
  region a planar tool's section sweeps (`swept::region`), each refused by a typed row with a
  witness. A planar profile edge (`plane2::Edge`: a line, an arc, a conic) and its readers.
- A trajectory's pose alone (`Trajectory::pose`, `Pose`) and an interval bound on the speed it
  carries a point or a box back at (`Trajectory::inverse_point_speed_bound`, `_over`), read through
  a pace's value and enclosure (`Pace::value`, `Pace::enclose`) and a path's pose and stretch
  (`Path::pose`, `Path::stretch`, `Path::speed_over`, `Stretch`).
- A ball's tube along a trajectory (`swept::canal`): in closed form along a constant twist, a face
  family fitted to its disc along any other, refused by rows typed with what they saw where it
  would meet itself. The rotation-minimising frame's step (`trajectory::reflected`) and a face
  family's tolerance, longest piece and Lagrange weights (`family::EXACT`, `LONGEST_PIECE`,
  `weights`).

## [0.2.0] - 2026-11-01

- Offsets (`offset`): a solid grown or shrunk by a distance (`grown`, `shrunk`), hollowed to a
  wall open at some faces, inside or out (`shelled`), and a sheet thickened over a range of its
  normals (`thickened`; `sheet_of` lifts faces off a solid), each the union of its faces' normal
  sweeps (`normal_sweep`) and the tubes and balls of the distance set (`Join::Round`) or the kites
  and parallelepipeds of faces extended until they meet (`Join::Sharp`).
- A plane square to a rational spline's sweep meets it in the spline with its weights kept, and a
  closed clamped spline meeting runs round from its last point to its first, not dropped there (an
  elliptic bore cut through a plate was the plate untouched; two cylinders tangent inside at a
  point were refused).
- A face whose chart collapses at the middle of its box (a horn torus) is meshed by its mean
  speed, not shrunk to slivers.
- A chamfer or a round run on past its ends (`fillet::flat_along`, `fillet::roll`) goes on until
  its section is wholly past one cut there, not each of its points past some cut: where two
  chamfers before it mitre at its end, a third took too little off (a cube chamfered edge by edge
  was 809.73 against 808).

- Generating sweeps' contact times (`swept::generating`): a tool point's contact chart along a
  trajectory that turns it, its form read once (`Form`: a sinusoid in the roll under a turn or a
  turn seen from a turn, a line under a slide seen from a turn), its roots
  (`NormalVelocity::roots`), the contact times of a point or a surface over the roll (`Chart`), and
  a trajectory with no chart refused by what it is (`Uncharted`).
- A cutter sectioned at its stations (`swept::cutter`) and its stations' contact curves traced into
  a candidate sheet (`swept::trace`).
- Sweeps of a constant twist (`swept`): a tool read as its faces (`swept::tool::Tool`, faces of
  revolution in `swept::surface`) carried by a screw, a turn or a slide, the characteristic walked
  through a blank with its rows typed (`swept::constant_twist`), and its sheet fitted and judged to
  a tolerance (`swept::sheet`, `swept::helical`).
- A ring's contact under a motion (`contact::RingFrame`) and its roots on a chart of the ring
  (`contact::Sinusoid::roots_over`), and a moving surface point (`contact::SurfacePoint`,
  `contact::contact`).

## [0.1.0] - 2026-11-01

The first release: the kernel as geomsolver builds and tests it.

- Geometry: planes, cylinders, cones, spheres, tori, extrusions and revolutions of curves,
  rational and polynomial B-spline curves and surfaces, traced intersection curves.
- Topology: oriented edge uses with p-curves, seams, degenerate poles, a validity check.
- Construction: spheres, cones and tori by their numbers; prisms (drafted too), revolutions and
  canals of line, arc and spline profiles.
- Booleans: union, cut and common, closed-form intersections where one exists, else traced; a
  meeting the kernel cannot resolve is refused.
- Measures: exact volume and area.
- Meshing: constrained Delaunay refined to a measured sag.
- Output: STEP AP214 with p-curves and seam curves, and STL.
- Errors carry a `Kind`.
- Deterministic elementary functions (`util::fmath`): the same bits on every target.
