# Changelog

What changes in each release of `cad-kernel`. Versions follow [semver](https://semver.org/):
until 1.0 a minor release may break the public API and a patch release never does.

Write each change under **Unreleased** as it lands. `scripts/release-cad-kernel.sh prepare X.Y.Z`
(in geomsolver) turns that section into the release's, and the release's notes are that section.

## Unreleased

- **A loop through a pole is closed along it** (#797): `Brep::close_poles` (**new**), run on every
  Boolean's result, ends each inverse pcurve reaching a pole at the value it arrives along and
  runs a degenerate edge along the pole between the use arriving and the use leaving, as a
  primitive's loop does. `Pcurve::Inverse` ends on a pole at the value its line from `a` to `b`
  gives, not the one the surface's inverse reads there. Two spherical sectors sharing their apex
  (brep-bench's singular/sectors-sharing-an-apex) mesh closed, where their pieces' triangles
  overlapped.
- **Two cones with one apex meet in lines through it**, exactly (`ssi::intersect`): the meeting
  had been traced, fitted with tens of thousands of knots, and measured in seconds.
- **A B-spline sheet lying in a plane is that plane** (`ssi::same`, so `ssi::intersect` answers
  `Same`): every pole within the tolerance of the plane, the sheet within its poles' hull. Two
  faces on them coincide in an arrangement as two planes' do, where they had been two faces
  coinciding along their shared edges. `query::Located::face_holds` (**new**): whether a face
  holds a point of a surface the same as its own, on its patch too where its surface has a rim,
  which the arrangement's coincidences now ask in place of `face_place`.
- **A root on the face two halves of a box share is found** (#796, `util::isolate::solve`): each
  box is read grown past its sides by `isolate::GROW` (**new**), and Kantorovich's theorem read in
  the norm that box is the unit ball of, so a crossing at the middle of a stretch is certified
  rather than dropped by both halves. A ray from inside a bilinear B-spline block met its side
  there and missed it, so points inside the block read as outside, and the block in common with a
  cylinder (brep-bench's `geometry/saddle-drum`) was refused: built again, as 0.3.0 built it.
- **A B-spline sheet's foot is not left on its rim** (`Surface::inverse_near`, `inverse`): a
  parameter held at the patch's edge is let go where the descent pulls it back in, where the foot
  was returned on the edge a long way from a point of the patch.
- **A traced edge is measured across planes** (#798, `props::volume`, `props::area`): a traced
  curve inverted onto its face is read over spans of its points, each node the curve's crossing of
  a plane and its rate there, where each of its thousand stretches was read with three projections
  at each of ten nodes. brep-bench's swept/ball-thread-groove measures in 0.04 s (volume) and 0.1 s
  (areas) where 0.4.0 took 13 s and 26 s, and its volume no longer moves with the solid (by 2e-10
  of itself, read by central differences). A traced curve's `Curve::point` is one projection, not
  three.
- **A ball's pipe is inverted off the stretches of its spine that can hold the nearest foot**
  (`geom::Pipe::nearest_feet`, **new**), not the whole coil: its sheet's side and inverse, read at
  every projection onto a meeting traced on it, about twelve times faster on a coil of six turns
  (swept/ball-thread-groove's Boolean 1.5 s to 0.2 s, its check 95 ms to 4 ms, its mesh 0.7 s to
  0.12 s).

## [0.4.0] - 2026-11-05

- **A round about one axis is turned, exact** (`fillet::roll`): between a plane square to an axis
  and cylinders or cones about it, round a closed loop, the piece is the section revolved — a torus
  face and its strips — where a canal was fitted as a B-spline net of hundreds of poles. A boss's
  rim and root rounded (brep-bench's fillet/boss-s-curve) is built, measured and meshed in
  milliseconds rather than 8.7 s, its faces two tori, not two B-splines. `Rolled::spine` and
  `Rolled::contacts` are then exact circles.
- **A Boolean is its complex's selection, with no other road** (#735 A4): a shell kept on one side
  and dropped on another is refused, where each face was once kept by its own sides; `Complex::try_new`
  is gone. Ways into faces from an edge within 1e-4 rad are tangent (`complex::TANGENT`), a fitted
  round being tangent to its faces only as closely as its fit.
- **Every arrangement lays touches** (`boolean::split` too; `lay_touches` is gone), and every edge a
  meeting laid is healed out of a Boolean's, a split's or a swept solid's result where one face is
  kept on both sides of it (`heal`, `boolean::selection`, **new**); `Complex::met` (**new field**)
  and `Complex::select_cells` (**new**). A split's sheet piece with no inside is merged into its
  edge, not refused.
- **A traced meeting is one per pair of surfaces** (`meeting::Shared::traced`, `add_traced`,
  **new**): traced over the box every face on the two surfaces fills, and handed to every later
  pair of faces on them.
- **A sweep cut from a blank on one arrangement** (`swept::closed::cut_from`, **new**): the blank
  split by the sweep's candidates, its cells kept where they are not swept.

- **The closed swept solid under a screw** (`swept::closed`): a polyhedral tool whose edges run
  along the screw's axis or square to it, on one arrangement — its level edges' helicoidal sheets
  open faces reaching past a box square to the axis (`arrangement_within` takes a sheet as it takes
  a solid). An edge or a face leaning to a screw's axis is refused as `Unbuilt::Leaning` (**new
  variant**), where every screw was `Unbuilt::Motion`.
- A sheet a level line screws into meets a coaxial cylinder, another such sheet of its screw and a
  level plane in closed form (`ssi::intersect`: helices as `Curve::Iso`, a ruling as a line), and
  any other plane in its cut, `geom::Curve::SheetCut` (**new variant**, `geom::SheetCut`), with the
  rulings the plane holds; crossings of such curves on a sheet, or of two cuts on one plane, in
  closed form (`query::crossings_in`); a point's foot on such a sheet from the ruling level with it
  (`Blend::inverse`). Each reads the sheet through `Blend::screwed_line` (`geom::ScrewedLine`).
  A profile screwed into a solid, public (`build::screwed`).
- **Booleans lay touches as edges** (`boolean::combined`, #735): a span two faces only touch along
  is laid in the Boolean's complex, so the cells either side are classified apart, and healed out
  of the result wherever one face of an operand is kept on both sides of it (its pieces joined, a
  slit closed): results keep their faces whole where they are manifold. `complex::Parts::twin_of`
  (**new field**): the edge each twin is of.

- **A body names each operand's faces by a step of its own** (#760): a recipe's body node may
  carry `steps`, the step each operand (by its id) adds to its faces' routes through that body,
  read into `recipe::Body::steps` (**new field**); an operand it does not list adds its name
  whole. A node's own `step` is retired (**breaking**: `recipe::Named::step` and
  `recipe::route_step` are gone, the language's rule for a step its caller's to apply).

## [0.3.0] - 2026-11-02

- A traced meeting dropped as an edge one face already has (#735 A2) is laid across the other face
  where it lies inside it (`boolean::combined` and every selection): a torus cut by the plane
  through its axis that holds its seam is half the torus again, not an open boundary — a regression
  of the surface arrangement (brep-bench's shell/half-torus-open-ends).
- A sheet's net whose first or last row of poles collapses to a point is a pole
  (`build::grid_sheet`): one vertex and a degenerate edge in each face, so the standard NURBS
  sphere is a closed boundary `check` holds.
- A plane through a quadric's asymptotic cone is tangent where the constant at the hyperbola's
  centre is rounding (`ssi::intersect`): a plane holding a hyperboloid's ruling meets it in two
  lines at any tolerance, where at a tolerance under some 1e-7 it came out as a hyperbola's
  branches. The closed swept solid of a leaning block's part turn builds at 1e-7.
- The solid a line bounds turned about an axis, its meridian exact (`swept::closed::revolved`),
  public.
- The closed swept solid (`swept::closed`) is one arrangement of its box and every candidate and one
  selection by membership: no separator planes, cut order, per-cell Booleans or deferred unions.
- Points along a shallow meeting are one vertex (#735 A3): crossings with a meeting read to its own
  reach; a triple point where two surfaces touch placed on the third's root along their curve; an
  edge touching a surface its own face touches placed at their touch curve; two closed-form curves
  of one meeting crossed whether open or closed (a plane tangent to a hyperboloid's throat, cut
  along its two rulings); the faces round an edge ordered where they stand farthest apart.
- **Rounds that take the whole of a face** (`fillet::roll`): a ball whose contact lands on another
  edge of the face it rolls on, or short of it by less than the face can hold, is rolled rather
  than refused — the face goes (a slab's top rounded from both edges to half its width, an L's
  inside edge filled as wide as both faces beside it, a hole's rims rounded to meet at its waist),
  or a strip of it stays between the rounds. Only an edge reaching inside the ball's band refuses
  it: the band is read against the contact's own height rather than 1% past its greatest.
- **Affine maps of solids** (`Brep::mapped`, `affine::Affine`): any invertible `p ↦ M p + t`
  carries a solid onto its exact image, its topology and pcurves its own. A kind whose image is
  its own kind is written so (`Surface::mapped`, `Curve::mapped`: a similarity scales, a plane is
  a plane, a line a line, a conic an ellipse, a B-spline its poles); anything else is wrapped,
  `geom::Surface::Affine` and `geom::Curve::Affine` (**new variants**), its foot found in space's
  metric (exact for a sphere's and a cylinder's image) and its meetings read in the preimage's
  closed forms and mapped back (`ssi::intersect`, `ssi::same`, `query::curve_surface`). Written to
  STEP as each face's exact net with its poles mapped (`step::face_net`), pcurves fitted in the
  net's parameters (`nurbs::arc_parameter`).
- **The ellipsoid** by its numbers (`build::ellipsoid`): the unit ball stretched along a frame's
  axes, one face on the sphere's image, its seam a half ellipse.
- **Faces sewn into a solid** (`sew::sewn`): sheets built apart — a B-spline net with a pole, a
  plane face, any shell's faces — joined where their boundaries are one curve by tolerant topology:
  poles made degenerate edges, vertices merged, an edge cut at another's vertex, edges along one
  another made one; the faces turned to agree, each shell outward and a void inward; refused as
  `Open`, `NotManifold` or `Refused` by what it is. A profile's region of its plane as a face of its
  own (`build::face`).
- Tolerant topology's vertex-onto-edge and edge-onto-edge rules as predicates (`tolerant::on_stretch`,
  `tolerant::same_edge`), which the Booleans and sewing both call; each edge's uses
  (`Brep::uses`); how hard a surface is to invert onto (`Surface::inversion`).
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
