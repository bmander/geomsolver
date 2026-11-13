# Changelog

What changes in each release of `cad-kernel`. Versions follow [semver](https://semver.org/):
until 1.0 a minor release may break the public API and a patch release never does.

Write each change under **Unreleased** as it lands. `scripts/release-cad-kernel.sh prepare X.Y.Z`
(in geomsolver) turns that section into the release's, and the release's notes are that section.

## Unreleased

## [0.12.0] - 2026-11-13

- **Free-form faces on the contact's road** (`swept::contacts`, #880 Q1b S6): a piece of contact
  ending on its face's boundary now ends at the root of `n·v` along that edge, so the pieces either
  side of an edge the faces are tangent across meet (the coil spring's tube and its ball ends,
  4.3e-5 apart before); and `piece_curve` takes the tool's `Located` (**breaking**) and unwraps a
  piece across a face's own seam — a parameter the surface gives no period whose box's two sides are
  one curve, an extrusion of a closed spline. brep-bench's `swept/coil-spring-pushed-aslant-through-plate`
  builds, and `swept/spheroid-wheel-screwed-through-rod`, each to a volume made apart from the build,
  and the screwed ball with a flat the closed class refuses builds through the road to its volume
  worked apart.
- **Membership's mesh is held to its sag as measured over every triangle** (`swept::member::Solid`):
  the mesher measures a triangle's sag at its middle and its inner sides' middles, and a point on a
  drafted cone stood 1.1e-3 off a mesh measured 9.5e-4, which `Swept::beyond` then proved outside the
  sweep; the sag is now the greater of the mesher's and each triangle's sides' and middle's distance
  from its face. brep-bench's
  `swept/drafted-slot-screwed-through-rod` builds, to its volume made apart from the build.
- **A contact piece is cut where its burial changes** (`swept::contacts`): under a screw or a slide a
  face's piece is cut, as a corner, where the path of its points passes from dipping inside the tool
  to rising off it, which is where the sheet's meeting with the posed tool leaves its rim; two
  stretches with a buried one left out between them are no longer given a rail as neighbours; a
  smoothly closed B-spline's nearest point found at its seam is refined from the seam's other end
  too and the nearer kept (`nurbs::BSpline`); and a meeting's walked parameters at a vertex the
  face's boundary holds take that node's (`boolean`), where at a sheet's nearly singular corner they
  stood 0.017 off. brep-bench's `swept/spline-bar-screwed-through-rod` builds, to its volume made
  apart from the build: all of Q1b's fourteen cases now do.
- **A foot on a blend's edge is polished in `u` after the coupled step** (`geom::Blend`): where 0.10.0
  stepped `u` alone at every step `v` was held at the patch's edge, Gauss–Newton now takes the
  coupled step throughout and, its foot settled with `v` at the edge, polishes `u` alone there:
  stepped alone throughout, feet the coupled step leaves for the inside were held, and
  brep-bench's box screwed inside a rod came out 0.015 off.
- **No two faces draw one diagonal, and a triangle with no plane is refined** (`mesh`): once every
  face is meshed, a side more than two triangles meet at is split at its middle on each face past
  the one keeping it, so a short traced edge two faces cut off with the same diagonal no longer
  leaves a side used three times (an STL refused as `EdgeUseCount`); a mesh with no such side is
  left as it was. A degenerate triangle (a sphere's first strip across its seam, two corners one
  point) stands off by its distance from its sides, where its sag read 0 and it was dropped, the
  face with it.

## [0.11.0] - 2026-11-12

- **A facet the mesh would count as turned is refined past the turning budget** (`mesh`): a
  facet facing against its surface at its middle, thicker than a tenth of the bar — what
  `Mesh::turned` counts and an exporter refuses — is refined whatever the budget that bounds the
  refinement for a turn short of that, as a sag past the bar is. A long face with a tight bend
  (a helical gear's tooth space, its root where the rack's tip corners pass) spent the budget
  on its bends first, and left a facet inside it turned over.
- **A circle touching an algebraic surface reads alike at every attitude** (`query`): beside a
  touch, the run a circle or an ellipse keeps within the tolerance of a plane, quadric or torus is
  proved by the Taylor model of the polynomial the surface's form is along it, as against a
  surface turned from a conic, besides the form over the curve's box. The box is the world's, and
  read into the surface's frame it proved the run only a hair wide: a circle in the plane tangent
  to a torus's tube, at one attitude in a dozen, read past any search's boxes and was refused.
- **An ellipse cuts in closed form under a planar motion, and a helical rack with rounded tips
  stays in the closed form** (#904 A6): `swept::planar_cut::Cutting::Ellipse` (**new**,
  **breaking** for exhaustive matches) cuts as a circle cuts, by Newton from its support point
  toward the instant centre. `swept::extruded::Screwed::read` sections an arc of the profile as
  the ellipse it is square to the axis, so a hob's tip round is carried up the helix with the
  flanks, where it sent the rack's space to the traced, fitted sheet (1.7e-2 mm and 4° off its
  contacts): the space is the rounded tooth's arithmetic to 6e-4 mm³.
- **A traced sheet is laid in its cut's own scale** (#904 A3): a cutter of revolutions' cut has a
  unit, the length its walk runs over at the band's middle station against `trace::REFERENCE_WALK`
  (**new**), so a pair with every length scaled is the same construction. `trace::Tracer` gains
  `unit` (**breaking** for code building a `Tracer` literally) and `sheet::settle` takes it
  (**breaking**: a new argument); `cutter::Loop::scaled` and `Loop::walked` (**new**). Every piece
  of a section loop is asked at the same 32 places, and the missed run measured by its faces'
  length, where a tip narrower than the reach's step was missed.
- **A revolved cutter's band is read where its contacts first reach the blank, and a margin column
  leaping beside the blank costs the sheet that column** (#904 A4): the band no longer moves with
  the half-plane a world axis picks, so a gear builds alike in any frame (`cutter::Cutter::turned`,
  **new**); outside the columns holding the blank, a column leaping a turn round is dropped with
  those beyond it, where every column's rows were trimmed back before it (`trace::charted` takes
  the columns `reached` and returns `Charted`, **breaking**).
- **A fold's gap outside the blank is crossed, not turned at, and a cell thinner than its first
  mesh's sag is sampled on a finer one** (#904 A1/A2): a station holding contacts in the blank on
  both sides of its fold's gap walks across it, where it turned onto the other root and ran a turn
  round the pinion; `query::interior` reads a cell its first mesh vouches no point of again on one
  ten times finer, so a tip relief a hundredth of a millimetre thick is sampled. The configured
  hypoid pinion at the mill its balance finds builds. A fold the admission refuses (E3) names the
  tool face whose generated surface folds.
- **A face's sampled loops are simple polygons** (`mesh`, #904 A9): two chords between the same two
  corners, an arc sampled as the very chord a line is, are split as crossing, so a face whose loop
  is a circle's segment triangulates, where its inside flooded past the loop.

## [0.10.0] - 2026-11-12

- **The contact's road takes a turn and a screw past the axis** (`swept::contacts`, `swept::trim`,
  `boolean`, #880 Q1b S5): brep-bench's seven S5 cases build through the front door, each to a
  volume computed apart from the build and to a STEP file that reads back as the solid — the rod,
  cone and ball spun or swung about a line through them, the coin turned about a line through its
  rim, the pin swung through a ring sector, and the drum's end and the cone's point screwed through a
  rod. Under a turn each sheet lies on the surface of revolution of its section's meridian (a plane,
  cylinder, cone, ball or torus where the meridian is a line or a circle, else a fitted
  `Surface::Revolution`), its rails circles about the axis, so sheets meet in circles; a section is
  cut where its meridian folds back and where it crosses the axis, a pole of the sheets either side;
  a part whose path dips into the tool on both sides is buried and left out; a contact along a
  face's own edge is the edge's fan's, and one along its seam its own; a piece beside a cone's apex
  runs on to it. `swept::contacts::carried` takes the surface a sheet lies on and the tolerance, and
  `Carried` gains `blend` (**breaking** for code calling it or building a `Carried` literally). A
  sheet's meeting with a posed face is read only where the face is, and not along the rim lifted
  whole turns on where it touches the face again; a trimmed piece inside the tool at an end of the
  roll is read so exactly, and one of that end's own faces by how it leads the motion where
  membership cannot settle. The arrangement takes an edge's crossing with a given meeting where the
  edge crosses the given curves (two sheets crossing at a degree) and leaves a sliver no outer
  cycle holds, a millionth of its face, out. A STEP pcurve leaving a surface's pole is written by
  its chord there, where its rate was NaN. The screwed bull-nose builds through the road too.
- **A point on a blend's edge is inverted onto it exactly** (`geom::Blend`): Gauss–Newton held `v` at
  the patch's edge but kept the coupled step in `u`, and a point of a sheet's end rim where its two
  rates nearly agree settled 3.8e-8 along the rim, which broke a loop in its parameters at the
  kernel's tolerance (the torus screwed through a rod, refused there since the turn's sheets came
  in); `u` is now stepped alone while `v` is held.
- **A round between two planes is a cylinder** (`fillet::roll`, `fillet::flat_along`): a run
  between two planes that ends with no tangent run-on is the prism on the round's section (the
  corner, its legs to the contacts and the ball's arc) carried past each end and cut off there,
  with no Boolean where an end is one plane square to the run, where it was a fitted 65 × 129
  bicubic canal cut by a Boolean. brep-bench's `fillet/box-one-edge` goes from about 120 ms and
  835 KB of STEP to a few milliseconds and 15 KB, `fillet/notched-cube-inside-corner` from 1.8 s
  to about 0.1 s, each to its closed forms. `topo::EdgeCurve::kind` (**new**).

## [0.9.0] - 2026-11-12

- **A sheet's meeting with the tool beside the rim it touches is read off the rim's exact rate**
  (`swept::contacts`): the rate across the rim was read by five-point differences of the distance
  a thousandth of the roll apart, and on a tool whose carry moves far in that step took the series'
  truncation (2.7e-5 for 1e-8 on the leaning torus), so the meeting beside the rim stood off the
  tool; and a branch passes smoothly from the rim's reading to the distance's own. The three S4
  cases now build at the kernel's own tolerance, the torus among them.
- **A tool the closed sweep refuses under a constant twist goes the contact's road**
  (`swept::closed::closed`, `cut_from`): what it refused as `Curved` is built from its contact's
  sheets, the end poses and their given meetings, trimmed by membership and judged; where that
  road refuses too, the `Curved` refusal stands. brep-bench's `swept/v-wheel-screwed-through-rod`
  and `swept/drum-leaning-to-a-screw` build through it, and the torus through its rod, each to a
  volume computed apart from the build and to a STEP file that reads back as the solid.
- **A sharp edge's fan is swept** (`swept::contacts::fan_runs`, `candidates`): each stretch of a
  sharp edge where its two faces' `n · v` differ in sign is carried as a sheet of its own, where
  the candidates refused the tool. `swept::contacts::Carried` gains `rails`, the paths of a sheet's
  ends (**breaking** for code building a `Carried` literally).
- **Membership reads exactly any tool that is where all its faces' surfaces have it inside**
  (`swept::member`): faces planes, cylinders, cones, balls or tori, the greatest of their signed
  distances, taken where every point of the tool's mesh stands on each surface's inner side and
  held to the tool's point location; it was one ball or one torus.
- **A torus screwed along a rod is swept, trimmed and cut** (`swept::contacts`, `swept::trim`,
  `swept::member`): brep-bench's `swept/torus-screwed-through-rod` to the groove's volume computed
  apart from the build, within 3e-5 at a tolerance of 1e-5. A tool bounded by one ball or one
  torus is read for membership by its exact signed distance; a trimmed piece proved inside or
  outside the sweep is dropped before its sides are read; and a fitted curve merges nodes within
  the tolerance, where a cubic through two nodes a hair apart swung 160 off. It builds to 1e-6 as
  well: a given meeting is held to the tolerance wherever the arrangement reads a reach, a touch
  run reaching an edge's own end is read at that end, and an edge whose search against a given
  partner is refused is read off the given curves.
- **A sweep's sheets are given their meetings with the tool at the roll's ends**
  (`swept::contacts::candidates`): what of a sheet passes back into the tool where it touches it
  along a rim, the zero set of the distance over its double root, read in the sheet's parameters,
  in place of a march that stopped short of the rim; and a given meeting is held to the tolerance
  where two meetings are crossed (`boolean`'s `triple_points`). Every pair of the leaning torus's
  candidates is now given, none searched.
- **A sweep's contact is cut at its double points** (`swept::contacts::face_contact`): a double
  point standing on a node of the grid (the leaning torus's, on its seams' crossing) was walked
  through twice as one closed piece with two corners, and every sheet carried from it was creased.
  A piece turning sharply is now cut there, its ends `Singular`, and the shared paths of the
  stretches ending there are placed on their exact end. The leaning torus's trim runs in 6 s.
- **A sweep's contact sheets are fitted at a cubic's rate and given their meetings with one another**
  (`swept::contacts`): a piece of contact is interpolated through points of the contact, refined
  where a span strays from it, where it was read along its polyline and doubled to four thousand
  spans; and every two sheets of one carry are given the paths they share, where their sections
  cross or touch in the carry's invariants, in place of a search. The leaning torus screwed through
  its rod: an inverse on a sheet 750 µs to 28 µs, every pair of its candidates arranged, the trim
  825 s to 26 s.
- **A meeting arriving at an edge two surfaces touch along ends on it** (`boolean`'s `landing`,
  `ssi::trace_landing`): near the junction the two part as the square of the distance, so a march
  stopped short; the stop was read as a touch of its own, its other branches were traced along the
  edge, and the arrangement refused ("a blend's pieces do not close"). The branch is now followed
  down to the edge at halved gaps and ends where it meets it: a screwed torus's carried contact
  arranges with the start pose it passes back through.
- **A recipe's `hull` node** (`recipe::Node::Hull`, **new**, **breaking** for exhaustive matches):
  the convex hull of its `points` (`build::hull`), each face called its `face`, so a solid read as
  the hull of finitely many points is built without a Boolean.

## [0.8.0] - 2026-11-11

- **The convex hull of points, as a solid** (`build::hull`): one plane face for each facet, its
  topology built whole with no block cut and no Boolean, its corners the points given. Quickhull
  on the exact orientation test finds the triangles, so points on one plane never decide by
  rounding; the triangles within the tolerance of one plane are one face, grown from the
  largest's plane and kept a disc, and a point midway along an edge is no corner. A tetrahedron,
  an octahedron or a pyramid is its corners' hull where it was a block cut by each face's plane
  (brep-bench's offset/tetrahedron-grown, swept/tetrahedron-screwed and the octahedra), and a
  convex solid slid along a line the hull of where it starts and where it stops. The filtered
  orientation and power predicates move into the kernel as `util::predicates`.
- **A circle or an ellipse against a surface turned from a conic, in closed form**
  (`query::conic_revolution`): the turned surface's algebraic form, read span by span from each
  arc's Bézier poles (`F(ρ, z) F(−ρ, z)`, or `F` alone for an arc symmetric about the axis), is a
  trigonometric polynomial along the curve, whose roots are isolated and whose runs within the
  tolerance are proved by its Taylor model. A ball's meridian meets the paraboloid that fits its
  bottom within the fourth power of the way from the pole it ends at, and the general reading,
  seeing a whole patch nearly coincide, gave up past its boxes: brep-bench's
  `tangent/ball-in-a-paraboloid-cup`, refused by 0.7.0 ("where a circle … meets a revolution was
  not proved"), builds, union, common and cut, to its closed forms.
- **A polyhedral tool's screwed sweep builds in a third of the time** (`swept::closed`; brep-bench's
  swept/tetrahedron-screwed, 2.9 s to 1.0 s on four cores, the same solid): a line against the
  sheet a straight edge is screwed into is read in closed form where it crosses cleanly
  (`query::line_screwed_line`: one unknown, the ruling's `v`, where the general reading isolated
  three), a touch or a meeting at an end still by the general reading; such a sheet's flux is
  integrated along its rulings by two Gauss nodes, exact there (`props`); a cell's interior
  points read a winding once for every ball clear of its mesh, not once a candidate
  (`query::interior`, the same points chosen); and a Boolean's edge-against-face and three-face
  readings run on every core, taken in order (`boolean`, the same vertices).
- **A root met flatly goes onto a crossing found beside its edge** (`boolean`'s `flat_roots`):
  where an edge of the other operand lying along a face the edge bounds crosses the edge, that
  crossing is one a root met flatly may be, as the edge's own cuts were. A torus's circle at a
  sweep's start pose meets the fitted sheet touching the torus along its rim within the tolerance
  of the touch, and since 0.7.0's restricted Bézier enclosures the search proves the fit's own
  crossing there, 2.7e-4 off the rim's: two vertices, and the sheet's face never closed. brep-bench's
  `swept/tumbling-torus`, built by 0.6.3 and refused by 0.7.0 ("a bspline's pieces do not close"),
  builds again: the same solid, in 6 s on the bench's runner where 0.6.3 took 15.
- **A face's contact under a slide, a turn or a screw, for any surface, carried over the roll**
  (`swept::contacts`, **new**, #880 Q1b): `face_contact` reads it as the zero set of `n·v` over
  the face's parameters in pieces (`Contact`, `Piece`, `End`), whole where the motion keeps the
  face in its own surface; `piece_curve` fits a piece, `carried` carries it exactly over the roll
  as one blend face (`Carried`), and `candidates` gives every face's sheets and the tool at both
  ends with their meetings given. A blend face carried by its own carry is inverted off the
  motion's invariants, exact to rounding. `query::Located::parameter_box` and `holds_parameters`
  (**new**) read a face's outline in its parameters.
- `util::space::Region`'s `Vertex` and `Edge` name which vertex or edge of the triangle,
  `Vertex(u8)` and `Edge(u8)` (**breaking**). `util::par::map_in_order` and `Meter::mark` /
  `restore` (**new**): parallel readings under a budget are refused at the same point every run.

## [0.7.0] - 2026-11-11

- **Two rounds tangent to one face union wherever their meeting ends at the touch**
  (`ssi::trace_beside`, `boolean`): two surfaces' meeting through a point where they touch and
  its branches cross ends there, and a march that steps across such a touch lying on a face's own
  edge now ends at the touch, where it ended a step past it, off that face ("beside a meeting
  known"). The curve then ran through that vertex twice, one stretch of it was never laid, and
  the union was refused ("a shell of the arrangement is kept on one side … and dropped on
  another") or built, as the steps fell: a box's corner rounded 5 and its top rim rounded 1
  refused, the same at its bottom rim built. A closed curve with no period has the vertex both
  its ends are at both of them wherever along it that vertex was read (`boolean`'s `spans`):
  read at its far end, the stretch from the last point on the curve round to it was dropped.
- `recipe::Named::said` (**new** field, **breaking** for code building a `Named` literally) and
  `Named::called()` (**new**): what a failure to build a
  node calls it, where the recipe's writer says (`"said"`), else its name between backticks — a
  node named by nothing its author wrote says what it is of. A body's `built_names` hold what
  `called()` gives, no longer bare names it puts backticks round.
- **A rail screwed many turns is inverted on its own turn** (`geom::Curve::inverse` of a
  `Curve::Iso`): Newton is seeded from the nearest of the rail's samples and of the points where
  its screw turns it to face the point, one a turn, by the one seeding `Blend`'s own inverse uses.
  From the samples alone, a rail whose pitch is small beside its radius had half its points read a
  turn off, so two edges along one helix were never merged: the square bar screwed four turns
  (brep-bench's swept/square-bar-screwed-through-rod) was refused with "a cylinder's pieces do not
  close", and now builds, its volume the closed form's to rounding, and cut from the rod gives
  the bench's settled volume and area.
- **Faster Booleans and evaluation on spline patches**: face pairs whose boxes overlap are
  proved apart where subdivided, outward-rounded surface enclosures separate them, avoiding
  expensive intersection seeding on disjoint sheets; unresolved pairs use the ordinary reader.
  The proof is `boolean::separated_nets` (**new**), made once for every two nets.
  Clamped cubic Bézier basis functions and their first two derivatives are evaluated directly.
  The full brep-bench Utah teapot run falls below one second on the development benchmark,
  preserving the mesh geometry and measurements to rounding (the existing pole-related
  measurement discrepancy is unchanged).
- **A finite screw whose roll does not carry its section past the blank builds**
  (`swept::helical::helical_sheet`): where an end of the section's columns that the roll cut
  short stands in the blank, or no column is left, the sheet is laid on the characteristic
  itself carried through the roll's own poses, `M(t)·c(s)`, where it was refused ("the roll does
  not carry the characteristic past the blank", or points of the sheet's edge in the blank).
  `constant_twist::Characteristic::at` reads a stretch whose two nodes are one point, a closed
  face's seam, as that point, where it read a ring between the face's last station and its first.
- **A hypoid gear's part builds in half the time** — four costs, each measured on geomsolver's
  hypoid drive and pair:
  - `swept::tool::Material::near` (**new**, default `|value| < r`): whether a point is within `r`
    of the material's boundary; `Tool::on` asks it, and `read::Boundary` reads it off the faces and
    edges within `r` alone, the first found deciding, where `value` read every face and edge nearer
    than the best found (a traced edge's foot by projections onto both surfaces). A gear's
    admission: 9.6 s → 3.8 s.
  - `query::Located::solid_place`: a point off the box its faces' boxes fill is out, and a ray
    reaches only out of that box, its directions tried by how soon they leave it; `settled` asks
    whether a stretch's end is on a net only where it stands by one of the net's spans
    (`near_net`), where it searched for the end's foot on every sheet a ray passed; `along` likewise
    at a curve's start, middle and end. A part's cut of its bore:
    1.8 s → 0.19 s.
  - `props`: a polynomial net's `G` tabled from the face's middle, each column filled only as far
    as asked, a span at a time by the net's rows read at that `v` (curves in `u`), where every
    column was integrated across the whole net in 2D; an inverse edge's Gauss nodes sought each
    from the last (`topo::Pcurve::at_and_derivative_from`, **new**).
- **Certified curve–surface readings several times faster on fitted sheets** (#814's general
  reading): a non-rational B-spline curve or net of degree below `enclose::BEZIER` is enclosed
  over a box by its spans' Bézier forms restricted to the box (`Net::restricted`,
  `BSpline::restricted`: blossoms in floats, each hull widened by its rounding), not de Boor in
  intervals — far tighter, so a search reads a fraction of the boxes, and each at a fraction of
  the cost; `isolate::solve_by` halves a box across the side it smears farthest along (its width
  times its Jacobian column), not the widest in the unknowns' own scales. geomsolver's hypoid
  pinion, as its document writes it, refused by every build since the certified readings (0.5.0) —
  its side's spline edges, hugging its cut's sheet of 240 × 66 poles, read past any search's boxes
  over the whole net — splits its sector in 1.7 s (3.2 before the certified readings), its volume
  as before them; a bull-nose screwed instant by instant builds in 10 s where it took 28. Where the
  whole net still reads past any search's boxes (a circle along a quintic tube's rim), the curve is
  read over the hull of the spans it can reach (`query::reached`).
- **Closed sweeps by new roads, asked before the arrangement**: a whole turn of any tool is the
  solid of revolution of its shadow in a half-plane through the axis (`swept::shadow::turned`,
  **new**, #880 Q1), built from exact curves with no surface met in space; brep-bench's
  `swept/v-wheel-turned-once-round-rod`, which the arrangement refused, builds in a tenth of a
  second. A prism under a motion that turns one plane in itself is swept slab by slab as the
  region its section sweeps in that plane (`swept::planar_solid::swept`, **new**, #880 Q2;
  `swept::subtraction::kept_normals`, `Prismatic::split`, `slabs`, `setting`, **new**). Candidates may
  be trimmed by certified membership instead of split on one arrangement (`swept::trim`, **new**:
  `trimmed`, `Trim`, `Report` and the candidate builders; `boolean::trimmed` and `boolean::Meet`,
  **new**, the arrangement's phases under a filter of the face pairs met; #880 E1).
- **A plane's cut of a screwed sheet is crossed with the ruling it holds** (#885): the cut has its
  domain as its stretch, and its crossing with a held ruling is read in closed form
  (`geom::SheetCut::held_along`, **new**), so an edge lying along that ruling is cut where the two meet.
- **The constant twist walks a prism's silhouettes and cap contacts through its vertices** (#721
  G1b), flat contact lines through convex cap vertices among them.

## [0.6.3] - 2026-11-09

- **Measures resolve an edge passing close to a surface's pole** (`props::settled`): where two
  quadrature readings disagree, refine their intervals locally instead of stopping after six
  doublings of the whole edge. brep-bench's `boolean/unequal-lens-aslant` now measures its two
  spherical caps to rounding, where its volume was 1.5% too large and its area 0.28% too large.
- **Edges rounded together, in one call** (`rounds::rounded(b, edges, r, tol)`, **new**): where the
  edges are straight, between planes, and each vertex they reach has all three of its edges among
  them (all convex or all concave), the solid is built anew by surgery on its boundary: each edge a
  cylinder, each face shrunk to the contact lines, each corner the triangle of the ball touching its
  three faces, nothing intersected or fitted. brep-bench's `fillet/box-all-edges` is built in a fifth
  of a millisecond, twelve cylinders, six planes and eight spheres, its volume and area its closed
  forms' to 5e-16. A set of edges no two of which share a vertex is rolled edge by edge
  (`fillet::roll`); anything else is refused by name.
- **Faster measures on analytic faces**: `props::area`, `volume` and `fluxes` integrate along one
  parameter of a plane, a cylinder, a cone, a sphere or a torus (and along an extrusion's rulings) in
  closed form, where a Gauss rule ran at every node of every edge: the same values to rounding, some
  five to twenty times faster.
- **Faster meshing, the same triangles**: `mesh` reads each point's place and normal once, however
  many triangles and passes ask for it, and looks for a point to weld to in a `util::space::Grid`
  of cells eight welds wide, one cell mostly, not 27: every mesh bit for bit as before, in about a
  third less time.

## [0.6.2] - 2026-11-09

- **A ring's sweep cut from a blank** (`swept::closed::ring_cut_from`, **new**): the blank split by
  the ring's candidates at once and kept where the sweep is not, as `cut_from` cuts any other
  tool's sweep. brep-bench's `swept/ring-pushed-in-at-a-lean` (a torus slid into a block at 35°)
  and `swept/torus-turned-across-block` (a torus turned about an axis skew to its own) are built,
  each within the Monte Carlo of its definition.

## [0.6.1] - 2026-11-09

- **A ring's tubes are refined only where they miss** (`swept::rings::fitted`): each interval along
  the roll and round each face is halved where the fitted net misses at its middle, with its
  neighbours kept within twice its length, where every one was halved at once. A quintic's miss
  falls some sixtyfold a halving, so halving them all overshot the tolerance as much wherever it
  was barely missed. The tumbling torus's tubes fall from 170 840 poles to about 14 000 (its STEP
  file from 16.5 MB to 1.6 MB), and its build from about 30 s to 15 s, its volume unchanged.

## [0.6.0] - 2026-11-09

- **A ring swept by a motion no constant twist takes is built closed** (`swept::rings`, **new**;
  `swept::closed::ring`, **new**, not yet a class the dispatch admits). A tool of revolution whose
  meridian is a loop clear of its axis (a torus, any ring of lines and arcs) has its contact read in
  closed form in each half-plane through its axis (`Ring::read`, `contacts`), its folds read from
  how the tool's distance bends over the roll, and each contact curve entered as a closed tube
  (`fitted`, `Fitted`, `Side`): its buried stretch dipped so it crosses the tool at the end poses,
  each face of a folding tube cut where it crosses the tool there and kept only outside it
  (`traces`, `trimmed`, `entered`, against `Ring::signed` and `Ring::holds`). The tumbling torus is
  built closed, its volume within the Monte Carlo of its definition. `boolean`'s given curves may
  end on one another.
- **The constant twist walks across a concave corner, through an apex and round a vertex**
  (`swept::constant_twist`). Where both faces' characteristic points meet a concave corner the walk
  crosses it and creases the sheet; where they part it ends a stretch and walks the others
  (`Characteristic::others`, `Characteristic::concave`, **new**), each stretch's sheet fitted on its
  own and the blank split by all of them at once (`swept::sheet::Fitted::parted`, **new**:
  **breaking** for code building a `Fitted` literally). A face of revolution meeting its own axis (a
  cone to a point, an ogive) is walked down one root to the apex and up the other
  (`Characteristic::apexes`, **new**); a prism's corner, rim or crease running to the end of its
  ruling goes on round the vertex there along the next fan (`Characteristic::vertices`, **new**).
  `Characteristic::sharp` and `Characteristic::concave()` name the creases.
- **Cutters are read from the tool's B-rep**: `swept::cutter::Cutter::of_boundary` and
  `section::Sectioned::of_boundary` (**new**) section a tool of revolutions about one line or about
  parallel lines (the spiral bevel's and the hypoid's) from its faces; `Cutter::read`, which read a
  recipe, is gone (**breaking**; `Sectioned::read` of a recipe stays).
  `swept::admission::Lone::read` reads a prism standing alone drafted (one way, or both from a
  parting plane), holed, of any loops, its cutter the prism of its widest section, and its caps are
  cut where it is undrafted or drafted with plane sides (`Lone::sectioned_across`, **new**).
- **A point's path under turns and slides, exactly** (`trochoid`, **new**): `Trochoid`, a sum of
  terms `(a + b t) e^{iωt}` in a plane, as `geom::Curve::Trochoid` and a recipe's `trochoid` profile
  edge (`recipe::node::Stroke::Trochoid`) — both **new** variants, **breaking** for exhaustive
  matches — cut by a plane in closed form. The planar class's pocket is read from the tool's B-rep
  (`swept::pocket::Pocket::read`, `pocket::Unread`, **new**), the Wankel housing's volume its closed
  form's to 1e-9. `swept::dispatch::Sweep`'s `pocket` is `pocketed: bool` (**breaking**), and `Seen`
  gains `Envelope` (**breaking** for exhaustive matches).
- **A sweep does not depend on where a revolution's chart starts**: `RevolvedSurface::started`,
  `turn_of`, `polar`, `meridian_param` and `over` (**new**); faces a Boolean cut from one surface of
  revolution are read once, over the span of the meridian they reach.
- **A point's foot on a cone or a torus across its axis**: `Surface::inverse` takes the nearer of
  the meridians in the point's own half-plane and the opposite one (a cone's generator past its
  apex, a spindle torus's circle across its axis), and `Surface::feet` (**new**) gives both.
- **Faster point location against fitted nets**: each net's knot spans are boxed in a
  bounding-volume hierarchy, so `query::curve_surface` reads a line against only the spans it meets
  and `Surface::inverse` of a net seeds Newton's method from the nearest;
  `swept::subtraction::Loops` holds its edges in one too. Classifying the tumbling torus's cells
  went from some seventeen minutes to a second.

## [0.5.0] - 2026-11-07

- **A face is split along a meeting through its pole**: an edge laid across a face ends on a pole
  of its surface at the value of the collapsed parameter it arrives along, not where the surface's
  inverse reads the pole, and the face's pole edge is cut there (its pieces closed again by
  `Brep::close_poles`, which `boolean::selection` now runs too). A ball halved through its poles or
  cut aslant through one, two balls meeting through a pole of each, a napkin ring with its ball's
  poles on its rims and a cone halved through its apex (brep-bench's singular cases) are built,
  where each was refused as "a shell of the arrangement … kept on one side … and dropped on
  another". `topo::leaving_pole` (**new**, crate-internal) is the value `close_poles` reads too.
- **A plane through a cone's apex meets it in the generators it holds**, exactly
  (`ssi::intersect`): two lines through the apex where the plane leans to the axis by more than
  they do. A cone cut aslant through its apex (brep-bench's
  singular/cone-cut-aslant-through-its-apex) is built, where the meeting was traced past a hundred
  thousand points and refused at the apex as a near touch.
- **Certified readings by interval Newton on per-kind enclosures, never a guess** (#814).
  `util::isolate` reads each box through a test behind one boundary — `Test` (`Krawczyk`, the
  default, and `Kantorovich`) and `Test1` (`Slope`, the default, and `Bounded`) — over a `System` or
  a `Function` that encloses itself in outward-rounded intervals (`Formula`, `Written`, `Parts`,
  `Poly`, `Closures`: **new**); `solve`, `roots` and `roots_by` take them (**breaking**: the
  second-derivative and `[|g'|, |g''|]` bound arguments are gone, `roots_charged` and
  `poly_bounds` with them). A root is returned only once settled within its certified box;
  otherwise the search gives up as `GaveUp::Unsettled { at }` (**new**; `GaveUp` is no longer
  `Copy`). A `System` encloses its values with its Jacobian over a box (`over`; `Closures` takes
  `at` and `over`), and may offer a frame its equations are read in (`frame`) and prove a run
  through a point (`run`); a `Function` may prove a stretch within the tolerance (`within`); a
  `Test1`'s monotone verdict carries the slope it was proved by (`Stretch::Monotone(Iv)`): a run is proved, its ends read as touches, never
  parted. A stretch's ends' signs are proved by enclosures, and two crossings a hair apart the
  opposite ways are kept as a pair. `util::interval` holds `Iv`, `Num` and `Jet` (**moved** from
  `turned`, which re-exports them) with `Iv`'s hull, meet and ends; `Num` gains `iv` and `from_iv`
  (**breaking** for an implementor). A posed turned curve (`turned::Crossing`, `Section`) reads
  in intervals. `enclose` (**new**): `Enclosed<P>` and `Patch<P>`, an
  enclosure of every `Curve` and `Surface` kind's point and first derivatives over a box, with
  `Carried` (a carry read over an interval, its turn's sine read once), `conic_at` and `scalar`.
  `query::curve_surface` reads every pair: the closed forms as fast paths, the general reading
  (`General`) for every other, through `Reader` (**new**, with `Fast` and `curve_surface_by`);
  `curve_surface_searched` returns `Meets` (**breaking**). The sampled search is retired: a reading
  that proves nothing is `Meets::Refused(Refusal)` (**new**, **breaking** for exhaustive matches),
  a ray abandons its direction, a Boolean is refused with the witness, and `crossings_in` answers
  `Result` (**breaking**). `Net::second_bounds` and `Heights::bounds` are gone (**breaking**);
  `Carry::jet`, `SheetCut::line`, `SheetCut::u_enclosed`, `Heights::coefficients`,
  `query::domain` and `query::curve_domain` are **new**. `Meter::sampled` is `Meter::measured`
  (**breaking**): the readings through a trace's measured enclosure.
- **Meetings given to an arrangement** (#810, #780 R5): `boolean::Given` and
  `boolean::arrangement_given` (**new**) — curves known to be two operands' surfaces' whole meeting,
  laid as an edge of both with no search for that pair (`meeting::Shared::give`, **new**).
  `arrangement_within` is `arrangement_given` with none. The sweep profile counts each pair's
  meetings given beside those found in closed form and traced. `swept::dump::dump` takes them
  (**breaking**: a new argument) and `replay` meets them; `swept::dump::Arrangement` gains `given`
  (**breaking** for code building one literally).
- **A bull-nose's round slid at a lean is built** (`swept::closed`): a torus band between two
  smooth seams, its contact's arc swept into the sheet over that arc of its contour, which the
  plane the neighbour's ruling sweeps touches along the seam point's path — given, not sought. A
  band meeting a neighbour in a sharp rim at a lean is refused as `Curved` at the rim. The closed
  class now gives every touch it knows by construction (#805's, C3's, L2's), and `ssi::bypassing`
  with `ssi::Recognised` (**new**) turns off `intersect`'s pair-by-pair recognition of each, on the
  workers `util::par` starts too (`util::bypass`, **new**: the bits, set for a closure and put back
  however it ends).
- **Membership reads its stretches of the roll best first** (`swept::member::Swept::member`), the
  one whose bound reaches deepest inside first: depth first, a point on the tool at the middle pose
  could spend its reads halving the stretch before it and never read the one after, where it lay
  deep inside, and was refused as `Near`.

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
