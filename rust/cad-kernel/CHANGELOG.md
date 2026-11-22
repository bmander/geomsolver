# Changelog

What changes in each release of `cad-kernel`. Versions follow [semver](https://semver.org/):
until 1.0 a minor release may break the public API and a patch release never does.

Write each change under **Unreleased** as it lands. `scripts/release-cad-kernel.sh prepare X.Y.Z`
(in geomsolver) turns that section into the release's, and the release's notes are that section.

## Unreleased

- **One rule of a surface onto a surface, before a Boolean and after it** (#994):
  `tolerant::one_surface`, new, is the reading `Arrangement::classed` joins faces into one surface
  by (within `ON_CURVE` tolerances over what the faces' boxes share and as far past it as
  `on_meeting` at `LEAST_SINE`; the same to the tolerance where they share nothing), and
  `fillet::one_surface` reads two faces of a body by it over their boxes, where it read them by
  `ssi::same` to the tolerance: faces a union made one surface, each kept on its own operand's
  surface leaning a hair off the others', are one side to a ball and one surface to the field.

- **Two curves on one surface cross within the reach in every reading of `query::crossings_in`**
  (#994): `curve_curve` is read at the reach in every test it makes (its doc says each is a test
  of two curves coming within it of one another); two cuts of screwed sheets and a carrier's
  roots are kept where they lie within the reach of the other curve inside its stretch by as
  much, where the stretch was read by the tolerance; whether two sheets, screws or planes are
  one, and a carrier's roots, are still read to the tolerance.

- **Two surfaces resting on one another at a point, their least distance in closed form, are
  known to meet there alone** (#994): `ssi::meeting_resting`, new, is `intersect` with the point
  `apart` finds them resting at, which `meeting::Shared::known` gives as their meeting (no curve
  and that touch), so an edge lying in either is read against it as every known meeting is
  (`boolean`'s `on_known`), where it was searched and, refused, sampled at 64 points for the touch
  (`resting_on`, which now reads only pairs whose meeting has no closed form).

- **Two surfaces clear of one another, or resting on one another at a point alone, meet in no
  curve by one rule** (#994): `ssi::apart` reads their least distance in closed form where it has
  one (`ssi::least`: two cylinders leaning apart, a sphere and a plane, cylinder, sphere or torus,
  and — new — a plane and a torus), and where it is within the tolerance asks the touch there the
  question an arrangement asks a resting touch (`ssi::touch` definite, `ssi::alone_over`),
  replacing `ssi::rods_apart` and `ssi::ball_clear` (below), each a rule of its own: the rods'
  lean bound, borrowed from `SHALLOW`, now follows from the touch's own radius (`sin θ ≥
  SHALLOW / √2`), and a ball a hair off a tube's axis — within the tolerance of it round a whole
  circle — is no longer read as meeting it in no curve. A ring resting aslant on a plane meets it
  in no curve in closed form, where it was traced. The plane–sphere arm `intersect` never reached
  (the two meet about the sphere's own axis) is gone.

- **A span of a meeting along a face's boundary is a rule of tolerant topology**
  (`tolerant::along_boundary`, new), and whether the edge it lies along was laid
  in the other face is phase 1's answer (`Meets::Along`), looked up, where it was read again from
  samples at the reach and disagreed with it (116 spans in `brep_coincident`'s strips, two in
  `composed_rounds`, the results the same to the last bit).

- **Two cylinders' loops: `D`'s roots isolated, not sampled** (`ssi::cylinder_loops`): whether
  the discriminant keeps above nothing is read from its enclosures (`util::isolate`) where it was
  read at 4096 samples and a golden-section search; the loops' chord bounds are named
  (`LOOP_CHORD`, `2 ON_CURVE tol`, the parameter's own resolution) and a loop past `LOOP_POINTS`
  points is traced rather than cut off. `spill`'s checks that a section lands on the seam read the
  arrangement's reach (`ON_CURVE` tolerances) where they read a millionth of the part's size.

- **A run's end read by one walk, and its sides by one rule** (#994): new `fillet::beyond_end`
  (the faces beyond a run's end face shorter than the section that cut it off too, with the vertex
  each is reached at), `fillet::others_at` (the faces at a vertex besides a run's sides),
  `fillet::goes_on` (a face going on from a side, `tangent` to either side's normal) and
  `fillet::one_surface` (two faces of a body on one surface the same way out: `ssi::same` and
  their outward normals one) — what `end_at`, `dome` and `meeting` read, now public so a reader of
  the same boundary reads the same ends.  `meeting`'s sides compare outward normals, not the
  `reversed` flags: two faces of one plane whose frames point opposite ways and whose flags differ
  face the same way and are one side.


- **A face too short for its ball is told one way** (#994): `fillet::larger_than_face`, new, is
  the sentence the rolled ball's refusal (`holds_ball`) and a lowering's carried one say alike —
  the face too short, the face across the edge it ends at, how far that edge stands off the other
  face and where, how far off the ball touches it, and what to do; the rolled ball's refusal now
  ends "so no ball of that radius rolls against it there — it would roll on across that edge
  against …. Make … reach past the ball's touch (or remove it, where it is a sliver a cut left), or
  round the edge smaller than … reaches".  `fillet::reaches`, new: whether a curve stands as far
  as a height off a surface, read exactly at its ends and where it meets the surface's offsets
  (`query::curve_surface`), no sample of it.
- **Faces on one surface are one surface whatever order a Boolean's operands come in** (#994,
  fixture 115's feet and lip at a plate's true edge): an arrangement reads which faces lie on one
  surface face by face before any phase asks (`Arrangement::classed`) — two of one kind joined
  where `ssi::same_over` holds over their two boxes, a class every face so joined, standing for it
  a surface every face of it lies within the reach of — where it had joined each face onto the
  first surface given an index over the box the faces before it filled, so a pad leaning `1e-7`
  rad read first stood for the plate's top along its whole length and the plate's other face on
  that plane was a surface of its own.  New: `meeting::Shared::class` (two classes of one kind are
  never `one`).  Two curves on one surface cross where they come within its reach
  (`query::crossings_in` reads `curve_curve`'s closed forms at `ON_CURVE` tolerances, as it keeps a
  carrier's roots); a vertex on an edge at its end is that end, the two made one
  (`tolerant::onto_edge` and `tolerant::Onto`, new: the stretch and its ends read from one
  inverse), and the box a vertex is sought on an edge in reaches as far as that rule.  So a plate,
  a lip and a foot whose pads lie flush in its thickness or abut its edges unite in every order,
  where some orders were refused ("a plane's pieces do not close", "a piece …
  touches … along no face", "a shell … kept on one side … and dropped on another") and the rest
  built with a face more or less.

- **Faces crossing along the line their boxes share are two surfaces, and a meeting along a
  face's boundary is kept** (#994, fixture 115's strips past the reach): `Arrangement::classed`
  reads two faces over what their boxes share and as far past it as a point near both is read onto
  their meeting at the least sine (`tolerant::on_meeting` at `LEAST_SINE`), so two faces whose
  boxes share only the line they cross along are one surface only where they lean apart less than
  a meeting is resolved at — read over the line alone, a plate's top and a lip's pad turned `1e-3`
  rad across it were one class, and a third operand's meeting with it was laid `3e-4` off the pad.
  And `Arrangement::kept` keeps a span of a meeting lying along a face's boundary within
  `ON_CURVE` tolerances at every eighth of it, as the vertex-onto-edge rule reads it — unless that
  edge lies in the other face's surface all along and is laid there as it is — where it had
  dropped it as outside the face and left the other face a slit.  Strips turned `1e-5` to `1e-3`
  rad across a plate's faces, in its thickness or abutting its edges, now unite in every order to
  their closed form, where some orders were refused ("a shell … kept on one side … and dropped on
  another").

- **A boss on a lobe of a plate, of the lobe's radius or a hair off it, builds** (#994, fixture
  141 of #961): three readings of an arrangement agree with the reach two features are one at.
  The complex orders faces round an edge a step into each from the face's own point at the edge,
  not the edge's, which stands within the reach of its faces and not on them
  (`complex::Complex::new`); a piece narrower than any outline the readings draw — a crescent a few
  tolerances wide — is read inside all the same (merged with #996: by the one reading of a strip
  thinner than its outline, `reading::beside_halves`, below), not dropped as having no inside; and two circles in one plane are read touching only where they
  stand within the tolerance of one another (`query::curve_curve` by `plane2::circles`), not where
  the chord they share stands within it of the first.  Refused before as "a shell of the
  arrangement is kept on one side of a cylinder … two of its cells joined round an edge" and "a
  plane's pieces do not close" at radii a micron apart.

- **Two cylinders resting on one another at a point meet in no curve, in closed form** (#994 item
  5): `ssi::intersect` reads two cylinders whose axes lean apart and stand at least the sum of
  their radii apart, less the tolerance, as meeting in no curve (`ssi::rods_apart`) — clear of one
  another, or touching at one point — as a ball resting on a plane is read.  They were traced, and
  the search for a meeting no edge crosses found a point beside the touch that the trace could not
  follow ("a cylinder and a cylinder meet at …, where no edge crosses, in a curve that could not be
  traced", or "nearly touching"): a blanket round over a clevis's top.  Two crossing, or leaning
  less than a degree apart (where they stand within the tolerance along a stretch), are traced as
  before.

- **Two surfaces resting on one another at a point meet there alone, whatever their kinds** (#994
  item 5): a definite touch (the relative curvature `Q` definite) is read alone whatever the
  faces' size — merged with #996's item 3, by its one rule, `ssi::alone_over` over the touch's
  radius (below); the second-order reach `8 √(2 tol / λ)` this entry first read it by
  (`ssi::resting`) is gone.  `ssi::touch_near` is Newton on the difference of the two exact unit normals,
  settled by the height a step changes, and finds a touch whose `λ` is small.  An edge refused
  where a surface of its own rests on the other is read as touching it there, alone over the
  touch's radius (`boolean::resting_on`).  A ball standing its radius or more from a cylinder or a torus meets it
  in no curve, in closed form (`ssi::ball_clear`).  A cylinder, a torus, a ball resting on a
  cylinder, a torus or a cone at a point was refused ("a cylinder and a torus meet at …, where no
  edge crosses, in a curve that could not be traced", "nearly touching", "pieces do not close").
  No public API changes.

- **Two cylinders whose axes lean apart meet in closed-form loops** (#994, fixture 146): where the
  lesser goes right through the other, `ssi::intersect` gives the meeting as two closed traces
  whose points are the graph of the meeting over the lesser's angle (`ssi::crossed_cylinders`,
  `cylinder_loops`), sampled so no projection between two crosses a neck to the other loop — no
  march.  The two ellipses of one radius and crossing axes are kept within the tolerance, the
  axes' gap now read by the axes' nearest points.  A loop turning back, or a lean under `SHALLOW`,
  is traced as before.  Rods of radius 6 a micron apart crossing were refused ("a cylinder's
  pieces do not close").

- **A run ends on every face beyond a short end face that faces on past it, and a ball too large
  for its face is said by that face** (#994, fixture 147's vane): `fillet::end_at` follows each
  edge an end face makes with the run's faces to its other vertex — only where that edge is no
  longer than the section's size (a round's radius, a flat's longer leg), so the faces past the far
  end of an end face as long as the run's whole end (a hub's side an arm's round ends on) do not
  end it — and a face there that faces on past the end (`fillet::faces_on`, new and public: its
  outward normal turned more than a degree from square to the run, off its end) and that a closed
  form reads (`fillet::cuts_off`, new and public: a plane, a cylinder, a cone, a sphere or an
  affine image) cuts the run off too, kept on its material's side, and its edges are none of the
  ball's band — a vane's toe cut by a slope `1e-4` over the foot, its upright face shorter than
  the section, cut its side's rolled round off at that face alone and refused the slope's edge with
  the side as entering the band ("larger than `vo0` can hold").  One lying alongside the run (the
  slope along the toe's own foot) ends nothing.  `holds_ball`'s refusal now says the face the ball
  overflows, the face it ends at, how far that edge stands off the other face and how far off the
  ball touches it.

- **Whether a face goes on tangent from a run's side is one public test** (#994):
  `fillet::tangent(n, m)`, new and public — two unit normals one either way, `1 - |n·m|` within
  `1e-9` — is what `end_at` reads an end face going on by and what a standing ball's dome reads
  an end no face goes on from by, so a caller predicting either reads it the same way.

- **A standing ball's dome carried past an end a face cuts across** (#994): `fillet::roll`, where
  the ball stands still over a stretch of its loop and the stretch ends where a face cuts across
  it (a lid's chord, a hub's side across an arm's rounded end), no longer refuses ("no stretch of
  its dome is carried past and cut off by a face yet").  The dome goes on round past that end as
  far as the faces' meeting stays out of the faces it ends on: round the whole loop (the meeting
  in closed form, the whole loop's hemisphere) where both ends are cut across, half round from
  the other end's section where a face goes on tangent there — then cut off by every face at the
  cut end (a plane's half-space, a curved face's solid).  Refused by name where the meeting comes
  back within those faces elsewhere, where the run going on ends apart from the ball's section,
  or where the surfaces meet in no closed curve in closed form.  No public API changes.

- **A round beyond a seam that starts where its touch crosses the seam** (#994): `spill::Beyond`
  has a new field `seamed: [bool; 2]` — with `run`, an end where the ball's touch crosses a line
  seam lying aslant the edge, the piece starting at the section through it (square to the edge,
  the touch the vertex on the seam, no strip of the next face there) and not carried past it, the
  edge's own run glued to it there.  `round_beyond` builds it; a cap carried past an edge's end
  whose touch has come back short of the seam is refused rather than built inside out.
  `flat_beyond` refuses a seamed end.  **Breaking** for code building `Beyond` as a literal: set
  the field (`[false; 2]` for what was built before).

- **A definite touch is read alone by its own orders, not by the faces' size** (#996 item 3): two
  faces touching at a point, their meeting there the point alone to second order, were read so only
  where the touch's radius — where a trace cannot follow them — was under a twentieth of the
  smaller face; a rim's round on a short side crossing a wide blend's foot was refused ("a cylinder
  and a cylinder meet … where no edge crosses, in a curve that could not be traced").  Now such a
  touch is alone where `b`'s distance along `a` grows as its second order says over the whole
  radius (`ssi::alone_over`), and `meeting::meet`'s search passes the touches it found, which it
  refused before in the same words.

- **A turn joined to the legs tangent to it, exactly or all but** (#996 item 1): a quarter elbow
  whose section is clipped where it reaches the bend axis (a spindle torus, its arc up to 308°) and
  the cylinders standing on its ends build for every tube from a hundredth under the legs' radius to
  a hundredth over, the tangent one among them, each with the step it has. `ssi::disc_reach` reads
  a spindle's and a horn's torus away from its other sheet, so a near-contact on the lemon past the
  turn's arc is clear of its face; `ssi::intersect` meets a plane through a torus's axis in its two
  meridians in closed form (the turn's caps: 3.3 s of traced marches before); the unseen-meeting
  search's bounds over a strip are the torus's parallels', which vanish at its poles, and a meeting
  it finds off either face is not traced; `Brep::loop_area` and a cycle's area in a Boolean are
  Simpson's rule, doubled until settled (a crescent 308° long and a thousandth wide read the wrong
  way round); a piece thinner than its outlines is read inside from beside its curved halves, not
  dropped as a sliver while its sides stay two edges; and an inverse pcurve ending within its edge's
  tolerance of a pole takes the pole's collapsed parameter from its end, not from the far side of
  the axis (a union 2.6 times the solid it made). A use within its vertex's or edge's tolerance of
  a pole is at it wherever the pole is read (`Brep::close_poles`, a half's points in a Boolean), a
  pole's half runs along the parameter the pole collapses, and two uses of one edge at one place,
  out and back, are a spur and not a seam: an axle and a slab through the near-miss turn's poles
  build ("loop 0 breaks in parameters" at the pole before). **A tube all but tangent to another at
  a seam is on it**: `flush::tubes_flushed` (called by `boolean::combined_written`, as `flushed` is)
  moves a cylinder whose end circle stands concentric with and in the plane of the other's circle,
  its radius off by no more than `FLUSH` tolerances, onto that radius, exactly; the issue's
  document, cored and flanged as the fixture is, and every near-miss of its legs whatever is joined
  or cut after, build as the tangent turn.

- **One area rule and one pole rule** (#996 cleanup): `topo::green_area` is the Simpson-on-Green
  routine `Brep::loop_area` and a Boolean's cycles both read, its samples kept as the steps double;
  `topo::near_pole` the one reading of a point within a tolerance of a pole, at every site that
  asks. A piece's inside is read with its first bar and its finest outline drawn once, and a
  meeting's touches read alone (`ssi::alone_over`) only where the search passes them.

- **The bench's cube tumbling 75° is built** (#942 T1d-cube, `swept::general`, `family`, `topo`):
  a fan's crossing with itself is built from the pinch it is born at — the runs' corner where the
  sheet's rate across their rim turns over — as the pairs of points of the two runs that meet, each
  solved by Newton and marched out from the Whitney umbrella's own pairs, not traced toward the
  pinch, where the trace lost itself on the runs' rim. A wall's foot is found by a bounded
  Levenberg–Marquardt (`Sheet::newton`); on a run of a fan crossing itself (`Sheet::layered`) its
  last foot is trusted only where it settles on the point, else the nearest four of a grid are
  settled and the nearest kept (`Sheet::inverse`), any other wall's as before. An inverse curve
  on a face takes its feet's chord for its rate where the face is singular. New
  `nurbs::BSpline::with_knot`; a branch's end carried onto a rail or a pinch bends only its last
  span. The 15° cube's volume moves by 4e-8 with its crossing built.

- **The cube tumbling 15° is built** (#942 T1d-cube, `swept::general`): a fan's runs are
  found before the bearers are met, and two fans' branches beside a shared rail are traced on each
  run's own wall (traced on the whole wall a fan crossing itself folds over, they ran 1e-5 off both
  walls); any other curve given of a whole wall is cut to what lies on each run's wall, not given
  whole. The bench's 75° cube now gets through its trim, refused by the solid's check (a loop
  breaking in its parameters by 6.8e-7 beside a run's end); the 10° and 12° cubes are refused in
  their trim, with no fan crossing itself.

- **A branch beside a rail ends where the two fans touch** (#942 T1d-cube,
  `swept::general`): two fans' second branch beside the corner's path they share reaches the path
  only where the two are tangent, and its trace stopped short there or ran on along the path (the
  cube tumbling 75°: 3.95e-6 past the corner, at a sine of 7e-7), the branch left hanging in a face
  as a slit. An end within its meeting's reach of the path is carried to where the walls' normals
  turn past each other along it, and to the road's corner there, the curve cut at it. New
  `nurbs::BSpline::segment`, a curve's stretch exactly. The 75° cube is now refused where the 15°
  one is, the boundary open along a rail edge.

- **A fan that passes back through itself is cut into runs, their crossing given** (#942 T1d-cube,
  `swept::general`): an edge's fan was one wall over the whole roll, its strips faces of one
  surface, and where the edge's line passed back through where it stood a while before two faces of
  one surface crossed — refused by the trim's witness as a meeting of a surface with itself (the
  bench's cube tumbling 75°, its edge 5's fan). Each two of a bearer's stretches are met as two
  surfaces, its wall restricted to each (`family::Sheet::restricted`); a fan crossing itself is cut
  into the fewest runs of stretches that part every two crossing ones, each run its wall restricted
  to it and a candidate of its own, given the rim it shares with the next, where the runs cross
  (traced run against run, one curve across a run's stretches), and each meeting of the whole wall
  restricted to the run (a rail over the run's span). A fan crossing itself nowhere is as it was.
  The cube tumbling 15° and 75° get past it. A seed on a branch traced already
  is read within `tolerant::on_meeting`'s reach (two fans meeting shallowly traced one branch twice,
  4e-7 apart); a dump writes each wall with the number it is shared by, so a trim read back shares
  its walls as built and replays bit for bit. Gated by `swept_general`'s replays of the 15° and 75°
  cubes, refused where they are now and replayed alike.

## [0.20.0] - 2026-11-21

- **A drafted corner where an arc meets a side at an angle is built** (#962 item 8b): `build::offset`
  knew only corners that move along a line as the offset grows, so a draft of an outline where an
  arc meets a line or another arc at a corner was refused ("a curve where a cone meets a plane, not
  built yet").  Such a corner's edge is always a conic — a parabola where a line meets a circle, an
  ellipse, parabola or hyperbola where two circles do — and is built exactly as a quadratic
  (rational) Bézier, each arc's side its cone, reflex corners mitred.  New: `build::drafted_away`,
  the section's own rule for a side the draft takes to nothing (with the height it goes at), and
  `build::corners_move_straight`.

- **Rounds against a spherical crown** (#962 item 7): an edge lying in its own face's surface meets
  the other operand's surface only on that surface's meeting with its face, so where the search is
  refused and that meeting has a closed form the edge is read against it, tangencies included (a cap
  on a torus touching it round a circle, "was not proved").  `fillet`'s `holds_ball` reads the
  ball's band as the strip its section sweeps, each station's reach measured to the crease, not by
  height alone (a 2 mm ball "larger than" an R800 crown).  A sphere face whose boundary runs through
  the chart's pole takes the collapsed parameter from the half-edge ("a sphere has a hole in no
  piece").

- **An error building a face says the face once** (#962 diagnostics): `build::Seg::length` is a
  length whatever the curve's parameter measures (an arc's angle was compared to a length
  tolerance), and `build::no_length` is the one rule for an edge of none; the kernel's check names
  the edge.

- **A round as tight as its corner, and a boss rounded onto a filled edge** (#962 items 1 and 5):
  `fillet::roll_beside` builds the part of a standing ball's piece past the end of its run, a run
  going on from there holding it; `recipe::Rolled` gains `beside`, which asks for it.  **Breaking**
  for code building `Rolled` as a literal.

- **A rim's pieces glued at a seam the body reads as tangent** (#962 item 2): a drawing tangent to
  only some 1e-7 rad left the body seeing no end and the assembly no shared section ("two pieces
  joined at a section share no face there").  `recipe::Lowered` gains `gaps`, one per join, how
  far apart the two caps a seam glues stand; the chain is glued at that gap and its edge tolerances
  measured.  `boolean::glued` takes two faces as one surface where `ssi::same_over` reads them so.
  New: `tolerant::least_sine_apart`, the least turn at which two meetings are told apart.
  **Breaking** for code building `Lowered` as a literal.

- **An edge is a stretch two faces touch along only where it lies in both**: a pair of faces' stretches
  were every edge of one operand lying in the other's face, whatever face of its own the edge
  bounds; now an edge is one only where it also lies in this face's surface.  A rim rounded past a
  reflex corner 1e-3 rad off tangent builds again (a regression the next entry exposed).

- **A near-contact off either face no longer refuses a Boolean** (#961 item 2): `ssi::unseen_beside`
  takes a `clear(p, r)` callback, and a leaf cell clear of either face is neither traced nor
  refused (a countersink beside a counterbore, refused where the cone's other nappe passed the
  wall's cylinder below the floor).  New: `query::Located::face_clear`, and `ssi::disc_reach`, how far
  along a surface the points within `r` of `p` can be.  `ssi::unseen` and `meeting::meet` behave as
  before.

- **Faces on one surface but for rounding are one surface in every phase** (#961 item 3):
  `ssi::same_over(a, b, tol, region)` holds two planes, cylinders, cones or tori one where every
  point of either inside `region` lies within `tol` of the other; `meeting::Shared::index_over` and
  `Shared::one` index each face's surface by it over the overlap of the faces' boxes, within
  `ON_CURVE`·tol, and every phase of the arrangement asks that one index.  Pads drawn flush from
  coordinates rounded to five decimals build.  `ssi::same` is unchanged; new: `meeting::common`.

- **A round or a flat beside a seam along its edge into a cone** (#962 item 6): `spill::Beyond` gains
  `run`, the planes a piece carried the whole length of the edge past the seam is kept behind, and
  `spill::round_beyond` returns those planes with the piece.  **Breaking.**

- **STEP: fitted curves held to a bar, the stated tolerance's where one is stated** (#961 item 4):
  every traced edge and cubic pcurve was refined to a fixed 1e-7 at every marched point, whatever
  tolerance was given (a tube with one cross-hole, 3.3 MB at any tolerance).  One Hermite fit marches
  pieces sized by their miss, over a traced curve's length rather than its point count, each held to
  the bar on both faces it trims.  `step::write` and `step::write_assembly` take a `step::Fit`:
  `Fit::Exact`, every fitted curve within `step::EXACT` = 1e-9 mm, or `Fit::Within(b)`.  The tube is
  997 KB by default and 55 KB at `--tolerance 0.01mm`.  **Breaking.**

- **A span of a traced meeting lying along edges a face already has is those edges** (#962 item 3):
  the arrangement took a traced meeting for an edge one of its faces has only where the whole curve
  was one `Traced` lying along one edge (`along_an_edge`), and a traced meeting is fitted, and runs
  on past the faces; now each span it lays is checked against the faces' edges, any number of them
  (`span_along_edges`), and laid once. A ring kept within a cylinder its round crosses 0.001 deep
  has its meeting with that cylinder cut at the ring's top circle, which it passes 5e-7 under; the
  union beside it laid the meeting again a hair from those two edges, tangent all along, and the
  walk round the cylinder's face — reading ties nearer the node since #942 F6 — ordered the two
  wrongly ("a cylinder's pieces do not close").

- **The roads under any motion read no threshold of their own** (#942 F6): every local number the
  T1d work added to the ring road, the general road, the trim, `closed`, `contacts` and the Boolean
  is now the tolerance's, a feature size's, the rounding's (`util::fmath::HALF_READ = √ε`, `G` carried
  within `ε^(1/(order+2))` of a rim), the normal form's (a narrow pair's crossing within `√3 − 1` of
  its bridge's half-width), the trace's step, or the structure's (a side of a face's square read by
  its own way, a halving to the roll's rounding), or is gone. New shared rules: `closed::grain` (the
  roll over which no corner of the tool moves the tolerance), `closed::road_trim` (the hair a trim reads
  at, #880 Q1b's), and
  `rings::on_double_curve`. `rings::cut_along` takes the tolerance. A chord whose ends are snapped
  to one place has no length and is dropped; a double curve is stepped adaptively, by its own
  chord's sag. Brep-bench's swept category run before and after: the 68 cases that built still
  build, each within 1e-10 in volume and area but `tumbling-torus` (3e-9, beside the 3.3e-5 it stands
  from the volume made apart), the 23 refused still refused. The full turn's tumbling torus reaches
  its refusal in 34 s where it took 30: its first swallowtail point now stands where its double
  curve settles it, at roll 1.47717 — settled to `tol / r` it stands there to the digit — where the
  fixed step of `2e-4` put it 5e-4 of the roll past (1.47767). Its stretches either side are fitted
  over their true spans, entered in 43 faces where they were 38, and its trim reads the more.

- **Faces tangent along an edge are ordered round it nearest first** (#942 F6): the complex reads
  the faces tangent along an edge a step into each, and that step was a hundredth of their least
  exact radius or a thousandth of the complex, whichever is less — a thousandth of the box where a
  face has no exact radius. It is now read sixty-four tolerances in first, and ten times farther
  only while two still agree within a tolerance's lean there (`4 tol / step`), to that step at
  most. The drum screwed a short lead through most of a turn has rims' ramps thinner than a
  hundredth folding against the cylinders its side sweeps; a thousandth of its box read past them,
  so its solid depended on the box's size — built right in a box with a twentieth of its extent to
  spare, judged wrong with a tenth, at the tool's size or at the extent, and cut from a slab a
  hundred times its sweep it left the slab almost whole (99993.7 where 99897.9 is the slab less
  the sweep). It is now right with a twentieth, a tenth and half the extent to spare, and from the
  slab. Gated by
  `swept_closed::a_drum_screwed_a_short_lead_cut_from_a_slab_a_hundred_times_its_sweep_takes_its_sweep`
  (1.5 s), the slab left whole without the rule. The screw domain's room, a twentieth (#806), stands:
  it now decides how much a sweep reads, not whether its solid is right.

- **A double curve crossing its tube's seam bounds a tail beside it, not the whole tube** (#942
  F6): a ring's tube entered beside a fold pair drops what lies above its double curve's preimage,
  the swallowtail's tail; that preimage, read in the tube's `[0, pieces)`, is now unwrapped along it,
  as it already was where it is cut at the seams between faces. One crossing the seam ran from one
  end of the tube to the other, and all but a sliver of the thin tumbling torus's middle stretch
  was dropped: its tube was sewn into two candidates where it is one, and its trim searched three
  more pairs (85 s to 117 s, the same refusal). Gated by
  `swept_rings::a_double_curve_crossing_its_tubes_seam_bounds_a_tail_beside_it_not_the_whole_tube`
  (slow tier), refused without the rule. `closed::ring_entered` gives what the ring road enters
  before it trims (`RingEntered`).

- **A trace of no length is no chord** (#942 F6): a ring's tube entered at an end pose leaves out
  a trace of its face's meeting with the tool whose points all stand at one place on the face's
  square — the tool touching the face there, not crossing it; a trace along a side, a rim lying on
  the tool, is kept. The near-orthogonal tumbling tori each had one,
  two points 2.4e-13 apart where a crossing of the face ends on the rim, and were refused ("two of a
  ring's sheet's crossings of the tool end at one place on a face's rim"); their tubes are now
  entered, and they are refused later, where the end pose's torus face is assembled.

- **A sampled stretch held nearer than its tolerance where its fit closes in** (`cubic::fit_held`,
  `cubic::HELD_SPANS`): the cubic doubled until it stands within `held`, the nearest fit met within
  `tol` standing where a doubling no longer brings it four times nearer or its spans pass
  `HELD_SPANS`.  `cubic::fit_sampled` is the same fit held to `tol` itself, as it was.
- **The recipe is typed values, never JSON** (`recipe::Recipe`, `recipe::Node`): a writer builds
  the nodes itself and the kernel builds them as they are, with no text written and read back.
  `recipe::build`, `recipe::node` and `recipe::node_named` take a `&Recipe` or a `&Node`;
  `Recipe::read`, `Node::read`, `Section::read`, `Sectioned::read` and `Profile::from_json` are
  gone (`Sectioned::of`, `Profile::of`); `Profile::to_json` is `Profile::section`; `Writer`'s
  methods return numbers, not JSON, and `Writer::at` and `Writer::triple` are gone (`moved`);
  `cubic::bspline_edge` returns a `recipe::Stroke` and drops the fit it never wrote anywhere read,
  and `cubic::spline_edge` is the same cubic as a `build::ProfileEdge`; `swept::region::profiles`
  returns `build::Profile`s, built directly.  `Recipe` and `Body` are `Default`.  `Kind::Recipe` is
  gone: `build::sheet` checks its net itself (`Failed`).  **Breaking.**

## [0.19.0] - 2026-11-20

- **A half-space bounded by the material it is kept with** (`recipe::Node::Half`, #953): the block
  behind a plane standing over the box of the nodes it is kept in common with, reaching past that
  box so only the plane's face meets them. **Breaking** for code matching `Node` exhaustively.
- **Sheets and faces sewn into the solid they enclose** (`recipe::Node::Sewn`, `SewnPiece`, #953):
  each piece a profile's planar face or a B-spline net's sheet, named, sewn to a tolerance
  (`sew::sewn`) and refused where they do not close. **Breaking** for code matching `Node`
  exhaustively.
- **The ten moments of a solid, and of a face** (`props::moments`, `props::face_moments`, #953):
  `∫ 1`, the three first and the six second moments of `(X − c) / h`, a solid's by the divergence
  theorem and a face's by area, each read off the faces' loops as the volume and the area are, so
  nothing is meshed. Only the moments asked for are read.
- **The nearest point on a long B-spline reads its spans through an index**
  (`BSpline::nearest_indexed`, `BSpline::gap_indexed`): built once for a curve and kept, it reads
  the spans `nearest` would and finds the same foot. `Located::edge_gap` is bounded by a long
  spline's spans' boxes, so a point beside a long edge skips measuring it, and a polynomial net's
  inverse tries its last foot first, accepted by the bound every other candidate is.

## [0.18.0] - 2026-11-19

- **A ring's tube cut at an end pose carries the places it was traced through as the cut's pcurve**
  (#942 F3): `rings::chord_curve` returns, beside the curve, the places on the face it passes — the
  traced points and the meeting's own point settled between each two — at the curve's own
  parameters, and the cut's pcurve is the coarsest cubic through every so many of them whose image
  holds the curve's bar (a quarter of the tolerance) at each and at 64 places along it, where it was
  the curve inverted onto the face from the line between its ends. Beside a fold pair's birth the
  leaf folds back on itself within 2e-4, and the half turn's cut, so inverted, landed on the other
  fold ("edge 28's pcurve is 2.15e-4 off it"); its build now reaches the pair's untraced double curve.

- **A ring's stretches are sewn along a cut as one row, the cut's** (#942 F3): `sew::sewn_open`'s
  `across` now names which row a face's parameters lie on (`Option<usize>`, the cut's index) where
  it said only whether they lay on one, and two rows are one where they are the same cut's. The
  stretches either side of a cut are fitted a billionth of the roll clear of it; read by their
  parameters' values (equal within 1e-9 of the value) the half turn's two rows, 6.3e-9 apart, were
  two, nothing was sewn across its cut, and a stretch's row lying along the next stretch's sheet was
  searched for where it met it and not proved. Gated with no sweep by
  `brep::two_stretches_of_a_sheet_fitted_either_side_of_a_cut_are_sewn_along_it_as_one_row`.

- **A given meeting's curve that is an edge a face already has is that edge, laid once** (#942 F3):
  a ring's tube, cut along its crossing of the tool where it starts and given that chord as their
  meeting, had it laid a second time on itself, and the ring along a spline's tube face was refused
  ("a bspline's pieces do not close"). A given curve equal to either face's edge curve is now that
  edge, as a traced meeting along an edge already was, its crossings with the meeting's other curves
  read first and kept on them. The ring along a spline's arrangement now assembles; the build is
  refused later, where its cells' membership does not settle. Gated by each of its tubes arranged
  with the torus where it ends, its chord a later span of a curve of several
  (`swept_rings::a_tubes_chord_given_as_its_meeting_with_the_tool_is_its_own_edge_laid_once`,
  refused without the rule).

- **Two curves on a face crossing where one ends on a circle the face touches a plane along**
  (#942 F3): `query::crossings_in` reads one curve against a surface carrying the other. Where a
  circle's plane is refused, the circle's cylinder is read instead. A bull-nose's round touches its
  flat end's plane all along the circle they share, so a curve on the round reaching that circle
  touches the plane there: a double root no search proves, where it crosses the circle's cylinder
  square. The plane stays first, since a helix about the circle's axis lies on its cylinder. Brep-bench's bull-nose swung partway into the ring sector was
  refused ("where a traced … meets a plane was not proved") and is now cut to its settled volume.

- **A meeting given by construction holds its points of touching** (#942 F3): `boolean::Given` gains
  `touches`, points where its two surfaces touch beside its curves. An edge of either through one is
  read as touching the other there, with no search: a touch is a double root, which no search proves
  by a change of sign. The slide road gives them where a band's contact crosses a seam: the
  neighbour's face at each end pose, and every face of its candidate's solid, touches the band's
  sheet only at the seam points. Brep-bench's bull-nose pushed in at a lean, its side's rim touching
  its round's sheet mid-span, was refused ("a circle … meets a extrusion was not proved") and is now
  cut to its closed form. `Given` literals need the new field (`touches: Vec::new()` for none).
  Gated with no sweep (`arrangement::a_rim_touching_a_wall_at_points_given_is_read_there_with_no_search`):
  a drum's rim touching a parabolic channel's two extruded walls at a point each is refused with
  nothing given, and read there, its volumes to the rounding, given the walls' meetings and the points.

## [0.17.0] - 2026-11-19

- **One fallback for shallow meetings, marched in both surfaces' parameters** (#942 F1): where the
  trace is refused as shallow, `meeting::marched` marches each seed no branch holds along the
  solutions of `A(u_a, v_a) = B(u_b, v_b)` — the general road's four-parameter march, moved into the
  kernel as `meeting::parametric`, bounded by both surfaces' boxes of parameters and closed where it
  returns to its start — where it followed level curves `f = 0` in one surface's parameters. Those
  strayed 1e-4 from the 75° tumbling cube's fans at their 0.02° pinch, followed a wall's extension
  past its patch, and closed on themselves about each seed near a ball's pole; across the suite they
  read no branch the march does not, and are gone (`meeting::level_curves`). A plane crossing a
  cylinder and a ball of radius 100 at 0.02°, the closed forms bypassed, is met in the closed form's
  two lines and its circle to 1e-9, where the general method gave three lines and eight pieces of the
  circle 4.5e-4 off. The general road's branches beside its rails and the trim's completion now go
  through it too (`meeting::through`), where they called the trace and a march of their own.

- **A trim missing a meeting completes it from where it parts a piece** (#942 F2): a piece the trim
  reads is held to one verdict at up to eight of its points, and two that differ, the one off the
  boundary proved clear of it, are a witness, where a piece was kept or dropped whole by its first
  point. Where the result does not close, the faces either side of each open edge are read the same
  way. From a witness the trim walks the face between its two points, traces the meeting of each
  surface crossing it there that it was not given, adds it to what it was given and trims again (the
  tumbling cube at 5°, its branches beside its rails taken out, builds the same solid). What it cannot
  complete is refused by name: a pair met by search that missed the crossing, or a surface crossing
  itself (the bench's tumbling cube at 75°, its edge 5's fan passing back through itself). Faces
  whose surfaces were given a meeting are now always met in the trim where their boxes meet, and
  `trim::Report::completed` counts the curves completed.

- **A ring's double curves are cut into its stretches by the fit itself** (#942 F4): each double
  curve point's place round the loop is read from the seams' angles of the stretch's own net at its
  row, where it read a whole instant's contact (a root on every meridian and its joins settled) to
  find the two joins of its piece. That was 70% of the tumbling torus turned whole: 19 314 instants,
  74 s of 106, now 0.14 s. The turned torus reaches its refusal in 32 s, and the thin one in 58 s,
  where it took over forty minutes. Both are still refused, in the trim's Boolean.
- The ring road says where its time goes where `SOLVENT_BREP_TIME` is set: each step of a ring's
  build (the shape changes found, each stretch fitted and its narrow pairs' crossings sought, the
  double curves traced and cut, the stretches entered and sewn) with how many instants it read, their
  time on the cores and the roots read alone; the trim's Boolean beside its pieces' membership; and
  the arrangement's edges read against faces, all told and by kinds of curve and surface. The
  `ring_profile` example builds brep-bench's two tumbling tori so.

- A sweep refused in its trim is written whole where `SOLVENT_SWEEP_DUMP` names a directory
  (`swept::dump::dump_trim`, #942 F5): the tool, the trajectory by its structure (a twist, one seen
  from another, a twist placed at a pose), the roll, the bar, the trim's reading, the candidates and
  the meetings given them. A face family's wall (`Surface::Family`) and the rails and rims along it
  are now written and read back bit for bit. `load_trim` gives a `Trimming` whose `replay()` runs the
  trim alone, so the general road's and a ring's refusals are questioned without remaking their
  candidates: where the sweep's boundary crosses a line and on which candidate's face
  (`Trimming::boundary_along`), and what passes near a point (`Trimming::through`), put from the
  command line by the `sweep_probe` example.

- **A face swept along its normal whatever bounds it, and a tube about any smooth edge** (`offset`):
  an offset refused a curved face its own parameter lines do not bound, and a crease along any
  curve but a line or a circle. A plane's, a cylinder's or a sphere's move along its normal is one
  affine map of space, linear in the distance (a step, a stretch square to the axis, a scaling about
  the centre), so the face's sweep is the face carried between its two moved copies on the offset
  surface, its pcurves its own, walled by the ruled sheet each edge sweeps between its two images
  (`Blend`, or a plane where they lie in one) — exactly, where a profile's prism or a band of
  revolution does not take the face. A crease along any other curve is the ball carried along it
  (`swept::canal`, the one tube a ball sweeps along a path), flat-ended; along a closed edge a ring
  with no end (`swept::canal::ring`, `family::ring`: the frame's holonomy unwound along the roll,
  the wall's `s` periodic), so each face's moved copy lies along it whole; refused, with the radius
  and a point, where the curve turns tighter than the distance. brep-bench's
  offset/cylinder-cut-at-thirty-degrees-grown and the slant-cut cylinder grown by ½, 1, 1½ and 2
  build to Steiner's formula within 1e-11; grown past the ellipse's curvature they are refused.

## [0.16.0] - 2026-11-18

- **The contact's road under any motion, begun** (`swept::general`, #880 T1d): a polyhedral tool's
  candidates under a motion no constant twist takes — its contact read per tool at each instant,
  chords of its faces' characteristic lines and stretches of its sharp edges, cut where its shape
  changes; each piece on a moving wall (`family::Sheet`) per edge or face, its strips the faces of
  one candidate; and the meetings known by construction given. No build takes it yet: a stretch born
  at a point of its edge pinches the edge's carried line, and two edges may sweep one surface.
- **A ring's fold pair narrower than the band it is read in is seen** (`swept::rings`): its least
  `s''` sought along the band on the curve's own branch, where samples alone read the whole curve as
  touching (the tumbling torus turned whole, and thin). A sheet is refused as folding where it turns
  over, not where it is merely thin.
- **A ring's meridian the instant touches whole is read through** (`swept::rings`, #880 T1d M2): on
  an arc of the meridian `n · v = A cos ψ + B sin ψ`, and where `A` and `B` both vanish — the ring's
  top and bottom carried along a path in the plane square to them, their velocity along the path —
  the arc's root was refused (no root there, so the contact's count changed round the axis:
  `Curved`). Beside such a meridian `A` and `B` are read in closed form, each `2 sin((θ − θ*)/2)`
  times a vector that is not small, so the two curves pass through it smoothly; and a piece ending
  on it is read a hair inside, on its own side, where the end's root stands apart from it. brep-bench's ring carried along a spline is admitted,
  its contact one shape over the roll, and its tubes fit. Beside the whole meridian each curve has a
  narrow fold pair, one fold on the meridian itself, where its branch's label turns over: a join
  inside a bridge is the bridge's (a fold piece on its own label), so the bridge holds the pair whole;
  and a trace of a face's crossing of the tool walked from both its ends is one chord. A trace
  leaving a face beside its corner ends at a crossing of the rim nearer than the corner; a crossing
  both faces beside its seam trace straight back into one corner is a cap about the vertex and
  dropped; a seam's place is matched within the snap its two faces' readings were merged at; and a
  meeting's chord takes the
  meeting's own point halfway between two of its points wherever its cubic strays from it, settled
  on the tool itself within `G`'s carried reach of the rim. Its sheets now close and are arranged;
  it is refused further on, the circle's band swept twice over beside its whole meridians, two faces
  coinciding along an edge by the start pose.
- The general road gives two edges' fans sharing a vertex the second branch of their meeting
  beside its path, traced from where an edge of either crosses the other (`general::beside_rails`):
  given the path alone, the arrangement searched the pair for nothing more, and the cube tumbling
  five degrees was left a hole between its top front edge's fan and its top side edge's. It builds.
  Where the march in space is refused — two fans meeting at 0.02° where the branch reaches the pinch
  a third's rail starts at, the cube at 75° — the branch is marched in the two sheets' own
  parameters (`general::marched`: the 3 × 4 Jacobian's null vector, minimum-norm Newton), landed on
  the patch's edge, and drawn as a cubic, or as a polyline where it turns within a few points. At
  75° the cube is still refused, a second branch between two other fans left in pieces.
- A ring's tubes sewn across the cuts of their roll (`sew::sewn_open`) merge a vertex only with
  vertices of other stretches on its own row, and cut a row's rim only at that row's vertices:
  beside a swallowtail's point, where a stretch a two-hundredth of a radian wide is cut, the two
  rows stand within the tolerance of one another, and the tumbling torus turned whole had a face's
  rim sewn to its own other row, the face left a loop walking one edge both ways. It is refused
  further on, an edge of one sheet lying within a few tolerances of another over a long stretch.
- **A rod threaded at a lead that grows is built** (`swept::contacts::pose_meeting`, #880 T1d): a
  branch of a sheet's meeting with the tool at an end pose has each end put at its own nearest corner
  that `h`'s rounding reaches, and only one within two cells of it — a strip's corner at a vertex,
  `h` whole there, extrapolated to a root half the square away and took the branch's other end, so
  both ends were one point and the branch, fitted to nothing, was dropped (the bottom edge's strip of
  the bar, where it leaves the bar posed at the start). brep-bench's
  `swept/square-thread-at-a-growing-lead` builds by the general road, to the volume and area OCCT
  and cadrs agree on at 7.5e-11; a cube tumbling two degrees builds.
- **An outline stops halving where halving no longer helps** (`query::Outline::of`): past eight
  halvings a chord is halved again only while a halving at least halves what it misses its run by —
  a run read off a sheet all but folded flat beside a swallowtail's point, its inversion settling
  nowhere, was halved twenty times over, a million points (#880 T1d, the tumbling torus turned
  whole). Each point of a run is inverted beside the middle of the chord it cuts, so it stays on its
  own leaf of a surface that folds, and the Boolean keeps the points it inverted onto a face within
  one arrangement. A face walked into one cycle is that cycle's piece whatever its area reads (no
  face is a hole alone): a tube's corner beside a swallowtail's point read its one cycle's area off
  by its sign, `a bspline has a hole in no piece`. The tumbling torus turned to 2.0 rad now reaches
  its trim's check in 25 s, where it ran past half an hour.
- **`closed` asks the general road where K2 cannot read the tool** (`swept::closed`,
  `general_built`, #880 T1d): a polyhedral tool under a motion no constant twist takes and keeping
  no plane, refused by K2 as `Curved` (no convex solid of revolution), has its candidates made by
  `swept::general` and trimmed by membership and judged; a tool the general road cannot read either
  keeps K2's refusal.
- **A face's spurs that keep its walk from closing bound nothing** (`boolean`'s pieces of a face):
  where the walk into pieces does not close, it is read again with each edge walked both ways
  between the same two nodes, to a node nothing else reaches, dropped (and the edge it hung from
  reread, until none is left) — a strip's two sides leaving its point of birth tangent within the
  tolerance, the start of one merged onto the other a little way along it (#880 T1d, the tumbling
  cube's fan at a pinch point of its edge's carried line), where it refused (`a family's pieces do
  not close`). A walk that closes keeps its spurs: a slit, a line two faces touch along. `Brep::check` narrates, under `SOLVENT_BREP_DEBUG`, the faces using an edge
  it finds open and the edges passing near it.
- **A bearer of the general road standing on one cylinder is that cylinder** (`swept::general`,
  #880 T1d): its lines all parallel and their feet on one circle, within a few times what the sheet is
  fitted to, its faces put on `Surface::Cylinder` with their edges inverted onto it. A bar's two
  edges at one radius under a screw whose lead grows are one surface to the trim. The thread is
  still refused, now where a bottom edge's strip is not cut beside its vertex.
- **A box along a spline frame is swept closed** (`swept::region`, #880 T1d): a side's contact a
  hair from its end is its corner's; a corner moving along one of its sides reads that side by its
  rate along it, its normals the sides' own; and every edge is held where the turn changes sense
  within the roll (`turning_back`), as at an end pose. `path.sv`'s box, once `Folded`, builds.
- **A ring's roll is cut where its contact changes shape** (`swept::rings::changes`,
  `fitted_over`, `entered_at`): each stretch fitted on its own, the tubes sewn across the cuts
  (`sew::sewn_open`, `sew::parts`) and trimmed by membership. A newborn pair's bridge is the sheet
  itself, and its double curve — where its outer leaves cross — is traced over the whole roll
  (`rings::double_curve`) and cut into each stretch, its tail dropped (`rings::cut_along`). Where the
  double curve ends on the tube's meeting with the tool at an end pose such a build is still refused:
  never returned. The roll is cut again at each swallowtail point, where the double curve's two
  halves meet; the tubes are sewn only along the rows at the cuts (`sewn_open`'s `across`), the
  nearest two edges paired first where an instant's contact doubles back on itself; and a fit's fold
  check reads the loop's last join as the first piece's. A double curve's seed is read with its
  first preimage wrapped round the loop (before the first face it is on the last, not the bridge),
  sought within two of the bridge's half-widths past each end; and a pair born and later vanishing
  has its double curve closed through its second swallowtail point, the roll cut at both. Every
  stretch is cut where its tube crosses the tool at either end pose (`entered_at`'s `rimmed`: a
  stretch whose rim is no end of the roll read plainly), where a double curve can end partway along.
- **`swept::contacts`'s meeting of a sheet with the tool at an end of the roll reads any sheet**
  (`Rolled`): a constant twist's blend as before, or a moving wall.
- **A contact's fit given up on keeps to its nodes** (`swept::contacts`): a branch whose fit stops at
  its most nodes with spans still wide is kept only where its poles stand within its nodes' box grown
  by its size; past it, it swung hundreds of kilometres out between nodes crowding at one spot.

- **A swallowtail narrower than a stretch is cut at its double point** (`swept::contacts`, #880
  T1e): where a contact's image in the motion's invariants turns back twice, its outer leaves cross
  over one path, and a stretch holding both folds held both leaves of the crossing, a sheet crossing
  itself the trim never parts. Each double point is found exactly, by Newton's method on its four
  equations, and the piece is cut at both its points: the outer leaves meet as stretches, the tail
  between is buried. A ring pushed in at 56°, just past its fold, refused with a hole in no piece,
  builds cut and alone within ten tolerances; at 60° it stood 1.6e-4 off and holds to 2e-6. Every
  whole lean from 55° to 85° builds. Under a screw only crossings within one turn are double points;
  two whose spans nest are not cut.
- **A tool of revolution has its seams turned off its contact's folds** (`swept::contacts`): turned
  about its own axis, the same solid, where a seam stands within a tenth of a radian of a fold of its
  contact, so no seam runs through the cusp's meetings on the tool posed at an end of the roll. The
  twist drill's flute wheel too steep for its helix builds alone, with no blank, and cut from the
  stock by its whole sweep, no longer within the stock's reach.
- **A branch of a sheet's meeting with a posed face ends where the face does** (`swept::contacts`):
  a branch meeting a face the section does not lie on is clipped to that face, its ends put on a
  corner of the sheet within the root of the tolerance; read whole it ran on past the face's rim.
- **A crossing of a swept sheet, a fitted net or a family's wall is placed on it** (`boolean`): an
  edge crossing such a surface at a shallow angle had its root off the surface by what the angle
  makes of the tolerance, read outside the face and dropped. Newton's method on the curve less the
  surface settles it onto the crossing. The ring pushed in at 80° builds; a branch crossing the
  torus's seam had left a hole in no piece.
- **Two splines of one meeting are crossed** (`boolean`, `query::enclosed_crossings`): where two
  curves given or found for one pair of surfaces have no closed form between them, their crossings
  are sought by their enclosures, each held back from an end lying on the other, and kept only where
  they part (a thousand tolerances along either way, more than eight apart). A meeting
  `enclosed_crossings` settles outside the stretches asked about no longer refuses the pair as
  shallow.
- **A frame carried along a curve** (`trajectory::Along`, K2): the rotation-minimising frame along a
  B-spline from its start (double reflection on a 4096-step grid; along a curve in a plane the
  turn about its normal by the tangent's angle, exactly but for rounding), with its pose, rate and
  stretches enclosed by the curve's hodographs, so a sweep's speed bound reads it — a rational
  curve's stretch fails closed. A caller sweeping along a curve writes no `Path` of its own.
- **`Along` is generic over its curve** (`trajectory::Guide`: a domain, a jet, derivative
  bounds), from a home anywhere on it (`Along::from`), the rotation-minimising frame or the curve's
  own (Frenet) one; geomsolver's frames along a path are this type over its own paths. The
  sampled enclosure a stretch is read from is `trajectory::Extremes`; `BSpline::hodograph` is the
  curve's derivative as a B-spline.
- **Bounds on a polynomial B-spline's first two derivatives** (`nurbs::derivative_bounds`,
  `BSpline::derivative_bounds`): its hodographs' longest poles, what `Along` encloses by.
- **A prism under a motion that keeps a plane is cut from a blank** (`swept::closed::cut_from`):
  the planar road `closed` takes before K2's (#880 Q2, `swept::planar_solid`) is asked with a
  blank too, its swept solid cut from it by the Boolean. brep-bench's
  `swept/key-carried-along-a-spline`, refused at the key's corner (`Curved`), is cut in about a
  second, to the volume and area OCCT and cadrs agree on within 3e-12.
- **The swept region's keeping test halves to a length, not a roll** (`swept::region`): a stretch
  of the roll is read at both ends alone once a point moves no farther over it than over
  `KEEP_STEP` of a roll turning the tool at its reach; a roll that is no angle — a curve's
  parameter, the tool running 30 along it a unit — let a corner's sliver covered for 5e-4 of the
  roll stand as boundary, and the region did not close.

## [0.15.0] - 2026-11-17

- **Any tool of revolution's path of pieces is cut from a blank piece by piece** (`swept::pieces`,
  the road `swept::closed::cut_from` takes after `raster`): each move's sweep in closed form where
  it has one — slid along the tool's axis, slid square to it, turned about a line along it, a
  ball-end mill at a slant — and the closed class's of that piece alone where not; each move after
  the first shut where it starts by its section, the move before holding the tool there, so no two
  moves' sweeps lie on one surface at their corner. brep-bench's `cnc/flat-end-mill-profile`,
  refused after two minutes, is cut in under two seconds, to its closed form within 3e-15.
- **`swept::raster` is the pieces' road's sweep of a ball-end mill's run at a slant**: the
  separate road `cut_from` took first is gone, and every path of pieces is cut by `swept::pieces`.
- **A smooth run of moves is one sweep** (`swept::pieces`, #931): moves square to the tool's axis
  running on smoothly are a chain, their sides sewn along their sections (a closed one a ring); a
  plunge into a chain or a whole turn, or a lift out of one, the column over the tool's top alone;
  a drill's pecks one slide. brep-bench's `cnc/bull-nose-profile` and `cnc/pocket-with-an-island`
  (timed out and refused before) build in about two seconds each, to their closed forms within
  2e-15; `cnc/flat-end-mill-profile` in 0.2 s. `patches::Revolved::read` takes a cone's apex as a
  point of its meridian (a drill).
- **`build::prism` takes an arc facing against its profile's normal** (`Seg::facing`): it wrote
  such an arc's cylinder's parameters along the normal and turned the face the wrong way, so a
  bull-nose's section, its corner mirrored across the axis, stood its cylinder 80 off an edge it
  bounds.
- **Sharp corners of a path of slides** (`swept::pieces`, #931): a chain ending where an earlier sweep holds the whole
  tool (a closed path come back to its start) is shut there by its section; and a ball's quarter in
  a half is built with its poles off the section and the equator (`raster::lune`), not on the
  axis, where the next move's side touches it along a great circle through the pole ("a sphere has
  a hole in no piece"). A bull-nose run round a triangle and back, and a ball-end mill round a slot
  narrower than itself and back, are cut to their closed forms; of 25 random polylines cut by a
  ball-end mill, 1 is refused (13 before).
- **A run of slides turning at sharp corners is one sweep** (`swept::pieces::mitred`, #931): each
  slide's halves carried and cut, on the side a corner turns toward, at the plane bisecting it
  (a mitre: a line on a plane, an ellipse on a cylinder), and the tool's wedge turned through the
  corner on the side it turns from, sewn — taken while each slide outlasts its mitres and the run
  turns through a right angle at most. A curve written as a fine polyline is one sweep a quarter
  turn: a bull-nose along 135 sides of a 180-gon in 1.4 s (15 s a side at a time), to the same
  volume. A half of the tool at a run's end is that wedge turned a half turn.
- **A line square to a torus's axis meets it in closed form** (`query`): as the chords of the
  circles at its height, and at the top or bottom of the tube, where they are one circle, only
  touches — where the quartic is `(ρ² − R²)²` and a line tangent to that circle its quadruple root,
  which no isolation proves (a slide's bottom edge ending on the torus a sweep before it turned
  at). A line all but level is the general reading's, as against a turned meridian.
- **Booleans at tangencies** (`complex`, `boolean`): two exact faces leaving an edge are tied only
  within `16 tol / ρ` of one another (a fit's `TANGENT` else), and a tie is read a hundredth of their
  least radius in, not a thousandth of the whole complex — a cylinder and a ball of its radius
  leaving a short edge 2e-6 apart were read swapped 0.14 in, and two cells joined; no vertex is taken
  onto a meeting found in closed form from farther than the tolerance, nor a root that lies flat
  along a face moved off such a meeting; and an edge merged with another records how far apart they
  stood (`Edge::tol`), as an operand's edge keeps its own through a Boolean.
- **A ball-end mill's path of slides is cut from a blank run by run** (`swept::raster`, the road
  `swept::closed::cut_from` takes for it before the arrangement): the mill read off its B-rep
  (`raster::Mill::read`), its moves in runs — each a polyline of the ball's centre in one plane
  holding the axis, every step forward along one way across it — each run's sweep built from its
  surfaces (half cylinders under the path, the ball's lune at each valley, the two cylinders'
  ellipse at each hill, the shank's walls and halves, cut off by a plane past the blank), the runs
  swept across the cores and cut in order, and the cut judged. brep-bench's
  `cnc/ball-end-surface-raster` (41 passes, 944 moves), refused after nine minutes by the
  arrangement's membership budget, is cut in about 7 seconds and agrees with cadrs's solid to
  1e-10. Any other tool or path takes the arrangement as before, as does a path whose runs the
  road could not sweep or cut.
- **Membership along a path of pieces skips the pieces far from the point** (`swept::member`): a
  slide's sweep lies within the box of the tool's box at its two ends, so a point outside it is read
  against no more of that piece. The judge reads a 944-move raster in 0.6 s rather than 6.5 s, and
  `build::SEAM` is public.
- **An outward shell ends where its opening is** (`offset::shelled`, `Side::Out`): each piece that
  reaches an opening is ended at its plane, and no other, where the whole wall was cut off at the
  plane. brep-bench's `shell/l-block-open-at-its-step-outward-sharp`, an L open at its step, kept
  none of its tall leg's walls past the step's plane and was wrong (904 for 1868); it is now its
  closed form, sharp and rounded. A wall meeting its opening aslant is cut off at the plane where
  it runs past it, rounded, and refused where nothing fills what it falls short of — sharp, always
  — where it was built short of the plane before.
- **A sheet's face is read by columns only at its fold** (`props`, #880 T1b): 0.14.0 read a carried
  section's sheet by columns wherever its area factor fell below `geom::NEAR_FOLD` across a face's
  box, which caught sheets whose rates merely lean together without folding. brep-bench's
  `swept/spline-bar-screwed-through-rod`, read so at every reading its trim made, took 606 s where
  0.13.0 took 73 and timed out on the board. A face is now read by columns where its sheet's area
  factor falls below a millionth at a point of a use, those its box is read from: the bar builds in
  58 s again, to the same volume, and every fold case reads as it did.

- **A fold reaches the contact's road from a drawing** (`swept::dispatch`, #880 T1c): under a constant
  twist the closed class's admission takes a tool whose sheet folds — the class's own `Folded`, and
  the walk's fold rows E3 and S3 — for the road, as it takes T1, T2, S1 and E1, keeping the first
  refusal where the road refuses too. A ring pushed into a block past its fold builds from a
  drawing, and so do a ball screwed round a helix tighter than itself and a rod turned along itself,
  whose fold's two leaves lie on one surface of revolution and are kept once.
- **The contact's road trims a sweep within the blank's reach where the whole does not close**
  (`swept::closed`, `swept::trim`, #880 T1c): cut from a blank, the sweep is wanted only inside a
  box holding the blank. Where the whole sweep's trim does not close, it is trimmed there: the
  candidates' pieces outside the box are dropped and paired only near it, and the box's faces are
  kept where the sweep holds them. The judge reads membership inside the box. `trim::Trim` gains
  `reach` (**breaking** for a struct literal). The twist drill's flute wheel too steep for its helix
  builds so: its flank's fold meets the wheel posed at the roll's ends far outside the stock. So
  does a pin domed over its axis's pole, turned about an axis its own meets, to its closed form.
  The whole sweep is still asked first, so every sweep the road built before builds as it did.
- **Two sections with one meridian image under a turn sweep one surface** (`swept::contacts`): a
  section's image about the axis was matched to a surface already made against that meridian's
  256-chord polyline, whose sag (~2e-5 on a unit arc) stood above the tolerance, so the second of
  two identical images — a rod's two end rims — was fitted a surface of its own, which `ssi::same`
  did not read as the first, and the trim kept both copies of where they overlap, leaving an edge
  three faces used. The match is measured on the meridian itself.

## [0.14.0] - 2026-11-14

- **A sheet that folds under a slide, a turn or a screw goes the contact's road** (`swept::closed`,
  #880 T1): what the closed class refuses as `Folded` under a constant twist is asked of the road,
  as `Curved` is, whose trim keeps of each folded sheet what bounds the sweep. brep-bench's
  `swept/ring-pushed-in-past-its-fold` and `swept/ball-screwed-round-a-tight-helix` build, each at
  the kernel's tolerance within 5e-10 of a volume made apart from the build; a ring slid past its
  fold and a ball screwed tighter than its radius, refused by name before, build too.
- **A stretch of contact is cut clear of its folds** (`swept::contacts`, #880 T1b): not within half
  its reach of where its image in the motion's invariants turns back, or where the contact runs
  within a tenth of a radian of the flow. A cut drawn onto a cusp ended the stretch on the fold's
  path, where its pieces did not close: a ring leaning just past its fold (60°) is built now, at
  every tolerance. Still refused: a swallowtail narrower than the contact's points (56°).
- **A swept sheet's face across its fold is measured by columns** (`props`, #880 T1b): where a
  carried section's sheet folds within a face's box, its area and flux are read as `∬ g du dv`
  column by column, not by Green's theorem, whose `G` carried the inverted uses' error beside the fold (the
  ball screwed tighter than its radius, 3.3e-9 off; now 2.4e-10). Every other face reads as before.
- **A half's way at a node where its face's rates nearly run together** (`boolean`, #880 T1b) is
  first read a fiftieth along it rather than a ten-thousandth: beside a fold's path a point stands
  in the parameters at the square root of its distance in space, and the way read off by radians.
  `Surface::area_factor` and `geom::NEAR_FOLD` say where a sheet's rates run together, for this
  and for `props`.
- **A ball rolls on a B-spline sheet** (`fillet::roll`): brep-bench's fillet/bezier-dome-one-edge
  builds, its volume within 1.2e-8 of one integrated from the patch alone. A sheet's offset is read
  by its signed distance, on the sheet carried on past its domain by its own polynomials:
  `Surface::Offset` (a sheet's frame and net, and the distance) is a new kind (**breaking** for code
  matching `Surface` exhaustively), made by `Surface::parallel` (`has_parallel`), which answers an
  `Offset` as `Surface::offset` does, with `Net::d2_continued` and `Surface::continued_inverse`.
  `ssi::same` takes two nets that agree over the domain they share for one surface (a net and itself
  carried on by `Net::extended`), so two solids sharing a sheet now join where the Boolean refused
  ("coincide or touch to higher order"); and an edge lying along a face is split where it crosses the
  face's edges though neither is a line or a conic (`query::enclosed_crossings`, where it was left
  whole), so a Boolean's root where an edge touches a round flatly is the round's contact crossing
  it. `roll`'s refusal for a kind with no offset read names the kinds it rounds.

## [0.13.0] - 2026-11-13

- **A drawing's sweep reaches the contact's road** (`swept::dispatch`, #880 Q1b): under a slide, a
  turn or a screw a tool the closed class reads no candidate for is admitted to it where the class
  asked first refused the sweep by a row the road answers (T1, T2, S1, E1), and its build takes the
  road; `dispatch::Closed` gains `first`, that refusal, said where the road refuses too (**breaking**
  for code building a `Closed` literally). A fold of a section's meridian image is read only from
  steps past the tolerance (`swept::contacts`), where a contact meeting the turn's velocity at its end
  was cut twice a hair from it.

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
