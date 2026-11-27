# The Rust B-rep kernel

Part of the [contract](../../CLAUDE.md) the code is held to. Read it when you touch `brep/`. An
issue number (#n) or a spec version ([0.n], see the [language changelog](../solvent-changelog.md))
says where a rule came from.

**The Rust B-rep kernel** ([plan](../rust-kernel-plan.md)); clean-room — OCCT is LGPL, this repo
MIT: its code is never translated, only its behaviour and conventions studied.  It is its own
crate, `cad-kernel` (`rust/cad-kernel/`, MIT OR Apache-2.0), which names neither the solver nor the
language; `gcs-solve` re-exports it as `brep` and adds what reads the model (`brep::sweep`,
`brep::export`), so its paths below are `gcs_core::brep`'s as before.  **It lives here and is
released from here**, to crates.io, by a tag `cad-kernel-vX.Y.Z` on main
(`.github/workflows/cad-kernel.yml`, prepared and tagged by `scripts/release-cad-kernel.sh`); no
other repository holds a copy.  A release is the commit the whole fast suite passed at, whose
`rust/cad-kernel/Cargo.toml` names the tag's version and whose `rust/cad-kernel/CHANGELOG.md`
has its section; its package builds alone and runs its smoke test (`rust/cad-kernel/tests/`),
documents without a warning, builds for wasm, and keeps semver against the version before it
(`cargo semver-checks`: under 0.x a breaking change is a minor bump).  What builds on the kernel
from outside pins a released version: it is measured beside other kernels on
https://github.com/bmander/brep-bench at the version that names.
`gcs_core::brep`:
`geom` (surfaces parameterised as OCCT/STEP do, closed-form inverses, signed distances, `Traced`
intersections), `topo` (`Brep`: oriented uses with pcurves, degenerate poles — a sheet's net whose row of poles
collapses to a point one too, `build::grid_sheet` as `sew::sewn`,
`brep::a_net_closed_round_onto_two_poles_is_a_ball`; `check`, `pinches`),
`build` (prisms, drafted prisms, revolutions and canals of line/arc/circle/spline profiles; the
sphere, the cone and the torus by their numbers as OCCT's `MakeSphere`/`MakeCone`/`MakeTorus` take
them, each its meridian turned — `sphere(c, axis, r)`, `cone(o, axis, r0, r1, h)`,
`torus(c, axis, major, minor)` — and the ellipsoid, `ellipsoid(c, axis, x, [a, b, c])`, the
unit ball on that frame stretched — and what is no such solid refused by name: the kernel's API
speaks the vocabulary its other users bring, which the language need not, `std.sets.Sphere`
staying a set over `distance`; `tests/brep.rs`'s `the_sphere_the_cone_and_the_torus_…`),
`props` (volume by Green's theorem), `query`, `ssi` (closed forms, else traced), `boolean` (split, arrange, classify,
assemble), `mesh` (constrained Delaunay refined to a measured sag), `step` (AP214 with pcurves and
seam curves), `recipe`.
**A traced intersection is written as a cubic, held to the kernel's own bar where no tolerance
is stated and to the tolerance where one is** (#961, `step::bezier_curve`, `step::hermite_fit`,
`step::Fit`): no entity of the standard writes a `Traced` curve, so the file carries a piecewise
Hermite cubic through its points and rates. **A reader measures the faces the file trims**: it puts
each face's loop where the edge's curve falls on the face, so how far a curve stands off decides
how far off the reader's volume, area and moments are — a polyline at 1e-4 mm read brep-bench's
half torus, a ball bored off its centre and the saddle drums 1e-6 to 1e-5 off their closed forms,
and a cubic fitted to 5e-5 mm in space took the half torus to 2e-6, the lens of two spline balls
to 7e-6 and the grazing cylinders to 1.2e-4, where the fit before #961 (a piece at every marched
point, each within 1e-7 mm) read them 1.6e-9, 6e-9 and 6.1e-9 off, and `Fit::Exact` reads them
3.7e-11, 1.3e-10 and 6.1e-10 off (no brep-bench case graded worse). So `step::write` takes two numbers apart: the uncertainty the file states (`tol`, what a reader may
take two points to be one within, and the bar of the polyline written where no cubic closes in) and
how near its fitted curves are held (`Fit`). **No tolerance stated is no licence**: `Fit::Exact`
holds every fitted curve within `step::EXACT` (a thousandth of `step::FIT`, 1e-9 mm), whatever the
stated uncertainty (1e-4 mm). **A tolerance stated is one** (`Fit::stated`): the curves within
half the uncertainty the export states for it (`export::step_accuracy`), and the file is the
smaller for it. Either bar is held **on the faces the curve trims**: each piece of the cubic is
kept where, at sixteen points, it stands within the bar of the exact curve at the same parameter
and its foot on each of the two surfaces stands within the bar of the meeting along that surface —
where the surfaces meet at an angle `θ`, a point `δ` off the meeting in space has a foot up to
`δ / sin θ` along the face, so at a grazing meeting the cubic is held nearer in space. Its pcurves
are fitted alike (`bezier_pcurve`, measured on the surface the file writes against the edge's foot
there), so the cubic and each pcurve's image stand within twice the bar of each other at every
parameter, and their ends are the edge's vertices. The pieces are marched along the curve as long
as the bar allows (a cubic's miss goes as the fourth power of its length, so each is shortened
where it misses and the next lengthened by the fourth root of the bar over the miss), stopping at
every break of the curve (a spline's knots); joints where the curve is smooth share their rate and
are double knots. **A traced curve is written over the length along it** (`Along`), not its own
parameter: that counts the march's points, and its speed steps at each (each stretch is a parabola
through a neighbour behind on one side of a point and ahead on the other), so no cubic over several
points follows it and every piece ended at a point. Over each stretch `s` is the cubic rising by
the stretch's length (Gauss–Legendre on the curve's speed) at the curve's own speed at either end,
so the curve in `s` is `C¹` across the points, and the curve and its pcurves are written over it
alike. **The pieces follow the bar, not the march**: held to 1e-7 mm and cut at every marched point,
the issue's tube with one cross-hole was 3.3 MB, each of its curves some 3,370 poles, the same at
every tolerance. Only where the fit does not close in (a piece shortened to `2⁻³⁰` of the edge, 4096
pieces) is the polyline through the curve written, within the stated uncertainty where no tolerance
is stated and within the bar where one is. A meeting with a closed form STEP carries — a line, a
circle, an ellipse, a conic as a rational quadratic — is written so and fitted nowhere (a plane
cutting a cone in a hyperbola or a parabola is traced yet). `tests/step_curves.rs` (the file read
back and every traced cubic and pcurve sampled 64 times a knot span: within the bar of the curve, of
both surfaces and, its feet, of the meeting along each face, at three stated tolerances and none,
each finer tolerance nearer and in more poles, none nearer than a sixteenth of its bar, for crossing
cylinders at several places and radii, a ball drilled off its centre, a frustum and a torus drilled;
a torus cut beside its axis (the half torus, cut through it, is cut along its meridians in
closed form since #996), the ball bored off its centre and the grazing cylinders as the file trims them,
measured by `props`: within 1e-8 with no tolerance stated, within the bar's strip at one; ellipses
written exactly), `tests/face_edits.rs::a_traced_curve_is_written_as_a_cubic`.
**An affine map carries a solid onto its image exactly** (`Brep::mapped`, `affine::Affine`, any
invertible `p ↦ M p + t`): its topology and pcurves are the solid's own, because every surface's
and curve's image keeps their parameters. A kind whose image is a kind of its own is written so
(`Surface::mapped`, `Curve::mapped`, each saying what its parameters become): a rigid map moves
it, a similarity scales it (a sphere stays a sphere, a cone a cone), a plane is a plane on its
frame made orthonormal again (its pcurves carried by the 2D map of its parameters), a B-spline its
poles mapped, a line a line at the speed the map gives it, a circle or an ellipse an ellipse about
its principal axes started where the map takes its start, a traced curve its points and surfaces,
a blend carried along a line the blend of its edges' images. Anything else is wrapped,
`Surface::Affine` and `Curve::Affine`, `A(S(u, v))` over the preimage's own parameters: an
ellipsoid is a sphere's image (`build::ellipsoid`, brep-bench's geometry/ellipsoid and
ellipsoid-half), an elliptic cylinder a cylinder's, a sheared torus, revolution, extrusion, screwed
blend or face family's wall theirs. **What is measure is measured in space; what is incidence is
read in the preimage.** A wrapped surface's foot is the nearest point in space's own metric —
exact for a sphere's or a cylinder's image (`Quadric`, its principal axes by `util::linalg::sym_eigen`, the
foot by Eberly's bisection, `ellipsoid_foot`, `ellipse_foot`), by Newton from the preimage's foot
otherwise — and its signed distance the distance there, on the side the preimage's says. Where it
meets another surface is where its preimage meets that surface carried back, found in the
preimage's closed forms at the tolerance shrunk by the map's greatest stretch and mapped back
(`ssi::intersect`'s `pulled_back`: a plane meets a sphere's image in an ellipse, a box's sheared
faces each other in lines); traced where the other, carried back, is still an image, or where the
preimages have none. A line meets one likewise (`query::curve_surface`); two images are one where
their preimages are, or their quadrics' forms agree (`ssi::same`). A map reversing space walks
every loop the other way round, and a face's sense follows its normal's image (`M⁻ᵀ`, by the sign
of `det M` and of its parameters' map). An image is not of revolution and has no offset in closed
form. STEP has no image of a surface: each wrapped face is written as its preimage's exact net
over the parameters it reaches, poles mapped (`step::face_net`, the net a sheet, a loft's face and a family's wall are written as too: one turn of a rational circle by
a sphere's or a torus's meridian, a cylinder's or a cone's generator, a revolution's spline, an
extrusion's spline by its step; a blend's or a family wall's fitted net), its pcurves fitted in the
net's parameters (`nurbs::arc_parameter`, a quarter arc reaching the angle `θ` at the fraction `τ`
where `τ / (1 − τ) = sin(θ/2) / sin((d − θ)/2)`) and measured on the net. `affine::every_kind_of_face_maps_to_its_image`
(every kind under a shear, a skew stretch, a reversing map and a similarity: valid, `|det M|`
times the volume, mapped back itself, its nets its surfaces, read back from STEP, meshed closed),
`affine::a_boolean_of_images_is_the_image_of_the_boolean`, `face_family::a_family_mapped_is_its_image`
and the `ellipsoid::` tests are the gate.
**Two spline sheets are searched for a meeting only if their enclosures may meet**
(`boolean::separated_nets`): overlapping face boxes alone need not mean the surfaces meet. Bisect
the parameter whose reach (its range times its derivative's enclosure) can take the most of what
the two boxes share of one axis, of those the one of greatest reach — the greatest reach alone
halves parameters along axes the sheets share wholly, and left the gate's own columns, 0.3 apart,
unproved — discarding a box
pair only when its outward-rounded point enclosures stand apart by more than the arrangement's
padding. An unbounded enclosure, a box no longer worth splitting, 256 box pairs read (those
discarded counted too) or one still meeting 32 bisections down — where crossing sheets lead the
search, which is given up there rather than followed to the budget — leaves the pair to the
ordinary intersection algorithm; uncertainty never means separation. The proof reads the two nets
alone, so it is made once for every two faces on them.
The teapot's rim and lid stand apart despite their overlapping boxes: projecting a grid onto
each other searched for meetings they cannot have. `brep::spline_columns_with_overlapping_boxes_keep_their_meetings`
holds separated and crossing polynomial sheets to their columns' exact volumes, and two columns
sharing a sheet (coincident spline supports, one net: `ssi::same`) to the one column they make,
that meeting read and not dropped; the volumes
hold with or without the proof, so it reads `separated_nets` itself: true of the separated
columns' sheets, false of coincident and of crossing ones and under a padding wider than the gap.
**A clamped cubic with four poles is evaluated in its Bernstein basis** (`nurbs::with_basis`),
including the first two derivatives scaled by its knot interval; other knot vectors still use
Cox–de Boor. Rational curves use the same basis in homogeneous coordinates as before.
`brep::a_cubic_bezier_and_its_quartic_have_the_same_two_derivatives` checks it against degree
elevation over translated and scaled domains, with and without weights, at the ends and outside
the domain, and checks that an unclamped cubic keeps its own basis.

A traced pair is seeded where edges cross faces; between analytic
surfaces `ssi::unseen` then searches one face for curves no edge reaches (cells proved clear by
the signed distance's Lipschitz and second-order bounds, else beside a known curve, else seeded),
refusing what it cannot resolve: an untraced meeting is never read as none (issue #58).
**What it cannot resolve off either face is none of theirs** (#961, `Located::face_clear`): the
search keeps to the face's box of parameters, grown past its edges (a cone's past its apex, onto
the other nappe), and to the box every face on the two surfaces fills, so it reaches where both
surfaces run on past the faces; a cell down to the leaf whose every point — within its reach of
its middle, and of that point's foot on the other surface — stands off one of the two faces is
answered clear and never refused. Off a face means outside it (`face_place`) with every edge of
its loops (a seam, an apex) farther by its box than the surface's points within that ball reach
along it (`ssi::disc_reach`, each kind's bound argued by a path in its comment; none where
the ball crosses a tube or reaches an apex or an axis, or a horn's or a spindle torus's other sheet):
a near-contact beside a face's edge is held to it and refused as before, and so is one within
both faces, and any near-contact of a meeting over boxes (`meeting::meet`), which has no faces.
A countersink beside a counterbore, whose cone past its apex came within 4e-7 of the wall's
cylinder run on below the floor, was refused so; `tests/countersink.rs`'s
`a_countersink_beside_*` hold the part to its closed form, centred by the wall, turned and turned
over, against a cylinder and a drafted wall's cone; `a_near_contact_within_both_patches_is_still_refused`
holds the same near-contact, met over boxes that hold it, refused; and
`a_face_is_clear_only_off_it_and_away_from_its_edges` the rule at a face's edges and a cone's apex;
`a_surfaces_points_within_a_ball_reach_no_farther_along_it_than_said` holds each bound to seeded
points and their paths. Clear is read only off the faces, so a refusal there no longer stands in
for the faces' own meeting: **an edge is a stretch two faces touch along only where it lies in
both their surfaces** (`boolean`'s stretches, each edge of one operand lying in the other's face
held to this face's surface too). A round's run cut off by the lobe cylinder it leaves 1e-3 rad
off tangent (a plate's rim past a reflex corner) has its wall's and end's upright edge on that
cylinder, which its round face reaches only at the foot, where the two cross at 1e-3 rad: read as
theirs, the foot's seed was dropped as a touch along it and the march, shallow there, ended 5 µm
short of it on no edge, the arrangement's shells kept and dropped on either side of the cylinder.
Before #961 a near-contact past the run's end, off the round face, was refused and the pair read
from its ridges, which hid it; now the foot is seeded, refused as shallow, and the pair read from
its ridges for that. `tests/rim_rounds.rs`'s `a_reflex_corner_off_tangent_rounds_its_rim` holds
the plate rounded 1 and 2, mirrored, to its closed form (`std.rounds`' runs cut off by the faces
they end on: the lobe's disc run on past the corner and within reach of the side is taken by
neither), and `a_run_cut_off_by_the_cylinder_it_leaves_off_tangent_is_met_to_its_corner` the run
cut off alone, turned 1e-3 to 1e-2, rounded 1 to 3 and mirrored.
**A turn's torus is read whole, its lemon and its poles included** (#996). A spindle torus
(`minor > |big|`, a section clipped where it reaches its axis) has in each meridian half-plane two
circles of its tube's radius, about `±|big|`: its outer sheet and its lemon, which meet at the two
poles on the axis (a horn's touch there, a ring's second lies wholly across it). A point's distance
from a surface of revolution is its distance from the meridian in its own half-plane, so a ball
about a point farther than its radius from the other circle holds points of its own sheet alone,
and `ssi::disc_reach` gives every torus the ring torus's reach there (`minor ≥ 2r`, `ρ ≥ 2r`): a
near-contact on the lemon past a turn's clipped arc, where a leg's cylinder runs on tangent to the
torus's continuation, is clear of the turn's face and no longer refused as unresolved. The
search's derivative bounds over a strip of `v` (`ssi::derivative_bounds`) are the parallels' own,
`|big + minor cos v|`, which vanish at a pole: read as the widest parallel's, a strip beside a
pole, all one point, was halved along `u` until the search's four million cells ran out. **A plane
through a torus's axis meets it in its meridian either side**, two circles in closed form
(`ssi::intersect`, as a plane through a cone's axis holds its generators): a turn's caps stand in
such planes, and traced, each cap took its circles in two marches of 100 000 points, a quarter
elbow and its two legs 3.3 s, now some 20 ms. **A loop's area in its face's parameters is
Simpson's, by one rule** (`topo::green_area`, which `Brep::loop_area` and a Boolean's cycles'
`fine_area` both read): Green's theorem, each curved use sampled at `n` steps and a straight one
by its chord, each pair of steps' bulge past its chord taken as a parabola's, `n` doubled from 32
— the samples kept, the new ones read between — until the readings at `n` and `n/2` agree to a
thousandth, or 4096 steps a use. A polygon's area misses each arc's segments by more than a thin
strip holds: a crescent between a turn's cap's rim and a leg's a thousandth inside it, 308° long,
read so turned the wrong way, a loop refused and a Boolean's cycle dropped as a hole. **A tube all but tangent to another at a seam is on it** (#996, `flush::tubes_flushed`, the
tube's sibling of an end all but flush with a plane): before a body's operand is combined with the
solid (`boolean::combined_written`), a cylinder of either whose end circle stands concentric with
and in the plane of a circle of the other — a leg on a turn's cap — with radii differing at all but
by no more than `FLUSH` of the Boolean's tolerances (a part in a million of their size) is moved
onto the other's radius, exactly: its surface, its circles and its seam carried square to its
axis, its ends' planes where they were (the operand's cylinders first, onto any circle of the
solid's; then the solid's, onto a circle of the operand's no cylinder bounds — a turn's rim). A
cylinder whose ends meet anything but planes square to its axis, or whose circles ask for two
radii, is left as drawn. So a turn a near-miss off its legs within that reach is the tangent turn,
its legs at its tube's radius, whichever order the three are written in and whatever is joined
or cut after; past it, its tube leaves a cap's crescent `δ` wide between the turn's rim and the
leg's, a step round the seam, built as the solid it is. Within the reach the arrangement's readings
of the seam parted — a point onto a vertex within the tolerance, a vertex onto an edge and an edge
onto an edge within eight, the turn's poles and the legs' crossings of its axis 2.3δ apart — and
one near-miss in three, joined to the legs or to anything after, was refused ("a shell of the
arrangement is kept on one side … and dropped on another", "a plane's pieces do not close",
"edge 3's pcurve is 1.3e-6 off it") or, at 1e-7, built 2.6 times too large. The readings below make
the step itself sound where the tube is past the reach. **A piece
thinner than any outline it is drawn to has an inside all the same** (`boolean`'s
`reading::inside`): where the spans of its outlines, drawn down to a few ten-thousandths of the
piece, hold no point, steps square to each curved half from its middle, down to twice the
tolerance, are read against the halves by the side of the nearest, and a piece whose sides stand
apart by more than the tolerance is kept; only one with no inside so is a sliver merged into its
edge. Read by its outlines alone, the crescent between a turn's rim and a leg's twelve
tolerances inside it was dropped while its two sides stayed two edges, and the turn's face hung by
its rim ("a shell of the arrangement is kept on one side of a plane and dropped on another"). **A
meeting the unseen search finds off either face is not traced**
(`ssi::unseen_beside`): a curve through it lies outside the faces or crosses an edge into them,
where that edge's crossing seeds it, and traced it was a march of a hundred thousand points no face
kept (a tube a ten-thousandth inside its leg, crossing the leg's cylinder run on under the seam,
seconds a build). `brep_export::a_turn_a_near_miss_off_its_legs_is_the_tangent_turn_or_built_with_its_step`
(δ either way by decades from 1e-8 to 1e-2, two, three, five and seven of each decade from 1e-8 to
1e-6, and the issue's document, its radius left to its ends' hints: the tangent turn at the tube's
radius within the reach, the step past it, each to its closed form within the tolerance's band
over its boundary) and `the_fixtures_elbow_cored_and_flanged_is_built` (the issue's document cored
by a bent passage of radius 17 and flanged at each end, to its closed form) are the gate.
**A use ending within its vertex's or its edge's tolerance of a pole is at the pole**, by one
reading wherever the pole is read (`topo::near_pole`: the point within that tolerance of the pole
in space, half a turn of the collapsed parameter moving it no farther than π times it): an inverse
pcurve's end (`topo::on_pole`, its edge's tolerance), the corner a loop is closed at by a pole edge
(`Brep::close_poles`, its vertex's and its edges'), and in a Boolean an edge's end laid across a
face (the arrangement's) and a half's points (`uv_seeded`, its edge's gap and its vertices'). Read
on the surface's own inverse instead, the leg's circle standing for a turn's rim a tolerance
outside it, ending across the axis from the pole by as much, came out half a turn round — the loop
broke in its parameters, its piece read as a hole, or, closed, it enclosed the wrong side (a union
2.6 times the solid). And **a pole's
half runs along the parameter the pole collapses** (`pieces_of_face`'s ways), its ends' other
parameter disagreeing by a node's width read as no way at all, where the pole's vanishing rate
along the first had made that disagreement its whole way and tied it with a meridian there. And
**two uses of one edge at one place, walked out and back, are a spur and not a seam**
(`reading::judged`): no side lies between them, so a cap's crescent whose two sides the
tolerance made one edge has no inside and is merged into its edge, where the side of one use had
read a point outside it as in it. So an axle joined along the bend axis and a slab cut along a plane through it, through the
turn's poles, and a bore across its bend are built over every near-miss
(`brep_export::a_solid_through_a_near_miss_turns_poles_is_built`; the fixture's refusal "loop 0
breaks in parameters" at a pole was the slab's at 1e-8 and the turn's alone at −1e-7).
`countersink::a_surfaces_points_within_a_ball_reach_no_farther_along_it_than_said`
(spindle and horn tori among its surfaces), `brep::a_plane_through_a_toruss_axis_meets_it_in_its_meridians`
and `brep_export::a_swept_elbow_joins_its_legs_at_their_tangent_seams` (ring, horn and spindle
sections, turns of 45° to 135°, one arc or two, the legs either way round or the turn first, and
a tube a thousandth wider than its legs, each held to Pappus and the legs' own union) are the gate.
**A march that steps across a touch ends at the touch** (#879, `ssi::trace_beside`). Two rounds
tangent to one face meet in a curve that ends where both touch it, a point where the two surfaces
touch and their meeting's branches cross (`ssi::touch`); where that point is on a face's own edge
(a corner's round ending at the side a rim's round is tangent to), the far side of the crossing is
off that face, so a step landing there is beside a meeting already known (`Touches::outside`).
Such a step ends the march at the touch it ran across — a touch behind it with a branch along the
way it came — not where it landed: ended past it, the curve ran through that vertex, and traced
the other way round as well, through it twice; an edge laid along it took one passage, the stretch
from the other was never laid, and two cells of the arrangement joined across it ("a shell … kept
on one side … and dropped on another"), or not, as the steps fell — a box's top rim refused where
its bottom rim was built. **A closed curve with no period has its vertex at both ends**
(`boolean::spans`): the vertex both ends of such a fitted meeting are is set at its start and at
its end wherever along the curve it was read; read at the far end and set down at the start alone,
it stood after the points it comes before, and the stretch from the last of them round to it was
not laid. `tests/rim_rounds.rs` is the gate (a rim's run crossing a corner's round at a box's sharp
corner, on either cap, to one volume).
**A definite touch is alone by its own orders, not by the faces' size** (#996 §3,
`Touches::alone`, `ssi::alone_over`). Where two faces touch at a point and `ssi::touch`'s `Q` is
definite, nothing within its radius `2 SHALLOW / λ` (`λ` its least bend) — where a trace cannot
follow the two and the search for unseen meetings cannot part them — meets but the point, so long
as the higher orders keep what the second says: `b`'s distance along `a` grows from its own at
the touch the way `Q` turns it, by at least `¼ λ ρ²`, on rings about the touch from its radius in
to a 256th of it, its samples closer than that growth lets the distance fall between them. From
its own: the two are found to touch within some tolerances of nought, and a loop they cross in
within that (a side turned `3e-4` off a lobe, 5e-7 inside the round's torus over 7e-3) is the
touch (`rim_seams::a_corner_too_nearly_tangent_to_round_is_said_by_its_faces`, built at `3e-4`). Shown, the touch is read alone and the
search passes it, however small the faces beside it — an arrangement's (`Touches::alone`), and
`meeting::meet_with`'s past the touches its critical set found, which before found each again,
traced nothing from it and refused the meeting; not shown, it is not. Before, a touch was
read alone only where its radius was under a twentieth of the smaller face — a rule for the
crossing touch, whose branches are asked after twice the radius out on both faces, and kept for
it — so a rim's round on a short side, a cylinder crossed square with a wide blend's cylinder at
the point where the blend's foot crosses the round's tangent line (`2 SHALLOW · 8 = 0.28` beside a
face 5.5 across), was refused: "a cylinder and a cylinder meet … where no edge crosses, in a curve
that could not be traced" (CADGenBench fixtures 104 and 122). Gates:
`touch_curves::a_rims_round_crossing_a_blends_foot_touches_it_at_a_point_alone` (the round's bar cut
from a wall going on into a blend 8 to 40, rounded 1 to 3, the touch on its face's edge and inside
it, turned and mirrored, to the closed form at 1e-9),
`meeting::a_cylinder_laid_across_a_surface_from_outside_touches_it_at_one_point_alone` (a cylinder
laid across a cylinder, a cone, a ball and a torus, crossed 90° to 25°) and
`rim_rounds::a_rims_round_crossing_the_foot_of_a_lugs_blend_is_built` (the issue's plate, rounded 1
to 3, either order, mirrored, to its accounted volume).
**A traced meeting is one curve, on both surfaces, keeping its turns** (`ssi::trace_beside`,
#684 E). A march ending at a patch's edge or a touch drops the points crowding that end (within a
quarter of its step `h0`) only while the chord from the point kept before them to the end passes
within `tol` of each, so a turn sharper than `h0` near an end is kept, not cut to a chord
(`drop_crowding`). Its fit is accepted near every traced point and then on both surfaces between its
points, to the trace's `tol`, refined where it strays for as long as that leaves fewer gaps off
them; where it does not (a gap narrower than the trace's noise, whose splitting crowds the cubic's
points and makes it wiggle the more) the fit with the fewest is kept (`fitted`). Of two traces, one
lying along the other — its ends within two marches' slack — is that meeting traced again from a
seed the first left unrecognised, and only one is kept, before either is fitted: the one reaching
farther, else the one whose longest step is shorter (`distinct`). **It is fitted through one seed a
place**: a seed within `tol` of one taken is that vertex again and is passed by. Each is placed
along the trace only to its search's width (1e-8 of a step), and two a hair apart placed apart
were both interpolated, the cubic magnifying their rounding into a swing out along the meeting and
back, on both surfaces all the way (a ring groove's floor ran 0.17 mm past its corner, a crossing
seeded again 4e-14 off it by another face's edge: `brep::a_trace_is_fitted_through_one_seed_a_place`
and `flat_faces::an_end_mill_turned_cuts_a_ring_groove` are its gate).
`brep::a_trace_keeps_a_sharp_turn_at_its_end` (a V of apex radius 0.003 mm, 0.03 mm from the sheet's
edge, against an `h0` of 0.2) and `object_builder::a_sharp_rims_fan_builds_at_the_displays_samples`
(`vthread.sv` at the page's samples, which each of the three builds alone) are the gate.
**A march stepping across a touch ends there** (brep-bench's nurbs/groin-vault): where two
surfaces touch and their meeting's branches cross, a step of the meeting's scale lands either side
of the little disc where they stand nearer parallel than a trace follows, and `∇a × ∇b` turns about
between its ends. A touch found there along the march's branch (`touch_near` from the chord's point
where the two sines' sizes say it vanished) ends the march as one arriving at a touch does, and the
branches no curve has arrived along are traced from it. The Boolean pools where traced curves of
one meeting end at one another (`boolean::ends_met`), the touch a vertex its half-branches share.
Marched on, two vaults' groins were each one curve through the crown, crossing the other at no
vertex, and one, its middle 1e-6 from the touch, was dropped as touching. `touch_near` is settled
by a step a thousandth of `tol`, or one within a tenth of it no longer shrinking, its differences'
rounding (the crown at a `tol` of 1e-9 cycled through steps of 1.3e-11).
`brep::a_trace_stepping_across_a_touch_ends_there`, `a_groin_vault_is_four_vaults_meeting_at_the_crown`
and `brep_export::a_groin_vault_is_built_exactly` (`groin_vault.sv`) are the gate.
**The point of a cone or a torus nearest a point is read across its axis** (`Surface::inverse`,
#721 G5): a cone's generator runs on past its apex into the other nappe, and a spindle torus's
meridian circle (`r > |big|`) crosses its axis to make the lemon, so each one's meridians in the
half-planes either side of the axis are one curve. The nearest point lies on the meridian in the
point's own half-plane (`u`) or the opposite one (`u + π`, the point at `−ρ` there), whichever is
the nearer; where the meridian reaches no farther than the axis (an ordinary torus, `big ≥ r`, the
near circle always nearer by the triangle inequality, the far one not measured) the own one is, as
before. Read in its own alone, a point behind a pointed tool's apex fell on the generator past it,
which the face's parameters took for the face. A face trimmed to one side of the axis may hold only
the other foot: `Surface::feet` gives both, the nearer first, and the tool's material
(`read::Boundary::nearest`) takes the nearest the face holds; a foot at a pole or an apex (a
degenerate edge's point) says no side, the face's chart having no normal there (just past a cone's
apex it reads the other nappe's), and the point's place decides. **Near a degenerate edge the face's parameters decide**
(`Located::face_place`): within the near-edge reach of a pole or an apex every edge of the face
meets in space, so the side of the edge nearest in space is no reading of where a point is — a
point of a cone's other nappe just past its apex was read beside its seam, in the face — and the
winding of the loops drawn in the parameters says. Gate: `swept_apex::behind_a_point_is_outside_the_tool`
(every point of a cone and a spindle torus its own foot, every point past the apex off the face,
the tool's material behind its point the distance to the apex, and behind it and beside it the
distance to the face's own nappe).

**A point is read against a face by one drawing of its boundary** (`query::Outline`): each edge
cut in chords until every chord's middle lies within a bar of the edge in space, carrying its
`sag`, how far a chord strays at most. Its winding says inside only for a point clear of every
edge by more than the sag; nearer, the side of the nearest edge says (`Located::face_place`, and
`boolean`'s `reading`: a piece's inner points, a hole's host). A piece of a Boolean is kept or
dropped at an `Inner` point (made only by `reading::inside`, so no unread point classifies a
piece): a middle of a level line's span across the piece's outline read so, the outline drawn
finer while none is; then, where every middle is on the other's boundary along no face of its
own on that surface, a point either side of the middles (a boss tangent to a side at its middle,
#597, `brep::a_boss_tangent_to_a_face_inside_it`). A piece no point of which leaves the other's
boundary that way is refused naming its face and the other's it touches. A piece with no `Inner`
point at all, its outline drawn as fine as it goes, has no inside: the sliver a surface touching
the other solid only along an edge of it cuts off (a hyperboloid along the ruling a face of the
other holds, #721 X6). It bounds nothing and is dropped; whether what is left closes is the
result's check
(`swept_closed::a_block_leaning_to_the_axis_turned_part_of_a_turn_is_its_slices_swept`). **A piece
narrower than any outline the readings draw has an inside all the same** (#994, merged with #996:
one reading, the strip's above, `reading::beside_halves`): a band a few tolerances wide between two
curves is narrower than the sag of any outline, and dropped as having no inside it left a hole — a
boss a micron wider than the plate's lobe it stands on overhangs the plate's top along the lobe in a
crescent that wide, and the boss's wall stood open along it ("a plane's pieces do not close", "a
shell … kept on one side … and dropped on another"); #994 read it off the exact boundary from the
halves' middles and quarters (`off_halves`), #996 by steps square to each curved half read against
the halves, and the one kept is #996's (`brep_lobes` and `bosses_on_seams` hold the lobe's). **Two circles in one plane meet as far apart as they stand** (`query::curve_curve`,
`plane2::circles`), not as the chord they share stands inside the first: that is half as deep where
their radii are one, so a lens twice the reach deep read as a touch between the two crossings the
cylinders through the circles meet in, a third vertex on the edge
(`brep_lobes::two_circles_overlapping_a_few_tolerances_cross_twice`). A fixed polygon of a
circle once cut its chords short by more than a narrow strip beside it is wide, and read the strip
at the hole next to it (a turned rim's chamfer
from 32 out, `round_lowering::a_wide_turned_rim_is_chamfered_and_rounded_exactly`).
A line meets a cylinder or a sphere in closed form (`query::line_cylinder`, `line_sphere`, one
chord, `line_chord`): one touch exactly at the foot where it is tangent, in one evaluation where a
sampled search takes thousands. A plane cutting every generator of a cone meets it in an
ellipse, exactly (`ssi`'s `plane_cone_ellipse`: about `A + d n + u₀ e` of radii `|d| c s / (c² −
κ²)` and `|d| s / √(c² − κ²)`, nothing where it is the nappe the surface is not; traced before, a
level plane's cut of a leaning frustum and the circle it cuts from a surface of revolution tangent
to it were sampled apart; `swept_turned::a_plane_cutting_every_generator_of_a_cone_meets_it_in_an_ellipse`),
and a circle and an ellipse in one plane cross where the ellipse meets the circle's cylinder,
certified, a touch one root (`query::curve_curve`;
`swept_turned::a_circle_and_an_ellipse_in_one_plane_cross_where_certified`). The cylinder over a slid
torus's apparent contour (#805, `ssi`'s `slid_meeting`) meets the torus at a pose along the slide in
its contact there, touching (each line along the slide through it meeting the torus there alone,
short of the fold), a plane square to the slide in the contour carried onto it, a plane along it in
lines along it through the contour's crossings (isolated certified; a touch left traced), a plane
leaning to it in the contour sheared along the slide onto it (`affine::MappedCurve`), and another
contour's cylinder along the same slide nowhere where the two contours stand apart
(`swept_slid::a_slid_contours_cylinder_meets_its_neighbours_in_closed_form`). Two coaxial surfaces of revolution meet in circles where their meridians cross in one half-plane,
two spline meridians too (a turned contact's meridian read as a spline's, #780 C3) (`ssi::spline_spline`: the first read against the second's signed distance,
its nearest point's normal; left traced where the two run together over a stretch), so two swept
edges' hyperboloids meet in exact circles
(`meeting::two_coaxial_hyperboloids_meet_in_circles_in_closed_form`). A spline meridian's meeting with another and a planar step's with another are read by one search
(`util::roots::sampled_roots`): sign changes of the distance between samples are crossings, and a
**touch is a root of the distance's rate**, bracketed where it changes sign beside a sampled
minimum of the distance's size (ties included) — never a least `|f|`, which a double root places
only to the square root of `f`'s rounding, too loosely for the curves meeting there to share its
vertex (a round's contact on the face it rolls on; a cube's edges touching its midsphere lost their
twelve touches, and the sphere's pieces a hole meeting an outer cycle at no vertex:
`roots::sampled_roots_place_a_touch_where_the_derivative_turns`,
`brep::a_cube_trimmed_by_its_midsphere_is_pinched_at_each_edges_middle`, `midsphere_common.sv`).
**Every curve's meeting with every surface is certified in outward-rounded arithmetic, or refused
with a witness — never sampled, never guessed** (#777 M1, V1, #814). Three boundaries hold it, each
a swap made without its callers knowing:

- **What proves a box** (`util::isolate::Test`, `Test1`). By default **the Krawczyk operator**: a
  box `X` about its middle `m` holds no root where `0 ∉ F(m) + J(X)(X − m)` or where `K(X) = m − Y
  F(m) + (I − Y J(X))(X − m)` misses it, and exactly one where `K(X) ⊂ int X`, `F(m)` and the
  Jacobian `J(X)` enclosed in intervals (`util::interval::Iv`), so rounding is inside the proof
  (#796, #780 R2), not a margin added after it. Only first derivatives are enclosed, and the
  mean-value inclusion holds for any Lipschitz function: across a C⁰ knot the enclosure is the hull
  of both one-sided slopes, so a kink costs nothing (#812: a hand bound of nought across a gear
  flank's kink certified a crossing 1e-3 off the flank, and a 64-tooth gear's material cell read as
  out; `roots::a_crossing_beside_a_sheets_kink_is_read_on_the_sheet`). In one unknown, **the
  interval slope**: a stretch is off where `0 ∉ g(m) + g'(X)(X − m)`, monotone — one root at most,
  where its ends differ in sign, bisected within them — where `0 ∉ g'(X)`. Kantorovich's theorem
  and hand bounds on `|g'|`, `|g''|` stay behind the same boundary (`Kantorovich`, `Bounded`) so a
  search can be read both ways and held to the same roots
  (`enclosures::the_tests_behind_the_boundary_find_the_same_roots`); no curve–surface reading uses
  them. A box is halved otherwise; one narrower than its width that cannot be parted is a touch
  where the caller's tolerance accepts its middle. A stretch's sign at its ends is proved by its
  value's enclosure, never read in floats — rounding's noise beside a double root once made a
  circle's tangency to a torus dozens of crossings — and two crossings a hair apart the opposite
  ways are kept as the pair they are: merged into one, a trace touching a plane flipped a ray's
  count and a ring turned about an axis built three times its volume
  (`swept_closed::a_ring_turned_about_an_axis_along_its_own_closes_into_its_sweep`). **A run is
  proved, not parted**: where a curve keeps within the tolerance of a surface for a while, no box
  can part its roots nor exclude them, so the run is proved — along the curve against an
  algebraic surface by the form's enclosure over a whole stretch (`Function::within`), and along a
  circle or an ellipse also by the Taylor model of the polynomial the form is along it
  (`query::Taylored`, the same at every attitude: the form over the curve's box, a box of the
  world's axes, proved the run beside a circle's touch with a torus only a hair wide, and at one
  attitude in a dozen the search ran out of boxes and refused a gear member's tool that built in
  the pair's own frame, #904 A4,
  `enclosures::a_circle_touching_a_torus_is_read_alike_at_every_attitude`), against a
  sheet piece by piece along a straight path of its parameters between feet found by
  Gauss–Newton, `C(t) − S(path(t))` enclosed so the tangents cancel (`System::run`) — its ends read
  as touches and nothing inside it searched again
  (`enclosures::a_curve_lying_along_a_surface_for_a_while_is_one_run_proved`; a tee's crotch, a
  canal touching its operands, a conrod's pocket rims). A curve all but tangent to a sheet is told
  off it along the sheet's normal (`System::frame`), where the world axes' components are not. **A box is read grown past its sides**
  (`isolate::GROW`), so a root on the face two halves share is inside both and the two that find it
  are one (`roots::a_crossing_on_the_face_two_halves_share_is_found_once`,
  `arrangement::a_bilinear_sheets_block_in_common_with_a_drum_is_the_drum_under_the_sheet`). **A box
  is halved across the side it smears farthest along** (`isolate::solve_by`): each side's width
  times its unknown's Jacobian column at the middle, among the sides still wider than the
  resolution, the widest side where that is not finite — so a box stays about as wide in what it
  reads every way. Halved across its widest side in the unknowns' own scales, a short edge against a
  whole fitted sheet stayed long along the sheet and read ten times the boxes
  (`brep_export::the_pinion_as_written_is_built_by_sectors`).
- **Not proved means refused** (#814 P0). A root is returned only once it is settled within the box
  it was certified in — Krawczyk's own contraction `X ← K(X) ∩ X`, then Newton's method within what
  is left — and the caller's tolerance accepts it there. A Newton step that leaves the certified
  ball, a polish that does not settle, or a box too narrow to part whose enclosure proved nothing
  gives up as `GaveUp::Unsettled`, with the box: never the last iterate labelled certified
  (`enclosures::a_root_that_cannot_be_settled_is_refused_never_returned`,
  `roots::a_root_certified_over_a_flat_system_is_where_it_is`). A reading that gives up is
  `Meets::Refused`, its witness (`query::Refusal`) the curve's and the surface's kinds, the stretch
  and the box: a ray locating a point abandons its direction (`Located::solid_place`: a point off
  the box its faces' boxes fill is out, past which no face stands, and a ray need reach only out of
  that box, the five directions tried by how soon they leave it), a Boolean is
  refused with it in words (`boolean`'s edge-against-face phase, an along-face crossing, a triple
  point; `query::crossings_in` answers the refusal too), and a fillet's leg says so
  (`enclosures::a_reading_that_proves_nothing_is_refused_with_its_witness`). The sampled search is
  retired as an answer path; the profile counts each reading proved or measured (`util::meter`).
- **What a kind of geometry is** (`enclose::Enclosed`): each `Curve` and `Surface` kind encloses,
  once, its point and first derivatives over a box of its parameters — a non-rational B-spline
  curve or net of degree below `enclose::BEZIER`, over a box within its domain and a few spans, by
  its spans' Bézier forms restricted to the box: their control points blossoms of its poles at the
  box's ends (the patch over the box in their hull), its partials' the difference nets' alike, each
  hull widened by a bound on its rounding (`Net::restricted`, `BSpline::restricted`) — tighter than
  de Boor read in intervals, which loses the blending's dependence on the one parameter, and read
  in floats; over many spans the hulls of its poles and their differences (`Net::hulled`); a long
  curve's spans it covers whole read off their Bézier forms blossomed once for the curve and kept
  (`enclose::span_forms`, the same enclosure,
  `enclosures::a_long_curves_kept_span_forms_enclose_as_its_spans_blossomed`); a stretch a rounding
  past a curve's domain read as the curve is there, clamped, its derivative with nought; a
  rational one, or one of a higher degree, by de Boor's algorithm in intervals span by span, the
  hull over the spans;
  lines, conics, quadrics, the torus, extrusions, revolutions, blends and their carries, a face
  family's runs by their own formulas in intervals; affine images through the map; turned and slid
  contacts by their own jets; a plane's cut of a screwed sheet by `−α/β` and, beside a ruling the
  plane holds, by divided differences enclosed by the mean value; a trace by its own error bound,
  its points grown by its chords — measured, not proved, and said so (`Patch::proved`). Each is held
  to samples of itself over random boxes
  (`enclosures::every_kind_of_curve_is_held_by_its_enclosure`, `…::every_kind_of_surface_is_held_by_its_enclosure`).
  No reading depends on a bound written by hand for a pair of types: `Net::second_bounds`,
  `blend_bounds`, `Heights::bounds` and the amplitude, screw and helix bounds are gone, and every
  closed form's search reads its function written once over `Num` (`isolate::Formula`), its value
  and rate enclosed by the same formula in intervals.
- **An end of a stretch is on a net only near one of its spans** (`query::near_net`): `settled`
  reads whether each end of the stretch stands on the surface, and a net's distance is a foot
  searched for — from afar, at every sheet a ray passed, by Newton from four spans and the grid —
  so an end standing farther than the tolerance from every span's box (`net_index`) is off it
  unread; likewise `along` reads a curve lying in a net only where its start, middle and end are
  each by a span. A ray placing a bore's points in a hypoid pinion's part read every tooth's sheet
  so: the cut of the bore 1.8 s, now 0.19 s.
- **How a pair is read** (`query::Reader`): `General`, **the general reading every pair has** — a
  surface with an algebraic form (a plane, a quadric, a torus) along the curve, one unknown, `F(C(t))`
  and `∇F · C'` enclosed from the curve's enclosure; an extrusion and a revolution, two unknowns,
  the curve against the profile seen along the way or against the meridian in `(ρ², z)` (no
  singularity on the axis, and a line along an extrusion's way no singular system); any other as
  `C(t) = S(u, v)`, three unknowns over the curve's stretch and the parameters the surface reaches,
  a line with no end clipped to a bounded surface's own box. A non-rational net is read by its spans' restricted Bézier
  patches and its boxes halved by smear (above): the hypoid pinion's side, a spline edge a few
  spans long hugging its cut's sheet of 240 × 66 poles, read past any search's boxes from #814 until
  both (`enclosures::a_spline_edge_hugging_a_fine_net_is_found_where_it_crosses`, its roots held to
  ones found apart from the kernel; the pinion's sheet is laid on fewer rows since #904 A4,
  `brep_export::the_pinion_as_written_is_built_by_sectors`); a line is read span by span, clipped to each span's box grown by its
  size (`net_index`); any other curve over the whole net, and only where that reads past any
  search's boxes (`GaveUp::Most`) over the hull of the spans its pieces reach (`query::reached`:
  halved about a span across, a piece reaching no span's box dropped). The whole first: a curve
  along a sheet across many spans is one run proved over the whole, which a box cutting the run
  proves less readily or not at all; the hull for a circle along a quintic tube's rim, which the
  whole net read by restricted Bézier patches reads past them
  (`swept_patches::a_bull_nose_along_a_spline_frame_closes_into_the_volume_its_faces_advance_through`) — and `Fast` (what `curve_surface` reads by), **the closed forms kept
  as fast paths**, each held to the general reading: a line against a plane, a cone, a torus and a
  conic turned as polynomials along it (`query::line_revolution`, a quartic in the meridian's own
  parameter), against a turned contact's meridian (#780 C3, `query::line_turned`) and the cylinder a
  slid torus's contact sweeps (#805, `query::line_slid`) through their jets in intervals, a cylinder
  and a ball as a chord, a level circle's screwed sheet on one unknown, a line along a screw's axis
  along the helices; a circle or an ellipse against an algebraic surface as the form along it, a
  trigonometric polynomial (`query::conic_algebraic`), and against a conic turned as the turned
  surface's algebraic form, span by span (`query::conic_revolution`: the arc's `F(r, z)` from its
  Bézier poles' barycentric coordinates, `F(ρ, z) F(−ρ, z)` — `F` alone where the arc is symmetric
  about the axis, so a contact on it keeps its order — a trigonometric polynomial of degree four
  along the curve, a run within the tolerance proved by its Taylor model about each stretch's
  middle, where the polynomial's terms cancel and its interval enclosure does not; a root kept on
  the bounded surface): the ball's meridian in a paraboloid cup that fits its bottom, within the
  fourth power of the way from the pole it ends at, which the general reading's two unknowns saw
  as a patch nearly coinciding and refused (brep-bench's `tangent/ball-in-a-paraboloid-cup`,
  `brep::a_ball_in_a_paraboloid_cup_that_fits_its_bottom`); a B-spline against a plane as its poles'
  heights (`query::spline_plane`) (`enclosures::the_general_reading_holds_every_closed_form`,
  `swept_closed::a_rims_sheet_in_closed_form_is_held_to_the_general_search`,
  `swept_turned::a_line_meets_a_turned_contacts_surface_where_sampling_finds_it`,
  `swept_slid::a_line_meets_a_slid_contours_cylinder_where_a_scan_finds_it`,
  `swept_closed::an_edge_is_read_certified_and_settled_as_a_ray_is`,
  `…::a_conic_and_a_spline_edge_are_read_certified`).

**A solid is the same wherever it stands** (#1002): a step of every input is to decide nothing
the kernel reads but the rounding of its coordinates. What is near, on or one is a part of the
size of what is read, and what is enclosed is read from a point of its own — never a part of a
point's distance from the origin. Three readings were, and a part built at the origin was refused,
or built hundreds of times slower, a few of its own sizes from it.
**Near is a part of what is searched** (`geom::NEAR`, a thousandth): a patch whose foot is
searched for — a face family's wall (`family::Sheet::inverse`), a blend (`Blend::inverse`), a
B-spline sheet and its offset (`net_search`, `offset_foot`) — keeps the foot Newton's method
settles on from the last one found where it stands within `NEAR` of the patch's own size of the
point (a layered wall's within `HALF_READ` of it), and searches the patch otherwise; a foot
refined from a hint (`Surface::inverse_near`) is on its patch within a millionth of that size, and
two of a blend's sections are one within the tolerance of theirs (`geom::tolerance`). The size is the patch's: a
wall's the box its pieces' boxes fill (`Sheet::size`), a blend's the box of its two sections (not
of its sweep: a coil comes back within its pitch of itself however long it runs) — and its foot
near by no more than the rise a turn of a screw carrying it (`Blend::reach`, #1004: a section
longer than its lead comes back within the lead of itself, and twists within it about the axis;
the screw's domain in brep-bench's fine thread with sharp crests, its edges 116 long screwed
thirty turns at a lead of 1, kept a stationary foot a tenth of the lead off a point of its own
rim, and the thread was refused) — a sheet's its
poles' box, a swept or turned curve's its own (the last two read once and kept by address,
`geom`'s `kept_size`). Read as a part of `1 + |p|`, the point's distance from the origin, the
reach was a millimetre for a wall a metre out and six for a blade 6.3 m aft — wider than its
edge is thick — so a point lying on the wall kept the last foot's wherever Newton's method stopped
within that of it (a rim, the blade's other side), the cubic through the blade's meeting with its
hub's cylinder never closed in on the trace, and the union built at the origin was refused from a
metre out, differently at each distance ("a shell of the arrangement is kept on one side of a
cylinder and dropped on another", "a plane's pieces do not close", "a cylinder's pieces do not
close"); and a part a hundredth of a millimetre across at the origin read a tenth of itself as
near. **A rational spline is enclosed from a point among its poles** (`enclose`'s `lift`): its
quotient's rate `(A' − P W') / W` cancels only as far as its terms are enclosed, and with each
pole read from the origin they are as wide as the spline stands far from it — a rational net ten
of its sizes out had rates enclosed ten times too wide, no box of a ray's reading against it was
proved, every direction placing a point against its solid was given up, and a NURBS ball there
could not be cut by a plane ("a piece of a plane touches the other solid's boundary … along no
face of either", after three minutes where the cut at the origin takes a tenth of a second). The
poles reaching the box are read from the middle of their hull (a family's run's from its middle
pole), the point carried back by the quotient itself (`quotient`); a curve's, a net's and a
family's run alike. **A blend's point is enclosed from
a point between its sections**: `(1 − v) A + v B` over a stretch of `v` is as wide as `A` and `B`
stand far from where they are read from; from the middle of their hull it is as wide as they stand
apart (a canal ten thousand sizes out was cut 700 times slower, and refused ten times farther).
A bar that says rounding reads the coordinates' size, as rounding does (`Traced::project`'s
`settled`, four units in the last place of the point): no tolerance stands near it until a part
is a million of its sizes out. **Not yet read so**: the bars `1e-9 (1 + |p|)` by which `query`,
`ssi`, `topo`, `fillet` and the swept admission take two points for one or a point for on a
surface, and `step_read`'s own — each the Boolean's tolerance for a solid standing its own size
from the origin and wider the farther it stands (some forty tolerances for the issue's blade) —
and the `1e-12 (1 + |p|)` a sheet's foot is settled within (`net_newton`, `continued_foot`,
`net_settle`), which passes the tolerance a thousand sizes out: a hundred thousand out a rational
net's feet stand a hundred tolerances off, and by its poles more. None decided a cut the gates
below read, to a hundred thousand sizes out. Gates,
`tests/far_from_origin.rs`:
`the_issues_blade_unites_with_its_hub_wherever_it_is_drawn` (the issue's document at the origin
and 6.318 m out: seven faces, fifteen edges, one volume within the tolerance's band),
`the_issues_blade_and_its_hub_moved_unite_as_at_the_origin` (its two solids moved 1, 2 and 4 m,
where it was refused a different way at each, united by the kernel alone to the origin's solid),
`a_cut_far_from_the_origin_is_the_cut_at_it` (every kind of solid — `affine`'s, a rational net
closed onto its poles, the blade — cut 10, 1000 and 100 000 sizes out to the origin's volume),
`a_point_of_a_patch_is_its_own_foot_wherever_the_patch_stands` (points of every searched patch,
on it and a millionth off, asked in no order, where it stands and a thousand sizes out, each its
own foot within the tolerance and a sheet's `1e-12 |p|`) and
`a_surface_is_enclosed_as_closely_far_from_the_origin_as_at_it` (`enclosures`' surfaces of every
kind over the same boxes, moved 30 000 out: every sample held, no enclosure wider but by the
coordinates' rounding). #1004's, `tests/swept_closed.rs`:
`a_point_of_a_long_section_screwed_a_fine_lead_is_its_own_foot` (a line 116 long screwed thirty
turns at a lead of 1, twenty thousand of its points asked in no order, a third on each rim, each
its own foot within the tolerance — and a point of its first rim, whose level start
(`Blend::from_level`) is on it, kept that foot though Newton's method reads it a unit in the last
place farther, not searched onto the next turn), `a_thread_with_sharp_crests_is_cut` (the bench's
thread a unit long, a twentieth of its volume within ten tolerances) and, slow tier,
`the_benchs_thread_with_sharp_crests_is_cut` (the bench's case itself, 20 s).

**A B-spline sheet's foot held at its patch's rim is let go once the descent pulls it back in**
(`geom::net_newton`): its other parameter settled alone, the foot was returned on the rim a long
way from a point of the patch, and the point read outside its face. **An edge a Boolean meets with
a surface and a ray a point is located by are read alike** (#780 R2, `query::curve_surface`; there
is no `ray` switch). **Whatever any reader returns is settled by one rule** (`query::settled`): a
curve lying in the surface (read at its quarters, then at steps a fraction of the surface's
feature apart) is `Along` before any reader is asked; a root past either end by more than the
tolerance is no meeting, one within it is at the end; a run of roots the curve stays within the
tolerance of the surface between is one meeting — a crossing where the surface's side changes
across it, read where the curve has left the tolerance, a touch where it does not — one, standing
where the surface's distance turns along the run as far as the curve keeps within the tolerance (its
rate read at those ends, past rounding's: counted inside the run, rounding's flips read two touches
on one machine and one on another, and a turned ring built three times its volume on the first),
at its middle where it does not turn — (a
double root rounding parts, a root two spans or stretches both find, and the noise a polynomial's
rounding makes where a line runs along a torus's tube top, its distance the fourth power of the
way: hundreds of crossings a micrometre apart before); and an end on the surface is a crossing
there, taking the run the curve keeps within the tolerance beside it — not only the roots within
the tolerance of it along the curve: a touch rounding places a little way in from an end on the
surface, kept beside the end's crossing, left a plane's pieces open
(`round_lowering::rounds_that_run_out_onto_a_face_are_rolled_and_cut_off`,
`rounds_examples::the_runouts_are_the_stacks`). An edge clipped to a face's box
(`query::curve_surface_within`) is read stretch by stretch — a stretch kept where the curve's
enclosure reaches the box, never by samples it can leave the box between — and settled once over
the whole, a stretch's end cut at the box no end of the edge; a stretch refused refuses the whole.
That edges within the tolerance of tangent were read one way by the polynomials and another by
samples is why a Boolean's edges once sampled the cone, the torus and the sheets
(`round_lowering::a_round_across_seams_between_arcs_on_a_drafted_prism_is_glued`,
`prism_twists::a_cap_of_a_tool_no_lone_prism_turned_is_the_closed_classs`, both building now).
**A point
where three surfaces' meetings meet is solved once** (#711, #777 M2:
`Arrangement::triple_points`, keyed by the three surfaces' indices among the shared meetings), and
every later finding of it from another of the three faces takes that vertex. Each two meetings'
crossings on a face are sought on every core, as phase 1 reads every edge against every face —
each reading the curves and surfaces alone, which nothing it finds changes — and taken in the
pairs' order, as read one after another: the same vertices, cut in the same order.

**Touch curves are found generally, and shallow crossings built, not refused** (#778, #777 M3).
A meeting's critical set is read in one surface's parameters, over the one whose partner's
distance is exact (`meeting::exact_distance`; a sheet's is a numerical foot, too rough to
difference twice): `f = b ∘ a`'s critical points by minimum-norm Newton steps — the Hessian
inverted on its well-bent ways alone — each classified by the Hessian's rank (`critical_set`):
rank 2 an isolated point (`ssi::touch`'s), rank 1 a point of a critical curve, rank 0 refused,
typed, naming the surfaces and the point (#717). A point where `f` still slopes along the null way
is a fold, no critical point. From each rank-1 point whose fold stands within reach of nought the
**ridge** `∂f/∂n = 0` is traced along the null way to the box's edge or round to its start
(`ridges`); where its fold is within `tol` of nought it is a touch (`Contact::Touching`), and
where the fold has the sign opposite `f`'s curvature across it the two crossings beside it are the
roots of `f` along `n` at each of its points, isolated from bounds measured off `f`'s own samples
(`ridge_branches`, #777 M1, read by `isolate::Bounded`: a surface's signed distance through a
numerical foot is no curve's reading and no interval encloses it) —
however shallow, the fold solved in its two-root form. **A crossing with no ridge, too shallow for
the march along `∇a × ∇b`, is marched in both surfaces' parameters at once** (`parametric`, #942
F1): the solutions of `A(u_a, v_a) = B(u_b, v_b)` followed along the null vector of their 3 × 4
Jacobian, each point settled by minimum-norm Newton — no foot read on either surface, so it holds
at a pinch of two fitted sheets and where one surface's parameters degenerate, held to within a
quarter of `tol` by the cubic through its points (a polyline where none follows), bounded by both
surfaces' boxes of parameters and closed where it comes back to its start. It replaced the level
curves `f = 0` in one surface's parameters, which strayed from the meeting by 1e-4 at the 75°
cube's 0.02° pinch, read a curve of the other wall's extension past its patch, and closed on
themselves round each seed near a ball's pole; across the suite they read no branch the march
does not. Gate: a plane crossing a cylinder and a ball, each of radius 100, at 0.02°, the closed
forms bypassed, met in the closed form's two lines and its circle to 1e-9
(`meeting::two_surfaces_crossing_at_a_fiftieth_of_a_degree_are_met_in_their_closed_form`). A
point is on such a branch within `8 tol / sin θ` (`on_a_branch`, A3's reach). **A meeting's analysis is done
once** (#780 R4): `meet_with` finds each side's critical set once and reads it for the singular
points, the ridges (`ridges_from`) and the seeds alike; the branches a point is asked about are
sampled once into chords filed by their boxes (`meeting::Sampled`), a point held only to the
chords within reach of it and no foot sought beyond reach of a branch's poles. Where the profile is
printed it counts each (`util::meter::analysis`: critical sets and their jets, branch tests, samples
and feet). `meeting::meet` reads the ridges
on its traced road before it marches; an arrangement reads them where its march is refused as nearly
touching (`Arrangement::traced_meeting`). Both go through **one fallback** (`meeting::marched`, #780
R3): the march, and where it is refused as shallow the ridges (where not read yet) and then each
seed no branch holds marched in both surfaces' parameters, each touch a stretch the march ends at,
each seed then traced alone and one refused again passed by (`traced_alone`); refused still where a
seed the march does not settle stands on the meeting, held by no branch. **The general road and
the trim go through it too** (`meeting::through`, the meeting through given seeds, no ridges read:
`swept::general::beside_rails`, and the trim's completion from a witness), where they called the
trace and a march of their own beside it. **The trace's refusal below
`SHALLOW` is no longer the kernel's answer**: it is where the meeting is read another way, known by
its type (`ssi::Shallow`: the kinds, the angle, the point), never by its words
(`touch_curves::a_march_refused_as_shallow_says_so_by_its_type`). The closed forms that know a touch
— `plane_cylinder`'s, a cone's generator, `parallel_touch`, `ball_in_tube`, `tube_along_torus`,
`tori_touching`, `pipe_end`, `screwed_circle_round`, and `extruded`'s plane through a spline
extrusion — are fast paths, each held to the general method with them bypassed
(`meeting::meet_with`, an argument, not a switch): within 1e-5 (2e-5 on a sheet's rim), a touch
being a double root placed to its differences' rounding, and each three to five orders faster
(`touch_curves::each_closed_form_for_a_touch_is_held_to_the_general_method`; every kind touching
along a curve and crossing at under a degree met completely with and without them,
`meeting::every_kind_touching_along_a_curve_and_crossing_shallowly_is_met_completely`). Held on
brep-bench's vault, as a prism of its arc and as its rational sheet, to 1e-9 of the closed forms:
cut at its crown and from a billionth to a tenth below it, and by a plane through the crown leaning
0.001°, 0.1° and 1° (`touch_curves::a_vault_cut_*`), through `combined`, `kept` and `split` alike,
six faces at the crown, eight below, seven leaning
(`touch_curves::the_boolean_reads_touches_and_shallow_crossings_through_each_road`); and a fitted
sheet touching a plane along `y = 0.2 sin x`, no closed form's, kept whole below it
(`touch_curves::a_fitted_sheet_touching_a_plane_along_a_wavy_curve_keeps_whole_below_it`).
A surface offset by a distance is a surface (a cone shrunk past its apex the cone of its own
apex), or what it closes onto where the distance takes its radius to nothing — a cylinder's axis, a
sphere's centre, a torus's middle circle — and nothing only past that (`Surface::offset`,
`geom::Offset`; `brep::a_surface_offset_moves_its_signed_distance_by_the_offset`); whether a kind
has an offset in closed form at all is its own question (`Surface::has_offset`). A caller that
wants a surface says so (`Offset::surface`, or `Surface::offset_surface`, which refuses a closing
offset as that), and none reads a collapse as no offset at all (a ball as wide as its cylinder,
[rounds.md](rounds.md)).
**The surface moved along its normal with its parameters kept** (`Surface::normal_moved`) is
what a face carried along its normal lies on (`offset::carried_sweep`): its point at `(u, v)` is
`S(u, v) + d N(u, v)` exactly, which `offset` and `parallel` do not promise — a cone's offset keeps
its frame and shifts `v`, so the moved cone is stood `d sin α` back along its axis with its radius
there grown by `d cos α` (less than nothing where its origin falls past the moved apex, which STEP
places at its apex, `step::cone_written`), and an ellipse swept along a line is read over its
rational spline's parameter and has none (`brep_offset::a_surface_moved_along_its_normal_keeps_its_parameters`).
**A sheet's offset is read by its signed distance** (`Surface::parallel`, `Surface::Offset`): a
B-spline sheet has no offset in closed form, and a fitted net would be an approximation, so the
surface standing `d` off it is the level `d` of its signed distance, `S + d N` over the sheet's own
parameters (`Surface::Offset` holds the sheet's frame and net, and `d`) — its point and partials
from the sheet's second partials by Weingarten's equations, its distance the sheet's less `d`, its
gradient the sheet's normal, a point's foot on it the foot on the sheet, warm from the last foot
read where that settles `d` off (a point on an offset stands too far off the sheet for the sheet's
own warm start, and every reading searched the whole sheet). It is a face's where a file gives one
(`OFFSET_SURFACE`, written so again) and where a sheet's face is moved along its normal (a sheet
thickened, `offset::carried_sweep`); the sweep's dump refuses it. Past the sheet's
domain it is read on the sheet **carried on by its own polynomials** (`Net::d2_continued`, the end
spans' polynomials past the knot they end at, and `Surface::continued_inverse`, Newton's foot with
no domain to hold it), where the sheet's own signed distance runs on along the tangent extension:
there the curvature is switched off at the edge, and a ball's centre rolling on past a face's
edge on it turned its curvature from 25 to 9 at once, which no cubic net fits (the canal missed
its bar by 1.7e-4). `Surface::parallel` is `offset`, in closed form, else a sheet's offset (an
offset's offset by the sum), and none for the kinds whose second partials are not read (any other
curve swept, a curve turned, a blend, a family's wall, an image).
**A spline or an ellipse swept along a line is the sheet it is** (`Surface::swept_net`): its
spline's poles (an ellipse's rational one) carried each way along the sweep, one straight span in
`v`, exact — so its offset is a sheet's, read by its signed distance as above, and a ball rolls on
it and a chamfer's leg is read from it. The span is as long each way as the curve is wide, and a
foot past it is read on the line it runs on: **a net that is one straight span along a parameter
carries on exactly along it** (degree 1, two poles, each pair weighed alike), so
`continued_foot` holds a foot to the domain's width past it along every other way only.
`sheet_rounds::a_sheets_offset_stands_its_distance_off_along_its_normal` is the gate, and
`brep::an_ellipses_foot_is_its_nearest_point` the swept ellipse's (its offset 2 in stands its
distance off with its foot at the point's own height, 3000 up the sweep and 500 below).
**A ball rolls on a sheet** (`fillet::roll`, brep-bench's fillet/bezier-dome-one-edge): its centre
runs on the faces' offsets read so, traced only round the run — the run's box grown by `r + 2r /
sin(θ/2)`, as far as a ball rolling on past an end needs, its section's corner within `r /
sin(θ/2)` of its centre: the whole body grown by its size, a sheet's continuation wandered off
under the solid and the trace took 23,000 points and four seconds to fit. Its contacts are read
along the offsets' gradients (the faces' own where they have one in closed form, the continued
sheet's past a sheet's edge), and a run that ends lays each sheet's strip on the sheet extended
as far as the strip reaches (`Net::extended`, exactly the sheet over its domain), the faces' meeting
past the ends read on those, not on a net held at its edge. **A sheet is the sheet of one net**
(`ssi::same`): two nets of the same degrees, knots and weights whose poles stand within `tol` are
one surface, and so are two whose nets over the domain they share are (`Net::segment`, after nine
of its points agree) — a net and itself carried on, so the Boolean reads a strip and the face it
lies on as coincident. A reparameterised net is not recognised. **An edge lying along a face
crosses its edges where their enclosures say** (`query::enclosed_crossings`, which the Boolean
reads where `query::crossings_in` has no closed form — `crossings_in` reads one curve against a
surface carrying the other: a line's plane square to the face, an ellipse's plane, and a circle's
plane — or its cylinder where the plane's reading is refused (#942 F3: a bull-nose's round touches
its flat end's plane all along the circle they share, so a curve on the round reaching that circle
touches the plane, a double root no search proves, and crosses the cylinder square; the plane first,
since a helix about the circle's axis lies on its cylinder and crosses its plane;
`solid_sweeps_examples::the_bull_nose_swung_partway_its_meridian_beside_its_foot_stops_inside_the_ring`)
— its meetings' crossings still searched by
`meeting::crossings` within their reach): where neither is a line or a conic, pairs of their
stretches whose enclosures (`Enclosed`, a spline's poles
restricted to the stretch; `Curve::bounds`, a span's whole hull, never shrank and the search gave
up) stand apart by more than `8 tol` are dropped, the rest halved until each is a hundredth of its
curve's box, and from each pair left Gauss–Newton on `a(s) − b(t)` settles the crossing; none
where 4096 pairs read or 256 left, or where a root is shallower than `ssi::SHALLOW` — one curve
running along the other (a face's edge and a round's cut of the same sheet at the same cap, where
Newton's method settles anywhere, and `meeting::crossings`, which `boolean::along_one_another`
guards only where they run along each other whole, read a crossing every few hundredths) or
touching it — which leaves the edge unsplit as before. So a
flat root finds its crossing (`Arrangement::flat_roots`): a fillet's canal cut off at a cap touches
the side its contact lies along, and the solid's edge where that side meets the cap touches the
canal at a root placed only by the fit's tangent, 1.8e-6 along the edge from where the contact,
lying along the side, crosses that edge; read, that crossing is the root, and the arrangement's
pieces close round it, where unread they did not ("a plane's pieces do not close"). The round of
brep-bench's case is held to its volume integrated from the patch alone, section by section along
x, the ball's arc in each from its centre 1 under the patch on the plane y = 1, to 1e-7 (it agrees
to 1.2e-8), in 0.3 s: `sheet_rounds::the_bezier_dome_block_rounded_along_one_top_edge`, with
`a_sheet_carried_on_is_the_same_sheet` and `splines_lying_in_a_face_cross_where_they_cross`.
A minimum is a `util::roots::Least`, its place and its value named, never a pair.
**A flat between two planes is a plane** (`fillet::flat_along`): the prism on its section's
triangle, carried on past each end and cut off there, no net fitted. A fitted flat is a B-spline
face, and a run chamfered after it that ends on it was refused, nothing cutting a run off at a
spline: a box's top edges chamfered one after another, each cut off by the plane of the one before
(`brep::a_box_top_chamfered_unequally_is_ten_planes`, brep-bench's chamfer/box-top-asymmetric;
`brep::chamfers_between_planes_are_planes`). **So a round between two planes is a cylinder**
(`fillet::roll`): its spine a line and every section the same, the piece is the prism on the
section (`fillet::round_section`: the corner, its legs to the contacts and the ball's arc, framed
about the run), its spine and contacts the lines the ball's centre and touches run along. A fitted
canal there was a net of 65 × 129 poles the Boolean after it cut: brep-bench's fillet/box-one-edge
took 120 ms and wrote 835 KB of STEP, and fillet/notched-cube-inside-corner 1.8 s
(`brep::rounds_between_planes_are_cylinders`: convex, concave, and runs whose spines go either way
along their edges, against their closed forms). Both prisms are cut off at each end as a run is
(`fillet::carried`), and an end cut by one plane square to the run is where the prism ends,
nothing intersected. A run going on tangent into a curved face at an end is still fitted, as the
piece that takes on from its section is. About one axis the flat is turned: between a plane square
to it, cylinders and cones about it, a closed loop's flat is the section's triangle revolved, a cone
exact (`fillet::coaxial`; a leg shrunk to a point its apex, [rounds.md](rounds.md)). So is a
round there (`fillet::roll`, `fillet::revolved`): the section — the corner, its legs to the
contacts and the ball's arc — in the meridian through a vertex of the loop, revolved, a torus face
and its strips exact, no canal fitted. A fitted canal round a circle was a net of hundreds of poles
that every later Boolean cut and every measure integrated: brep-bench's fillet/boss-s-curve took
8.7 s, most of it its area, and now takes milliseconds, its volume and area to 1e-15
(`brep::a_bosss_rim_and_root_rounded_to_meet_are_two_tori`; the hole's waist too).

**How nearly flat is too flat is the ball's to say** (`fillet::flat_enough`, brep-bench's
fillet/nearly-flat-ridge): two faces turning through `τ` meet at a ball's centre read within `tol /
sin τ`, and a ball of radius `r` stands its round `r (1 / sin(θ/2) − 1)` off their edge; the round is
refused as too nearly flat only where that is no more than a hundred times the first. Faces turning
less than a degree were refused whatever the ball: a ridge of 179° rounded 180 stands 0.0069 off its
edge, its centre read within 6e-7, and is rounded as any other; a crease of a ten-millionth of a
degree rounded 1 is not (`brep::a_nearly_flat_ridge_is_rounded_by_a_ball_wide_enough`). A flat
between faces turning less than a degree is still refused, none being asked for.

**Rounds together lose what they close to nothing** (`rounds::rounded`, brep-bench's
fillet/plate-full-round): where a round's two corners' balls are one, the round has no length — it
vanishes, its arc at either end one edge, its faces' loops going straight on past it — and a face
whose loop closes on itself with no area, two contact lines between the same two vertices, is gone,
the two rounds either side of it meeting along that line. A plate rounded to half its thickness is
its sides' half cylinders, each in two faces, and its corners' balls, each in two, `4896 + 558π` to
1e-16; a face rounded away to nothing, no loop left (a cube rounded to half its side), is refused by
name (`rounds_together::a_plate_rounded_to_half_its_thickness_loses_its_sides`). **A face the round
swallows** (`rounds::swallowed`, fillet/round-swallowing-a-tiny-face): where the edges cannot be
rounded as they stand and a face lies between two of them that stand nearer each other than the
ball is wide (`rounds::narrow_face`), the face is deleted
(`defeature::deleted`), the edge where the faces beside it now meet rounded in place of its two, and
the answer kept only where every point read along the face's edges stands clear of the result's
boundary, all on one side — a corner broken by a flat 0.0001 wide, rounded 1, is the corner rounded
(`rounds_together::a_flat_thinner_than_the_ball_is_swallowed_by_its_round`; a flat wide enough to
hold the ball keeps its rounds and itself). **A cell is a cell by its own measure**
(`complex::cells_found`): a shell holds a volume past `1e-12` of the whole's cube, or, under that,
past its own area times the tolerance (its faces' areas read only then); the flat's gap, 2.5e-8 of
a complex 40 across, was read as no cell and the flat as closing no gap.

**A side of faces cut from one sheet** (`fillet::ring`, brep-bench's fillet/nurbs-cylinder-rim): a
face on a B-spline sheet, and every face across an edge from it whose net goes on where its own ends
along `u` (`Net::joined_u`: clamped, one degree and `v` knots, the last row of poles the next's
first, rational weights in one ratio), are one side to the ball, which rolls on the offset of the
nets joined — no face's own net, carried on by its polynomials, is the sheet past its edge. The run
round them is one closed loop; the canal starts where its contact crosses an edge between them, its
contact is cut where it crosses each other one, and the side's strip is one face on each of them,
on that face's own surface, between the run, the contact and the edges up from the run to it
(`fillet::build_ring`), so the Boolean meets each strip on the surface of the face it lies on. A
cylinder whose side is the rational circle carried up in two faces is the analytic cylinder's
rounded rim to 5e-11 (`sheet_rounds::a_rim_on_a_side_cut_from_one_sheet_rolls_round_it`).

**A ball whose radius varies along the edge** (`fillet::roll_varying`, brep-bench's
fillet/variable-radius-*): at each point `p` of a run that ends, the ball of radius `radius(p)`
touching both faces with its centre in the plane square to the edge there — the centre on both
faces' offsets by that radius and in that plane, by Newton's method from `r` off each face — its arc
between its contacts the section. The run is its edges in order from one end to the other, each
end cut off as `roll` cuts one, the ball rolled on past it by the edge's own curve carried on; the
canal and the spine are fitted within `roll`'s bar, and the strips and caps are `roll`'s
(`build_open`). A cube's edge rounded by a radius growing along it, an L's inside edge, a hexagon's
obtuse edge and a half cylinder's rim rounded by a radius growing round it are their closed forms
to 1e-10 (`varying_rounds`). Refused: a radius of no size anywhere along the edge, a run that closes,
a face's offset that is no surface, a face that cannot hold the ball (`holds_ball_by`).
**Edges rounded together, the kernel placing every piece** (`rounds::rounded(b, edges, r, tol)`,
brep-bench's fillet/box-all-edges): a set of edges rounded in one call. Where they are straight,
between planes, and each vertex they reach has three edges, all of them among the set and all
convex or all concave, the solid is built anew by surgery on its boundary — nothing intersected
and nothing fitted. Each edge becomes a cylinder about its spine, the edge moved `r / tan(θ/2)`
along each face (θ the angle the faces span on the ball's side) and `r` off both planes; each face
keeps its loops, each rounded edge's use replaced by its contact line between the touches of the
balls at its ends; each vertex becomes the triangle of the ball touching its three faces, bounded
by the three cylinders' end arcs — each spine runs through that ball's centre, so each run ends in
the plane through it square to its edge, on a great circle (the sphere's frame turned so the
triangle stands about `u = π`, clear of its seam and poles). Every edge is shared by construction
and every pcurve written exactly (a line in a plane's or a cylinder's parameters, a great circle's
ends on the sphere), so the cube of brep-bench's case is built and checked in a fifth of a
millisecond, where rolling each edge and cutting each piece and corner cell took seconds and left
fitted canals. Refused by name: a face no plane carries, an edge no line does, a corner whose
third edge stays sharp or of other than three edges (the mitre two rounds meet in is rung 3), a
corner between convex and concave rounds, a round wider than a face beside it holds (a contact
line run past its corners). A set of which no two edges share a vertex is rolled instead, each
edge in turn (`fillet::roll`, any faces), each piece cut off a convex edge and filled at a concave
one. `rounds_together` is the gate: a box (and one turned off the axes), a triangular prism and a
cavity's twelve concave edges against Steiner's formula — a convex polyhedron rounded all over is
the one its faces moved in by `r` bound, grown by the ball — to 1e-12, and the refusals.
`Brep::bounds` encloses (issue #59; through-prism cutters, tool culls): `Curve::bounds` exact for
lines and conics, the poles reaching a B-spline's range, a trace by its measured sagitta; a face bulges
past its edges only off a `ruled` surface (`Surface::bounds_over`: meridians at each axis's
extreme angle). Never a sample grid: extremes between samples were missed.
**A mesh runs along a surface's rulings** (`brep::mesh`): a face on a surface straight along one
parameter — a swept curve's, a cylinder's or a cone's `v`, a sheet one linear span across — is
triangulated with that parameter's scale shrunk (`STRAIGHT`), so its triangles join neighbouring
rulings across the face and only the curved way is refined; every triangle is still held to the
bar in space. An edge's samples are its chords within the bar at their middles, their quarter
points and every knot they span (one inflection at most a cubic's span), not a cut at every knot:
a fitted curve of hundreds of spans is sampled as finely as it bends. A plane crossing a sheet one
linear span across whose second row is its first moved by one step is met in the first row's
curve carried along the step onto it (`ssi::intersect`), exactly, not traced and fitted.
`swept_speed::a_swept_gears_faces_mesh_along_their_rulings` and
`a_plane_across_a_swept_sheet_meets_it_in_its_curve_carried_there` are the gates.
**A meeting along a sheet's iso line is read over the face's own parameters** (#1013 T1,
`boolean::iso_parameter`). Where two faces meet in a closed-form curve whose parameter is one of
a face's net's own, the curve is cut to the face's parameter box (`Located::grown_parameters`)
before its spans are read. That holds whether the net's other parameter is constant along the
curve (a level iso line) or runs (a row carried onto a plane). The other parameter is found at
each of nine samples by Gauss–Newton along it alone, so a fold the curve crosses reads as well as
any other point. A net may run on past its face and fold back over it, as a swept tube's sheet
does past its cuspidal edge. The curve's stretch on the folded-back layer is then the face's own
points again: read by its middle's nearest foot, it was put on the face and laid a second time.
Only its parameters put it off the face. `patch_layers::a_patch_folded_over_its_rim_is_notched_across_the_rim`
is the gate (brep-bench's `nurbs/folded-patch-notched-at-its-rim`, which also has a control on
the net cut down to the face).
**A line's or an edge's crossings with a sheet's face are searched on the face's own stretch of
the net** (#1013, `Located::read_surface`). For the solid's rays and for the arrangement's edges
against faces, a face whose surface is a B-spline net is read on that net cut down to its parameter
box, grown by a sixteenth each way (`Located::grown_parameters`, the box the rule above cuts to;
`Net::segment`, the same surface at the same parameters, cut once a face, and only for a face a
ray or an edge reaches). No crossing past the face's parameters is a crossing of the face. A net running on
past its face and folded back over it left the interval search nothing to contract at the fold: a
block under a folded patch took 2.2 s a Boolean, against 0.02 s with the net cut down, and now
takes 0.02 s either way. `rust/cad-kernel/examples/patch_fuzz.rs` reads the folded patch at
random (patch, trim, box), every Boolean against its closed form, whole net and cut down.
**A sheet's face is traced and settled on its own stretch of the net** (#1013 T2). A meeting is
traced (`Arrangement::traced_on`) on its surface class's net cut down to the union of its faces'
grown parameter boxes. It is the union, and not each face's box, because a meeting is traced once
for every face of the class, and cut at one face's edge it would stop there and the next face would
never be traced. A class whose faces lie on more than one net is left whole. An edge's crossing is
settled onto the face's own stretch (`on_surface` on `read_surface`), and a span's on-patch test
reads it too. A net folded back past its face, its other layer a few ten-thousandths beneath the
face, had the meeting traced along that layer, the edge's root settled onto it, and the span's
middle inverted onto it and read off the patch. Each was needed alone: without any one, a block
under such a patch pierced by a square hole was refused. `patch_layers::a_patch_over_a_near_layer_is_pierced`
is the gate (brep-bench's `nurbs/patch-over-a-near-layer-pierced`). `patch_fuzz` draws half its
patches so (`c s³`, `c` from 1e-5 to 3e-2): on three seeds `main` fails 669 of 1,800 Booleans, T1
alone 420, with T2 none. The same reading keeps a march clear of a cusp or a pinch of the net more
than a sixteenth of the face's parameters past it.
**A face's parameter box is grown only while its net stays regular** (#1013 T3, T4,
`query::regular_growth`). The box `grown_parameters` returns, which every reading above uses, is
grown a sixteenth each way. On each side the added strip is read on a 4 × 17 grid across the other
parameter's grown range, corners included. That side's growth is halved, at most twelve times,
while anywhere in the strip `S_u × S_v` falls below half its least along the face's own edge, or
turns more than 60° from the normal at the face's edge beside it. A fold flips the normal, and a
pinch `(s³, t³, st)` turns it about the point however narrow the region where `S_u × S_v` is small,
so no sample need come near either. A face's own edge at a pole reads nought there and holds
nothing back. **A spline's foot steps by Gauss–Newton where Newton's curvature is not positive**
(`BSpline::refined`): an iso line beside a fold, `x = s²`, had `|C'|² + (C − p)·C''` negative a step
from its end, and the search stopped there, the foot of the face's rim read 3e-3 off at the end.
The gates are `patch_layers::a_cuspidal_patch_trimmed_nearer_is_sliced_through_its_cusp` and
`a_pinched_patch_trimmed_nearer_is_sliced_beside_its_pinch`, the face within a sixteenth of the
singularity. `patch_fuzz … cusp` and `… pinch` draw faces trimmed as near as `0.505` to the cusp
or the pinch, cut by planes through points within 0.003 (5e-4) of them, each side against the
net cut down: on three seeds, `main` fails 213 of 900 cusp cuts and 555 of 900 pinch cuts, T2's
commit 41 and 7, and this none, the slowest 0.06 s where `main`'s took 5 s.
**A march ends where it can go no farther on a pair of sheets** (#1013, `ssi::trace_landing`).
Five ends, each read where the march stands, none needing the faces:
- **landed back where it stood**: eight steps running that each carry it on less than a
  thousandth of their length (a fitted sheet's foot held, each 1e-4 step landing 1e-8 on);
- **run back onto its own path**: asked every sixteenth step of the path but its last four
  points (`on_path`), a point within a quarter step of it. A meeting crosses itself only where a
  surface is singular or the two touch, so a march come back anywhere but by its start entered
  a loop, and ends where its second lap began (by its start, a closed curve). The crossed tori's
  march round a swallowtail of a swept sheet beside a cap's torus, 2.5e-4 across, lapped for its
  hundred thousand steps: 780 s;
- **across a sheet's fold**: a sheet's normal turned about between two points of a march, which a
  regular march's few degrees a step never do. Past it the meeting runs on the net's other layer,
  which no face on one layer holds; carried on, the crossed tori's meeting beside B's sheet's
  near-singular point turned back along itself and was fitted 7e-4 off both sheets;
- **at a touch, one point**: a march arriving at a touch known within `8 tol` ends at the point it
  is known by (`end_at_touch`), where its other branches leave from; settled again from the
  arriving march, the sheets' touch at the crossed tori's shared pose stood 8.4e-6 off itself,
  two vertices twice the tolerance apart, and the face's cut through it did not close;
- **short of a touch this trace knows, at it**: once every march has run, an open curve whose end
  stopped within a touch's reach and heading for it, a touch this trace leaves branches from, is
  carried on to the point the touch is known by (the fourth branch ended 4.8e-3 short of the touch
  its three others leave). Asked as the march stops, the touch was not yet known; and a touch the
  trace never met is left to the meeting known beside it (`round_onto_round`'s boss, its torus
  touching the plane at the plane's edge, where the march ends at its step).
`touch_near` takes sixteen Newton steps (`TOUCH_STEPS`): a touch it finds it settles in a dozen,
and the crossed tori asked it a hundred times beside their folds, most running all of thirty-two
to nothing. A sheet's grid seeds settle each sign change by false position (Illinois) and drop a
side at once where the distance changes by more than twice its ends stand apart (a distance is
1-Lipschitz, so the foot leapt; halving gave such a side up after forty readings).
**An edge's gap is measured as a reader measures it** (`Brep::measure_tolerances`, read as
`standing`: 64 places, the knots and the ends, and halfway between each two). Read at 33, the
crossed reverse sheet stated a rim's gap of 1.1e-5 where its pcurve stood 1.5e-5 off between two of
them, and a piece of it in the union failed `check`. The gate is
`swept_tube::the_crossed_members_combine` (slow tier): brep-bench's crossed members combined as its
runner asks, at their own tolerance, union, difference and intersection each `check`ed, meshed
within a thousandth of its volume, and the three held to `|A ∪ B| + |A ∩ B| = |A| + |B|` and
`|A − B| = |A| − |A ∩ B|` to 1e-7 (they agree to 3e-9). At the kernel's own tolerance, a billionth
of the size, five hundred times finer than the sheets are fitted, they are refused: the runner
asks at the sweep's (brep-bench's `Tumble::tolerance`).
**A spline extruded meets a surface the same all along it in closed form** (#644,
`ssi::extruded`): a plane along the extrusion or a cylinder about a line along it in the lines
along it through where the spline crosses or touches that surface, exactly; a plane across it (square
to it, or at a slant meeting each line along it at `ssi::SHALLOW` or more) in the spline carried
along it onto the plane, its poles carried, exactly (and a line's in a line); a cone about such a
line in the spline carried along it onto the cone, a quintic over the spline's own parameter on
its knots each twice more (`nurbs::interpolate_on`, so the spline itself is held exactly and only
the height is fitted), a span halved where it strays past a quarter of the tolerance. Only where
every meeting is steeper than `ssi::SHALLOW` and no two crossings may hide between the samples
crossings are read at (the distance turning there within the tolerance of zero or past it): a
touch or a crossing nearly one (a tool's side in the pocket it cut, along the pocket's wall) is
traced, where touches are read. Traced, a
straight meeting took the step the spline's sharpest turn anywhere allows — a tooth space's prism
met its tip in eight thousand steps fitted through a thousand, and a 16-tooth pinion's STEP grew
sixfold. `brep::an_extruded_spline_meets_surfaces_along_it_in_closed_form` and
`planar_subtraction::a_flanks_prism_meets_the_tip_and_the_chamfers_in_closed_form` are the gates.
**Four curves closing a loop are filled by their bilinearly blended Coons patch, exactly a net**
(`nurbs::coons`; brep-bench's fill/coons-patch-of-four-parabolas): opposite curves made one space
(the lower raised, `BSpline::elevated` — Bézier pieces raised and each inner knot taken out again,
so each knot is once more and the curve as continuous as it was, never a kink `Net::kinks` would
read — and each given the other's knots), each ruled blend a net whose linear direction's poles are
at the other's Greville abscissae, the patch the sum less the corners' bilinear net, its boundary
rows the curves' own poles bit for bit. Each curve's parameter is carried onto `[0, 1]`; no fill by
another rule (an energy, a Hermite blend) is made under this name. Rational curves and an open loop
are refused. `coons::the_block_under_the_coons_patch_of_four_parabolas` is the gate.
**A B-spline curve's foot** (`BSpline::nearest`) is the nearest of seventeen samples a knot span,
read off the span's Bézier form, the spans nearest box first and none past the nearest sample
found, refined by Newton; each thread holds the last few feet by curve and point (a trace asks a
swept surface's distance and gradient at one point), the answer it would find again. A spline swept
or turned is inverted near a hint (a pcurve's) by Newton on its curve where that lands on the
surface, and globally otherwise, as a sheet is.
**The surface arrangement** (#735): any set of surfaces divided into one consistent cell complex, a
Boolean, a split and the closed swept solid each a selection over it. **A1, the complex**
(`brep::complex::Complex`): every face once, each parting two cells, an edge bounding any number of
faces taken in order round it — by the way into each at the edge's middle, and where two ways agree
to `1e-6` (faces tangent along it) by where each stands a step into the face, its curvature's
difference; an edge of one or two uses is ordered by its uses' senses alone, no geometry read. Its
shells are the classes of face sides joined round edges; a shell bounding positive volume is a
bounded cell's outer one, any other a void of the smallest such holding a point of it, or the
unbounded cell's (found only when asked, `cells`). A touch along an edge (four faces round it) or at
a point is a state it holds, not a check failure; `select` keeps the faces between kept and unkept
cells, each wedge of a kept cell round an edge where the result touches itself given an edge of its
own, and says where (`Touch`). `complex::two_blocks_meeting_along_an_edge_…`, `…_at_a_corner_…`,
`a_void_is_a_shell_…`, `faces_tangent_along_an_edge_…`, `a_ball_in_a_cylinder_of_its_radius_…` and
`a_block_tangent_to_a_cone_along_a_generator_…` are the gates. Ways into faces from an edge within
`TANGENT` (1e-4 rad) of one another are ordered a step in: a fitted surface — a round's net — is
tangent to the faces it joins only as closely as its fit, some 1e-5, and read to first order that
error put a round's strip on the wrong side of it, every cell one shell (the round-lowering cases,
each a Boolean's selection with none falling back). `TANGENT` decides two fitted surfaces' ways
alone, and caps the rest (`complex::tie_between`): where either surface is exact, the tie is what
an edge's point read off its face within `8 tol` turns that exact surface's normal by, twice,
`16 tol / ρ` at its least radius (the smaller of two) — so an exact face and a fitted one leaving
an edge a millionth apart are ordered as they leave it, not on the fitted tie (#942, the ring's
ends kept: with it, and with `TANGENT` itself at 1e-12, the fast suite and the slow tiers of the
arrangement, the kernel and the sweeps stand; `SOLVENT_BREP_DEBUG` names each pair the fitted tie
alone would still tie). The step is read nearest first (#942 F6):
sixty-four tolerances in, and ten times farther only while two ways still agree within what a
tolerance's lean turns a way by there (`4 tol / step`), to at most a hundredth of the least exact
radius among them or a thousandth of the complex. Read at that farthest step alone, the order was the
box's: a drum screwed a short lead has rims' ramps thinner than a hundredth folding against the
cylinders its side sweeps, a step of a thousandth of a box past them swapped a ramp with its
cylinder, and as the box grew — a twentieth more room, or a slab a hundred times the sweep cut —
the solid was judged wrong or the slab left whole
(`swept_closed::a_drum_screwed_a_short_lead_cut_from_a_slab_a_hundred_times_its_sweep_takes_its_sweep`).
**The step is read from the face's own point at the edge, not the edge's** (#994): an edge lies
within the reach of its faces (`ON_CURVE` tolerances), not on them — a meeting laid on a class of
faces one surface lies on the class's surface — and a step sixty-four tolerances long read from a
point standing eight off the face turns its way by an eighth of a radian: a boss a micron wider
than the lobe of a plate it stood on, its wall one surface with the lobe's, was read 0.12 rad off
its way where it touched the blend beside the lobe, ordered across it, and two cells joined
(`brep_lobes::a_boss_or_a_bore_on_a_lobe_of_its_own_radius_builds`: radii `1e-8` to `1e-4` off,
the axis within the reach of the lobe's). **Not yet**: a boss of the lobe's radius whose axis
stands `1e-6` to `1e-5` off the lobe's is refused.
**Two ways in that a step in turns the other way round are one way where they stood apart by no
more than the tolerance** (#942, `complex::turned_back`): `g` apart at the edge and `h` the other
way a step `s` in, their curvatures `2 (g − h) / s` apart across it bring them back across one
another having stood apart in the edge's order by `g² s / 4 (g − h)` at most, and within a
tolerance that order is none the complex holds — they are ordered as a run of tangent ways, a step
in, however far apart they leave the edge. Past `TANGENT` alone, a fitted sheet leaning 2e-4 into
a ring at first order beside a corner of its pieces, and 3e-6 along the rest of its touch, was put
inside the ring along a short edge of the touch and outside it along the next, three cells one:
brep-bench's 80° tumbling torus refused at every spelling of its motion but one, the one that stood
the ring's seam elsewhere (the pair stood apart by 2e-9).
`arrangement::a_sheet_tangent_to_a_face_at_the_tolerance_is_ordered_round_its_rim_by_a_step_in`
(a quadratic sheet leaning 2e-4 under a plane at its rim, the wedge under it a cell) and
`swept_rings::the_tumbling_torus_is_built_at_every_spelling_of_its_motion` are the gates. **A2, the meeting** (`brep::meeting`):
two surfaces' meeting over boxes of their parameters as branches (each crossing or touching all
along), singular points (a touch alone, or branches crossing) and coincidence, complete — boundary
crossings, every critical point of one's distance across the other's parameters (a closed branch
encloses one; one at zero is a touch), and between analytic surfaces `ssi::unseen`'s proved search
for the rest; an arrangement computes each pair of surfaces' meeting once (`meeting::Shared`,
surfaces the kernel holds the same one index) and hands it to every two faces on them — a closed
form found the first time, and a traced meeting traced over the box every face on the two surfaces
fills, kept, and handed to every later pair of faces on them, which are seeded only off it, so one
meeting curve bounds every face that uses it, traced or not. `meeting::a_*_meeting_with_every_kind_is_complete` (every pair of the kernel's surface kinds,
seeded placements, against a dense sampling of one across the other) and the singular-point cases
are the gates. **The zero set of a scalar function, traced** (#942, `brep::zero_set`): where a
sheet is trimmed by its depth inside a union of poses rather than by meeting other surfaces, the
trim is the zero set of `f(u, v) = min over s of g(u, v, s)` over a rectangle of the sheet's
parameters, and `zero_set::trace` returns it as pieces whose chords keep within the caller's
tolerance of it in space, each point carrying the minimiser, cut at corners where the minimiser
switches (`End::Switch`, solved as `g(·, s₁) = g(·, s₂) = 0`) and at the rim and given inner edges
(fold curves; a touch of one, `f` vanishing to second order, is reached as a seed found there, never
solved for), or closed. Seeds come from the rim and the edges (sign changes and touches), the
caller, and a halving search: with `Field::bound` enclosing `f` and its gradient, each cell is
cleared, shown monotone in a parameter (no closed curve inside; its edges' zeros isolated by the
bound and seeded), or halved to the tolerance and answered beside a traced piece or returned in
`Search::unresolved` — so every component is traced but one within the tolerance of another, one in
an unresolved cell, and a touch of an inner edge between its samples (they are sampled at the
caller's feature size); with a Lipschitz bound alone what it does not clear is answered beside a
traced piece within the feature size of it, at once, and with none the search is sampled (`Search::sampled`): cells'
middles pulled onto the zero set, rays from `f`'s critical points. The march steps by arclength in
space, corrects by Newton along the gradient held to half the step, keeps a step only where its
chord's quarter points, each pulled onto the zero set, moved no farther in space than the tolerance
(not `|f| / |∇f|`, no measure where a sheet's rates all but coincide) and are undercut by no other
branch, and refuses by name
a step under the rounding. `zero_set::through` traces the pieces through the caller's seeds alone,
with no boundary seeds and no search, and says so (`Search::sought` false). Every number is the caller's tolerance or feature size, the rounding's or
structural (#942 F6). `zero_set::*` are the gates: loops inside a rectangle (one a fiftieth of the
feature across, two a two-hundredth apart), the union of two discs with its corners at the circles'
crossings, a parabola touching an inner edge and a cusp ending on one, a curve rim to rim and one
touching the rim, a ball's depth swept along a line and round a circle (a stadium, an annulus), a
long meeting all but tangent under a Lipschitz bound, and `through` from one seed.
**A sheet lying in a plane is that plane over its patch** (#780 C4): a B-spline
surface every pole of which stands within the tolerance of a plane lies within it everywhere (its
poles' hull), and `ssi::same` reads the two as one — a coincidence, not a touch to trace — so a
piece of either on a face of the other is one face of the complex naming both; a point is on a
face whose surface is the same as its own only where it is on that face's patch too, where the
surface has a rim (`query::Located::face_holds`), not where the sheet's foot is clamped to the
patch's edge (`closed_class::a_fitted_sheet_lying_in_a_candidates_plane_is_one_surface_with_it`).
**A3, tolerant topology** (`brep::tolerant`): every rule by which two features are
the same in one place — a point onto a vertex within `tol` (`Pool`, each vertex keeping how far the
farthest point merged into it stood: `topo::Vertex::tol`, which `check` reads an edge's end
against), a vertex onto an edge and an edge onto an edge within `ON_CURVE` tolerances, a point onto
a meeting within `on_meeting`, a flat root onto a crossing within `FLAT`, a sliver into its edge —
applied once, in that order. **A surface onto a surface** (#961, `meeting::Shared::index_over`,
`ssi::same_over`): two faces' surfaces standing within `ON_CURVE` tolerances of each other over the
part of space both faces' boxes hold (their edges' bounds) are one surface of the arrangement,
whatever angle they lean apart at elsewhere — the reach at which an edge lying along another is
that edge. `same_over` reads planes by the corners of the region, and cylinders, cones and tori as
one turned by the lean about a point of its axis (its apex, its centre) and moved, a point within
the region moving no farther than the lean times its distance from that point, the move and the
difference of their sizes; a sphere has no axis, and `same` reads two by centre and radius alone.
A surface swept or turned from a curve (`Extrusion`, `Revolution`), a blend, a sheet, an affine
image, a family and an offset stay on `same`, parallel to `1e-12`. The face the two pieces become
is written on the lower operand's surface (the solid's, in a body's Boolean), and the other
operand's edges and corners laid on it stand off it by no more than the two stood apart there,
within the reach (`brep_coincident`'s `built` holds every edge of every result to it, and the
result to `check` at the ten tolerances `combined` checks it to). Every phase asks the one index each face's surface is given (`surface_index`): the
meetings, a flat root's given curves, and which face of another operand a piece lies on, a piece
holding a point of such a face read on it before its place in that solid is. Read with `same`'s
fixed `1e-12` on the angle, two faces meant on one plane and drawn through points rounded to a
few decimals (`1e-6` to `1e-10` rad apart) were two planes whose meeting was laid along an edge
both bound, and the complex refused them as coinciding along it, or a piece of one touched the
other along no face (fixture 115's strips at five, seven and nine decimals);
`brep_coincident::a_pad_leaning_a_hair_off_a_plate_is_united_with_it`,
`flush_blocks_overlapping_in_part_are_united`, `coaxial_rods_of_one_radius_leaning_a_hair_are_united`,
`frustums_of_one_cone_leaning_a_hair_are_united`, `sectors_of_one_torus_leaning_a_hair_are_united`,
`a_ball_drawn_a_few_tolerances_off_itself_is_one_ball` and
`fixture_115s_strips_unite_at_every_precision` are the gates (volumes to the closed form, either
operand first), and `a_pad_leaning_past_the_reach_keeps_its_foot` holds a lean past the reach two
planes. **Which faces are on one surface is read face by face, before any phase asks, and the answer
is the same whatever order the faces come in** (#994, `Arrangement::classed`): every two faces of
one kind not clear of the other operands are joined where `same` or `same_over` over their two boxes
holds, each class of faces so joined given one index (`meeting::Shared::class`; two classes of one
kind are never one), and the surface standing for it is one every face of the class stands within
the reach of over its own box, the lowest operand's first. Joined one face at a time onto the first
index whose surface was the same over the box the faces before it filled, a class took whichever
face was asked first for its surface: a foot's pad leaning `1e-7` rad about its own length, read
first, stood for the plate's top along the whole of a lip's pad — `1e-5` off at its far end — and
the plate's other face on that very plane was a surface of its own. **A class takes the index of a surface
given the arrangement (`give`) that one of its faces' surfaces is the same as, in any frame** (#880,
`Shared::class`): `ssi::same` to the tolerance, the rule `Shared::index` reads a face against a
surface given by, so a meeting given on a surface is found from a face on it however either is
written — and only an index no other class has taken, never one read the same over a region alone,
which would join what the classes parted. Taken where a face was the given surface exactly, a pose
of the ring road turned about its own axis — its seam stood elsewhere, the same torus in another
frame — was given an index of its own and lost the meetings given it: the 80° tumbling torus judged
wrong at one seam position, the full turn refused at every one but its own.
`arrangement::a_face_takes_the_index_of_a_surface_given_it_is_the_same_as_in_any_frame` (a torus's
meeting with a plane given on the torus with its seam turned, laid as given) and
`swept_rings::the_tumbling_torus_is_built_wherever_its_start_poses_seam_stands` and
`the_full_turn_is_built_wherever_its_end_poses_seams_stand` are the gates. **The rules an arrangement reads
two features as one by agree on the reach** (`tolerant`): two curves on one surface cross where they
come within `ON_CURVE` tolerances, inside both stretches by as much (`query::crossings_in`:
`curve_curve`'s closed forms read at that reach in every test they make, each a test of two curves
coming within it of one another; two cuts of screwed sheets and a carrier's roots kept on the other
curve within it, inside its stretch by it, the carrier's roots and whether two sheets, screws or
planes are one read to the tolerance), so an edge of a face leaning within the reach crosses the edges of the
face it lies on, where read to the tolerance it lay across the face as a slit; a vertex standing on
an edge within the reach of its end is that end, the two made one in step 3b (`tolerant::at_end`,
`collapsed`), where it was neither cut at nor joined and one point was two vertices; and the box a
vertex is sought on an edge in is padded by that reach.
`brep_coincident::strips_in_a_plates_thickness_at_its_true_edge_unite_in_every_order` and
`strips_abutting_a_plates_true_edges_unite_in_every_order` (a plate, a lip and a foot bent from
sheet, their pads in the plate's thickness or abutting its edges, each leaning within the reach,
united in all six orders and with the strips first: closed, every edge on its faces, the closed
form's volume and as many faces in every order; of 128 such placements, under the rules before, 110
were refused in some order and 36 built with a face more or fewer in one order than another) and
`fixture_115s_lip_and_foot_unite_with_the_plate_at_its_true_edge_in_any_order` are the gates.
**Two faces are read over what their boxes share and as far past it as a point near both is read
onto their meeting at the least sine** (#994, `Arrangement::classed`, `tolerant::on_meeting` at
`LEAST_SINE`): two faces whose boxes share only the line they cross along are one surface only where
they lean apart less than the arrangement resolves a meeting at, and two otherwise, their meeting
laid. Read over that line alone, any two planes crossing along it within `1e-3` rad were one there —
a plate's top and the raised half of a lip's pad turned `1e-3` rad across it one class, the plate's
top standing for it — and a foot's meeting with the class was laid on the pad's lowered half `3e-4`
off it. **That reading is the one rule of a surface onto a surface, and what reads a Boolean's result
reads it too** (#994, `tolerant::one_surface`): `classed` joins two faces by it, and two faces of one
body are one surface the same way out to a ball's sides and to the field (`fillet::one_surface`,
`BodyFaces::same_surface`) by it over their own boxes — each face a Boolean keeps carries its own
operand's surface, which may lean off the class's within the reach, so read as the same surface to
the tolerance alone (`ssi::same`, axes parallel to `1e-12`) two pieces of a plate's top the union
made one surface were two to the ball rolled along them and to the field after it
(`faces_a_union_made_one_surface_are_one_surface_after_it`). Not yet read by it: `Shared::one` (two indices of two kinds, a
sheet lying in a plane, or not given a class) and `Surfaces::one` for a face a trim met with nothing
read over what the boxes share without the reach past it, and two faces whose boxes share nothing as
not one; routing them through `tolerant::one_surface` moves what a Boolean reads, and was not tried. **A span of a meeting lying along a face's boundary within `ON_CURVE` tolerances, at every
eighth of it, is on that face** (`tolerant::along_boundary`, a rule of tolerant topology beside
its siblings, read by `Arrangement::kept`): the edge it lies along, as the vertex-onto-edge rule
reads a point so near, merged into it in step 4, the other face cut along it, where phase 1 did not
lay that edge in the other face as it is (`Meets::Along`, kept as `Crossings::along_face`, which
`kept` looks up — then the span would be a second edge beside it: `flush_ends`'s side turned `1e-8`
under a boss) — read Out of the face at the tolerance alone it was dropped, and a lip's foot turned
`1e-3` rad, crossing the side of a foot abutting it a tolerance and a half under that foot's edge,
kept the meeting leading up to it as a slit, its two sides one shell. Whether phase 1 laid the edge
was read again from nine samples of it against the other surface at the reach, and disagreed with
phase 1's own answer, read at the tolerance or the edge's own and through the faces' classes: in
`brep_coincident`'s strips at a plate's true edge 116 spans along an edge phase 1 had laid were
read as beside one it had not, and in `composed_rounds`' round beside a seam into a cone two the
other way (merged in step 4 all the same: the strips' 804 Booleans the same faces, edges and volume
to the last bit either way, the round's fill the same to the last bit).
`brep_coincident::strips_crossing_a_plates_faces_in_its_thickness_unite_in_every_order` and
`strips_crossing_a_plates_true_edges_unite_in_every_order` (the strips turned `1e-5` to `1e-3` rad
about their own lengths, united in the six orders and with the strips first: closed, the closed form
of the turned strips' volume, `Strips::united`, and as many faces in every order) are the gates.
**Not yet**: strips abutting a plate's two edges whose turns about the middle of the sheet's
thickness differ put four planes through one point, the plate's corner, two pairs of them crossing
there at `1e-4` to `1e-3` rad — a vertex minted at one pair's crossing stands `tol / θ` off where
the other pair's exact line passes, past the `ON_CURVE` reach a closed form's meeting takes a vertex
onto (`points_on`), and the union is refused in some orders; and strips turned `1e-6` to `1e-4` rad
across a plate's faces in its thickness, or `1e-6` to `3e-6` abutting its edges, past the reach and
under `LEAST_SINE`, are refused, or built with a face more, in some or all orders.
A non-rational B-spline's heights over a plane are its distances from it, so its reading against a
plane (`query::spline_plane`) proves a run where a stretch's heights are enclosed within half the
tolerance (`HeightsWithin`), settled as one meeting: a contact curve running
in the plane of a torus's seam circle over a stretch read 185 roots a hair apart and was halved past
any search's boxes (#880 Q1b). The crossing a flat root goes onto is one of its edge's own cuts, or
one found beside the edge (`Crossings::beside`): where an edge of the other operand lying along a
face the edge bounds crosses it. A circle of a torus at a sweep's end pose meets the fitted sheet
that touches the torus along its rim flatly; the fit crosses the circle within the tolerance of the
touch, a proved search finds that crossing 2.7e-4 off, and the rim's own crossing of the circle is
the root (#880 R1, brep-bench's `swept/tumbling-torus`, refused by 0.7.0;
`swept_rings::the_tumbling_torus_as_the_bench_spells_it_is_built_closed`). A search's warm start (`util::warm`: a sheet's, a blend's, a wall's
last foot) is forgotten at every arrangement and every parallel item and kept as it was across
narration, so a split is the same bits with `SOLVENT_BREP_DEBUG` set and unset and on any thread
(`arrangement::a_split_is_the_same_narrated_or_not_on_any_thread`). **A4, the arrangement**
(`boolean::arrangement`, `arrangement_within`, `members`, `selected`): any number of solids' faces
split by all the others at once — where three faces of three operands meet at a point no edge
passes, the two meetings on each face cut there (`triple_points`) — a span two faces only touch along
laid as an edge, every piece a face of the complex but a sliver, coincident pieces one face naming
each operand; a traced meeting along an edge one face already has is that edge, laid once (`along_an_edge`),
and so is a span of one along edges either face has, however many (`span_along_edges`: a trace runs on
past the faces, and a face may hold its stretch in two edges — a ring kept within a cylinder its
round crosses 0.001 deep, built at the cylinder's tolerance and cut at its top plane's circle where
the meeting passes 5e-7 under it; laid again beside them, a hair apart and tangent all along, the
walk round the face could not order the two, `curved_ends::a_rim_ending_on_another_arc_at_a_corner_is_built`),
and laid across the other face too wherever it lies inside it (`Meetings::along_edges`: a torus's
seam in the plane through its axis that holds it, `brep::a_torus_cut_along_its_seam_is_half_of_it`); a meeting
known before any is sought given it (`Given`, `arrangement_given`, #810): curves on two operands'
surfaces that are their whole meeting, laid as an edge of both with no search for that pair, and
the points where the two touch besides (`Given::touches`, #942 F3), an edge of either through one
read as touching the other there with no search, a touch being a double root no search proves
(gated with no sweep by `arrangement::a_rim_touching_a_wall_at_points_given_is_read_there_with_no_search`:
a drum's rim touching a parabolic channel's two extruded walls at a point each, refused with
nothing given, read there given the walls' meetings and the points, its volumes to the rounding);
a given curve that is an edge either face already has (equal to its curve, no tolerance read) is
that edge, laid once, its crossings with the meeting's other curves read first and kept on them
(#942 F3: a ring's tube cut along its crossing of the tool where it starts, given that chord as
their meeting — the ring along a spline, its tube face's "pieces do not close" with the chord laid
twice, now arranged; gated by
`a_tubes_chord_given_as_its_meeting_with_the_tool_is_its_own_edge_laid_once`, each of its tubes,
its chord a later span of a curve of several, arranged with the torus where it ends and no curve
laid twice over one stretch; the ring's two tubes come no nearer than 0.9, so its
near-coincidence was that chord and not one surface crossing itself) —
what a sweep knows of its candidates' touches by construction
(generating.md, #810); each cell's membership in each operand read off a face of it bounding the cell, else at
a point deep inside. **A trim is the same phases with no cell** (#880 E1, `boolean::trimmed`): only the pairs of faces a `Meet` names met (an edge with a face only where a face it bounds is), each piece kept, turned or dropped by a side test its caller gives, coincident pieces stood for by the lowest operand's, and the kept pieces assembled and healed as a selection's are; with no `Meet` the arrangement is as it was (generating.md's trimming by membership). **A Boolean is a selection over it**: each piece's two sides' membership read
as before, the complex formed, each shell kept as the operation keeps its sides — which agree round
it, with no face kept by its own sides instead: a shell kept on one side and dropped on another is
refused, two cells joined round an edge — and the result touching itself along an edge given an edge for each
wedge; `split` is the complex's shells inside the solid, but for a shell holding every face of it
by both sides — a sheet's piece inside a cell that parts nothing, touching none of its faces: it
bounds nothing and is no cell (#812, `arrangement::a_sheets_piece_parting_nothing_is_no_cell`;
returned as one, of volume nought and no point inside, it refused a gear built whole), while a
piece slit into a cell from its boundary is of the cell's shell. **Every arrangement lays its touches as
edges, and a selection's result heals what parts nothing** (`heal`, `boolean::selection`): a span
two faces only touch along is laid as an edge (a Boolean's, a split's and an arrangement's alike),
so the cells either side are classified apart; then, in a Boolean's or a split's result or a swept
solid's, an edge a meeting laid — a touch, or another operand's surface crossing a face where none
of that operand is kept — is no edge wherever one face of an operand is kept on both sides of it:
that face's pieces joined again across it, and a slit left inside it closed, so the result is the
manifold B-rep it would be had the meeting never been laid, its faces whole and touching where
they touch. A meeting bounding a face kept on one side only stays an edge, and an operand's own
edge (a seam) is never taken out (`arrangement::a_boolean_lays_a_touch_and_heals_it_where_a_face_is_kept_on_both_sides`,
a rod lying on a block united and cut either way; `a_hole_whose_caps_touch_the_bore_and_the_rim_is_cut_whole`,
the V-twin disc's set-screw hole, its caps tangent to the bore and the rim, at twelve angles, to
its section's run in closed form; `a_split_heals_a_touch_out_of_each_cell`). **A branch arriving
at a touch along an edge joins it** (`boolean`'s `landing`, `ssi::trace_landing`, #880 Q1b): along
the edge, one surface stands off the other by `d²(c(s) + O(d))` a distance `d` across it, so a
transversal branch of the two's meeting leaves the touch where `c` changes sign, and near it the two
part too slowly for a march to land. A march stopped beside such an edge is followed down to it —
at gaps halved to the tolerance, the branch's root along the points that far off the edge nearest
the last — and ends where the last two levels run on to the edge, a point of it, one curve; it
stopped short before, the stop read as a touch of its own whose other half-branches were traced
along the edge, and the pieces did not close (`swept_contacts::a_contact_carried_back_through_its_start_pose_is_arranged_with_it`,
the leaning torus's carried contact re-entering the start pose it touches along its rim). **A given
meeting is held to the tolerance** where two meetings on a face are crossed (`triple_points`,
`meeting::Shared::given`): its curves lie on both its surfaces, so the reach a trace is held to —
the tolerance over the sine of its surfaces' angle — does not apply, and at a point where several
surfaces are tangent it cut two given curves at a near miss far from where they meet. So too
wherever a given meeting's reach is asked: their crossing is kept where the two curves cross, with
no Newton on the three surfaces (singular where all three are tangent, it slid 3e-3 along the
valley). **Two given curves one operand holds as edges cross on a face of another at a vertex they
share** (#942, `Arrangement::given_crossings`, `Meetings::given_along`): a sweep's sheet passing back
over itself holds two given curves as its own edges, each laid across a pose's face as that edge
(#942 F3); phase 1 reads an operand's edges against the other operands' faces, never against one
another, and `triple_points` reads only the meetings laid, so their crossing, a point of the pose's
face and the sheet's two, was cut by nothing — the full turn's start rim dangled on the start torus
where the tube, late in the roll, passes over it, and a piece leaked round it. Each two such edges
of one operand laid across one face are cut where their curves cross, to the tolerance
(`arrangement::two_given_edges_of_one_operand_crossing_on_a_face_share_a_vertex`); only given
curves are read so, since a sound operand's edges cross nowhere but at its vertices. **A piece
merged into another edge keeps its own pcurve, carried** (#942, `carried_pcurve`): where an
operand's edge has a polyline pcurve of its own (a double curve's chord on one leaf of a sweep's
sheet, drawn through the places its trace passes) and is merged into another's (the other leaf's
drawing, one curve in space), its pcurve's vertices are moved to the other's parameter of their
nearest points, a stretch that does not hold given its middle until it does; inverted onto the face
from the other's curve, the full turn's landed on the leaf past the fold, 1.3e-5 off. It holds where
the surface along it stands within a quarter of the tolerance of the other's curve, or the merge's
gap, at each stretch's quarters; else the inverse is read as before. The merge's gap is read at the
eighths of the piece and at each vertex of either polyline (they crowd where a drawing bends: the
full turn's two drawings part by 4.1e-6 within a ten-thousandth of an end, which the eighths read
as 7.7e-7). A vertex is taken onto a given curve only within `ON_CURVE` tolerances (`points_on`: two
sheets meeting smoothly along their shared rail, the sine of their angle nought, took a vertex 3e-3
off it). **An edge is read against a known meeting by one rule** (#990, `Arrangement::known`,
`meeting::Shared::known`, `on_known`): it lies in each of its own faces' surfaces, so it meets
another operand's surface only where one of those meets that surface. Where a face of its own lies on that very surface, the
edge lies along it (a rolled fill's crease, a fitted meeting of a sphere and a cylinder, against
the cylinder a ring's cap lies on). Else the meetings of its faces with that surface that are known
— given, with the points the two are given to touch at, or found in closed form, whatever the
other faces it bounds do — are where it meets it: where it crosses their curves, all along one it
lies along, or through a touch given. Where every one of those crossings is read in closed form (an
edge and a meeting's curve both lines or circles, `query::curve_curve`; a tangency among the
crossings, and a meeting's line running on without end) that is the reading, with no search; the
search would read a double root it cannot settle at the kernel's tolerance (a fill's cap in the
plane of a vane's end, which the next fill's torus touches round its contact circle: "read more
boxes than any search may"), and a touch it did settle was placed only to the square root of the
tolerance. Where a crossing is only sampled (`meeting::crossings`) the search is read first, and
each of its roots taken onto the crossing nearest it (a crossing of two curves all but tangent — a
rim crossing a path near a fold of its contact — the search settles where the sampled polish
misses it), or read off the crossings alone where the search is refused (a swept sheet's rail
leaves the tool posed at its end tangent to it). **An end on the meeting is read there by
construction** (#942, the ring's ends kept): where an end of the edge lies on one of the known
curves and the edge runs on from it within `ON_CURVE` tolerances of the surface, that run is the
touch `at_its_ends` would read at the end — read there, and the search reads only the rest of the
edge, from where it first stands off by more (a transversal crossing at the end is no touch: its
run is shorter than the step a touch is read across). A ring's kept leaf's rim, an edge of the end
pose's wedge, stood within 1e-7 of the end piece it leaves at their shared corner, and the search
read boxes along that run for five seconds an end before refusing; the ring along a spline is
built in 3.5 s where it took 8.7
(`arrangement::an_edge_leaving_a_point_of_a_given_meeting_and_hugging_the_face_is_read_at_its_end`,
an edge `k v³` off a curved face from a point of the parabola given as their meeting: 0.11 s of
search before, 0.03 after). A closed form meeting in no curve is no knowledge
of it: it leaves out a point the two touch at, which the edge may pass
(`a_cap_in_a_plane_a_torus_touches_round_a_circle_meets_it_on_that_circle`). So a drum's rim,
touching a channel's walls at two points, is read with nothing given off its top's meeting with
each wall — the ruling through the point, which the rim touches there — each touch a vertex, where
until #962 the search was refused
(`arrangement::a_rim_touching_a_wall_at_points_given_is_read_there_with_no_search`). Until #990
three paths stated this and disagreed: a given meeting was read only where every face the edge
bounds was given one, a closed form only after a refused search, and a touch the search settled
was placed again by its own crossing with a meeting worked out anew. **A touch run reaching an
edge's own end is that end** (`at_its_ends`): a root the edge reaches from its nearer end within
`ON_CURVE` tolerances of the surface all the way is read there, a touch (a swept sheet's rail
leaves the pose it starts on tangent to it, and the run's far end was read as a vertex of its own).
**A piece the collapse made a loop of no length is collapsed too** (`collapsed`, step 3b): a piece a
few reaches long whose ends the merges made one vertex — one a hair past the reach, beside a piece
that collapsed — is gone, where it was left a loop round the vertex and the face's pieces did not
close. **A Boolean is the
selection, with no other road**: a shell kept on one side and dropped on another is refused (two
cells joined round an edge), where each face was once kept by its own sides; the fifteen round
cases that took that road did so because a fitted round leaves the edge some 1e-5 off the face it is
tangent to, which `TANGENT` now reads as tangent. Every Boolean of the suite gives the same
result through it as before (the agreement gate: the whole fast suite). What the arrangement holds
that the pairwise Boolean never met: a cycle of a face's pieces whose outline is coarser than it is
wide has its side read along its curves (`fine_area`, a cusp-thin strip between two curves tangent
at its end); a meeting off either surface's patch is not laid (a hyperboloid's rulings past its
heights, which a revolution's inverse puts on its rim); a traced meeting along an edge either face
has is that edge (`along_an_edge`); two meetings crossing on a face with no closed form for it are
crossed by search (`meeting::crossings`: polished from each curve's own ends first, where an end
on the other curve is the meeting exactly, then from chords standing within reach and their sags,
where they may cross within those or stand at a least of the chords' distance — #780 C2, whose
contact ruling's foot a polish from chords crawled toward along two tangent curves and stopped
4e-4 short of, after three thousand polishes) and the point polished onto all three surfaces
(`shared_point`); and a cell too thin to mesh is read inside at its faces' points stepped in
(`Complex::inside_points`). **Points along a shallow meeting are one by its reach** (A3): a
crossing with a meeting is read to that meeting's `on_meeting` reach at the point, not the pool's
(a meeting traced between surfaces meeting at a few degrees is known no better); a triple point
whose three surfaces' Newton cannot settle because two of them touch there is the third's root
along the touching pair's closed-form curve where it crosses that steeply (`touch_point`), else the
vertex either tangent meeting already has within the stretch they stay within reach of each other
(`crossing_in_touch`); an edge touching a surface where a face of its own touches that surface
along a curve touches it at its crossing with that curve (the known meeting's, above), not at a
double root placed to the square root of the tolerance; two closed-form curves of one meeting are crossed open
or closed, over their stretch within the faces' box (`crossings_among`: a plane tangent to a
hyperboloid at its throat meets it in two rulings, cut where they cross); and the faces round an
edge are ordered where their ways in stand farthest apart along it, not at its middle beside a
touch. **A plane through a quadric's asymptotic cone is tangent where its constant at the centre is
rounding** (`ssi`'s plane–quadric closed form): the quadric's misfit to its meridian, or the
cancellation of the terms that constant sums — not its square root, the hyperbola's semi-axis,
against the tolerance, which a plane holding a hyperboloid's ruling exactly (some 1e-14 left, a
semi-axis of some 1e-7) failed at any tolerance finer, its rulings coming out as branches
(`meeting::a_plane_holding_a_hyperboloids_ruling_meets_it_in_two_lines_at_any_tolerance`).
**A plane tangent to a cone along a generator meets it in that line** (`ssi`'s plane–cone closed
form, a plane through the apex leaning to the axis by the half-angle, judged by the gap): a cone
narrowing along its axis too, its half-angle below nothing (`build::cone`'s frustum from the larger
radius), whose tangent planes were traced and refused as nearly touching
(`swept_closed::a_frustum_slid_aslant_closes_into_its_shadow_carried`, a ruling slid into its
tangent plane against the frustum at its end poses).
**Parallel cylinders and cones touching along a line, and a ball in a torus's tube, meet in
closed form** (`ssi::parallel_touch`, `ball_in_tube`): two cylinders or cones whose axes are
parallel and whose sections' circles touch at two heights (their centres apart by the sum or the
difference of their radii) touch along the line through those points — a frustum at an end pose
and the cone its generator turns into about an axis along its own; a ball centred on a torus's
middle circle with its tube's radius lies in its solid and touches it round the meridian through
its centre — where they were traced and refused as nearly touching
(`swept_closed::round_tools_turned_part_of_a_turn_sweep_their_sections_by_pappus`). **Two
surfaces clear of one another, or resting on one another at a point alone, meet in no curve — one
rule, whatever their kinds** (#994 item 5, `ssi::apart`): where their least distance, signed (less
than nothing where they cross by so much), is more than the tolerance they are clear; within it
either way they meet at most about the pair of points it is reached at, and the touch there is
their whole meeting where it is alone by the very rule an arrangement reads a resting touch by
(`ssi::touch` definite, and `ssi::alone_over` over its radius, below) — read, as a ball resting on
a plane is, as meeting in no curve, and never traced. What is kind by kind is only the closed form
of the least distance and where it is reached (`ssi::least`): two cylinders whose axes lean apart,
the axes' gap less both radii (each bounds a convex rod, and the plane square to their common
normal between them keeps one on each side); a sphere and a plane, a cylinder, a sphere or a torus,
whose signed distance is the distance itself, that distance from the centre less the radius (it
changes by no more than the distance moved, so nothing of the surface is nearer the centre); a
plane and a torus, the core circle's least height less the tube's radius. Traced, the search for a
meeting no edge crosses found a point beside the touch, the second-order reading took it as a touch
alone only where its disc is small beside both faces, and the trace from it gave nothing (a blanket
round over a clevis's top: "a cylinder and a cylinder meet at …, where no edge crosses, in a curve
that could not be traced"). **How far a near-touch may spread is derived, not borrowed**: the two
stand within the tolerance over an ellipse about the touch `√(2 tol / λ)` long (`λ` the least size
of their relative curvature: for rods leaning `θ`, `sin² θ / (r + s)`, the ellipse
`√(2 tol (r + s)) / sin θ`), which at a lean of a millionth is the part's length, a touch along a
curve the meeting's ridges read (`meeting::marched`) and the faces tangent there are laid along.
`alone_over` holds the touch's distance to its second order out to its radius `2 SHALLOW / λ`, and
two rods part only linearly past `(r + s) / sin θ` — slower than the second order says from
`2√2 (r + s) / sin θ` on — so they rest alone exactly where `sin θ ≥ SHALLOW / √2` (0.71°),
whatever their radii; where `λ` is too small beside the larger curvature to read (`touch`'s `1e-6`)
there is no point touch at all. Before, two rods were read by a rule of their own, at a lean of
`SHALLOW` (a degree, the sine below which a trace is ill-conditioned) or more, and a ball by one of
its own, against a cylinder or a torus off their axes and core circle by more than the tolerance —
so a ball `2 tol` off a tube's axis, within the tolerance of it round the whole circle, met it in no
curve. Nearer, or sunk in, they cross and are traced as before (`cylinders_touching`: radii from 1
on 2 to 0.5 on 20 and of one radius, leans from 90° to 2°, whole or ending at the touch, each
Boolean the rods' own volumes; 0.5° and under left to the trace; sunk a tenth, still crossing;
`resting_touches::a_ring_resting_aslant_on_a_plane_meets_it_in_no_curve`, a ring leaning 15° to
60° on a plane or under it — resting, within the tolerance either way, and clear — in no curve,
where it was traced, sunk a hundredth traced, all but level never no curve;
`a_ring_resting_aslant_on_a_plate_is_its_own_volume_under_every_boolean`, built before by the
general method and now in closed form; and
`a_ball_a_hair_off_a_tubes_axis_is_never_read_as_meeting_it_in_no_curve`). **Two cylinders whose
axes lean apart meet in a graph over the
lesser's angle** (#994, fixture 146; `ssi::crossed_cylinders`, `cylinder_loops`): its point at angle
`u` round the lesser is on the other where a quadratic in the height along the lesser's axis holds,
whose roots `h± = (−B ± √D) / A` are the two branches. Three bands, for axes crossing (or passing a
hair apart) at radii equal or nearly, whose two loops come within `2 √(2 r δ)` at two necks: the
radii's difference `δ` and the axes' gap both within the tolerance — the singular case, the two
ellipses crossing at the necks, as before, and no wider (a plane through the necks, a pocket's floor
through a groove's axis, meets the two cylinders in curves that past the tolerance miss one another
by `δ`); past it, where `D` keeps above nothing all round (the lesser right through the other), two
closed traces whose points are the graph's, on both surfaces to rounding, no chord longer than a
quarter of the branches' gap at its ends, so a projection between two never crosses a neck to the
other loop (marched, the trace jumped between the loops at a neck, a cylinder's pieces not closing);
and where `D` falls below nothing (a loop turning back: the lesser cutting into the other's side, or
one neck closed by an axes' gap), or the axes lean under `SHALLOW`, traced. **The lean's bound is
the meeting's, not the closed form's**: the graph keeps within the tolerance of both surfaces down
to a lean's sine of about `ε (r + s) / tol`, but two rods whose axes cross meet at the lean itself
where the lesser's normal lies along the plane of both axes (`n_a · n_b = ± cos θ` there), so
under `SHALLOW` the meeting is as ill-conditioned somewhere as `SHALLOW` bounds every meeting by,
a trace's included. **Whether `D`
keeps above nothing is proved, not sampled**: `D` is one formula over the angle, its roots isolated
from its own enclosures (`util::isolate`), so a dip below nothing narrower than any sampling (a
neck `√(2 e / r)` wide where the axes pass `e` apart) is never missed, and a dip to within its
rounding of nothing is a touch, traced; it was read at 4096 samples and a golden-section search
about each least one. A loop's chords are no longer than `LOOP_CHORD` (an eighth) of the radius —
sagging `r / 512` where the loop bends as the cylinder does, so `Traced::project` starts well inside
its basin — refined no finer than a chord within the arrangement's reach of its ends
(`2 ON_CURVE tol`) or than the parameter parts, and a loop that would take more than `LOOP_POINTS`
points is traced, where it was cut off there and given as it stood. The gates are
`crossed_cylinders`: rods of 6 crossing at 90°, 60° and 30°, radii 0 to `1e-2` apart either way and
axes `1e-7` to `1e-4` apart, every Boolean either way the closed form's (the strips' parallelogram
integrated by Gauss–Legendre) to `1e-7` of what they share; and fixture 146's block, obround pocket
and groove in its floor through the ends' axes, its radius a micron to a tenth off or its axis a
tenth aside, in three orders, to its own closed form to `1e-10`. Still refused: radii `1.5e-7` to
`5e-7` apart (a tolerance to four), where the groove stands that far through or short of the
pocket's sides and floor edges — a sliver of the side no deeper than a few tolerances between lines
`2 √(2 r δ)` apart, which the arrangement cannot place — and rods of one radius with axes a hair
apart (one loop turning back at a neck, traced and refused: "a shell of the arrangement is kept on
one side of a cylinder and dropped on another"). **Two surfaces resting on one another at a point
meet there alone, whatever their kinds** (#994 item 5): a definite touch (an elliptic one, its
relative curvature `Q` definite) is read alone by the one rule above, merged with #996's §3
(`ssi::alone_over`, over the touch's radius, whatever the faces' size); #994 read it over eight
discs `√(2 tol / λ)` (`ssi::resting`), refused past a twentieth of the smaller face, and that
reading is gone. `ssi::touch_near` finds the point by Newton on the difference of the two exact unit normals over one
surface's tangent plane, settled by the height a step changes (a thousandth of `tol`), not by a step
a fraction of `tol` long, which a small `λ` does not let it reach. An indefinite `Q` (a saddle
against a rod, two vaults' crown) is two branches crossing at the point, which the trace starts
from, as before. An edge whose search is refused where a surface of its own rests on the other (a
blend's seam circle through the point where a torus rests on the blend) meets it there alone within
the touch's radius, shown alone over it (`ssi::alone_over`), a touch, the rest of its stretch searched either side (`boolean::resting_on`).
Where the two surfaces' least distance has a closed form, the touch `ssi::apart` reads is kept as
their known meeting — no curve, and that point (`ssi::meeting_resting`, `meeting::Shared::known`) —
and an edge lying in either is read against it by #990's one rule (`boolean`'s `on_known`): a touch
where it passes through the point, nothing elsewhere, no sample and no search; `resting_on` reads
only pairs whose meeting has no closed form (a torus on a blend). And in
closed form, a ball whose centre stands its radius or more from a cylinder or a torus meets it in no
curve where its touch is alone, by the one rule above (`ssi::apart`). The gates are
`resting_touches`: rods across and at 60°, a
ring and a ball resting on an R20 cylinder, an R20 torus and a frustum's side, every Boolean either
way the operands' own volumes (read the old way, six of the twelve refused: "nearly touching",
"a cylinder's pieces do not close", a seam circle "not proved"); the same sunk a tenth still
crossing; a rod resting on a torus's inner equator crossing through its touch (refused before:
"a shell of the arrangement is kept on one side of a cylinder and dropped on another"); and as
documents a hoop resting on a beam (on `main`: "a cylinder and a torus meet at …, where no edge
crosses, in a curve that could not be traced"), and the clevis of fixture 122's class, a band
bowed in under its eye, the eye blended into the top by R20 and every edge of the top rounded — the
bowed side's round a torus resting on the blend where the blend leaves the top — built and its
boundary the field's about that touch. **Not yet**: a shallow crossing, not a touch — a rod sunk
`0.01` into a frustum's side at 30° — is still traced and refused as nearly touching.
**A face's
pieces are ordered by their curvature where they leave a point tangent** (`boolean::pieces_of_face`):
each half's way out of a node is read a ten-thousandth along its own length, and where two of the
ways there agree within `TANGENT_TIE` (a thousandth of a radian, what a step along an arc turns its
chord) they are read again at one length along each — first where a tolerance's lean (`4 tol / δ`)
turns a way by the tie itself, `δ = 4 tol / TANGENT_TIE`, no more than a quarter of the shortest of
those tied (#942 F6; nearer the node no reading parts two ways the tie holds as one), and ten times
on while they still agree — each half's chord to its point that far from the node, the part of its way there found by
halving (a curve's parameter runs at no one speed: a trace's counts its points, which crowd where it
sets out, and two probes of its parameter there, extrapolated to that length, once read a way 80
radians off, #791) — so two arcs touching there part by their
curvature, where an arc of five times another's radius and length read alike to rounding (a drum at
its end pose and the cylinders its generators turn into, in a plane square to the turn); and a
fitted round's own small lean off tangent, read at the scale the face is drawn at, does not rule
(`brep::a_face_rounded_from_both_edges_short_of_its_middle_keeps_a_sliver`).
**A cylinder touching a torus round a meridian meets it there** (`ssi::tube_along_torus`): of the
tube's radius, its axis through a point of the tube's middle circle along that circle's tangent, it
is the ball there carried along the tangent and touches the torus round the meridian through that
point, where it was traced and refused as nearly touching (a bull-nose mill's round at an end pose
and the cylinder its meridian sweeps under a slide,
`flat_faces::a_bull_nose_mill_slid_to_a_stop_inside_the_block`).
**A level circle screwed meets what the screw's axis runs along in closed form**
(`ssi::screwed_circle_meeting`): at each `v` the circle is level, so a plane square to the axis
meets the sheet in the circle the screw lifts there; a cylinder about the axis in the helices of
its points at that radius (`Iso`, one where it touches); a cylinder about another line along the
axis (a round tool's side at an end pose) where the two circles of that level meet,
`A(v) cos θ + B(v) sin θ = C(v)`, its two branches joined where they turn and ended at the circle
where the two coincide — its points written in closed form and handed over as a traced meeting, so
no near touch is traced (`swept_closed::a_screwed_circle_meets_planes_and_cylinders_in_closed_form`);
a cone about another line along the axis likewise, its radius `R(v)` the cone's at the height the
circle has climbed to (#780 C2: a frustum's rim screwed and the frustum at an end pose), each branch
ending at the coincidence on the roots of the coefficients' rates there — the circle's points square
to the centre's way on a cylinder, the feet of the cone's contact rulings on a cone, where the sheet
touches it (`swept_closed::a_frustums_screwed_rim_and_ruling_meet_what_they_touch_in_closed_form`);
any other plane (a sector's side through the axis) where its line in each level meets the circle,
the same branches (`branches`). **A line screwed meets a level circle screwed by the same screw, and
a cone, in closed form** (#780 C2): the circle's sheet in the helices of the line's points the screw
carries to the circle's height onto the circle, `|w(u) − R_{s(u)} c₀|² = r²`, isolated by its enclosure,
written once over `Num` (`ssi::screwed_line_circle`, #814); a cone where each ruling meets it, `A(v) u² + B(v) u + C(v) = 0`, the
two roots joined where they turn and a ruling the cone holds returned whole, the cut ended on it
at its limit — the roots of the coefficients' first differences across it, or where the sheet
touches the cone along it their second (`ssi::screwed_line_cone`, on `branches`, which reads a
screwed circle's meetings alike). A branch's end at a coincidence is that circle's point square to the
centre's way there, the way read across it (a one-sided reading leans by the hair and bends the
first chord, which its projection then slides onto the circle); points are added beside a turn,
where the roots part as a square root, and wherever either turns by more than `π/1024`, which a
traced edge's flux reads one stretch at a time. Two sheets of one level circle under one screw are
one surface only where they share some of its turn (`one_screwed_circle`); apart along it they meet
nowhere. Where a closed form's traced pieces end on one another (a branch on its coincidence
circle), each end is laid as a point on both (`boolean`'s `ends_on_one_another`), which no
crossing of two curves finds. **A line meets the sheet exactly** (`query::line_screwed_circle`): the
circle level with the line's point is at the `v` its height gives, linear along the line, and
screwed back the point's distance from the circle's centre is `|A(t)|`; `G = |A|² − r²` has its
roots isolated by bounds on `G'` and `G''` over each stretch, so a ramp thinner than any sampled
step has both crossings found — a ray's parity, a solid's point location, by it
(`swept_closed::a_line_meets_a_screwed_rims_sheet_exactly`: a sampled search was wrong at seven of
400 points about a ramp half a millimetre a turn). **A line along the screw's axis meets any circle's
sheet exactly** (`query::line_along_screw`, #791): the screw keeps a point's distance from the
axis, so the line meets the sheet only along the helices `ssi::screwed_circle_round` reads on its
cylinder, each where it has turned round to the line's angle; a touch helix read once and placed
where the circle's distance from the axis turns, the line tangent there (the seam of the cylinder a
ball's pipe touches along that helix) meets the sheet at the helix's own point whatever the
rounding, where searched it was found at one lead and missed at another and the helix crossed the
seam uncut (`swept_closed::the_drawn_ball_end_mills_pipe_beside_its_screwed_side_is_arranged_at_every_lead`).
**A line meets a straight edge's sheet in closed form where it crosses it cleanly**
(`query::line_screwed_line`): the sheet is ruled, `S(u, v) = Q(v) + u W(v)`, so a line `p + t d`
meets the ruling at `v` only where the two are coplanar, `F(v) = (Q − p) · (d × W) = 0`; turned
back by the screw's turn at `v` the triple product is `α(v) + β(v) cos θv − γ(v) sin θv`, three
quadratics, its roots isolated by its enclosures, and at a root `F' = det[d, W, S_v]`, so a double
root is a line tangent to the sheet. Only a reading whose every root is simple, the line across
its ruling there, and clear inside or clear outside the sheet's square and the line's stretch is
answered so; a touch, and a meeting at an end of either, are the general reading's, by its rules
(`enclosures::the_general_reading_holds_every_closed_form`,
`swept_closed::a_straight_edges_sheet_in_closed_form_is_held_to_the_general_search`). The
three-unknown search had made a ray of a point's location against a screwed polyhedron's sheet
the slowest read of its build.
**A circle's sheet meets a plane square to the axis in closed form** (`ssi::screwed_circle_level`),
each run of the circle's points reaching the plane ended where its `v` reaches the sheet's edge,
not the tolerance past it: the meeting's end is the vertex where the edge's circle crosses the
plane, on both (#791: set out a millionth past it, the meeting stood that far off the vertex where
it, the foot's rim and a touch helix touch). The sheet's inverse starts level with the point
(`Blend::level_circle`, `LevelCircle`) and not from the last foot found: a ramp thinner than that
start's reach holds its other side within it.
**Two tori touching round a meridian meet in closed form** (`ssi`'s `tori_touching`: their axes
along one another, their middle circles at one height and touching, their tubes of one radius — a
torus at an end pose of a turn about an axis along its own, and the torus a meridian of it turns
into): the meridian, and in each level the one pair of the two tori's circles that crosses, as the
rim's branches are (`branches`), drawn in toward the tube's top and bottom, where they reach the
meridian tangent to the level circle. **A traced curve is read between its points on a parabola**
(`Traced::between`: through the two and a neighbour, by their chords' lengths, where the three run
smoothly — chords within `SMOOTH_RATIO` of each other's length, turning by less than
`acos(SMOOTH_COS)`), then pulled onto both surfaces: a chord's sag, quadratic in it, stood off the
curve where two surfaces all but touch by more than the projection could recover, and onto another
branch of their meeting — a branch leaving a point where three surfaces touch pairwise read on the
wrong side of the circle it leaves along, and a piece of the arrangement lost
(`swept_closed::two_tori_touching_and_a_plane_on_both_hold_every_piece`).
**A ball's pipe under a screw meets in closed form** what touches it: the ball at either end pose
round the great circle there (`ssi`'s `pipe_end`), and a cylinder about the screw's axis in the
helices of the circle's points at its radius, the roots of a trigonometric polynomial of degree two
round the circle, a touch where it only reaches nought (`screwed_circle_round`). **An edge's flux
settles** (`props`'s `settled`): read over its pieces, then twice as many, the coarser kept where
the readings agree to rounding. Otherwise the interval whose rule and its two halves disagree
most is split, against one error budget set by the edge's integrand's mean magnitude. The reading
with the least estimated error is kept: ever nearer an exact pole the inverse's rounding grows,
so the finest is not always the best. Refinement ends at that budget, after 32 splits without an
improvement, or at its depth/work bounds (twenty levels, 4096 splits). A circle close to a ball's
pole sweeps half a turn of `u` in a tiny part of its span: six doublings of the whole edge still left
brep-bench's boolean/unequal-lens-aslant 1.5% off its volume. Local refinement resolves that turn
without refining the whole circle to its width.
`brep::an_unequal_lens_near_a_pole_measures_its_spherical_caps` holds its volume and area to the
closed forms of its two caps under three placements of the balls' charts.
**A face's inner integral is closed on the analytic surfaces** (`props`'s `Closed`, prepared once a
face): Green's `G`, the area's `|S_u × S_v|` or the flux's `X · (S_u × S_v)` integrated along one
parameter, is read in closed form on a plane, a cylinder, a cone, a sphere and a torus (along either parameter) and
along an extrusion's rulings — in the surface's frame each integrand is a sum of powers and
products of the angle's sine and cosine and the length along it — where it was a Gauss rule of
forty points or more at every node of every edge. An area whose `|·|` changes sign within the span
(a sphere's past a pole, a spindle torus's) is still read by the rule. Every face of spheres,
cones (to a point too), tori and cylinders, whole, moved and cut at a slant, agrees with the rule
to 6e-16, and a cube rounded all over is measured in a millisecond and a half, where its area took
26 ms. **A sheet carried between two straight edges is integrated along its rulings by two nodes**:
its point is affine in `u` at every `v` (the carry is rigid, its rate affine in the point), so the
flux's `g` is a polynomial of degree two at most in `u` and `G` is exact by a two-node Gauss rule,
read along `u` whichever parameter the face spans less (along `v` the screw's turn makes it no
polynomial, and the rule read forty nodes a stretch); its area keeps the rule. A tetrahedron
screwed a turn and a half had its arrangement's 550 faces' fluxes read in 0.41 s, and now in 0.06 s
(`swept_closed::a_tetrahedron_screwed_is_built_to_the_benchs_measures`).
**A polynomial net's `G` is tabled from the face's own middle** (`props::Prefix`): `G` from any `u`
knot differs by a function of `v` alone, which closes to nought round a loop, so each `v` span's
column is filled out from the knot nearest the face's middle only as far as a `u` asked reaches, a
span at a time by the net's rows read at that `v` (the sheet and `S_v` B-spline curves in `u`
there) — a fitted sheet's face was integrated across its hundreds of spans per column, each node a
2D evaluation. **Every other reading of a net's face is Green's theorem cell by cell** (`props`'s
`Cells` and `by_cells`, #880): the area, a rational net's flux, a field's flux and a weighed area
on a B-spline or offset face take `G` along each stretch of a use — cut where it crosses a knot
line (`crossings`, a kink a sixteenth lands on exactly crossed there) so it lies in one cell — from the
knot line bounding its strip on the side of the face's middle, one span's rule, and where the loop
hands over from one knot to another at `v_c` add the strips' columns between, `± Σ H_j(v_c)`,
`H_j = ∫ I_j dv` tabled once per strip a whole cell at a time along `v` (the constant each table
starts from cancels: a closed loop crosses each knot line as often each way).  The cell holding a
crossing is read by its part on the face's side of it along the knot line, so nothing beyond the
trim is read: a sheet trimmed at its fold folds back past it, `|S_u × S_v|` kinked there, and a
cell read whole across it was 1.1e-7 off a face of the crossed forward member.  Each span across, cell
along and stretch is read by ten nodes where their Kronrod extension agrees (`checked`); the
cost is the cells the face spans and its uses' nodes, where `G` gathered across every span from the
middle cost each node the spans between — 34 to 50 million integrand reads a face of the crossed
forward member, its area 78 s, now 3 to 11 million and 18 s.  **A read costs a sum over the
poles, not a basis** (`Net::basis`, `Net::d1_by`): each way's basis kept where it is read again —
a column's nodes across, read at every node along in the same order (`Across`), and its basis along
once at each — `Surface::d1`'s very bits, and a polynomial net's area off its rates alone
(`Net::rates_by`).  **And a strip's whole cells are read on every core** (`Cells::columns`, from
`MANY_CELLS` on), each as one thread reads it, summed in their order, the reads the workers made
counted as the caller's: the same measure, to the bit, and the same reads
(`brep::a_nets_area_read_on_every_core_is_one_threads`).  The crossed reverse member's area, its
faces on nets of some 940 × 850 poles, 297 s to 32 s on four cores, each face's the same.  A face reaching a collapsed row
(a degenerate use's line, or a row all but collapsed) keeps `integrated`'s reading along the
parameter collapsed; a stretch is read as there (ten nodes, `checked`), and a use's ends closed
where its curve arrives; a line in the parameters is cut at every knot line it crosses, in closed
form.  A net creased at its knots needs nothing more: each cell's nodes lie inside it, on its own
span's side (`brep::a_creased_net_is_read_cell_by_cell`).  `brep::a_nets_face_is_read_cell_by_cell` (an annulus of a paraboloid on a
graded net, the polynomial net and the same net rational; the integrand's reads, `props::reads`,
no more than doubled as the knots are doubled in `u` and then in `v`),
`brep::a_polygon_on_a_net_is_read_cell_by_cell` (a pentagon with a square hole whose sides lie on
knot lines, a triangle with a pole row, a triangle inside one cell) and
`brep::a_rational_torus_is_read_cell_by_cell_across_its_seams` hold area and flux to their closed
forms within 1e-12. **An inverse edge's nodes are sought from the last** (`Pcurve::at_and_derivative_from`):
each node's foot from the previous node's, carried along by its rate, where that node is just
behind it, the branch still the one nearest the line between the ends — from that line, Newton
took some ten steps a node on a fitted sheet's curved edge.
**A swept sheet's face across its fold is read by columns** (`props`'s `by_columns`, #880 T1b):
where a carried section's sheet (`Surface::Blend`) folds at a face — its area factor
(`Surface::area_factor`, `|S_u × S_v|` over `|S_u| |S_v|`) below a millionth at a point of a use the
face's box is read from, no point inverted but those, the fold point's path — a point beside the
fold stands in the parameters at the square root
of its distance in space, and a use inverted onto the sheet there is off along the fold's way by
that much. Green's theorem reads it through `G`, gathered from the face's middle and no smaller
there: the ball screwed tighter than its radius had two such faces read 1.65e-7 and 4e-8 off
their own boundaries (by their edges, ½∮ x × dx), its volume 3.3e-9 off. Read as `∬ g du dv`
column by column in `u` instead — each column's crossings with the loops bisected on the uses' own
curves, `g` gathered between them, the columns cut at the uses' ends and turns — the error stands
where `g` itself vanishes, and the ball reads 2.4e-10 off
(`swept_loci::the_tight_ball_holds_at_the_kernels_tolerance`); its side is its loops' way round the
parameters. A sheet running round (a period) keeps Green's reading, as every other face does, to the
bit. (Gated on the area factor below `geom::NEAR_FOLD` anywhere across the box instead, a spline bar
screwed four turns, its sheets leaning so without folding, was read by columns at every reading its
trim made, its build eight times as long.)
**A traced edge inverted onto its face is measured across planes** (`props`'s `along_trace`,
#798): Green's integral is the curve's, however parameterised, so it is read not by the trace's
own parameter — which counts its points, its rate stepping at each, every stretch a rule of its own
and each node three projections onto both surfaces — but over spans of its points whose chords
turn within 15° of the first, each by its distance `σ` along its chord `n`: a node is where the
curve crosses the plane `n·x = σ` (`Traced::across`, Newton on both surfaces and the plane), its
rate `T / (T·n)` read at that point, as exact as the trace, so each span is one smooth integrand.
It is read on cells of its stretches, doubled until two readings agree to rounding or each cell is
one stretch, so its nodes crowd where the march crowded its points: spread evenly along the chord,
they missed the turn at a span's end, the tip of the sliver a plane through a screwed rim's ramp's
axis cuts from it, by 2e-5 of the span, and that ramp's volume, held to its closed form within 1e-6
(`swept_closed::planes_through_the_axis_cut_a_screwed_rims_ramp_in_closed_form`), now holds it to
3e-14 where the stretches' central differences held it to 3e-7. A span not crossed as a graph over
its chord (its rate past 60° from it, or a crossing off the stretch it was sought in) is read
stretch by stretch as before. A canal's
meeting with an end disc, a thousand stretches, measured in 9 s and does in 30 ms, and the same
whichever way the solid is moved, where the stretches' central differences moved it by 2e-10
(`brep::a_rod_grooved_by_a_balls_helix_is_measured_along_its_traced_meetings`, brep-bench's
swept/ball-thread-groove, its volume and areas in under 5 s, its end discs one area).
**A sheet a line square to a screw's axis screws into meets what the screw keeps in closed form**
(`ssi`'s screwed meetings, a `Blend` of a line and itself carried by a screw): a coaxial cylinder
in the helices of the line's points at its radius (`Curve::Iso`), one where the line passes at it
(the sheet touching it along the foot's helix, judged by the gap); another such sheet of the same
pitch in the helix of the point where the two lines meet carried level, none where they run
parallel there; a level plane in the one ruling at its height; any other plane in its cut
`S(−α/β, v)` (`Curve::SheetCut`, **a new curve kind**: `α + u β` the ruling's reach to the plane),
with the rulings it holds as lines, read about the nearest held ruling by differences so the cut
runs through the point it touches the sheet at as exactly as anywhere. **The cut crosses each ruling
it holds at that ruling's `v`, in closed form** (`SheetCut::held_along`, `query::crossings_in`; #885),
and the two curves of the meeting are crossed there (`crossings_among`), so an edge lying in the
sheet along a held ruling — a tool's edge at a sweep's end pose, its own sheet's ruling there — is
cut where the cut leaves it, by the vertex inside it, and the cut is laid from that point, not to
the face's corner (`arrangement::a_meeting_crossing_an_edge_lying_in_the_other_surface_cuts_it_there`).
Two curves on one sheet,
each a rule `u(v)`, cross where the rules agree, and two planes' cuts of two such sheets on one
plane where the plane meets the sheets' helix (`query::crossings_in`); a point's foot on such a
sheet starts from the ruling level with it (`Blend::inverse`), with no search
(`meeting::a_screwed_lines_sheet_meets_what_keeps_the_screw_in_helices`,
`a_plane_holding_a_screwed_sheets_ruling_meets_it_in_the_ruling_and_a_cut_through_the_touch`, at
1e-6, 1e-7 and 1e-9). An open sheet reaching past the domain is an operand of `arrangement_within`
as a solid is, its pieces within the domain parting cells.
`arrangement::a_plane_crossing_where_two_surfaces_touch_…`, `a_plane_tangent_to_a_hyperboloid_…`
and `a_plane_crossing_a_shallow_meeting_…` are the gates, at 1e-6 and 1e-7. **The closed swept
solid is one arrangement and one selection, and so is a sweep cut from a blank** (`swept::closed`,
`swept::closed::cut_from`: the blank the domain, its cells kept where they are not swept;
generating.md);
`arrangement::three_blocks_arranged_at_once_…`, `every_cells_membership_…`,
`the_arrangement_is_the_same_in_any_order_…` and `coincident_faces_are_one_face_…` are its own.
**What a cut from a blank costs is the sweep's, not the blank's** (#847). An arrangement may be met
within a reach (`arrangement_reached`, its operand 1): the operands after it are left out wherever
they lie outside it, each piece read against the reach — a box — before the domain, and the pieces
read across the cores. Two meetings lying along one another on a face (a screwed level edge's sheet
touching the cylinder its foot runs on along the foot's helix, which a second sheet meets the
cylinder along too) cross nowhere, and are not searched for crossings (`along_one_another`: every
point of one within `ON_CURVE` tolerances of the other — the reach at which a vertex cuts an edge it
lies inside and two edges along one curve are one, which then join them — over more than that
reach), where every pair of their chords had been cut — hundreds of vertices along one helix
(`screwed_copies::two_meetings_lying_along_one_helix_are_read_as_one_curve`). A screwed sheet's
piece, and a helix or a plane's cut on it, is boxed by what its parameters reach of the sheet — its sections
over the stretch, each corner's arc over the turn there, stepped by the advance — within the ring
the whole sheet turns in (`Surface::bounds_over`, `Curve::bounds`), where it had been boxed as that
ring, a sliver of a cell a hundred millimetres across whose interior was sought over all of it
(`screwed_copies::a_piece_of_a_screwed_sheet_is_boxed_by_what_it_reaches`). A shell's volume is its
faces' fluxes summed, each face's read once (`props::fluxes`) and turned with it, not each shell
integrated whole. A selection's edges a healed meeting cut are one again (`rejoined`): two pieces of
one curve, end to end at a vertex no other edge reaches and walked one after the other by every
face using them (a seam's two uses in one loop too), are one edge, the vertex and the sliver beside
a true corner gone. And work asked of the cores from a worker of theirs is done on that worker
(`util::par::threads` is one there), not on threads started for it, which a cell's interior read
inside a read of every cell spent most of its time doing.
**Work on a sheet graded to hundreds of knots each way is read once, and on every core** (brep-bench's
crossed tumbling tori, whose reverse member's sheets are nets of some 940 × 850 poles beside a spine
all but pinched a second time). A net's span index (`query::net_index`) is built once however many
threads ask for it at once, the others waiting for it, and a large one's rows and spans are read on
every core even from a worker (`query::indexed`): every worker asking waits on it, so the cores are
otherwise idle. A sewn shell is turned by its volume's sign (`sew::orient`), its faces' fluxes
estimated first where a face lies on such a net (`props::fluxes_bounded`: the midpoint rule on two
grids over its loops' box, weighed by their winding number, its bound the readings' difference twice
over and `|g|` a cell where each row crosses a loop) and the estimate taken only where every shell's
volume stands clear of ten times its bound — else every flux is read exactly, as on any smaller face
(`brep::a_large_nets_flux_is_estimated_within_its_bound`). A Boolean's traced meetings (`ssi::Traced`)
are read on every core before its face pairs are laid in order: each two surfaces' pairs traced in
their order, each knowing what its predecessors traced, from no warm start — the same arrangement
however they were shared out; a build under a budget traces them in order. A screw's pose is read in
closed form, one twist seen from another with no level between (`Screw::at`,
`brep::a_screws_pose_in_closed_form_is_its_composition`), and a STEP file's points written where they
go: each reading the same numbers, and the same file, to the bit. The tube road traces its two sheets
side by side once the collar is traced (`faces::solid`): a sheet's runs leave seeds on the pinch's
rim of its own side alone, so neither seeds the other, and what each leaves is read after in order,
as before — the crossed reverse member 15.4 s to 10.0 s, its file the same. A span's Bézier points are linear in
the poles reaching it, so a net's index reads each span's weights once each way
(`nurbs::blossom_weights`) and every row's and column's points by them, not a blossom a point; and
a large tree's first levels are grown on threads of their own (`util::bvh`, past 32 768 items), the
very tree one thread grows (`brep::a_large_tree_answers_as_a_scan_of_its_boxes`). A sheet's foot
(`geom::net_search`) is the thread's warm foot, without its spans' boxes searched, where that foot
stands within a thousandth of the sheet's size of the point — no box is nearer than nought, so the
search keeps it anyway — and where the boxes leave a point off the sheet unsettled, its nearest poles
are read through a tree of them (`query::net_poles`, past 4 096 poles) as the scan of every pole
reads them, the first in the rows' order among equals
(`brep::a_large_sheets_feet_are_its_nearest_points_warm_or_cold`): the crossed tori's Boolean
20 s to 12 s, the case 28.5 s to 20.6 s, its file the same.
**A Boolean's work is where its solids meet** (#589): a face of one whose box misses the box the
other's faces fill is clear of it (`boolean::Arranged::clear`) — nothing of the other meets it, so
it is one piece, its own loops, outside the other: kept by a union and by a cut of the solid it
belongs to, dropped otherwise, with no point of it read; an edge whose `Curve::bounds` misses that
box is not sampled against the other's faces. A face's outline is drawn the first time it is asked
about (`Located::drawn`), its box read once (`Located::boxed`), the solid's size once. A run of
Booleans on one solid (a body's operands, `recipe::body`) hands each step's `boolean::Known` to the
next (`combined_known`): the box of each face it carried whole, and which were checked as faces of
a result, which `Brep::check_settled` reads for their loops' ends and their edges' uses alone. A
plane's `Surface::inverse` and `d1` are its frame's coordinates, read before any angle; a straight
image in a face's parameters (`Pcurve::straight`) is its corners in `loop_area` and its ends in the
check. So a shaper's run of 270 bites against a blank (`bench bites`, in `make bench`) builds in 3.3 s
where it took 21.8; what remains grows with the blank's two faces every bite meets (each of 744 edges
by the end, traced again at every bite). `brep::a_run_of_cuts_hands_its_far_faces_on` is the gate.
**An end all but flush with a face is on it** (#836, `brep::flush`): before a body's written
operand is combined with the solid (`boolean::combined_written`, which `recipe::body` and the
export's `combine` over swept material call, at the Boolean's own tolerance; never a round's or a
chamfer's, built on the faces already, `combined_known`), each planar face of the
operand standing parallel to a plane of a face of the solid near it, farther than the Boolean's
tolerance but within `FLUSH` (a thousand) of them — a part in a million of the two solids' size,
wherever they stand — is moved onto the nearest such plane, with every face of the operand on its
plane: exactly, its plane, edges and corners carried along its normal and every other edge reaching
it a line along the normal lengthened or shortened, the faces beside it (planes, cylinders and
extrusions along the normal, which the move leaves where they were) given their uses anew. A face
whose walls are anything else stays as drawn, as does every end farther off. So a cut ending a few
nanometres past another's floor is cut on it and leaves no strip of wall
(`brep_flush::an_end_within_reach_of_a_face_is_moved_onto_it`,
`an_end_whose_walls_lean_is_left_where_it_is`, `brep_export::an_end_all_but_flush_with_a_face_is_cut_on_it`).
**A sliver is found by its boundary** (`sliver::slivers`): a face thinner than a bar somewhere — two
points of its boundary nearer than the bar, on edges sharing no vertex (no corner is a sliver,
however sharp: a round's run-out meets its fill's cap tangentially, and a seam cut in pieces passes
a vertex twice), where the boundary between them runs more than four bars and sixteen times their
distance (two loops, or one far round) — and an edge shorter than the bar of no such face; a pole
is none. Read off the B-rep, it is the same wherever the solid
stands (`brep_flush::a_sliver_is_found_by_its_boundary_wherever_it_stands`).
**The recipe is typed values, never text** (`recipe::Recipe`, `recipe::Node`): `solid::cad` and
every node writer build the nodes themselves, and every reader — this kernel's build and meridian,
`section`, the swept cutter and sector, `solid::exact`/`canal`, the native host's `Session` —
matches a `Node`; nothing is written as JSON and read back.  A profile stays as written
(`recipe::Section`, the numbers the native host hands on) until `Profile::of` reads it, and
`Profile::section` writes one back.  A new node or field is one variant or field and one arm of
each match.  Every writer writes its points through one `recipe::Writer` (millimetres, about the
world's origin or `cad::recipe_about`'s), which returns the numbers themselves. **A kernel error
has a kind** (`brep::KernelError`, `Kind`): `Brep::check`'s edge used as often each way, more than
once, is `NotManifold` (material touching along it: what `boolean::combined` and the body's
stratum act on), one used more often one way or never `Open` (a face missing: the kernel's own failure,
which the stratum never reads as rounds meeting; `brep::an_edge_used_twice_each_way_touches_and_one_used_once_is_open`), a recipe of another kind
than a route builds (`Recipe::meridian`, `Sectioned`) is `Refused` (its caller builds it
another way); its message is all it says, and it becomes a `String`
where a caller outside `brep` wants one. Never match an error's text.
**This kernel is every export's default** (phase 5): OCCT answers only `--kernel occt` /
`--stl-backend occt` / `SOLVENT_KERNEL=occt` (the oracle; refused in a build without OCCT), its
STEP checked by `step_check` against `Solid::of`. `gcs-cli tests/brep_oracle.rs` builds every
corpus recipe node both ways (where OCCT is off the bar a closed form or OCCT's own declaration
decides: a whole revolution of lines and arcs is held to Pappus, `pappus`; a Boolean whose edges
OCCT declares far coarser than its operands' — two tori touching in the hypoid's tooth space — to
that tolerance times its area, ours rebuilt at tolerances ten times either way within 1e-9,
`rebuilt`; a general surface OCCT builds for an analytic one of ours must lie on ours within its
declared face tolerance); `tests/brep_census.rs` holds a census and `brep_body_debug`;
`SOLVENT_BREP_DEBUG` narrates, `SOLVENT_BREP_TIME` times slow Boolean steps. OCCT's volume of a
STEP it reads (~1e-5) is no gate on our accuracy. Rung 1 is done: every corpus solid with a recipe
agrees with OCCT (volume 1e-9, faces by kind) but for named refusals of self-touching designs
(surfaces meeting under 1°, pinches).
**Phase 1 (OCCT's shape, our files):** `SOLVENT_WRITER=rust` reads OCCT's solid into `brep`
(`backend/dump.cpp` → `Session::brep_json` → `brep::json::read`; uses ordered by vertex and
parameters since OCCT's wire explorer misreads a closed edge) and writes STEP/STL by ours.
`Surface::BSpline` is a `nurbs::Net`, **rational where it carries weights** (`BSpline`/`Net`
`weights: Option<…>`, finite and positive; `None` divides nothing, so polynomial exports keep their
bytes; `nurbs::arc` is a circle exactly; STEP's `RATIONAL_B_SPLINE_*` complex entities;
`tests/rational.rs`, `brep_oracle.rs`'s OCCT round trip); `Pcurve::Curve` written exactly; `Edge::tol` is
measured and `check` honours it, so it no longer proves pcurves meet edges: a reader holds
tolerances to a bar itself. `props::fluxes` tables a polynomial B-spline face's `G` per span (a
rational one's Gauss stretch by stretch) and **closes every
loop in the face's parameters** across the kernel's gaps (unclosed, the volume moved with the
origin). The mesher is a neighbour-array CDT, refined for sag **and** for a facet turning from the
surface's outward normal (signed), never at a pole's corner, whose normal is degenerate — a
revolution's sphere holds its `v` to the poles (`build::revolve`: run a rounding past one, its
chart's normal turned over and the mesher refined into it without end, until float32 wrote the
triangles as none, `canal::a_half_balls_pole_is_meshed_as_any_point`, #684 D); slivers get a
point off the longest side; each point's place and normal are read once, whichever triangles
it is a corner of, and the refinement asks only the triangle's middles anew; points nearer than
1e-9 are welded (filed in a `util::space::Grid` of cells eight welds wide, each looked for in the
cells a weld about it meets — its own, mostly, where it was 27; with each point's place read once
for every pass, the same triangles, bit for bit, in a third less time), and a vertex a face's loops pass through twice at one place (a hole touching the
outer loop, #597) is one corner of its triangulation. **A face's loops, as sampled, are simple
polygons in its parameters** (`mesh::crossings`): two chords that meet anywhere but at the corner
they share — that cross (a crescent thinner than its arcs' chords sag, the drill's end face, #64),
touch, lie along one another from that corner, or run between the same two corners (a circle's
segment thinner than its arc's chord sags, its arc one chord, the very chord its line is: a boss's
end cap poking 28 µm out of a cylinder about another axis, #904 A9) — are split at their middles,
each edge's samples shared by its faces, until none do; corners are one as the triangulation
welds them, by id and place, and an edge the face uses twice (a seam, or a slit the face lies
either side of) is one chord each way, no fault (`brep::a_face_thinner_than_its_chords_sag_still_meshes`,
`a_segment_thinner_than_its_arcs_chord_sags_still_meshes`). **A use's last corner is read where
the triangulation lays it**, the next use's first (each use's last sample left for it): a file's
vertex gathers points its tolerance apart, so two uses' ends at one vertex may stand apart in the
face's parameters, and a crescent of two arcs each one chord, 3 µm apart at one end (CADGenBench
248's plane face #17), read with both ends was a thin triangle while the triangulation was given two
corners and no area — refused, and with it the body's volume and its mesh; read as laid, its two
chords run between the same two corners and are split (#993 stage 2,
`brep::a_crescent_whose_ends_stand_apart_at_a_vertex_meshes`). An edge is sampled from first pieces no
midpoint test can be blind to: a B-spline's knot spans, a family rail's pieces, a screwed rail a
quarter turn at most (`edge_params`; a coil of four turns has its ends, middle and quarter
points in one line, and was one chord). `Mesh::turned` counts facets facing against their
surface (thicker than a tenth of the bar); the export refuses any. **The refinement for a turn
short of that is bounded by a budget, and a facet the export would count is refined past it**, as
a sag over the bar is: a helical gear's flank, one face the gear's width long whose root bends
tight, spent the budget on its bends before a facet inside it turned right over was reached, one
a tooth, and which turned on the chamfers at its ends (#904 A7,
`helical_mesh::a_helical_gear_chamfered_at_both_faces_meshes_facing_out`). A face whose facets beside an edge face
against it — a sheet folding along that edge tighter than the edge's chords sag, which no point
inside the face can mend (a round rim screwed a short lead, its ramp folding against the wall its
side sweeps over about a tenth of a micrometre) — asks the chords those facets stand on halved, and
the faces are meshed again with the edges so sampled, up to `TURNED_ROUNDS` times
(`closed_characteristics::the_drum_screwed_bores_its_helical_groove`, its STL agreeing with the
field). **No two faces draw one diagonal**: each face is triangulated alone, its boundary's samples shared
by id, so two faces may each join the same two samples by an inner side (two samples of an edge
they share, not neighbours along it, cutting off the sample between) — a side of the written mesh
four triangles meet at, which a strict reader of an STL refuses though the B-rep is sound.  Once
every face is meshed, each face past the one that keeps such a side has its two triangles split at
the side's middle (`mesh::split_shared_diagonals`); a mesh with no such side is untouched, so a
placed copy's mesh is still a fresh evaluation's (`bindings`).  A seat tube's bore and its top
tube's wall beside the seat stays' ends (#904 / `shaft_drive` finding 22,
`frame_cluster::a_seat_tube_its_top_tube_and_both_stays_write_one_closed_mesh`).
**Phase 2 (the pattern ours):** `brep::pattern` turns a one-sector body into the whole (the
kernel's union is never made): sides matched by a pitch's turn, revolution faces keep the sector's
surface with pcurves shifted along `u`, pieces of one surface across a side become one face, a ring
keeps its last junction as seam. `Net::segment` cuts sheets exactly. `Built::mesh` meshes the
sector once (`mesh::mesh_with`) and shares seam points by index. Full verification reads our STEP
back by OCCT and measures it by `props` (1e-5; OCCT's volume held only to 1e-4).
**Phases 3–5 (the gear without OCCT):** `brep::sweep` builds an admitted swept body: blank from its
meridian region (`brep::planar`, `recipe::meridian`), cutters sectioned exactly (`brep::section`,
`planar::Chart`; revolutions about lines parallel within the recipe's `tol`, the sine between
the lines times the meridian's reach, as a solved line is; a stretch of a planar Boolean's piece is
on the other's boundary only if its middle and both its ends are, `planar.rs`), sheets traced and
fitted (`sweep::sheet`, `nurbs::interpolate_net`; a fitted
edge still in the blank between its nodes — a cutter's corner grazing it at stations where it is
elsewhere a fan the rows stop before, `contact_trace::charted` — is carried on past it by its own
surface, `Net::extended`, only where what that puts in the blank is within the fit's bar, else left
as it is: an edge in the blank divides nothing; `brep_export.rs`'s gear at 0.625), blank split
(`boolean::split`), cells judged by the field (`sweep::sector`; else `sector::whole`), sector
patterned. `brep::export::{exact,step,stl}` is shared by `solventc` and `app/export-worker.ts`
(`gcs_solid_exact`, and `gcs_solid_exact_note` for the sliver its solid holds, `brep::export::slivers`,
which the page says beside the file as `solventc` does, #836), so native and wasm files are
byte-identical: the core's trigonometry, `exp`,
`ln`, `pow`, `hypot` are its own (`fmath`, `x.dsin()` via `Det`; no platform libm). With OCCT and
`--verify-step full` the CLI reads our STEP back (`cad::read_back`). A face met twice is told apart
by `cutter::OCCURRENCE`; a trace ends off a sheet's patch (`Surface::off_patch`, a revolution's or
an extrusion's of a curve too); a revolution about the axis is kept by `pattern`'s turns; interpolations
past `nurbs::BANDED` points solve banded; a gross sheet falls back to chord length.
**Rung 3 (the app reads the exact solid):** a static solid's `EvaluatedSolid` is the B-rep's
(`solid::Exact`, `Sketch::exact_solid`, cached against `solid::reads`; `from_brep`), built about its
own origin (`cad::recipe_about`, every point written through `recipe::Writer`), faces named by document path (caps leading as the facet term's, so
edges are named alike), meshed no coarser than `BREP_RELATIVE_SAG` or 64 steps a turn, one
polyhedral `Prim`. The facet term answers what the exact path refuses (touching tori, empty
solids); `SOLVENT_SOLIDS=facets` throughout. Silhouettes on curved faces are traced on the surfaces
(`EvaluatedSolid::silhouettes`), never the mesh's seams, which zigzag.  **A body a generating sweep
cuts is `exact_solid`'s too** (#536): built as the export builds it (`solid::exact::swept`) where
the sketch builds its own solids; where a host builds them (`FieldMeshing::Deferred`, `Supplied`)
it is the host's build supplied, and until one is the refusal names `brep::export::Builder`
(`object_builder.rs`'s `exact_solid_builds_a_generated_body_as_the_export_does`).
**A face family's walls** (#243 L2, `brep::family`): `Surface::Family` over a `Sheet` — an
edge's kind (`Run`: line, round, spline), the pieces of the interval with six nodes each
(`Stations`, the Vandermonde inverses kept), and the edge's numbers at every node — `S(u, s)` the
edge's own form in `u` (never differenced) of numbers that are, between nodes, the degree-five
polynomial through them; `S_s` the polynomial's own derivative, a rail's `S_ss` a central
difference of it (as a loft's rail's).  `u ∈ [0, 1]` (periodic for a lone circle), `s` the running
number; `breaks` the pieces' joints, so `props` integrates each piece exactly; `bounds` by each
number's polynomial range (constant term ± the rest), never a sample; `inverse` per piece nearest
box first, warm from the thread's last foot.  Rails are `Curve::Rail` (the wall at `u = 0`,
parameter `s`), rims at the ends `Curve::Line`/`Circle` where they are one, else `Curve::Rim`; an
edge vanished at an end is a degenerate edge, its rails meeting in one vertex, and a section
vanished whole has no cap.  **The fast path** (`lower`) puts a wall on a plane (a line edge in one
plane), a cylinder or cone (a line turned about one axis, or coaxial arcs square to their axis,
radius constant or affine in the height) or a torus (an arc turned about an axis in its plane) —
read off the numbers, never a word — kept only when the sheet's points at every node lie on it
and the face's area there (`props::area`) agrees with the sheet's, both within `AGREE` (1e-10).
**A section carried rigidly** (`carried`, #243 L6): where every station is the first one moved
by one screw (a turn about one axis and a step along it in proportion, a turn alone included) or
by one step along a line — read off the section's points at every node and both ends, the turn's
axis off the station turned farthest, its centre off the one nearest a half turn, each turn
unwound from the last, every point then held to the motion within `AGREE` of the size and the turn
running one way — the solid is built **exact throughout**: each rail the corner's path
(`Curve::Iso` of the swept wall, parameter `[0, 1]`), each wall the analytic surface where it is
one, else the surface its first rim sweeps (`swept`: `Surface::Blend` of the rim with itself under
the `Carry`), each held as above against the solid on its sheets (`general`, edge for edge);
where any wall will not lower, the general solid is kept, lowered wall by wall.  The sheet is
exact only at its nodes (between them within its piece check), so the rails must be the motion's
too: on the sheet's rails a lowered face's area missed by 2e-10.  `Blend::inverse` starts warm
from the thread's last foot on it where that settles near the point (as a family's wall does: the
evaluated solid asks at every triangle's corners, and searched afresh the spring meshed 6× slower),
else seeds each `u` where the screw turns it to face the point (`turned_to`; a grid along `v` alone
starts on a coil's next turn), and a rail (`Curve::Iso`) is read the same way, the nearest of
its samples and of those turns, one seeding for both (`Blend::nearest_v`) (#886: a rail of 5.8
turns rising 6 a turn, its 33 samples 16 apart along it at the radius 14.4 of the helix two sheets
meet a cylinder in, was read a turn off, so `same_edge` never merged that helix's two pieces and
the cylinder's pieces did not close;
`swept_closed::a_rail_screwed_many_turns_is_read_on_its_own_turn`,
`two_sheets_meeting_a_cylinder_in_one_helix_split_it_once` and, slow tier,
`the_square_bar_screwed_four_turns_closes_into_its_helical_sweep` are the gate). A fitted section
carried by its own carry (a piece of a tool's contact carried over a roll, `contacts::carried`) is
read off the motion's invariants before any warm start (`Blend::carried_uv`, #880 Q1b): a slide keeps
a point's shadow on the plane square to it, a turn its radius and height about the axis, a screw its
radius and its height along the helix modulo a turn's rise; the section's points, eight a span, are
read once a section (`carried_samples`, kept by the spline's address with the spline), every local
least of their gap from the point in those invariants a seed, the four least closed on by golden
section, each polished by Newton and the nearest in space kept. Seeded from sixteen points round
the section, the leaning torus's contact under `torus-screwed-through-rod`'s screw — one closed curve
carried 456° — was read 0.13 to 2.2 off at points of its own sheet
(`swept_contacts::a_contact_carried_over_the_roll_is_inverted_exactly_at_its_own_points`); `inverse_near`
runs Newton from its hint first, and a foot it settles with `v` held at the patch's edge is
polished in `u` alone there (the coupled step, clamped, settled a point of a sheet's end rim 3.8e-8
along it where the sheet's two rates nearly agree, a loop broken in its parameters at the kernel's
tolerance; stepping `u` alone at every step held at the edge feet the coupled step leaves for the
inside, the box screwed inside a rod 0.015 off, `swept_closed::a_box_screwed_inside_a_rod_is_a_sealed_tunnel_of_its_volume`,
#880 Q1b S5); `step::blend_net`
fits quintic along a screw a cubic cannot follow at the fit's cap.  A twisted duct, a thread, an auger's
flight and a coil lie on blends; a section that changes as it runs stays on its sheets
(`tests/helix.rs`, `a_rigid_screw_lies_on_its_swept_rim`).
**Creases** (#243 L7, `creases`): a joint of the pieces at which some wall's `S_s` turns, read
from either piece (`Sheet::rate_in`) at five places along it, by more than `CREASE` (`1 − cos`,
1e-6) is a section the walls meet at an angle — a ruled loft's, where a family runs chains in
step.  The solid is built in **bands** between them (`Band`, `Stations::band`/`Sheet::band`: each
wall's pieces of the band alone), each wall a face a band under its one name, each band's rims
edges shared with the next, rails a band at a time; the caps and the vanished ends as before, and
each band's walls lowered on their own (a ruled bottle's two cones).  A joint within `CREASE_GAP`
of another or of an end is none: the pieces closing in on a law's kink (`max`) stay in their face.
A family in bands is never carried rigidly.
STEP writes a sheet as the net fitted within `step::FIT` (`family_net`, cubic along the edge and
quintic along `s`: a cubic through 512 rows misses a 3½-turn auger by 1e-6), rails and rims as
cubics within it.
**A stretch of a traced or formula curve in a profile is held as near as a surface the kernel
fits** (`cad::HELD_MM`, which is `step::FIT`, 1e-6 mm; `brep::cubic::fit_held`): the stretch is
what the solid's face is carried along or turned from, and a reader measures the solid by that
face, so the bar of what may be refused (`cad::FIT_MM`, 1e-4 mm) is no licence to stand that far
off — a hyperbola turned, fitted within `FIT_MM` alone, read 1.03e-6 off its closed form where
brep-bench grades to 1e-6, and within `HELD_MM` reads 1.7e-9 off. The samples are doubled until
the fit stands within `HELD_MM`; where a doubling no longer brings it four times nearer (a cubic
through a smooth curve comes sixteen times nearer, so the fit is at the curve's own roughness) or
its spans pass `cubic::HELD_SPANS`, the nearest fit met within `FIT_MM` stands, and only a curve
with no fit within `FIT_MM` is refused, as before. `tests/solid_lang.rs`
(`a_stretch_turned_is_held_as_near_as_a_fitted_surface`, a hyperbola stated by its foci turned, to
1e-8 of its closed form;
`a_fit_is_held_where_it_closes_in_and_stands_within_its_tolerance_where_it_does_not`) is the gate.
Rung 2 is done: `Curve::BSpline` (`brep::nurbs`), `Surface::Extrusion`/`Revolution`, a face's
stretch of a traced or formula curve (`k from p to q`, `CurveE::trim`, `cad::HELD_MM`), and a
section carried to its own image (`build::ruled`: a drafted prism's offset — its corners where an arc
meets a side at an angle on the conics they move along, `build::ruled_by`, #962 — a canal's screwed disc;
`Surface::Blend`, `Curve::Iso` rails, `step::FIT`), each edge joined to the edge it became at equal
parameter fractions.  Two sections drawn apart are no input to it (#243 L3): what a section becomes
along a path is the face family's (`brep::family`).
`curve_surface` drops roots off the surface. The meter reads a static body by `accuracy::Exact` and
a field taking splines as chords within `CHORD_SLACK`. Edge sampling and `curve::tessellate` test
quarter points (a midpoint misses a cubic's S-bend).
Phase 0: `SOLVENT_ABI_TRACE=FILE` logs native entry points, Boolean face pairs
(`backend/probe.hpp`, `BOPDS_InterfFF`) and smallest pieces; `rust/gcs-cli/tools/abi_trace.py`.
**A B-rep is reflected by an improper motion** (#578, `Brep::reflected`): what a mirrored
instance's solid is, its body's own B-rep moved by a rigid motion of determinant −1. Every point
is the reflection's and every parameter its own: a vertex and an edge's curve moved (`Curve::reflected`
— a line's and a B-spline's poles; a circle's and an ellipse's frame made right-handed by its normal,
which its points do not read; a traced intersection between its two surfaces reflected), and each
face's surface reflected and placed right-handed again by the one rule a native kernel's
left-handed placement is read by (`Frame::rehanded`, `brep::json`): a plane and a B-spline sheet
by their normal `z = x × y`, an extrusion by its section's `y`, and a surface of revolution
(cylinder, cone, sphere, torus, a curve turned) by `y = z × x`, which runs its `u` the other way,
so its pcurves' `u` is negated (`Pcurve::u_negated`, a circle's and an ellipse's in its parameters
too). A reflection turns which way a loop walks about the material, so every loop is walked the
other way round (`reverse_loop`); and it turns `S_u × S_v` against the material normal, which a
revolution's `u` run the other way turns back, so a face's sense is turned exactly where its `u`
was not. Every edge is still used once each way, and `Brep::check` holds. A blend's or a family's
face (a swept section's), whose carry a reflection would turn the other way, is refused by name.
`tests/mirrored.rs` is the gate (`a_reflected_brep_holds_over_planes_cylinders_cones_and_a_torus`:
the check, every placement right-handed, every vertex its preimage's reflection and the volume;
`a_reflected_faces_material_side_pcurves_and_loops_follow`: on a B-spline sheet and every face of a
turned part, a point and its material normal reflected, each pcurve on its edge, each loop's
signed area in its parameters agreeing with its face's sense), and `gcs-cli/tests/mirror_oracle.rs`
in the native slow tier (`OCCT=1`) holds it to OCCT's `gp_Trsf::SetMirror`
(`solvent_cad_mirror`): the body built by OCCT where it was solved, mirrored there and dumped back,
its volume, its faces by kind and every vertex ours.

**Faces deleted are a gap their neighbours close** (`brep::defeature::deleted`; brep-bench's
`edit/` delete cases).  The faces sharing an edge with a deleted face are each run on as a sheet
of its own surface, over its face and past it as far as the solid reaches and half as far again (a
plane over the reach's extent, a cylinder or a cone over a whole turn and the reach along its axis
— a cone stopped short of its apex by a twentieth of the solid unless a face reaches nearer — a
sphere or a torus whole; any other surface refused by name), faces on one surface
(`fillet::one_surface`, a Boolean's rule) sharing one sheet.  The solid and the sheets part space into cells (`boolean::arrangement`); a **gap** is a
bounded cell bounded by a piece of a deleted face, a piece of a sheet run on past its face, and
nothing else but pieces of the faces round the deleted ones — no face the solid keeps that is not
round them.  The result is the solid's cells with every gap turned over, so a gap outside it (a
round on a convex edge, a hole, a flat) is taken in and one inside it (a round in a concave
corner, a boss) given up; nothing is classified by shape.  A deleted face left between a cell kept
and one not closes no gap, and is refused naming it (a box's top: its sides run on and never meet).
Each sheet's pieces are read as pieces of its first face, and the faces on its exact surface as
one (`boolean::selection_by`, the complex left as it was), so the selection's healing joins a face to its run-on piece (a top over a round's corner, a side over
a notch's mouth, a cone over a groove) and the result has the faces it had before the feature;
where a hole's rim and the disc closing it are each a loop of that one edge, both go (`heal`).
`tests/defeature.rs` (a convex round and a concave one, a hole, a boss, a ball's flat, an
L-block's step, a notch parting a side, a box's top refused) is the gate, and brep-bench's 47
delete cases its corpus.

**Offsets are the union of the pieces their points are nearest** (`brep::offset`; brep-bench's
offset, shell and thicken cases).  A point outside a solid within `d` of it is nearest a point
inside a face, on an edge or at a vertex, so the solid grown by `d` (`offset::grown`) is the solid
with each face's **normal sweep** (`normal_sweep`: every `q + s n(q)`, `s` over `[0, d]`), the tube
of radius `d` about each crease the faces part at, and the ball about each corner nearest which
some point is — a corner where two or more such creases meet and none turns the other way, or a
corner of no crease whose faces' normals differ there (a cone's apex; `smooth_at` reads each face
just inside it, so a sphere's pole is smooth) — every piece within `d` of the solid and every such
point in a piece, united by the Booleans.  **A face no prism or band of revolution takes is carried
between its moved copies wherever its surface moved along its normal keeps its parameters**
(`offset::carried_sweep`): `Surface::normal_moved` is the surface whose point at `(u, v)` is
`S(u, v) + h N(u, v)` — a plane moved, a cylinder's, a sphere's or a torus's radius grown, a cone
stood `h sin α` back along its axis with its radius there grown by `h cos α` (its radius at the
origin less than nothing where the origin falls past the moved apex; STEP places it at its apex,
`step::cone_written`), a sheet's offset (`Surface::Offset`, `S + h N` read exactly over the sheet's
own parameters, though it has no closed form), a spline swept along a line read as its sheet — and
`Surface::offset` makes no such promise (a cone's keeps its frame and shifts `v`; an ellipse swept
along a line is read over its rational spline's parameter, and its face is refused).  So the face
moved to `h` is the face on that surface with its own pcurves (`offset::moved_face`), each edge the
edge's curve carried along the normal at its foot with its parameter kept (`Curve::Parallel`,
`parallel::Parallel`: `q + (P − B)` at the foot, so it stands exactly `h` off `q`; its rate by
Weingarten's partials; its foot and the foot's rate the face's pcurve's where that is a line or a
curve of the parameters, `parallel::Foot`, searched for only under an inverted pcurve; written to
STEP as the cubic through it), and the wall an edge sweeps the `Blend` of the edge and its image at
one parameter — the normals along the edge, exactly (a plane where they lie in one).  **On a plane,
a cylinder or a sphere the move is one affine map of space**, linear in `h` (a step, a stretch
square to the axis, a scaling about the centre), so there the face moved is its image instead, its
edges' images of the kernel's own kinds (an ellipse's an ellipse), which the closed forms of the
meetings and a file's exact curves take — and the wall stands on the face's neighbouring plane along
the edge and along each ruling where its far image crosses that plane, and **a plane meets a ruled
sheet one of whose rails lies in it exactly in that rail and those rulings** (`ssi::rail_in_plane`:
`(1 − v) α(u) + v β(u)`, the rail's height `α` nothing all along, is `v β(u)`; traced, the rail's
ends missed its vertices by 4e-6 and the plane's pieces did not close).  The move stretches area by
`(1 − h κ₁)(1 − h κ₂)`, a quadratic in `h` read exactly from the moved surface's partials at three
distances, and where its least over the sweep reaches nothing on a grid of each knot span of the
face's box of parameters (and of each stretch between where a torus's curvature across its tube
turns, its equators; a cone's runs one way along each generator) the sweep folds and is refused
(`offset::unfolded`, every carried face); a cone's face reaching its apex, where it has no normal,
is refused; a sweep that runs into itself across the face, a fold no point's own curvature shows,
is not detected.  Gates: `brep_offset::a_spline_sheet_is_thickened_on_its_exact_offset`
(brep-bench's parabolic trough, its closed forms to 1e-9, and refused past its vertex's radius of
curvature), `a_sheet_curved_both_ways_is_thickened_to_steiners_volume` (a paraboloid sheet, its
ruled walls blends, against Steiner's `∫ (t − t² (κ₁ + κ₂)/2 + t³ κ₁κ₂/3) dA` by Gauss's rule),
`a_cone_cut_aslant_grown_is_steiners` (a frustum cut aslant, a convex body's Steiner formula),
`a_torus_cut_aslant_shrunk_and_its_face_thickened` (a ring two spiric curves bound, shrunk the ring
of the tube shrunk under the plane moved in, and its torus lifted out and thickened either way,
`∫∫ ∫ (1 + h/r)(1 + h cos v/ρ) dh dA`, each over its meridians in closed form — grown, its traced
creases' tubes are not held to a family's `EXACT` yet, and `offset::swept_tube` refuses them), `a_spline_extrusion_cut_aslant_is_thickened` (the trough's profile swept and cut
aslant, its side lifted out and thickened, `∫ (t − t² κ/2) dA` by Gauss's rule, and refused past
its vertex's curvature), `a_cone_placed_at_its_apex_is_thickened_inward` (onto a cone whose radius
at its origin is less than nothing), and the same solids among
`step_roundtrip::the_sample_reads_back`'s.  **An extrusion's face is the rectangle its line sweeps
along its normal, carried along its curve** (`offset::extruded_sweep`): the normal of `c(u) + v z`
stands square to `z` and alike all along each line, so the face's sweep is the face family of four
lines in the plane of `z` and `n(u)` as `u` runs, closed on itself round a closed curve
(`family::ring`, a loop of any walls, its walls on a plane put on it) — the offset of an ellipse or
a spline, which has no closed form, its moved face exactly `c(u) + s n(u)` swept along `z`; as a
distance set's piece the rectangle reaches back past the face a quarter of the distance (or less,
clear of the fold), every point of it within `d` of the solid, so no Boolean meets the face along
its own surface; refused where the sweep would fold, past the curve's least radius of curvature
(`brep_offset::an_elliptic_cylinder_grown_and_shrunk`).  **A spline face that lies on a sphere is
put on it** (`offset::on_spheres`, every offset's first step): the sphere's equation on a span of a
net of degrees `du`, `dv` is a polynomial of degrees `2 du`, `2 dv`, so vanishing on a grid of
`(2 du + 1) × (2 dv + 1)` points of every span it vanishes on the sheet (`sphere_of`); the face
keeps its edges, its pcurves read again on the sphere, its axis through a pole the net collapses
to, and is held to its area (`recharted`) — a ball is the set, not the net that drew it
(`a_spline_ball_grown_is_a_ball`).  **A crease along a curve no closed form carries a tube
along is the ball carried along it** (`offset::swept_tube`, `swept::canal::tube`), its ends flat; a
closed one's is a ring with no end (`swept::canal::ring`, `family::ring`), so each face's moved copy
of it lies along the ring's wall whole; refused by the tube's rows where the curve turns tighter
than the distance, or where it closes at a corner (`brep_offset::a_cylinder_cut_aslant_grown_is_steiners`,
`a_ball_cut_aslant_grown_is_steiners`).  **Where the tube cannot be built, the crease is joined by
the sector of it its faces' normals span** (`offset::sectors`): every point nearest the crease
stands in it, and it lies on the faces' outward side, where the curve's centre of curvature is not,
so it does not fold where the tube would (a cylinder cut at 45° grown by 4, its ellipse turning on
2.12).  It is the face family of the arc of the ball's disc between the two normals and its two
radii, widened past each normal into the face's own sweep by an eighth of a turn and its apex stood
back into the solid by a quarter of `d` (or less, clear of the fold) — the hull of an arc of the
disc and a point of it lies within `d` of the crease — so it holds the crease inside it and meets
its neighbours across, as the tube does, not face to face; a closed crease's is a ring with no end
(`family::ring`), so each moved face's copy of the crease lies along its arc's wall whole; a
sheet's, exact between its normals, along any curve.  Shrunk (`offset::shrunk`) is the dual, the solid less
the inward sweeps and the tubes and balls of its concave creases and corners.  **A round
swept past its radius stops at its centre**: past the centre of its section's circle (a torus's
tube, a sphere's) the moved face turns inside out, and a point there is no nearer the face than the
focal distance allows a nearest point to be, so a distance set's piece (`Reach::cut_at_axis`) is
cut there and the far side's faces own what lies beyond — a concave fillet grown past its radius,
a convex round shrunk past it, in one step (`brep_offset::rounds_offset_past_their_radius_in_one_step`);
a sweep in its own right (a sheet thickened) past it is still refused.  A crease's tube as wide as
its circle is a horn torus, its section's arc closed on the axis as a sphere's meridian is (a plate
grown by its hole's radius).  Nothing is
classified by shape: where moved faces meet (a groove grown shut, a flange shrunk away, a torus
grown past its hole into an apple) the union decides, as the definition does.  **Sharp joins**
(`Join::Sharp`) are the other set an offset is asked for, its faces moved and extended until they
meet: along a line between two planes the prism of the kite between the edge, its moved copies and
the line the moved planes meet in; at a corner of three planes the parallelepiped the moved planes
and the faces' own bound; anything else refused by name.  **A shell** (`offset::shelled`) is the
wall within `t` of the faces not left open — inside, the solid in common with the pieces of those
faces; outside, the pieces less the solid, **each piece that reaches an opening ended at its plane,
and no other** (`offset::ending`, `offset::ended`: the pieces ended alike united and cut once, so
the plane holds one face where it ends them, a box's rim one ring; brep-bench's shell/l-block-open-at-its-step-outward-sharp, whose
tall leg's walls, past the step's plane, the whole plane once cut away: 904 for 1868).  A round's
joins along an opening's edges and at its corners are kept on the solid's side of its plane; a wall
or a join meeting it square ends in it and is left whole, so the riser of an L open at its step
stands on the opening's edge and its floor runs on past the plane; one lying in the plane is the
opening's neighbour's, and left whole too.  A rounded wall meeting the plane aslant is kept on the
side it stands on, and what it falls short of the plane is the opening's own round's to fill (its
tube along the shared edge, its ball at the shared corner), or it is refused; a sharp wall aslant,
whose moved face would have to be carried on to the plane, is refused, as is a wall aslant across
it, never cut where it should not be.  Inside, a moved wall
runs on across a face left open until it meets it (a curved face's sweep carried across that side
of its box of parameters to the end of its surface's range, `Reach::across`), which the solid then
trims: in a ball's cap the cavity is the inner ball whether the opening reaches it or not.  **A
sheet** (`offset::sheet_of` lifts faces off a solid) **thickened** (`offset::thickened`) is the
union of its faces' sweeps over `[s0, s1]`, its creases joined only by the wedge of the tube between
the two moved faces and its corners by the ball's sector their normals span, its rims swept straight.
**A sweep** is exact: a plane's the prism of its loops (an arc's frame turned to the profile's
normal); a cylinder's, cone's, sphere's or torus's bounded by its own parameter lines the section
between its meridian and its moved copies, turned through its span (a whole tube's the tube of the
far radius less the near one's, each cut at the axis).  Past a surface's axis the section stands on
the far side of it: for a sweep in its own right that part mirrored and turned half a turn on is
added (a bend thickened past its radius); for an offset's piece it is cut away
(`Reach::cut_at_axis`), since a point there is nearer the surface on the axis's far side and not
this face's.  Any other face of those kinds is carried between its moved copies (above), and so is
a spline sheet's and a spline extrusion's; a surface whose offset keeps no parameters of its own
(an ellipse swept along a line, its face not bounded by its parameter lines) or is not read (a
curve turned, a blend, a family's wall, an image) is refused naming the face and a point on it —
never approximated.  A known limit of
the Booleans, not a rule of offsets: a meeting through a sphere's pole is read less exactly than one
clear of it (a pole near a corner ball's kept boundary put a grown tetrahedron 9e-7 off), so a
corner's ball is built with its poles along the corner's mean normal, clear of the circles the
tubes end on; the deeper fix splits a meeting at a chart's degenerate point.  `tests/brep_offset.rs` is the gate: a box, an L, a tetrahedron, a cone, tori past their
hole and shrunk, a groove filled and a flange taken away, boxes and caps shelled, an open box and a
bend thickened, an L shelled outward open at its step, sharp and round, a block open at its
sloping top, rounded, a cylinder cut at 45° grown past its ellipse's curvature, a cone cut aslant
grown, a torus cut aslant shrunk and its face thickened, a spline extrusion cut aslant thickened, rounds offset
past their radius, an elliptic cylinder grown and shrunk, and a spline ball grown, each to its
closed form to 1e-9; sheets thickened past their curvature, an elliptic cylinder shrunk past its
curvature and that block shelled sharp refused.

**A closed spline meeting is one edge round** (brep-bench's geometry/elliptic-hole): a plane square
to a spline's sweep meets it in the spline carried along the sweep, its weights and knots kept
(`ssi::intersect`; a rational rim carried without its weights is another curve, on neither
surface), and a clamped spline whose ends meet, running round a loop far wider than the tolerance,
is split as a closed curve is (`boolean::closed_domain`), its last stretch running round to its
first — not dropped as an open one's past its last point, which left an elliptic bore out of its
plate and one use of a cylinder's seam unmatched where a branch touches it inside at one point
(`a_tee_tangent_at_a_point_unites`).
**A face's parameters are scaled by its speed at their box's middle unless the chart nearly
collapses there** (`brep::mesh`): a horn torus's pinch, every `u` one point, would shrink `u` to
nothing over the whole face; there the mean speed over the box is taken.
`brep_offset::an_elliptic_bore_cuts_through_its_plate`, `a_horn_torus_meshes_within_its_sag`.

**A loop through a pole is closed along it** (#797; brep-bench's singular/sectors-sharing-an-apex).
Two cones with one apex meet in the generators of the first that the second holds, exactly
(`ssi`'s `cones_at_one_apex`: `A + R cos(u − φ) = ±cos β`, none where the cones touch along a
generator); traced, the meeting of two spherical sectors sharing their apex crowded its steps
toward the apex and was fitted with 36 000 knots a curve, and each face's area took seconds. A
pcurve read through the surface's inverse ends on a pole at the value its line from `a` to `b`
gives (`topo`'s `on_pole`: at a pole every value of the collapsed parameter is the one point, and
the inverse reads one value whatever the curve, a cone's apex `u = 0`), and a Boolean's result
closes every loop that passes through a pole along it (`Brep::close_poles`): each use the inverse
carries there ends at the value its tangent points along, and between the use arriving and the
use leaving a degenerate edge runs from the one to the other — an operand's pole edge kept, cut
to the span the face reaches, or one laid — as a primitive's loop does. It mends the parameters alone: which use follows which at the pole is the face's split as walked. Unclosed, a cone's piece
through the apex was walked across the pole by a jump in its last use's last instant, or by its
operand's pole kept whole across a turn the piece does not span: its pieces' triangles overlapped,
265 triangle sides of the sectors' common not shared by two. `brep::two_sectors_sharing_an_apex_meet_in_lines_and_mesh_closed`
(the common's volume and areas their closed forms by Gauss–Bonnet, its mesh closed) is the gate.

**A face is split along a meeting through its pole** (brep-bench's singular cases: a ball halved
through its poles, a ball cut aslant through one, two balls meeting through a pole of each, a
napkin ring with its ball's poles on its rims, a cone halved through its apex). In the face's
parameters a pole is a segment, its degenerate edge running along the collapsed parameter, and a
meeting through it arrives on that segment where it runs, not at the segment's ends: an edge laid
across a face (`FaceCut::internal`) ends on a pole at the value it arrives along
(`topo::leaving_pole`, the foot of a point a hair out along its tangent, which `close_poles` reads
too), and each pole edge of the face is cut at every such value inside it (`boolean::poles_cut`),
its pieces no edges of their own (`Half::pole`: a piece's loop leaves them out, and the result's
`close_poles` — a Boolean's, a split's and a selection's — lays one between the uses arriving and
leaving). Read at the pole itself, the halving plane's meridian ran aslant across the sphere's
parameters from one pole to the other and the pole edges were whole: the sphere came out one
piece the meeting only slit, its sides in two cells at once, and the Boolean was refused ("two of
its cells joined round an edge"). **A plane through a cone's apex meets it in the generators it
holds**, exactly (`ssi`'s `plane_through_apex`: `R cos(u − φ) = −cos α (n·z)`, two where the plane
leans to the axis by more than they do; one, a touch, read as above; none left to the trace);
traced, a plane cutting a cone aslant through its apex crowded its steps toward the apex past a
hundred thousand points and was refused there as a near touch. `brep::a_ball_cut_through_its_poles_is_its_part`,
`meetings_through_a_balls_pole_split_it` and `a_cone_cut_through_its_apex_is_its_part` are the
gate, each case to its closed forms to 1e-9 and meshed closed at the bench's sag. **A half's point
at a pole takes the half's own value of the collapsed parameter** (#962's 7, `boolean`'s
`uv_seeded`): there the surface's inverse reads it as it likes — a sphere's `u` as 0 or 2π — and
a piece whose boundary runs along the seam's meridian into the pole and out along the opposite one
was drawn from 2π back to π across itself, its area read with the wrong sign: taken for a hole,
the face was refused ("a sphere has a hole in no piece"), a slab on a ball with its face on the
seam's meridian, a vane's side through a crowned disc's axis, a fill over a crown's top
(`a_cut_through_a_pole_along_the_seams_meridian_parts_the_sphere`, to its closed form to 1e-9;
`sphere_rounds`).

**The convex hull of points is a solid of plane faces, one a facet** (`build::hull`;
brep-bench's offset/tetrahedron-grown, swept/box-slid-aslant, loft/square-to-octagon and the
octahedra are each the hull of their corners): built with its topology whole, no block cut and no
Boolean. The triangles are found exactly, by quickhull on the filtered orientation test
(`util::predicates::orient`, moved down from the solver's Delaunay machinery, which re-exports
it): a point is placed beyond a triangle only when it stands strictly past its plane, so what lies
on one plane never decides by rounding, and a point on a triangle's plane is no corner. They are
then made faces at the kernel's tolerance of the points' box: a face grows from its largest
triangle across each edge to a neighbour whose corners all lie within the tolerance of *that
first triangle's* plane — never a run of neighbours each nearly level with the next — and only
while it stays a disc (a neighbour met along one edge adds a corner the face does not have), so
its rim is one cycle; a corner on the edge between two faces only, within the tolerance of the
line through its neighbours there, is dropped from both. Points within the tolerance of one kept
are merged into it (`tolerant::Pool`, the first given kept); each face is the plane of its
largest triangle, its edges lines between its corners, and every corner a point given. Fewer
than four points, points on one point, one line or one plane (within the tolerance), and a point
not finite are refused by what they are. `brep_hull::the_platonic_solids_are_the_hulls_of_their_corners`
(the dodecahedron's pentagons on their planes only to rounding),
`points_inside_on_a_face_or_along_an_edge_are_no_corners` (a cube's corners given twice, a hair
off, with its centre, its faces' and edges' middles and a lattice through it: eight corners,
twelve edges, six faces), `the_benchs_slid_box_and_lofted_octagon_are_hulls` (to the bench's
closed forms), `a_cloud_of_points_gives_one_hull_in_any_order` and
`points_bounding_nothing_are_refused` are the gate, each hull checked closed, every point given on
or beneath every face, no two faces across an edge one plane, and `V − E + F = 2`.

**Faces built apart are sewn into a solid by tolerant topology** (`sew::sewn`; brep-bench's
geometry/nurbs-cone): a rational net whose last row is nine copies of its apex (`build::sheet`)
and the disc closing its base (`build::face`, a profile's region of its plane alone), or any
shell's faces, joined where their boundaries are one curve, nothing intersected. The rules are
`tolerant`'s, against the tolerance the pieces were built to, in one order: an edge whose curve
lies within it of its start is a pole, a degenerate edge its face uses once (a B-spline's by its
poles, which hold it); vertices merged by the pool; a boundary edge cut at another's vertex
(`tolerant::on_stretch`, by `boolean::cut_with`, as a Boolean cuts its edges), so a side held in
pieces pairs one to one; two boundary edges between the same vertices one edge where the middle of
one lies on the other (`tolerant::same_edge`, a Boolean's own test) — a third along them
`NotManifold`, none `Open`. The curve kept is the one bounding the face hardest to invert onto
(`Surface::inversion`: searched over a closed form, a closed form over a plane), the face given
first where they are alike, so its pcurve stays as built and the other side's is its own
where the curves are one parametrisation, else the kept curve inverted. Each edge's tolerance is
measured and one standing past `ON_CURVE` off a face refused, never widened: what holds the rest of
two curves matched by their middles. The faces are turned
to agree across every edge — a one-sided shell refused — each connected set a shell turned
outward, a void (inside an odd number of the others) inward. The cone is its closed forms to
rounding, its edges' tolerances 0: `brep::a_nurbs_cone_is_its_side_sewn_to_its_base` and
`brep::faces_sewn_are_a_solid_by_tolerant_topology` (a box's faces facing any way, its top in
halves, a void, faces a hair apart, a face missing, four faces along an edge) and
`brep::curved_faces_sewn_across_curves_parametrised_apart` (two nets and a cylinder end to end and
two discs, every circle begun at another angle and one run at another pace, so each is cut at the
other's vertex and the dropped halves inverted onto a net and onto the cylinder across its seam)
are the gate.

**What the file writes is in the standard's parameters** (#993 stage 1, `step::step_uv`,
`step::turned`): a face's curves in its parameters are written as ISO 10303-42 reads the surface.
A cone's `v` runs along its axis there, `C + (R + v tan α)(cos u x + sin u y) + v z`, so the
file's `v` is the kernel's (along the generator, as OCCT's is) times `cos α` — written as the
kernel's, every pcurve on a cone lay off its edge (a drilled frustum's traced edges by 2.5 mm,
which OCCT read 1.4e-4 off in volume). A lemon (the kernel's torus of negative major radius, which
a `toroidal_surface`'s positive lengths forbid) is the `DEGENERATE_TOROIDAL_SURFACE` of major
radius `−big` with `select_outer` false, its curves at `(u + π, π − v)` and its face's sense
turned. A curve in a net's parameters is written as a line only where sixteen uneven samples find
one (eighths of a turn, where an affine torus's arc parameter and its angle agree by symmetry,
found lines 0.075 mm off their edges), otherwise as the cubic within `EXACT`. Gates:
`step_parameters::every_pcurve_lies_on_its_edge_at_the_edges_parameter` (each pcurve, read through
the reader's evaluation of the standard, on its edge within `FIT + 2·EXACT`),
`a_lemon_is_written_as_a_degenerate_torus`.

**A STEP file never states less than it holds** (#993 stage 1, `step::write_stating`,
`step::statement`). The uncertainty a file states is the tolerance asked (at least 1e-7) where
that covers a quarter more than its geometry holds, and otherwise what it holds, rounded up to 1,
2 or 5 × 10ᵏ (`creased.sv` states 5e-5, a drill 1e-6, an exact solid 1e-7); `export::step`'s
stage line says when a file states more than was asked. What a file holds is, an edge, its
distance off each of its faces (measured afresh at 64 points, its curves' knots, halfway between
them and every point a pcurve's fit reads — not `Edge::tol`, which the kernel lets go stale) with
the measured misses of its curve, its faces' surfaces and its pcurves, and each vertex's
tolerance. **A fitted pcurve is held no nearer than its edge stands off its face, and no curve is
written in more than `3 MOST + 1` points** (`step::MOST`, 4096 pieces): held to `EXACT` where the
edge stood 1e-7 off its net, a toolpath's pcurve never closed in and fell to a polyline of
millions of points (a screw's whole turn 3.1 GB and 227 s, now 0.14 s). Past the bound a pcurve
is left out, a seam's two written both or neither — an edge needs only its curve in space, and
the reader walks a face without them — and a traced curve's polyline is written within what
`MOST` points hold, the file stating it. **The representation a curve on a surface prefers is the
exact one**: `.CURVE_3D.` wherever the curve in space is written as itself, `.PCURVE_Sk.` only
where that curve is a fit and the `k`-th pcurve is the kernel's own spline in the parameters of a
surface written exactly. **A face family's wall and its rails, turning at their stations' joints,
are fitted piece by piece and joined in value** (`step::family_pieces`, `fitted_stretches`): a
net smooth across the joints stops 1.7e-2 off at its cap of rows.  Each piece's grid is doubled
only the way it misses (`nurbs::fit_net_each`): a ball's ring along a spline ellipse needs its
columns round the tube, never its rows along it, and doubled alike was written as 7233 rows of 129
(381 MB, 14 s) where 566 hold it. A face on an offset is written
as `OFFSET_SURFACE` of its sheet's net, so a solid read with one is written again. Gates:
`step_roundtrip::the_sample_reads_back` (no edge read looser than its file states, no curve past
`3 MOST + 1` points), `a_lofts_file_states_what_its_fits_hold`,
`a_files_uncertainty_covers_the_solids_own`, `a_toolpaths_pcurve_is_written_in_few_points`,
`a_drill_is_written_within_a_minute`, `the_curve_in_space_is_the_master_where_it_is_exact`, and
in the slow tier `a_family_wall_is_written`, `a_slid_contour_reads_back`,
`a_solid_read_is_written_again_within_a_minute`.

**A rational net's foot is not stopped by a seam or a pole** (`geom::net_settle`, `pole_newton`):
a foot Newton's method holds on the domain's edge is sought again across a closed net's seam and
from beside a collapsed row, and only a strictly nearer foot is taken (a point on a ball's net
with poles was read 2.19 off). **A sheet's offset at a pole is the limit of its normal along the
collapsed row** (`offset_at_pole`): a ball's net offset is the ball grown, an apex's offset the
circle of its rows' limits. Gates: `net_inverse::a_point_on_a_rational_net_is_read_back_to_itself`,
`a_balls_net_offset_is_a_ball`.

## Reading STEP (#993 stage 1)

`step_read::read(text)` gives the manifold solids a STEP file holds, each a `Brep` with the file's
entity for every face, edge and vertex (`Given`), or a `Refusal {why, entity, msg}` naming the
entity at fault: **read correctly or refused, never read wrong.** `step_read::reading` gives the
solids and what the file holds beside them. OCCT is the oracle in tests only.

**Every file the kernel writes reads back to the same solid**, stated as one property
(`step_roundtrip::same`): every written face, edge and vertex read once from its own entity, of
its kind and sense, each edge between its written vertices; nothing laid but a pole's degenerate
edge where the surface its face is written on has that pole (a net's collapsed row, a cone's apex,
a sphere's pole), one at the point of each the built solid holds there — a degenerate edge bounds
nothing and has no length, so one the built solid keeps where its face is written on a surface with
no pole there (a loft's side collapsed to a point, written as the plane triangle it is) is set aside
on both sides, with the vertices only it ends at; each face's area and flux, the volume and the area within what the
writer's own fits allow (`Bars`: rounding where a curve is written as itself, twice the fit times
its length for a traced or fitted curve, a fitted surface's share, and 1e-6 of the area on a
surface with no closed form, whose area rule is not exact where its flux's is); the solid read,
written again, passing `step_check::verify` and reading to itself; and no edge read looser than
the file states. A `Brep` of several lumps or with a void is written as the solids it is (below)
and its solids read are joined back into one (`step_roundtrip::joined`). Over the library's exact solids (`census`, a tool): all 496 the same, no
edge read looser than its file states, none refused and none a panic. Gates:
`step_roundtrip::the_sample_reads_back` (a sample taking every writer path a solid that reads back
takes, `PATHS`, at 1e-7), `the_sample_reads_back_at_export_tolerances`,
`a_nets_collapsed_side_reads_back` (`pyramid`); a solid that does not read back is an ignored test
named for its defect, and there is none.

**The kernel writes every solid its reader reads** (#993 stage 2): a body a file gives is
exported again, so `step::write` and `step_check` are handed B-reps the kernel did not build, and
write each exactly or refuse it with a reason — never a panic, which aborts the wasm build. **A
surface swept or turned from a line, a circle or an ellipse** (`Surface::Extrusion`,
`Surface::Revolution`, which a reading makes of a file's `SURFACE_OF_LINEAR_EXTRUSION` or
`SURFACE_OF_REVOLUTION`, a cone's nappe turned from its line) **is written as STEP's own entity
over that curve** (`Out::swept`): the standard's two sweep any curve over its own parameter, as the
kernel's do — the extrusion `C(u) + v V`, the revolution the curve turned by `u` — so the face's
parameters are the file's and its pcurves are written in them unchanged, a line's vector its own
`d` (unit direction and length) so its parameter is the line's. It is not written as the exact
rational net: the net would replace the file's surface by another form of it, its parameters the
arc's and not the angle's, and a second reading would give a net where the first gave a
revolution. A curve no entity of the standard writes is refused (a turned contact's meridian and a
slid contour keep their fitted cubic, #780, #805). `step_check::verify` holds a swept surface's
axis (a revolution's point and direction) or vector (an extrusion's direction and length) and
its curve — a line's point, direction and length, a conic's placement and radii, a spline's
degree, poles, knots and weights — to the solid's (`Solid::of`, a curve the writer fits checked
for a spline alone), and an offset's distance and basis net; `step_roundtrip::same` compares
those numbers too, as it does every other surface's, a curve the writer fits held as `verify`
holds it — the axis or vector the solid's and the curve read a B-spline, the fit's numbers no
solid's (`a_slid_contour_reads_back`, `leaning_ring.sv`'s extrusion of a slid contour, refused
while `same` compared the fit's 650 numbers with the solid's five). A `Brep` whose indices leave
its arrays, or whose geometry holds no finite uncertainty, is refused, not followed. Gate:
`step_rewrite::every_solid_read_is_written_again` (every checked-in file the reader reads, and
nine written by hand — a prism on extrusions of lines, upright and aslant; an extrusion of a
circle, of a circle aslant read as an ellipse, of an ellipse; a revolution of a line along its
axis and meeting it, of a circle and of an ellipse beside the axis — each written again, verified,
and read back the same solid, every face on its original surface; the nine panicked the writer);
`step_rewrite::the_corpus_is_written_again` (a tool over `$STEP_CORPUS`): the 32 CADGenBench
inputs each written again, verified and read back the same, 217's drilled void as a
`BREP_WITH_VOIDS`. `same` reads a pole as the reader does, where the collapsing parameter
sweeps no more than the file's uncertainty or the vertex's tolerance (a tolerant vertex a hair off
a sphere's pole or 1e-5 off a net's collapsed row is that pole).

**A body is written as the solids it is** (#993 stage 2, `step::lumps`). ISO 10303-42 holds a
`closed_shell` to be a `connected_face_set` (its faces, with their edges and vertices, connected),
a `manifold_solid_brep` to be bounded by one outer closed shell, and a `brep_with_voids`'s voids
to be closed shells inside it, apart from it and each other: so a `Brep` whose faces fall into
several components by the edges they share is no one closed shell, and one written so (every face
in one `CLOSED_SHELL`, as `step::write` did) does not conform, and what a reader makes of it is
that reader's guess. Its
shells are nested as the reader judges them (`step_read::sound::nesting`, one rule for both: each
faces out, a lump, or in, a void, a void inside the lump nearest about it), and each lump is
written as its own `MANIFOLD_SOLID_BREP` among the representation's items (as the reader reads
several solids in one product's shape), a lump with voids as a `BREP_WITH_VOIDS` whose voids are
`ORIENTED_CLOSED_SHELL`s of orientation `.F.` (as OCCT writes them and the reader reads them):
each void's closed shell faces out of its void, a closed shell of its own, every face's sense and
loops turned, and the oriented shell turns it back. A lump inside a void is a solid of its own.
The faces are written lump by lump, its outer shell's then its voids', each in the `Brep`'s
order; one shell is one solid whatever it faces, as before. Shells that nest as no solid's do
are refused at the write. `write_assembly` writes a part so (each of its solids styled), and
`step_check::verify` walks what was written: as many solids and shells as the lumps say, each
void an oriented shell turned, each face's sense in its solid (its shell's turn undone) the
solid's. Gate: `step_rewrite::lumps_and_voids_are_written_as_the_solids_they_are` (a block with a
void holding an island, a block with two spherical voids, two blocks apart: their solids, voids
and turned shells counted, verified, and read back the same body twice over), and OCCT's
`void_ap214.step` in `every_solid_read_is_written_again`.

**A STEP file's syntax is read as Part 21 means it, whatever the cores** (`step_file`). What
writers write is read: a byte-order mark, comments wherever a token may stand, every kind of line
end, entities several to a line or one over many, strings holding a statement's punctuation, every
form of real that has one meaning, typed parameters and user keywords, and the header (`Header`,
with its schemas' names). The data section is cut into runs at `;`…`#` and each run must parse
exactly to its end, so a cut inside a string or a comment cannot pass unseen; where any run fails
the section is parsed again from its start on one core, and that answer, entities or error, is the
answer. Every error names the instance it is in where there is one, and the line and byte where it
stopped. What is not read (a SCOPE, a second DATA section, edition 3's sections) is said to be
unsupported, not malformed. **The parser follows no structure past its bounds**: a list nested
deeper than `step_file::DEPTH` (64) is refused as malformed — following it would overflow the
stack, an abort no `catch_unwind` reaches and which stops wasm — so any bytes the reader is given
return, a refusal or solids. Gates: `step_file::syntax_as_writers_write_it`,
`line_ends_of_every_kind`, `reals_as_the_grammar_writes_them`, `strings_decode_as_names`,
`errors_say_where`, `answer_independent_of_runs`, `no_panic_under_fuzz`;
`step_mutants::syntax_never_panics` (a written file truncated at every 97th byte, 300 seeded
single-byte substitutions, its lines shuffled, a list nested 100,000 deep, each within 5 s).

**A file's schema, units and uncertainty are read, never assumed** (`step_read::context`). A file
is read under AP203, AP214 or AP242, named by its FILE_SCHEMA; any other schema is refused naming
it, because an entity means what its schema says. A unit of length or plane angle is an SI unit
under any prefix, or a conversion-based unit followed through its factors down to SI, never read
by its name; a unit that cannot be resolved is refused naming the unit. Every length read is in
millimetres and every angle in radians. The uncertainty is the context's stated distance accuracy,
in the unit its own measure names, absent where the file states none: it is reported
(`Context::uncertainty`), and no check is widened by it. Gates: `step_file::header_and_schema`,
`units_by_conversion_factor`, `units_unresolved_refused`,
`step_read::a_files_unit_of_length_scales_what_it_reads`.

**A solid stands in its product's space, and a placed solid is the solid moved.** A solid — a
MANIFOLD_SOLID_BREP or BREP_WITH_VOIDS — is an item of a representation its product's shape
definition reaches, and its name is that product's. A representation relationship without a
transformation shares that space. A mapped item among a representation's items places its map's
representation by the rigid motion taking the map's origin onto its target; a relationship with an
item-defined transformation places its first representation in its second. Each axis is read in
its own representation's units, so a part in inches mapped into a product in millimetres is
placed, not scaled. The solid is read in its own context and moved exactly (`Brep::moved`). An
axis placement that is merely an item places nothing (a datum, or the part's frame), and a mapped
item in a draughting model presents the shape and places nothing. Product structure is refused
`Unsupported` naming its entity — a usage occurrence, the same representation placed twice,
solids in two products' shapes, a transformation that is not two axis placements (one that could
scale or mirror): assemblies are not read from STEP yet. Gates:
`step_file::solids_found_with_names`, `a_placement_item_places_nothing`, `a_mapped_solid_is_moved`,
`a_mapped_solid_reads_moved`, `product_structure_refused`.

**What a file holds beside its solids is said, not refused and not hidden**
(`step_read::reading`, `Beside`): surface models, wireframes, sets of curves, tessellated bodies,
construction geometry and annotation, each by its entity. `read` is its solids. A file holding no
solid is refused saying what it holds. Gates: `step_file::beside_the_solid_is_said`,
`what_is_no_solid_refused`.

**A file's curve or surface is read as the kernel holds the same points, exactly, nothing
refitted** (`step_read::geometry`). A B-spline's knots are its written ones or those its kind
implies (uniform, quasi-uniform, piecewise Bézier); knots not clamped are clamped over the domain
by insertion, and `closed_curve`/`u_closed` are not believed. A parabola's arc is the quadratic
Bézier over its own parameter and a hyperbola's rational quadratics, each bounded by its edge's
vertices or its trims. A trimmed curve is its basis, a one-segment composite or an identity
replica its parent. An offset of a plane, cylinder, cone, sphere or torus is the surface of that
kind, a spline sheet's is `Surface::Offset`. An extrusion's curve is carried into the plane square
to its direction (an affine image: the same points). A cone written with a negative radius is the
cone its formula gives, its origin at the apex (against the standard's `radius >= 0`, read because
the formula's points and normal are unambiguous for any radius). A placement's reference direction
left unset is the standard's (`first_proj_axis`: `[1, 0, 0]`, or `[0, 1, 0]` about an axis along
`±x`). Refused naming the entity: what has no exact form here (an offset curve, a composite of
several segments, a moved replica, a self-intersecting offset, a line of no length, a conic or
line whose two halves sweep the same points facing opposite ways where nothing says which), and
any kind not listed. Gates: `step_kinds::a_b_spline_reads_whatever_says_its_knots`,
`a_curve_wrapped_is_the_curve_it_wraps`, `a_surface_wrapped_is_the_surface_it_is`,
`an_offset_of_an_elementary_surface_is_one_of_its_kind`, `a_conic_without_end_is_read_as_its_arc`,
`a_degenerate_torus_is_its_spindle_torus`,
`a_placement_without_a_reference_direction_takes_the_standards`.

**A form that waits on its faces is seated on them** (`geometry::Seat`, `Sheet::seated`, called
once from `topology::solid`). A curve turned off its axis's plane, or a line turned skew to its
axis, is the exact rational net of its revolution; the net has no period, so its turn starts at
the file's own seam where a face's seam runs along it, and otherwise runs only over the angles its
faces take, a quarter of the widest gap past each end (CADGenBench 249 read an edge 217 mm off its
face where a face crossed the file's `u = 0`). A line meeting its axis is the revolution of the
half its faces lie on, refused where they lie on both; an open conic in its axis's plane is the
revolution of its arc bounded by its faces. Gates:
`step_kinds::a_curve_turned_off_its_axis_plane_is_seated_where_no_face_crosses`,
`a_line_turned_off_its_axis_plane_sweeps_its_hyperboloid`.

**A STEP file's topology is read to the solid it writes, identity the file's**
(`step_read::topology`). Each VERTEX_POINT, EDGE_CURVE and ADVANCED_FACE is read once by its `#n`,
and a vertex or edge shared by reference is shared in the `Brep`; nothing is merged by proximity,
and what the reader lays (a pole's edge, a seam, the far piece of an edge cut at a pole) names no
entity or the file's one. Each edge's ends are the points of its curve nearest its vertices, an
arc running counter-clockwise from its first to its second and a closed edge a whole turn; an edge
running against its curve is laid along it with its vertices swapped and every use turned. A loop
whose edges do not meet at the same vertex entity is malformed (the schema's `edge_loop` rule); a
shell of no face is refused `Empty` naming it; an edge farther off its faces or vertices than
max(100 × the stated uncertainty, 1e-3 × the solid's size) is refused naming the edge; a boundary
`Brep::check` refuses is Open, NotManifold or Unsound naming the face or edge the check names.
Gates: `step_read::every_solid_the_kernel_writes_reads_back_to_itself`,
`what_is_not_read_is_refused_naming_the_entity`, `an_empty_shell_is_refused_naming_it`.

**A face's parameters are walked round each loop, and a walk keeps its branch**
(`topology::walk_loop`, `track`). Each use starts where the last ended, its points read on the
branch nearest the last, in as many steps as its curve turns round a period (by the surface's
speed along it: a thread's helix of sixteen turns read by chords was no turn at all), a step
sweeping more than a twelfth of a period halved. The file's curve in the face's parameters is the
use's pcurve where its image is the edge's — at the edge's own parameter, or as a set within the
tolerance the edge measures anyway — and never widens a tolerance; otherwise the edge inverted
between the walk's ends (`Pcurve::Inverse`) — where a point of its walk stands half a period from
the line between them the inverse changes branch mid-edge, which `sound` refuses as a leap of a
period. A file without pcurves reads the same. **A point's foot on a spline sheet near a hint is
a function of the point and the hint alone** (`Surface::inverse_near`, `geom::net_search`): where
Newton's method from the hint settles off the sheet, the whole sheet is searched cold for a nearer
foot — not from the thread's last foot, which the search keeps wherever it stands within a
thousandth of the point's size of the sheet — so a walk's end and its pcurve's end read the same
point the same (benchy's face #27487, its corner crumpled back within 9.5e-5 of its corner vertex:
the vertex read on the corner as one edge's end and on the crumple as the next's start, a loop
broken in parameters where the file's is whole). The outer
loop is the one enclosing the most of the face's parameters, never FACE_OUTER_BOUND. A sphere's
file curve a whole turn off in `v` (OCCT's habit) is moved back. At a pole the collapsed
parameter's branch keeps the face on the loop's left; a use's end there is its curve's own where
its tangent and its feet agree, else its foot a hundredth of its span in; an edge crossing its
face's pole between its ends is cut there, `close_poles` laying the pole's edge. **A seam the file
does not write is laid**: a loop winding once round a pole is joined to it by a meridian or a
generator; two loops winding a period opposite ways (a cylinder between two circles, Parasolid's
habit) by a seam at one value of the period, vertex to vertex where one stands there, else cutting
the other's edge, and crossing no other loop of the face; a whole torus or sphere bounded by lone
vertices gets its seams. A band where no seam can be laid, a loop that both winds and crosses its
pole, an edge two seams would cut, are refused `Unsupported` naming the entity. Gates:
`step_read::a_cap_written_with_no_seam_reads_to_its_closed_form`,
`a_band_written_with_no_seam_reads_to_its_closed_form`,
`a_spheres_curve_a_turn_off_in_v_reads_where_the_face_is`,
`a_lens_whose_circle_passes_a_pole_reads_to_its_closed_form`,
`an_edges_feet_off_its_face_run_at_their_own_rate`,
`net_inverse::a_points_foot_near_a_hint_does_not_depend_on_what_was_asked_before`.

**A tolerant edge is read by its 3D curve, whatever its file names its master**
(`topology::file_pcurve`). A `SURFACE_CURVE`'s `master_representation` (ISO 10303-42: the
representation preferred where a tolerant file's 3D curve and pcurves differ) is not followed: the
edge is its 3D curve, a face's own pcurve kept only where its image is that edge's (above), else
the curve's foot. Followed — each face on the master pcurve's surface bounded by it, the others by
their own pcurves where they follow it — the rule read the kernel's own thin lens 5.5e-6 under
its volume (its `.PCURVE_S1.` pcurves fits within `step::EXACT`, agreeing with each other to
6e-12 and 6.6e-9 off the 3D circle), read faces of OCCT's own files 3e-3 mm² off where OCCT and the
measurer agree to 3e-7, put 212 and 229 further outside their rows' bars, and set apart from both
references faces they agree on (CADGenBench's `.PCURVE_S1.`, 201's edge #6220 6.7e-2 mm off a 3D
curve and second pcurve 6.4e-4 apart, read #6071 a hundredth under both): the writers that say
`.PCURVE_S1.` (OCCT, CADGenBench's, the kernel's, arris's — 97,631 of the corpus's 118,648 surface
curves) say it of every edge, of the pcurves they fitted. A pcurve said to lie on another surface
than its face's is a curve of no face of the edge's. Gates:
`step_kinds::a_tolerant_edge_is_read_by_its_3d_curve_whatever_its_master`,
`a_master_pcurve_on_another_surface_is_set_aside`; `step_corpus::nothing_is_read_wrong`.

**A net creased at its knots is measured as a smooth one** (`props::Prefix::rows`): each `v`
span's column of `G` reads the net's rows at its nodes, its far end too, through that span's own
knots and poles. A B-spline read at a knot takes the next span, and across a knot of the degree's
multiplicity — where a net is joined in value only and `S_v` jumps — that node tabled `G` from the
wrong patch: a face family's wall, written piece by piece between its stations, read its flux
2.8e-5 off while its area, which takes no table, stayed right. A line in a face's parameters is
cut, in closed form, where it crosses such a crease, since `G` jumps there. **A stretch of a use
is read by ten nodes only where they read it** (`props::checked`): each stretch between breaks is
checked by the rule's 21-point Kronrod extension and kept, to the bit, where the two agree to
rounding; otherwise the extension is kept within its estimated error, or the stretch is settled.
An edge one of whose knots stood 1.5e-9 past its start had its half turn on a rational net read by
ten points, a face's area 3.8e-5 off (brep-bench's frustum screwed across a ring sector, and drum
leaning across a turn, the corpus's last two files read wrong). **An inverse curve's rate reads
second derivatives on its foot's side of a joint** (`topo::second`): a net's own in the span
holding the foot, any other surface's differences taken on one side of a break within their step.
**A polynomial net's `G` away from the flux's table is read along its iso curve** (`props::Iso`),
the rows across the held parameter read once per node, the same values to rounding (a 91×143
quintic's area in half the time). Gates:
`step_roundtrip::a_net_creased_at_its_stations_is_measured`,
`rational::a_rational_arcs_stretch_is_read_to_its_closed_form` (a 340° band of two rational arcs,
1e-8 off under the ten nodes alone), `step_read::a_foot_just_past_a_joint_runs_at_its_own_rate`,
`step_corpus::nothing_is_read_wrong`.

**A face reaching a pole is integrated along the parameter the pole collapses**
(`props::integrated`): where a degenerate edge lies in its loop, or a sheet's row is all but
collapsed, `G` is nought at the pole, so the value a use arrives along weighs nothing in the
measure. Its edges' rules are settled stretch by stretch and cut where a pcurve crosses a sheet's
knot line or an area's turning line (on a net, where its normal turns over along the use); a spindle
torus's area is the closed form of `∫|R + r cos v|`. Each use's ends are closed to where its curve
arrives, a hair in, so the region read is the one `sound` judges. A solid farther from the origin
than ten times its size is measured moved to its middle (`props::volume`). Gates:
`step_read::the_value_a_use_arrives_at_a_pole_along_weighs_nothing`,
`a_spindle_torus_reads_to_its_closed_form`.

**A tolerant boundary is read as known to within its measured tolerances** (`step_read::sound`,
run on every solid `topology::solid` builds). Tolerances are measured (`Brep::measure_tolerances`),
never taken from the file and never widened to pass a check; the file's stated uncertainty is
reported, not enforced (CADGenBench's own files measure up to nine times what they state). Pieces
of a face's loops within their tolerances of each other touch and are read — a pcurve wiggling at
a tolerant edge, a cusp at a corner, two loops pinched at their vertices, an edge used both ways
through a corner — and so are an edge shorter than its tolerance, two vertices within their
tolerances (identity is the file's) and a face of no area: these are what a tolerant file means,
not defects. **A refusal is for a boundary that is not a solid's by more than its tolerances can
explain.** A crossing is refused where it is transversal at the tolerance's scale: both loops are
followed out from it to a radius at which every arc end stands farther than the tolerance from
the other arc, measured in space, and they cross an odd number of times; never within a few
tolerances of a corner both reach. A loop reaching round its surface more than a period is refused
only where the face and its images a period along cover a point of the surface twice (a band
whose seam leans is read). No tolerance may exceed `sound::LOOSE` (1e-3) of the boundary's size:
past that the reading says nothing (the corpus's right readings stand at or under 4.6e-4, the next
above at 7e-2). **Each loop turns as its face's sense says** (#993 stage 2, `Brep::turned`, the
one rule `Brep::check` holds a solid to): the outer loop counter-clockwise in the face's
parameters about its sense, each hole clockwise, by the signed area it encloses there — read
against the measured tolerances of its edges and vertices, never the uncertainty the file states
(a bound, which on CADGenBench 240 is 0.02 mm while its edges measure 1e-4). A loop turning the
other way is a face whose stated normal points into the material its own edges bound it on: its
sense and its boundary contradict each other, and which the file meant is not guessed. CADGenBench
240's cylindrical face #92335 is one — `.F.`, 0.012 mm wide, the faces either side tangent to it
and facing exactly the other way; read, it was refused only at the export, as "the built
boundary", where the document's export checks a given body. **A sliver its tolerances cannot
orient turns no way** (`topo`'s `sliver`): its area in its parameters, carried into space at its
first corner, no more than its length times the widest tolerance among its edges and vertices; or
every point of each of its uses within twice that of another of them — two edges of a strip
narrower than their tolerance are no farther apart — where its area in its parameters is no
measure (CADGenBench 242's face #75264, a spline's strip some 1e-5 mm wide whose feet wander along
its narrow way: read turning one way from the file and the other from the same edges written
again, so the export of the file was refused when read as a given body). Gates:
`step_sound::a_loop_crossing_itself_is_refused`,
`holes_crossing_outside_or_overlapping_are_refused`, `a_band_whose_seam_leans_is_read`,
`a_band_wound_round_a_cylinder_overlaps_itself_only_where_its_turns_meet`,
`tolerances_past_the_geometry_are_refused`, `the_kernels_own_solids_are_sound`,
`a_face_turned_against_its_loops_is_refused`, `brep::a_face_thinner_than_its_tolerance_turns_no_way`.

**The foot of an edge is where its face is: a face whose edges lie along its sheet's domain's rows
is bounded by those rows** (`topology::sound_solid`, `rows`, `along_row`). Where `sound` refuses a
face read by its edges' nearest feet, the solid is read again with each use of that face lying along
a row of its spline sheet at the domain's edge — the row not a pole's (its ends and middle not within
the tolerance the boundary is checked to, max(1e-9 × size, the stated uncertainty), of one another),
every one of 65 points of the
edge within the reader's bound for an edge off its faces (max(100 × the stated uncertainty, 1e-3 ×
the solid's size)) of the row at its foot on the row, the feet running one way — its pcurve laid
along that row (a line where the feet run linear in the edge's parameter, else through them), and
taken where that reading passes `Brep::check` and `sound`; again for the next face `sound` then
refuses, up to eight; else the first refusal stands. A face sound as first read is never read again,
so no trimmed face near a row changes. What is newly accepted: a face whose nearest-foot reading is
not a boundary (its uses leap) read as bounded by the rows its edges stand along, their tolerances
measured against those rows as any edge's are, and held to `sound::LOOSE` as any. Benchy's face
#27487 is its sheet's whole domain, the net's corner crumpled (its normal turning over within 0.06
of `u` and 0.0012 of `v` of it): its edges stand up to 4.4e-3 off the rows and within 1e-4 of the
crumple, and read by those feet their curves in the face's parameters leapt into the crumple and
back; read on the rows, their tolerances measure 4.2e-3 to 4.4e-3 (OCCT's 5.8e-3) and every one of
benchy's 1385 faces is within its bar of both references. **A pole's row is judged at the
tolerance points are one at, never at the bar for an edge off its face** (#993 stage 2): written
again, the file states 1e-2 (what its edges stand off the rows, `step::write_stating`), its bar a
millimetre, and read again by its feet the face leaps as before — the writer's curves in its
parameters, which lie along the rows, kept by no rule that keeps a file's curve only where it is as
near the edge as the sheet's nearest foot, which the crumple is. Judged a pole within that bar, the
face's 0.28 mm row was a point, its edge laid on a row across it 0.28 mm off and the face refused:
the solid a document gives untouched could not be exported, its export refused by the reader that
read it. Gate:
`step_kinds::a_face_whose_edges_lie_along_its_sheets_rows_is_bounded_by_them` (a slab under that face,
refused before, its faces their sheets' whole domains after),
`step_rewrite::a_face_read_on_its_sheets_rows_is_written_again`,
`given_solids::a_body_read_on_its_sheets_rows_exports_and_reads_back`, the corpus row.

**A use is one curve in its face's parameters, its region what its leaps close, and the shells nest
by their senses** (`sound`'s `Drawn::leap` and `reach`, `shells`). A face's region is its uses
closed across each leap by the chord between the leap's two sides in the face's parameters — `props`
closes each use's ends to where its curve arrives so (a hair in, carried back by its rate) — and a
leap is judged by what that chord does to the region: a chord whose image in space keeps within
twice the tolerance of the leap's start (`reach`, read at 256 steps with half a step's run between)
adds or takes only points within that disc, at most `π (2τ)²`, and is read — a corner's slop, or a
use's feet swinging round a pole it ends at or passes within its tolerance of, whatever the swing in
parameters (CADGenBench 215, 240, 242 and 247, read face by face within every bar). A leap of a
whole period, a chord across a closed sheet's seam (reaching round the sheet), or ends apart in
space past twice the tolerance adds a strip and is refused naming its face. A face on a spline sheet
closed on itself is read on the window of the sheet its loops lie in (below), so a band round it is
one region (CADGenBench 217). Each shell faces out or in as points stepped off its largest faces are
located against it, its signed volume used where none locates or they tie (217's drilled void of
three faces); a lump lies inside nothing or inside a void, a void inside a lump, none crossing
another; any number of lumps is sound, in one shell's faces as files written before #993 stage 2
put them; a boundary of no faces is refused. Whether faces that are not neighbours cross in space is not decided yet. Gates:
`step_sound::a_use_leaping_a_period_is_refused`, `a_use_swinging_round_a_pole_it_ends_at_is_read`,
`shells_inside_out_or_apart_are_refused`, `step_mutants::an_empty_shell_is_refused`.

**A face bounded by vertices alone is the whole of a surface that closes**
(`topology/closed.rs::whole`, called once from `topology::face`; a sphere and a ring torus keep
`whole_sphere` and `whole_torus`). **A ring torus is one whose hole is wider than a millionth of its
radius** (`topo::poleless`): narrower, it is a horn torus to every tolerance a pole is read at
(`pole_at`), its middle a pole the reader closes a corner at with the pole's degenerate edge and
`close_poles` mends, whichever way the platform's `sin` rounded the radius it was built with
(`step_roundtrip::a_use_ending_at_a_horn_toruss_pole_a_bit_open_reads_back`).
ISO 10303-42 lets a face's bounds be `VERTEX_LOOP`s. Whether
the surface closes is read from the surface — its points at a parameter's two ends agreeing, or
its period — never from the file's `u_closed`, and the face is laid in one of two shapes: closed
round one parameter with both end rows of the other collapsed to points (a spheroid's net, a
revolution of a half ellipse), its seam along the closed parameter's start and its poles each the
file's vertex where one stands there, laid otherwise; or closed round both and pinched nowhere (a
torus's net), with both seams. A vertex standing on a seam the reader lays cuts it there, and a
period starts at a vertex. Refused naming the face: a surface that does not close round its
vertices; a vertex on neither a pole nor a seam laid; and a surface pinched to a point inside its
domain (`pinched`) — a whole spindle torus, a circle crossing its axis turned, an ellipse about
its own axis turned twice over — which touches itself and is the face of no manifold solid, one
vertex saying nothing of which part is meant. **An edge on a line of no length is a point only at
its vertex** (`Walk::points_only`): a loop of edges each running from one vertex to itself along a
`LINE` whose point stands at that vertex within the stated uncertainty (brepkit's torus seams) is
that point, its edges naming the seams the reader lays; a line of no length between two vertices,
or off its vertex, is refused as malformed. Measures read a net's face cut where its normal turns
over along a use (its area factor's kink, a spindle torus's net crossing its axis), and an inverse
curve's foot held on a sheet's domain edge — the edge standing off its face past it — runs along
that edge alone (CADGenBench 246's whole-domain face read 6.5e-4 over it). Gates:
`step_kinds::a_face_bounded_by_vertices_alone_is_the_whole_of_a_surface_that_closes` (a spheroid's
rational net bounded by its south pole, the spheroid as a revolution, a ring torus by a vertex on
its equator, to 1e-9, and both refusals), `an_edge_that_is_a_point_reads_as_one`.

**A closed conic turned across its axis is the revolution of its arc on its faces' side**
(`geometry::Seat::Crossing`, `crossing_side`): each point of its faces is carried round into the
conic's plane, the side is the one where `±ρ` lies on the conic, and the surface is the
`Revolution` of the exact rational arc between the two crossings of the axis (a lemon, or the
apple, of a spindle torus written as a circle turned); where its faces lie on both sides or on
neither, the whole turn's net is kept. **A net the reader made takes no curve of the file's in
its parameters** (`geometry::UNMAPPED`): a turned, swept, throat or crossing net's rational arcs
meet the file's angle only at their knots, and a file's curve read past the clamped net's domain
was taken for on its edge, point and foot both at the domain's edge (springrep's coin, 3.55 mm).
A turned net's seam goes wherever a face's seam runs at one angle (`Sheet::seated`; springrep
writes it at `u = π`). A file's curve lying wholly past a closed spline sheet's domain is moved by
its width, and one leaving it otherwise is dropped for the walk's own (`closed::onto_net`). **A
closed edge's vertex may stand anywhere on its closed spline** (`geometry::restarted`,
`closed::restarted`, in `read_edge`): the spline is cut at the vertex and its two pieces put end
to end by knot insertion, the same points, the vertex taken for a knot within 1e-9 of the domain
of one; that edge's curves in its faces' parameters are then the walk's. Gates:
`step_kinds::a_conic_turned_across_its_axis_is_its_arc_on_its_faces_side` (the lemon, and the
apple by Pappus, to 1e-9), `a_closed_edge_on_a_spline_runs_round_from_its_vertex`; the nets' three
rules by their corpus rows (springrep coin-turned-about-a-line-through-its-rim,
bull-nose-carried-along-a-spline, v-wheel-turned-once-round-rod).

**A band round a torus is the part of its tube its bounds keep left** (`topology::band`'s
`lift`). A face between two loops each winding once round one period of a surface closed round
both (a torus's band round its axis or round its tube, written with no seam) is joined by a seam
the reader lays from the one loop to the other. Its other parameter is periodic too, so the two
loops' walks, each on the branch its own inverse gave, settle nothing of which way round the tube
the seam runs: the second loop is moved by whole periods onto the side of the first the face lies
on — its left as it runs, its right where the face's sense turns the surface's normal — and the
seam is laid across the face, never round the rest of the tube. On a cylinder or a cone, which
close round one parameter only, nothing moves; two loops running the same way are refused as
before. Read the way the walks left it, a band reaching more than half round its tube took the
rest of it and was refused as turning the wrong way (NIST's FTC 11 face #1195, a rim rounded
160.66° of its tube, now read to its closed form 797.485). Gate:
`step_kinds::a_band_round_a_torus_is_the_part_its_bounds_keep_left` (eight files: the band's
reach, the torus's axis up or down, the vertices in line or a quarter turn apart).

**A surface closed on itself is periodic whether or not its form has a period, and a loop on it is
walked as on any periodic surface** (`topology::Rolled`, `closes`, `rolled`, `unrolled`, `window`).
A B-spline sheet (or its offset) whose rows at a parameter's two ends are one curve, to 1e-7 of its
size, and whose derivative across them keeps its direction to a thousandth of a radian (a pole's
row, nought, asks none; a writer's fitted net closes with its tangents 4e-5 apart) closes along that
parameter, read off its own points and tangents and never off the file's `u_closed`. The kernel's
net holds no period, so a loop crossing the closing line was no one curve in its face's parameters —
a leap of the domain's width, a break, an edge read millimetres off its face. Such a sheet is rolled
out: put three times end to end along each parameter it closes in (knots of full multiplicity
between, each copy's poles as written: the same points, exactly), each loop set out from the middle
copy, and walked with the domain's width as its period, the walk's every rule — branches, a pole's
collapsed parameter (a file curve leaving a pole on the branch that keeps the face on the left; a
seam's second use a whole period from its first, which, where the loop came round the pole between
them along an apex's own closed edge, is that branch itself), a band's seam between two winding
loops laid along the net's iso-curve (the net cut there by knot insertion, its row), a loop winding
round the net's collapsed row joined to it along one — as on a cylinder or a sphere. Each face is
then read on the sheet as the file gives it where its loops, moved by whole widths, lie within its
domain, else on the window of the rolled sheet one width wide — centred on its loops' reach, or
starting where they reach right round exactly (a seam written or laid); a seam leaning a hair, its
uses a hair more than a width apart (CADGenBench 217's, by 2.5e-7), split between the window's two
edges — the sheet cut where no loop crosses and its pieces put end to end, held to the file's at 25
points; its loops' ends within 1e-9 of the window's edges are put on them. A face reaching round
farther than a hair more than once, or a use whose points read on the window leap half its width (a
foot past an edge read on the far one: 217's band, its window starting at the least its loop
reached, was measured at half its area), is refused naming it. **A face on a sheet covering its
points twice facing opposite ways is the one of its two readings the solid needs**
(`topology::twice`, `doubled`, `other_foot`). On a whole curve turned a whole turn about an axis it
is symmetric about (kentron's oblate spheroid) every point is the sheet's twice, its normal turned
on the second, so a face's sense says nothing of which way it faces and its bound reads two faces:
on one half the region on its left facing one way, on the other the rest of the sheet facing the
other. The walk took whichever its first foot found — kentron's spheroid with its profile started
half a turn on, the same surface, was refused inside out, and a spheroid cut by two meridians read
as its small wedge twice, facing both ways, volume nought: read wrong. Where the sheet holds another
foot of a face's point half a width along one of its periods, its normal turned, and no curve of
the file's is in the face, the face is read on both halves (each use's first step sought from the
half asked for, a use leaving a pole from the side the loop arrived on); faces meeting tangent along
an edge are read on opposite sides of it, as with any neighbour they meet tangent, which leaves each
group two readings, one the other turned; and of those the one the solid is sound with is taken.
Refused naming the face where both are sound or no reading meets its neighbours. A face's sense on
such a sheet is no statement: a whole spheroid written with either sense, or about its axis turned
over, is the spheroid. Gate: `step_kinds::a_face_on_a_sheet_covering_its_points_twice_is_the_one_the_solid_needs`.
Not read yet: a closing line that is a crease past a thousandth of a
radian (no period: a loop across it leaps, refused); an extrusion or a revolution of a closed spline
curve, which `Curve::period` gives none, is not rolled. A sheet crossing itself is no closed sheet:
springrep's ring pushed in past its fold extrudes a profile through itself (`C(1.9013)` and
`C(6.2019)` one point), and face #10191's loop runs up the line where the sheet meets itself and
leaves it on the other sheet, a leap `sound` refuses rightly (read past it, the face is 15% over).
Gates: `step_read::a_face_across_a_closed_nets_closing_line_reads_to_its_closed_form` (half a drum
across its net's closing line with no seam, a drum on a net with no seam whose vertices stand off
the closing line, a ball's cap round the net's pole off it, each wherever the closing line stands),
`step_roundtrip::the_sample_reads_back` (the round trip takes a closed net turned, the same points
at every parameter to 1e-12, for the surface written), and the corpus rows (brepkit ellipsoid and
ellipsoid-half, kentron and springrep oblate spheroid and lens of two nurbs balls, springrep v-wheel
turned round a rod and bull nose carried along a spline).

**The corpus says whether a file was read right, and accounts for every file that is not**
(`tests/step_corpus.rs`, `scripts/step-corpus/`). Every file of `$STEP_CORPUS` — the 32 CADGenBench
inputs, the STEP every kernel brep-bench runs here writes for its settled cases, a sample of both
written again by OCCT under other writers' habits (no pcurves, AP203, AP242, inches), and NIST's
public test parts as commercial systems exported them — is refused with a witness naming the entity,
or read to exactly the solids its text holds: each ADVANCED_FACE, EDGE_CURVE and VERTEX_POINT read
once (`Given.faces`, `edges`, `vertices`; 0 for what the reader lays itself), its volume and area
within their bars of its row of `step_corpus/reference.tsv`. The reference is the case's closed form
at the bench's own grade where the file holds the case's solid and the case states one (`closed`); a
case's value that is a measure — the mean of kernels' values of their own models, or the measurer's
of the reference shape — is said to be one (`measured`). Otherwise the file read two independent
ways — OCCT, healing off, its solids only, and only where its adaptive integration has converged,
and brep-bench's measurer — and, where both share an error, a third reader's quadrature of the faces
in question (`mass+quad`: brep-bench's Utah teapot, whose case area is 7.1e-6 under its whole faces'
Gauss quadrature from the file's own control points, its knob's four patches read low by OCCT, the
measurer and the kernels whose mean the case takes), and where the account shows one reader misreads
the file, the other's alone (`account.tsv`'s fifth column, `occt!`: brepkit's torus grown past its
hole, a whole spindle torus, which the measurer reads as its apple). **A reading's totals are held
to its row's bar or to what the file determines, whichever is wider** (`determined`, `bars`,
`closure`). Its volume: the flux of a boundary is linear in the point it is taken about, `Φ(o + t) =
Φ(o) − t·C`, `C = ∮ n dA` the boundary's closure vector (nought where it closes), so moving the
solid by `t` changes its volume by exactly `t·C/3` (CADGenBench 212 moved 10 mm: +1.044e-6,
`t·C/(3V)` +1.044e-6); nothing in the file fixes that point, and over points in the solid's box the
volume spans `size·|C|/3`, so the file determines it to `size·|C|/(3V)`, `size` the box's diagonal,
measured only where the volume is outside the row's bar. Its area: the sum of its faces' own bars,
which holds the faces the per-face check cannot (unmeasured, laid by the reader, disputed). A
reading within that but outside its row's own bar (the readers agree closer than the file closes,
and it does not) is said (`read~`), not failed, and so is a face within its bar but outside the two
readers' agreement.

**Every file not read right has an account** (`step_corpus/account.tsv`, a verdict, a witness entity
and the evidence, written into the table by `table.py`): `gap`, the reader's; `defective`, no
solid's boundary by ISO 10303-42 — a loop not closed by vertex identity, a loop's sense against its
face or its neighbours, a closed shell using an edge otherwise than once each way, bounds that cross
or overlap, a face that covers itself, crosses its surface's self-intersection or has no area at
all, a whole surface that meets itself ("the domains of faces and closed shells, if present, are
2-manifolds", a 2-manifold "a non-self-intersecting surface"; a horn torus, which only touches
itself at a point, is read), a real written `NaN` — or edges standing off their faces past
`sound::LOOSE` of its size, each shown without the kernel's reader (`check.py`: OCCT's analyzer
healing off, OCCT's projections of each edge's curve onto its faces' surfaces, the text's own loop,
use and sense checks); `assembly`; `nosolid`; `open`, not settled by the evidence in hand;
`reference` and `bar`, a row corrected; `unchecked`, faces whose one reference no second reader has
checked, a face past its bar of it said and not failed until a third reader's table holds it. A file
that represents no solid, an assembly, or a defective file is finished by its refusal, and a
defective file read is no pass until its account says what its own geometry holds; a dispute the
account settles (a reader shown to misread) is no excuse. `step_corpus::nothing_is_read_wrong` and
`nothing_panics` hold at every commit; `the_corpus_reads` is the stage's finish line, its table the
account by source and verdict; `the_table_is_whole` holds every account to a verdict of the
vocabulary and a witness.

**The corpus holds each face, not only the totals** (`step_corpus::Refs`, `face_bar`,
`faces_judged`; `scripts/step-corpus/faces.py`). A total within its bar does not show a solid read
right: a face read at half its area in a part of thousands moves the total by less than a loose bar
(CADGenBench 214's face #28635, its sheet's whole domain, read 11.27 mm² against 16.82 while the
file's totals passed). So a file read right in total also has each face, named by its ADVANCED_FACE,
within its bar of OCCT's area of that face and of brep-bench's measurer's, and, where a third
reader's quadrature over the file's own boundary has measured it (`oracle/<path>.quad.faces.tsv`,
`face_quadrature`), of that, a reader past its bar of it set aside for that face. OCCT's faces are
named by the shells' order where the kinds agree, else by OCCT's own record of the translation (a
face it split is the sum of its parts); the measurer's faces are put on their surfaces from the
edges' curves in space, scaled by the unit read off the faces both readers share. The bar is `Σ
max(tol, u)·length + 1e-6·area + 1e-9 mm²` over the face's edges — the area a tolerant boundary's
slack can sweep, at the edge's measured tolerance or the file's stated uncertainty, whichever is
larger, plus the quadrature's — its constants measured on the corpus (the worst agreeing face past
its slack stands 9.4e-8 of its area off). Where the two readers disagree past the bar the face is
disputed and may be read to either; a reader whose faces do not sum to the file's area is no
reference for any of them; a table missing or of other bytes is said and counted, never taken for
agreement. The per-face tables (340,000 faces) are read from `$STEP_CORPUS/oracle/` at run time, not
committed. What the bar cannot see: a thin face's slack can approach a tenth of its area.
`step_survey --faces [--past-sound]` prints the same judgement a face a line. Gates:
`step_corpus::nothing_is_read_wrong`, `step_mutants::no_mutant_is_read_wrong`.

**Never wrong on files written wrong** (`tests/step_mutants.rs`, `scripts/step-corpus/mutants.py`).
Each mutant of the corpus — one edit of a well-formed file: a sense or an orientation flipped, a
face dropped or listed twice, vertices swapped, a vertex moved, a radius scaled, a direction
negated, a knot or a weight changed, a pcurve pointed at another surface, the unit or the
uncertainty changed, a reference retargeted to the wrong type, to nothing, or to itself — is
refused, or read to OCCT's reading of that same mutant within its row's bar; never read where OCCT
finds no solid or an inverted one; never a panic; never more than 60 s. Where an edit leaves the
curves and surfaces whole and only loosens a vertex, the bar is the corpus's own rule at the
tolerance OCCT's reading ends with, and the unedited solid's measures are accepted too. Gates:
`step_mutants::no_mutant_is_read_wrong` (over `$STEP_CORPUS/mutants/`),
`step_mutants::kernel_written_mutants` (333 mutants of six solids the kernel writes, on every
push).
