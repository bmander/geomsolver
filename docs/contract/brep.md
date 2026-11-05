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
A traced pair is seeded where edges cross faces; between analytic
surfaces `ssi::unseen` then searches one face for curves no edge reaches (cells proved clear by
the signed distance's Lipschitz and second-order bounds, else beside a known curve, else seeded),
refusing what it cannot resolve: an untraced meeting is never read as none (issue #58).
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
result's check (`swept_closed::a_block_leaning_to_the_axis_turned_part_of_a_turn_is_its_slices_swept`). A fixed polygon of a
circle once cut its chords short by more than a narrow strip beside it is wide, and read the strip
at the hole next to it (a turned rim's chamfer
from 32 out, `round_lowering::a_wide_turned_rim_is_chamfered_and_rounded_exactly`).
A line meets a cylinder or a sphere in closed form (`query::line_cylinder`, `line_sphere`, one
chord, `line_chord`): one touch exactly at the foot where it is tangent, in one evaluation where a
sampled search takes thousands. Two coaxial surfaces of revolution meet in circles where their meridians cross in one half-plane,
two spline meridians too (`ssi::spline_spline`: the first read against the second's signed distance,
its nearest point's normal; left traced where the two run together over a stretch), so two swept
edges' hyperboloids meet in exact circles
(`meeting::two_coaxial_hyperboloids_meet_in_circles_in_closed_form`). Any other curve's meeting
with a surface, a spline meridian's with another and a planar step's with another are read by one search
(`util::roots::sampled_roots`): sign changes of the distance between samples are crossings, and a
**touch is a root of the distance's rate**, bracketed where it changes sign beside a sampled
minimum of the distance's size (ties included) — never a least `|f|`, which a double root places
only to the square root of `f`'s rounding, too loosely for the curves meeting there to share its
vertex (a round's contact on the face it rolls on; a cube's edges touching its midsphere lost their
twelve touches, and the sphere's pieces a hole meeting an outer cycle at no vertex:
`roots::sampled_roots_place_a_touch_where_the_derivative_turns`,
`brep::a_cube_trimmed_by_its_midsphere_is_pinched_at_each_edges_middle`, `midsphere_common.sv`).
**A curve's meeting with a surface is certified wherever a bound is known** (#777 M1, V1):
`util::isolate` drops a stretch where the function cannot reach nought across it (its first
derivative's bound), reads one root where it is monotone (its second's), halves it otherwise, and
calls a touch only a stretch narrower than its width that cannot be parted; over several unknowns
it isolates each root by Kantorovich's theorem, and reads a stretch it drops where the function
turns back within the tolerance of nought as a touch, as a sampled search did. A stretch is read
monotone only by a margin past rounding — one ending on a double root has `|g'|` at its middle all
but its bound, and claimed monotone there, its ends a hair below nought, lost the touch to both
sides (`roots::a_double_root_a_stretch_ends_on_is_found`; a cone's rim circle tangent to a slid
frustum's side) — and a crossing is polished to rounding, as the sampled search's was (a seam's
crossing a hair off left a chamfered hemisphere's edge out of order). Its certificate
is the root: Kantorovich's nearer radius is read as `2η/(1 + √(1 − 2h))`, which over a system
all but flat (a line against a sheet fitted flat) does not cancel to nothing, and a first Newton
step out of the ball halves the box rather than returning its middle
(`roots::a_root_certified_over_a_flat_system_is_where_it_is`; once it put a slid bar's corner
edge on its plane at 7.86 rather than 9). **An edge a Boolean meets with a surface and a ray a
point is located by are read alike** (#780 R2, `query::curve_surface`; there is no `ray`
switch): a line meets a plane, a cone, a torus and a conic turned (`query::line_revolution`, a
quartic in the meridian's own parameter) as polynomials along it, a cylinder and a ball as a
chord, a swept sheet or a B-spline net by the three-unknown search (`query::curve_sheet`, the
sheet's second derivatives bounded by `blend_bounds` and `Net::second_bounds`); a circle or an
ellipse meets a plane, a cylinder, a ball, a cone and a torus as the surface's algebraic form
along it, a trigonometric polynomial bounded by its amplitudes (`query::conic_algebraic`); and a
B-spline curve meets a plane as its poles' heights, a scalar B-spline bounded by its
differences span by span (`nurbs::Heights`, `query::spline_plane`)
(`swept_closed::an_edge_is_read_certified_and_settled_as_a_ray_is`,
`…::a_conic_and_a_spline_edge_are_read_certified`). **Whatever any reader returns is settled by
one rule** (`query::settled`): a curve lying in the surface (read at its quarters, then at the
sampled search's steps) is `Along` before any reader is asked; a root past either end by more
than the tolerance is no meeting, one within it is at the end; a run of roots the curve stays
within the tolerance of the surface between is one meeting — a crossing where the surface's side
changes across it, read where the curve has left the tolerance, a touch at its middle where it
does not (a double root rounding parts, a root two spans or stretches both find, and the noise a
polynomial's rounding makes where a line runs along a torus's tube top, its distance the fourth
power of the way: hundreds of crossings a micrometre apart before); and an end on the surface is
a crossing there, taking the run the curve keeps within the tolerance beside it, as the sampled
search reads it — not only the roots within the tolerance of it along the curve: a touch rounding
places a little way in from an end on the surface, kept beside the end's crossing, left a
plane's pieces open (`round_lowering::rounds_that_run_out_onto_a_face_are_rolled_and_cut_off`,
`rounds_examples::the_runouts_are_the_stacks`). An edge clipped to a face's box
(`query::curve_surface_within`) is read stretch by stretch and settled once over the whole, a
stretch's end cut at the box no end of the edge. That edges within
the tolerance of tangent were read one way by the polynomials and another by the samples is why
a Boolean's edges once sampled the cone, the torus and the sheets
(`round_lowering::a_round_across_seams_between_arcs_on_a_drafted_prism_is_glued`,
`prism_twists::a_cap_of_a_tool_no_lone_prism_turned_is_the_closed_classs`, both building now). A
level rim's sheet is met by its own bound on one unknown, kept as the faster road and held to the
search (`swept_closed::a_rims_sheet_in_closed_form_is_held_to_the_general_search`). Only a pair
with no certified reading samples, and the build's profile counts it (`util::meter`). **A point
where three surfaces' meetings meet is solved once** (#711, #777 M2:
`Arrangement::triple_points`, keyed by the three surfaces' indices among the shared meetings), and
every later finding of it from another of the three faces takes that vertex.

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
roots of `f` along `n` at each of its points, isolated with a bound (`ridge_branches`, #777 M1) —
however shallow, the fold solved in its two-root form. A crossing with no ridge, too shallow for
the march along `∇a × ∇b`, is traced as `f = 0` in either surface's own parameters
(`level_curves`: one scalar equation, its tangent square to `∇f`, its step halved where it turns
sharply, as at a parabola's vertex where a plane leans a hair to a vault's crown). A point is on
such a branch within `8 tol / sin θ` (`on_a_branch`, A3's reach). **A meeting's analysis is done
once** (#780 R4): `meet_with` finds each side's critical set once and reads it for the singular
points, the ridges (`ridges_from`) and the seeds alike; the branches a point is asked about are
sampled once into chords filed by their boxes (`meeting::Sampled`), a point held only to the
chords within reach of it and no foot sought beyond reach of a branch's poles; and a level curve's
step goes to the point its look-ahead projected, the next station's gradient the one read there.
Where the profile is printed it counts each (`util::meter::analysis`: critical sets and their jets,
branch tests, samples and feet, a level curve's readings of `f`). `meeting::meet` reads the ridges
on its traced road before it marches; an arrangement reads them where its march is refused as nearly
touching (`Arrangement::traced_meeting`). Both go through **one fallback** (`meeting::marched`, #780
R3): the march, and where it is refused as shallow the ridges (where not read yet) and then the
level curves through the seeds no branch holds, each touch a stretch the march ends at, each seed
then traced alone and one refused again passed by (`traced_alone`). **The trace's refusal below
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
A minimum is a `util::roots::Least`, its place and its value named, never a pair.
**A flat between two planes is a plane** (`fillet::flat_along`): the prism on its section's
triangle, carried on past each end and cut off there, no net fitted. A fitted flat is a B-spline
face, and a run chamfered after it that ends on it was refused, nothing cutting a run off at a
spline: a box's top edges chamfered one after another, each cut off by the plane of the one before
(`brep::a_box_top_chamfered_unequally_is_ten_planes`, brep-bench's chamfer/box-top-asymmetric;
`brep::chamfers_between_planes_are_planes`). A run going on tangent into a curved face at an end
is still fitted, as the piece that takes on from its section is. About one axis the flat is turned: between a plane square
to it, cylinders and cones about it, a closed loop's flat is the section's triangle revolved, a cone
exact (`fillet::coaxial`; a leg shrunk to a point its apex, [rounds.md](rounds.md)). So is a
round there (`fillet::roll`, `fillet::revolved`): the section — the corner, its legs to the
contacts and the ball's arc — in the meridian through a vertex of the loop, revolved, a torus face
and its strips exact, no canal fitted. A fitted canal round a circle was a net of hundreds of poles
that every later Boolean cut and every measure integrated: brep-bench's fillet/boss-s-curve took
8.7 s, most of it its area, and now takes milliseconds, its volume and area to 1e-15
(`brep::a_bosss_rim_and_root_rounded_to_meet_are_two_tori`; the hole's waist too).
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
**A spline extruded meets a surface the same all along it in closed form** (#644,
`ssi::extruded`): a plane along the extrusion or a cylinder about a line along it in the lines
along it through where the spline crosses or touches that surface, exactly; a cone about such a
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
each a Boolean's selection with none falling back). **A2, the meeting** (`brep::meeting`):
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
are the gates. **A3, tolerant topology** (`brep::tolerant`): every rule by which two features are
the same in one place — a point onto a vertex within `tol` (`Pool`, each vertex keeping how far the
farthest point merged into it stood: `topo::Vertex::tol`, which `check` reads an edge's end
against), a vertex onto an edge and an edge onto an edge within `ON_CURVE` tolerances, a point onto
a meeting within `on_meeting`, a flat root onto a crossing within `FLAT`, a sliver into its edge —
applied once, in that order. A search's warm start (`util::warm`: a sheet's, a blend's, a wall's
last foot) is forgotten at every arrangement and every parallel item and kept as it was across
narration, so a split is the same bits with `SOLVENT_BREP_DEBUG` set and unset and on any thread
(`arrangement::a_split_is_the_same_narrated_or_not_on_any_thread`). **A4, the arrangement**
(`boolean::arrangement`, `arrangement_within`, `members`, `selected`): any number of solids' faces
split by all the others at once — where three faces of three operands meet at a point no edge
passes, the two meetings on each face cut there (`triple_points`) — a span two faces only touch along
laid as an edge, every piece a face of the complex but a sliver, coincident pieces one face naming
each operand; a traced meeting along an edge one face already has is that edge, laid once (`along_an_edge`),
and laid across the other face too wherever it lies inside it (`Meetings::along_edges`: a torus's
seam in the plane through its axis that holds it, `brep::a_torus_cut_along_its_seam_is_half_of_it`); each cell's membership in each operand read off a face of it bounding the cell, else at
a point deep inside. **A Boolean is a selection over it**: each piece's two sides' membership read
as before, the complex formed, each shell kept as the operation keeps its sides — which agree round
it, with no face kept by its own sides instead: a shell kept on one side and dropped on another is
refused, two cells joined round an edge — and the result touching itself along an edge given an edge for each
wedge; `split` is the complex's shells inside the solid. **Every arrangement lays its touches as
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
its section's run in closed form; `a_split_heals_a_touch_out_of_each_cell`). **A Boolean is the
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
along a curve touches it at its crossing with that curve (`touch_root`), not at a double root
placed to the square root of the tolerance; two closed-form curves of one meeting are crossed open
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
(`swept_closed::round_tools_turned_part_of_a_turn_sweep_their_sections_by_pappus`). **A face's
pieces are ordered by their curvature where they leave a point tangent** (`boolean::pieces_of_face`):
each half's way out of a node is read a ten-thousandth along its own length, and where two of the
ways there agree within `TANGENT_TIE` (a thousandth of a radian, what a step along an arc turns its
chord) they are read again at one length along each — a ten-thousandth of the shortest of those
tied, each half's chord to its point that far from the node, the part of its way there found by
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
carries to the circle's height onto the circle, `|w(u) − R_{s(u)} c₀|² = r²`, isolated with a bound
(`ssi::screwed_line_circle`); a cone where each ruling meets it, `A(v) u² + B(v) u + C(v) = 0`, the
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
settles** (`props`'s `settled`): read over its pieces, then twice as many, until two readings agree
to rounding, the coarser kept — a ball's great circle passing within a few degrees of a pole sweeps
half a turn of `u` there, and a sixteenth of a turn a piece left a ball's piece 0.2% off its area.
**A sheet a line square to a screw's axis screws into meets what the screw keeps in closed form**
(`ssi`'s screwed meetings, a `Blend` of a line and itself carried by a screw): a coaxial cylinder
in the helices of the line's points at its radius (`Curve::Iso`), one where the line passes at it
(the sheet touching it along the foot's helix, judged by the gap); another such sheet of the same
pitch in the helix of the point where the two lines meet carried level, none where they run
parallel there; a level plane in the one ruling at its height; any other plane in its cut
`S(−α/β, v)` (`Curve::SheetCut`, **a new curve kind**: `α + u β` the ruling's reach to the plane),
with the rulings it holds as lines, read about the nearest held ruling by differences so the cut
runs through the point it touches the sheet at as exactly as anywhere. Two curves on one sheet,
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
**The recipe is read once** (`recipe::Recipe::read`): every reader — this kernel's build and
meridian, `section`, the swept cutter and sector, `solid::exact`/`canal`, the native host's
`Session` — matches a typed `recipe::Node`, never the JSON; a profile stays as written
(`recipe::Section`, the numbers the native host hands on) until `Profile::of` reads it. A new node
or field is one arm of `Node::read` and of each match. Every writer writes its points through one
`recipe::Writer` (millimetres, about the world's origin or `cad::recipe_about`'s). **A kernel error
has a kind** (`brep::KernelError`, `Kind`): `Brep::check`'s edge used as often each way, more than
once, is `NotManifold` (material touching along it: what `boolean::combined` and the body's
stratum act on), one used more often one way or never `Open` (a face missing: the kernel's own failure,
which the stratum never reads as rounds meeting; `brep::an_edge_used_twice_each_way_touches_and_one_used_once_is_open`), a recipe of another kind
than a route builds (`Recipe::meridian`, `Sectioned`) is `Refused` (its caller builds it
another way), an unreadable recipe `Recipe`; its message is all it says, and it becomes a `String`
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
point off the longest side; points nearer than
1e-9 are welded, and a vertex a face's loops pass through twice at one place (a hole touching the
outer loop, #597) is one corner of its triangulation. An edge is sampled from first pieces no
midpoint test can be blind to: a B-spline's knot spans, a family rail's pieces, a screwed rail a
quarter turn at most (`edge_params`; a coil of four turns has its ends, middle and quarter
points in one line, and was one chord). `Mesh::turned` counts facets facing against their
surface (thicker than bar/100); the export refuses any. A face whose facets beside an edge face
against it — a sheet folding along that edge tighter than the edge's chords sag, which no point
inside the face can mend (a round rim screwed a short lead, its ramp folding against the wall its
side sweeps over about a tenth of a micrometre) — asks the chords those facets stand on halved, and
the faces are meshed again with the edges so sampled, up to `TURNED_ROUNDS` times
(`closed_characteristics::the_drum_screwed_bores_its_helical_groove`, its STL agreeing with the
field).
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
as it is: an edge in the blank divides nothing; `brep_export.rs`'s 20-tooth gear), blank split
(`boolean::split`), cells judged by the field (`sweep::sector`; else `sector::whole`), sector
patterned. `brep::export::{exact,step,stl}` is shared by `solventc` and `app/export-worker.ts`
(`gcs_solid_exact`), so native and wasm files are byte-identical: the core's trigonometry, `exp`,
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
starts on a coil's next turn); `inverse_near` runs Newton from its hint first; `step::blend_net`
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
Rung 2 is done: `Curve::BSpline` (`brep::nurbs`), `Surface::Extrusion`/`Revolution`, a face's
stretch of a traced or formula curve (`k from p to q`, `CurveE::trim`, `cad::FIT_MM`), and a
section carried to its own image (`build::ruled`: a drafted prism's offset, a canal's screwed disc;
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

**Offsets are the union of the pieces their points are nearest** (`brep::offset`; brep-bench's
offset, shell and thicken cases).  A point outside a solid within `d` of it is nearest a point
inside a face, on an edge or at a vertex, so the solid grown by `d` (`offset::grown`) is the solid
with each face's **normal sweep** (`normal_sweep`: every `q + s n(q)`, `s` over `[0, d]`), the tube
of radius `d` about each crease the faces part at, and the ball about each corner nearest which
some point is — a corner where two or more such creases meet and none turns the other way, or a
corner of no crease whose faces' normals differ there (a cone's apex; `smooth_at` reads each face
just inside it, so a sphere's pole is smooth) — every piece within `d` of the solid and every such
point in a piece, united by the Booleans.  Shrunk (`offset::shrunk`) is the dual, the solid less
the inward sweeps and the tubes and balls of its concave creases and corners.  Nothing is
classified by shape: where moved faces meet (a groove grown shut, a flange shrunk away, a torus
grown past its hole into an apple) the union decides, as the definition does.  **Sharp joins**
(`Join::Sharp`) are the other set an offset is asked for, its faces moved and extended until they
meet: along a line between two planes the prism of the kite between the edge, its moved copies and
the line the moved planes meet in; at a corner of three planes the parallelepiped the moved planes
and the faces' own bound; anything else refused by name.  **A shell** (`offset::shelled`) is the
wall within `t` of the faces not left open — inside, the solid in common with the pieces of those
faces; outside, the pieces less the solid, cut off at each opening's plane.  Inside, a moved wall
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
this face's.  A surface with no offset in closed form, or a curved face not bounded by its own
parameter lines, is refused naming the face and a point on it — never approximated.  A known limit of
the Booleans, not a rule of offsets: a meeting through a sphere's pole is read less exactly than one
clear of it (a pole near a corner ball's kept boundary put a grown tetrahedron 9e-7 off), so a
corner's ball is built with its poles along the corner's mean normal, clear of the circles the
tubes end on; the deeper fix splits a meeting at a chart's degenerate point.  `tests/brep_offset.rs` is the gate: a box, an L, a tetrahedron, a cone, tori past their
hole and shrunk, a groove filled and a flange taken away, boxes and caps shelled, an open box and a
bend thickened, each to its closed form to 1e-9; a spline sheet refused.

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
