# The Rust B-rep kernel

Part of the [contract](../../CLAUDE.md) the code is held to. Read it when you touch `brep/`. An
issue number (#n) or a spec version ([0.n], see the [language changelog](../solvent-changelog.md))
says where a rule came from.

**The Rust B-rep kernel** ([plan](../rust-kernel-plan.md)); clean-room — OCCT is LGPL, this repo
MIT: its code is never translated, only its behaviour and conventions studied.  It is its own
crate, `cad-kernel` (`rust/cad-kernel/`, MIT OR Apache-2.0), which names neither the solver nor the
language; `gcs-solve` re-exports it as `brep` and adds what reads the model (`brep::sweep`,
`brep::export`), so its paths below are `gcs_core::brep`'s as before.  Its own repository
(https://github.com/bmander/cad-kernel) is this directory's history, published by
`scripts/sync-cad-kernel.sh`; there it builds and runs its smoke test (`rust/cad-kernel/tests/`)
alone, and it is measured beside other kernels on https://github.com/bmander/brep-bench.
`gcs_core::brep`:
`geom` (surfaces parameterised as OCCT/STEP do, closed-form inverses, signed distances, `Traced`
intersections), `topo` (`Brep`: oriented uses with pcurves, degenerate poles; `check`, `pinches`),
`build` (prisms, drafted prisms, revolutions and canals of line/arc/circle/spline profiles; the
sphere, the cone and the torus by their numbers as OCCT's `MakeSphere`/`MakeCone`/`MakeTorus` take
them, each its meridian turned — `sphere(c, axis, r)`, `cone(o, axis, r0, r1, h)`,
`torus(c, axis, major, minor)` — and what is no such solid refused by name: the kernel's API
speaks the vocabulary its other users bring, which the language need not, `std.sets.Sphere`
staying a set over `distance`; `tests/brep.rs`'s `the_sphere_the_cone_and_the_torus_…`),
`props` (volume by Green's theorem), `query`, `ssi` (closed forms, else traced), `boolean` (split, arrange, classify,
assemble), `mesh` (constrained Delaunay refined to a measured sag), `step` (AP214 with pcurves and
seam curves), `recipe`. A traced pair is seeded where edges cross faces; between analytic
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
boundary that way is refused naming its face and the other's it touches. A fixed polygon of a
circle once cut its chords short by more than a narrow strip beside it is wide, and read the strip
at the hole next to it (a turned rim's chamfer
from 32 out, `round_lowering::a_wide_turned_rim_is_chamfered_and_rounded_exactly`).
A line meets a cylinder or a sphere in closed form (`query::line_cylinder`, `line_sphere`, one
chord, `line_chord`): one touch exactly at the foot where it is tangent, in one evaluation where a
sampled search takes thousands. Any other curve's meeting with a surface, a spline
meridian's with another and a planar step's with another are read by one search
(`util::roots::sampled_roots`): sign changes of the distance between samples are crossings, and a
**touch is a root of the distance's rate**, bracketed where it changes sign beside a sampled
minimum of the distance's size (ties included) — never a least `|f|`, which a double root places
only to the square root of `f`'s rounding, too loosely for the curves meeting there to share its
vertex (a round's contact on the face it rolls on; a cube's edges touching its midsphere lost their
twelve touches, and the sphere's pieces a hole meeting an outer cycle at no vertex:
`roots::sampled_roots_place_a_touch_where_the_derivative_turns`,
`brep::a_cube_trimmed_by_its_midsphere_is_pinched_at_each_edges_middle`, `midsphere_common.sv`).
A minimum is a `util::roots::Least`, its place and its value named, never a pair.
**A flat between two planes is a plane** (`fillet::flat_along`): the prism on its section's
triangle, carried on past each end and cut off there, no net fitted. A fitted flat is a B-spline
face, and a run chamfered after it that ends on it was refused, nothing cutting a run off at a
spline: a box's top edges chamfered one after another, each cut off by the plane of the one before
(`brep::a_box_top_chamfered_unequally_is_ten_planes`, brep-bench's chamfer/box-top-asymmetric;
`brep::chamfers_between_planes_are_planes`). A run going on tangent into a curved face at an end
is still fitted, as the piece that takes on from its section is.
`Brep::bounds` encloses (issue #59; `through:` cutters, tool culls): `Curve::bounds` exact for
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
surface (thicker than bar/100); the export refuses any.
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
