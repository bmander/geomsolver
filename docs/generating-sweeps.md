# Generating sweeps: the supported class

**Decision (2026-09-22).** The goal is a declarative hypoid gear pair. The two prerequisite
projects set on 2026-09-10 were an arbitrary spacetime sweep and reliable Boolean meshes.
The first has turned out to be a research problem. Five milestones and Phase 3a–3d of
`solid/swept_boundary` close the closed-form cases. The tumbling, slid and tilted cylinders
and the thin plate still refuse. The ordinary cylinder still fails 324 centroid checks
through the production constructor. We therefore stop pursuing arbitrary sweeps. Solvent
supports one class of sweep: the class that generates bevel and hypoid gears. It refuses
everything else, naming the condition that failed.

This is a scope reduction, not a relabelling. A sweep outside the class is refused even when
some construction happens to produce a plausible mesh for it.
[The execution plan](generating-sweeps-plan.md) holds the phases and exit gates.

## Why this class is tractable

For a full revolution under a rigid motion, the contact condition on one circle of latitude
is `A cos θ + B sin θ + C = 0` (`RevolvedSurface::contacts`). It has at most two roots, and
they are found in closed form. The swept boundary is then a small, known set of pieces:

- one envelope per profile segment (line, arc or circle), and
- one sharp-edge sweep (a fan of normals) per convex crease.

A rack (a prism under a translation seen from a rotation, 2026-10-03, issue #61) is simpler
still: the contact condition of a point of the prism's side is affine in the roll,
`s n·d − n·(ω×(p−o)) − s t n·(ω×d) = 0` (`NormalVelocity::Affine`), one root in closed form.

Nothing about the boundary has to be discovered by tracing an arbitrary field. If the tool
clears the blank at both roll limits, the end caps lie outside the blank and need not be
built. A general tumbling sweep has none of these properties, which is where the
general project spent its effort.

## The class

A sweep `removal := sweep(tool over t in (a, b), under: generating)`, or `sweep(tool, through:
body, …)` over the
roll it takes through the body's material (`solid::roll::interval`, the one interval every reader
asks), is **admitted** when all of
the following hold. Each condition is a predicate evaluated on the solved model, and each
failure is reported with its witness.

| # | Condition | Why | Refusal names |
| --- | --- | --- | --- |
| T1 | The tool is a full revolution with a line/arc/circle profile — a region solid among them, its profile its meridian (`solid::region::meridian`, its faces `RevolvedSurface::meridian_loops`, #209) — a prism of lines and arcs, drafted or not (a drafted one's corners joining lines or going on tangent, drafted inward where it runs both ways from its parting plane: below), or an intersection (`common`) of such, under fixed placements, its caps clear of the blank over the roll unless it is one undrafted prism standing alone or placed (below, #684 F, AC; under a screw, a turn or a slide, AE); or a union of such tools, each member swept alone (below). | Every face has closed-form contacts. An intersection creates only convex creases. A prism within another tool, or drafted, cuts with its sides only (`ExtrudedSurface`); an undrafted prism alone or placed is sectioned across where its cap reaches the blank (T3), or under a constant twist its characteristic walked over the cap. | The first operand outside the class, a cut in the tool or a union within an intersection, the prism whose cap passes through the blank where it is not an undrafted prism alone or placed, or the drafted prism and why its sides are not read. |
| T2 | Every profile vertex inside the swept region is convex or tangent. | A convex corner sweeps a fan of normals. A concave corner sweeps nothing, but it trims two envelopes against each other, which is not supported. | The vertex and its turning angle. |
| M1 | The motion is one rotation about a fixed axis, or one rotation (or one translation) relative to another rotation (`motion(about: …)`, `motion(along: …)`, `motion(m, relative_to: n)`), at constant ratios, with a finite increasing roll. | This is the generating motion of a face-milled bevel or hypoid, including skew axes, and of a rack cutting a spur gear. | The motion node outside the class. |
| M2 | Each tool point's contact condition changes over the roll, or holds a nonzero value. | Where it is zero throughout, the point is on the boundary at every time, and a contact time cannot parameterize the surface; a nonzero constant touches at no time and bounds nothing (#684 F: a prism's side square to a rack's line of centres). A single rotation is like that everywhere (its tool-frame twist is constant), so the generator's motion is a relative one. This is the native path's existing refusal. It is checked at the samples, at a pole (whose normal is the axis), and between samples: the equation is C + a cos + b sin in the roll, and where (a, b) turns right round between two samples, or winds round a sample cell, with roots on every side, a point there has C and the amplitude both zero. That happens where the tool's normal passes through the point where two meeting axes cross. | A tool point whose path reaches the blank. |
| E1 | The tool at both roll limits has no common volume with the blank. | Then no end caps are needed inside the blank (the existing refusal). A sweep rolling `through:` its body (#271, `solid::roll`) holds it by construction: its roll is found by marching out from 0 until the tool is proved clear of the body's material, so only a stated `from:`/`to:` can fail it. | The limit and the overlap volume. |
| E2 | Every source point whose contact reaches the blank has at most one contact time within the roll. | One sheet per placement, as the bevel pinion already satisfies. | The source point and both times. |
| E3 | The generated area factor J stays away from zero over the region reaching the blank. | A zero is a fold of the envelope: the cusp of undercut. J alone passes through infinity where a point's two contact times merge, which is a fold of the time chart and not of the surface. So the sign is read from J times the rate of the contact condition, which changes sign only at a true fold. | The contact point and the factor. |
| E4 | A placement's sheet does not cross itself inside the blank. | Sheets of different placements may cross, since the kernel split handles that. A self-crossing sheet is the defect the hypoid investigation found (`split_at_crossings`). | The pair of sheet points. |

### Screws: the constant-twist class

A screw (`motion(about: axis, advance: lead)`) has constant tool-frame twist: every tool point
moves the same way relative to the tool at every time, so M2 fails everywhere and a contact time
cannot parameterize the surface. Instead the contact set is fixed on the tool. Its **characteristic**
is, on each ring of a revolved face, the roots of the same `A cos θ + B sin θ + C = 0`, read once
under the screw's twist, and the boundary is that curve carried along the screw:
S(s, t) = M(t)·c(s), its normal the tool's. `admit_body` dispatches on `Family::screw()`. A screw's
sweep is asked T1, T2, M1, E0 and E1 as above, then (`solid::constant_twist`):

| # | Condition | Why | Refusal names |
| --- | --- | --- | --- |
| S1 | The characteristic is regular through the reach and a margin past it: no ring is about to lose its two roots (a double point, where the curve turns back on the ring), none is stationary, the walk never jumps at a concave corner, and it meets no convex corner at one point from both faces. | The sheet is one regular curve carried along; at a convex corner it runs round the corner's fan (below). Met at one point, a convex corner leaves a sharp edge in the sheet, which is not built. | The face or the corner, and the ring. |
| S2 | At most one characteristic point of a ring reaches the blank, and every point that does lies on one stretch. | One sheet per placement, one point per station. | The second point. |
| S3 | The characteristic never runs along the screw's velocity: `(c′ × v)·n` keeps one sign and stays above `least_factor`. | A zero is a fold of the sheet. Signed, so a fold passed between two samples is seen by the sign it turned. | The face and the factor. |
| S4 | The section square to the axis (each characteristic point carried along its path to one height) does not cross itself. | The sheet is the same at every height, turned and raised, so it meets itself exactly where its section does. | The two points. |

The walk samples `rows` rings along every face, then samples the stretch it found `rows` times
across its reach, so a reach that is a sliver of a large round is asked as finely as one that is a
whole face. Each point's path through the blank is read a degree of turn apart. These rows are
sampled too.

The sheet (`brep::sweep::helical`) is exact at every node and every withheld point: rows run along
the sheet's section square to the axis, `Characteristic::on_section`. The characteristic itself has
a corner wherever the tool's profile changes curvature, by a step along the screw's path that the
sheet does not show, while its section is as smooth as the sheet. Columns run along the roll over
the stretch that carries the section past the blank. It is fitted, judged and refined by the loop
the traced sheet shares (`sheet::settle`). The split, cells, agreement and files are the generating
class's. `rust/examples/twist_drill/` is the worked case (`tests/twist_drill.rs`).

### Turns and slides: the constant twist's two limits (issue #684)

A turn about one axis (a screw of no advance) and a slide along one line (a screw of no turn) have
constant twist too, so their contact set is fixed on the tool as a screw's is, and the boundary
is the characteristic carried. `Family::twist` reads all three, a slide as a screw turning at no
ratio about the line along its direction through the origin; `admit_body` asks each the rows above,
T1, T2, E0, E1 and S1–S4. What differs is the section the sheet is read along (`Screw::station`):

- a **slide's** is the plane square to it, a station a height along it, as a screw's is; its sheet
  is the section extruded, its columns spread across the blank's heights (eight to start, the fit
  refining them), and each point's path through the blank is read in 360 steps across them;
- a **turn's** is a half-plane through its axis, a station an angle about it: each characteristic
  point is carried round to it at its own height and radius, so the sheet is the section revolved,
  its columns over the roll (a turn keeps every height). Each node's delay to the section is read
  along the walk, within half a turn of the one before, as the section is one curve. S4 reads the
  section in its half-plane, by radius and height.

One refusal is the limits' own:

| # | Condition | Why | Refusal names |
| --- | --- | --- | --- |
| S1 | No ring the walk passes between two of its stations is in contact all round where its path enters the blank. | Such a ring is all characteristic, and the boundary leaves the walked branch along it. A turn about an axis the tool's own axis meets has one, every normal of it through the point where the axes meet; a slide along the tool's axis has one wherever the normals are square to it. The roots either side need not move (the ring's two roots stay in the plane of the two axes), so a sampled station never sees it: the contact equation's amplitude turns right round instead, and where it turns more than a quarter turn between two walked stations, the least of the ring's three coefficients between them is sought (Brent's method). The field gate found the first one, on a torus leaning toward the turn's axis, before this row did. | The face, and a point of the ring in the blank. |

E1 holds as for a screw, so a slide passes the tool right through the blank and a turn swings it
across from clear to clear, unless the turn is a whole one (below). Still refused: a flat face the motion keeps in its own plane (a cap
square to the turn, a face along the slide) where it reaches the blank, a ring in contact all
round at a sampled station (S1); and a characteristic that closes on itself within the blank's
reach, a tool passed wholly through it (S1). A prism tool is read along its rulings (below, #684 M). A ball under
a turn or a slide is still its tube (`solid::canal`, below), never asked these rows.
`tests/turns_and_slides.rs` and `tests/solid_sweeps_examples.rs` are the gate: a torus slid up a
rod, square (its tube's lens, `rust/examples/solid_sweeps/slide.sv`) and tilted, a cylinder slid
with its caps clear (a circular segment), and a torus turned across a ring sector, upright
(Pappus) and leaning across the turn (`solid_sweeps/turn.sv`), each removed volume computed apart
from the core — a tilted torus's from its centre circle alone — and held to the exact solid, whose
STL agrees with the field; and the straight-fluted drill
(`rust/examples/twist_drill/straight_flute.sv`, the twist drill's wheel slid square to the axis,
its flute the wheel's own profile).

### A whole turn (issue #684 N)

A sweep depends only on the path its motion takes, not on how fast or how often it is taken. A
turn carried round a whole turn or more therefore sweeps what one whole turn sweeps: a solid of
revolution, its boundary the characteristic's section in a half-plane through the axis revolved
all the way round (`Characteristic::whole`). Its contacts are read once round from the roll's
start where they are read (`SweepContacts::read`, `whole`, `domain`), so the walk, its rows, the
sheet and the accuracy meter all ask and build of a roll of two turns what they do of one. The rows are a turn's, S1–S3 and S4's
section read by radius and height, but for one:

- **E1 is not asked.** The tool at either end of a whole turn lies within what the turn sweeps,
  so its ends need not be clear of the blank and no end pose is built: the roll may start and end
  inside it, as a part turned once in a lathe against a form tool does.

The sheet (`brep::sweep::helical`) is laid as a turn's is, its columns once round from the roll's
start, the last column's pose the first's to the bit, so its first and last columns are one curve.
Only its rows' ends (the characteristic's ends) must leave the blank; the columns' ends lie where
they will. Fitted and judged as any, the sheet is then cut into faces round the turn
(`Fitted::named` with `around`, `brep::build::grid_sheet`), four of them, and where it runs onto
and off a convex corner's fan, into bands across them too, every neighbour sharing its edges and
the last round closing on the first's. One face of a closed surface would carry a seam across the
blank, which a split does not finish; a ring of faces carries an edge. The split, cells and
agreement are the class's; the blank's own seams, where it is a solid of revolution about the same
axis, are crossed as any edge is.

`tests/turns_and_slides.rs` (`a_whole_turn_grooves_the_whole_ring`: `turn.sv`'s leaning torus
round the whole ring, over one turn and over two, Pappus over its shadow six times the sector's)
and `tests/solid_sweeps_examples.rs` (`rust/examples/solid_sweeps/whole_turn.sv`: a V wheel leaning
across a rod turned once round it, its sheet twelve faces, three bands each four round; Pappus over
the wheel's shadow, read by the signed distance from the nearer flank round each circle about the
axis) are the gate, each STL agreeing with the field; `tests/object_builder.rs` builds the wheel's
at the page's samples. A ball turned a whole turn is still its tube, the canal's whole torus.

### A joint driven by a law: a sweep is its path's (issue #684 L)

What a sweep removes is every pose its motion stands the tool at over the roll, whatever the rate
each is reached at, how often, or in which order. A joint driven by a law of the roll
(`fix(value == law) f`, #391 S5) stands the tool at the joint's own pose at the law's value, so over
a roll `[a, b]` it stands it at the joint's poses over the values the law takes there. The law is
continuous (its enclosure carries only continuous functions), so those values are every one between
its least and its greatest, and the sweep is the **joint's sweep over that interval**: a turn, a
slide or a screw at a constant rate, admitted and built as one. A cam's rise, dwell and return, a
rocker swung out and back, a slide that overshoots and settles: each sweeps what its joint does
over its reach.

- **The reach** (`Law::range`) is the global minimum search the swept field uses
  (`interval::minimum`), run each way over the law's enclosure narrowed by the mean value form (its
  value at a stretch's middle and its rate's enclosure across the rest), to `RANGE` (1e-12
  radians, or that fraction of the law's values at the roll's ends where they pass a radian). Its
  least and greatest are values the law takes at the search's witnesses; a law the search leaves
  unsettled is not ranged. A law no enclosure is carried of (`tan`, `exp`, the roundings, …) is not
  ranged, and its sweep is refused as before.
- **The restatement** (`solid::paced`): a body reaching a sweep under a driven joint is built from a
  copy of the drawing in which that sweep, in its place and under its name, is the joint's sweep
  (`MotionDef::Driven`'s `of`) over a roll stated as the law's reach (`paced::lowered`). Admission,
  the exact construction, the accuracy meter and the page's stepped build read the copy
  (`solid::lowered`), before a union is distributed (#684 C), so a union under a drive is its
  members' sweeps under the joint.
  The export holds what it builds to the drawing's own field, the driven sweep's, so the gate
  compares the restatement with the law it restates. A ball under a drive becomes the canal's
  closed form; read along its path, it would stop where the law turns back (B1).
- **Not read so:** a driven joint seen from another motion (`seen_from:`, a cam's follower as the
  cam sees it). Its screw moves as the law runs, a path no constant-rate joint takes, and it is
  refused at M1, saying so. A law that stands still over the roll moves nothing and is not read.

The native OCCT path refuses such a body by name (`--kernel rust` builds it).
`tests/paced_sweeps.rs` is the gate: laws' reaches against their closed forms (1e-12), monotone,
kinked, dwelling and turning back within the roll; `slide.sv`'s wheel risen, dwelt and returned,
the slide's lens; `ball.sv`'s ball rocked out and back, Pappus; a drive seen from a turn, refused.
`tests/solid_sweeps_examples.rs` holds `rust/examples/solid_sweeps/rocked.sv` (`turn.sv`'s cutter
rocked out and back by the simple harmonic law) to `turn.sv`'s figure, each STL agreeing with the
driven sweep's field, and `tests/object_builder.rs` builds it at the page's samples.

### Sharp edges: a convex corner's fan (issue #684 E)

A convex corner of the tool's profile turns to a circle of the tool, and every normal between its
two faces' there is the tool's. A point of the circle is on the boundary wherever one of those
normals is square to its velocity, which under a constant twist is again `A cos θ + B sin θ + C =
0` on the circle, its coefficients the fan normal's (`RingFrame::coefficients`). So the corner is
walked as a piece of its own: its stations are the fan's normals, turned in the plane through the
axis from the first face's outward normal to the second's, and each station's ring is the circle.
Where the two faces' characteristics meet the circle at different points (the jump S1 refused
before), the walk runs from the one round the fan to the other, and the sheet there is the
circle's own sweep, tangent to both faces' sheets. S1–S4 are asked of the fan's nodes as of any
face's (`solid::constant_twist`, `Piece::Corner`), under a screw, a turn or a slide alike.

A join is a corner where the faces' normals turn more than a millionth of a radian apart, with the
tool's material between them; a circle's own ends and a tangent join are not. Where both faces'
characteristics meet a convex corner at one point, every normal of its fan is in contact there and
the sweep has a sharp edge along that point's path: S1 refuses it, naming the face. A V wheel slid
square to its axis is one, its rim moving along its own tangent where it is nearest the rod.

A fan is usually short and turns far: a 60° V wheel set along a thread's helix touches over 0.06 mm
of its rim while the normal turns 100°, since the rim moves nearly along itself there. The sheet's
rows are therefore laid by length along the section plus each turn of its normal counted as the
section's length over a half turn, which gives the fan a third of them, and the rows where the walk
runs onto and off the fan are rows of the grid exactly. The fitted sheet is cut there into faces
(`Fitted::named`): one sheet across the fan's sharp turn let the kernel read a point of one flank on
the other, and its split found no cell.

`rust/examples/solid_sweeps/vthread.sv` is the case: a V wheel screwed up a rod as a thread is
ground. `tests/solid_sweeps_examples.rs` holds it to a section read apart from the core (each
point's helix back through the roll tested against the V by the signed distance from its nearer
flank) to 2e-6, and `tests/turns_and_slides.rs` a V wheel slid tilted 25° to its shadow along the
slide to 3e-6, each STL agreeing with the field; slid square, it is refused at S1.

### Prisms under a constant twist (issue #684 M)

A prism's side has no rings, but it has rulings: the lines along it from one cap to the other,
one through each point of its edge. A screw, a turn or a slide moves the points of a ruling as one
affine field moves them, `v(p) = ω × p + d` read on the tool, and a ruling's normal is one (a
drafted side's too: each side is ruled between its two end sections, #684 G). So the contact
`n·v` is linear along each ruling, and a ruling has one point of the characteristic, read from its
contact at the ruling's two ends — or none on the tool, where that point lies past a cap. A convex
corner of the profile is a ruling the two sides share, carrying every normal between theirs, so
it is walked as a revolution's corner is (#684 E): a station on it is a normal of its fan, and its
ruling the same line at every station. The walk crosses from a side round a corner's fan onto the
next side where the characteristic jumps there (`solid::constant_twist`, `Piece::Ruled`), and S1–S4
are asked of every node as of a revolution's; the sheet is built as a revolution's is
(`brep::sweep::helical`), cut into faces where it runs onto and off a fan.

A ruling's root runs out along it toward a cap wherever its normal turns square to `ω × e`, `e`
the ruling's direction — under a turn, where the normal lies in the plane of the axis and the
ruling; at a corner, as its fan's normal turns toward such a side's. Stations spread evenly in the
piece's own parameter crowd far from where the root moves fast, so a piece's stations are spread
by the characteristic's length instead (`Ruled::lengths`: each root held a quarter of the ruling
past either cap, each turn of the normal counted as the ruling's length over a half turn). The
walk then samples the root's run out to the blank's edge and the margin past it as finely as
anywhere.

Where the motion moves every point of each ruling alike — a slide, or a screw along the prism's
own extrusion — the contact is the same all along each ruling, and no ruling has one root: the
rulings in contact are whole, the silhouette. Each side and each corner's fan is read across at
256 parameters for the roots of `n·v`, each refined between two that differ in sign, and each
silhouette ruling is a piece of its own whose stations run along it (`Piece::Line`). Two of them
in the blank are two stretches (S2); the sheet is one of them carried.

A motion that keeps the plane of the tool's prisms (a turn about an axis along the extrusion, a
slide square to it) is still the planar subtraction class's (below): `admit_body` asks it the
planar rows outright, as it did before the constant twist read prisms. A cap of a prism standing
alone or placed is walked where the characteristic runs over it (below, #684 AE), and the crease
where a drafted prism running both ways from its parting plane folds (below, #684 AG); any other
prism's caps T1 asks clear of the blank over the roll (`admission::ends`). One row more is the
prism's own:

| # | Condition | Why | Refusal names |
| --- | --- | --- | --- |
| S1 | No ruling of a side or a corner is in contact all along it where its path enters the blank. | There the characteristic runs along the ruling, off the stretch the walk follows, which crosses each ruling once; a side every ruling of which is in contact is a face the motion keeps in its own plane (a side square to a turn's axis, or along a slide). A ruling's contact is nought at both its ends there, so between two of the coarse stations where both ends' contacts change sign (or are nought), the least of the two together is sought (Brent's method). Under a slide, a side in contact at every parameter read is that face. | The side or the corner, and a point of the ruling in the blank. |

`rust/examples/solid_sweeps/square_thread.sv` is the case: a rectangular bar standing across a rod,
screwed along it as a lathe cuts a square thread. The tip touches along its middle and the
corners where it meets the flanks out along their rulings, ever farther as their fans' normals
turn toward the flanks', which the screw slides along and never touches; the groove widens outward
by the turn a corner point makes to come round to the tip's plane. `tests/solid_sweeps_examples.rs`
holds it to that groove in closed form, `2π L/p ∫ r (w + p acos(r₀/r)/π) dr`, within 5e-6, and
`tests/prism_twists.rs` holds a bar of square section stood on a corner and leaning 20° across a
turn (the groove its corner's line turned, a hyperboloid of one sheet, in closed form), the same bar
drafted 2° (its corner still a line, its square distance from the axis quadratic in the height),
and the bar slid aslant across a cube's top (its lowest corner's silhouette ruling slid, a plane),
each STL agreeing with the field and each volume within 7e-6; with an upright bar turned still the
planar subtraction class's, and refused: a side square to a turn and a side along a slide (S1), and
the thread's bar too short to stand past the rod (S1: its corners reach the caps at the prism's
vertices, below). `tests/object_builder.rs` builds the thread at the page's samples.

### A prism's cap under a constant twist (issue #684 AE)

A cap is a plane, so under a screw, a turn or a slide its contact `n·v` is linear across it, and
nought along one line of it: under a turn the line where the cap meets the plane through the axis
and the cap's normal, which a screw's advance moves across the cap. Where a side meets the cap,
the rim carries every normal between theirs, a fan as a corner's does, and its line is the side's
edge there; so a rim of a plane side is walked as a prism's corner is (`Ruling::Rim`), its
stations the fan's normals turned from the side's to the cap's, its ring the rim's line and one
root a station, read from the contact at the line's two ends. Read along the side's rulings, the
side's root reaches the rim partway along the side rather than at a station's end, so the side's
stations take in those where the contact at a ruling's capped end is nought (`Ruled::crossing`,
read at 256 parameters and refined between two that differ in sign), and the walk steps there onto
the rim's fan. Off the fan's far end it steps onto the cap's contact line (`Piece::Line`, from rim
to rim: the points where the line crosses the profile's edges, in order along it, each stretch
between two whose middle lies on the tool), across the cap, and off its far end round the next
rim onto the next side. S1–S4 are asked of every node as of any; the sheet is the characteristic
carried (`brep::sweep::helical`), cut into faces where it runs onto and off each rim's fan, which
turns a right angle.

The tool is a prism standing alone or placed, undrafted, its profile one loop (`admission::Lone`,
as the generating class's caps ask), and only its plane sides' rims are walked. Its own rows:

| # | Condition | Why | Refusal names |
| --- | --- | --- | --- |
| S1 | No corner's, rim's or crease's root runs off its line within the walk's margin of the blank. | There it reaches a vertex of the prism, where three faces meet and every normal of the cone between them is the tool's: the sweep has a sharp edge along the vertex's path, which one sheet does not build. | The corner, rim or crease, and a point of the walk in the blank. |
| S1 | A side's root reaches a cap only over a rim that is a line, and the cap's contact line leaves the cap only onto one. | A round side's rim is its arc turned to a fan of its own, which is not walked. | The side or the cap, and a point of the walk in the blank. |
| S1 | Under a slide, or a screw along the extrusion, the walk runs over no cap. | A cap is in contact there all over or nowhere, and the silhouette, whole rulings, turns at a vertex of the prism to run along a rim. | The silhouette's piece, and a point of it in the blank. |

The rows are sampled, as every row is. `rust/examples/solid_sweeps/flat_end.sv` is the case: a
bar leaning 20° out from upright, its flat end facing down toward the axis, turned across
`turn.sv`'s ring sector. The end's contact line turned is a cone, the cut's floor, falling outward;
the inner side's line, crossing its rulings because the section is turned 5° in its plane (a side
facing the axis squarely would touch all along a ruling), is the wall; the rim's fan joins them.
`tests/solid_sweeps_examples.rs` holds it to the bar's shadow read apart from the core — each
height's slice of the bar, a polygon of its six half-spaces, its least and greatest distances
from the axis bounding what the turn removes at that height, summed by Simpson's rule — within
2e-5 at 0.001 mm (at 0.01 mm the fitted sheet's bias over the floor is 9e-5 of the part), its STL
agreeing with the field. `tests/prism_twists.rs` holds the bar leaning 45° screwed 8 mm a turn,
whose advance moves the end's line `k cot 45°` across it, to arithmetic that reads what the screw
keeps, `z − kθ`: a block point is removed where that lies within the span the bar's points at its
radius give, read along each line up through the bar (within 2e-5 at 0.002 mm); and refuses the
square thread's bar too short to stand past the rod (its corners reach the caps at vertices), the
bar slid aslant short enough for its caps to stand in the cube (a silhouette over a cap) and the
flat end's bar within a `common` (T1). Leaning 40° and screwed 6 mm a turn, the bar is admitted
but the screw's sheet across the rim's fan misses its normals' bars, and the export refuses it.
`tests/object_builder.rs` builds `flat_end.sv` at the page's samples.

### A drafted prism running both ways under a constant twist (issue #684 AG)

Drafted inward both ways from its parting plane, a prism's side folds there in a convex crease
across it whose fan turns from the lower half's normal to the upper's (#684 P). The crease of a
plane side is a line, the halves' edge at the plane, so it is walked as a cap's rim is
(`Ruling::Rim`, `Across::Half`): its stations the fan's normals from the half below's to the half
above's, its ring the crease's line, stepped onto partway along a half's stations where that
half's root reaches the plane (`Ruled::crossing`, as a rim is) and off its other end onto the
other half at the station where its root leaves the plane. A crease turns a few degrees, so its
stretch of the sheet is not cut out as a face of its own (`helical::SHALLOW_FAN`: a fan turning a
tenth of a radian or less stays in the faces either side, its ends still rows of the grid): cut
out, the band was 0.08 mm wide, and the split met the block's side plane in a pcurve 3e-5 off its
edge; the flanks either side are near enough alike that neither is read on the other, which is
what cutting a sharp fan out is for (#684 E). The rows are those above: a corner
of a half whose root reaches the plane does so at a vertex of the prism (S1), a round side's
crease is not walked (S1), and under a slide, or a screw along the extrusion, the silhouette turns
at the crease, which T1 refuses. Drafted outward, the crease is concave, and T1 refuses it as
before.

`rust/examples/solid_sweeps/creased.sv` is the case: a bar of oblong section leaning 20° across
the turn, its inner side facing the axis, drafted 2° both ways from the plane its section is
drawn in. Each half of the inner side touches along its own line, a little either side of the
plane through the axis, and the crease carries the contact between them, nearest the axis where
it crosses the block's middle. `tests/solid_sweeps_examples.rs` holds it to its shadow read as
`flat_end.sv`'s is, from its ten half-spaces, within 2e-5 at 0.001 mm, and `tests/prism_twists.rs`
the same bar screwed 6 mm a turn by `z − kθ` within 2e-5 at 0.001 mm (1.7e-5 at 0.01), each STL
agreeing with the field; the bar of square section stood on a corner and drafted both ways is
refused at S1, its characteristic, the corner's line, reaching the plane at the vertex where the
corner meets the crease. `tests/object_builder.rs` builds `creased.sv` at the page's samples, where
the display's fewer samples first showed the thin band's split failing.

### A rack (issue #61)

A prism's stations are planes square to its extrusion, a station a distance along it, and its
section is the profile itself (`brep::sweep::cutter`, `Kind::Extruded`): each edge a face, the
walk's left turns its convex fans. The reach samples stations along the prism; the band runs
from the first hit to the last, and may not grow past the prism's own length (`Band::limit`).
Under a motion keeping the profile's plane (a rack's: a slide seen from a turn about the plane's
normal) every station meets the same curve, so the sheet is not traced (issue #70):
`brep::sweep::extruded` builds it from the profile's planar envelope (`generate.rs`, the curve the
drawing's `envelope` is) — each edge cut as a line or arc over the rolls its cut lies on it, each
convex corner as a point between the rolls its sides reach it, read exactly by
`generate::cut_at`, interpolated by Hermite spans halved to 0.1 µm (or the fit's bar) at their
quarter points, and extruded linearly over the prism's length. The rack's tooth space is 28 spans
in 40 ms, and every contact the trace finds lies within 5e-6 mm of it
(`rack_cut::the_racks_sheet_is_its_planar_envelope_extruded`). Any other motion is traced.
A sector's flat caps join across its sides (`brep::pattern`, `Kept::Flat`): a plane square to
the axis is kept by the turn, its pcurves turned about the axis's foot, and a ring of it is an
annulus with no seam. A blank reaching its axis is built whole (neighbouring sectors' sides
would meet in it), so `rust/examples/generation/rack_cut_gear.sv` has a bore.
`tests/rack_cut.rs` is the gate: one space's volume against arithmetic (an arc of each circle per
roll, unioned over the roll) to 1e-4, the whole 20-tooth gear patterned from one sector, field
agreement, and the toleranced build. The OCCT oracle (`--kernel occt`) sections revolutions only.

### Drafted prisms (issue #684 G)

A drafted prism's section at ordinate `h` is its profile moved `|h| tan draft` into its material
(`build::drafted`). Where each corner joins two lines or goes on tangent, a point at a fixed
fraction of each edge moves along a line as `h` runs, so each side is the ruled surface between its
edge's sections at the prism's two ends: a plane where the edge is a line, a cone where it is an arc
(`ExtrudedSurface::drafted`, read through the field's `Sections`). A point of it has a normal of its
own, the same along each ruling, so its contact condition is read by the motion in closed form as
an undrafted side's is, and every row is asked of it as of any prism's side: the caps clear of the
blank (T1), M2, E1–E4. A prism running both ways from its profile's plane is two such halves,
folding where the draft parts (below, #684 P). T1 refuses, naming the prism and why its sides are
not read:

- a prism drafted outward both ways from its profile's plane: its sides meet there in a concave
  crease, which trims the two halves' envelopes against each other;
- an arc meeting a side at an angle, a corner the draft moves along the curve its cone and the
  side meet in (`Sections::linear`; `build::drafted` does not build it either);
- a side that is no line, arc or circle, or one the draft takes to nothing within the prism.

The cutter sections a drafted prism at a station as the blend of its half's two end sections
(`brep::sweep::cutter`, `Half::ends`, each walked step for step as the profile is), and walks
it by the profile's lengths, not its own: a walk length then names the same fraction of each edge
at every station, a ruling of each side, as it does along an undrafted prism whose section is the
same at every station. The sheet's rows are walk lengths read across the stations, and walked by
its own lengths a section that shrinks with depth moves where its contacts leave the roll's window,
so a station outside the blank ended the chart inside it. The sheet is traced: a drafted side's
normal leans out of the profile's plane, so a motion keeping that plane does not reduce it to the
profile's planar envelope extruded (`Cutter::prism_profile` is the undrafted prism's only).

A drafted tooth under a rack's motion keeps its plane, so before this the planar subtraction class
read it (below), as it still reads whatever the generating rows refuse; it is now built by its
sheet. `rust/examples/solid_sweeps/helical.sv` is a case no planar class reads: the rack's tooth
leaned 15° across the blank, as a helical rack's teeth are, and drafted 2°. `tests/drafted_tools.rs`
and `tests/solid_sweeps_examples.rs` are the gate: the drafted rack's narrowing space and the
helical one, each held within 2e-4 to arithmetic made apart from the core — each section of the
blank square to its axis cut by the tooth's section there, a quadrilateral read from the tooth's
four drafted planes, swept by `rack_cut`'s arc-per-roll arithmetic and summed across the blank by
Gauss–Legendre — 72.065 mm³ and 71.836 mm³ (undrafted 80.68 and 83.26), each exact solid within
7e-5 of it and its STL agreeing with the field; a drafted slot's sides lying on the tool's field
with its normal, its lines' planes and its arcs' cones; and both T1 refusals above, on the helical
tooth, where no planar class takes the sweep instead.

### A drafted prism running both ways from its plane (issue #684 P)

A prism drafted from a parting plane it runs both ways from, as a tool forged or cast from a
parting line is, is thickest at the plane: its section there is the profile, and each side's
section moves `|h| tan draft` into the material either side. So each side is two halves, each the
ruled surface between the profile and its own end's section, and they meet at the plane in a crease
across the side. Drafted inward the crease is convex, and every normal between the two halves' is
the tool's there: a fan, as a convex corner's is, but running along the parting section rather than
along a profile corner. A point of the crease is on the boundary wherever one of those normals is
square to its velocity, so the fan carries the contact from the one half's envelope, along the
crease point's path, to the other's (`ExtrudedSurface::drafted` reads the halves,
`SweepContacts::creases` pairs them). The halves are asked every row as sides are; a half's end at
the plane is no cap's rim (T1). The crease is asked as a cap's rim is (#684 F): where a half's
contact in the blank meets it, the half's envelope stepped onto the crease and the fan stepped off
it must run the same way, or E3 refuses the fold; each fan normal touches the blank once (E2); and
the fans' contacts join E4's. Under a motion keeping the profile's plane every normal of the fan is
in contact at the instant both halves are, so the fan carries the contact nowhere, and the halves'
envelopes meet in a ridge. Drafted outward, the crease is concave and trims the halves' envelopes
against each other, which T1 refuses.

The cutter's stations run along the extrusion and cross the crease's fan between the halves
(`brep::sweep::cutter`, `Drafted`): a station below the plane is its ordinate, the fan's turning is
laid as a millimetre of stations a radian (`CREASE_SCALE`), and a station above the plane is its
ordinate that much further on, its section the blend of that half's two end sections. A fan's
stations are the profile at the plane, each point's normal turned from the lower half's to the
upper's. A draft's crease turns a few degrees, and its fan carries a contact only a little further
along its point's path (a fraction of a degree of the roll on the case below), so its stretch of the
sheet is about as long in space as in stations; laid at 20 mm a radian, it left columns standing on
one curve, a fold of the fitted sheet.

`rust/examples/solid_sweeps/parted.sv` is the case: `helical.sv`'s tooth, leaned 15° across the
blank, drafted 2° both ways from a plane through the blank's middle. `tests/solid_sweeps_examples.rs`
and `tests/drafted_tools.rs` are the gate: its space, 79.70 mm³ (the tooth drafted one way from the
front cuts 71.84, the undrafted 83.26), and the unleaned rack tooth's drafted the same way, 77.40 mm³
(undrafted 80.68), each within 2e-4 of arithmetic made apart from the core — each section of the
blank square to its axis is what of a square the tooth's eight half-planes keep, each side's
material below the plane and above it, swept by `rack_cut`'s arithmetic, over pieces of the blank
split where the crease passes a corner of the profile — each STL agreeing with the field, and built
in the page (`tests/object_builder.rs`). Drafted outward, the tooth is refused at T1.

### A prism's cap in the blank (issue #684 F)

A prism standing alone as the tool — the swept solid the prism itself, or the prism placed (`p
at(under: m) τ`, `p in f`, #684 AC), undrafted, its profile one loop of lines and arcs
(`admission::Lone`) — may pass a cap through the blank. The cap is a plane, and a
plane's contact at each instant is a line on it; where a side meets it, the rim carries every
normal between the two, a fan as a revolution's convex corner does. So the caps are faces of the
tool (`ToolSurface::Cap`, read over the box the profile lies in), asked E1 and M2–E4 with the
sides where one reaches the blank, and the rims are read too: at each rim point where a face's
contact in the blank meets the rim, the face's envelope stepped onto the rim and the fan stepped
off it must run the same way (E3; where they do not, the fan sweeps back over the face's envelope,
as a flat end swung on an arm does, its corners digging past it), each fan normal touches the blank
once (E2), and the fans' contacts join E4's.
Stations along the extrusion end at the cap, so the prism is sectioned across instead
(`brep::sweep::cutter`, `Kind::Across`): a station is a distance along a direction in the
profile's plane, its section the profile's chord there carried along the extrusion — the low cap's
chord, a ruling of the side at the chord's far end, the high cap's chord and the near end's ruling,
its four corners the caps' rims, convex, each a quarter turn of fan. The walk runs from a side over
a rim and across a cap as a revolution's meridian runs from a flank over its tip. A section's side
leaps from one face's normal to the next's where it crosses a corner of the profile, and the
section closes to a ruling at the profile's ends along the direction, so one more row asks for a
band clear of both:

| # | Condition | Why | Refusal names |
| --- | --- | --- | --- |
| T3 | Of 36 directions across the profile, the best sections the band the contacts in the blank reach (their tool points, sampled) clear of the profile's corners and its ends by a quarter of the band's width and two of the reach's 96 stations either side. | A corner in the band is a crease the one fitted sheet does not build (a vertex of the prism, its path a sharp edge of the sweep); a section closing at an end has no chord. The margin is what the sheet's widening asks. | The band, the corner or end it comes nearest, and a contact at that end of the band. |

The rows are sampled, as every row is. The sheet (`sheet::capped_sheet`) is traced at the
admitted direction's sections and is the generating class's from there on: the rims' fans are
walked a millimetre a radian (`RIM_SCALE`), so the rows step through a rim, from a cap's contact
time to a side's, a radian of the motion at a time and `charted` keeps them; and each chord is
walked as long as the band's middle one, so the stations of a tapering profile start each piece at
the same walk length and the rows lie on the same pieces in every column. Only the core's kernel
builds it; the native path refuses it by name.

`rust/examples/solid_sweeps/bar_end.sv` is the case: a bar pointing at a disc's centre, rolled
as a rack's tooth on a circle of 16 mm, its flat end 18 mm from the centre. Seen from the disc the
end keeps that distance, so its envelope is the floor of the space (the 18 mm cylinder) and its
rims carry the walls; its sides face along the slide and touch nothing (M2's nonzero constant).
Its profile tapers through the disc, so the space is no prism of one section. Sectioned across the
disc's height (T3's choice: the profile's corners are all above or below the disc), it is built
within 1e-6 of the part's volume at 0.01 mm, and of the space within 2e-6 at 0.002 mm, against
each height's space read by `rack_cut`'s arithmetic apart from the core and integrated over the
height, its STL agreeing with the field (`tests/solid_sweeps_examples.rs`, `tests/prism_caps.rs`,
with the page's stepped build in `tests/object_builder.rs`). The same bar swung on an arm is
refused at E3, a corner on its profile's side within the disc at T3, and the bar within a `common`
at T1.

A placed prism (#684 AC) is read through its placements: its sides and caps are the prism's,
carried by the placements' pose (`SweepContacts::read`), and its cutter is the prism's with its
profile moved by them (`Cutter::prism` follows a recipe's `placed` nodes down to its prism), so the
sections across, T3's spread and the sheet are read where the placed prism stands.
`rust/examples/solid_sweeps/next_tooth.sv` is the case: `bar_end.sv`'s bar placed a quarter turn
of the roll along the rack, as a rack's further teeth are, and rolled a quarter turn earlier. It
cuts the bar's space turned a quarter turn round the disc, built within 1e-6 of the same volume,
its STL agreeing with the field, which reads the placed tool apart from its sections
(`tests/solid_sweeps_examples.rs`, with the page's stepped build in `tests/object_builder.rs`).

Still refused: a corner of the profile within the band, a vertex of the prism whose path crosses
the blank (T3); and a cap of a prism within an intersection, a drafted prism or a profile with
holes (T1). A cap under a screw, a turn or a slide is the constant twist's, which walks it (above,
#684 AE).

### Pocketed prisms under a planar motion: the planar generating class

A Wankel rotor is the other half of envelope theory (#65): not the union of a tool over a motion
but the largest body inside a moving pocket at every pose, an intersection over the motion. By De
Morgan it is its blank less the pocket's *wall* swept over the motion, which is a cut, so the body
rule says it already (`swept cut rotor`, `rust/examples/wankel/`). Over a whole period the cut
leaves the blank within the pocket's **inner envelope** (`envelope::planar`), carried along the
prisms. A sweep whose tool is a prism cut by a pocket (a housing; a prism alone is a rack, asked
the rows above) is asked these rows (`solid::planar_class`) in place of T1–E4:

| # | Condition | Why | Refusal names |
| --- | --- | --- | --- |
| P1 | The motion turns the pocket's plane in itself about axes square to it, the same face up. | Then every pose is a rigid motion of the plane and the sweep is planar. | The motion. |
| P2 | The tool is a stock cut by one pocket, both prisms standing on the pocket's plane and spanning the blank's height along its normal (the blank boxed by its own B-rep); the pocket's profile is one closed curve. | The section is the same at every height through the blank, so the body is a prism of it. | The solid that is not. |
| P3 | The roll is a whole period: the last pose is the first. | A closed envelope with no limit poses to cap. | The roll. |
| P4 | The pocket's inner envelope is one regular loop: every kept contact found and linked, its pieces meeting at corners, with no fold or crossing. | The rotor's flanks are its pieces, its apexes their corners. | The point where it fails. |
| P5 | The stock's outer wall never reaches the blank: every point of the blank's mid-height section, on a grid, lies within the stock at every pose of a grid of the roll. | Then the pocket alone decides what is kept. | The point the wall reaches. |

The envelope is read by sampling the pocket's profile — a curve of the drawing, the Wankel's bore
the apex's own planar envelope (`envelope(apex, under: rotor_turn, …)`, read through
`generate::kernel_eval`): at each sample every root in the roll of the
contact condition `∂X/∂s × ∂X/∂θ = 0` (brackets on a grid of the roll, then regula falsi), each
image kept where it lies inside the curve at every pose (the curve's signed distance read on a grid
and bounded between by the image's speed, to a thousandth of a radian), images touched at three
samples or more corners, the rest linked by continuity and marched onto their corners. The body
(`brep::sweep::planar`) is the blank kept in common with the envelope's prism, a cubic B-spline a
piece fitted within `FIT_MM` through contacts solved exactly; nothing is traced, split or judged
cell by cell. The accuracy meter reads it as the envelope's region times the blank's slab
(`accuracy::Planar`). Below the limiting K = R/e = 3 the bore crosses itself and the pocket is no
face, refused before any row is read.

### Prisms cut under a planar motion: the planar subtraction class

A conjugate wheel (#590) is what a tool would cut if it were a gear shaper: `space :=
sweep(star over t in (0deg, 540deg), under: handle_turn, seen_from: wheel_turn)`, `space cut
wheel`. Its tool is always in mesh (E1), its arms' straight sides touch twice in a passage (E2),
and a stepped handle is no single revolution or prism (T1). Each of those rows exists so
that no sheet needs trimming; under a motion that keeps the tool's plane the trimming is cheap and
exact in the plane. Within a slab of the blank that every member spans or misses, the section is
the same at every height (or, drafted, moves as its draft does), and so is what the sweep removes:
`R = ∪ₜ poseₜ(section)`, the slab's cut the blank less `R`'s prism. A prism sweep the generating
rows refuse at T1, T2, E1 or E2 is asked these rows instead (`solid::planar_subtraction`):

| # | Condition | Why | Refusal names |
| --- | --- | --- | --- |
| P1 | The motion turns the tools' plane in itself about axes square to it, the same face up. | Every pose is a rigid motion of the plane. | The motion. |
| Q1 | The tool is prisms square to that plane, their profiles lines and arcs, joined by `union` and `common`; a drafted one is read at stations through its slab (#607). | Each piece's contact is exact. | The member that is not. |
| Q2 | Every member's caps lie outside the blank, or on a plane where the blank is split into slabs; a drafted slab's region keeps its shape through the slab. | Within a slab the section is one (drafted, one shape lofted); the cap planes inside the blank are the slab planes, so a stepped tool's bands split the blank themselves. | The blank whose extent is unknown; the depth of a drafted member where its region changes, to step the tool at. |
| Q3 | The roll is a whole period of the motion (the tool stands where it began, its own symmetry counted), or the tool is clear of the blank at both ends. | A whole period closes `R` and leaves no caps (#585). | The roll, the pose in the blank and the period to roll. |
| Q4 | The kept boundary is regular: its pieces meet at corners or leave the blank's shadow, with no fold and no crossing. | What the region keeps must be loops the builder can fit. | The end that meets nothing, the fold or the crossing. |

`R`'s boundary (`envelope::swept_region`) is read from every candidate a piece of the section can
make — a line's contact (the foot of the perpendicular from the instant centre), each side of an
arc's, a corner's path while the line from the instant centre lies in its fan of normals, and, over
a roll that is no period, every edge held at each end pose — each point valid only on the section's
boundary at its own pose and **kept only where it lies outside the section at every pose**: the
meaning of the cut, which drops a second contact, an envelope's other branch and every trim at a
concave corner where members meet. Kept stretches are linked with `R` on their left through
crossings (solved exactly), junctions at one roll, end poses and the sides of a box about the
blank's shadow, so only loops meeting the blank are built. Where several stretches meet at one
point — at an end pose's corner, the held edge, the corner's path and a hair of the next edge
that the keeping test's tolerance keeps — the boundary of the union takes the rightmost turn, and a
hair joined at neither end is dropped as bounding nothing (#613: a box reaching past the rack
tooth's tip, as a placement turned 30° makes it, meets one). Each slab is the blank less its loops'
prism (`brep::sweep::subtraction`, one Boolean a placement); a drafted slab is lofted through
stations, each stretch a B-spline surface. Nothing is traced, split or judged cell by cell, and
field agreement gates the export as for every sweep. The #585 star wheel is 24 candidates and 33
kept stretches, read in half a second; its volume agrees with arithmetic made apart from the core
to 1e-4, with the field's at every probe 0.02 mm off the surface, and lies below the long form's
(#589) by its scallop; drafted 8° it is lofted through nine stations and exports in seconds
(`tests/planar_subtraction.rs`). A drafted slab's stations are read within a box grown until they
are alike, since a change only where the box's sides cut the region is none of the blank's; the
radiator valve's measured handle is one (#612).

### A union tool: every class, member by member (issue #684 C)

A sweep distributes over union: under one motion over one roll, `sweep(A ∪ B) = sweep(A) ∪
sweep(B)`, so `blank − sweep(A ∪ B)` is `blank − sweep(A) − sweep(B)`. A body cutting the sweep
of a union is admitted and built from a copy of the drawing with that cut written as one cut a
member (`solid::distributed::lowered`): each member's sweep under the union's motion, over the
union's roll (a `through:` roll is the union's, found once), placed wherever the union's sweep
is, and its faces named by their routes in the union (`removal.tool.ring`). Each member's sweep
is then whatever it is alone, and is asked that class's rows: a ball's under any motion is its tube
(`canal`), a revolution's under a screw, a turn or a slide the constant twist's, under a relative
rotation the generating class's. No row is added. What the union's crease would fail as one tool
— a concave corner where the members meet, carried through the blank (T2), or two sheets
crossing (E4) — is no corner and no crossing of either member's sweep, and the body's split and
its cells, judged by the field of the union's sweep, trim one sheet by the other as they trim
any two placements'. A member refused is refused by its own row, saying so: `` `groove` is
outside the constant-twist class (S1: …): its tool's member `bead2`: … ``.

A union is its members where it is the whole tool, a union within a union, or a placement of
one, each member placed as it is. Still refused: a union within an intersection or a cut
(`(A ∪ B) ∩ C`, T1, which is not distributed yet); and a body that cuts a ball's tube beside a
sweep of another class, which the static remainder does not read as either form, written as a
union or as two cuts. A union of prisms under a motion keeping their plane is not distributed:
the planar subtraction class reads its swept region whole (below). Only the core's kernel builds
a distributed cut; the native path refuses it by name. `tests/union_tools.rs` is the gate: two
overlapping tori on one axis slid up a rod (`rust/examples/solid_sweeps/union.sv`), the groove
what of the rod's disc either tube's shadow covers, computed in rows across the rod apart from
the core and held to the exact solid (2e-8 off a chord-by-chord integral), whose STL agrees with
the field; that union placed half a turn round the rod and turned about its own axis; the two
cuts written one after another, the same solid to 1e-9; two balls swung on one arm, two tubes
(Pappus); two tori rolled through the post, each admitted to the generating class and built as
the long form builds them; and a member refused by its own row, and a union within an
intersection refused at T1.

### A ball along any path: the canal class (issue #684 D)

A ball looks the same however it is turned, so its sweep depends on its centre's path alone:
everything it passes through is the points within its radius of that path, the tube about it,
with the ball where the path starts and where it ends. No contact is solved and no clearance is
asked (its end poses are the construction's own), so the tube may stop inside what it cuts and a
sweep may stand alone; it is built for any motion (`solid::canal`), and the body it cuts is
static material, so no other class is asked.

- **In closed form** where the motion is one turn, screw or slide (`Family::twist`): the disc
  square to the path revolved (a torus's stretch), carried along the helix (a screw's `Blend`) or
  along the line (a cylinder, built exactly; a slide's ends in the blank were refused at E1).
- **Fitted** under any other motion — a roll, a frame along a path (#398 C), one the drawing
  moves, a joint driven by a law: the centre's path read off the motion, its disc the one round
  wall of a face family (`brep::family`) whose stations are the path's points and the frame square
  to it, carried from the start by double reflection, its seam turned off the path's own normal
  as a turn's is; between the stations, degree-five polynomials held within a part in 10¹⁰ of the
  disc's diameter of the disc read between their nodes. Where the numbers lie on a cylinder or a
  torus (a frame along an arc carries the centre round its circle), the wall lies on it.

Each is held to the sweep's own field before it stands (the canal's gate). Two rows refuse the
tube that would meet itself, read along a path no closed form carries at points an eighth of the
radius apart along it (at least 1024, at most 32768), so **sampled**, and said so:

| # | Condition | Why | Refusal names |
| --- | --- | --- | --- |
| B1 | The centre's path never stops and turns nowhere tighter than the ball's radius. | Tighter, the discs square to the path cross inside the turn and the tube folds; where the centre stops, its path has no tangent and no disc. | The roll and the radius, and the point of the path. |
| B2 | The centre's path comes back within the ball's diameter of itself nowhere it has run half a turn of a circle of the radius from (read with the points' spacing to spare). | Two discs that far apart along a path that turns no tighter than the radius meet only where the path comes back: the tube crosses itself. | The two rolls and their distance, and the point between them. |

Under a turn or a screw the tube is refused where its centre stands within the radius of the axis
or a helix of more than a turn climbs no more than the diameter a turn; there, as before, the
constant-twist class is asked instead. The native path refuses a ball's tube by name.
`tests/ball_paths.rs` and `tests/solid_sweeps_examples.rs` are the gate: a ball slid with its ends
in a block (half a capsule, its faces a cylinder and two balls), along a spline in space (its
torsion), rolled along a trochoid (once refused at M2, a pole in contact at every time), along an
arc (a torus), and riding a frame along a spline in a block's top face
(`rust/examples/solid_sweeps/path.sv`), each volume π r² L + 4/3 π r³ with L integrated apart from
the core and held to 1e-9, the body's STL agreeing with the field; and B1 and B2, each refused by
name with a point of the path.

### Undercut is excluded deliberately

A pinion with few teeth often has some undercut. There, the envelope of the blade tip's
round trims the flank envelope. A real gear has this, and E3/E4 refuse it. The first
version refuses undercut and requires the design to avoid it. Supporting it means one
*declared* intersection between two known envelopes of the same sweep. `BoundarySeam` and
`EnvelopeSeam` already perform that operation. Supporting it is a later, separately gated
extension. An undeclared crossing stays refused.

The classes are implemented as `solid::admission::admit_body`. `solventc --step/--stl` runs it
before building a body with swept cuts. Placements whose blank reads the same at every point
the checks read are checked once: the gear's 24 indexed cuts are one check.

## Refused, and how

Everything outside the class is refused at elaboration or export, never meshed on a
best-effort basis. The refusal names the condition row and the witness. The tumbling and tilted
cylinders, the thin plate, the dumbbell and every prism sweep but a rack's with its caps clear, a
prism's standing alone or placed whose cap's sections pass T3, a prism's under a screw, a turn or
a slide with its caps clear (#684 M) or crossed over their rims (AE), a pocketed prism's under a planar motion and a prism tool's
under a motion that keeps its plane are outside the classes; a slid cylinder is in the constant-twist class where its flat caps, which the
slide keeps in their own planes, stay clear of the blank. Their fixtures stay
in the tree as **negative controls**: each must
produce its refusal.

## Acceptance

An admitted sweep is accepted only by evidence that caught the earlier failure. The
traced-sheet mesh arrangement passed its recorded volume and closed-shell gates while
19–37% of the hypoid pinion's triangles disagreed with the material field. So:

- **Field agreement is the gate.** Probe points 0.1 mm inside and outside every face or
  triangle against `MaterialEvaluator` (`a_hypoid_pinion_space_agrees_with_its_field`), with
  zero disagreement allowed.
- Recorded tooth-space volumes (`native_surfaces/gear_cells.rs`) and encoded shell/STEP
  round-trip checks remain necessary but are not sufficient.
- The admission predicates are **sampled** in the first version and must say so.
  Certifying them over whole regions uses the interval machinery
  (`docs/interval-geometry.md`), which already covers the circular crown. That
  certification is scheduled, not assumed.

## What was removed

These tracks were closed and deleted on 2026-09-25; git history keeps them.

- `solid/swept_boundary`, the general certified sweep closure, and its Phase 0–3 records
  (`docs/swept-boundary*.md` and `docs/fixtures/`).
- `gcs-cli/src/cad/mesh_sweep.rs`, the traced-sheet Manifold arrangement, with
  `solid::sweep_candidates` (`--stl-backend manifold`). It failed the field gate on 18% of a
  hypoid space.
- The Ju et al. reference experiment (`experiments/ju-sweep-reference`).
- The CGAL Mesh_3 backend (`--stl-backend cgal`), superseded by the core's own refinement
  (`docs/field-meshing.md`).

## Open decisions

1. **Backend.** The native OCCT split-and-classify path (`cad/native/sweep_boundary.rs`)
   already exports the common-apex bevel pair from source alone. It agrees with the field on
   both sides of every sheet node and reproduces the recorded volumes. It is native-only and
   took 6–11 minutes per member (12–20 s since docs/native-speed-plan.md). The one-core WASM target needs a kernel-free path
   eventually. The plan builds on the OCCT path first and keeps the admission predicates
   in the core, so a later backend reuses them.
2. **Supported offsets.** Whether the hypoid offsets the design needs satisfy E2–E4 is
   an unmeasured question. The investigation found crossing envelope branches at 15° to 45°,
   but in the mesh path, which also had construction defects. Phase 1 of the plan measures
   it. If the needed offsets fall outside, widening the class (declared branch trimming) is
   a decision for the user, not a default.
3. **Undercut.** It is refused for now, as described above.
