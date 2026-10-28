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
| T1 | The tool is a full revolution with a line/arc/circle profile — a region solid among them, its profile its meridian (`solid::region::meridian`, its faces `RevolvedSurface::meridian_loops`, #209) — a prism of lines and arcs whose caps stay clear of the blank over the roll, or an intersection (`common`) of such, under fixed placements; or a union of such tools, each member swept alone (below). | Every face has closed-form contacts. An intersection creates only convex creases. A prism cuts with its sides only (`ExtrudedSurface`), so a cap in the blank would be a face the class does not build. | The first operand outside the class, a cut in the tool or a union within an intersection, or the prism whose cap passes through the blank. |
| T2 | Every profile vertex inside the swept region is convex or tangent. | A convex corner sweeps a fan of normals. A concave corner sweeps nothing, but it trims two envelopes against each other, which is not supported. | The vertex and its turning angle. |
| M1 | The motion is one rotation about a fixed axis, or one rotation (or one translation) relative to another rotation (`motion(about: …)`, `motion(along: …)`, `motion(m, relative_to: n)`), at constant ratios, with a finite increasing roll. | This is the generating motion of a face-milled bevel or hypoid, including skew axes, and of a rack cutting a spur gear. | The motion node outside the class. |
| M2 | Each tool point's contact condition changes over the roll. | Where it does not, the point is on the boundary at every time or at none, and a contact time cannot parameterize the surface. A single rotation is like that everywhere (its tool-frame twist is constant), so the generator's motion is a relative one. This is the native path's existing refusal. It is checked at the samples, at a pole (whose normal is the axis), and between samples: the equation is C + a cos + b sin in the roll, and where (a, b) turns right round between two samples, or winds round a sample cell, with roots on every side, a point there has C and the amplitude both zero. That happens where the tool's normal passes through the point where two meeting axes cross. | A tool point whose path reaches the blank. |
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
| S1 | The characteristic is regular through the reach and a margin past it: no ring is about to lose its two roots (a double point, where the curve turns back on the ring), none is stationary, and the walk never jumps at a profile corner. | The sheet is one regular curve carried along. A convex corner's fan is not built for a screw yet. | The face and the ring. |
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

Two refusals are the limits' own:

| # | Condition | Why | Refusal names |
| --- | --- | --- | --- |
| S1 | No ring the walk passes between two of its stations is in contact all round where its path enters the blank. | Such a ring is all characteristic, and the boundary leaves the walked branch along it. A turn about an axis the tool's own axis meets has one, every normal of it through the point where the axes meet; a slide along the tool's axis has one wherever the normals are square to it. The roots either side need not move (the ring's two roots stay in the plane of the two axes), so a sampled station never sees it: the contact equation's amplitude turns right round instead, and where it turns more than a quarter turn between two walked stations, the least of the ring's three coefficients between them is sought (Brent's method). The field gate found the first one, on a torus leaning toward the turn's axis, before this row did. | The face, and a point of the ring in the blank. |
| S4 | A turn's roll is less than a whole turn. | A turn carried round a whole turn or more lays its sheet over itself; its sweep is then a solid of revolution, which is not built yet. | The turn, in degrees. |

E1 holds as for a screw, so a slide passes the tool right through the blank and a turn swings it
across from clear to clear. Still refused: a flat face the motion keeps in its own plane (a cap
square to the turn, a face along the slide) where it reaches the blank, a ring in contact all
round at a sampled station (S1); a prism tool, whose side has no rings (T1); a characteristic that
closes on itself within the blank's reach, a tool passed wholly through it (S1); and a whole turn
(S4). A turn of one ball is still its tube (`solid::canal`), never asked these rows.
`tests/turns_and_slides.rs` and `tests/solid_sweeps_examples.rs` are the gate: a torus slid up a
rod, square (its tube's lens, `rust/examples/solid_sweeps/slide.sv`) and tilted, a cylinder slid
with its caps clear (a circular segment), and a torus turned across a ring sector, upright
(Pappus) and leaning across the turn (`solid_sweeps/turn.sv`), each removed volume computed apart
from the core — a tilted torus's from its centre circle alone — and held to the exact solid, whose
STL agrees with the field; and the straight-fluted drill
(`rust/examples/twist_drill/straight_flute.sv`, the twist drill's wheel slid square to the axis,
its flute the wheel's own profile).

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
and a stepped or drafted handle is no single revolution or prism (T1). Each of those rows exists so
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
is then whatever it is alone, and is asked that class's rows: a ball's under a turn is its tube
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
pocketed prism's under a planar motion and a prism tool's under a motion that keeps its plane are
outside the classes; a slid cylinder is in the constant-twist class where its flat caps, which the
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
