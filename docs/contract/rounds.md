# Rounds, chamfers and draft

Part of the [contract](../../CLAUDE.md) the code is held to. Read it when you touch
`solid/lowering.rs`, `brep/fillet.rs`, `brep/corners.rs`, or `std.rounds.Rounded`. An issue number
(#n) or a spec version ([0.n], see the [language changelog](../solvent-changelog.md)) says where a
rule came from.

**Rounds and chamfers are words of `std.rounds`** (#183 steps 4–7). `repeat a in X` over a solid (a
face path) runs over its faces (F4): set aside, and where the faces are not given the elaborator
names them once the solids are built (`solid::faces::face_paths`, structural: each primitive's
own faces under its route, `FacePath`) and elaborates again (`program::Again::Faces`,
`flatten::expand_faces`, `Walk::face_links`); a `when` in the body keeps a copy — `when f in X
and f != g` (F8; 0.91, #267: `Walk::faces_named`, `Walk::stands_for`, the `if` retired). A relation word's body may hold blocks, instances, sets and body statements (F5,
`parser/definitions.rs`); `A.body` in a body statement is the solid a face path is of. **The
stratum** (F6): a body's terms that read its faces (`faces::readers_of`: a region over them, or a
body standing on one, [0.198], [sets.md](sets.md)'s shell) are left out of the
boundary they read and applied last by the field (`document.rs`: fills, then cuts). A face path
is matched against `solid::exact`'s names (`BodyFaces` is built from `exact::build`, shifted to its
origin; cached in `Caches::faces`); one naming nothing holds nowhere, and a region over it is
`RegionField::empty`, no term of a body. A block over a solid's faces whose path names none the
solid lists (`FacePath::matches`, eaten or not) is **W116** at the document's statement it was met
under (`Pending::at`: the round, not `std.rounds`' block), once per statement (`Walk::once`),
naming the steps the faces go on by past the longest beginning of it that names any
(`flatten::resolve::names_no_face`, asked by `Walk::face_links`, [0.203], #760): `part.pad` where
the pads are `part.joint[0].pad`, … says `joint[0]`, `joint[1]`. `std.rounds`: `Run`, `Fill`,
`Corner`, `CornerFill`,
`Flat`, `FlatFill` (sets over faces) and the words `A round(r: …) B`, `round(r: …) X`, `A chamfer(d:
…) B`, `chamfer(d: …) X`; in a region's body `std.rounds.Rounded(a, b, r)` and
`std.rounds.Chamfered(a, b, la, lb)` over two of its own sets (`set` formals, #191; `p inside S(…)`
may be written in place). **Each edge and corner once** (#415, [0.148]): the six sets are
`symmetric` in their faces (`Flat`'s each with its leg, `symmetric((a, la), (b, lb))`), so the words
— which reach an edge as `(a, b)` and `(b, a)` and a corner in six orders — make one set, one solid
and one cut or fill of each (`flatten::same`, [language.md](language.md)): `rounds/bores.sv` is 18
regions, not 52 (240, not 1152, before the words paired only faces that meet).
`tests/symmetric.rs` holds the count and the block rounded and chamfered all over to the words
written without `symmetric`.
**Only faces that meet** (#625, [0.187]): the words pair `a` and `b` only where `b meets a`, and
round a corner only of three that each meet the other two ([language.md](language.md)), so a body
whose faces are many — every operand's listed, eaten or not — makes copies for its edges and
corners, not for the cube of its faces: #625's web cut by four turned spokes lists 28 faces and
rounds 16 edges and no corner (`tests/meets.rs`).
**Which edges are the faces'** (#763, [0.204]): a word takes every edge where its faces meet, and
a block over the faces picks among them — by name, or by how each face lies (`when f parallel l`,
[language.md](language.md)) — so the words take no option saying which edges: `rounds/hub.sv`
fills a bar's sides into a post (`f parallel std.z and g parallel std.z`) and leaves the arc where
the post rises from the bar's top sharp (`tests/faces_along.rs`, the closed form's volume to 1e-9).
A set's `S.surface` is the set with each bound made an equality (`Written::equality`,
`sets::surface_of`; a grown side's is the offset surface); a distance from a one-bound region's
surface reads it (`sets::one_bound`); a set measured from is applied under `PredWalk::held`, so each
equality its body states is a bound at its number, `in: (d, d)` (`Written::held`, in `apply`),
and `region_term` reads every probe by one rule. **Lowering** (step 7, `solid::region::meridian::meridian_corner`): a `Rounded`/`Chamfered`
term whose sets are two of the region's own bounds (same measure, at its level, moved in by the
radius for a round's centres) is built in the meridian by X1's construction (`brep::corners`,
moved from `tests/fillet_meridian.rs`) — a round at any corner of lines and arcs, an offset
cone's generator and rounded apex included (#207) — and held to the region read point by point
beside each face (`region::meridian::agrees`; a disagreement is read by the field); refused (and read by the
field) at a charted corner (none a meridian folds) or a flat between two arcs. **A chamfer's flat
is its sum of distances** (#196, #208, [0.73]): between a line and an arc it bows, the conic `|q −
c| = α + β d_L` (`corners::sum`), a **`Seg::Conic`** — the rational quadratic Bézier over its ends
and where their tangents meet — carried by every meridian reader: planar Booleans (`meets_steps`'
roots, `contains` in closed form), the field's `Edge::Conic` (`Wall::conic`: pieces'
control-triangle discs, split while wide), `Meridian::Conic`, and the recipe's rational B-spline
(`region_profile`, read back by `meridian_region`). `std.rounds.Chamfered` is multiplied through by
its legs, so a leg of nothing cuts nothing (a bound weighing every measurement by nothing holds
everywhere, else E040, `measured_term`). `tests/rounds.rs` is the gate (X2's closed forms in the
slow tier), `tests/offset_corners.rs` (Pappus, `rounds/blank.sv`) the offset cone's.

**Rounds that know of one another** ([0.149], X0's findings 2 and 5): a named use's faces are
under its label ([language.md](language.md)), and **the stratum stands in layers**
(`solid::faces::layer`, run by `program::solids` after the body rule): a body's readers grouped
by the use that made them (`faces::path::use_key`, the name up to its first `#` segment), each use
one layer above the deepest use whose label one of its face references reaches
(`path::names_use`: a path past the body reaching into the label, never the whole body's empty
path nor a route through another operand), cycles E041 where the first is made. Layers below the
top are bodies of their own, **of the body's name**, each the stock of the one above (a stock of
its body's name adds no step to a route, as a swept solid's split-off sweep), their readers'
references moved to them (`RegionTerm::remapped`); so each layer reads the body as its other terms
and every layer beneath make it, and the field, the recipe and the native host apply it as late
terms of its own body — nothing new below the elaborator. No body whose readers name nothing of
each other changes. **The faces a block runs over** list a face a reader of the body (or of a
layer beneath) made with that use (`FacePath::of`), listed only to a block whose path names the
use's label and that stands outside it (`FacePath::listed_by`), so `round(r) block` never rounds its
own faces; a region's bounds no set names (`p inside a`) are none of its faces there. A block over
a solid's faces is recorded however it was named (`Walk::expand_pending`), and the elaborator names
the faces again while a pass's solids give them otherwise (`elaborate`, at most `FACE_PASSES`
times): a round over another's faces runs once that one is made, a round over an operand's
rounds' faces included. `rounds/composed.sv`, `tests/composed_rounds.rs` are the gate (the
chamfer's two edges rounded, the closed form's volume and the hand's intermediate's to 1e-9).

**A round over a round** (#879, [0.210]). **A face a round made is listed by the name the body's
boundary has for it**: a region the table recognises leaves a `round` or a `flat`
(`lowering::face_name`, `Section::face`: the `Stroke`'s name and the kernel's for a rolled one and
a corner's), so
`faces::face_paths` lists `vert.taken.round` — never the name of the set in the word's body
(`run`), which `solid::exact` names no face by: listed so, a round over a chamfer's face was made
(`flat` both ways, by chance) and one over a round's paired the top with a face the body has not,
and made nothing, silently. A region no entry recognises keeps its terms' names. **A name once**,
however many solids of one use leave a face of it (a round's every edge): a block runs over the
name, and a reference to it reads the faces as one in pieces (`run`'s piece per edge). **Which
faces meet, and how each lies, is read in the layer asked in** (`faces::Layers`, `faces::Layered`):
each layer's stratum read once (`ListedFaces::read_above`: a face of a use beneath the layer that
its stratum has not is no face there, where one of the layer's own cannot be said of), and a
`when` reads the layer of the use it stands in (`use_key` of its scope's prefix), never
beneath the layer above each use that made a face it asks of (`Layers::layer`); `FaceRead` and
`Apart` carry the layer, so `program::at_solve` asks again where it was asked. Read off the top
alone, `ab` and `bc` met where the corner's round read them and not once a round stood over its
face: the corner's `when b meets a` came out false, the corner was unmade, the faces named again
without it, and the passes went round until `FACE_PASSES` left the second round naming nothing
(W116). The faces are named again where a round has come to stand in another layer
(`faces::Layers::same_uses`), and from nothing, with every pass, where they are read again at the pose
the drawing solves at (`renamed` begun again). **A rim over rounded corners** is a round of a
solid of the rounded body (`part := solid(box)`, `part.box.near round(r) part.box`): the corners'
faces are `part.box`'s, bare or named, so the top is paired with each side and each corner's
round — a `Run` along each straight edge and a `Ring`'s stretch round each arc, glued at the
seams (`assemble::same_section`) — and with a named round's faces alone (`box.near round(r)
box.vert`) the stretches end square at the seams, their caps `start` and `end`. Rounds that name
nothing of each other each read the body before either, as ever: a rim's run and a corner's
round then cross at the body's sharp corner, a mitre of two radii. `rounds/rim.sv`,
`tests/rim_rounds.rs` are the gate (the box rounded 5 at its upright edges, its top rim rounded 1
and its bottom chamfered 1: the closed form's volume by Pappus to 1e-9, ten planes, eight
cylinders, four tori and four cones; either cap, either word; the arcs alone; the crossing built
on either cap to one volume). **A round that is not built is said by its body and its label**:
its solid's name is the use's that made it (`#17.0.….taken`), which no document wrote, so its
recipe node carries what a failure calls it (`"said"`, `` `box`'s round or chamfer `vert.taken` ``:
the body whose late sets take it, `step_through`; `recipe::Named::called`, what every
`built_names` holds).

**Against the smooth surface** ([0.149], #422): `F.smooth` (`Member::Smooth`, `FaceRef::smooth`, read
with `surface`) is the faces named and every face going on from them tangent across a seam
(`BodyFaces::smooth`: an edge two faces bound whose outward normals are one at its middle), a side
of it or a distance from its surface the nearest patch's (`Reading::side`; `Reading::distance` of a
smooth surface). `Run`, `Fill`, `Flat` and `FlatFill` read each face's side, its ball's centres'
reach and its legs off it, the crease and the meeting off the faces themselves. **The lowering
table builds the section against the chain** (#422): it recognises a smooth reference as its face
(`recognise::same`, `side_of`: what a region rounds is its own faces' edge) and reads the chain
where the section reaches it (`lowering::seams_beside`): a seam where the face goes on tangent
into another, running beside the edge — sharing no vertex with it — that the section's footprint
on the face lands past (a flat's leg, where a ball touches it: `lowering::footprint`, Newton's
steps in the plane square to the edge at points along it). A seam beyond the footprint is none of
it (`rounds/spill.sv` with a boss of 19.5, its leg short of the seam: built as the ring alone), nor
is one the edge runs into at its end (a slot's side into its half circle: the table's own going on
into the next piece). **Nor is a seam between two pieces of one surface** (#952,
`lowering::pieces`: faces going on across seams on the same surface, `Surf::same` — a plane's
normal one and its origins in it, a cylinder's, a cone's or a torus's axis one line, a ball's
centre one point, to a billionth): a plate's top in the pieces a union of its operands leaves it,
each named by its own operand, is one plane to the section, which stands against the surface and
not the names of its faces. Its footprint is read off the pieces together (`lowering::off_pieces`),
the seams it reaches are those where they go on into another surface, and a chain's next face,
where it lands and what lies past it are read the same way (`piece::onto`, `piece::next_face`).
A lobed flange drawn as a disc, a sector and lobes unioned, its upright edges rounded and its top
rim rounded over them, had each lobe's ring refused as reaching a seam that runs only part of
its edge — the arc where the disc's piece of the top begins — and a rib's straight run as
reaching one not along it; both are the rim over one top (`tests/rim_rounds.rs`:
`a_round_over_a_top_in_pieces_stands_against_one_plane`, to the closed form, and
`a_rim_over_a_plate_of_pieces_is_the_rim_over_one_top`, to the union taken the other way round,
whose piece of the top is reached nowhere past, to 1e-9). The section against a chain is the one
against whichever face of it is nearest: a round's ball touches the face its offset meets the
other face's in, and changes face where it touches the seam; a flat is each face's own sum of
distances with the other face, and changes face where it crosses the seam's normal surface —
there a point's distance from either face is its distance from the seam, so the two are one set:
- **along a straight edge beside a seam along it, or round a circle beside a seam about its axis**
  (`lowering::chain`, `piece::seam_along`, `seam_round`): in the section the corner of the other
  face's trace and a chain of the face's own trace to the seam and each next face's from its seam
  (lines and circles, up to four past the edge's own, `piece::CHAIN`, each found where the last
  one's footprint lands past another seam, `piece::onto`); the ball against the piece whose touch
  lies on it, the flat a line between lines else a conic's arc over each piece in turn
  (`corners::sum_between`; two circles with unequal legs a Cartesian oval, refused), changing
  piece on the chain's normal at each seam it crosses. Swept or turned like any section: the strips
  on the chain's side split at the body's seams, the flat in pieces, the one over the face's own
  surface what the table built before (`rounds/ledge.sv`, a rib beside a block's rounded rim; a
  boss centred on a rounded disc; a rib on a top going on through a round into a slope).
- **round a circle beside a seam not about its axis** (a boss near a plate's rounded rim,
  `rounds/spill.sv`; `piece::beyond`: the face a plane going on into a torus about its normal): a
  flat's ring kept within the seam's normal surface — the cylinder through the seam along the
  plane's normal (its `Kept`) — and beyond it the flat against the torus, sectioned in the
  half-planes through the ring's axis — the planes square to its circle, the face's meeting with
  the wall — between where the leg on the plane crosses the seam, fitted
  (`brep::spill::flat_beyond`): a lens of the torus's strip, the fitted flat and the cap on the
  normal cylinder, glued to the ring there. A round's ring a stretch between where its ball's touch
  crosses the seam, and between them the ball rolled against the torus — its centre where the
  wall's offset meets the torus's, its canal fitted — a piece of the canal, the plane's strip to
  the seam, the torus's lens, the wall's strip and a cap at each end in the plane of the ball's
  arc, glued to the stretch at both (`brep::spill::round_beyond`).
- **along an edge no line or coaxial circle carries** (a boss up through a slope near its rounded
  edge; `piece::traced_beyond`: the face a plane going on into a cylinder along a straight seam or a
  torus about its normal): a flat cut along the run (`fillet::flat_along`) kept short of the seam's
  normal surface — the plane through a straight seam along the face's normal (`Rolled::kept`) — and
  the flat beyond it sectioned square to the run between where the leg crosses the seam, found by
  bisecting the footprint along the run; a round's ball rolled along the run
  (`fillet::roll_across`, its band let cross the tangent seam) with the stretch between the planes
  of its arc where it touches the seam cut away (`Rolled::wedge`), and the ball rolled against the
  next face there, glued at both.

`brep::spill` has **one section model** (#425): every piece beyond is read along the meeting of
the face it stands on and the wall, in the planes square to it — a ring's circle a meeting like
any other, whose square planes are the half-planes through its axis. Its edges are the meetings
`ssi::intersect` gives in closed form wherever it gives one through their ends (a ring's crease
its circle, a slope's an ellipse; a round cap's edge down the wall a line where its plane holds a
cylinder's or a cone's axis, an ellipse where it cuts a cylinder aslant, a circle on a sphere),
fitted only where the meeting must be traced; the seam is the circle or the line it is.
A `Beyond` says by its type which it is (#426, in both layers, `spill::Across`): a chamfer's
`Flat` — its legs and the seam's normal surface (`split`) it starts at, its signed distance
negative on the edge's side — or a round's `Round` — its radius, nothing else; `size()` reads it.
The ring off the seam's axis and the traced run classify the chain once between them
(`piece::chain_seam`: the plane's normal, the face beyond, the seam and its normal surface) and
build their piece with one constructor (`piece::beyond_of`), the legs on the chain from the wall
first whichever face of the edge's the chain's is.
`spill.sv`'s STEP is a cone, one fitted flat, and the plate's and the boss's faces. `assemble`
glues a piece beyond to the ring whose own axis puts its section where the ball touches the seam
(a round) or that is kept short of its `split` (a flat), else to a rolled run, until #423 pairs
them where they are made.

The recipe's `"beyond"` (after the rolled runs in a lowered node's joins): the edge it runs
`"along"` (the `"face"` it stands on, meeting the wall in that edge), `"toward"`, the
`"chain"`'s face and the `"wall"` with their `"sides"`, the `"seam"` (a circle or a line), a flat's
`"legs"` and `"split"` surface or a round's `"radius"` alone (which of the two is read by the key),
and the `"ends"`; a rolled run's `"kept"` and `"wedge"`; all
written through the node's `Writer`, so `cad::recipe_about` carries them. **A piece beyond is held
by the piece it is made with** (#423: `Ring::beyond`, `Rolled::beyond`), never a `Piece` of its
own: `assemble` keeps it with its partner, drops it with its partner where another region names the
same edge (two regions naming one edge in either order still make one pair, as `canonical` needs),
and copies that join — no rule of geometry pairs them, so two runs kept short of one seam's normal
surface are each glued to their own. `boolean::glued` glues on
a curved face too, an edge matched by a point of it on the other's (a traced meeting and a fitted
one). It is symmetric in its two pieces (#424): the shared faces' boundaries are paired run by run
between the vertices both have, each side's edges lying along the other's, and each side cut where
the other has a vertex within a run, so either may hold an edge in pieces (a traced flat's or a
round's crossing cut where its closed net is seamed) and the pieces are glued in either order
(`solids_sharing_a_face_cut_differently_are_glued_either_way`). So `traced_beyond` starts its piece
at the run's first edge's middle, as a run with no seam beyond it does, and nothing steers where
its seam falls. **Refused by name** (`piece::spilled`: "its section reaches across the seam where a face
goes on tangent into another …", the field reading the region): on both sides of its edge; into
more than one face; along a seam that is not along a straight edge, or a curved seam beside one;
beside two seams; beside a seam running only part of its edge; past a face it goes on into across
an edge no seam, or a chain of more than four; round a circle beside a chain whose further seam is
not about its axis; for a piece beyond (`chain_seam`, a ring's and a traced run's alike) from a face
no plane, into a face no cylinder along a straight seam nor torus about the plane's normal, or
beside a wall neither a plane, a cylinder, a cone nor a sphere; off the seam's axis, round a stretch
of a circle, where the edge or the wall's strip crosses the seam's normal surface or the leg
crosses no seam; along a traced edge, where the run crosses the seam's normal surface or its
footprint crosses the seam other than twice. The gate holds every one to the
field as ever. `tests/composed_rounds.rs` is the gate (each against a closed form or a quadrature
of the set's own words: `spill.sv` at every boss radius from 21 to 21.7, chamfered and rounded, to
1e-6, and its volume to the field's mesh within the mesh's own stand-off; the straight, coaxial and
three-face chains to 1e-9; the traced flat to 1e-8 and the traced round to 1e-6; two bosses up
through one slope, each chamfered over its round, to 1e-8), `tests/brep.rs` the strip across a
tangent seam unioned into its body (the kernel's touch across a tangent edge, before the lowering
relied on it). `fillet_oracle` holds none of these yet: whether OCCT's fillet
overflowing onto a tangent face reads them as `std.rounds` does is open. A region of a body's faces the planes do not close is
supported by the bodies it reads grown by `SUPPORT_REACH` of its longest length
(`region::general`), so a fill the table refused is meshed. The field's composition of an operand
rounded by a cut would read a sheet of nothing where the cut's face lies on the face it rounds,
and a fill whose flat crosses that sheet ends a feature curve on another the mesher cannot keep
apart (`delaunay::refine::protect`), so **an operand built exactly is read by its own boundary**
(#419, [fields.md](fields.md)): a boss on a plate drawn in a plane of its own, its chamfer onto
the plate's rim round, meshes (`tests/composed_rounds.rs`, the user's part at boss radii 21 to
21.7, all but one in the slow tier).

**Rounds built exactly: the lowering table** (#194, #183 L7, `solid::lowering`): a region over a
body's faces is recognised by its terms' shapes (`recognised`: two or three `FaceSide`s alike, an
`Angle` gate per pair at a half turn, the common-part-against-meeting sum (the meeting cut or not
where its edge ends, `meeting_of`: std's is, #195), an `Ends` per pair, and
one bound saying the section — a ball's `centres` set or a flat's sum), **never by a word's name**
(`by_hand.sv`'s own `Rolled` is built by the same entry), read off the stratum (`faces::cached`,
`BodyFaces::plane`/`cylinder`/`segment`/`circle`/`coaxial`/`normal`) into **pieces**, one per edge
where a face reference names a face in pieces (`run`: every edge between the two groups, each kept
within the whole run's ends, as the field reads them): `Run` (the section along a straight edge
between planes, or a plane and a cylinder, or two cylinders, about lines along it — the section's
`Trace`s lines and circles, the ball where their offsets meet (`twin.sv`), a chamfer's flat beside
a circle the conic its sum of distances makes (the section read as a meridian's corner,
`Corner2`, its round and its flat by the meridian's own rule: `chain::cut_chain` with no seam,
the ball's arc `brep::corners::arc_between` and the flat `corners::sum_between`, which
`corners::ball`/`sum` call too — **one section builder** for every run and ring, with a chain or
without, and one chain walk, `piece::chain_from`, #427;
`Stroke::Conic`, a rational quadratic profile edge: an extrusion face; turned in a ring, a
revolution's; between two circles a Cartesian oval, refused) — carried on past each end until the plane faces it
ends on cut it off (#195's meaning: a square one is the prism's end, one at a slant a half-space it
is kept within, `Run::within`, the recipe's `"within"` as a point and a normal; a run-out); a
cylinder or a sphere it ends on carried into until every line of its section's hull has crossed
it, and kept within or outside that solid — **and where some line passes it by**, a pin through
the crease or a bore breaking out of a case's skin in front of a boss's end (#904 A8), the run on
the far side of it along the same line, between the same faces, ending on it alone too, the two
are one run over both and the gap, kept outside its surface (`piece::across_cuts`): what the field
reads, each point within the ends of the run nearest it and both runs' ends one surface; ends on
two faces (a short slot along the crease, a round end each) still refused, `a run its curved end
does not cut off` (`round_lowering::a_run_a_pin_cuts_across_is_one_run_kept_outside_it`, the closed
form to 1e-9; `a_round_between_cylinders_on_skew_axes_is_built`); where several faces meet the run's end (two chamfers
before it mitred there), carried on until its whole section lies past one and the same of their
cuts (`fillet::Trim::past`) — each point past some cut is not past the region all of them keep, and
the run took too little off (`brep_offset::edges_chamfered_one_after_another_mitre_at_their_corners`) (#218, `Run::kept`, `Kept` over a `Surf`, the recipe's
`"kept"` read by `recipe::surface_of` and built by `build::kept_by_solid`; one `ends_at` sorts a
piece's ends: `post.sv`, a D's flat, a quarter ball); a face along the edge or
another curved end refused, `unbuilt`), `Ring` (a circle where a
plane meets a coaxial cylinder or cone: the section in the meridian at its start turned through its
angle, `Ring::angle`; a sphere centred on the axis is a circle in the meridian (`Trace::Circle`,
`knob.sv`'s neck); a stretch ends at seams, at planes through the axis at its ends' meridians,
or at a plane not through it — carried on round until its whole section has crossed it (each
point's first crossing a sinusoid's rising root, `first_over` — a point on the plane at the
stretch's end rising there at once, never a turn on, #948), and kept within it, #218: a D's rim
at its flat — or at a cylinder or a sphere it runs into, carried a
quarter of the way through what its section's points spend in that solid and kept on its side,
`Kept`: the twin bar's rim; and in either case no further than the meridian halfway to where its
circle comes back within the faces it ends on, square there whatever of its section they leave,
"Where a run's end does not cut its section off" below), a run and a stretch going on from one
another **glued** at their shared section (a face going on tangent across a seam is no end,
`BodyFaces::continues`, the field's and the lowering's one rule), and so two stretches going on
from one another across a seam between arcs (#574, `assemble::same_section`: one's turn ending in
the section the other's starts in, the two cones or cylinders tangent along the seam's line) —
never unioned, for two tori tangent along that section's circle meet in nothing a trace can
follow, and their union was refused at some drafts and radii and not at others (`rounds/notch.sv`,
a drafted dog-bone notched in each waist, its far rim eight stretches: the closed form's volume to
1e-9 at every draft and radius the issue found refused),
`Corner` (`corner_at` each vertex three faces share, planes or a cylinder or a sphere among
them, #218: the ball `r` from each surface, a 3×3 solve on the tangent planes then Newton's steps
on the signed distances, `roots::newton_onto` — none for planes; the vertex within its normal
cone; the cell less the ball, `fillet::corner_among` over the faces' surfaces, `Corner::faces`,
the recipe's corner `"faces"`, `fillet::corner` its three-plane case; plus each edge's run or
ring stretch, cut short at its section even where it ran out past a slanted end, a ring's
stretch turned only to the corner's meridian, glued there: `dshaft.sv`), `Rolled` (along an edge no line or coaxial circle carries — a tee's crotch, a bore's rim, a
run-out across a slope: the ball rolled along the whole run, `brep::fillet::roll`, its canal
fitted (a cylinder, exact, where both faces are planes: [brep.md](brep.md)), carried past an end
and cut off there (`fillet::end_at`'s `Cut`s: the plane, a curved end's solid, or — where a face
goes on tangent across a seam into another piece's — the section
that piece ends at, refused where the ball crosses the seam elsewhere, `seam_aligned`:
`bullnose.sv`); one per run of the region (`BodyFaces::runs`), winding round both faces or not,
cut short at a corner's section (the plane of the ball's arc) and glued there (`frustum.sv`); a
flat the same with a line for the arc, `fillet::flat_along` (`crotch.sv`); `Beyond` (#422: a ring's
or a traced run's section past a seam's normal surface, against the chain's next face, glued to the
edge's own piece there — "Against the smooth surface" above); the recipe's
`"rolled"` rolled against
the body's stratum, rebuilt from the `"stratum"` terms `cad.rs` writes beside it, the gate's
against `BodyFaces::brep` brought back by `BodyFaces::back`); no pieces where the faces never meet
or an edge turns the other way. **The gate**: the
pieces' B-rep held to the region's own field (`agreement`, by area; empty only where the field
reads empty), and holding what the field reads just inside each edge (`beside_edges`: a probe of
the boundary built sees nothing of a piece never built), before anything reads them. Memoised against the region's own terms (`solid::reads::reads_own`)
and the stratum's `Arc` (`Caches::lowering`; a key over `solid::reads` walks every round of
the body), asked once a recipe. The gate's construction — the pieces' B-rep and its mesh — is
remembered against the recipe node up to the order it lists things in (`lowering::gated`,
`canonical`; a rolled one's stratum by identity; `Caches::gated`, the latest few): `std.rounds`' words
state each edge and corner once (#415), but a word that does not — a document's own written
without `symmetric`, a round over faces drawn `in` a plane or riding a frame — makes a region for
each order of the faces it names, and `Corner(a, b, c)` and `Corner(b, a, c)` lower to one
construction, built and meshed once and held to each region's own field. `cad.rs` builds a body's lowered regions in one set as one recipe node, `"lowered"`
(`assemble`: each distinct piece once, a run cut short at a corner's section and **glued** there,
`boolean::glued`; two runs mirrored at a convex corner cut in the plane halving it and glued
there, below; the rest unioned — two runs crossing are a mitre), the first region standing for
the rest; a region lowered to nothing is no operand, nor material a `Through` cutter reads
(a convex edge's fill, `edit_rounds.rs`); a body's face readers go in `"on_late"` /
`"cut_late"`, applied after `bound` (`faces::stratum`, the one split the field and the recipe
share; the native host applies them so too). `sweeps_among` sends a lowered region to the exact path. Kernel rules the stack found and
this needs (ported from #79–#110): edges along a face cut where they cross its edges
(`query::crossings_in`), touches read to second order (`ssi::touch_near`/`touch`,
`trace_beside`/`unseen_beside`), equal crossing cylinders' ellipses split where they cross
(`boolean::crossings_among`), `query::line_cylinder`, `Surface::offset`, a fitted edge read at its
own `tol`; a plane or a cylinder touching a cylinder along a line is one line, judged by the gap
(the half chord is its square root, #218), and a vertex an edge lying in a face makes on that
face's edge is on every face that edge bounds (a meeting of the face beside it may end there). `cad::recipe_about` (the app's exact solid, built about its own origin) writes a
lowered node's pieces, corners and balls about it with the rest: every node writer writes its points
through one `brep::recipe::Writer`, so no key can be left behind. Refused by the kernel, by name
(#218): a meeting that branches where two surfaces touch (`roll`), a fitted canal met tangent by
a cylinder (`runout.sv`'s lug rounded all over: ill-conditioned), and rounds of separate
statements touching along a line (no manifold file describes it). **The oracle** (#218's 8): `gcs-cli tests/fillet_oracle.rs` (OCCT) builds each rounds
example's body less its lowered regions, rounds the same edges by OCCT (`How`: its fillet or
chamfer, `solvent_cad_round`, `Session::round`, each edge the one through a point the pieces give;
a convex body rounded all over by its offsets, `solvent_cad_opened`; a chamfer beside a curved face,
where OCCT's chamfer is ruled between legs along the faces, by a loft through `std.rounds.Flat`'s
sections in closed form, `solvent_cad_loft`) and holds ours to it (volume 1e-9 where both are
analytic, 2e-5 where either fits a B-spline; face kinds alike, OCCT's B-spline standing only for an
analytic face of ours it fits). OCCT's fillet and offsets misread an edge along or ending on a
periodic surface's seam (`rail`, `post`, `frustum`): asked of the same solid with its seams turned
(`seams_turned`). `tests/round_lowering.rs` is the gate (every volume the closed form's to
1e-9, a rolled one #85's quadrature's to 1e-6: `slot.sv`, `pad.sv`, `rail.sv`, `knob.sv`,
`tee.sv`, `bore.sv`, `runout.sv`, `twin.sv`, `post.sv`, `dshaft.sv`, `groove.sv`, `bores.sv`,
`frustum.sv`, `crotch.sv`, `bullnose.sv`, `dome.sv`, `point.sv`).

**Where a run ends on a face all but along it** (#952, a cast bracket's side plate, its lug's
rim and its outline rounded). A face a run ends on is a half-space, read wherever the run's
region is, so four rules keep it to the end it stands at, each in `BodyFaces::end_faces`, which
the field (`conditions::ends`, through `end_sides`) and the lowering (`piece::ends_at`) both read
— and only where every edge of the set is one the lowering carries a section along
(`BodyFaces::carried`): a ball rolled along a traced run is cut by `fillet::end_at`'s own reading.
What a run ends on is a face, or a plane square to an edge where its run ends on none
(`faces::EndBy`).
- **A face whose surface the run itself goes across is no face it ends on** (`ends_at`,
  `goes_across`: some point of the run's edges past the surface on the side that would keep it,
  by more than the boundary's tolerance — exactly for a plane against a line or an arc, at 64
  points otherwise). No side of it holds the run, and kept within the side the run sets out on
  the ring was cut to a lens it could not be glued to its other quarter by ("two pieces joined at a
  section share no face there"): a plate's side meeting a lug's rim at 1.77°, the arc dipping under
  the side's plane for 3.5° and coming back across it, or crossing the rim, its plane cutting the
  arc a second time across the disc. The run ends there square, as at a seam: the field's meeting
  goes on past that end, and the ring stops at its end's meridian.
- **Nor is a plane turning back over the run at a reflex corner of less than a right angle**
  (`ends_at`: the run outside it, and its outward normal along one of the run's faces'): kept off
  it, the run lost what of its section the plane reaches back over, `s / tan θ` from the corner at
  depth `s` in from its side — at 7.6° the round cut back along 15 of a side's 15.5, its cap meeting
  the next corner's round at its touch in a sliver 6 µm wide. What an end cuts is what lies past
  it, and past such a corner the plane cuts all of the run: it ends square there. At a right angle
  the plane is the square end; past one it cuts nothing alongside, and stays.
- **A run-out goes no further than the edge it runs out along** (`end_faces`'s ends read
  `Some(v)`): where the plane a run ends on at `v` makes a straight edge with one of the run's faces
  going on from `v` the way the run is carried, the faces that edge's run ends on at its far end, on
  the side it runs on — or the plane square to it there where it ends on none — are the run's too,
  but one the run goes across. Carried past a convex turn of 1.13° until its whole section had
  crossed the next side, a run went a hundred millimetres on along it and, past the corner where
  that side turns back in, cut a groove along its plane through the plate's top. The lowering
  carries a run no further for them (`piece::ends_beyond`): each is a cut it is kept within (a
  curved one where it reaches the prism or the ring at all).
- **Two runs mirrored at a convex corner meet in the plane halving it** (`assemble::mitred`):
  where two runs' edges, and no third's, end at one vertex no corner's ball stands at, each
  section reaching over the other's edge, and each run is the other's image in the plane halving
  the turn, each is cut in that plane and **glued** to the other there, not carried on and unioned —
  so long as what else each is kept within, its prism's other end included, is the other's too, or
  holds the section in that plane and every corner of what the other was carried over
  (`carried_over`: its prism past the plane within all it is kept in), so the union is the regions'
  as before. Unioned, a run carried a hundred millimetres along the other lay inside the other's
  piece, its round all but on the other's, and where the other was cut the two caps lay in one
  plane: the union left a tongue of a face 5 µm wide between the rounds.
`tests/shallow_ends.rs` is the gate (the half disc's rim and the rim from a crossing, by Pappus;
a plate's rim round a polygon two of whose sides turn 1.13° beside a reflex corner, and one whose
side turns 7.6° back in, by the stack's mitre with each run square at a reflex corner; each to
1e-9 and exported with no sliver). Square at a reflex corner leaves the wedge between the two ends
unrounded; the ball turned about the corner's edge — a ring about that edge through the turn,
glued to both — is what fills it, and waits on a set that says it.

**A seam tangent to the drawing's numbers** (#962: a deck plate's stepped edge, a plate of twelve
lobes and waists, a hook's band). Where two faces meet tangent is one rule, the body's: their
outward normals one there to `faces::TANGENT`, a millionth (`BodyFaces::continues`, `seam`), so no
edge, no end and no corner. The pieces going on from one another across such a seam — a run and a
ring's stretch, or two stretches — are one section there by the same reading
(`assemble::one_section`: their sections through the seam's point, each point of either within a
billionth of the construction's size, and four `TANGENT`s of its distance from that point, of the
other's), not only where their directions agree to a billionth. An author states numbers to five
or six places, and a seam is tangent to those alone: `rounds/step.sv`'s side leaves its lobe a
ten-millionth of a radian off the lobe's tangent, the lobe meets its arc of 5 half a millionth off.
There the body read no end and the assembly no one section, so the pieces were unioned, their caps
a millionth apart, and the Boolean could not classify them ("a piece of `end` (a plane) touches
`near` … along no face of either", naming the round's own construction planes); or two
stretches' caps through one seam, planes whose normals agree to the rounding of the arcs' centres
and not to `1e-12`, were never the one face the glue pairs ("two pieces joined at a section share
no face there"). Each join at a seam carries how far apart its two sections stand
(`Assembly::gaps`; the recipe's join `[i, j, gap]`), the glue reads them that far apart
(`recipe::chained`: eight tolerances and twice the gap) and measures the chain's edges once it is
whole, as at a rolled piece's fitted cap; and `boolean::glued` takes two plane faces as one
surface where each's corners lie within its tolerance of the other's plane. `std.rounds.Grown`
reads a vertex's faces by the same rule (`solid::grown`: two normals one to `TANGENT` are no
corner, so no ball's sector is cut at a seam; [sets.md](sets.md)). **A face a run ends on reaching
back across a seam** is refused by name: a flush boss whose side one end of a run along an
obround's flat stands on, kept outside it, reaching back over the section where the flat runs
tangent into the half circle. The half circle's ring is its own region's, whose run ends nowhere
there, so nothing cuts it: the shape is the ring's whole section beside the run's cut short, and
where the boss crosses the ball's arc there, the run's round and the ring's, tangent along that
arc, meet it in one point — not one face to glue, and neither unioned nor cut from the two glued
by this kernel yet (its search for where the boss's meeting with the run's round meets the ring's
torus, at a point where it only touches it, does not end): "where its round goes on tangent
across a seam at […], a face one of its runs ends on reaches back across the section the two
share" (`Assembly::across`), said by the round's body and label. `tests/rim_seams.rs` is the gate
(the step at every line length down to none and every radius to past its concave arc's, plates
of four, six and twelve alternating arcs, each stated to the micron, mirrored and turned, round and
chamfer, either cap; seams `1e-8` to `5e-7` off tangent; `step.sv` as drawn, with the stack's mitre
at the frame's corners; the boss slid along the rim through the seam, built either side of the
section and refused over it; each volume the closed form's to 1e-9, a boss's chord by quadrature,
and the plates grown 1 by Steiner's). Seams turned past `TANGENT` and short of a few thousandths
of a radian are corners whose pieces are refused yet (a ring and a run unioned there, a turn of
`1e-6` to `1e-4`: "a cylinder's pieces do not close").

**Where a run's end does not cut its section off** (#962 item 3: a disc's rim chamfered past its
flats, a rim ending on another arc's face, a channel's fill broken by a hole). `std.rounds` says a
run is carried past its edge's end, cut off by the faces it ends on, and read against the meeting
— the faces' surfaces' meeting within those faces (`crease.ends`) — only where its nearest point
of that meeting is on the crease (`distance(p, crease) <= distance(p, meeting)`, the field's
`conditions::Rest`). So the run goes past its end no further than **halfway to where the meeting
comes back** within the faces it ends on: round a circle the meridian halfway across the gap to
where the circle is next within them all (`piece::resumed`: sampled at a 720th of a turn and
bisected, the gap itself where it never is — the crease's other end, so the two ends meet in the
middle and the ring is a whole turn — and nothing where the circle goes on within them at once);
along a line the plane square to it halfway across the chord the face it ends on cuts from the
line, where the line comes back out within them. The piece is turned or carried that far, square
there, and kept within (or outside) each face it ends on, whatever of its section that face
leaves: the set's own extent, not a construction's. Short of it, where every point of the section
has crossed a plane at that end (#218's `first_over`; each comes back within only past the
halfway meridian) or stays a quarter of the way into a curved face's solid, the piece stops there
as before — the same shape. Only where the set's edges are **one run**: in several (a crease a
slot cuts across) each point is read against the ends of the run nearest it (#881), the meeting
is no one curve within one set of faces, and a piece goes on only as far as each end cuts its
whole section off, refused by the old names where it does not (a short slot along the crease, a
round end each: `round_lowering::a_refused_rounds_piece_is_said_at_its_round`). A disc of 75 with four flats 74.5 out (#962's 149), its rim chamfered
1, was refused ("a stretch of a circle its end does not cut off"): the section's inner half never
reaches the flat. Each rim's stretch now turns to the meridian halfway to the next arc, kept
within its two flats, and two stretches carried to one meridian are one section there, glued; a
face both are kept within cuts their caps alike, so it is no face reaching back across a section
the two share (`Assembly::across` reads it of one alone). A disc of 20 whose far side a disc of
100 overhangs by 1, rounded deeper than 1 (#962's 125, "a stretch of a circle its curved end does
not cut off"), turns the smaller rim's ring all round, kept within the larger disc. A straight run
some of whose section's lines pass the curved face it ends on by is still joined to the run along
the same line beyond that face (`across_cuts`) where there is one — now **between the same
surfaces**, not the same faces: a trench under a channel's floor cuts the floor in two faces of
one plane, each named by the channel, and the two runs on either side were never one (#962's 133,
"a run its curved end does not cut off"); and where there is none, the run is carried to the
plane halfway across the chord (`run_piece`'s `halfway`), kept outside the face. A round asked of
one face of a smooth chain (#962's 125, a round of the R90 arc alone) ends square at the tangent
seam: no face ends it there, so the meeting is the arc's whole circle, and the crease is the
nearest of it only over the arc's own meridians — the set itself stops square, as the stretch
does. `tests/curved_ends.rs` is the gate (the disc with one, two and four flats, chamfered just
under, at and over the flat's depth and rounded, with a bore keyed four times, once and not at
all — a key whose arc turns past half round crossed by its side's plane, ending square — mirrored
and turned, either cap; the two discs in common, rounded under and just either side of the
overhang and well over it, mirrored, and chamfered, and in union; a drum of the two standing on a
flange, its foot filled; a pocket's floor-to-wall fill broken by a hole inside, on and beyond its
wall, at the pocket's end with no edge beyond, and across a trench under the floor, square-on and
skewed; each volume the closed form's to 1e-9, the section left at each depth integrated by
Gauss–Legendre or tanh–sinh, the pocket's against the kernel's body without the fill). Refused by
the kernel yet, said where it stands: the round exactly as deep as the larger disc overhangs (the
round's torus touching that cylinder: "where a traced … meets a plane was not proved: its search
read more boxes than any search may"), and a chamfer past the overhang on some caps and depths,
the smaller disc's whole ring kept within the larger's cylinder unioned with the larger's ring
whose leg lies on that cylinder ("where a bspline … meets a cylinder was not proved").

**A piece cut off at a plane goes clear past it** (#962 item 4: an eye across an arm, its caps
flush with the arm's sides, filled to the arm's top and bottom; a channel's cutter rounded before
it is cut). A run carried past a plane it ends on at a slant was carried only until the last line
of its section crossed it, and a stretch of a ring turned only until the last point of its section
had (`first_over`): the plane then stood through the piece's end, turned from it by however far
the face was off square. A drawing states its numbers to five or six places, so a side is square
to an edge, or through a ring's axis, only to those — #962's 122 has an arm's side through a point
the solve leaves 0.8 µm off its other end's line across 67 mm, a turn of 1.2e-8 — and the piece
was cut along its own end by a plane a hundred-millionth of a millimetre off it, a sliver the
Boolean could not classify ("a run cut off at its end: a piece of `far` (a plane) touches `far` (a
plane) … along no face of either"; turned 1e-7, "a shell of the arrangement is kept on one side of
a plane (`far`) and dropped on another: two of its cells joined round an edge"), or, where the
turn was under a billionth of the run, left square with its cap that far off the face, which the
Boolean joining it to its body could not read ("a piece of the slot's `c0` (a plane) touches
`near` …"). Now a plane that cuts the piece at all — some line of its section crossing it short of the edge's
end or past it, to a part in 10¹² — is gone clear past: at least a thousandth of the section's
width past where it first meets it (`piece::clear_of`; a ring's stretch that far round in turns, no
further than halfway to where a point of its section comes back across it, `over_from`), so the
plane crosses the piece's sides and the cut end is the face's plane itself, exactly, at any slant.
No tolerance stands in for square: the set is cut where the face is. `tests/flush_ends.rs` is the
gate (an eye and a block boss flush with both sides, proud of both, flush with one, and — rounded
along the eye's cylinder or the boss's sides alone — 50 µm and half a millimetre short or proud,
the flush one their closed form's limit; round and chamfer, the eye's axis at the arm's middle and
off it; the eye and the block across the arm at a slant, kept between the sides' planes; a block
flush with the arm's end too; the arm's sides turned 1e-8 to 1e-3 of a radian; a slot open at both
sides, its rim rounded and its floor filled; its cutter rounded first, its ends square or turned
1e-8 to 1e-3, flush or proud; a half-round lug flush with a side, its centre 1e-8 to 0.1 off that
side either way; and #962's 122 as the issue reduced it, against the same drawing with its side
square: each volume the closed form's to 1e-9, each exported with no sliver). Refused by the
kernel yet, said where it stands, and not by the round: a boss's cap flush with a side only to the
drawing's numbers, the two planes crossing within the cap. The body's own union, unrounded, is
refused there — at a turn of 1e-8 it cannot read the two as one face ("a shell of the arrangement
is kept on one side of a plane (`s0`) and dropped on another", or "a piece of the eye's `far` (a
plane) touches the part's `s2` (a plane) … along no face of either"), and from 1e-7 to 1e-3 it
leaves strips of the top and the bottom nanometres to microns wide between the cap and the side,
which the export refuses ("`part` holds 4 faces thinner than the export's 10.000 µm"); a round
over the cap then fills the shelf the strip is ("the solid touches itself along a line").
`flush.rs` moves a face parallel to a plane onto it, not one turned from it: a cap turned from
the side by what the drawing's numbers leave is the next rule it needs. Rounded over its caps (`part.eye`, not `part.eye.eo`), a boss short of a side
meets the top in an edge of its own, filled however narrow the shelf it stands on: the set is not
continuous at flush there, as it is along the boss's sides.

**A face along a run cuts it lengthwise or not at all** (#962 item 4: a boss drawn standing on a
part's top, `from:` the top's height, and running past the part's side). Where the boss overhangs
the side, its underside is a face the fill along the boss's sides ends on, lying along the run in
the plane of the floor the fill stands on; a ring's stretch round a disc boss so overhanging ends
on that underside too, a plane square to its axis. Such a plane stands at no distance along the
run, so it is no end of it and carries the piece nowhere (it was refused, "a run carried on along
the face it ends on", and the ring "a stretch of a circle its end does not cut off"). Its
half-space is the side `BodyFaces::end_faces` keeps — where the run's own edge lies in the plane,
the material's behind it, which holds the fill on its floor — and it cuts the piece along its
length or not at all (`piece::along_run`: some point of the section past it, by a billionth of the
construction): where none is, no cut; where one is, the piece kept within it, a prism or a
revolution clipped by a plane along it, exactly. A boss sunk into the top had no such face and was
built before. No face of #962's 122's eye lies along a run: its caps are in the side's planes,
square to the fills. **Two faces flush only to the drawing's numbers where a run ends** are named
where they cannot be built (`piece::flush_only`, `Run::flush`, carried into what the failure calls
the round, `cad.rs`): a boss's end drawn flush with a side the solve leaves turned 1e-8 from it
leaves a strip of the boss's underside half a nanometre wide that the body, under its tolerance,
reads as none, while the fill is cut exactly at the side — and the Boolean joining them cannot close
the boss's wall half a nanometre past the fill's end: "`part`'s round or chamfer `filled`, which
ends where `arm.s0` and `boss.b0` are flush only to the drawing's numbers, turned 1.0e-8 of a radian
from one another at […]: a plane's pieces do not close". Turned 1e-7 to 1e-4 the body keeps the
strip (4.2 nm to 4.2 µm), which the export warns of with or without the round, and the round builds
beside it; at 1e-3 the strip is 42 µm and nothing is warned of. A boss short of the side by a hair,
rounded over its faces, leaves a strip of the top as wide as the hair between the side and its end's
fill, which is whole however narrow the shelf it stands on: the set's own, and the body's unrounded.
`tests/flush_ends.rs` holds the standing boss proud of one side, of both, flush with one and proud
of the other, flush with both, a hair (1 µm, 50 µm, 0.5 mm) short or proud along its sides alone
(the flush one their limit) and short over all its faces, at the part's corner flush with its side
and its end, round and chamfer; the side turned 1e-8 to 1e-3 either way, flush and 2 proud; and the
disc boss 3 and 5 past the side, its ring turned through `2π − 2 acos(e/ρ)`; each volume the closed
form's to 1e-9, each sliver the export warns of one the body has unrounded.

**A ball as wide as its cylinder** (brep-bench's fillet/cylinder-rim-to-dome): an offset by a
radius that takes a surface's radius to nothing is what the surface is drawn about, never nothing
(`Surface::offset` → `Offset`: a cylinder's axis, a sphere's centre, a torus's middle circle; `None`
only past it, where no point stands that far off). So a rim rounded to its cylinder's own radius
has its ball's centres on the axis alone, and the top shrinks to a point: the round is a
hemisphere. In the lowering it is a ring like any other, its section's arc centred on the axis,
which `build::revolve` turns into a sphere (`dome.sv`, the cylinder to half its height under the
dome: the closed forms' volume and area to 1e-9, a cylinder, a sphere and a plane), and the field
reads the centres as that axis (`conditions::closed_onto`, [sets.md](sets.md)). `fillet::roll`
meets the two offsets as they are: where one closes onto a curve and meets the other's in a point,
the ball's centre **stands still** there (`fillet::standing`), touching the first face round the
great circle square to the curve and the second at a point; its canal is the half of its sphere
on the point's side, exact, and the piece that hemisphere, the first face's strip down to the
circle and all of the second face inside the loop (`fillet::strip`, the one strip rule a rolled
canal's piece uses too). Cut, it is the dome, square to the axis or not — a top tilted 20° is
touched off the dome's pole, `z = 10 − 5 / cos 20°`, its rim an ellipse the lowering rolls a
ball along (`round_lowering::a_rim_tilted_off_square_rounded_to_its_cylinders_radius_is_a_dome`)
— and upside down it fills a bore's floor into a hemisphere
(`brep::a_cylinders_rim_rounded_to_its_own_radius_is_a_dome`). Past a closing offset the band a
ball needs is the whole of the face on the region's side (`holds_ball`), so a cylinder shorter
than the radius is refused as too small to hold it. Refused by name too: both offsets closing, one
closing onto a point (the ball is that face's sphere), the centre running along the curve it
closes onto, and a stretch of a loop; and where only a surface will do — a chamfer's leg, a ball
beyond a seam — an offset that closes is refused as that, not as too wide
(`Surface::offset_surface`: a spill's legs and balls). A cone shrunk past its apex is the same sheet
placed at its own apex, never none.

**A ball that takes the whole of a face** (brep-bench's fillet/slab-half-round,
l-block-inside-consumed, hole-waist, sliver-strip, boss-s-curve): the band a ball's contact
bounds on the face it rolls on — the points of the face nearer the other face's surface than the
contact — must be entered by none of the face's other edges (`fillet::holds_ball`), but an edge
*on* the contact, or beyond it, is the face's far boundary reached, not an edge in the band: the
strip runs to it, and the face goes (a slab's top rounded from both edges to half its width, two
rounds tangent along its middle; an L's inside edge filled as wide as both faces beside it, the
quarter cylinder meeting the faces past them square) or what is left goes on beyond the contact (a
strip 0.0002 tall between two rounds of 0.9999). Within ten of the boundary's tolerances, or of
the edge's own where it is wider (a fitted contact of an earlier round), an edge is on the contact
(`fillet::face_consumed`). An edge under the contact's greatest height nowhere is clear of the
band at once; any other is read against the contact (`Band::enters`): a crossing of the level
under the contact's lowest height is inside, found exactly by `curve_surface`, and between the
two each point against the contact's height at the point of it nearest, sampled and refined where
the edge may stand inside. A ball any wider reaches inside the face — past an L's far edge, across a
round's contact — and is refused as larger than the face can hold, as before (rung 3: a ball
rolling against the edge it overflows, or across the face beyond). `tests/brep.rs`'s
`rounds_that_take_the_whole_of_a_face` is the gate (each against its closed form to 1e-9, and each
one wider refused).

**A flat as wide as its cylinder** (brep-bench's chamfer/cylinder-rim-to-point): a chamfer's leg is
a set like the offset it is read from (`fillet::Leg`) — face `k`'s surface met with the other's
offset by the leg, a curve, or, where that offset closes onto a curve (a cylinder's axis) or a point
(a sphere's centre), the point where it crosses face `k`'s surface. There the face shrinks to the
point: every section of the flat ends at it, the flat closes onto it as a cone onto its apex, and
nothing about it is a special case — the flat's contact with that face is a degenerate edge, as a
sphere's pole is, and its strip all of the face inside the loop (`fillet::strip` with no walk, the
rule a standing ball's point of contact uses). Between surfaces turned about one axis with straight
meridians (a plane square to it, cylinders and cones about it) each distance is affine in the radius
and the height, so the flat is the section's triangle turned once, exact (`fillet::coaxial`; a cone,
its apex on the axis where a leg shrinks a face). Otherwise the flat is fitted, its column on the
shrunk face standing still: a rim tilted 20° off square chamfered to its cylinder's radius closes
onto where the axis crosses the top, `250π (1 − 1 / (3 cos 20°))` to 1e-11
(`brep::a_cylinders_rim_chamfered_to_its_own_radius_is_a_cone` — square, unequal legs and a bore's
floor filled to a drill's point — `brep::a_tilted_rim_chamfered_to_its_cylinders_radius_closes_onto_the_axis`;
a hemisphere's base shrunk to its centre, the sphere's offset closing onto a point, a spherical
sector to the fit's bar, `brep::a_hemispheres_rim_chamfered_to_its_radius_is_a_spherical_sector`;
`point.sv` through the ring and the tilted rim through the lowering's run,
`round_lowering::a_rim_chamfered_to_its_cylinders_radius_is_a_cone`,
`round_lowering::a_rim_tilted_off_square_chamfered_to_its_cylinders_radius_closes_onto_the_axis`).
Refused by name: both faces shrinking to points, a shrunk face holding another edge inside the loop
(the flat would take it), a loop winding round the shrunk face's surface, what the offset closes onto
missing that surface or lying in it, a stretch of a loop (no stretch closing to a point is cut off
yet), and a leg past the radius (`None`: no point stands that far in).

**The closing case** (#197, `rust/examples/conrod/`): a forged rod, every round and chamfer a
statement over the faces its solids gave it, built exactly (pockets drawn as prisms, so their walls
are planes and cylinders). What the stratum leaves is a finding, not a bug: rounds of separate
statements each read the body before the other, so a pocket's rim beside its corners' fills, or a
shank face's fill where a flange's run and a side's fill meet, leave material touching along a
line — the field reads it, the late Boolean's `Brep::check` (`topo::NOT_MANIFOLD`) refuses it and
`recipe::body` names it. Kernel rules it found: a traced meeting arriving tangent to a line or
conic crosses it once (`query::tangential`), and a lowered region's chains touching only along a
line stand side by side as two shells (`recipe::beside`, where their union fails and their common
is empty). Elaboration of many face-reading regions is linear: `flatten::Walk::region_of` reads a
plain declared name without building the alias table, and
`region::in_general_position`/`folded` key a face-reading region on `reads_own` and the strata
(`faces::cached`'s `Arc`s), never `reads` (which walks the body's every other round). Nor is a
body rounded all over quadratic in its rounds (`rounds/bores.sv`: nine faces, 18 regions, one for
each pair and three that meet, #625; 240 for every pair and three, and 1152, one for each order,
before #415): a
`when` reads only the aliases its condition names (`flatten::resolve::Lazy`, the table's own
rule per key; each answer held to the whole table under debug assertions, on a walk of a few
hundred aliases), and so does each pass over the blocks set aside; a
stratum's key (`faces::cached`) asks only the solids its walk reaches whether they read the body;
and whether a body's boundary is the field's (`solid::has_sweep`, asked before every evaluation, a
remembered one too) is remembered against `reads` (`Caches::swept`).
`tests/conrod.rs` is the gate.

**Draft (§6.9, 0.80, #245):** `solid(std.solids.Drafted(f, from:, to:, draft: θ))` ([0.199], #737;
through 0.198 `draft:` beside a prism's labels), or the same set written by hand — recognised by its
lean term, the angle read to a billionth of a degree (`solid/defined.rs`) — is
`SolidDef::Prism::draft` (radians; `None` undrafted): the section at ordinate `h` is `f` moved `|h|
tan θ` into its material from its own plane, the parting plane (an outer loop in, a hole out). Every
evaluator reads it: the facets (`primitive::prism`'s `taper`, `offset` by mitres, a side breaking at
the plane; `drafted_away` refuses a side taken to nothing, at validation, saying the ordinate's
size at which it comes to nothing from the face's own plane, to three figures, and that the sides
lean from it, #765), the field
(`ExtrudedField::drafted`: the side's value plus `|w| tan`, times the cosine — exact only where
`PlanarField::draftable`: convex loops of lines and whole circles, else refused) and the exact
kernel (the recipe's prism `"draft"`; `brep::build::drafted`: `offset` the profile — a line moved
square, a circle's radius, corners where lines cross or where sides go on tangent, an arc at an
angle refused — and `ruled` lofts each side of the plane, `boolean::glued` there; OCCT refuses it).
Faces keep the prism's names, a side crossing the plane two pieces of one name; a round over such a
face takes the pieces as one (`round(r) X` never rounds their crease), and the field's set distances
read a face in pieces on several surfaces meeting convex surface by surface
(`Reading::convex_pieces`, `conditions`: within, and at least `d` off where also within), else a
projection flips between pieces at the crease. The lowering table's run ends on a cone
(`Surf::Cone`, the recipe's `"kept"` cone a lofted frustum, `kept_by_solid`). A canal across the
parting plane (two pieces each side, `brep::fillet::roll`) is refused, naming the turn (rung 3).
Refused besides: a drafted side as a set or a carrier, an extruded surface or a tool. `rod.sv` is
drafted 3°; `tests/draft.rs` is the gate.
