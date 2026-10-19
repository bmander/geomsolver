# Rounds, chamfers and draft

Part of the [contract](../../CLAUDE.md) the code is held to. Read it when you touch
`solid/lowering.rs`, `brep/fillet.rs`, `brep/corners.rs`, or `std.Rounded`. An issue number (#n) or
a spec version ([0.n], see the [language changelog](../solvent-changelog.md)) says where a rule came
from.

**Rounds and chamfers are words of `std.sv`** (#183 steps 4–7). `repeat a in X` over a solid (a
face path) runs over its faces (F4): set aside, and where the faces are not given the elaborator
names them once the solids are built (`solid::faces::face_paths`, structural: each primitive's
own faces under its route, `FacePath`) and elaborates again (`program::Again::Faces`,
`flatten::expand_faces`, `Walk::face_links`); a `when` in the body keeps a copy — `when f in X
and f != g` (F8; 0.91, #267: `Walk::faces_named`, `Walk::stands_for`, the `if` retired). A relation word's body may hold blocks, instances, sets and body statements (F5,
`parser/definitions.rs`); `A.body` in a body statement is the solid a face path is of. **The
stratum** (F6): a body's terms that read its faces (`faces::readers_of`) are left out of the
boundary they read and applied last by the field (`document.rs`: fills, then cuts). A face path
is matched against `solid::exact`'s names (`BodyFaces` is built from `exact::build`, shifted to its
origin; cached in `Caches::faces`); one naming nothing holds nowhere, and a region over it is
`RegionField::empty`, no term of a body. `std.sv`: `Run`, `Fill`, `Corner`, `CornerFill`, `Flat`,
`FlatFill` (sets over faces) and the words `A round(r: …) B`, `round(r: …) X`, `A chamfer(d: …) B`,
`chamfer(d: …) X`; in a region's body `std.Rounded(a, b, r)` and `std.Chamfered(a, b, la, lb)`
over two of its own sets (`set` formals, #191; `p inside S(…)` may be written in place).
**Each edge and corner once** (#415, [0.148]): the six sets are `symmetric` in their faces
(`Flat`'s each with its leg, `symmetric((a, la), (b, lb))`), so the words — which reach an edge
as `(a, b)` and `(b, a)` and a corner in six orders — make one set, one solid and one cut or fill
of each (`flatten::same`, [language.md](language.md)): `rounds/bores.sv` is 240 regions, not 1152.
`tests/symmetric.rs` holds the count and the block rounded and chamfered all over to the words
written without `symmetric`.
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
(`region_profile`, read back by `meridian_region`). `std.Chamfered` is multiplied through by its
legs, so a leg of nothing cuts nothing (a bound weighing every measurement by nothing holds
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
into the next piece). The section against a chain is the one against whichever face of it is
nearest: a round's ball touches the face its offset meets the other face's in, and changes face
where it touches the seam; a flat is each face's own sum of distances with the other face, and
changes face where it crosses the seam's normal surface — there a point's distance from either face
is its distance from the seam, so the two are one set:
- **along a straight edge beside a seam along it, or round a circle beside a seam about its axis**
  (`lowering::chain`, `piece::seam_along`, `seam_round`): in the section the corner of the other
  face's trace and a chain of the face's own trace to the seam and each next face's from its seam
  (lines and circles, up to four past the edge's own, `piece::CHAIN`, each found where the last
  one's footprint lands past another seam, `piece::onto`); the ball against the piece whose touch
  lies on it, the flat a line between lines else a conic's arc over each piece in turn
  (`corners::conic_between`; two circles with unequal legs a Cartesian oval, refused), changing
  piece on the chain's normal at each seam it crosses. Swept or turned like any section: the strips
  on the chain's side split at the body's seams, the flat in pieces, the one over the face's own
  surface what the table built before (`rounds/ledge.sv`, a rib beside a block's rounded rim; a
  boss centred on a rounded disc; a rib on a top going on through a round into a slope).
- **round a circle beside a seam not about its axis** (a boss near a plate's rounded rim,
  `rounds/spill.sv`; `piece::beyond`: the face a plane going on into a torus about its normal): a
  flat's ring kept within the seam's normal surface — the cylinder through the seam along the
  plane's normal (its `Kept`) — and beyond it the flat against the torus, sectioned in the
  half-planes through the ring's axis between where the leg on the plane crosses the seam, fitted
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
  the flat beyond it sectioned square to the run (`spill::Along::Meeting`) between where the leg
  crosses the seam, found by bisecting the footprint along the run; a round's ball rolled along
  the run (`fillet::roll_across`, its band let cross the tangent seam) with the stretch between
  the planes of its arc where it touches the seam cut away (`Rolled::wedge`), and the ball rolled
  against the next face there, glued at both.

The recipe's `"beyond"` (after the rolled runs in a lowered node's joins): the edge it runs
`"along"` (a ring's `"axis"` and `"crease"`, or the `"face"` it meets the wall in), `"toward"`, the
`"chain"`'s face, the `"wall"` and the `"split"` surface with `"sides"`, the `"seam"` (a circle or a
line), the `"legs"` or the ball's `"radius"`, and the `"ends"`; a rolled run's `"kept"` and `"wedge"`; all
written through the node's `Writer`, so `cad::recipe_about` carries them. `boolean::glued` glues on
a curved face too, an edge matched by a point of it on the other's (a traced meeting and a fitted
one), or to a run of the other's end to end (a traced flat's crossing cut where that flat is
seamed). **Refused by name** (`piece::spilled`: "its section reaches across the seam where a face
goes on tangent into another …", the field reading the region): on both sides of its edge; into
more than one face; along a seam that is not along a straight edge, or a curved seam beside one;
beside two seams; beside a seam running only part of its edge; past a face it goes on into across
an edge no seam, or a chain of more than four; round a circle beside a chain whose further seam is
not about its axis; off the seam's axis, round a stretch of a circle, from a face no plane, into a
face no torus about the plane's normal, where the edge or the wall's strip crosses the seam's
normal surface or the leg crosses no seam; along a traced edge, from a face no plane, into a face no
cylinder along a straight seam nor torus about the plane's normal, where the run crosses the seam's
normal surface or its footprint crosses the seam other than twice. The gate holds every one to the
field as ever. `tests/composed_rounds.rs` is the gate (each against a closed form or a quadrature
of the set's own words: `spill.sv` at every boss radius from 21 to 21.7, chamfered and rounded, to
1e-6, and its volume to the field's mesh within the mesh's own stand-off; the straight, coaxial and
three-face chains to 1e-9; the traced flat to 1e-8 and the traced round to 1e-6), `tests/brep.rs`
the strip across a tangent seam unioned into its body (the kernel's touch across a tangent edge,
before the lowering relied on it). `fillet_oracle` holds none of these yet: whether OCCT's fillet
overflowing onto a tangent face reads them as `std.sv` does is open. A region of a body's faces the planes do not close is
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
`Corner2`, its round and its flat by `brep::corners::ball`/`sum`, the meridian's own rule;
`Stroke::Conic`, a rational quadratic profile edge: an extrusion face; turned in a ring, a
revolution's; between two circles a Cartesian oval, refused) — carried on past each end until the plane faces it
ends on cut it off (#195's meaning: a square one is the prism's end, one at a slant a half-space it
is kept within, `Run::within`, the recipe's `"within"` as a point and a normal; a run-out); a
cylinder or a sphere it ends on carried into until every line of its section's hull has crossed
it, and kept within or outside that solid (#218, `Run::kept`, `Kept` over a `Surf`, the recipe's
`"kept"` read by `recipe::surface_of` and built by `build::kept_by_solid`; one `ends_at` sorts a
piece's ends: `post.sv`, a D's flat, a quarter ball); a face along the edge or
another curved end refused, `unbuilt`), `Ring` (a circle where a
plane meets a coaxial cylinder or cone: the section in the meridian at its start turned through its
angle, `Ring::angle`; a sphere centred on the axis is a circle in the meridian (`Trace::Circle`,
`knob.sv`'s neck); a stretch ends at seams, at planes through the axis at its ends' meridians,
or at a plane not through it — carried on round until its whole section has crossed it (each
point's first crossing a sinusoid's root, `first_over`), and kept within it, #218: a D's rim at
its flat — or at a cylinder or a sphere it runs into, carried a
quarter of the way through what its section's points spend in that solid and kept on its side,
`Kept`: the twin bar's rim), a run and a stretch going on from one
another **glued** at their shared section (a face going on tangent across a seam is no end,
`BodyFaces::continues`, the field's and the lowering's one rule),
`Corner` (`corner_at` each vertex three faces share, planes or a cylinder or a sphere among
them, #218: the ball `r` from each surface, a 3×3 solve on the tangent planes then Newton's steps
on the signed distances, `roots::newton_onto` — none for planes; the vertex within its normal
cone; the cell less the ball, `fillet::corner_among` over the faces' surfaces, `Corner::faces`,
the recipe's corner `"faces"`, `fillet::corner` its three-plane case; plus each edge's run or
ring stretch, cut short at its section even where it ran out past a slanted end, a ring's
stretch turned only to the corner's meridian, glued there: `dshaft.sv`), `Rolled` (along an edge no line or coaxial circle carries — a tee's crotch, a bore's rim, a
run-out across a slope: the ball rolled along the whole run, `brep::fillet::roll`, its canal
fitted, carried past an end and cut off there (`fillet::end_at`'s `Cut`s: the plane, a curved
end's solid, or — where a face goes on tangent across a seam into another piece's — the section
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
`canonical`; a rolled one's stratum by identity; `Caches::gated`, the latest few): `std`'s words
state each edge and corner once (#415), but a word that does not — a document's own written
without `symmetric`, a round over faces drawn `in` a plane or riding a frame — makes a region for
each order of the faces it names, and `Corner(a, b, c)` and `Corner(b, a, c)` lower to one
construction, built and meshed once and held to each region's own field. `cad.rs` builds a body's lowered regions in one set as one recipe node, `"lowered"`
(`assemble`: each distinct piece once, a run cut short at a corner's section and **glued** there,
`boolean::glued`; the rest unioned — two runs crossing are a mitre), the first region standing for
the rest; a region lowered to nothing is no operand; a body's face readers go in `"on_late"` /
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
where OCCT's chamfer is ruled between legs along the faces, by a loft through `std.Flat`'s sections
in closed form, `solvent_cad_loft`) and holds ours to it (volume 1e-9 where both are analytic, 2e-5
where either fits a B-spline; face kinds alike, OCCT's B-spline standing only for an analytic face
of ours it fits). OCCT's fillet and offsets misread an edge along or ending on a periodic surface's
seam (`rail`, `post`, `frustum`): asked of the same solid with its seams turned (`seams_turned`).
`tests/round_lowering.rs` is the gate (every volume the closed form's to
1e-9, a rolled one #85's quadrature's to 1e-6: `slot.sv`, `pad.sv`, `rail.sv`, `knob.sv`,
`tee.sv`, `bore.sv`, `runout.sv`, `twin.sv`, `post.sv`, `dshaft.sv`, `groove.sv`, `bores.sv`,
`frustum.sv`, `crotch.sv`, `bullnose.sv`).

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
body rounded all over quadratic in its rounds (`rounds/bores.sv`: nine faces, 240 regions; 1152, one
for each order of every pair and triple, before #415): a
`when` reads only the aliases its condition names (`flatten::resolve::Lazy`, the table's own
rule per key; each answer held to the whole table under debug assertions, on a walk of a few
hundred aliases), and so does each pass over the blocks set aside; a
stratum's key (`faces::cached`) asks only the solids its walk reaches whether they read the body;
and whether a body's boundary is the field's (`solid::has_sweep`, asked before every evaluation, a
remembered one too) is remembered against `reads` (`Caches::swept`).
`tests/conrod.rs` is the gate.

**Draft (§6.9, 0.80, #245):** `solid(f, from:, to:, draft: θ)` (or `depth:`) is
`SolidDef::Prism::draft` (radians; `None` undrafted): the section at ordinate `h` is `f` moved `|h|
tan θ` into its material from its own plane, the parting plane (an outer loop in, a hole out). Every
evaluator reads it: the facets (`primitive::prism`'s `taper`, `offset` by mitres, a side breaking at
the plane; `drafted_away` refuses a side taken to nothing, at validation), the field
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
