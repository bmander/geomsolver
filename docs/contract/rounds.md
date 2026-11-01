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
`RegionField::empty`, no term of a body. `std.rounds`: `Run`, `Fill`, `Corner`, `CornerFill`,
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
it, and kept within or outside that solid; where several faces meet the run's end (two chamfers
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
point's first crossing a sinusoid's root, `first_over`), and kept within it, #218: a D's rim at
its flat — or at a cylinder or a sphere it runs into, carried a
quarter of the way through what its section's points spend in that solid and kept on its side,
`Kept`: the twin bar's rim), a run and a stretch going on from one
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
`canonical`; a rolled one's stratum by identity; `Caches::gated`, the latest few): `std.rounds`' words
state each edge and corner once (#415), but a word that does not — a document's own written
without `symmetric`, a round over faces drawn `in` a plane or riding a frame — makes a region for
each order of the faces it names, and `Corner(a, b, c)` and `Corner(b, a, c)` lower to one
construction, built and meshed once and held to each region's own field. `cad.rs` builds a body's lowered regions in one set as one recipe node, `"lowered"`
(`assemble`: each distinct piece once, a run cut short at a corner's section and **glued** there,
`boolean::glued`; the rest unioned — two runs crossing are a mitre), the first region standing for
the rest; a region lowered to nothing is no operand, nor material a tool `through:` the body reads
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
