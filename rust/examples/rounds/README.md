# Rounds and chamfers, written in Solvent

A round or a chamfer is not a feature of the kernel here. `round` and `chamfer` are words of the
standard library (`rust/lib/std.sv`), written over things the language says anyway, and you can
write your own the same way (`by_hand.sv`). Issue #183 is the plan this follows.

What the words are built from:

- **A face is a set**: `block.near` is the points of the near face, `block.near.surface` the whole
  plane it lies on, and `p inside block.near` the side of it the material is on.
- **The distance from a set**: `distance(p, centres) >= r`, where `centres` is itself a set, such
  as where a ball's centre may stand.
- **A bound on measurements**: `distance(p, f1) + distance(p, f2) <= 10mm`, or one distance against
  another, `distance(p, crease) <= distance(p, meeting)`.
- **Where an edge ends**: `p inside crease.ends`, so a run along an edge stops at the faces it
  ends on.
- **The material's angle at an edge**: `angle(a, b, at: p) <= 180deg` holds at a convex edge.
- **Repetition over a solid's faces**: `repeat a in A { repeat b in B if b != a { … } }`, and a
  relation word whose body makes a solid and cuts it from (or adds it to) the body the faces are
  of.

In the app the words are also tools: **Fillet** (⇧F) and **Chamfer** (⇧C) under Solids. Click
the edges of a solid you want (a chain of them, or a face for every edge of it), type the size and
press Enter, and the statements above are written into the document for you — one per edge or
face, or `round(r: …) body` when every face is picked.

A solid made from a set is a **region**. Where every term is about one axis it is a meridian
turned once, and a round or chamfer between two of its own bounds is built exactly in that
meridian. Otherwise the field reads it point by point — and where the set is one the lowering
table recognises (#194, `solid::lowering`), recognised from what its body says and never from the
word's name, it is also built exactly and held to that reading: a run along a straight edge
between planes, or between a plane and a cylinder along it, is a prism whose face is a cylinder
(cut off by the faces its edge ends on, square or at a slant), a run round a circle where a
plane, a cylinder, a cone or a sphere meet about one axis is a ring whose face is a torus (a
stretch of one where the circle goes on tangent into a flat side), a corner where three faces
meet is the ball's patch, a flat is a plane (or a conic's prism or turn beside a curved face), a
round along any other edge is the ball rolled along it, traced, its canal fitted, and a flat along
one the sum of distances sectioned and fitted the same way. Those bodies show as exact solids and export by
`solventc --step` and `--stl`; a region no entry builds is meshed by the page in the background, and its export is
refused, naming why.

Read them in this order:

| File | What it shows |
|---|---|
| `block.sv` | **The two words.** A block whose near rim is rounded (`block.near round(r: 2mm) block`) and whose far rim is chamfered (`block.far chamfer(d: 1mm) block`). Each run is cut off by the faces it ends on, and the runs mitre at the corners. Built exactly: 11350.525 mm³. |
| `pocket.sv` | **A concave round fills.** A pocket rounded inside (`round(r: 1.5mm) dish.pocket`): the floor's edges are filled, and where three meet at a corner the fill is a ball's. Built exactly. |
| `boss.sv` | **A round on a curved edge.** A boss filleted where it stands on a plate (`pad.boss round(r: 3mm) pad.plate`): a ring, with no special case for the circle. Built exactly, a torus's face. |
| `by_hand.sv` | **Your own round.** The round written out in the file: a set `Rolled(a, b, r)` and a word `A rolled(r) B` that makes it a solid and cuts it, in about twenty lines. The same set as the library's, so the same entry builds it. |
| `disc.sv` | **A round in a region's own body, built exactly.** A disc defined as a set, its top rim rounded and its bottom rim chamfered by `std.Rounded` and `std.Chamfered`. Lowered into the meridian, so `solventc --step` writes a torus and a cone (1196.946 mm³, the closed form's). |
| `blank.sv` | **Rounds on an offset cone.** A gear blank's face as a set: the pitch cone grown by the addendum (`std.Offset`) and shrunk by the dedendum, between a toe sphere and a heel sphere, the tip's rims rounded by `std.Rounded`. The grown cone is still a line of the meridian, so the rounds are built exactly there: `solventc --step` writes two tori (1353.180 mm³, the closed form's). A chamfer beside a sphere bows with it, the sum of distances a parabola's arc in the meridian, and is built exactly too. |
| `regions.sv` | **Regions in general position.** An ellipsoid by its foci, Steinmetz's bicylinder, and a Voronoi cell of a ball, each read point by point. |

The fillet stack's examples (#195), each the stack's program written with the words:

| File | What it shows |
|---|---|
| `pad.sv` | **A cast mounting pad.** A boss's root filled, two of the plate's edges rounded off and cut flush by the faces they end on, and a bore drilled through after, which neither round reaches. Built exactly: 26549.817 mm³. |
| `knob.sv` | **Curved faces.** A turned knob whose rod is filled where it stands on a disc (a cylinder against a plane) and where it enters a ball (a cylinder against a sphere): one word for both. Built exactly: two tori, the neck's section a line and two arcs in the meridian (26224.398 mm³). |
| `rail.sv` | **A curved side, ending flush.** A rod half sunk along a plate, filled on both sides by one statement, each run ending where the rod and the plate end together. The rod splits the plate's top in two; in a section the rod is a circle the ball rolls against. Built exactly: 26306.866 mm³. |
| `tee.sv` | **A traced crotch.** A pipe tee whose crotch is filled round the saddle where the cylinders meet, a curve nothing in the drawing carries. Built exactly: the ball rolled along the saddle, traced, its canal fitted (#85's quadrature's volume to 1e-11). |
| `bore.sv` | **A rim a cut made.** A solid pipe drilled square into its wall, the bore's rim rounded off, said of the one body that is drilled. Built exactly: rolled against the pipe as the cut leaves it. |
| `slot.sv` | **Tangent chains.** An obround boss's foot and a rounded pocket's rim: where two faces meet tangent there is no edge, so the runs carry on round the ends with no step. Built exactly: runs and stretches of rings, glued at the sections they share. |
| `runout.sv` | **Runs that end on a face.** A boss's root on a slope cut off at a slant by the face it runs onto, and a hole's rim cut off where the hole breaks out of a wedge's end: carried on past the edge's end, and cut off by that face however it stands. Built exactly: each ball rolled on past its edge's end and cut off at the face's plane. |
| `post.sv` | **A run that ends on a curved face.** A plate's top edge rounded up to a post standing at that edge: each run ends on the post's wall, so it is carried on into the post and cut off by its cylinder, kept outside it. Built exactly (the closed form's volume to 1e-9). |
| `dshaft.sv` | **Corners with a curved face.** A D-shaped shaft rounded all over: the flat's edges run into the round and are cut off by it, the rims are stretches of rings cut off by the flat, and where the flat, the round and a cap meet the ball touching all three rounds the corner, its cell with a patch of the round among its faces. Built exactly (Steiner's formula over the shrunk D, to 1e-9). |
| `twin.sv` | **Between two curved faces.** Two rods side by side, overlapping, made one: their walls meet in two straight seams, and in a section both are circles. Built exactly: the ball stands where the two circles grown by its radius meet, each seam's fill a prism of three arcs (the closed form's volume to 1e-9). Chamfered instead, the flat's sum of distances from the two walls is one from the rods' axes: an ellipse with the axes its foci, a prism over a rational quadratic (the closed form's volume to 1e-9). A flat of unequal legs there is a Cartesian oval, which no rational curve holds: refused by name. |
| `groove.sv` | **A run mitred against a ring on one cylinder.** A rod's groove along a plate, its rims rounded all round: at each corner a run along the top meets a ring's stretch round the groove's end, each with a face on the groove, and their rounds meet in a curve ending where both touch it — a point where the two surfaces touch and their meeting is two branches crossing, traced up to it. Built exactly (a quadrature in the distance from the groove's axis, to 1e-9). |
| `bores.sv` | **Corners where a face meets two curved walls.** A slab drilled twice, the bores overlapping, rounded all over: where a cusp between the bores reaches a face, the ball touching the face and both walls rounds the corner, its cell over a plane and two cylinders. Built exactly, and OCCT's fillet agrees (`fillet_oracle`). |
| `frustum.sv` | **Rolled runs beside corners.** Half a frustum rounded all over: its generators are straight, but a section square to one cuts the cone in no circle, so the ball is rolled along it; each rolled run is cut short at the section of the corner at its end (the plane of the ball's arc) and glued to that corner's cell. Built exactly: Steiner's formula over the shrunk half frustum to 1e-9, the generators alone #85's quadrature over an open spine (1e-6). OCCT's fillet rounds the generators alone to the same volume (7e-9); all over, it fills the corners beside them by surfaces of its own, and differs from ours only there. |
| `crotch.sv` | **A flat along a traced edge.** A tee's crotch chamfered: the flat — where the walls' distances, each over the other's leg, sum to one — sectioned square to the saddle between its legs and fitted, as a rolled round's canal is. Built exactly (a quadrature over sections square to the saddle, to 1e-6). OCCT's chamfer is ruled between legs measured along the faces, a different flat, so the oracle asks OCCT instead what it can say of ours: the distances of points of our flat from the two walls of OCCT's own tee, which sum as the flat says to 1e-8. |
| `bullnose.sv` | **A traced run going on into a ring across a tangent seam.** A boss standing on the seam where a plate's flat land runs on smoothly into a bullnose, its root filled: a ring's stretch on the land, and on the bullnose a ball rolled along a curve no line or circle carries, carried on past the seam and cut off at the section where the stretch ends. Built exactly (#85's quadrature over the whole turn, to 1e-6). Off the seam the ball crosses it in another section than the edge does, and no one section joins the two: refused, saying so. |

`rust/gcs-core/tests/rounds_examples.rs` holds each file to what its comments say, the suggested
edits included, and the stack's examples to the stack's closed forms and #85's quadrature, each
round's volume read off the field within 2e-3 (the slow tier). `tests/round_lowering.rs` holds the
exact constructions to closed forms to 1e-9 (a rolled one to #85's quadrature, 1e-6), and to their
fields. `tests/rounds.rs` holds the words
to closed forms: Steiner's formula for a block rounded all over, the mitre, Pappus for the boss's
ring, the volume three flats take at a corner. `tests/offset_corners.rs` holds the rounds and flats
on an offset cone's corners, `blank.sv` among them, to Pappus over the corners worked out by hand.
