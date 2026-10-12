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

A solid made from a set is a **region**. Where every term is about one axis it is a meridian
turned once, and a round or chamfer between two of its own bounds is built exactly in that
meridian. Otherwise the field reads it point by point — and where the set is one the lowering
table recognises (#194, `solid::lowering`), recognised from what its body says and never from the
word's name, it is also built exactly and held to that reading: a run along a straight edge
between planes, or between a plane and a cylinder along it, is a prism whose face is a cylinder
(cut off by the faces its edge ends on, square or at a slant), a run round a circle where a
plane, a cylinder, a cone or a sphere meet about one axis is a ring whose face is a torus (a
stretch of one where the circle goes on tangent into a flat side), a corner where three square
faces meet is the ball's patch, a flat is a plane, and a round along any other edge is the ball
rolled along it, traced, its canal fitted. Those bodies show as exact solids and export by `solventc --step` and
`--stl`; a region no entry builds is meshed by the page in the background, and its export is
refused, naming why.

Read them in this order:

| File | What it shows |
|---|---|
| `block.sv` | **The two words.** A block whose near rim is rounded (`block.near round(r: 2mm) block`) and whose far rim is chamfered (`block.far chamfer(d: 1mm) block`). Each run is cut off by the faces it ends on, and the runs mitre at the corners. Built exactly: 11350.525 mm³. |
| `pocket.sv` | **A concave round fills.** A pocket rounded inside (`round(r: 1.5mm) dish.pocket`): the floor's edges are filled, and where three meet at a corner the fill is a ball's. Built exactly. |
| `boss.sv` | **A round on a curved edge.** A boss filleted where it stands on a plate (`pad.boss round(r: 3mm) pad.plate`): a ring, with no special case for the circle. Built exactly, a torus's face. |
| `by_hand.sv` | **Your own round.** The round written out in the file: a set `Rolled(a, b, r)` and a word `A rolled(r) B` that makes it a solid and cuts it, in about twenty lines. The same set as the library's, so the same entry builds it. |
| `pad.sv` | **Rounds that add and rounds that take away.** A cast mounting pad: a boss's root filled, two of the plate's near edges rounded, a bore drilled through after. Concave or convex is read off the geometry. Built exactly: 26549.817 mm³. |
| `slot.sv` | **Rounds along tangent chains.** An obround boss's foot filled and a pocket's rim rounded off, each along flat sides and round the half circles or corners they go on into. Where two faces meet tangent there is no corner, so nothing ends a round there. Built exactly: runs and stretches of rings, glued where they meet. |
| `rail.sv` | **A round beside a cylinder, over a face in pieces.** A rod half sunk along a plate, filled on both sides where it stands. The rod splits the plate's top in two, so the round is a run along each line where a piece meets the rod; in a section the rod is a circle the ball rolls against. Built exactly: 26306.866 mm³. |
| `knob.sv` | **A section with a curved side.** A knob turned about one axis: a rod on a disc, a ball on the rod. The rod's root is filled against the disc and its neck against the ball; in a section through the axis the sphere is a circle, so the neck's section is a line and two arcs. Built exactly: two rings whose faces are tori, 26224.398 mm³. |
| `tee.sv` | **A round no line or circle carries.** A pipe tee, its crotch filled where the branch meets the main pipe on a saddle. The ball is rolled along that loop, traced, its arc swept into a canal fitted as a B-spline surface. Built exactly: its volume #85's quadrature's to 1e-11. |
| `bore.sv` | **A rolled round taken away.** A bore drilled into a solid pipe's wall, its rim rounded off: convex, along a saddle, rolled against the pipe as the cut leaves it. |
| `runout.sv` | **Rounds that run out onto a face.** A boss's root against a sloped top, ending obliquely at the near face, and a hole's rim breaking out of a wedge's low end: each rolled on past where its edge ends and cut off by the face there. |
| `disc.sv` | **A round in a region's own body, built exactly.** A disc defined as a set, its top rim rounded and its bottom rim chamfered by `std.Rounded` and `std.Chamfered`. Lowered into the meridian, so `solventc --step` writes a torus and a cone (1196.946 mm³, the closed form's). |
| `regions.sv` | **Regions in general position.** An ellipsoid by its foci, Steinmetz's bicylinder, and a Voronoi cell of a ball, each read point by point. |

`rust/gcs-core/tests/rounds_examples.rs` holds each file to what its comments say, the suggested
edits included. `tests/round_lowering.rs` holds the exact constructions to closed forms to 1e-9,
and to their fields. `tests/rounds.rs` holds the words to closed forms: Steiner's formula for a block
rounded all over, the mitre, Pappus for the boss's ring, the volume three flats take at a corner.
