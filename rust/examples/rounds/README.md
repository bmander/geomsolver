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
meridian. Otherwise the field reads it point by point, and the page meshes its surface in the
background.

Read them in this order:

| File | What it shows |
|---|---|
| `block.sv` | **The two words.** A block whose near rim is rounded (`block.near round(r: 2mm) block`) and whose far rim is chamfered (`block.far chamfer(d: 1mm) block`). Each run is cut off by the faces it ends on, and the runs mitre at the corners. |
| `pocket.sv` | **A concave round fills.** A pocket rounded inside (`round(r: 1.5mm) dish.pocket`): the floor's edges are filled, and where three meet at a corner the fill is a ball's. |
| `boss.sv` | **A round on a curved edge.** A boss filleted where it stands on a plate (`pad.boss round(r: 3mm) pad.plate`): a ring, with no special case for the circle. |
| `by_hand.sv` | **Your own round.** The round written out in the file: a set `Rolled(a, b, r)` and a word `A rolled(r) B` that makes it a solid and cuts it, in about twenty lines. |
| `disc.sv` | **A round in a region's own body, built exactly.** A disc defined as a set, its top rim rounded and its bottom rim chamfered by `std.Rounded` and `std.Chamfered`. Lowered into the meridian, so `solventc --step` writes a torus and a cone (1196.946 mm³, the closed form's). |
| `regions.sv` | **Regions in general position.** An ellipsoid by its foci, Steinmetz's bicylinder, and a Voronoi cell of a ball, each read point by point. |

The fillet stack's examples (#195), each the stack's program written with the words:

| File | What it shows |
|---|---|
| `pad.sv` | **A cast mounting pad.** A boss's root filled, two of the plate's edges rounded off and cut flush by the faces they end on, and a bore drilled through after, which neither round reaches. |
| `knob.sv` | **Curved faces.** A turned knob whose rod is filled where it stands on a disc (a cylinder against a plane) and where it enters a ball (a cylinder against a sphere): one word for both. |
| `rail.sv` | **A curved side, ending flush.** A rod half sunk along a plate, filled on both sides by one statement, each run ending where the rod and the plate end together. |
| `tee.sv` | **A traced crotch.** A pipe tee whose crotch is filled round the saddle where the cylinders meet, a curve nothing in the drawing carries. |
| `bore.sv` | **A rim a cut made.** A solid pipe drilled square into its wall, the bore's rim rounded off, said of the one body that is drilled. |
| `slot.sv` | **Tangent chains.** An obround boss's foot and a rounded pocket's rim: where two faces meet tangent there is no edge, so the runs carry on round the ends with no step. |
| `runout.sv` | **Runs that end on a face.** A boss's root on a slope cut off at a slant by the face it runs onto, and a hole's rim cut off where the hole breaks out of a wedge's end: carried on past the edge's end, and cut off by that face however it stands. |

`rust/gcs-core/tests/rounds_examples.rs` holds each file to what its comments say, the suggested
edits included, and the stack's examples to the stack's closed forms and #85's quadrature, each
round's volume read off the field within 2e-3 (the slow tier). `tests/rounds.rs` holds the words
to closed forms: Steiner's formula for a block rounded all over, the mitre, Pappus for the boss's
ring, the volume three flats take at a corner.
