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

`rust/gcs-core/tests/rounds_examples.rs` holds each file to what its comments say, the suggested
edits included. `tests/rounds.rs` holds the words to closed forms: Steiner's formula for a block
rounded all over, the mitre, Pappus for the boss's ring, the volume three flats take at a corner.
