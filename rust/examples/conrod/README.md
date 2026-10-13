# A forged connecting rod, rounded in Solvent

The closing case of the fillet work (#66's rung 4, #183's L8, #197): a connecting rod with
rounds almost everywhere, written with the standard library's `round` and `chamfer`. It began as
X0's paper program, `docs/fillets/x0/conrod.sv`, written before any of it parsed.

`rod.sv` is the model and `rod.svd` the drawing: the rod in its plane of swing, from the side, a
section across the shank where the I is, and an isometric.

## How it is written

The sketch is the sharp outline in the plane of swing: two eyes, two bores, the shank's tapering
outline and the pockets' outline (a flange's thickness inside the shank's sides, as far clear of
each eye). Each solid is a prism of one of those faces, and the rod is the shank, plus the eyes,
minus the bores and the two pockets.

Every round and chamfer is then one statement over the faces those solids gave the rod:

| Statement | What it does |
|---|---|
| `rod.shank.left round(r: rf) rod.big_end` (×4) | The shank's sides filled into the eyes, 6 mm: concave, so material is added. |
| `round(r: rp) rod.front_pocket` (×2) | Each pocket rounded inside, 2 mm: the floor's edges, the upright corners, and a ball where three meet. |
| `rod.big_end chamfer(d: ch) rod.big_hole` (×2) | Both ends of each bore. |
| `round(r: re) rod.big_end` (×2) | Each eye's rims, 1 mm. |
| `round(r: re) rod.shank` | The flanges' four long edges, each run cut off by the eye it ends on. |

None of these says which way to round or where an edge ends. Concave edges are filled and convex
ones cut, read off the geometry, and each run is carried past the end of its edge and cut off by
the face it ends on (here, an eye's wall). Each round reads the rod as the rod's other terms leave
it (the stratum), so it does not matter that the shank-to-eye edges come from a union and the
pocket's edges from a cut.

The pockets are drawn as prisms rather than as the paper's region of distances from the shank's
faces. That way their walls are planes and cylinders, which the lowering table builds exactly.

## What it is built as

Every round and chamfer above is recognised by the lowering table (#194, #218) and built
exactly, so `solventc --step` and `--stl` write the rod: 94 faces, 61370.577 mm³. The fills are
prisms beside a cylinder, the eye rims and bore chamfers are rings, the pocket corners are balls
touching two planes and a cylinder, and the flange runs are carried into the eyes and cut off by
them.

## The edges left sharp, and why

Two sets of edges are left sharp. Each round reads the rod before any other round, so where rounds
from separate statements meet they cannot blend:

- **The shank's faces into the eyes** (`rod.shank.near round(r: re) rod.big_end`, and so on). At
  the vertex where a shank face, a shank side and an eye wall meet, this 1 mm fill, the flange's
  1 mm run and the 6 mm fill all meet. Each was made against the sharp rod, so what they leave
  touches itself along a line.
- **The pockets' rims** (`rod.front_pocket round(r: re) rod.shank`). At each pocket corner the
  rim's straight run and its ring's stretch both cut down to the corner's line, past the 2 mm
  fill that corner was given. What remains touches itself along that line.

Add either set and the field still reads the rod as written: the page meshes it, and it reads
the same everywhere else. The exact build refuses it and says why and where, for example:

    cut_late: rounds or chamfers of separate statements meet where the body they each read leaves
    them touching along a line, not built yet (#218): not a manifold: edge … near [7.2874, -8.5,
    33.2098] …

A pocket's rim on its own (without the inside round) is built: at each corner its run and its
ring's stretch touch along the corner's line, and they stand side by side as two shells.

Which edges "every other edge" means was finding 5 of the paper program. The answer here is the
edges a round can reach without meeting another statement's round at a corner. Rounding the rest
needs a round that reads another round, which is still an open question.

## How it is checked

`rust/gcs-core/tests/conrod.rs`:

- The rod has material where these comments say it does.
- The bores' chamfers and the eyes' rims match Pappus's closed forms to 1e-9 of the rod's volume.
- The flanges' four runs take the same volume, and so do the two pockets' rims.
- The edges left sharp are refused by name.
- In the slow tier: the rod is built and written as STEP and STL, and the field's reading of
  three of its rounds (a side into an eye, a pocket inside, a flange's edge) matches the kernel's
  construction to within 2e-3.
