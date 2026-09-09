# Periodic analytical-face representatives

The six analytical faces with periodic seams now have embedded annular representatives.
Their largest bounded boundary displacement from the original extracted STEP curves is
**1.0066420937655798e-10 mm**. The full seam audit takes **19.01 seconds**; the focused
Python suite has **53 passing tests**. Together with the 75 nonperiodic face results in
[TRIM-LOOPS.md](TRIM-LOOPS.md), every analytical face has an individually embedded
representative. Inter-face embedding and a common shared 3D topology remain unproved.

## Exact seam data and representative construction

The inventory identifies one twice-used edge on each periodic face. Both incidences are
straight constant-U parameter curves with exactly matching finite V intervals and opposite
wire orientation. The seam arcs have disjoint endpoints in the planar wire. The checker
refuses other seam structures or unequal V intervals rather than inferring correspondence
from numerical proximity.

Let `L` and `R` be the two imported constant U values, with `L < R`. The construction uses
the earlier exact midpoint closure at ordinary corners. At the four seam corners it uses
the corresponding raw seam endpoint, so both seam copies retain the same V interval
including its endpoints. It then clamps every representative Bezier U pole to `[L,R]`.
This last step is an explicit bounded change to the representative, not a claim that the
original curve lies inside the slab. All changed curves undergo fresh injectivity,
adjacency and non-neighbor separation checks.

The checker additionally proves that every open non-seam curve lies strictly inside the
slab, allowing side contact only at the four designated seam corners. Bernstein pole
bounds prove strictness in each piece interior; every internal piece endpoint is checked
separately. A curve touching a slab side elsewhere cannot pass. The two seam meridians
themselves must remain exactly unchanged by the representative construction.

## True period and annular embedding

The representative's actual angular coordinate is defined symbolically as

```text
angle(U) = L + 2*pi * (U-L)/(R-L).
```

The two seam curves consequently map to exactly the same meridian, separated by exactly
one mathematical revolution. No decimal approximation is substituted for this period.
For `L <= U <= R`, the angular change from the rational representative coordinate U is
bounded by `abs(2*pi - (R-L))`. The existing outward rational pi enclosure bounds this
quantity rigorously. Add it to the maximum U coefficient change and use the existing
sphere/cone derivative bounds to obtain the full world-distance displacement bound.

The rational UV wire must pass the complete Jordan-loop test. Its bounded interior lies
inside the slab, and only the two designated seam arcs meet its sides. The spherical
latitude must stay away from both poles; cones require positive radius and nonzero axial
slope. On this regular domain the surface map identifies only equal-V points on opposite
seam sides. Identifying those two oppositely oriented arcs produces an embedded annulus;
no other boundary or interior identifications are possible under these checks.

This proves existence of a specific representative of the tolerant face. It does not
modify the STEP files, assert that their raw curves close exactly, or establish that every
STEP reader uses the same representative. Realizing all representative faces together
with shared 3D edges, ruling out inter-face crossings, checking the global swept material
and proving continuous mating remain distinct requirements.

## Complete results

| Member | Face | Seam edge | Displacement bound, mm |
|---|---:|---:|---:|
| Pinion | 0, toe sphere | 146 | 1.007e-10 |
| Pinion | 146, back cone | 441 | 2.686e-11 |
| Pinion | 147, heel sphere | 442 | 6.648e-11 |
| Gear | 0, toe sphere | 290 | 5.806e-11 |
| Gear | 289, back cone | 872 | 5.147e-11 |
| Gear | 290, heel sphere | 874 | 9.274e-11 |

All bounds are below the separate 1e-8 mm representative displacement budget. This budget
is not a replacement for the nominal geometry or full solid acceptance criteria.

Tests refuse unequal seam intervals, adjacent seam arcs, extra slab-side contact and
spherical pole crossings. An out-of-slab control point exercises explicit correction-error
accounting; a width substantially different from a full period yields a correspondingly
large displacement bound. Earlier loop tests retain crossing and budget-exhaustion controls.

```sh
python3 experiments/cad-backend/check_trim_seams.py \
  /private/tmp/solvent-cad-edges.json /private/tmp/solvent-cad-trim-seam-small.json \
  --member 0 --face 146
python3 experiments/cad-backend/check_trim_seams.py \
  /private/tmp/solvent-cad-edges.json /private/tmp/solvent-cad-trim-seams.json
python3 -m unittest discover -s experiments/cad-backend -p 'test_*.py'
```

The small back-cone trial takes 2.12 seconds including loading the complete edge evidence.
Filtered runs cannot claim complete seam coverage. `trim-seam-results.json` binds the
complete report, original inputs and prior nonperiodic-face report by SHA-256.
