# Consistent embedded spline regions

All **288 intended spline-face joins** now pass a whole-pair embedding check after an
explicit common-boundary construction. All **35,532 nonincident spline-pair separations**
remain positive after the same adjustments. Each member therefore has a consistently
joined, embedded representative of its complete spline-face region. The maximum movement
of any face, and maximum discrepancy of any representative shared curve from its encoded
3D edge, are both **7.750197126168117e-10 mm**.

This connects the individual-face and pair-separation results. The analytical blank faces
still need to be attached to these spline regions. This is not yet a closed-solid or
matched-pair acceptance result, and the source STEP files remain unchanged.

## One common representative for every spline face

Every incident spline-face pair shares exactly one full natural boundary with the same
unit-U parameterization. Its separately encoded boundary curves use identical polynomial
bases, but their poles can differ slightly. `spline_joins.prepare` checks these bases and
sets both boundary columns to their exact rational coefficient-wise average. It processes
all joins once, creating one surface per face that is reused in every subsequent test.
Multiple assignments to the same complete boundary are refused.

Changing the two ends of a face independently for separate pair tests would not establish
a consistent surface collection. Here all its boundary changes are present before any
join is tested. The maximum Euclidean coefficient change bounds the entire surface
movement by the B-spline convex-hull property, including interiors and end corners.
An exact same-basis bound also compares each common curve with the original standalone
3D edge throughout its full interval. The separate movement allowance is 1e-8 mm.

## Whole joined rectangles

The checker reverses a face's transverse parameter as necessary so that one face ends at
the shared curve and the other starts there. The U parameter remains the common intrinsic
edge parameter. It then concatenates the two control nets, retaining the shared column
once, and constructs a clamped tensor spline over `[0,1] x [0,2]`. A degree-fold interior
knot at the interface provides exact C0 continuity. Both original halves and their full
parameter domains are retained without refitting.

The [global injectivity test](SPLINE-EMBEDDING.md) is applied to this entire joined
rectangle with one fixed linear projection. Positive definiteness of the projected
Jacobian's symmetric part proves that no two distinct parameters coincide. Hence the two
face interiors cannot overlap, the shared edge has no gap, and no second intersection
exists elsewhere. This is stronger than checking local normals or points near the join.
The proof requires continuity, not C1 continuity across the concatenation knot; it does
not assert a smooth parameterization at that artificial interface.

Every one of the 360 spline faces belongs to at least one verified joined rectangle.
The checker independently derives all incident pairs from the bound edge/vertex inventory
and requires exact agreement with the single-boundary join inventory. Pairs sharing only
a vertex or more than one boundary would require different checks and cannot pass this
inventory gate silently.

## Preserving separation from other faces

For each previously verified nonincident pair with distance lower bound `d`, the new lower
bound is `d - movement(face_a) - movement(face_b)`. All 35,532 values remain strictly
positive. The earlier pair inventory must exactly match the newly derived nonincident
pairs and refer to the same edge and vertex evidence. Every original proof row must be
complete and verified. The report records the minimum remaining exact rational bound.

Consequently the adjusted spline faces on each member meet only through their intended
shared boundaries. This statement is per member in its local frame, not a collision check
between the assembled gear and pinion. Analytical attachments and their shared vertices
remain part of the unfinished closed boundary realization.

## Results and reproduction

| Member | Verified joins | Covered spline faces | Maximum face movement, mm |
|---|---:|---:|---:|
| Pinion | 96 | 120 | 7.751e-10 |
| Gear | 192 | 240 | below 7.751e-10 |

The full join and separation-transfer audit takes **55.16 seconds**. The focused suite has
**69 passing tests**. New controls include both transverse parameter orientations, a gap
that cannot be silently joined, an overlap away from a shared edge, and explicit common
coefficient construction with bounded movement and unchanged input data.

```sh
python3 experiments/cad-backend/check_spline_joins.py \
  /private/tmp/solvent-cad-vertices.json \
  /private/tmp/solvent-cad-spline-separation.json \
  /private/tmp/solvent-cad-spline-joins.json
python3 -m unittest discover -s experiments/cad-backend -p 'test_*.py'
```

`spline-join-results.json` binds the full report, original source and STEP files, and prior
pair-separation evidence. Each joined-pair report records its original faces, edge ID,
boundary directions, exact projection and Jacobian margins, and common-edge error bound.
Attaching analytical faces, global swept-material coverage, continuous mating and
source/reader accuracy transfer remain necessary for the full gear-pair goal.
