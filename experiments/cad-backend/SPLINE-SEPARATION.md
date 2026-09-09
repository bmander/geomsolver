# Separation of nonincident spline faces

All **35,532 spline-face pairs with no common topological vertex** are strictly separated
over their complete finite surfaces. The conservative minimum separation bound is
**0.003638559993994761 mm**. The full audit takes **34.96 seconds**. The **288 incident
pairs** are recorded separately and are not verified by this check.

This is the first inter-face geometric result after the individual embedding audits. It
applies to the actual indexed polynomial spline coefficients, not just sampled meshes or
unrotated reference faces. Pairs involving analytical faces, the intended joins of incident
faces and a common shared 3D boundary realization remain open.

## Complete pair inventory

`check_spline_separation.py` binds the complete spline inventory to the extracted edge
data and the named vertex endpoints. It checks unique edge/vertex inventories, valid
endpoint IDs, ordered face-wire closure and single-cycle vertex links. It derives each
face's incident edge and vertex sets from these records. A common vertex sends a pair to
the explicit incident-pair inventory; it does not silently disappear from the report.
Every other pair of spline faces on the same member is tested. This is a per-solid check,
not a gear/pinion assembly collision check.

Each spline's finite ordered wire must cover its exact full clamped parameter rectangle.
The polynomial coefficient and knot validation is shared with `spline_embedding.py`.
Non-unit weights, discontinuous bases, invalid dimensions and changed trim domains are
refused. Filtered two-face and single-member trials cannot claim complete pair coverage.

## Exact support planes

A clamped polynomial B-spline lies in its control-point convex hull. Dot-product extrema
therefore bound its complete support in any chosen direction. Strictly disjoint support
intervals provide a separating plane. The checker first tries the three world-coordinate
directions; those alone discharge 34,024 pairs.

For overlapping boxes, control-net secants choose three additional candidate directions
per surface. Their cross products produce a local normal and two transverse directions.
Directions are quantized to integer coefficients solely to keep the exact arithmetic
small. Acceptance uses rational dot products against every control coefficient; neither
the secants nor their quantization are presumed to bound the geometry. These additional
support planes discharge another 1,460 pairs without subdivision.

For a separating direction `n` with positive support gap `g`, every point pair is at least
`g / sum(abs(n_i))` apart. The L1 norm is an upper bound on the Euclidean norm, so this is
a conservative rational distance lower bound without a rounded square root.

## Complete subdivision for the remaining pairs

The remaining 48 cases use exact tensor-spline subdivision. Midpoint knot insertion raises
the split knot's multiplicity to the degree, then extracts the two clamped child nets
with their shared boundary coefficients. Their parameter rectangles cover the complete
parent, including the split boundary. This changes the representation, not the surface.

Each unresolved surface-product cell is replaced by both children of the larger surface
node. The split axis is chosen from control-net secant extents; this affects cost only.
Every accepted product cell has a strict separating support plane. Exhausting the queue
proves coverage of the entire original product domain. Exhausting the explicit cell budget
retains unresolved work and prevents a separation claim. The minimum lower bound across
all accepted cells bounds the complete surface pair.

The small difficult gear pair, faces 1 and 3, passes with 17 product cells and a distance
bound above 0.0057 mm. Its geometry-only trial takes 0.39 seconds; the complete input-bound
two-face command takes 1.80 seconds. It precedes the full indexed run.

## Results and controls

| Member | Verified nonincident pairs | World boxes | Additional planes | Subdivided | Unverified incident pairs |
|---|---:|---:|---:|---:|---:|
| Pinion | 7,044 | 6,572 | 472 | 0 | 96 |
| Gear | 28,488 | 27,452 | 988 | 48 | 192 |

The minimum lower bounds are 0.021142132287000237 mm for the pinion and
0.003638559993994761 mm for the gear. These are conservative proof bounds, not exact
closest-point distances. The actual joins of incident faces still require separate
checks; a small positive distance is neither expected nor sufficient there.

The focused suite has 65 passing tests. New controls include intersecting and touching
planes, oblique separated planes with overlapping world boxes, exhausted budgets, and
exact preservation of the independent polynomial `(u²,v³,u*v)` through tensor subdivision.
The existing embedding tests also cover the extracted shared polynomial validator.

```sh
python3 experiments/cad-backend/check_spline_separation.py \
  /private/tmp/solvent-cad-vertices.json /private/tmp/solvent-cad-spline-separation-small.json \
  --member 1 --faces 1 3
python3 experiments/cad-backend/check_spline_separation.py \
  /private/tmp/solvent-cad-vertices.json /private/tmp/solvent-cad-spline-separation.json
python3 -m unittest discover -s experiments/cad-backend -p 'test_*.py'
```

`spline-separation-results.json` binds the complete report, original source, STEP files,
edge and vertex evidence. Each report row identifies the face pair, method, tested and
accepted product cells, unresolved work and exact rational lower bound. Incident rows
name the actual shared vertices and edges. Global swept-material coverage, continuous
mating and source/reader accuracy transfer remain separate acceptance requirements.
