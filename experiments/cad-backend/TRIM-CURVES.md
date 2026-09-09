# Analytical trim curves: injectivity and separation

The 81 analytical sphere/cone faces of the indexed pair contain 1,188 parameter curves.
`check_trim_curves.py` binds their complete ordered inventories to the previously checked
edge/face evidence and the actual STEP hashes. It makes two limited geometric checks:

1. Every individual finite pcurve is injective in its UV chart.
2. Every pair of non-neighboring curves within each wire is strictly disjoint in that chart.

All 1,188 individual curves and all 106,004 non-neighboring pairs pass. This extends the
combinatorial wire/vertex checks with geometric evidence; it is not yet a Jordan-loop or
surface-embedding certificate. The 360 spline faces already have exact rectangular trims
and are outside this analytical-curve audit.

## Exact arithmetic and finite domains

`trim_curves.py` converts polynomial B-splines to Bezier pieces by exact rational knot
insertion. Two de Casteljau splits restrict each piece to the actual finite curve interval.
The same polynomial identities handle terminal extensions, including the six tiny imported
interval overruns; neither endpoint snapping nor domain clamping occurs. Rational weights,
discontinuous splines, malformed curves and empty domains are refused.

For each complete curve, one coordinate has derivative Bernstein coefficients of a single
sign on every piece, with at least one strictly signed coefficient per piece. Consequently
that coordinate is strictly monotone, even if its derivative vanishes at isolated points.
Exact continuity between pieces makes this a whole-curve injectivity check. A curve for
which this sufficient condition fails is unresolved, not declared self-intersecting.

For separation, the full Cartesian product of two curves' piece intervals is covered by
Bezier convex-hull boxes. Strict separation along either coordinate accepts a box pair.
Otherwise the larger box's polynomial is bisected, retaining both children. Acceptance
requires exhausting every pending pair; reaching the explicit work budget is unresolved.
There are no sampled crossing tests or floating-point comparisons in these proof decisions.

Tests include an interior crossing, tangential contact, a closed polynomial loop, a retraced
path, disconnected monotone spans, zero-speed endpoints, rational-curve rejection and both
left/right terminal extensions. An insufficient subdivision budget cannot report separation.

## Corners and remaining topology

Only 574 of the 1,188 ordered corners have exactly equal UV endpoints. The other 614 have
small nonzero differences in the extracted binary64 coefficients. The largest absolute
coordinate differences are approximately 1.951e-12 in U and 8.718e-11 in V. These are **raw
parameter differences**, not millimetre distances; cone and sphere charts use different
units. The checker records their exact rational values and does not join or weld them.

The earlier 3D edge/vertex correspondence bounds still apply. Completing the trim argument
requires an explicit treatment of these corners, separation of adjacent curves away from
their shared corner, and periodic chart identifications. UV injectivity also does not prove
that a periodic analytical surface maps the entire trimmed region injectively. Global
material exposure and continuous pair mating remain separate acceptance requirements.

## Reproduction

Using the edge and referenced face inventories from [EDGES.md](EDGES.md):

```sh
python3 experiments/cad-backend/check_trim_curves.py \
  /private/tmp/solvent-cad-edges.json /private/tmp/solvent-cad-trim-curves.json
python3 -m unittest discover -s experiments/cad-backend -p 'test_*.py'
```

The full report records every curve's monotone axis, every wire's pair count and unresolved
pairs, and every corner's unsnapped difference. The committed `trim-curve-results.json`
binds that report and its inputs by SHA-256 and records the scoped results.
