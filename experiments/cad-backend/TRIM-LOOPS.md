# Closed representatives of analytical trim loops

All 81 analytical faces now have an explicitly constructed simple closed UV representative.
The largest bound on the corresponding boundary's movement on its sphere or cone is
**6.777312644212086e-11 mm**, below the separate 1e-8 mm displacement budget. All 75 faces
without a periodic seam also pass a sufficient injectivity check for their complete
bounded representative region on the analytical surface. The six seam faces remain open.

These are existence and displacement results for a precise representative of the tolerant
boundary. They do not claim that the raw STEP parameter curves have exactly coincident
endpoints, or that an arbitrary STEP reader constructs this representative. The input STEP
files and their extracted curves remain unchanged.

## Construction and displacement

The [raw trim audit](TRIM-CURVES.md) records 614 nonzero UV corner gaps. For each ordered
wire, the new checker takes the exact rational midpoint of the incoming endpoint and the
outgoing endpoint as the shared corner. It changes just the first pole of the outgoing
Bezier piece and the last pole of the incoming piece. All other coefficients remain fixed.
The exact finite polynomial pieces include the imported terminal extensions.

This construction is deterministic, closes every corner exactly, preserves internal piece
joins, and permits direct comparison at corresponding piece parameters. The Bernstein
convex-hull property bounds each coordinate displacement by the largest coefficient change.
It does not assume that moving an endpoint preserves simplicity; the entire resulting
loop is checked again.

For an exactly orthonormal analytical frame, the sphere displacement is bounded by
`R * (delta_u + delta_v)`. A cone is bounded by
`(abs(R) + max_abs_v) * delta_u + delta_v`, using the union of original and changed UV
control hulls for `max_abs_v`. These follow from bounds on the two surface partial
derivatives along the rectangle joining corresponding UV points. The cone bound uses
`abs(sin(angle)) <= 1`; it does not depend on a sampled derivative or kernel tolerance.

## Complete Jordan-loop check

Each changed curve must again have a globally strictly monotone coordinate. All pairs of
non-neighboring curves must be disjoint by exact Bernstein-box subdivision. Neighboring
curves must intersect only at their now-exact common corner.

The adjacent-curve check covers the complete Cartesian product of their finite polynomial
pieces. Away from the shared endpoint, strictly separated convex-hull boxes suffice. At
the endpoint, an exact separating line through the corner must place the two open curve
pieces in opposite open halfplanes. Nonnegative Bernstein projections and a positive
projection at each non-shared endpoint establish this for whole pieces, including tangent
zeros at the common corner. Candidate line selection is a rational heuristic; all control
point projections are subsequently checked exactly. Inconclusive boxes retain both split
children, and exhausted budgets prevent acceptance.

Continuous individual injective arcs, exact cyclic closure, disjoint non-neighbors and
adjacent intersections only at the named corner give a Jordan loop. This argument checks
all curve pairs, rather than excluding small regions around the corners. Negative controls
include retracing, a second intersection between adjacent curves, a bow tie, a constant
piece and unresolved subdivision budgets.

## Mapping the bounded region into 3D

The bounded interior of a Jordan loop lies inside its boundary's coordinate box. For a
sphere, the checker requires that box's latitude remain strictly between `-pi/2` and
`pi/2`. For a cone it requires positive radius and a nonzero cosine of the cone angle.
Together with a U extent strictly below `2*pi`, these are sufficient for injectivity of
the surface parameterization throughout the box. Pi and trigonometric comparisons use
the existing outward rational enclosures.

Seventy-five representative faces pass this check and have no repeated seam edge. Six
faces use one edge twice and require periodic identification:

| Member | Toe sphere | Back cone | Heel sphere |
|---|---:|---:|---:|
| Pinion | face 0, edge 146 | face 146, edge 441 | face 147, edge 442 |
| Gear | face 0, edge 290 | face 289, edge 872 | face 290, edge 874 |

Even a strict rectangle-injectivity result cannot discharge these topological seam
identifications. The checker explicitly leaves them unresolved. It also does not prove
separation between different faces or consistency of all representative face boundaries
as one shared 3D topology. Global swept-material coverage, continuous mating and
source/reader accuracy transfer remain necessary for final matched-pair acceptance.

## Reproduction and results

The small pinion cone with five boundary curves passed in 1.12 seconds. The full final
audit, including chart checks, passed its loop/displacement scope in 23.41 seconds.
The focused experiment suite has 48 passing tests. `trim-loop-results.json` records the
input/report hashes, verifier revision, per-member counts and the remaining seam faces.

```sh
python3 experiments/cad-backend/check_trim_loops.py \
  /private/tmp/solvent-cad-edges.json /private/tmp/solvent-cad-trim-loop-small.json \
  --member 0 --face 1
python3 experiments/cad-backend/check_trim_loops.py \
  /private/tmp/solvent-cad-edges.json /private/tmp/solvent-cad-trim-loops.json
python3 -m unittest discover -s experiments/cad-backend -p 'test_*.py'
```

Filtered runs cannot claim a complete analytical pair. Multiple-wire faces are refused
until a containment audit is provided; every analytical face in this pair has one wire.
