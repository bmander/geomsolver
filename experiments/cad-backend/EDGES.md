# Shared 3D edges and incident-face correspondence

The indexed pair has 1,317 shared edges: 443 on the pinion and 874 on the gear. Every edge
has two oppositely oriented incidences. This audit compares its separate 3D curve with the
parameterized boundary curve on each incident face, over the complete finite edge interval.
It does not infer correspondence from the CAD kernel's `SameParameter` flag.

## Inventory and parameter binding

`export_edges.py` reads both actual indexed STEP files and records all unique 3D curves,
all face incidences, their orientations and parameter curves. The 3D curves comprise
1,158 cubic polynomial B-splines, 154 circles and five lines. Their parameter ranges agree
exactly with both incident parameter-curve ranges in the extracted binary64 representation.

`edge_data.py` checks input hashes, complete edge indices, two opposite incidences per edge,
and equality of all parameter ranges. Each face's incidence-curve multiset must match its
previously extracted complete wire inventory, including repeated seam uses. Unknown faces,
omitted curves, duplicate edges, changed ranges and changed reference inputs are refused.
This associates the new edge evidence with the already bounded faces and finite domains.

## Three correspondence methods

All 1,446 B-spline-face incidences are natural isoparametric boundaries. On a clamped
surface, the boundary curve is exactly the first or last row/column of its control net.
The adapter accounts for affine parameter changes and reversals. The 3D curve and extracted
boundary must have identical degrees, knots and unit weights. Their largest pole-difference
norm bounds the entire curve difference by the nonnegative partition-of-unity property.
The complete direct trial bounds every such incidence within 1.56e-9 mm.

When a line or circle on an analytical face has a line parameter curve, the checker factors
out the shared elementary functions. Equal-frequency circles are expressed as a constant
plus cosine and sine coefficient vectors; the sum of coefficient-error norms bounds their
difference for every angle. Constant-meridian cone lines use an affine error bound over
the finite interval. This avoids refining long circular edges solely because separate
interval sine/cosine evaluations lose their common dependence.

The remaining analytical-face incidences use a cubic Taylor correspondence bound with a
fourth-derivative remainder. Exact knot insertion and polynomial finite differences prepare
the 3D and 2D spline pieces. The initial partition includes every knot of both curves and
the actual edge endpoints, including terminal polynomial extensions. For each cell, the
checker evaluates `E(t) = curve3d(t) - surface(parameter_curve(t))` and uses

```text
|E_i(t)| <= |E_i(c)| + |E_i'(c)| h + |E_i''(c)| h²/2
             + |E_i'''(c)| h³/6 + sup_cell |E_i''''| h⁴/24.
```

The norm of the component bounds must be at most **0.000002 mm**. This separate edge
allowance fits inside the 0.001 mm target when added to the earlier nominal face bounds.
A center-distance lower bound above the allowance is a definite correspondence failure.
An inconclusive cell retains both halves; exhausting the explicit cell budget retains
all unresolved intervals and prevents complete acceptance.

`curve_taylor.py` stores derivative/factorial coefficients for one parameter, using the
existing outward rational arithmetic. Its sine/cosine recurrence follows `s'=c*x'` and
`c'=-s*x'`. Integer multiples of a rigorously enclosed pi reduce angles before the shared
transcendental evaluator is called; parity restores the original sine/cosine signs.
Unknown higher Taylor coefficients cannot be silently invented by mixing different orders.
No additional numerical dependency enters the Solvent runtime.

Successful adaptive checks audit an exact, ordered cover from the original finite start
to end parameter. There is no floating conversion in the one-dimensional coverage check.
Partial edge selections and interrupted/budget-limited runs cannot claim complete members.

## Complete pair results

Both complete member audits pass, with no unresolved intervals, violating centers or
refinement refusals. [Recorded results](edge-results.json) retain input, verifier and raw
report hashes, along with the endpoint and vertex-link evidence. A separate aggregation
check confirms that every edge/side pair appears exactly once and rechecks all accepted
one-dimensional parameter covers.

| Member | Edge/face incidences | Adaptive cells checked | Maximum correspondence bound (mm) | Audit time (s) |
|---|---:|---:|---:|---:|
| Pinion | 886 | 16,321 | 1.997e-6 | 517.1 |
| Gear | 1,748 | 26,310 | 2.000e-6 | 896.5 |

The full audits ran as two concurrent processes. These are offline correspondence proofs;
small selected-edge checks take fractions of a second to a few seconds, and the focused
unit suite remains below one second. Initial experiments measured quadratic through
quintic Taylor bounds; the cubic bound balances refinement against per-cell arithmetic.

Adding each member's worst nominal face bound, edge bound and endpoint/vertex bound gives
conservative distance chains of 0.000999090 mm for the pinion and 0.000975157 mm for the gear,
still below 0.001 mm. This relates the nominal supports to the exported boundary
representations; it does not prove global material exposure or geometric embedding.

The distorted-edge control is rejected after one adaptive cell, with a proved center
correspondence error greater than 0.00124995 mm.

## Controls and reproduction

Small complete-edge trials pass on both members, including the pinion face with the largest
native sampled correspondence residual. The focused tests cover polynomial derivatives
and end extensions, affine/reversed boundary parameters, exact circle factorization,
Taylor products and trigonometric composition through sixth order, a separate long-series
trigonometric reference, and known spatial correspondence errors. Moving an actual exported
shared-edge pole by 0.01 mm produces a proved center-distance failure.

```sh
/private/tmp/solvent-occt-env/bin/python experiments/cad-backend/export_edges.py /private/tmp/solvent-cad-indexed-coefficients.json /private/tmp/solvent-cad-analytic-faces.json /private/tmp/solvent-cad-edges.json
python3 experiments/cad-backend/check_edges.py /private/tmp/solvent-cad-edges.json /private/tmp/solvent-cad-edge-trial-0.json --member 0 --edges 1 2 3 --max-cells 512
python3 experiments/cad-backend/check_edges.py /private/tmp/solvent-cad-edges.json /private/tmp/solvent-cad-edge-audit-0.json --member 0 --max-cells 2048
python3 experiments/cad-backend/check_edges.py /private/tmp/solvent-cad-edges.json /private/tmp/solvent-cad-edge-audit-1.json --member 1 --max-cells 2048
python3 -m unittest discover -s experiments/cad-backend -p 'test_*.py'
```

The complete member runs are not accepted until their final reports say `status: verified`
and `complete_selected_members: true`, and both member inventories are accounted for.
Analytical trim geometry, source/reader error and global material and mating acceptance
remain distinct tasks even after edge correspondence passes.

## Endpoint geometry and combinatorial vertex links

`export_vertices.py` reads all 876 actual topological vertices and associates both ends of
every shared edge with their named vertices, respecting intrinsic edge-parameter direction.
`check_vertices.py` evaluates each independent curve representation at both finite endpoints.
All 2,634 endpoint distances are bounded by 1.56e-8 mm; all vertices are accounted for.
The check includes closed edges whose two parameter endpoints name the same vertex.

The oriented incidence data also reconstructs every face's ordered wire from its previously
exported parameter curves. Each adjacent edge pair must meet at the same named vertex.
For each vertex, face corners connect incident edge ends into a link graph. Every node must
have degree two and the graph must be connected: one cycle. Reusing a vertex to join two
otherwise closed shells would produce two disconnected cycles and is refused. All 441
face wires close through their named vertices, and all 876 vertex links pass. These are
combinatorial checks; they do not assert that the curved boundaries are geometrically simple
or that the resulting surface is embedded.

The full endpoint and link audit takes approximately 15 seconds. A vertex displaced by
0.01 mm is rejected. Tests include an exact endpoint distance, the distinct ends of a closed
edge, and two separate closed fans incorrectly sharing one vertex.

```sh
/private/tmp/solvent-occt-env/bin/python experiments/cad-backend/export_vertices.py /private/tmp/solvent-cad-edges.json /private/tmp/solvent-cad-vertices.json
python3 experiments/cad-backend/check_vertices.py /private/tmp/solvent-cad-vertices.json /private/tmp/solvent-cad-vertex-audit.json
```
