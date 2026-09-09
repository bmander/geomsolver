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
Separate vertex geometry, analytical trim topology, source/reader error and global material
and mating acceptance remain distinct tasks even after edge correspondence passes.
