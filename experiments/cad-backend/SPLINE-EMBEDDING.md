# Whole-face embedding of the indexed spline surfaces

All 360 polynomial spline faces in the actual indexed STEP pair pass an exact rational
injectivity and differential-regularity check on their complete parameter rectangles.
This includes 120 pinion faces and 240 gear faces: generated working flanks, generated
root fillets and root closures. Their finite trims are checked against the same complete
rectangles. The proof uses the imported coefficients directly, including each indexed
copy's rounding differences; it does not transfer injectivity from a visually similar
reference face or from a distance bound alone.

Together with the [analytical representatives](TRIM-SEAMS.md), this establishes individual
embedding for every face in the pair. It does not establish that different faces meet only
at their intended common boundaries, or that all tolerance-adjusted analytical boundaries
and spline boundaries realize one shared 3D topology.

## One fixed projection per complete face

For a surface `S(u,v)`, two secants of its control net choose a constant linear projection
`A` from three coordinates to two. The checker forms the left inverse of those secants
with exact rational Gram-matrix arithmetic. They are only a way to choose a candidate;
their independence does not itself prove surface injectivity.

The projected surface `G = A*S` has a 2-by-2 Jacobian. Each derivative is bounded over the
entire clamped rectangle using its exact B-spline derivative control coefficients:

```text
dP_i = degree * (P_(i+1)-P_i) / (knot_(i+degree+1)-knot_(i+1)).
```

The derivative basis is a nonnegative partition of unity, so coefficient extrema enclose
each projected derivative everywhere. There are no sampled Jacobians or finite-difference
approximations. Non-unit weights, discontinuous bases, malformed control nets and
nonpositive derivative knot spans are refused.

For interval enclosures of the entries `a,b,c,d` of the Jacobian, the checker requires

```text
lower(a) > 0,
lower(d) > 0,
4*lower(a)*lower(d) > max_abs(b+c)^2.
```

These prove that the symmetric part of the Jacobian is positive definite at every point.
If the initial projection does not satisfy this sufficient condition, one positive
constant scaling of its second output balances the off-diagonal bounds. The scaling is
selected once from the whole-domain bounds and applied to the complete face. The checker
does not change projections between cells, which would invalidate the global argument.
Failure remains unresolved rather than being reported as a self-intersection witness.

## Why this proves global injectivity

For distinct parameter points `p,q` in the convex rectangle, let `h = p-q`. The fundamental
theorem of calculus along their connecting segment gives

```text
h dot (G(p)-G(q)) = integral_0^1 h^T J_G(q+t*h) h dt > 0.
```

The skew-symmetric part contributes zero to the quadratic form. Thus `G(p)` and `G(q)`
cannot agree, and consequently neither can `S(p)` and `S(q)`. This rules out distant
self-intersections as well as local folds. The same Jacobian condition implies that the
surface derivative has rank two on each polynomial span. The complete audit additionally
checks that the knot multiplicities guarantee at least C1 continuity, so this regularity
claim includes the interior knot lines. The lower-level helper can establish injectivity
for C0 bases, but reports full-rectangle regularity as unproved in that case.

The exact unit-square trim checker validates every boundary segment, including split
sides, their order and closure. That trim rectangle must equal the spline's full clamped
domain; the checker cannot apply a proof to undeclared polynomial continuations.

## Controls and reproduction

The small trial covers all ten reference roles, one copy on each member, before the
complete 360-face audit. Tests include a saddle-shaped embedded graph, a graph requiring
one global output scaling, non-unit knot spans, a cusp and a regular self-intersection.
The latter uses `(t^2-1, v, t^3-t)` over `-1.5 <= t <= 1.5`: its tangent never vanishes,
but `t=-1` and `t=1` produce the same point. It is not accepted. Other controls refuse a
wrong trim domain, discontinuous basis and rational weights. The focused suite has 61
passing tests.

```sh
python3 experiments/cad-backend/check_spline_embedding.py \
  /private/tmp/solvent-cad-edges.json /private/tmp/solvent-cad-spline-embedding-small.json \
  --index 0
python3 experiments/cad-backend/check_spline_embedding.py \
  /private/tmp/solvent-cad-edges.json /private/tmp/solvent-cad-spline-embedding.json
python3 -m unittest discover -s experiments/cad-backend -p 'test_*.py'
```

The full report records each face's exact projection, output scaling, Jacobian intervals,
positive-definiteness margins and trim domain. `spline-embedding-results.json` binds those
results and the earlier analytical-face evidence to the same original source and STEP
files. Filtered runs cannot claim complete indexed-spline coverage. Inter-face embedding,
shared topology realization, global swept-material coverage, continuous mating and
source/reader accuracy transfer remain separate acceptance requirements.
