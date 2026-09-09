# Whole-triangle distance to supporting surfaces

The export accuracy target is 0.001 inch (0.0254 mm). Sampling triangle centroids
cannot establish maximum error between samples. This checker bounds every point
of each existing encoded STL triangle against its associated CAD **supporting
surface**, using a default 0.02 mm budget. The remaining 0.0054 mm is unallocated
headroom, not an established allowance for every other error source.

This is one part of export acceptance. It does **not** establish that the chosen
surface points lie inside the finite face trims, that the mesh covers those trims
in the reverse direction, or that the CAD surfaces match the nominal generating
geometry. Those questions must be combined with this result before claiming the
end-to-end target. In particular, a polygonal hole in a plane can have zero
support-distance error while still differing from a curved trimmed boundary.

## Binding the actual exported mesh

`export_mesh_parameters.py` checks the STEP/STL hashes against the tessellation
metadata, remeshes the STEP using the same settings, and matches every triangle
against the actual binary32 STL coordinates. It preserves cyclic orientation,
refuses duplicates or omitted triangles, and exports the surface coefficients,
UV coordinates and mesh node coordinates for each face. File order is not assumed.
`check_mesh_deviation.py` independently repeats the exact triangle binding and
requires complete coverage of the encoded triangle inventory. It has no CAD-kernel
dependency. The STEP reader and coefficient/UV extraction remain part of the
input trust boundary; the bound does not certify arbitrary STEP reader behavior.

## Curvature bound

For a C1 parameter surface F with bounded piecewise second derivatives on a
rectangle containing the UV triangle, let a, b, c bound the Euclidean norms of
F_uu, F_uv, F_vv. Let Δu and Δv be that triangle's coordinate spans. At any
barycentric combination q of its three parameter vertices q_i, Taylor's theorem
about q and cancellation of the first derivative give

```
|F(q) - sum_i lambda_i F(q_i)|
  <= (a Δu² + 2 b Δu Δv + c Δv²) / 8.
```

Each coordinate's weighted variance is at most its squared span divided by four;
Cauchy–Schwarz bounds the mixed absolute moment by Δu Δv / 4. Adding the largest
encoded-vertex displacement |x_i-F(q_i)| therefore bounds the distance from every
point of the flat triangle to some point on the supporting surface.

Polynomial B-splines are partitioned into exact rational Bézier rectangles.
Derivative control hulls give the three curvature bounds, and exact polynomial
evaluation bounds each encoded vertex displacement. Rational and non-C1 splines
are refused by this implementation. Planes have zero second derivatives; cone
bounds use their explicit equations and the norm of the encoded coordinate frame.
Transcendental evaluations use the existing outward dyadic interval arithmetic.

If a triangle's bound is too large, its *proof domain* is divided into four flat
subtriangles. Their XYZ vertices remain exact barycentric combinations of the
original STL vertices. New parameter witnesses are proposed by a damped local
projection, then independently checked by exact/interval surface evaluation.
The projection's convergence is never accepted as an error bound. Each child's
curvature bound covers its complete interior; the children cover the unchanged
original triangle. Failure or exhaustion of the subdivision limit is a refusal.
The piecewise witness maps need not provide a continuous reverse parameterization.

## Spheres and polar charts

Linear interpolation of a sphere's UV coordinates is unsuitable near its poles:
several parameter longitudes can represent the same pole. For spheres the checker
instead computes the exact squared distance from the centre to the triangle,
including edge and degenerate cases, and the greatest vertex radius. Convexity
bounds the greatest radius over the whole triangle. Outward square roots enclose
the resulting radial error.

If B contains the encoded frame axes, a bound δ < 1 on the spectral norm of
BᵀB-I bounds every singular value's distance from one by δ. Adding radius*δ
therefore accounts for its departure from an exactly orthonormal frame. No
parameter-chart choice or polar snapping enters this distance bound.

## Commands and small cases

```sh
python experiments/cad-backend/export_mesh_parameters.py INPUT.step INPUT.stl mesh-parameters.json
python3 experiments/cad-backend/check_mesh_deviation.py mesh-parameters.json mesh-bound.json --tolerance-mm 0.02
python3 -m unittest discover -s experiments/cad-backend -p 'test_mesh_deviation*.py'
```

Use the CAD-enabled interpreter for the exporter. The tests include a plane with
displaced encoded vertices, a polynomial whose corners are exact but whose
interior bulges, sphere polar/seam cases, a chord through a sphere's centre, and
unsupported rational/low-continuity splines. Native cube, sphere and tooth-space
fixtures exercise actual STL binding before complete gear audits.
