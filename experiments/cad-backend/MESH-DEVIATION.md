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
On refusal, reported cell bounds describe the unresolved cells only. No whole-face
or whole-mesh maximum is reported unless all of its triangles pass.

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

## Finite rectangular faces

The complete gear meshes have 360 polynomial faces whose natural domains are
unit squares. `export_mesh_face_domains.py` rereads their actual STEP wires and
requires exact coefficient identity with the mesh parameter input. The independent
`check_mesh_face_domains.py` reuses the exact rectangle checker: each finite wire
must cover all four sides once, close as a directed cycle, and have no holes.
It also checks all original mesh parameter vertices against the same unit square.

The triangle-bound evaluator refuses spline evaluation outside its natural knot
domain, and clamps proposed witnesses to that domain. Convex combinations of
accepted witnesses stay in the unit square. Consequently, for these faces, the
already checked forward triangle-to-support distance is also a forward distance
to the actual **finite face**. No distance report is trusted or recomputed by the
domain checker: this implication combines two separately verified results bound
to the identical mesh parameter input.

```sh
python experiments/cad-backend/export_mesh_face_domains.py mesh-parameters.json mesh-domains.json
python3 experiments/cad-backend/check_mesh_face_domains.py mesh-domains.json domain-result.json
```

An optional `--reference indexed-coefficients.json --member 0` checks exact
coefficient and natural-domain identity with an earlier indexed reference. It does
not verify or enlarge any previous reference error claim. Reference reports retain
their original source-solve, finite-coverage and material limitations.

The [finite-domain result](mesh-face-domain-results.json) covers all 120 pinion
spline faces (73,706 triangles) and 240 gear spline faces (105,087 triangles).
The checks took 2.5/3.7 seconds including reference identity. All remaining 81 faces
are explicitly listed as unverified here. A triangle-to-face bound is one-way:
reverse face coverage, analytical curved trims and the remaining nominal-source
error transfer still require separate treatment.
