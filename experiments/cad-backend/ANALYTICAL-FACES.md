# Remaining analytical faces in the indexed solids

The [indexed B-spline audit](INDEXED-SURFACES.md) covers 360 faces. The other 81 faces are
four spherical toe/heel faces and 77 conical tip/back faces. This audit bounds each of
those complete supports against the solved nominal blank, over a finite parameter slab
containing every imported trim curve.

## Finite parameter slabs

`export_analytic_faces.py` inventories the remaining faces from the same hashed indexed
STEP files. It exports their origins, frames, radii, cone angles and every wire's ordered
parameter curves. The actual curves are lines or clamped polynomial B-splines of degrees
3–10. The adapter checks that the wire explorer visits every edge. The independent
checker binds the indexed inventory, STEP files, original source and solved blank supports.

`parameter_bounds.py` bounds lines exactly from their endpoints and splines by their
control-point convex hulls. The spline degree, knots, endpoint multiplicities, dimensions
and unit weights are checked before using the polynomial partition-of-unity property.
Rational and unsupported parameter curves are refused. The coordinate extrema of all
boundary curves enclose every bounded face-interior component; the conical support bound
is valid throughout that entire slab and for every revolution angle.

Six imported edge ranges extend slightly past their stored end knots. They are not clamped
or silently shortened. Exact knot insertion obtains each terminal Bézier polynomial, and
outward interval de Casteljau evaluation encloses its continuation over the excess range.
The checker retains both the original curve hull and the extension bounds. This follows
the pinned OCCT 7.9.3 evaluation convention: the nonperiodic parameter remains unchanged
while span selection chooses an end span, then the spline evaluator uses that parameter.
See [Geom2d_BSplineCurve::D0](https://github.com/Open-Cascade-SAS/OCCT/blob/V7_9_3/src/Geom2d/Geom2d_BSplineCurve_1.cxx#L157-L174)
and [BSplCLib::LocateParameter](https://github.com/Open-Cascade-SAS/OCCT/blob/V7_9_3/src/BSplCLib/BSplCLib.cxx#L207-L286).

This establishes a conservative parameter slab. It does not independently prove that the
curved wires are simple, agree with their separate 3D edges, or enclose the intended
material region. Those topological and material claims remain separate.

## Support bounds

All four exported sphere frames are exactly orthonormal as rational binary64 data. For
actual center `c` and radius `r`, and the nominal origin-centered radius `R`, every point
has support distance at most `norm(c) + abs(r-R)`. Frame orthonormality and positive radii
are checked explicitly; no floating-point orthogonality tolerance substitutes for them.

All 77 cones have the exact canonical member-coordinate frame and an origin on the z axis.
For their finite curve-derived interval `v`, the cylindrical meridian is

```text
r(v) = reference_radius + v*sin(angle)
z(v) = origin_z + v*cos(angle).
```

The nominal solved meridian line is `A*r - B*z = C`. Its distance numerator is affine in v:

```text
A*reference_radius - B*origin_z - C + v*(A*sin(angle)-B*cos(angle)).
```

Outward rational trigonometric bounds enclose that expression. Dividing its largest
absolute value by a lower bound on `sqrt(A²+B²)` gives the support-distance bound.
The projected meridian radius must stay positive throughout the slab, so the closest
point lies on the intended cone branch. Unsupported frames and branches are refused.
Each face must match exactly one nominal toe/heel sphere or tip/back cone within 0.001 mm.

## Results and controls

[Recorded results](analytic-face-results.json) retain input, STEP, verifier and raw-report
hashes. All 81 faces pass. The complete audit checks 1,188 parameter curves and explicitly
bounds six terminal polynomial extensions in approximately 0.85 seconds.

| Member | Sphere faces | Cone faces | Maximum nominal support bound (mm) |
|---|---:|---:|---:|
| Pinion | 2 | 27 | 1.096e-11 |
| Gear | 2 | 50 | 1.354e-12 |

Together with the 360 indexed B-splines, every one of the pair's 441 faces now has a
nominal support-distance bound below 0.001 mm. For the generated B-splines the earlier
result is stronger parameter correspondence; for the blank and root closures it remains
a distance to an infinite analytical support. These are not interchangeable claims about
finite target coverage or material exposure.

Controls use a cubic with known values outside its original knot interval, piecewise
linear terminal spans, a translated sphere with an attained exact distance, and a cone
with a known meridian offset. Changed bases, rational trim curves, malformed knot ranges
and unsupported frames are refused. Increasing an actual exported sphere radius by
0.01 mm is rejected. The arithmetic tests retain their existing independent references;
none of these controls certifies native STEP-reader conversion accuracy.

```sh
/private/tmp/solvent-occt-env/bin/python experiments/cad-backend/export_analytic_faces.py /private/tmp/solvent-cad-indexed-coefficients.json /private/tmp/solvent-cad-supports.json /private/tmp/solvent-cad-analytic-faces.json
python3 experiments/cad-backend/check_analytic_faces.py /private/tmp/solvent-cad-analytic-faces.json /private/tmp/solvent-cad-analytic-face-audit.json
python3 -m unittest discover -s experiments/cad-backend -p 'test_*.py'
```

Separate 3D edge consistency, analytical trim topology, source and assembly-frame error,
STEP-reader conversion, global material coverage and continuous mating remain acceptance
work. No production-pair completion is asserted by this support audit.
