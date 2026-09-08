# Whole-patch closure support bounds

The finer tooth-space candidates have six closure faces each: root and tip, plus toe/heel
faces split at the flank/fillet join. Their intended supports are cones and spheres.
The [recorded audit](closure-results.json) bounds **every parameter point** on these twelve
exported B-spline support surfaces against those nominal supports. It does not merely
sample the patches. This addresses one part of CAD accuracy; it is not full gear acceptance.

## Inputs and binding

The ignored Rust test `export_cad_boundary_supports` reads the solved default 24:48,
module-2 reference. It exports the tip/root/back cone meridians in member-local coordinates
and the nominal toe/heel sphere radii. The original CAD source's tooth counts and blank
meridians must agree exactly with this separately read support export.

`export_supports.py` reads the actual finer tooth-space STEP files, checks their hashes
against the accepted candidate reports, and identifies the six closures by midpoint
correspondence with the original interpolation data. Matching must be unique and within
1e-7 mm. It extracts the **actual** B-spline degrees, complete knot vectors, poles and weights
through the STEP reader. It does not reconstruct a new fitted surface for verification.

`check_supports.py` runs without Open CASCADE or Rust. It verifies file hashes, reference
agreement, complete member/role coverage and each closure's designated support. It then
interprets the exported binary64 coefficients as exact rational numbers. All knot insertion
and polynomial arithmetic use Python fractions. Partial selections are explicitly reported
as incomplete closure coverage; an interrupted multi-face run cannot claim completion.

## Bound construction

Positive rational weights and clamped, nonperiodic knot vectors are required. Repeated
exact knot insertion raises each interior knot multiplicity to its degree. The resulting
tensor Bézier patches partition the entire original parameter rectangle. Homogeneous poles
are `(w*x, w*y, w*z, w)`; no floating-point Bézier conversion is trusted.

On each patch let the Bernstein polynomials be `X, Y, Z, W`, so its point is
`(X/W, Y/W, Z/W)`. The basis functions are nonnegative and sum to one. Thus every polynomial
value lies between its smallest and largest Bernstein coefficients. Products are formed
exactly using the binomial identity for products of Bernstein basis functions, applied in
both parameter directions. There is no sampled derivative estimate or assumed error band.

For an origin-centered sphere of radius `r`, form

```text
F = X² + Y² + Z² − r² W²
distance to sphere = |F| / [W² (norm(point) + r)]
                  <= max_absolute_coefficient(F) / [min_coefficient(W)² r]
```

For a cone meridian `A*radial − B*z − C = 0`, form

```text
L = B Z + C W
F = A² (X² + Y²) − L²
```

Choose the common sign so `A > 0`, and require a strictly positive Bernstein lower bound
for `L`. These conditions select the positive-radius cone branch; squaring cannot silently
accept the opposite nappe. Orthogonal projection onto the meridian line has positive
radius because its radial numerator is `B²*radial + A*(B*z+C) > 0`. It is therefore a point
on the intended cone. Factoring the squared residual gives the conservative bound

```text
distance to cone <= max_absolute_coefficient(F)
                  / [min_coefficient(W) min_coefficient(L) max(|A|, |B|)]
```

Here `sqrt(A²+B²) >= max(|A|,|B|)` bounds the meridian-normal length, and the remaining
positive conjugate factor is at least `L`. All numerators, lower bounds and comparisons
are rational. Only reporting converts a result to binary64, rounding upward if necessary.

## Results and controls

There are 256 Bézier patches per closure, 3,072 total. All twelve whole-patch distance
bounds pass the 0.001 mm target, taking approximately 26 seconds in the standalone checker.

| Closure | Pinion upper bound (mm) | Gear upper bound (mm) |
|---|---:|---:|
| Root | 0.000003889 | 0.000001094 |
| Tip | 0.000006246 | 0.000001383 |
| Toe, fillet portion | 0.000008193 | 0.000000733 |
| Toe, flank portion | 0.000001013 | 0.000000930 |
| Heel, fillet portion | 0.000003178 | 0.000000385 |
| Heel, flank portion | 0.000000999 | 0.000001030 |

The geometric tests use exact rational sphere and cone patches, an interior bulge whose
corners remain unchanged, and an opposite-nappe cone. Nonuniform knot-insertion tests
compare both coordinates and complete parameter coverage against an independently known
rational sphere parameterization. Invalid knot vectors, nonpositive weights and malformed
control nets are refused. Binding tests reject changed supports, changed source files,
missing closures and duplicate member provenance, and preserve honest partial reports.

## Reproduction and remaining scope

```sh
SOLVENT_CAD_SUPPORTS_OUTPUT=/private/tmp/solvent-cad-supports.json cargo test --manifest-path rust/Cargo.toml export_cad_boundary_supports -- --ignored --nocapture
/private/tmp/solvent-occt-env/bin/python experiments/cad-backend/export_supports.py /private/tmp/solvent-cad-sections.json /private/tmp/solvent-cad-supports.json /private/tmp/solvent-cad-one-cut-checked /private/tmp/solvent-cad-closure-coefficients.json
python3 experiments/cad-backend/check_supports.py /private/tmp/solvent-cad-closure-coefficients.json /private/tmp/solvent-cad-closure-bounds.json
python3 -m unittest discover -s experiments/cad-backend -p 'test_*.py'
```

For a small iteration, append `--faces 0_toe_round` to the checker; that one-face audit takes
about two seconds and explicitly does not claim complete closure coverage.

These are one-way distances from complete **tooth-space closure support surfaces** to
infinite nominal spheres/cones. They do not prove the reverse coverage of a finite trimmed
face, source-solve accuracy, decimal STEP-to-binary64 reader conversion error, propagation
through subsequent indexed Booleans/STEP writes, or generated flank/fillet surface error.
The very small closure residuals must not be reported as whole-gear production accuracy.
