# Whole-domain generated-fillet correspondence

The shared scripts are now `export_generated.py` and `check_generated.py`; the
original full-fillet evidence was produced at commit `9e6e610` before those renames.

The four generated root fillets have a direct parameter rectangle: spherical face-width
fraction and generating-arc fraction. Unlike the working flanks, their parameter endpoint
does not require a moving tip-trim solve. This audit compares the actual STEP fillet support
surfaces with the independent common-crown parameterization over that complete rectangle.

## Reference and exported coefficients

`export_cad_fillet_references` reads the solved Solvent reference. For each generating arc
it obtains interval enclosures of the pre-revolution position and first two derivatives
at parameter zero. With the declared arc sweep `omega`, its circular coefficients satisfy

```text
profile(u) = center + a*cos(omega*u) + b*sin(omega*u)
a = -profile''(0)/omega²
b = profile'(0)/omega
center = profile(0)-a
```

Every coefficient remains an interval; no midpoint replaces an enclosure. The export also
records the member, side, arc direction, tooth-space index and finite generating-roll span.
The source circle/sphere intersection selects the same common-crown branch as the existing
independent characteristic equations. `crown_fillet.py` then applies crown rotation and the
inverse member rotation, using the exact integer tooth ratio. It checks the selected
semicircle, nonzero divisors and declared roll domain on every accepted parameter box.

The material definition remains the complete indexed swept subtraction. A local envelope
correspondence bound does not by itself establish which parts remain exposed after all cuts.

`export_generated.py` reads the actual finer tooth-space STEP files and extracts their B-spline
degrees, knots, poles and weights. It matches the four fillet faces against the original
source interpolation data and verifies accepted input hashes. The independent checker binds
the source, reference and STEP hashes, and compares nine reference positions per fillet with
the original finer source grid before attempting a surface bound.

## Quadratic Taylor bound with a third-derivative remainder

Exact rational knot insertion partitions each exported B-spline into 256 Bézier patches.
The present checker requires polynomial surfaces (all weights exactly one), as found in
these exports. It refuses unsupported rational weights. Exact de Casteljau subdivision
preserves each patch and its parameter domain during refinement.

Let `E = CAD - reference` at common parameters. On a parameter cell centered at `c`, with
half-widths `h_u, h_v`, the componentwise quadratic Taylor bound is

```text
|E_i| <= |E_i(c)|
       + |E_i,u(c)| h_u + |E_i,v(c)| h_v
       + |E_i,uu(c)| h_u²/2 + |E_i,uv(c)| h_u h_v + |E_i,vv(c)| h_v²/2
       + M_uuu h_u³/6 + M_uuv h_u² h_v/2
       + M_uvv h_u h_v²/2 + M_vvv h_v³/6
```

Each `M` bounds the corresponding third derivative of **the difference**, throughout the
cell. CAD derivatives come from exact Bernstein coefficient differences; their convex hulls
enclose the whole cell. The reference is differentiated by bivariate interval jets. The
Euclidean norm of the three component bounds must be at most 0.001 mm. A center-distance
lower bound exceeding that target is a definite correspondence failure, not a request for
endless refinement. Otherwise inconclusive cells retain both children until proved or the
explicit work budget is exhausted.

The interval arithmetic rounds outward to an 80-bit dyadic grid; it does not assume a
floating-point transcendental error allowance. Sine and cosine use degree-25-or-lower Taylor
polynomials with explicit remainder bounds on `[-4,4]`. Arctangent uses a half-angle reduction
and a degree-41 alternating polynomial with a remainder on the reduced argument. Square
roots use the existing independent integer-square-root enclosure. Fixed coefficients have
zero parameter derivatives, so the jet evaluator avoids unnecessary derivative arithmetic.

Two algebraic changes reduce interval overestimation without changing the reference:
compute meridian radius as `dx*sqrt(1+(dy/dx)²)` on the proved positive-`dx` branch, and cancel
the common radial derivative from the characteristic normal before bounding its roll ratio.
The cancelled divisor must remain nonzero. A linear Taylor bound was measured first; the
quadratic bound reduced representative whole-patch bounds by roughly 6–23 times.

Each successful run checks an exact, gap-free slab cover of every original Bézier rectangle
using the existing independent cover checker. Its binary64 adapter is used only after
verifying that every rational partition endpoint is represented exactly. Partial selections,
budget exhaustion and interrupted runs cannot claim a complete surface.

## Completed bounds

All four complete fillet support surfaces pass the 0.001 mm target. The
[recorded results](fillet-results.json) retain source, coefficient, STEP, verifier and raw
report hashes. Every run has no unresolved cells or violating centers; its accepted cells
also pass the independent exact cover check across the complete parameter square.

| Member / side | Cells checked | Accepted cells | Bound target (mm) | Audit time (s) |
|---|---:|---:|---:|---:|
| Pinion / 0 | 576 | 416 | 0.001 | 85.2 |
| Pinion / 1 | 556 | 406 | 0.001 | 82.1 |
| Gear / 0 | 556 | 406 | 0.001 | 81.3 |
| Gear / 1 | 576 | 416 | 0.001 | 86.0 |

These times cover refinement and coverage verification while the four audits ran
concurrently; source loading and the nine-point reference comparison precede that timer.
The reference comparisons are bounded by 3.6e-13 mm at their sampled positions. They are
association checks, not a source-solve error certificate.

The distorted pinion coefficient net fails after one cell, with a 0.0352578 mm lower bound
on its center correspondence error. The earlier linear-Taylor whole-surface runs were
explicitly interrupted after the improved method passed its small controls. Their partial
reports remain as `solvent-cad-fillet-bound-*-*-second-order-partial.json` under `/private/tmp`;
they are not accepted surface certificates. No geometric target was relaxed.

## Controls and reproduction

Tests compare reciprocal, square-root, trigonometric and arctangent derivatives through
third order with independent formulas, including mixed derivatives. A longer Taylor
reference and Machin's identity check the transcendental enclosures. An exact paraboloid
tests the Taylor bound and parameter scaling through subdivision; an interior distortion
must fail even though corner samples are unchanged. A deliberately distorted coefficient
net from the real pinion fillet is also a negative control. This tests the mathematical
coefficient audit, not detection of tampering by the STEP reader.

```sh
SOLVENT_CAD_FILLETS_OUTPUT=/private/tmp/solvent-cad-fillet-references.json cargo test --manifest-path rust/Cargo.toml export_cad_fillet_references -- --ignored --nocapture
/private/tmp/solvent-occt-env/bin/python experiments/cad-backend/export_generated.py /private/tmp/solvent-cad-sections.json /private/tmp/solvent-cad-fillet-references.json /private/tmp/solvent-cad-one-cut-checked /private/tmp/solvent-cad-fillet-coefficients.json
python3 experiments/cad-backend/check_generated.py /private/tmp/solvent-cad-fillet-coefficients.json /private/tmp/solvent-cad-fillet-bound-0-0.json --member 0 --side 0 --max-cells 8192
python3 -m unittest discover -s experiments/cad-backend -p 'test_*.py'
```

Repeat the bound command for members 0/1 and sides 0/1. For a short iteration, add
`--patches 1`; its report explicitly excludes complete-surface acceptance. The raw report
retains all accepted parameter cells, upper bounds, unresolved cells and any violating
center witness. Full results require `status: verified` and `complete_surface: true`.

The bound is a parameter correspondence for the extracted binary64 support surface against
the ideal common-crown construction with enclosed solved coefficients. Source-solve and
assembly-frame error, STEP-reader conversion, finite trimmed-face coverage, exposed swept
material, subsequent indexed CAD operations and working-flank accuracy remain separate.
