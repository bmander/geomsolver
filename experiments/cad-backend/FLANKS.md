# Generated working-flank correspondence

The working flanks use the same common-crown characteristic and member transforms as the
[root fillets](FILLETS.md). Their extra complication is a moving tip trim: the end of the
straight generating profile depends on the spherical face-width coordinate. The independent
checker isolates that endpoint and differentiates its defining equation, rather than
interpolating sampled trim parameters or using finite differences.

## Tip equation

`export_cad_flank_references` records interval enclosures of the solved straight meridian's
start and derivative, its generating-junction parameter, and the solved tip-cone meridian
in member coordinates. The cone coefficients are nominal exact binary64 inputs; the
export does not enclose the original solve or the coordinate transform's numerical error.

Write the tip meridian as `A R - B Z = C`, with `A > 0`, cylindrical radius `R > 0`,
and sphere radius `rho`. The selected cone-sphere intersection has axial height

```text
Ztip(rho) = (-B*C + A*sqrt((A²+B²)*rho²-C²))/(A²+B²).
```

The evaluator checks the positive square-root and radial branches. For a source-line
parameter `T`, the common-crown characteristic gives a generated member height `Z(q,T)`,
where `q` is normalized face width and `rho(q)` is affine. The tip equation is
`F(q,T) = Z(q,T) - Ztip(rho(q)) = 0`.

On the proved positive characteristic branch, `Z(q,T)` can be computed algebraically,
without trigonometric roll evaluation. With crown center `(cx,cy)`, source meridian
radius/height `(r,h)`, derivatives `(dr,dh)`, and revolution direction `(ct,st)`, put

```text
offset = h*dh/dr
P = rho²-h²
dot = r+cx*ct+cy*st
Xrotated = (P+offset*dot)/sqrt(P+2*offset*dot+offset²)
Z = (other_teeth*Xrotated + member_sign*teeth*h)/sqrt(teeth²+other_teeth²).
```

`member_sign` is positive for the pinion and negative for the gear. This is the axial
component of the same crown/member rotations used to evaluate the final flank. Nonzero
divisors, source semicircle and characteristic branch are checked by interval arithmetic.

## Root isolation and derivatives

A scalar bisection provides a candidate bracket. Acceptance requires opposite endpoint
signs **throughout the face-width interval**, and a strictly signed `F_T` throughout the
bracket. These prove one root inside that bracket for each face-width value and enclosed
coefficient choice. They do not assert the absence of other branches outside the bracket.
Interval Newton contracts the enclosure without discarding any proved root.

Direct subtraction of the two height intervals overestimates their variation. A mean-value
form, `F(q0,T) + F_q(Q,T)*(Q-q0)`, intersects the direct enclosure when checking endpoint
signs and contracting the root. This preserves the equation and tolerance. In the first
pinion patch it reduced refinement from 15 cells to 3, eliminating the initial root-proof
refusals. Those small trials precede the complete-surface runs.

The implicit derivatives are enclosed by

```text
T'   = -F_q/F_T
T''  = -(F_qq + 2 F_qT T' + F_TT T'²)/F_T
T''' = -(F_qqq + 3 F_qqT T' + 3 F_qTT T'² + F_TTT T'³
         + 3 (F_qT + F_TT T') T'')/F_T.
```

The final source parameter is `Tjoin + v*(Ttip(q)-Tjoin)`. Composition through the shared
interval jets gives all mixed derivatives required by the quadratic Taylor correspondence
bound. The full source-line and generating-roll domains must hold over every accepted cell.
A bounded cache binds the entire reference definition, exact face-width interval and
requested derivative order; different geometry cannot reuse an old root.

Tests use an independently soluble square-root root, multiply its equation by a varying
factor to exercise mixed partials, check coefficient/domain cache separation, and refuse
missing or nonuniform roots. A cone with radius equal to axial height supplies a separate
exact tip-height control. Existing Taylor, Bernstein and transcendental controls remain in
the shared suite.

## Completed bounds

All four complete working-flank support surfaces pass the 0.001 mm target. The
[recorded results](flank-results.json) bind coefficient, source, STEP, verifier and raw
report hashes. Each run covers all 256 original Bézier patches, with no unresolved cells,
violating centers or refinement refusals. The complete parameter square also passes the
independent exact cover audit.

| Member / side | Cells checked | Accepted cells | Audit time (s) |
|---|---:|---:|---:|
| Pinion / 0 | 768 | 512 | 85.9 |
| Pinion / 1 | 768 | 512 | 85.7 |
| Gear / 0 | 256 | 256 | 30.4 |
| Gear / 1 | 256 | 256 | 30.8 |

Times measure refinement and cover checking while four processes ran concurrently; loading
and the nine-point association check precede the timer. A negative control displaces one
interior pinion B-spline pole by 0.1 mm. It fails after one cell, with a proved center
correspondence error exceeding 0.03525 mm. This checks the coefficient audit's sensitivity;
it does not certify the STEP reader.

## Reproduction and scope

```sh
SOLVENT_CAD_FLANKS_OUTPUT=/private/tmp/solvent-cad-flank-references.json cargo test --manifest-path rust/Cargo.toml export_cad_flank_references -- --ignored --nocapture
/private/tmp/solvent-occt-env/bin/python experiments/cad-backend/export_generated.py /private/tmp/solvent-cad-sections.json /private/tmp/solvent-cad-flank-references.json /private/tmp/solvent-cad-one-cut-checked /private/tmp/solvent-cad-flank-coefficients.json --kind flank
python3 experiments/cad-backend/check_generated.py /private/tmp/solvent-cad-flank-coefficients.json /private/tmp/solvent-cad-flank-bound-0-0.json --member 0 --side 0 --max-cells 2048
python3 -m unittest discover -s experiments/cad-backend -p 'test_*.py'
```

Repeat the checker for both members and both sides. `--patches 1` is the small iteration
mode and explicitly cannot claim a complete surface. The shared exporter reads the actual
STEP B-spline coefficients; the checker verifies their correspondence across exact Bézier
partitions and audits the accepted parameter cover. Nine source positions per flank check
reference association before refinement, with differences below 1.1e-10 mm on this pair.
Those samples do not establish the accuracy of the original solve.

The target remains 0.001 mm for correspondence between the extracted support surface and
the nominal common-crown reference. Finite trimmed-face coverage, exposed swept material,
source/assembly error, STEP-reader conversion, later indexed CAD operations and continuous
mating remain separate acceptance tasks.
