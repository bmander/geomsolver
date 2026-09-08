# Whole-interval geometry bounds

The circular-crown verifier bounds entire parameter rectangles for the sphere
intersection, selected roll branch and differential regularity of the generated surface.
This applies to the documented nominal generating system. Transfer from the finite-accuracy
solved source and global interference remain separate checks.

## Arithmetic contract

`interval::Interval` is a finite closed interval with private endpoints. Addition,
subtraction, multiplication and division expand result bounds to adjacent binary64
values. Division by an interval containing zero fails. Square preserves its nonnegative
range. The contract assumes IEEE binary64 operations with gradual underflow and no
algebraic reassociation. Square root uses the correctly rounded operation specified by
[Rust's `f64::sqrt` contract](https://doc.rust-lang.org/std/primitive.f64.html#method.sqrt),
then expands outward. Invalid input, unsupported function domains and overflowing
finite enclosures return errors, never narrowed substitute intervals.

Rust documents unspecified precision for ordinary
[`sin`](https://doc.rust-lang.org/std/primitive.f64.html#method.sin) and
[`cos`](https://doc.rust-lang.org/std/primitive.f64.html#method.cos) calls. The interval
evaluator instead evaluates Taylor polynomials of degrees 49 and 48 with outward-rounded
arithmetic. For maximum absolute input `M`, their remainder bounds are `M^50/50!` and
`M^49/49!`, also computed outward. Inputs outside `[-8,8]` radians are refused; there is
no unproved argument reduction or approximate-pi assumption. The resulting intervals
are intersected with the known mathematical range `[-1,1]`.

This follows the interval inclusion principle described in the primary paper
[The design of the Boost interval arithmetic library](https://guillaume.melquiond.fr/doc/06-tcs-rnc5.pdf).
The implementation is dependency-free and changes no process rounding mode.

`RevolvedSurface::generating_profile_bounds` encloses the position and first derivative
of a line or circular generating profile before revolution. Cached binary64 coefficients
define the analytic function being bounded. These are profile bounds, not bounds for
the revolution or generated envelope. Source solve error and differences between the
cached coefficients and intended dimensions remain separate. The same query is exposed
through `gcs_surface_profile_bounds` and browser `generatingProfileBounds`.
`generating_profile_jet_bounds` additionally returns the second derivative `duu` in the
core, using the same analytic coefficients and outward rounding. A line has exactly zero
second derivative; a round profile differentiates its sine/cosine terms analytically.
First-order queries do not compute curvature, so overflow in a second derivative cannot
invalidate an otherwise finite first-order enclosure.

## Circular-crown branch equations

For crown center `(cx,cy)`, meridian radius `r(u)`, height `h(u)` and spherical face
coordinate `rho`, define

```
c² = cx² + cy²
q  = (rho² - h² - c² - r²) / (2 c r)
w  = sqrt(1 - q²)
cos(theta) = (cx q + cy w) / c
sin(theta) = (cy q - cx w) / c
```

The selected circle/sphere intersection is transverse when `1-q² > 0`. The verifier
also proves `cos(theta) > 0`, that the selected sine is negative, and that the other
circle intersection has positive sine. Exactly one intersection therefore belongs
to the declared lower crown semicircle. No inverse-trigonometric branch selection is
hidden in the interval computation.

Using the unnormalized normal `(h' cos(theta),h' sin(theta),-r')`, put

```
Px = cx + r cos(theta)
Py = cy + r sin(theta)
A  = -r' Px - h h' cos(theta)
B  = -r' Py - h h' sin(theta)
```

The roll equation is `A sin(t) + B cos(t) = 0`. A nonzero bound on `A` gives the
continuous root `t = atan(-B/A)`. Comparing bounds on `-B/A` with enclosed tangents of
the declared roll limits keeps that root inside the interval and excludes the other
pi-separated roots. The generating normal also has a positive squared-length bound
`r'²+h'²`.

The verifier starts with `u in [0,1]` and `rho in [0.9 Rm,1.1 Rm]`. An inconclusive
rectangle is split deterministically, retaining both children. Every leaf must establish
all seven strict margins, including the surface area factor below. Reaching the subdivision limit fails the check; no unresolved
cell is discarded. The resulting cover includes all four flanks and four root fillets
for each of the nine existing tooth-count/size cases.

## Generated-surface regularity

A regular contact equation alone does not prevent a cusp on its generated surface.
Use the source chart `S(u,theta) = (cx+r cos(theta), cy+r sin(theta), h)` and the
documented member motion `X = B_i(t)^-1 C(t) S`, with the selected root `t(u,theta)`.
Let `kappa = z_g/z_p` for the pinion and `kappa = -z_p/z_g` for the gear. These are
exact ratios of integers, equal to the signed cotangents of the pitch angles.
Pulling the generated tangents back through their common rigid rotation gives

```
X_u     -> S_u     + V t_u
X_theta -> S_theta + V t_theta
V = kappa (-cos(t), sin(t), 0) cross S
```

The contact equation makes `V` tangent to the source. Since the orthogonal source
tangents have positive lengths, write `V = alpha S_u + beta S_theta`. Their two-by-two
coefficient determinant is

```
J = 1 + alpha t_u + beta t_theta
```

Thus the generated oriented area vector, pulled back to the crown frame, is
`J (S_u cross S_theta)`. A bound excluding zero from `J` proves rank two, independently
of the local numerical intersection solver's rank test. To evaluate this without
large interval overestimates from cancelling terms, define

```
D = r r' + h h'
G = cx cos(theta) + cy sin(theta)
K = cy cos(theta) - cx sin(theta)
E = r' (r'^2 + h'^2) + h (r' h'' - h' r'')
H = A^2 + B^2

J = 1 + kappa sign(A) h [E K^2 + (D/r) (r' G + D)^2] / H^(3/2)
```

For review, this follows by differentiating the roll equation:
`t_a = (B A_a - A B_a)/H`, for `a = u,theta`. Substitution gives
`alpha = kappa sign(A) h K/sqrt(H)`,
`beta = -kappa sign(A) h (r' G+D)/(r sqrt(H))`,
`t_u = E K/H` and `t_theta = -D (r' G+D)/H`.
No division by `h'` or `r'` is required. In particular, this proof includes the
fillet/root endpoints where `h' = 0` and an axial slicing equation can be tangent.

The sphere-coordinate change also remains regular:
`d theta/d rho = rho/(c r sqrt(1-q^2)) > 0`. The branch margins, positive source
normal length, and nonzero `J` therefore establish a regular generated map on each
whole `(u,rho)` rectangle, including its boundary. Indexing a tooth by a rigid rotation
preserves this result. It does not establish injectivity or absence of intersections
between distinct surface points, teeth or members.

The current report covers 72 rectangles with 435 accepted leaf cells. The independent
checker confirms every margin, the exported `J` enclosures and exact coverage. All 72
domains retain positive orientation. The weakest reported sphere discriminant bound is
greater than `0.7808`; the roll tangent margin is greater than `0.1598`; the dimensionless
area factor is greater than `0.00977`. These are geometric regularity margins, not
machining accuracy or distance bounds.

## Independent review

Arithmetic tests export exact binary64 bit patterns. A standalone Python checker uses
`Fraction` to verify arithmetic and square-root enclosure, and different, higher-degree
exact rational Taylor polynomials to check trigonometric point intervals.

The branch report exports every leaf rectangle, its radius/height/first- and
second-derivative input enclosures, area-factor interval and seven positive margins.
A second checker recomputes the inequalities
using rational interval arithmetic and rational square-root enclosures. Exact sweep
slabs verify coverage, so overlaps cannot compensate for gaps. Negative controls reject
empty, missing and duplicate cells, overstated margins and invalid area-factor enclosures.

A separate numerical test differentiates the complete generated position map, composing
the crown and member rotations directly and refining the finite-difference step. It
agrees with the area factor at 72 points across both members' flanks and fillets. This
is a check of the derivation, separate from the interval proof. The earlier 12:36 deep-section
cusp is a negative control: its sphere intersection and contact branch remain regular,
but `J` changes sign between crown heights `0.13` and `0.14` mm. An interval spanning
that cusp cannot certify a regular generated surface.

```sh
SOLVENT_INTERVAL_OUTPUT=/private/tmp/solvent-interval-arithmetic.tsv \
SOLVENT_BRANCH_OUTPUT=/private/tmp/solvent-crown-branches.json \
cargo test --manifest-path rust/Cargo.toml -p gcs-core --test core -- --quiet

python3 rust/gcs-core/tests/verification/interval_arithmetic.py \
  /private/tmp/solvent-interval-arithmetic.tsv
python3 rust/gcs-core/tests/verification/crown_branches.py \
  /private/tmp/solvent-crown-branches.json
```

The branch proof applies to the nominal circular-crown equations with the supplied
profile enclosures and intended common-crown alignment and motions. The independent
checker treats those profile enclosures as inputs; it does not re-solve Solvent or
prove that finite-accuracy solved frames equal the ideal alignment. Bounds transferring
that solve error to the geometric construction are still needed. Existing point
comparisons remain an independent numerical check of their correspondence.

These inequalities establish differential regularity for the nominal generated flanks
and fillets in the nine checked configurations. They do not establish a global admissible
parameter range or prevent distinct regular surface points from intersecting. Source
error transfer, global non-interference, analytic face assembly and tolerance-controlled
export remain open gates for the finished gear pair.
