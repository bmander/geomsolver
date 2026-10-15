// A clothoid, the transition curve a railway or a road puts between a straight and a bend: its
// curvature grows in proportion to its length, κ = s/A², so a train entering it turns the wheel
// steadily rather than all at once.
//
// The body carries the curve's point `p` with its tangent `tl` and left normal `nl`, each 10 long
// and square. Those rows leave three freedoms — where `p` is and which way `tl` points — and three
// rate rows say how they change as the formal `s` runs (#339 D2):
//
//   - `rate(p over s) parallel tl`: the curve runs along its tangent;
//   - `rate(p over s) distance(1, along: tl)`: at unit speed, so `s` is arc length. A rate alone
//     before `distance(k, along: l)` is its component along `l`: never a length of its own, whose
//     row would be quadratic in the rate;
//   - `rate(q over s) distance(10mm * s / A^2, along: nl)`: the tangent's tip turns toward the
//     normal at κ = s/A², a number over the formal like any dimension's.
//
// The drawn instance is the home: the straight ends at `o`, heading along the datum. The bend is
// the circle that osculates the transition where it ends, at s = L, so the track's curvature never
// jumps: its radius is A²/L, claimed below, and its centre on the normal.
//
// Closed form: the Fresnel integrals, x(s) = ∫₀ˢ cos(σ²/2A²) dσ, y(s) = ∫₀ˢ sin(σ²/2A²) dσ.
unit mm
use std

A := 50mm
L := 80mm

component Clothoid(A: Length, s: Length) {
  p := point
  q := point
  k := point
  tl := line(p, q)
  nl := line(p, k)
  length(10mm) tl
  length(10mm) nl
  tl angle(90deg) nl
  rate(p over s) parallel tl
  rate(p over s) distance(1, along: tl)                 // unit speed: s is arc length
  rate(q over s) distance(10mm * s / A^2, along: nl)    // the tangent turns at κ = s/A²
}

in std.front {
  o := point
  x := point
  fix((0mm, 0mm)) o
  fix((-40mm, 0mm)) x
  straight := line(x, o)
  cl := Clothoid(A: A, s: 0mm)
  cl.p coincident o                          // the home: where the straight ends...
  straight angle(0deg) cl.tl                 // ...heading the way it does
  bend := circle hint(r: 30)
  transition curvature(t == L) bend          // the bend osculates the transition's end
  claim radius(A^2 / L) bend
}
transition := cl.p over s in (0mm, L)
