// D0 (#339): an ellipse's points moving inward at unit speed square to it, probed in today's
// spelling. The target is `../offset_front.sv`.
//
// The body is a point `p` over two formals, the distance `d` moved and the ellipse's eccentric
// angle `u`. It has no algebraic row: both of `p`'s freedoms come from the rates along `d` (unit
// speed along the front's normal, and the gauge — square to the front), and at the home, d = 0,
// from the Cauchy data: `p` on the ellipse at `u`.
//   - At the home the Cauchy data is `std.Ellipse`'s point, drawn here as `rim`.
//   - At d1 the closed form is drawn as today's computed curve, E(u) + d1·n(u), `n` the inward
//     normal. At u1 the path from the rim to the front is claimed d1 long and square to both
//     curves — the rate rows, integrated in closed form. All theorems.
// Closed form: x(d, u) = E(u) + d·n(u), folding first at d = b²/a (15.625 here), at the ends of
// the major axis, where the ellipse's radius of curvature is least.
unit mm
use std

a := 40mm
b := 25mm
d1 := 12mm
u1 := 50deg

component Front(c: point, a: Length, b: Length, d: Length, u: Angle) {
  p := point(x: c.x + a * cos(u) - d * b * cos(u) / sqrt(b^2 * cos(u)^2 + a^2 * sin(u)^2), y: c.y + b * sin(u) - d * a * sin(u) / sqrt(b^2 * cos(u)^2 + a^2 * sin(u)^2))
}

in std.front {
  c := point
  fix((0mm, 0mm)) c
  e := point
  e coincident(t == u1) rim
  m := point
  m coincident(t == u1) front
  path := line(e, m)
  k := point
  te := line(e, k)
  length(10mm) te
  rim tangent(at: e) te
  j := point
  tm := line(m, j)
  length(10mm) tm
  front tangent(at: m) tm
  claim length(d1) path            // it moved d1...
  claim path perpendicular te      // ...square to the ellipse...
  claim path perpendicular tm      // ...and so square to the front: the gauge
}
rim := trace(std.Ellipse(c, a: a, b: b, tilt: 0deg).p over u in (0deg, 360deg))
front := trace(Front(c, a: a, b: b, d: d1).p over u in (0deg, 360deg))
