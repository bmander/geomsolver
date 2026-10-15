// D0 (#339): the tractrix, probed in today's spelling. The target is `../tractrix.sv`.
//
// The body without its rate row is `Drag`: the string's end `q` on the line, `a · u` along it,
// the string `a` long. It leaves one freedom at every place, the string's bearing, which the rate
// row `rate(p over u) parallel string` supplies away from the home.
//   - At the home (u = 0) the drawing holds it: `p` square above `o`. DOF 0. Delete that line and
//     the report is DOF 1: one rate row is what the body is short.
//   - At u1 the closed form is drawn as today's computed curve and `p` is put on it. The body's
//     algebraic row (the string's length) and its rate row (the path runs along the string, said
//     as today's tangency) are claimed there: both theorems.
// Closed form: x = a(u − tanh u), y = a sech u, with the string's end at (a·u, 0).
unit mm
use std

a := 30mm
u1 := 1.2

component Drag(o: point, a: Length, u: Scalar) {
  q := point
  p := point hint((0, 30))
  q level(up) o
  o distance(a * u, along: right) q
  string := line(p, q)
  length(a) string
}

component Closed(o: point, a: Length, u: Scalar) {
  p := point(x: o.x + a * (u - (exp(2 * u) - 1) / (exp(2 * u) + 1)), y: o.y + a * 2 / (exp(u) + exp(-u)))
}

in std.front {
  o := point
  fix((0mm, 0mm)) o

  // the home: the drawn instance, and what holds there alone
  home := Drag(o, a: a, u: 0)
  o distance(a, along: up) home.p

  // the place u1, on the closed form
  p := point hint((14, 16))
  q := point hint((36, 0))
  q level(up) o
  o distance(a * u1, along: right) q
  string := line(p, q)
  p coincident(t == u1) tr
  k := point hint((20, 10))
  tl := line(p, k)
  length(10mm) tl
  tr tangent(at: p) tl
  claim length(a) string            // the body's algebraic row holds on the closed form
  claim tl parallel string          // and its rate row: the path runs along the string
}
tr := Closed(o, a: a).p over u in (0, 3)
