// D0 (#339): an ellipse whose points move inward at unit speed square to it, in the proposed
// spelling. Does not parse: it stops at `initially`. D4's first gate: a first-order PDE.
//
// A body over two formals: `d`, how far the front has moved, and `u`, the ellipse's eccentric
// angle. The first formal written in `over (…, …)` is marched; the second is sampled at nodes and
// its rates are read off them (the method of lines). `initially` is the Cauchy data: at d = 0 the
// point is the ellipse's. Then:
//   - `nl` is the front's inward normal, drawn from `rate(p over u)` — a row reading a rate along
//     the sampled formal, so algebraic at every step;
//   - the law: `p` moves along `nl` at unit speed;
//   - the gauge: `p` moves square to the front, so no node slides along it. Without the gauge
//     the rates at each node are one row short: "`rate(p over d)` is free along the front".
// The swept surface is regular while rank [x_d, x_u] = 2; here x_d is unit and square to x_u, so
// while x_u ≠ 0. At d = b²/a the front folds at the ends of the major axis (an undercut's kind).
//
// Closed form: x(d, u) = E(u) + d·n(u), n the inward normal; the fold at d = b²/a = 15.625.
unit mm
use std

component Front(c: point, a: Length, b: Length, d: Length, u: Angle) {
  p := point
  initially {
    c distance(a * cos(u), along: right) p
    c distance(b * sin(u), along: up) p
  }
  k := point
  nl := line(p, k)
  length(10mm) nl
  rate(p over u) angle(90deg) nl              // the inward normal (u runs counter-clockwise)
  rate(p over d) parallel nl                  // the gauge: square to the front
  rate(p over d) distance(1, along: nl)       // the law: at unit speed
}

in std.front {
  c := point
  fix((0mm, 0mm)) c
}
front := Front(c, a: 40mm, b: 25mm).p over (d in (0mm, 15mm), u in (0deg, 360deg))
// past the fold: refused at d = 15.625, where the front's rank is lost
folded := Front(c, a: 40mm, b: 25mm).p over (d in (0mm, 16mm), u in (0deg, 360deg))
