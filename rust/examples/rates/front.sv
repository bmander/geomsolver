// An offset front: an ellipse whose points move inward at unit speed, square to it — the
// first-order PDE of a wavefront, marched along how far it has moved by the method of lines
// (#339 D4).
//
// The body runs over two formals: `d`, how far the front has moved, and `u`, the ellipse's
// eccentric angle. The first is **marched**; the second is **sampled**, at the Gauss–Lobatto
// nodes of pieces of its interval, and a rate along it is the derivative of the polynomial through
// a piece's nodes. `initially` is the Cauchy data: at d = 0, for every u, the point is the
// ellipse's. Then:
//
//   - `nl`, ten long in the ellipse's plane, is square to `rate(p over u)`: the front's normal, a
//     row read at every step from the nodes (its seed picks the inward one, continuity keeps it);
//   - the law, `rate(p over d) distance(1, along: nl)`: `p` moves along `nl` at unit speed;
//   - the gauge, `rate(p over d) parallel nl`: it moves square to the front, so no node slides
//     along it. Without the gauge the rates leave a freedom, refused naming `p`.
//
// The swept surface is regular while rank [x_d, x_u] = 2; here x_d is unit and square to x_u, so
// while x_u ≠ 0. At d = b²/a the front folds at the ends of the major axis, where the ellipse is
// most curved: run `d` past 15mm and the march is refused just short of it, at u = 0, where a
// node's area |x_d × x_u| falls to a hundredth of what it was — a fold, as an undercut is.
// `folded` is the same front run to 16mm: `solventc` says how far each is marched, and where
// `folded` is refused.
//
// A point is held on the surface where the closed form puts the front: `held`, on the major axis
// 35 from the centre, 5 inside the ellipse's end. The contact finds its place there, (5, 0°), and
// the line along the surface's normal at it is claimed along the ellipse's plane's: the front
// never leaves it.
//
// Closed form: x(d, u) = E(u) + d·n(u), n the inward unit normal; the fold at b²/a = 15.625.
unit mm
use std

a := 40mm
b := 25mm

component Front(c: point, x1: point, y1: point, z1: point, a: Length, b: Length, d: Length,
                u: Angle) {
  xa := line(c, x1)
  ya := line(c, y1)
  za := line(c, z1)
  p := point hint((a * cos(u), b * sin(u), 0mm))
  k := point hint((0.75 * a * cos(u), 0.6 * b * sin(u), 0mm))
  nl := line(p, k)
  length(10mm) nl
  c distance(0mm, along: za) k               // the normal in the ellipse's plane...
  nl perpendicular rate(p over u)            // ...square to the front
  rate(p over d) parallel nl                 // the gauge: square to itself
  rate(p over d) distance(1, along: nl)      // the law: at unit speed
  initially {
    c distance(a * cos(u), along: xa) p      // the Cauchy data: the ellipse
    c distance(b * sin(u), along: ya) p
    c distance(0mm, along: za) p
  }
}

c := point hint((0, 0, 0))
fix((0, 0, 0)) c
x1 := point hint((10, 0, 0))
fix((10, 0, 0)) x1
y1 := point hint((0, 10, 0))
fix((0, 10, 0)) y1
z1 := point hint((0, 0, 10))
fix((0, 0, 10)) z1

front := trace(Front(c, x1, y1, z1, a: a, b: b).p over (d in (0mm, 15mm), u in (-60deg, 60deg)))
folded := trace(Front(c, x1, y1, z1, a: a, b: b).p over (d in (0mm, 16mm), u in (-60deg, 60deg)))

// `held` on the major axis, 35 from the centre: where the front at its end has run a − 35 = 5.
ya := line(c, y1)
held := point hint((35, 0, 0))
held coincident front
c level(ya) held
c distance(35mm) held
above := point hint((35, 0, 10))
up := line(held, above)
length(10mm) up
front perpendicular(at: held) up
plumb := line(c, z1)
claim up parallel plumb                      // the swept surface is the ellipse's plane
