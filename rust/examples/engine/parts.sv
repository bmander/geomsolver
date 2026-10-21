// The reciprocating parts, as the end view and the side view each draw them.
//
// Each part reads the numbers it needs from the engine's table, `engine.dims`, as constants of
// its definition: the `use` below lists them, so a piston is `D` wide in every view.

use engine.dims (D, ch, ph, rpin)
use std (offset)

// A line tangent to two circles at both ends — a belt run, a crank web's flank, a cam's flank.
// `side` says which side of the centre line: 1 the left of `k1` to `k2`, -1 the right.  That is
// stated as both circles on the run's other side, which is the outer tangent there — neither a
// crossed one nor the run of no length where overlapping circles meet; the seeds are the two
// contact points at the bearing square to the line of centres.
part Span(k1: circle, k2: circle, side: Scalar) {
  a := point hint(at: k1, bearing: atan2(k2.center.y - k1.center.y, k2.center.x - k1.center.x) + side * 90deg)
  b := point hint(at: k2, bearing: atan2(k2.center.y - k1.center.y, k2.center.x - k1.center.x) + side * 90deg)
  s := line(a, b)
  a coincident k1
  b coincident k2
  s tangent(at: p1) k1
  s tangent(at: p2) k2
  when side > 0 {
    cw(a, b, k1.center)
    cw(a, b, k2.center)
  }
  when side < 0 {
    ccw(a, b, k1.center)
    ccw(a, b, k2.center)
  }
}

// A piston: a rectangle about its small end, the crown `ch` above the pin, two rings under the
// crown.  `pin` is drawn as a circle where the view looks along it and not where it does not,
// which is the one difference between the end view's piston and the side view's.
part Piston(small: point, pin: Int) {
  w := D - 0.5mm
  cl := point hint((small.x - w / 2, small.y + ch))
  cr := point hint((small.x + w / 2, small.y + ch))
  sl := point hint((small.x - w / 2, small.y + ch - ph))
  sr := point hint((small.x + w / 2, small.y + ch - ph))
  (crown := line(cl, cr)) -> (rs := line(cr, sr)) -> (skirt := line(sr, sl)) -> (ls := line(sl, cl)) -> close
  cl offset(dx: -w / 2, dy: ch) small
  cr offset(dx: w / 2, dy: ch) small
  sl offset(dx: -w / 2, dy: ch - ph) small
  sr offset(dx: w / 2, dy: ch - ph) small
  when pin == 1 {
    k := circle(center: small) hint(r: rpin)
    radius(rpin) k
  }
  r1 := line(hint((small.x - w / 2, small.y + ch - 6mm)), hint((small.x + w / 2, small.y + ch - 6mm)))
  r2 := line(hint((small.x - w / 2, small.y + ch - 12mm)), hint((small.x + w / 2, small.y + ch - 12mm)))
  r1.p1 coincident ls
  r1.p2 coincident rs
  r2.p1 coincident ls
  r2.p2 coincident rs
  cl distance(6, along: down) r1.p1
  cl distance(6, along: down) r1.p2
  cl distance(12, along: down) r2.p1
  cl distance(12, along: down) r2.p2
}

// A point placed from `o` by two ordinates — the corner of an outline, a centre on a pitch.
// One statement where a point and its two runs were three.
part At(o: point, dx: Length, dy: Length) {
  p := point hint((o.x + dx, o.y + dy))
  p offset(dx: dx, dy: dy) o
}

// An axis-aligned rectangle about a point: `a` is its lower-left corner offset from `o`.
part Box(o: point, x0: Length, y0: Length, x1: Length, y1: Length) {
  a := point hint((o.x + x0, o.y + y0))
  b := point hint((o.x + x1, o.y + y0))
  c := point hint((o.x + x1, o.y + y1))
  d := point hint((o.x + x0, o.y + y1))
  profile := (ab := line(a, b)) -> (bc := line(b, c)) -> (cd := line(c, d)) -> (da := line(d, a)) -> close
  a offset(dx: x0, dy: y0) o
  b offset(dx: x1, dy: y0) o
  c offset(dx: x1, dy: y1) o
  d offset(dx: x0, dy: y1) o
}
