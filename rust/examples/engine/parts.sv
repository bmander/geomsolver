// The reciprocating parts, as the end view and the side view each draw them.
//
// Every component reads the dimension table (`use engine.dims`), so a piston is `D` wide here and
// `D` wide in every view, and none of these takes a number the table already states.

use engine.dims

// A line tangent to two circles at both ends — a belt run, a crank web's flank, a cam's flank.
// `side` says which side of the centre line: the seeds are the two contact points at the bearing
// square to the line of centres, so the solve starts on the branch that was asked for.
component Span(k1: circle, k2: circle, side: Scalar) {
  point a hint(at: k1, bearing: atan2(k2.center.y - k1.center.y, k2.center.x - k1.center.x) + side * 90deg)
  point b hint(at: k2, bearing: atan2(k2.center.y - k1.center.y, k2.center.x - k1.center.x) + side * 90deg)
  line s(a, b)
  a on k1
  b on k2
  s tangent(at: p1) k1
  s tangent(at: p2) k2
}

// A connecting rod's *phantom* position, seen along the axis: the centreline and the two eyes,
// the small end riding the bore axis one rod length from the pin.  The rod itself is the part
// `engine.conrod` designs; this is the outline a draughtsman ghosts in for a second position.
component Rod(pin: point, axis: line) {
  point small hint(x: pin.x, y: pin.y + L)
  line cl(pin, small) class axis
  small on axis
  pin distance(L) small
  circle big(center: pin) hint(r: rbig)
  circle sm(center: small) hint(r: rsmall)
  radius(rbig) big
  radius(rsmall) sm
}

// A piston: a rectangle about its small end, the crown `ch` above the pin, two rings under the
// crown.  `pin` is drawn as a circle where the view looks along it and not where it does not,
// which is the one difference between the end view's piston and the side view's.
component Piston(small: point, pin: Int) {
  param w = D - 0.5mm
  point cl hint(x: small.x - w / 2, y: small.y + ch)
  point cr hint(x: small.x + w / 2, y: small.y + ch)
  point sl hint(x: small.x - w / 2, y: small.y + ch - ph)
  point sr hint(x: small.x + w / 2, y: small.y + ch - ph)
  line crown(cl, cr) -> line rs(cr, sr) -> line skirt(sr, sl) -> line ls(sl, cl) -> close
  small distance(-w / 2, along: x) cl
  small distance(ch, along: y) cl
  small distance(w / 2, along: x) cr
  small distance(ch, along: y) cr
  small distance(-w / 2, along: x) sl
  small distance(ch - ph, along: y) sl
  small distance(w / 2, along: x) sr
  small distance(ch - ph, along: y) sr
  repeat pin {
    circle k(center: small) hint(r: rpin)
    radius(rpin) k
  }
  line r1(hint(x: small.x - w / 2, y: small.y + ch - 6mm), hint(x: small.x + w / 2, y: small.y + ch - 6mm)) class thin
  line r2(hint(x: small.x - w / 2, y: small.y + ch - 12mm), hint(x: small.x + w / 2, y: small.y + ch - 12mm)) class thin
  r1.p1 on ls
  r1.p2 on rs
  r2.p1 on ls
  r2.p2 on rs
  cl distance(6, along: down) r1.p1
  cl distance(6, along: down) r1.p2
  cl distance(12, along: down) r2.p1
  cl distance(12, along: down) r2.p2
}

// A point placed from `o` by two ordinates — the corner of an outline, a centre on a pitch.
// One statement where a point and its two runs were three.
component At(o: point, dx: Length, dy: Length) {
  point p hint(x: o.x + dx, y: o.y + dy)
  o distance(dx, along: x) p
  o distance(dy, along: y) p
}

// An axis-aligned rectangle about a point: `a` is its lower-left corner offset from `o`.
component Box(o: point, x0: Length, y0: Length, x1: Length, y1: Length) {
  point a hint(x: o.x + x0, y: o.y + y0)
  point b hint(x: o.x + x1, y: o.y + y0)
  point c hint(x: o.x + x1, y: o.y + y1)
  point d hint(x: o.x + x0, y: o.y + y1)
  line ab(a, b) -> line bc(b, c) -> line cd(c, d) -> line da(d, a) -> close
  o distance(x0, along: x) a
  o distance(y0, along: y) a
  o distance(x1, along: x) b
  o distance(y0, along: y) b
  o distance(x1, along: x) c
  o distance(y1, along: y) c
  o distance(x0, along: x) d
  o distance(y1, along: y) d
}
