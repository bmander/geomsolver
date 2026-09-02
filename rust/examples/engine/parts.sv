// The reciprocating parts, as the end view and the side view each draw them.
//
// Every component reads the dimension table (`use engine.dims`), so a piston is `D` wide here and
// `D` wide in every view, and none of these takes a number the table already states.

use engine.dims

// A line tangent to two circles at both ends — a belt run, a crank web's flank, a cam's flank.
// `side` says which side of the centre line: the seeds are the two contact points at the bearing
// square to the line of centres, so the solve starts on the branch that was asked for.
component Span(k1: circle, k2: circle, side: Scalar) {
  port a: point hint at k1 bearing (atan2(k2.center.y - k1.center.y, k2.center.x - k1.center.x) + side * 90deg)
  port b: point hint at k2 bearing (atan2(k2.center.y - k1.center.y, k2.center.x - k1.center.x) + side * 90deg)
  line s(a, b)
  a on k1
  b on k2
  s tangent(at: p1) k1
  s tangent(at: p2) k2
}

// One crank throw seen along the axis: the pin at `theta` from the bore axis, clockwise from top
// dead centre, and the web drawn as the two tangents from journal to pin.
component Throw(o: point, axis: line, journal: circle, theta: Angle) {
  port pin: point hint(x: o.x + R * sin(theta), y: o.y + R * cos(theta))
  line arm(o, pin) class axis
  o distance(R) pin
  axis angle(-theta) arm
  circle kp(center: pin) hint(r: rp)
  radius(rp) kp
  l: Span(journal, kp, side: 1)
  r: Span(journal, kp, side: -1)
}

// A connecting rod's *phantom* position, seen along the axis: the centreline and the two eyes,
// the small end riding the bore axis one rod length from the pin.  The rod itself is the part
// `engine.conrod` designs; this is the outline a draughtsman ghosts in for a second position.
component Rod(pin: point, axis: line) {
  port small: point hint(x: pin.x, y: pin.y + L)
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
  cl distance(-6, along: y) r1.p1
  cl distance(-6, along: y) r1.p2
  cl distance(-12, along: y) r2.p1
  cl distance(-12, along: y) r2.p2
}

// A throw seen from the side, edge on: the pin is a short cylinder along the crank axis, drawn
// as a rectangle about `pin`, and the two webs run from the journal up to it.  Where `pin`
// stands is the caller's — its height is the end view's, by projection.
component ThrowSide(o: point, pin: point) {
  point a hint(x: pin.x - 10mm, y: pin.y + rp)
  point b hint(x: pin.x + 10mm, y: pin.y + rp)
  point c hint(x: pin.x + 10mm, y: pin.y - rp)
  point d hint(x: pin.x - 10mm, y: pin.y - rp)
  line ab(a, b) -> line bc(b, c) -> line cd(c, d) -> line da(d, a) -> close
  pin distance(-10, along: x) a
  pin distance(rp, along: y) a
  pin distance(10, along: x) b
  pin distance(rp, along: y) b
  pin distance(10, along: x) c
  pin distance(-rp, along: y) c
  pin distance(-10, along: x) d
  pin distance(-rp, along: y) d
  // the webs, from the journal's flanks to the pin's
  line wl(hint(x: o.x - 22mm, y: o.y), hint(x: pin.x - 10mm, y: pin.y))
  line wr(hint(x: o.x + 22mm, y: o.y), hint(x: pin.x + 10mm, y: pin.y))
  o distance(-22, along: x) wl.p1
  o distance(0, along: y) wl.p1
  pin distance(-10, along: x) wl.p2
  pin distance(0, along: y) wl.p2
  o distance(22, along: x) wr.p1
  o distance(0, along: y) wr.p1
  pin distance(10, along: x) wr.p2
  pin distance(0, along: y) wr.p2
}

// A main journal seen from the side: a rectangle 24 long about the axis point `o`.
component Journal(o: point) {
  point a hint(x: o.x - 12mm, y: o.y + rj)
  point b hint(x: o.x + 12mm, y: o.y + rj)
  point c hint(x: o.x + 12mm, y: o.y - rj)
  point d hint(x: o.x - 12mm, y: o.y - rj)
  line ab(a, b) -> line bc(b, c) -> line cd(c, d) -> line da(d, a) -> close
  o distance(-12, along: x) a
  o distance(rj, along: y) a
  o distance(12, along: x) b
  o distance(rj, along: y) b
  o distance(12, along: x) c
  o distance(-rj, along: y) c
  o distance(-12, along: x) d
  o distance(-rj, along: y) d
}

// A point placed from `o` by two ordinates — the corner of an outline, a centre on a pitch.
// One statement where a point and its two runs were three.
component At(o: point, dx: Length, dy: Length) {
  port p: point hint(x: o.x + dx, y: o.y + dy)
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
