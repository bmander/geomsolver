// The reciprocating parts, as the end view and the side view each draw them.
//
// Components receive the shared dimension table as `dims`, so a piston is `dims.D` wide
// in every view. The caller passes `engine_dims` from `engine.dims`.

use engine.dims

// A line tangent to two circles at both ends — a belt run, a crank web's flank, a cam's flank.
// `side` says which side of the centre line: the seeds are the two contact points at the bearing
// square to the line of centres, so the solve starts on the branch that was asked for.
component Span(k1: circle, k2: circle, side: Scalar) {
  a := point hint(at: k1, bearing: atan2(k2.center.y - k1.center.y, k2.center.x - k1.center.x) + side * 90deg)
  b := point hint(at: k2, bearing: atan2(k2.center.y - k1.center.y, k2.center.x - k1.center.x) + side * 90deg)
  s := line(a, b)
  a on k1
  b on k2
  s tangent(at: p1) k1
  s tangent(at: p2) k2
}

// A connecting rod's *phantom* position, seen along the axis: the centreline and the two eyes,
// the small end riding the bore axis one rod length from the pin.  The rod itself is the part
// `engine.conrod` designs; this is the outline a draughtsman ghosts in for a second position.
component Rod(pin: point, axis: line, dims: group) {
  small := point hint(x: pin.x, y: pin.y + dims.L)
  cl := line(pin, small)
  small on axis
  pin distance(dims.L) small
  big := circle(center: pin) hint(r: dims.rbig)
  sm := circle(center: small) hint(r: dims.rsmall)
  radius(dims.rbig) big
  radius(dims.rsmall) sm
}

// A piston: a rectangle about its small end, the crown `ch` above the pin, two rings under the
// crown.  `pin` is drawn as a circle where the view looks along it and not where it does not,
// which is the one difference between the end view's piston and the side view's.
component Piston(small: point, pin: Int, dims: group) {
  w := dims.D - 0.5mm
  cl := point hint(x: small.x - w / 2, y: small.y + dims.ch)
  cr := point hint(x: small.x + w / 2, y: small.y + dims.ch)
  sl := point hint(x: small.x - w / 2, y: small.y + dims.ch - dims.ph)
  sr := point hint(x: small.x + w / 2, y: small.y + dims.ch - dims.ph)
  (crown := line(cl, cr)) -> (rs := line(cr, sr)) -> (skirt := line(sr, sl)) -> (ls := line(sl, cl)) -> close
  small distance(-w / 2, along: x) cl
  small distance(dims.ch, along: y) cl
  small distance(w / 2, along: x) cr
  small distance(dims.ch, along: y) cr
  small distance(-w / 2, along: x) sl
  small distance(dims.ch - dims.ph, along: y) sl
  small distance(w / 2, along: x) sr
  small distance(dims.ch - dims.ph, along: y) sr
  repeat pin {
    k := circle(center: small) hint(r: dims.rpin)
    radius(dims.rpin) k
  }
  r1 := line(hint(x: small.x - w / 2, y: small.y + dims.ch - 6mm), hint(x: small.x + w / 2, y: small.y + dims.ch - 6mm))
  r2 := line(hint(x: small.x - w / 2, y: small.y + dims.ch - 12mm), hint(x: small.x + w / 2, y: small.y + dims.ch - 12mm))
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
  p := point hint(x: o.x + dx, y: o.y + dy)
  o distance(dx, along: x) p
  o distance(dy, along: y) p
}

// An axis-aligned rectangle about a point: `a` is its lower-left corner offset from `o`.
component Box(o: point, x0: Length, y0: Length, x1: Length, y1: Length) {
  a := point hint(x: o.x + x0, y: o.y + y0)
  b := point hint(x: o.x + x1, y: o.y + y0)
  c := point hint(x: o.x + x1, y: o.y + y1)
  d := point hint(x: o.x + x0, y: o.y + y1)
  profile := (ab := line(a, b)) -> (bc := line(b, c)) -> (cd := line(c, d)) -> (da := line(d, a)) -> close
  o distance(x0, along: x) a
  o distance(y0, along: y) a
  o distance(x1, along: x) b
  o distance(y0, along: y) b
  o distance(x1, along: x) c
  o distance(y1, along: y) c
  o distance(x0, along: x) d
  o distance(y1, along: y) d
}
