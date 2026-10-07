// A Jansen leg whose stride bends at a stated radius where it touches the ground, its toe rod
// solved.
//
// The leg and its traced stride are `stride.sv`'s: the heel-to-toe rod `h` is left unbound, so it
// is an unknown of the drawing (`leg.h`) and a column of the stride.  Here the ground is level at a
// height nobody states and touches the stride once, at its bottom; a circle of radius 150 osculates
// the stride at that same place.  The rod solves so the stride is exactly that flat where it
// stands: 66.869.
//
// "At that same place" is one unknown two contacts share.  Both the tangency and the curvature are
// pinned to `s`, an input nothing binds (`param s: Angle`), so the contact's place along the
// stride is that unknown, and both own it (issue #70).  Written as two contacts, each with its own
// place, tied only by the circle touching the ground, the condition holds to third order in the
// distance between the two places: a degenerate root, which solved 3.5e-4 of the rod off.
//
// The crank is still the drawing's one freedom.  Change `radius(150)` and the rod solves again;
// the circle's centre stays straight above the stride's bottom.

use std (offset)

a := 38      // the axle stands this far to the right of the pivot...
l := 7.8     // ...and this far above it

component Leg(axle: point, pivot: point, theta: Angle, h: Length) {
  m := 15      // the crank
  j := 50      // crank pin to the top of the upper triangle
  k := 61.9    // crank pin to the knee
  b := 41.5    // pivot to top          — the upper triangle,
  d := 40.1    // pivot to back
  e := 55.8    // top to back
  c := 39.3    // pivot to knee         — the rocker
  f := 39.4    // back to heel          — the tie between the triangles
  g := 36.7    // knee to heel          — the lower triangle, the foot, with h heel to toe
  i := 49      // knee to toe

  orbit := circle(center: axle) hint(r: m)
  radius(m) orbit
  datum := line(pivot, axle)
  pin := point hint((15, 0))
  crank := line(axle, pin)
  pin coincident orbit
  datum angle(theta) crank

  top := point  hint((-24, 31))
  knee := point hint((-27, -46))
  rod_j := line(pin, top)
  rod_k := line(pin, knee)
  distance(j) rod_j
  distance(k) rod_k

  back := point hint((-75, 8))
  (ub := line(pivot, top)) -> (ue := line(top, back)) -> (ud := line(back, pivot)) -> close
  distance(b) ub
  distance(e) ue
  distance(d) ud

  heel := point hint((-59, -28))
  rod_c := line(pivot, knee)
  rod_f := line(back, heel)
  distance(c) rod_c
  distance(f) rod_f

  toe := point hint((-43, -92))
  (lg := line(knee, heel)) -> (lh := line(heel, toe)) -> (li := line(toe, knee)) -> close
  distance(g) lg
  distance(h) lh
  distance(i) li

  ccw(pivot, pin, top)
  cw(pivot, pin, knee)
  ccw(pivot, top, back)
  cw(back, knee, heel)
  ccw(knee, heel, toe)
}

in std.front {
  axle := point
  pivot := point hint((-38, -7.8))
  fix((0, 0)) axle
  axle offset(dx: a, dy: l) pivot

  // the leg, with its crank angle and its toe rod both left unbound
  leg := Leg(axle, pivot)
}
path := leg.toe over theta in (0, 360)

// level ground of unstated height, touching the stride at its bottom
in std.front {
  g0 := point hint(y: -92)
  g1 := point hint((0, -92))
  ground := horizontal line(g0, g1)
  g0 distance(60, along: x) g1
  fix(x == -60) g0

  // a circle of stated radius, osculating the stride where the stride touches the ground
  k := point hint((-45, 50))
  osc := circle(center: k) hint(r: 140)
  radius(150) osc
}
param s: Angle hint(318)
in std.front {
  path tangent(t == s) ground
  path curvature(t == s) osc
}
