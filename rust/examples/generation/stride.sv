// A Jansen leg that stands on level ground, its toe rod solved.
//
// Jansen chose his eleven rod lengths (the "holy numbers") by evolving them for a long, flat
// stride.  Here the solver finds one of them from that requirement.  The toe's path is a curve of
// the drawing, traced from the leg as the crank runs a whole turn (`over theta in (0, 360)`), and
// nothing writes a formula for it.  The heel-to-toe rod `h` is a formal the drawn leg leaves
// unbound, so it is an unknown of the drawing (`leg.h`) and a column of the traced stride as well.
//
// The ground is level at a height nobody states, and it is held tangent to the stride twice, once
// at each of the path's two shallow dips: a stride that rolls along the ground rather than rocking
// on one low point.  That fixes the ground's height and the rod together.  The rod solves to
// 65.684, within a twentieth of Jansen's 65.7: the holy numbers are, very nearly, the leg that
// stands on the ground at both dips.
//
// The crank is still the drawing's one freedom: drag `leg.pin` round its circle and the leg walks
// along the ground it was solved to stand on.  Change `a` or `l` (where the axle stands) and the
// rod is solved again.
//
// Try instead: delete the second tangency and state `fix(y == -95) g0`, and the rod solves so the
// stride's lowest point just reaches that ground.

use std

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
  pin := point hint(x: 15, y: 0)
  crank := line(axle, pin)
  pin coincident orbit
  datum angle(theta) crank

  top := point  hint(x: -24, y: 31)
  knee := point hint(x: -27, y: -46)
  rod_j := line(pin, top)
  rod_k := line(pin, knee)
  distance(j) rod_j
  distance(k) rod_k

  back := point hint(x: -75, y: 8)
  (ub := line(pivot, top)) -> (ue := line(top, back)) -> (ud := line(back, pivot)) -> close
  distance(b) ub
  distance(e) ue
  distance(d) ud

  heel := point hint(x: -59, y: -28)
  rod_c := line(pivot, knee)
  rod_f := line(back, heel)
  distance(c) rod_c
  distance(f) rod_f

  toe := point hint(x: -43, y: -92)
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
  pivot := point hint(x: -38, y: -7.8)
  fix(x == 0, y == 0) axle
  pivot distance(a, along: x) axle
  pivot distance(l, along: y) axle

  // the leg, with its crank angle and its toe rod both left unbound
  leg := Leg(axle, pivot)
}
path := leg.toe over theta in (0, 360)

// level ground of unstated height, standing on the stride at both dips
in std.front {
  g0 := point hint(x: -60, y: -92)
  g1 := point hint(x: 0, y: -92)
  ground := horizontal line(g0, g1)
  g0 distance(60, along: x) g1
  fix(x == -60) g0
  path tangent ground hint(t: 25)
  path tangent ground hint(t: 315)
}
