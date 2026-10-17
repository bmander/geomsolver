// A Jansen leg that stands on level ground, its toe rod solved.
//
// Jansen chose his eleven rod lengths (the "holy numbers") by evolving them for a long, flat
// stride.  Here the solver finds one of them from that requirement.  The crank's angle is `t`, a
// number that runs a whole turn, and the toe's path is the drawing's own toe as `t` runs
// (`trace(leg.toe over t)`); nothing writes a formula for it.  The heel-to-toe rod `h` is a formal
// the drawn leg leaves unbound, so it is an unknown of the drawing (`leg.h`): a number that does
// not run with `t`, which the stride is solved through.
//
// The ground is level at a height nobody states, and it is held tangent to the stride twice, once
// at each of the path's two shallow dips: a stride that rolls along the ground rather than rocking
// on one low point.  That fixes the ground's height and the rod together.  The rod solves to
// 65.684, within a twentieth of Jansen's 65.7: the holy numbers are, very nearly, the leg that
// stands on the ground at both dips.
//
// The drawing stands at `t = 0` and is followed round the turn (`1 DOF over t`), the rod and the
// ground the same at every instant: the leg walks along the ground it was solved to stand on.
// Change `a` or `l` (where the axle stands) and the rod is solved again.
//
// Try instead: delete the second tangency and state `fix(y == -95) g0`, and the rod solves so the
// stride's lowest point just reaches that ground.

use std (offset)

param t: Angle over (0deg, 360deg)

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
  length(j) rod_j
  length(k) rod_k

  back := point hint((-75, 8))
  (ub := line(pivot, top)) -> (ue := line(top, back)) -> (ud := line(back, pivot)) -> close
  length(b) ub
  length(e) ue
  length(d) ud

  heel := point hint((-59, -28))
  rod_c := line(pivot, knee)
  rod_f := line(back, heel)
  length(c) rod_c
  length(f) rod_f

  toe := point hint((-43, -92))
  (lg := line(knee, heel)) -> (lh := line(heel, toe)) -> (li := line(toe, knee)) -> close
  length(g) lg
  length(h) lh
  length(i) li

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

  // the leg, its crank at `t` and its toe rod left unbound
  leg := Leg(axle, pivot, theta: t)
}
path := trace(leg.toe over t)

// level ground of unstated height, standing on the stride at both dips
in std.front {
  g0 := point hint(y: -92)
  g1 := point hint((0, -92))
  ground := horizontal line(g0, g1)
  g0 distance(60, along: u) g1
  fix(x == -60) g0
  path tangent ground hint(t: 25)
  path tangent ground hint(t: 315)
}
