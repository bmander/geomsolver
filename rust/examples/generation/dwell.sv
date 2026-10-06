// A dwell linkage: where a coupler curve is nearly a circle, and how big that circle is.
//
// A point on the coupler of a four-bar traces a curve as the crank turns, and where that curve
// runs close to an arc of a circle, a link of the circle's radius hung from the coupler point
// barely moves while the point runs along the arc: the output dwells.  A designer wants the arc's
// centre of curvature (where the dwell link's far end will sit) and its radius (the link's length).
//
// `path` is the coupler point's curve as the crank runs a whole turn, traced from the linkage
// (`over theta in (0, 360)`); nothing writes a formula for it.  `osc` osculates it with the crank
// at 170°, where the curve's curvature is stationary and the arc is longest: the circle's centre
// is the curve's centre of curvature there, from the trace's exact second derivative (no
// differences).  The coupler point's offset `ap` along the coupler is left unbound, so it is an
// unknown of the drawing, and the stated radius of the dwell arc (25) is what fixes it: it solves
// to about 25.5.
//
// The crank is the drawing's one freedom: drag `bar.A` round and the coupler point runs along the
// curve, close to `osc` from about 150° to 190°.  Edit the dwell radius, or `beta` (the coupler
// point's angle off the coupler), and `ap` and the circle are solved again.

use std

r2 := 10      // the crank
r3 := 30      // the coupler
r4 := 25      // the rocker
beta := 50deg // the coupler point's angle off the coupler

component FourBar(o2: point, o4: point, r2: Length, r3: Length, r4: Length, beta: Angle,
                  theta: Angle, ap: Length) {
  A := point hint((-9.85, 1.74))
  B := point hint((14.34, 19.49))
  P := point hint((-8.19, 27.23))
  base := line(o2, o4)
  crank := line(o2, A)
  coupler := line(A, B)
  rocker := line(o4, B)
  arm := line(A, P)
  distance(r2) crank
  base angle(theta) crank
  distance(r3) coupler
  distance(r4) rocker
  distance(ap) arm
  coupler angle(beta) arm
  ccw(A, o4, B)               // the rocker's pose: B left of the line from A to its pivot
}

in std.front {
  o2 := point
  o4 := point
  fix((0, 0)) o2
  fix((30, 0)) o4

  // the linkage, its crank angle and its coupler point's offset both left unbound
  bar := FourBar(o2, o4, r2: r2, r3: r3, r4: r4, beta: beta)
}
path := bar.P over theta in (0, 360)

// the dwell arc: a circle osculating the coupler curve at 170°, of the radius the link will have
in std.front {
  k := point hint((11.6, 11.9))
  osc := circle(center: k) hint(r: 25)
  path curvature(t == 170) osc
  radius(25) osc
}
