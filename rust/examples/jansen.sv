// Jansen's linkage — the walking leg of Theo Jansen's Strandbeesten
// (https://en.wikipedia.org/wiki/Jansen%27s_linkage).  A crank turning on one fixed point
// drives two rigid triangles hinged on another, and the toe of the lower triangle walks: a
// long, nearly level stride along the bottom of its path and a raised return over the top.
//
// The leg is one component, written once.  It states nothing but lengths — the eleven rods,
// in the proportions Jansen calls the holy numbers — so every joint is where two rods of
// stated length meet, and each is one of two mirror poses.  The ccw/cw lines pick the pose the
// machine is built to; the seeds only start the solve near it.  The component takes the crank
// angle `theta` as a formal, and the drawn leg is given `t`, the crank's angle declared as a
// number that runs a whole turn: the leg is drawn at `t = 0` and followed round from there
// (DOF 0, `1 DOF over t` — the crank, which is how a mechanism is counted).  The stride is the
// drawing's own toe as `t` runs.

use std (offset)

param t: Angle over (0deg, 360deg)

// the frame the leg hangs from: the crank's axle, and the pivot both triangles swing on
a := 38      // the axle stands this far to the right of the pivot...
l := 7.8     // ...and this far above it

component Leg(axle: point, pivot: point, theta: Angle) {
  // the holy numbers, in the lettering of Wikipedia's figure
  m := 15      // the crank
  j := 50      // crank pin to the top of the upper triangle
  k := 61.9    // crank pin to the knee
  b := 41.5    // pivot to top          — the upper triangle,
  d := 40.1    // pivot to back
  e := 55.8    // top to back
  c := 39.3    // pivot to knee         — the rocker
  f := 39.4    // back to heel          — the tie between the triangles
  g := 36.7    // knee to heel          — the lower triangle, the foot,
  h := 65.7    // heel to toe
  i := 49      // knee to toe

  // the crank on its circle, at angle theta from the pivot-to-axle line
  orbit := circle(center: axle) hint(r: m)
  radius(m) orbit
  datum := line(pivot, axle)
  pin := point hint((15, 0))
  crank := line(axle, pin)
  pin coincident orbit
  datum angle(theta) crank

  // the two rods the pin drives
  top := point  hint((-24, 31))
  knee := point hint((-27, -46))
  rod_j := line(pin, top)
  rod_k := line(pin, knee)
  length(j) rod_j
  length(k) rod_k

  // the upper triangle, one rigid body swinging on the pivot
  back := point hint((-75, 8))
  (ub := line(pivot, top)) -> (ue := line(top, back)) -> (ud := line(back, pivot)) -> close
  length(b) ub
  length(e) ue
  length(d) ud

  // the rocker from the pivot to the knee, and the tie from the back down to the heel
  heel := point hint((-59, -28))
  rod_c := line(pivot, knee)
  rod_f := line(back, heel)
  length(c) rod_c
  length(f) rod_f

  // the lower triangle: the foot, with the toe at the bottom
  toe := point hint((-43, -92))
  (lg := line(knee, heel)) -> (lh := line(heel, toe)) -> (li := line(toe, knee)) -> close
  length(g) lg
  length(h) lh
  length(i) li

  // which of each joint's two poses — each holds through the whole turn of the crank
  ccw(pivot, pin, top)      // the top above the pin-to-pivot line, the knee below it
  cw(pivot, pin, knee)
  ccw(pivot, top, back)     // the back behind the pivot-to-top side
  cw(back, knee, heel)      // the heel below the knee-to-back line
  ccw(knee, heel, toe)      // the toe below the knee-to-heel side
}

in std.front {
  axle := point 
  pivot := point hint((-38, -7.8))
  fix((0, 0)) axle
  axle offset(dx: a, dy: l) pivot

  // the leg, its crank at `t`
  leg := Leg(axle, pivot, theta: t)
}

// where the toe goes over a whole turn of the crank
path := trace(leg.toe over t)
