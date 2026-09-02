// Jansen's linkage — the walking leg of Theo Jansen's Strandbeesten
// (https://en.wikipedia.org/wiki/Jansen%27s_linkage).  A crank turning on one fixed point
// drives two rigid triangles hinged on another, and the toe of the lower triangle walks: a
// long, nearly level stride along the bottom of its path and a raised return over the top.
//
// The leg is one component, written once.  It states nothing but lengths — the eleven rods,
// in the proportions Jansen calls the holy numbers — so every joint is where two rods of
// stated length meet, and each is one of two mirror poses.  The ccw/cw lines pick the pose the
// machine is built to; the seeds only start the solve near it.  The component takes the crank
// angle `theta` as a formal: drawn with it unbound, the crank is the drawing's one freedom
// (dof 1, Under — drag `leg.pin` round its circle and the leg takes a step); asked for its
// `toe` as `theta` runs, the same component is the stride, and the drawing's own pose is
// where the trace is anchored.

// the frame the leg hangs from: the crank's axle, and the pivot both triangles swing on
param a = 38      // the axle stands this far to the right of the pivot...
param l = 7.8     // ...and this far above it

component Leg(axle: point, pivot: point, theta: Angle) {
  // the holy numbers, in the lettering of Wikipedia's figure
  param m = 15      // the crank
  param j = 50      // crank pin to the top of the upper triangle
  param k = 61.9    // crank pin to the knee
  param b = 41.5    // pivot to top          — the upper triangle,
  param d = 40.1    // pivot to back
  param e = 55.8    // top to back
  param c = 39.3    // pivot to knee         — the rocker
  param f = 39.4    // back to heel          — the tie between the triangles
  param g = 36.7    // knee to heel          — the lower triangle, the foot,
  param h = 65.7    // heel to toe
  param i = 49      // knee to toe

  // the crank on its circle, at angle theta from the pivot-to-axle line
  circle orbit(center: axle) hint(r: m) class construction
  radius(m) orbit
  line datum(pivot, axle) class construction
  point pin hint(x: 15, y: 0)
  line crank(axle, pin)
  pin on orbit
  datum angle(theta) crank

  // the two rods the pin drives
  point top  hint(x: -24, y: 31)
  point knee hint(x: -27, y: -46)
  line rod_j(pin, top)
  line rod_k(pin, knee)
  distance(j) rod_j
  distance(k) rod_k

  // the upper triangle, one rigid body swinging on the pivot
  point back hint(x: -75, y: 8)
  line ub(pivot, top) -> line ue(top, back) -> line ud(back, pivot) -> close
  distance(b) ub
  distance(e) ue
  distance(d) ud

  // the rocker from the pivot to the knee, and the tie from the back down to the heel
  point heel hint(x: -59, y: -28)
  line rod_c(pivot, knee)
  line rod_f(back, heel)
  distance(c) rod_c
  distance(f) rod_f

  // the lower triangle: the foot, with the toe at the bottom
  point toe hint(x: -43, y: -92)
  line lg(knee, heel) -> line lh(heel, toe) -> line li(toe, knee) -> close
  distance(g) lg
  distance(h) lh
  distance(i) li

  // which of each joint's two poses — each holds through the whole turn of the crank
  ccw(pivot, pin, top)      // the top above the pin-to-pivot line, the knee below it
  cw(pivot, pin, knee)
  ccw(pivot, top, back)     // the back behind the pivot-to-top side
  cw(back, knee, heel)      // the heel below the knee-to-back line
  ccw(knee, heel, toe)      // the toe below the knee-to-heel side
}

point axle  hint(x: 0, y: 0)
point pivot hint(x: -38, y: -7.8)
ground axle
pivot distance(a, along: x) axle
pivot distance(l, along: y) axle

// the leg, with its crank angle left unbound — the drawing's one freedom
leg: Leg(axle, pivot)

// where the toe goes over a whole turn of the crank
curve path = leg.toe over theta in (0, 360)
