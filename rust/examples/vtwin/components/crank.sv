// The crank train, seen along the axis: the pin `R` from the axis on the disc (`components.disc`),
// and the arm from the axis to it.
//
// The pin's angle is the drawing's one degree of freedom.  `theta` is defined nowhere, so it is a
// free variable — the instance's own unknown, `crank.theta` — and the arm's dimension reads it:
// the callout shows whatever angle the crank is at, and dragging the pin turns it.  Everything
// in both banks follows from where the pin is; `theta0` in the table is only where it starts.

use components.dims
use components.parts
use components.disc

component Crank(o: point, ref: line, dims: group) {
  point pin hint(x: o.x + dims.R * sin(dims.theta0), y: o.y + dims.R * cos(dims.theta0))
  line arm(o, pin)
  o distance(dims.R) pin
  arm angle(theta) ref
  circle path(center: o) hint(r: dims.R)
  radius(dims.R) path
  // the clevis pin's end, seen on
  circle kp(center: pin) hint(r: dims.rpin)
  radius(dims.rpin) kp
  plane disc_axes(origin: o, toward: pin)
  disc: Disc(disc_axes, dims: dims)
}

// Open this file to preview the crank; drag its pin to turn it.
preview {
  unit mm
  line ref(std.origin, std.up.toward)
  crank: Crank(std.origin, ref, dims: vtwin_dims)
}
