// The crank train, seen along the axis: the pin `R` from the axis on the disc (`components.disc`),
// and the arm from the axis to it, in the view `layout.front` along the crank axis.
//
// The arm stands at `theta` from the reference up the page, and its dimension reads it: the
// assembly binds it to its crank angle `t`, so the callout shows whatever instant the drawing is
// shown at.  The disc is drawn in its own frame, turned to the pin, and everything in both banks
// follows from where the pin is.

use std
use components.dims
use components.parts
use components.disc

component Crank(layout: group, dims: group, theta: Angle) {
  in layout.front {
    // at `theta = 0` the pin stands straight up the reference
    pin := point hint((layout.origin.x, layout.origin.y + dims.R))
    arm := line(layout.origin, pin)
    layout.origin distance(dims.R) pin
    arm angle(theta) layout.ax
    path := circle(center: layout.origin) hint(r: dims.R)
    radius(dims.R) path
    // the clevis pin's end, seen on
    kp := circle(center: pin) hint(r: dims.rpin)
    radius(dims.rpin) kp
    disc_axes := std.Turned(layout.origin, pin)
  }
  disc := components.disc.Disc(disc_axes, dims: dims)
}

// Open this file to preview the crank; `theta` is left unbound, so it is the preview's own
// unknown, and dragging the pin turns it.
preview {
  unit mm
  in std.front {
    ref := line(std.origin, hint((0, 1)))
    fix((0, 1)) ref.p2
  }
  layout := {front: std.front, origin: std.origin, ax: ref}
  crank := Crank(layout, dims: components.dims.vtwin_dims)
}
