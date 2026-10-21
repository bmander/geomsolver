// The crank train, seen along the axis: the pin `R` from the axis on the disc (`components.disc`),
// and the arm from the axis to it, drawn in the plane the axis `o` is drawn in — the view along
// the crank axis.
//
// The arm stands at `theta` from the reference up the page, and its dimension reads it: the
// assembly binds it to its crank angle `t`, and its timeline shows whatever instant the drawing is
// shown at.  The disc is drawn in its own frame, turned to the pin, and everything in both banks
// follows from where the pin is.

use std
use components.dims (R, rpin)
use components.parts
use components.disc

// `o` is the crank axis, seen end on, and `ref` the reference the angle is measured from.
part Crank(o: point, ref: line, theta: Angle) {
  in o.plane {
    // at `theta = 0` the pin stands straight up the reference
    pin := point hint((o.x, o.y + R))
    arm := line(o, pin)
    o distance(R) pin
    arm angle(theta) ref
    path := circle(center: o) hint(r: R)
    radius(R) path
    // the clevis pin's end, seen on
    kp := circle(center: pin) hint(r: rpin)
    radius(rpin) kp
    disc_axes := std.Turned(o, pin)
  }
  disc := components.disc.Disc(disc_axes)
}

// Open this file to preview the crank; `theta` is left unbound, so it is the preview's own
// unknown, and dragging the pin turns it.
preview {
  unit mm
  in std.front {
    ref := line(std.origin, hint((0, 1)))
    fix((0, 1)) ref.p2
  }
  crank := Crank(std.origin, ref)
}
