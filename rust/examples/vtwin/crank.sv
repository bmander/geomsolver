// The crank train, seen along the axis: the pin `R` from the axis on the disc (`vtwin.disc`),
// and the arm from the axis to it.
//
// The pin's angle is the drawing's one degree of freedom.  `theta` is defined nowhere, so it is a
// free variable — the instance's own unknown, `crank.theta` — and the arm's dimension reads it:
// the callout shows whatever angle the crank is at, and dragging the pin turns it.  Everything
// in both banks follows from where the pin is; `theta0` in the table is only where it starts.

use vtwin.dims
use vtwin.parts
use vtwin.disc

component Crank(swing: plane, side: plane, top: plane, o: point, ref: line, o_s: point, o_t: point) {
  in swing {
    point pin hint(x: o.x + R * sin(theta0), y: o.y + R * cos(theta0))
    line arm(o, pin) class axis
    o distance(R) pin class shown at (0, -26)
    arm angle(theta) ref class shown at (0.35, 34)
    circle path(center: o) hint(r: R) class phantom
    radius(R) path
    // the clevis pin's end, seen on
    circle kp(center: pin) hint(r: rpin)
    radius(rpin) kp
  }
  disc: Disc(swing, side, top, o, pin, arm, dir: 90deg - theta0, o_s: o_s, o_t: o_t,
             draw_side: 0, draw_top: 0)
}
