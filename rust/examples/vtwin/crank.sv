// The crank train, seen along the axis: the shaft in its bearing behind the plate, the disc in
// front of it with the pin pressed in `R` from the axis, and the eye both rods share on the pin.
//
// The pin's angle is the drawing's one degree of freedom.  `theta` is defined nowhere, so it is a
// free variable — the instance's own unknown, `crank.theta` — and the arm's dimension reads it:
// the callout shows whatever angle the crank is at, and dragging the pin turns it.  Everything
// in both banks follows from where the pin is; `theta0` in the table is only where it starts.

use vtwin.dims
use vtwin.parts

component Crank(o: point, ref: line) {
  point pin hint(x: o.x + R * sin(theta0), y: o.y + R * cos(theta0))
  line arm(o, pin) class axis
  o distance(R) pin class shown at (0, -26)
  arm angle(theta) ref class shown at (0.35, 34)
  circle path(center: o) hint(r: R) class phantom
  radius(R) path
  circle disc(center: o) hint(r: rdisc)
  radius(rdisc) disc class shown at (-2.1, 32)
  circle shaft(center: o) hint(r: rshaft) class hidden
  radius(rshaft) shaft
  circle brg(center: o) hint(r: rbrg) class hidden
  radius(rbrg) brg
  circle kp(center: pin) hint(r: rpin)
  radius(rpin) kp
  circle eye(center: pin) hint(r: reye)
  radius(reye) eye
}
