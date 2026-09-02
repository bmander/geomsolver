// The side view: what the assembly adds to the longitudinal section beyond its parts — the crank
// axis, the four pistons on the bore axes at the heights the end view gives their small ends, and
// the timing drive edge on.  The block, the head, the crankshaft and the rods are parts of their
// own, drawn in this view by the document.

use engine.dims
use engine.parts

// The timing drive on the front face, edge on: the pulleys are rectangles, the belt two lines.
component DriveSide(o: point, cam: point) {
  crankpulley: Box(o, x0: front - 55mm, y0: -rcp, x1: front - 30mm, y1: rcp)
  campulley: Box(cam, x0: -55mm, y0: -rcam, x1: -30mm, y1: rcam)
  line beltf(crankpulley.d, campulley.a) class belt
  line beltb(crankpulley.c, campulley.b) class belt
}

component SideSection(o: point) {
  a0: At(o, dx: front - 70mm, dy: 0mm)
  a1: At(o, dx: back + 60mm, dy: 0mm)
  line axisline(a0.p, a1.p) class axis
  // the four pistons on the pitch, each at the height its rod's small end is given
  repeat 4 as i {
    ax: At(o, dx: front + 25mm + P / 2 + i * P, dy: 0mm)
    port small: point hint(x: o.x + front + 25mm + P / 2 + i * P, y: o.y + R + L)
    ax.p distance(0, along: x) small
    piston: Piston(small, pin: 0)
  }
}
