// The side view: what the assembly adds to the longitudinal section beyond its parts — the crank
// axis, the four pistons on the bore axes at the heights the end view gives their small ends, and
// the timing drive edge on.  The block, the head, the crankshaft and the rods are parts of their
// own, drawn in this view by the document.

use engine.dims
use engine.parts

// The timing drive on the front face, edge on: the pulleys are rectangles, the belt two lines.
component DriveSide(o: point, cam: point, dims: group) {
  crankpulley := Box(o, x0: dims.front - 55mm, y0: -dims.rcp, x1: dims.front - 30mm, y1: dims.rcp)
  campulley := Box(cam, x0: -55mm, y0: -dims.rcam, x1: -30mm, y1: dims.rcam)
  beltf := line(crankpulley.d, campulley.a)
  beltb := line(crankpulley.c, campulley.b)
}

component SideSection(o: point, dims: group) {
  a0 := At(o, dx: dims.front - 70mm, dy: 0mm)
  a1 := At(o, dx: dims.back + 60mm, dy: 0mm)
  axisline := line(a0.p, a1.p)
  // the four pistons on the pitch, each at the height its rod's small end is given
  repeat 4 as i {
    ax := At(o, dx: dims.front + 25mm + dims.P / 2 + i * dims.P, dy: 0mm)
    small := point hint(x: o.x + dims.front + 25mm + dims.P / 2 + i * dims.P, y: o.y + dims.R + dims.L)
    ax.p distance(0, along: x) small
    piston := Piston(small, pin: 0, dims: dims)
  }
}
