// The side view: what the assembly adds to the longitudinal section beyond its parts — the crank
// axis, the four pistons on the bore axes at the heights the end view gives their small ends, and
// the timing drive edge on.  The block, the head, the crankshaft and the rods are parts of their
// own, drawn in this view by the document.

use engine.dims (back, front, rcam, rcp)
use engine.parts
use std (vertical)

// The timing drive on the front face, edge on: the pulleys are rectangles, the belt two lines.
part DriveSide(o: point, cam: point) {
  crankpulley := engine.parts.Box(o, x0: front - 55mm, y0: -rcp, x1: front - 30mm, y1: rcp)
  campulley := engine.parts.Box(cam, x0: -55mm, y0: -rcam, x1: -30mm, y1: rcam)
  beltf := line(crankpulley.d, campulley.a)
  beltb := line(crankpulley.c, campulley.b)
}

part SideSection(o: point) {
  a0 := engine.parts.At(o, dx: front - 70mm, dy: 0mm)
  a1 := engine.parts.At(o, dx: back + 60mm, dy: 0mm)
  axisline := line(a0.p, a1.p)
}

// Open this file to preview what the side view adds: the crank axis, and the timing drive edge on
// from the crank to the camshafts where the head carries them.
preview {
  unit mm
  in std.front {
    side := SideSection(std.origin)
    cam := engine.parts.At(std.origin, dx: front, dy: engine.dims.camh + engine.dims.gasket)
    drive := DriveSide(std.origin, cam.p)
  }
}
