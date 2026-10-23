// The side view: what the assembly adds to the longitudinal section beyond its parts — the crank
// axis, the four pistons on the bore axes at the heights the end view gives their small ends, and
// the timing drive edge on.  The block, the head, the crankshaft and the rods are parts of their
// own, drawn in this view by the document.

use engine.dims (back, front, rcp)
use engine.end_view (bh, bt, idl_dy, pld, ridl, wf, xp0, xp1)
use engine.parts
use std (vertical)

// The timing drive on the front face, edge on, sized as the end view makes it
// (`engine.end_view`): the crank pulley its teeth between two guide flanges, the cam pulleys (one
// behind the other here) their teeth the pulley's whole width, each with its tooth roots dashed
// across it; the tensioner's roller; and the belt, its width from its lowest point, under the
// crank, to its highest, over the cams.
component DriveSide(o: point, cam: point) {
  ri := rcp - pld                // the crank pulley's tips, and a cam pulley's, twice its teeth
  rk := 2 * rcp - pld
  crank_near := engine.parts.Box(o, x0: xp0, y0: -(ri + bt + 2mm), x1: xp0 + wf, y1: ri + bt + 2mm)
  crank_teeth := engine.parts.Box(o, x0: xp0 + wf, y0: -ri, x1: xp1 - wf, y1: ri)
  crank_far := engine.parts.Box(o, x0: xp1 - wf, y0: -(ri + bt + 2mm), x1: xp1, y1: ri + bt + 2mm)
  crank_roots := engine.parts.Box(o, x0: xp0 + wf, y0: -(ri - bh), x1: xp1 - wf, y1: ri - bh)
  cam_teeth := engine.parts.Box(cam, x0: xp0 - front, y0: -rk, x1: xp1 - front, y1: rk)
  cam_roots := engine.parts.Box(cam, x0: xp0 - front, y0: -(rk - bh), x1: xp1 - front, y1: rk - bh)
  roller := engine.parts.Box(o, x0: xp0 + wf, y0: idl_dy - ridl, x1: xp1 - wf, y1: idl_dy + ridl)
  // the belt, a millimetre in from the crank pulley's flanges either side
  b0 := engine.parts.At(o, dx: xp0 + wf + 1mm, dy: -(ri + bt))
  b1 := engine.parts.At(o, dx: xp1 - wf - 1mm, dy: -(ri + bt))
  b2 := engine.parts.At(cam, dx: xp1 - wf - 1mm - front, dy: rk + bt)
  b3 := engine.parts.At(cam, dx: xp0 + wf + 1mm - front, dy: rk + bt)
  (under := line(b0.p, b1.p)) -> (beltb := line(b1.p, b2.p)) -> (over := line(b2.p, b3.p)) ->
    (beltf := line(b3.p, b0.p)) -> close
}

component SideSection(o: point) {
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
