// The end view: what the assembly adds to the section through cylinder 1 beyond its parts — the
// bore axis the crank and rods are placed against, and the timing drive on the front of the
// engine.  The block, the head, the crankshaft, the rods and the piston are parts of their own
// (`engine.block`, `engine.head`, `engine.crankshaft`, `engine.conrod`), drawn in this view by
// the document.

use engine.dims (deck, front, rcam, rcp)
use engine.parts
use std

// The timing drive on the front of the engine: crank pulley, two cam pulleys, the belt over them.
part Drive(o: point, cam_i: point, cam_e: point) {
  pcrank := circle(center: o) hint(r: rcp)
  pcam_i := circle(center: cam_i) hint(r: rcam)
  pcam_e := circle(center: cam_e) hint(r: rcam)
  radius(rcp) pcrank
  radius(rcam) pcam_i
  radius(rcam) pcam_e
  b1 := engine.parts.Span(pcrank, pcam_i, side: -1)
  b2 := engine.parts.Span(pcam_i, pcam_e, side: -1)
  b3 := engine.parts.Span(pcam_e, pcrank, side: -1)
  // the pulleys as solids, forward of the block's front face where the side view puts them
  crank_pulley := solid(face(pcrank), from: front - 55mm, to: front - 30mm)
  cam_pulley_i := solid(face(pcam_i), from: front - 55mm, to: front - 30mm)
  cam_pulley_e := solid(face(pcam_e), from: front - 55mm, to: front - 30mm)
}

part EndSection(o: point) {
  top := engine.parts.At(o, dx: 0mm, dy: deck + 30mm)
  bore := line(o, top.p)
}

// Open this file to preview what the end view adds: the bore axis, and the timing drive on the
// crank and on the two camshafts where the head carries them, its pulleys as solids.
preview {
  unit mm
  in std.side {
    end := EndSection(std.side.origin)
    cam_i := engine.parts.At(std.side.origin, dx: engine.dims.camx,
                             dy: engine.dims.camh + engine.dims.gasket)
    cam_e := engine.parts.At(std.side.origin, dx: -engine.dims.camx,
                             dy: engine.dims.camh + engine.dims.gasket)
    drive := Drive(std.side.origin, cam_i.p, cam_e.p)
  }
}
