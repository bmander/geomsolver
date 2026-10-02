// The end view: what the assembly adds to the section through cylinder 1 beyond its parts — the
// bore axis the crank and rods are placed against, and the timing drive on the front of the
// engine.  The block, the head, the crankshaft, the rods and the piston are parts of their own
// (`engine.block`, `engine.head`, `engine.crankshaft`, `engine.conrod`), drawn in this view by
// the document.

use engine.dims
use engine.parts

// The timing drive on the front of the engine: crank pulley, two cam pulleys, the belt over them.
component Drive(o: point, cam_i: point, cam_e: point, dims: group) {
  pcrank := circle(center: o) hint(r: dims.rcp)
  pcam_i := circle(center: cam_i) hint(r: dims.rcam)
  pcam_e := circle(center: cam_e) hint(r: dims.rcam)
  radius(dims.rcp) pcrank
  radius(dims.rcam) pcam_i
  radius(dims.rcam) pcam_e
  b1 := engine.parts.Span(pcrank, pcam_i, side: -1)
  b2 := engine.parts.Span(pcam_i, pcam_e, side: -1)
  b3 := engine.parts.Span(pcam_e, pcrank, side: -1)
}

component EndSection(o: point, dims: group) {
  top := engine.parts.At(o, dx: 0mm, dy: dims.deck + 30mm)
  bore := line(o, top.p)
}
