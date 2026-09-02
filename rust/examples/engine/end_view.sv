// The end view: what the assembly adds to the section through cylinder 1 beyond its parts — the
// bore axis the crank and rods are placed against, and the timing drive on the front of the
// engine.  The block, the head, the crankshaft, the rods and the piston are parts of their own
// (`engine.block`, `engine.head`, `engine.crankshaft`, `engine.conrod`), drawn in this view by
// the document.

use engine.dims
use engine.parts

// The timing drive on the front of the engine: crank pulley, two cam pulleys, the belt over them.
component Drive(o: point, cam_i: point, cam_e: point) {
  circle pcrank(center: o) hint(r: rcp) class belt
  circle pcam_i(center: cam_i) hint(r: rcam) class belt
  circle pcam_e(center: cam_e) hint(r: rcam) class belt
  radius(rcp) pcrank
  radius(rcam) pcam_i
  radius(rcam) pcam_e
  b1: Span(pcrank, pcam_i, side: -1) class belt
  b2: Span(pcam_i, pcam_e, side: -1) class belt
  b3: Span(pcam_e, pcrank, side: -1) class belt
}

component EndSection(o: point) {
  top: At(o, dx: 0mm, dy: deck + 30mm)
  port bore = axisline
  line axisline(o, top.p) class axis
}
