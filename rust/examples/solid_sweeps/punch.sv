// A slot punched through a plate: a drum lying on its side, its flat ends upright, pushed up
// through a plate from clear below it to clear above. A slide moves every point of the drum the
// same way at every time, so the slot's walls are where the drum's side moves along itself, the
// two lines along it square to the slide, and its flat ends, which the slide keeps in their own
// planes, touch all over, each sweeping a strip of its plane: its diameter square to the slide,
// carried (docs/generating-sweeps.md, the constant-twist class). The drum passes wholly through
// the plate, so its characteristic closes on itself (#684 W): along one line, across one end,
// back along the other and across the other end, a rectangle carried up as a closed tube, creased
// at its four corners. The plate loses the drum's shadow along the slide, `length × 2 radius`,
// through its thickness: 8 × 4 × 4 = 128 mm³ of its 800, leaving 672 mm³.
unit mm
use std
use std.place (coords)
use std.motion (slides)

length := 20mm       // the plate: its length and width, and its thickness
width := 10mm
thick := 4mm
radius := 2mm        // the drum: its radius and its length
drum := 8mm

in std.top {
  private b0 := point
  private b1 := point
  private b2 := point
  private b3 := point
  b0 coords(du: 0mm, dv: -width / 2) std.top
  b1 coords(du: length, dv: -width / 2) std.top
  b2 coords(du: length, dv: width / 2) std.top
  b3 coords(du: 0mm, dv: width / 2) std.top
  private near_side := line(b0, b1)
  private far_end := line(b1, b2)
  private far_side := line(b2, b3)
  private near_end := line(b3, b0)
}
plate := solid(face(near_side, far_end, far_side, near_end), from: 0mm, to: thick)

// the drum, drawn in a plane through its axis, which lies along the plate below its middle
private across := axis
fix(dir == (0, 1, 0)) across
private along := axis
fix(dir == (1, 0, 0)) along
bench := plane(u: across, v: along)
fix(origin == (length / 2 - drum / 2, 0, -5mm)) bench
in bench {
  private e0 := point
  private e1 := point
  private e2 := point
  private e3 := point
  e0 coords(du: 0mm, dv: 0mm) bench
  e1 coords(du: radius, dv: 0mm) bench
  e2 coords(du: radius, dv: drum) bench
  e3 coords(du: 0mm, dv: drum) bench
  private near_face := line(e0, e1)
  private side := line(e1, e2)
  private far_face := line(e2, e3)
  private spindle := line(e3, e0)
}
construction tool := solid(face(near_face, side, far_face, spindle), about: spindle)

// the push up through the plate, 16 mm over the roll, from clear below it to clear above
private up := axis
fix(dir == (0, 0, 1)) up
ram := frame
ram slides(16mm) up
construction slot := sweep(tool over t in (0deg, 360deg), under: ram)
slot cut plate
