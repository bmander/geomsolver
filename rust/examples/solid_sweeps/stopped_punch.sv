// A drum plunged aslant into a plate, stopping inside it: `punch.sv`'s drum lying on its side,
// pushed 8 mm up a line leaning toward its length, so its axis ends 2.63 mm up the plate's 4. No
// contact walk builds it: pushed aslant, the drum's flat ends lean to the push, and the push ends
// inside the plate, so where it stops is the drum's own side at the end pose. The kernel builds it on one arrangement (the closed class, #721): the plate split at once
// by every face the sweep's boundary can lie on — the drum where the push enters it at the start
// and leaves it at the end, its side's two lines square to the push swept into the planes tangent
// along them, each rim swept into an oblique cylinder — and each cell kept where the drum, carried
// back along the push, misses it. At each `y` across the drum its section is the rectangle 8 long
// and `2√(4 − y²)` high, swept along the push into a hexagon of which the plate keeps `0 ≤ z ≤ 4`;
// the plate loses that, integrated across the drum.
unit mm
use std
use std.solids
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
plate := solid(std.solids.Prism(face(near_side, far_end, far_side, near_end), from: 0mm, to: thick))

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
construction tool := solid(std.solids.Revolution(face(near_face, side, far_face, spindle), about: spindle))

// the push: 8 mm up a line leaning 0.3 toward x
private up := axis
fix(dir == (0.3, 0, 0.9539392014169457)) up
ram := frame
ram slides(8mm) up
construction slot := solid({ p | some t in (0deg, 360deg): p in tool at(under: ram) t })
slot cut plate
