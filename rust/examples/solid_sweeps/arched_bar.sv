// An arched bar pushed aslant through a plate: a bar 10 mm long whose section is an arch, a cubic
// spline over a 2 mm chord, pushed down through the plate along a line leaning 30° from upright
// toward the bar's length, from clear above it to clear below. A slide moves every point of the bar
// the same way, so the points in contact (where the bar's normal stands square to the push) are a
// set fixed on the bar: on the arch's side the two lines along it where its tangent stands upright,
// and round its sharp edges the stretches where the faces either side meet the push oppositely;
// with the bar at both ends of the push beside them, they bound what it sweeps. The arch's side is
// a spline swept along a line, which no other reading of a tool takes: read on the whole bar, each
// piece of contact is carried exactly along the push, every meeting of two of them is given by
// construction, and what the bar pushed back holds is trimmed away (the contact's road,
// docs/generating-sweeps.md, #880 Q1b). The bar passes wholly through the plate, so each level of
// the plate loses the bar's shadow along the push: across it at each `y`, the bar's length and the
// arch's height there leaned back by the push's slope, so the shadow is `L W + tan 30° A`, `W` the
// arch's width (2√2, between the points where its tangent stands upright) and `A` its area (3.6):
// the plate loses 4 × (20√2 + 3.6 tan 30°) ≈ 121.45 mm³ of its 3200, leaving 3078.55 mm³.
unit mm
use std
use std.solids (cut)
use std.place (coords)
use std.motion (slides)

length := 40mm       // the plate: its length and width, and its thickness
width := 20mm
thick := 4mm
bar := 10mm          // the bar: its length, and the push's lean from upright
lean := 30deg

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

// the bar's section, drawn square to its length a millimetre above the plate: the arch's spline
// through its four control points, and the chord under it
private across := axis
fix(dir == (0, 1, 0)) across
private up := axis
fix(dir == (0, 0, 1)) up
bench := plane(u: across, v: up)
fix(origin == (8mm, 0, thick + 1mm)) bench
in bench {
  private k0 := point
  private k1 := point
  private k2 := point
  private k3 := point
  k0 coords(du: -1mm, dv: 0mm) bench
  k1 coords(du: -3mm, dv: 2mm) bench
  k2 coords(du: 3mm, dv: 2mm) bench
  k3 coords(du: 1mm, dv: 0mm) bench
  private arch := spline(k0, k1, k2, k3)
  private chord := line(k3, k0)
}
construction arched := solid(std.solids.Prism(face(arch, chord), from: 0mm, to: bar))

// the push, 10 mm along a line leaning `lean` from straight down toward the bar's length
private push := axis
fix(dir == (sin(lean), 0, -cos(lean))) push
ram := frame
ram slides(10mm) push
construction slot := solid({ p | some t in (0deg, 360deg): p in arched at(under: ram) t })
slot cut plate
