// A hole drilled through a plate: a drill, a cylinder ground to a 118° point, pushed down along
// its own axis from clear above the plate to clear below. A slide along the tool's own axis moves
// every point of it straight along that axis, so each ring of the drill (a circle about its axis)
// meets the motion at one angle all round: it touches nowhere, or, where its normal lies square to
// the slide, all round. The drill's side is such rings, at its greatest radius, and the ring of
// greatest radius sweeps the cylinder of that radius, the hole's wall (docs/generating-sweeps.md,
// the constant-twist class, #684 X). The drill passes wholly through the plate and its shadow along
// the slide is its whole disc, so the plate loses that cylinder through its thickness, whatever
// the point: π radius² thick = π × 2² × 4 = 16π ≈ 50.27 mm³ of its 800, leaving 749.73 mm³.
unit mm
use std
use std.place (coords)
use std.motion (slides)

length := 20mm       // the plate: its length and width, and its thickness
width := 10mm
thick := 4mm
radius := 2mm        // the drill: its radius, its length, and its point's included angle
reach := 8mm
included := 118deg

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

// the drill, drawn in a plane through its axis, which stands upright over the plate's middle,
// its point 2 above the plate
private across := axis
fix(dir == (1, 0, 0)) across
private up := axis
fix(dir == (0, 0, 1)) up
bench := plane(u: across, v: up)
fix(origin == (length / 2, 0, thick + 2mm)) bench
in bench {
  private tip := point
  private lip := point
  private shank := point
  private back := point
  tip coords(du: 0mm, dv: 0mm) bench
  lip coords(du: radius, dv: radius / tan(included / 2)) bench
  shank coords(du: radius, dv: reach) bench
  back coords(du: 0mm, dv: reach) bench
  private cone := line(tip, lip)
  private side := line(lip, shank)
  private end := line(shank, back)
  private spindle := line(back, tip)
}
construction drill := solid(face(cone, side, end, spindle), about: spindle)

// the push down through the plate, 20 mm over the roll, from clear above it to clear below
private down := axis
fix(dir == (0, 0, -1)) down
ram := frame
ram slides(20mm) down
construction hole := sweep(drill over t in (0deg, 360deg), under: ram)
hole cut plate
