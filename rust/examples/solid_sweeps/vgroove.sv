// A V groove cut by a V wheel: a wheel with a sharp 60° rim, its axis lying across a block, slid
// along the block square to that axis from clear of one end to clear of the other. A slide moves
// every point of the wheel the same way at every time, so each flank touches where it moves along
// itself, its meridian in the plane through the wheel's axis square to the slide; and the two
// flanks' meridians meet the rim at one point, where the rim runs along the slide, so every normal
// between the flanks is the wheel's there. The groove's walls are the flanks' meridians slid, and
// they meet in a sharp edge along that point's path, the groove's root (docs/generating-sweeps.md,
// the constant-twist class, a sharp edge along one point's path, #684 V). The block loses the V's
// section, `depth² × tan(flank)`, along its length: 4 tan 30° × 20 = 46.188 mm³ of its 2000,
// leaving 1953.812 mm³.
unit mm
use std
use std.place (coords)
use std.motion (slides)

length := 20mm       // the block: its length along the slide, its width across and its height
width := 10mm
height := 10mm
wheel := 6mm         // the wheel: from its axis to its rim, how far the rim dips into the block,
depth := 2mm         // each flank's angle from the wheel's plane, and how deep the V is, from the
flank := 30deg       // rim to the wheel's bore
vee := 3mm

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
block := solid(face(near_side, far_end, far_side, near_end), from: 0mm, to: height)

// the wheel, drawn in a plane through its axis, which lies across the block a little before it:
// out from the axis downward, the rim toward the block, and along the axis
private down := axis
fix(dir == (0, 0, -1)) down
private along := axis
fix(dir == (0, 1, 0)) along
bench := plane(u: down, v: along)
fix(origin == (-8mm, 0, height - depth + wheel)) bench
in bench {
  private a0 := point
  private a1 := point
  a0 coords(du: 0mm, dv: -1mm) bench
  a1 coords(du: 0mm, dv: 1mm) bench
  private wheel_axis := line(a0, a1)
  private v0 := point
  private v1 := point
  private v2 := point
  v0 coords(du: wheel, dv: 0mm) bench
  v1 coords(du: wheel - vee, dv: -vee * tan(flank)) bench
  v2 coords(du: wheel - vee, dv: vee * tan(flank)) bench
  private lower := line(v0, v1)
  private bore := line(v1, v2)
  private upper := line(v2, v0)
}
construction vwheel := solid(face(lower, bore, upper), about: wheel_axis)

// the slide along the block, 36 mm over the roll, from clear before it to clear past it
private way := axis
fix(dir == (1, 0, 0)) way
feed := frame
feed slides(36mm) way
construction groove := sweep(vwheel over t in (0deg, 360deg), under: feed)
groove cut block
