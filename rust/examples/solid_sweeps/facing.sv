// A wall cut by a bar's side facing the axis squarely: a rectangular bar leaning out from upright
// in a plane through the axis, standing past the block above and below, turned about the block's
// axis from clear of one end to clear of the other. A turn moves each ruling of the bar as one
// affine field moves it, so a side touches at one point a ruling, as it crosses them; but the
// inner side faces the axis squarely, its normal in the plane through the axis and its rulings,
// so every ruling's contact is the same all along it, and the side touches along the one ruling
// where that is nought, the one in that plane, all along it (docs/generating-sweeps.md, the
// constant-twist class, a ruling in contact all along it, #684 AF). The turn carries that line
// round into a cone, the cut's wall: at height z it stands `reach + z tan(tilt)` from the axis,
// and the bar's outer side stands beyond the block at every height. So the block loses, within
// its 60°, `π/6 ∫ (outer² − (reach + z tan tilt)²) dz` over its heights:
// π/6 (88 − 2/3 tan² 20°) = 46.0306 mm³ of its π/3 · 80 = 83.7758, leaving 37.7452 mm³.
unit mm
use std
use std.place (coords)
use std.motion (turns)

inner := 8mm         // the block: a ring sector about z, its radii, half height and angle
outer := 12mm
half := 1mm
sector := 60deg
reach := 10mm        // the bar: how far from the axis its inner side crosses the block's middle,
tilt := 20deg        // how far it leans out from upright, its section's depth from the inner side
depth := 3mm         // to the outer and its width across, and how far it stands each way along
width := 4mm         // its length from the plane its section is drawn in
length := 3mm

// the bar, its section drawn square to its length, its inner side facing the axis
private across := axis
fix(dir == (cos(tilt), 0, -sin(tilt))) across
private along := axis
fix(dir == (0, 1, 0)) along
holder := plane(u: across, v: along)
fix(origin == (reach, 0, 0)) holder
in holder {
  private b0 := point
  private b1 := point
  private b2 := point
  private b3 := point
  b0 coords(du: 0mm, dv: -width / 2) holder
  b1 coords(du: depth, dv: -width / 2) holder
  b2 coords(du: depth, dv: width / 2) holder
  b3 coords(du: 0mm, dv: width / 2) holder
  private lower := line(b0, b1)
  private outer_side := line(b1, b2)
  private upper := line(b2, b3)
  private inner_side := line(b3, b0)
}
construction bar := solid(face(lower, outer_side, upper, inner_side), from: -length, to: length)

// the block: a ring about z in common with a wedge `sector` round from x
in std.front {
  construction centerline ax := line(std.origin, hint((0, 1)))
  fix((0, 1)) ax.p2
  private q0 := point
  private q1 := point
  private q2 := point
  private q3 := point
  q0 coords(du: inner, dv: -half) std.front
  q1 coords(du: outer, dv: -half) std.front
  q2 coords(du: outer, dv: half) std.front
  q3 coords(du: inner, dv: half) std.front
  private wall_in := line(q3, q0)
  private underside := line(q0, q1)
  private wall_out := line(q1, q2)
  private topside := line(q2, q3)
}
block := solid(face(wall_in, underside, wall_out, topside), about: ax)
in std.top {
  private w0 := point
  private w1 := point
  private w2 := point
  fix((0, 0)) w0
  w1 coords(du: 2 * outer, dv: 0mm) std.top
  w2 coords(du: 2 * outer * cos(sector), dv: 2 * outer * sin(sector)) std.top
  private wa := line(w0, w1)
  private wb := line(w1, w2)
  private wc := line(w2, w0)
}
construction wedge := solid(face(wa, wb, wc), from: -2 * half, to: 2 * half)
wedge common block

// the turn about the block's axis, and the wall it sweeps
swing := frame
swing turns(1) ax
construction groove := sweep(bar over t in (-140deg, 140deg), under: swing)
groove cut block
