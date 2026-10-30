// A floor cut by a bar's flat end: a rectangular bar leaning out from upright, its end facing down
// toward the axis, turned about the block's axis from clear of one end to clear of the other. A
// turn moves each ruling of the bar as one affine field moves it, so its sides touch at one point
// a ruling; and its end, a plane, touches along the line where the plane through the axis and its
// normal meets it (docs/generating-sweeps.md, the constant-twist class, a prism's cap, #684 AE).
// The turn carries that line round into a cone, the cut's floor, falling outward; the bar's inner
// side touches along a line crossing its rulings, the wall; and the rim where they meet carries
// the contact from the one to the other round its fan, every normal between the side's and the
// end's. The section is turned `twist` in its plane so that no side faces the axis squarely: one
// that did would touch all along a ruling. The block loses what the bar's shadow covers, each
// point at radius r and height z where some point of the bar stands, read slice by slice.
unit mm
use std
use std.place (coords)
use std.motion (turns)

inner := 8mm         // the block: a ring sector about z, its radii, half height and angle
outer := 12mm
half := 1mm
sector := 60deg
reach := 10mm        // the bar: how far from the axis the middle of its end's inner rim stands,
tilt := 20deg        // how far it leans out from upright, its section turned `twist` in its plane,
twist := 5deg        // its section's depth from the inner side to the outer and its width across,
depth := 3mm         // and how far it stands up from its end
width := 4mm
length := 3mm

// the bar, drawn in the plane of its end, which faces down `tilt` toward the axis
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
  b0 coords(du: width / 2 * sin(twist), dv: -width / 2 * cos(twist)) holder
  b1 coords(du: depth * cos(twist) + width / 2 * sin(twist), dv: depth * sin(twist) - width / 2 * cos(twist)) holder
  b2 coords(du: depth * cos(twist) - width / 2 * sin(twist), dv: depth * sin(twist) + width / 2 * cos(twist)) holder
  b3 coords(du: -width / 2 * sin(twist), dv: width / 2 * cos(twist)) holder
  private lower := line(b0, b1)
  private outer_side := line(b1, b2)
  private upper := line(b2, b3)
  private inner_side := line(b3, b0)
}
construction bar := solid(face(lower, outer_side, upper, inner_side), from: 0mm, to: length)

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

// the turn about the block's axis, and the floor and wall it sweeps
swing := frame
swing turns(1) ax
construction groove := sweep(bar over t in (-140deg, 140deg), under: swing)
groove cut block
