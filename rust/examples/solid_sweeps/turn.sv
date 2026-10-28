// A groove swung across a curved block: a torus-tipped cutter leaning off upright, turned about
// the block's axis from clear of one end to clear of the other. A turn moves every point of the
// cutter the same way at every time, so the groove's wall is where the cutter's characteristic
// (the points of it moving along its own surface) is carried round: that curve laid in a
// half-plane through the axis and revolved (docs/generating-sweeps.md, the constant-twist class,
// #684). Leaning across the turn, the cutter's axis runs skew to the block's, and the wall is
// none of the torus's own circles. Keep `lean` clear of the front plane's tilt (a cutter whose
// axis meets the block's has a ring in contact all round, and is refused), and the roll clear of
// the block at both ends.
unit mm
use std
use std.place (coords)
use std.motion (turns)

inner := 8mm         // the block: a ring sector about z, its radii, half height and angle
outer := 12mm
half := 1mm
sector := 60deg
reach := 5mm         // the cutter: how far its axis stands from the block's, how far its tube's
ring := 2.5mm        // centre from its own axis, and the tube's radius
tube := 2mm
lean := 25deg        // how far the cutter's axis leans from upright, across the turn
from := -140deg      // the roll, from clear of the block to clear of it
to := 140deg

// the cutter, drawn in a plane through its axis, which leans `lean` across the turn
private across := axis
fix(dir == (1, 0, 0)) across
private along := axis
fix(dir == (0, sin(lean), cos(lean))) along
holder := plane(u: across, v: along)
fix(origin == (reach, 0, 0)) holder
in holder {
  private ta := point
  private tb := point
  ta coords(du: 0mm, dv: -1mm) holder
  tb coords(du: 0mm, dv: 1mm) holder
  private cutter_axis := line(ta, tb)
  private tc := point
  tc coords(du: ring, dv: 0mm) holder
  private tip := circle(center: tc) hint(r: 2)
  radius(tube) tip
}
construction cutter := solid(face(tip), about: cutter_axis)

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

// the turn about the block's axis, and the groove it sweeps
swing := frame
swing turns(1) ax
construction groove := sweep(cutter over t in (from, to), under: swing)
groove cut block
