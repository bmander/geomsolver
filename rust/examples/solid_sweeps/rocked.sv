// `turn.sv`'s groove cut by a cutter that rocks: the same torus-tipped cutter, leaning across the
// turn, swung out from `from` to `to` and back by a law of the roll, slowing to a stop at each end
// as a rocker driven by a cam does. A sweep is everything the cutter passes through, whatever the
// pace it is carried at or how often it passes: so a joint driven by a law sweeps what the joint
// does over the values the law takes, here the turn from `from` to `to`, and the groove is
// `turn.sv`'s, built exactly as a turn's (docs/generating-sweeps.md, a sweep is its path's, #684
// L). Edit the law — let it dwell, overshoot, or rock twice — and the groove is the turn over the
// least to the greatest value it reaches. Keep both clear of the block, as `turn.sv` says.
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
from := -140deg      // how far the cutter rocks, from clear of the block to clear of it
to := 140deg

param t: Angle over (0deg, 360deg)   // the roll the rocking is written over: once out and back

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

// the turn about the block's axis, rocked out and back by the law, and the groove it sweeps
swing := frame
swing turns(1) ax
fix(value == from + (to - from) * (1 - cos(t)) / 2) swing
construction groove := sweep(cutter over t, under: swing)
groove cut block
