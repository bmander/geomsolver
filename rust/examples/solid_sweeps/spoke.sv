// A slot swung into a curved block: a pin lying along a radius of the turn, its axis meeting the
// block's at the centre, swung about the block's axis from clear of one end to clear of the
// other. A turn moves each point square to the plane through it and the axis, so every ring of
// the pin (a circle about its own axis) touches at its two points in the plane of the two axes:
// its side along its top and bottom lines, which the turn carries round into the slot's flat roof
// and floor. One ring touches all round: on the rim of the pin's end every normal between its side
// and its end face is the pin's, and the ring whose normals pass through the centre lies on the
// sphere about the centre, which the turn keeps, and sweeps a zone of it, the slot's end wall
// (docs/generating-sweeps.md, the constant-twist class, #684 X). The branch beyond that ring,
// across the end face, runs within the zone and is trimmed. The slot's section in a half-plane
// through the axis is the band |z| ≤ radius from the block's inner wall out to that sphere, of
// radius √(reach² + radius²), turned through the block's 60°:
// π/6 (2 radius (reach² + radius² − inner²) − 2 radius³ / 3) = π/6 (57.25 − 1/12) ≈ 29.93 mm³ of
// the block's 83.78, leaving 53.84 mm³.
unit mm
use std
use std.place (coords)
use std.motion (turns)

inner := 8mm         // the block: a ring sector about z, its radii, half height and angle
outer := 12mm
half := 1mm
sector := 60deg
start := 6mm         // the pin: where it starts and ends along its radius, and its radius
reach := 11mm
radius := 0.5mm
from := -140deg      // the roll, from clear of the block to clear of it
to := 140deg

// the pin, drawn in the plane through its axis, which runs out along x through the centre
private lift := axis
fix(dir == (0, 0, 1)) lift
private out := axis
fix(dir == (1, 0, 0)) out
holder := plane(u: out, v: lift)
fix(origin == (0, 0, 0)) holder
in holder {
  private e0 := point
  private e1 := point
  private e2 := point
  private e3 := point
  e0 coords(du: start, dv: 0mm) holder
  e1 coords(du: start, dv: radius) holder
  e2 coords(du: reach, dv: radius) holder
  e3 coords(du: reach, dv: 0mm) holder
  private base := line(e0, e1)
  private side := line(e1, e2)
  private tip := line(e2, e3)
  private spindle := line(e3, e0)
}
construction pin := solid(face(base, side, tip, spindle), about: spindle)

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

// the turn about the block's axis, and the slot it sweeps
swing := frame
swing turns(1) ax
construction slot := sweep(pin over t in (from, to), under: swing)
slot cut block
