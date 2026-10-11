// X0 (#183): solid_fillet_knob.sv, a knob turned about one axis: a disc, a rod standing on it, a
// ball on the rod. The rod's root (plane against cylinder) and its neck where it enters the ball
// (cylinder against sphere) are both filled. Does not parse yet.
unit mm
use std
in std.front {
  a0 := point
  a1 := point
  fix((0, 0)) a0
  fix((0, 60)) a1
  spindle := line(a0, a1)
  p0 := point
  p1 := point
  p2 := point
  p3 := point
  fix((0, 0)) p0
  fix((30, 0)) p1
  fix((30, 8)) p2
  fix((0, 8)) p3
  (dbot := line(p0, p1)) -> (dside := line(p1, p2)) -> (dtop := line(p2, p3)) ->
    (daxis := line(p3, p0)) -> close
  q0 := point
  q1 := point
  q2 := point
  q3 := point
  fix((0, 8)) q0
  fix((4, 8)) q1
  fix((4, 40)) q2
  fix((0, 40)) q3
  (rbot := line(q0, q1)) -> (rwall := line(q1, q2)) -> (rtop := line(q2, q3)) ->
    (raxis := line(q3, q0)) -> close
  c0 := point
  s0 := point
  s1 := point
  fix((0, 44)) c0
  fix((0, 36)) s0
  fix((0, 52)) s1
  shell := arc(center: c0, start: s0, end: s1) hint(r: 8)
  shut := line(s1, s0)
}
disc := solid(face(dbot, dside, dtop, daxis), about: spindle)
rod := solid(face(rbot, rwall, rtop, raxis), about: spindle)
ball := solid(face(shell, shut), about: spindle)
knob := solid(disc)
rod union knob
ball union knob

knob.rod round(2mm) knob.disc
knob.rod round(1.5mm) knob.ball
