// A knob turned about one line: a disc, a rod standing on it, and a ball on the rod's end, each a
// section turned about the spindle and joined into one body. Two rounds fill the rod's two roots:
// where it stands on the disc (a cylinder meeting a plane) and where it enters the ball (a
// cylinder meeting a sphere). The second is the same word as the first; that the faces are curved
// is read off the geometry, as whether the edge is concave is.
//
// The sections are drawn on the front plane, the spindle up its middle. Orbit (right-drag) to see
// the knob. Edit a radius and apply (⌘↵). Write `round(r: 1mm) knob.disc` to round the disc's own
// outer rims too, convex, so cut.
unit mm
use std (round)

in std.front {
  a0 := point
  a1 := point
  fix((0, 0)) a0
  fix((0, 60)) a1
  spindle := line(a0, a1)

  // the disc: 30 in radius, 8 thick
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

  // the rod: 4 in radius, from the disc's top to inside the ball
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

  // the ball: 8 in radius about a point 44 up the spindle, half a disc turned
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

knob.rod round(r: 2mm) knob.disc     // the root: a ring on the disc
knob.rod round(r: 1.5mm) knob.ball   // the neck: a ring where the rod enters the ball
