// A knob turned about one axis (issue #66, rung 2): a disc, a rod standing on it and a ball on the
// rod. The rod's root is a ball rolled between a plane and a cylinder; its neck, where it enters the
// ball, between a cylinder and a sphere — a section with a curved side, its face a torus still.
unit mm
a0 := point
a1 := point hint(x: 0, y: 60)
fix(x == 0, y == 0) a0
fix(x == 0, y == 60) a1
axis := line(a0, a1)
p0 := point
p1 := point hint(x: 30, y: 0)
p2 := point hint(x: 30, y: 8)
p3 := point hint(x: 0, y: 8)
fix(x == 0, y == 0) p0
fix(x == 30, y == 0) p1
fix(x == 30, y == 8) p2
fix(x == 0, y == 8) p3
(dbot := line(p0, p1)) -> (dside := line(p1, p2)) -> (dtop := line(p2, p3)) -> (daxis := line(p3, p0)) -> close
q0 := point hint(x: 0, y: 8)
q1 := point hint(x: 4, y: 8)
q2 := point hint(x: 4, y: 40)
q3 := point hint(x: 0, y: 40)
fix(x == 0, y == 8) q0
fix(x == 4, y == 8) q1
fix(x == 4, y == 40) q2
fix(x == 0, y == 40) q3
(rbot := line(q0, q1)) -> (rwall := line(q1, q2)) -> (rtop := line(q2, q3)) -> (raxis := line(q3, q0)) -> close
c0 := point hint(x: 0, y: 44)
s0 := point hint(x: 0, y: 36)
s1 := point hint(x: 0, y: 52)
fix(x == 0, y == 44) c0
fix(x == 0, y == 36) s0
fix(x == 0, y == 52) s1
shell := arc(center: c0, start: s0, end: s1) hint(r: 8)
shut := line(s1, s0)
disc := solid(face(dbot, dside, dtop, daxis), about: axis)
rod := solid(face(rbot, rwall, rtop, raxis), about: axis)
ball := solid(face(shell, shut), about: axis)
knob := solid(disc)
rod union knob
ball union knob
root := fillet(rod, disc, r: 2mm)
root union knob
neck := fillet(rod, ball, r: 1.5mm)
neck union knob
