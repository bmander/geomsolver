unit mm
use std
construction centerline line spindle(std.origin, std.up.toward)
private point t0 hint(x: 3, y: -0.8)
private point t1 hint(x: 4.5, y: 0)
private point t2 hint(x: 3, y: 0.8)
ground t0
ground t1
ground t2
private line e0(t0, t1)
private line e1(t1, t2)
private line e2(t2, t0)
construction solid tool(face(e0, e1, e2), from: -1.5mm, to: 1.5mm)
private point a0 hint(x: 2.5, y: 0)
a0 distance(2.5mm, along: u) std.front
a0 distance(0mm, along: v) std.front
private point a1 hint(x: 2.5, y: 5)
a1 distance(2.5mm, along: u) std.front
a1 distance(5mm, along: v) std.front
construction centerline line pivot(a0, a1)
motion turn(about: pivot)
solid swept(tool, under: turn, from: -50deg, to: 50deg)
