unit mm
use std
construction centerline line spindle(std.origin, std.up.toward)
private point b0 hint(x: 2, y: -3)
private point b1 hint(x: 4, y: -3)
private point b2 hint(x: 4, y: 0)
private point b3 hint(x: 2, y: 0)
ground b0
ground b1
ground b2
ground b3
private line e0(b0, b1)
private line e1(b1, b2)
private line e2(b2, b3)
private line e3(b3, b0)
construction solid tool(face(e0, e1, e2, e3), from: -1mm, to: 1mm)
private point h0 hint(x: 4, y: -1.5)
h0 distance(4mm, along: u) std.front
h0 distance(-1.5mm, along: v) std.front
private point h1 hint(x: 5, y: -1.5)
h1 distance(5mm, along: u) std.front
h1 distance(-1.5mm, along: v) std.front
construction centerline line hinge(h0, h1)
motion turn(about: hinge)
solid swept(tool, under: turn, from: 0deg, to: 30deg)
