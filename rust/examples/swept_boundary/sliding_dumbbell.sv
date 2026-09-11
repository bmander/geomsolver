unit mm
use std
construction centerline line spindle(std.origin, std.up.toward)
private point c0 hint(x: 3, y: -1)
private point c1 hint(x: 3.25, y: -1)
private point c2 hint(x: 3.25, y: 1)
private point c3 hint(x: 3, y: 1)
ground c0
ground c1
ground c2
ground c3
private line bottom(c0, c1)
private line wall(c1, c2)
private line top(c2, c3)
private line axis(c3, c0)
construction solid bar(face(bottom, wall, top, axis), about: axis)
construction solid tool(bar)
private point ca hint(x: 3, y: 1)
ca distance(3mm, along: u) std.front
ca distance(1mm, along: v) std.front
private point ba hint(x: 3, y: 0.5)
private point ta hint(x: 3, y: 1.5)
private line da(ba, ta)
ca midpoint da
da parallel spindle
private arc ma(center: ca, start: ba, end: ta)
radius(0.5mm) ma
construction solid ball_a(face(ma, da), about: da)
ball_a on tool
private point cb hint(x: 3, y: -1)
cb distance(3mm, along: u) std.front
cb distance(-1mm, along: v) std.front
private point bb hint(x: 3, y: -1.5)
private point tb hint(x: 3, y: -0.5)
private line db(bb, tb)
cb midpoint db
db parallel spindle
private arc mb(center: cb, start: bb, end: tb)
radius(0.5mm) mb
construction solid ball_b(face(mb, db), about: db)
ball_b on tool
private point rail_end hint(x: 10, y: 0)
rail_end distance(10mm, along: u) std.front
rail_end distance(0mm, along: v) std.front
construction centerline line rail(std.origin, rail_end)
motion feed(along: rail, advance: 4mm)
solid swept(tool, under: feed, from: 0deg, to: 360deg)
