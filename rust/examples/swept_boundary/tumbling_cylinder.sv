unit mm
use std
construction centerline line spindle(std.origin, std.up.toward)
private point c0 hint(x: 3, y: -1)
private point c1 hint(x: 4, y: -1)
private point c2 hint(x: 4, y: 1)
private point c3 hint(x: 3, y: 1)
ground c0
ground c1
ground c2
ground c3
private line bottom(c0, c1)
private line wall(c1, c2)
private line top(c2, c3)
private line axis(c3, c0)
construction solid tool(face(bottom, wall, top, axis), about: axis)
private point hub hint(x: 3, y: 0)
hub distance(3mm, along: u) std.front
hub distance(0mm, along: v) std.front
private point hub_out hint(x: 4, y: 0)
hub_out distance(4mm, along: u) std.front
hub_out distance(0mm, along: v) std.front
construction centerline line tumbler(hub, hub_out)
motion turn(about: tumbler)
solid swept(tool, under: turn, from: -30deg, to: 30deg)
