// Continuous material example. Native sweep boundary export is not connected yet.
unit mm
use std

component GeneratingCut(tool: solid, generating: motion, start: Angle, finish: Angle) {
  solid body(tool, under: generating, from: start, to: finish)
}

construction centerline line spindle(std.origin, std.up.toward)
private point center
center distance(3mm, along: u) std.front
center distance(0mm, along: v) std.front
private point bottom hint(x: 3, y: -1)
private point top hint(x: 3, y: 1)
private line diameter(bottom, top)
center midpoint diameter
diameter parallel spindle
distance(2mm) diameter
private arc meridian(center: center, start: bottom, end: top)
radius(1mm) meridian
construction solid tool(face(meridian, diameter), about: diameter)
motion generating(about: spindle)
removal: GeneratingCut(tool, generating, start: -60deg, finish: 60deg)
