// Continuous material example. Native sweep boundary export is not connected yet.
unit mm
use std

component GeneratingCut(tool: solid, generating: motion, start: Angle, finish: Angle) {
  body := solid(tool, under: generating, from: start, to: finish)
}

construction centerline spindle := line(std.origin, std.up.toward)
private center := point
center distance(3mm, along: u) std.front
center distance(0mm, along: v) std.front
private bottom := point hint(x: 3, y: -1)
private top := point hint(x: 3, y: 1)
private diameter := line(bottom, top)
center midpoint diameter
diameter parallel spindle
distance(2mm) diameter
private meridian := arc(center: center, start: bottom, end: top)
radius(1mm) meridian
construction tool := solid(face(meridian, diameter), about: diameter)
generating := motion(about: spindle)
removal := GeneratingCut(tool, generating, start: -60deg, finish: 60deg)
