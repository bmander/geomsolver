// Continuous material: a ball swept through a turn about the spindle, everything it passes through
// — a stretch of a torus with the ball at each end, built exactly.
unit mm
use std (on_u)

component GeneratingCut(tool: solid, generating: frame, start: Angle, finish: Angle) {
  body := sweep(tool over t in (start, finish), under: generating)
}

in std.front {
  construction centerline spindle := line(std.origin, hint((0, 1)))
  fix((0, 1)) spindle.p2
  private center := point
  center on_u(d: 3mm) std.front
  private bottom := point hint((3, -1))
  private top := point hint((3, 1))
  private diameter := line(bottom, top)
  center midpoint diameter
  diameter parallel spindle
  length(2mm) diameter
  private meridian := arc(center: center, start: bottom, end: top)
  radius(1mm) meridian
}
construction tool := solid(face(meridian, diameter), about: diameter)
generating := frame
std.Turn(generating, about: spindle, ratio: 1)
in std.front {
  removal := GeneratingCut(tool, generating, start: -60deg, finish: 60deg)
}
