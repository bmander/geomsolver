// Continuous material example. Native sweep boundary export is not connected yet.
unit mm
use std (on_u)

component GeneratingCut(tool: solid, generating: motion, start: Angle, finish: Angle) {
  body := solid(tool, under: generating, from: start, to: finish)
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
  distance(2mm) diameter
  private meridian := arc(center: center, start: bottom, end: top)
  radius(1mm) meridian
}
construction tool := solid(face(meridian, diameter), about: diameter)
generating := motion(about: spindle)
in std.front {
  removal := GeneratingCut(tool, generating, start: -60deg, finish: 60deg)
}
