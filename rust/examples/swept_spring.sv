// A coil spring: a ball swept along a helix. The ball turns about the upright axis and climbs
// `pitch` every turn, for `turns` turns; the body is everything it passes through — the ball's
// disc carried along the helix, with the ball at each end — built exactly. ⌘B shows it in the
// glass box. Keep `wire_r` below half the pitch, or neighbouring turns run into each other (and
// the spring is read by its field instead).
unit mm
use std (right_of, screws)

coil_r := 12mm     // from the axis to the wire's centre
wire_r := 2mm      // the wire's radius
pitch := 8mm       // how far it climbs each turn
turns := 2         // how many turns

// A sphere of radius `r` about `center`: a half disc turned about its upright diameter.
component Sphere(center: point, r: Length) {
  private bottom := point hint((center.x, center.y - r))
  private top := point hint((center.x, center.y + r))
  private diameter := line(bottom, top)
  center midpoint diameter
  vertical diameter
  private meridian := arc(center: center, start: bottom, end: top) hint(r: r)
  radius(r) meridian
  body := solid(face(meridian, diameter), about: diameter)
}

in std.front {
  construction centerline spindle := line(std.origin, hint((0, 1)))
  fix((0, 1)) spindle.p2
  private start := point hint((coil_r, 0))
  start right_of(d: coil_r) std.origin
  private ball := Sphere(start, r: wire_r)
}

climb := frame
climb screws(pitch) spindle
spring := sweep(ball.body over t in (0deg, turns * 360deg), under: climb)
