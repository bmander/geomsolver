// A coil spring: a ball swept along a helix. The ball turns about the upright axis and climbs
// `pitch` every turn, for `turns` turns; the body is everything it passes through. Change a
// number and apply (⌘↵) to watch it refine again; ⌘B shows it in the glass box. Keep `wire_r`
// below half the pitch, or neighbouring turns run into each other.
unit mm
use std

coil_r := 12mm     // from the axis to the wire's centre
wire_r := 2mm      // the wire's radius
pitch := 8mm       // how far it climbs each turn
turns := 2         // how many turns

// A sphere of radius `r` about `center`: a half disc turned about its upright diameter.
component Sphere(center: point, r: Length) {
  private bottom := point hint(x: center.x, y: center.y - r)
  private top := point hint(x: center.x, y: center.y + r)
  private diameter := line(bottom, top)
  center midpoint diameter
  vertical diameter
  private meridian := arc(center: center, start: bottom, end: top) hint(r: r)
  radius(r) meridian
  body := solid(face(meridian, diameter), about: diameter)
}

construction centerline spindle := line(std.origin, std.up.toward)
private start := point hint(x: coil_r, y: 0)
std.origin horizontal start
std.origin distance(coil_r, along: right) start
private ball := Sphere(start, r: wire_r)

climb := motion(about: spindle, advance: pitch)
spring := solid(ball.body, under: climb, from: 0deg, to: turns * 360deg)
