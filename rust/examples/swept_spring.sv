// A coil spring: a ball swept along a helix. The ball turns about the upright axis and climbs
// `pitch` every turn, for `turns` turns; the body is everything it passes through. Change a
// number and apply (⌘↵) to watch it refine again; ⌘B shows it in the glass box. Keep `wire_r`
// below half the pitch, or neighbouring turns run into each other.
unit mm
use std

param coil_r = 12mm     // from the axis to the wire's centre
param wire_r = 2mm      // the wire's radius
param pitch = 8mm       // how far it climbs each turn
param turns = 2         // how many turns

// A sphere of radius `r` about `center`: a half disc turned about its upright diameter.
component Sphere(center: point, r: Length) {
  private point bottom hint(x: center.x, y: center.y - r)
  private point top hint(x: center.x, y: center.y + r)
  private line diameter(bottom, top)
  center midpoint diameter
  vertical diameter
  private arc meridian(center: center, start: bottom, end: top) hint(r: r)
  radius(r) meridian
  solid body(face(meridian, diameter), about: diameter)
}

construction centerline line spindle(std.origin, std.up.toward)
private point start hint(x: coil_r, y: 0)
std.origin horizontal start
std.origin distance(coil_r, along: right) start
private ball: Sphere(start, r: wire_r)

motion climb(about: spindle, advance: pitch)
solid spring(ball.body, under: climb, from: 0deg, to: turns * 360deg)
