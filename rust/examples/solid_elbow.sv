// A hollow duct elbow: carry one hollow section along a constrained arc.
unit mm
use std
param bend_radius = 30mm
param half_size = 9mm
param wall = 2mm
param bend_angle = 90deg

point center hint(x: bend_radius, y: 0mm)
std.origin horizontal center
std.origin distance(bend_radius) center
outer: CenteredRectangle(center, w: 2 * half_size, h: 2 * half_size)
inner: CenteredRectangle(center, w: 2 * (half_size - wall), h: 2 * (half_size - wall))
face annulus(outer.loop, holes: inner.loop)

// The guide is drawn in plan; the section stands perpendicular to its start tangent.
plane plan(origin: std.origin, toward: std.front.toward, from: std.front, fold: 0deg)
in plan {
  point turn_center hint(x: 0mm, y: 0mm)
  ground turn_center
  point entry hint(x: bend_radius, y: 0mm)
  point exit hint(x: bend_radius * cos(bend_angle), y: bend_radius * sin(bend_angle))
  construction centerline line inlet(turn_center, entry)
  construction centerline line outlet(turn_center, exit)
  horizontal inlet
  inlet angle(bend_angle) outlet
  construction centerline arc guide(center: turn_center, start: entry, end: exit)
  radius(bend_radius) guide
}
solid body(annulus, along: guide)
