// A hollow duct elbow: carry one hollow section along a constrained arc.
unit mm
use std
bend_radius := 30mm
half_size := 9mm
wall := 2mm
bend_angle := 90deg

in std.front {
  center := point hint((bend_radius, 0mm))
  std.origin horizontal center
  std.origin distance(bend_radius) center
  outer := std.CenteredRectangle(center, w: 2 * half_size, h: 2 * half_size)
  inner := std.CenteredRectangle(center, w: 2 * (half_size - wall), h: 2 * (half_size - wall))
  annulus := face(outer.loop, holes: inner.loop)
}

// The guide is drawn in plan, the top plane; the section stands perpendicular to its start tangent.
in std.top {
  turn_center := point
  fix((0mm, 0mm)) turn_center
  entry := point hint((bend_radius, 0mm))
  exit := point hint((bend_radius * cos(bend_angle), bend_radius * sin(bend_angle)))
  construction centerline inlet := line(turn_center, entry)
  construction centerline outlet := line(turn_center, exit)
  horizontal inlet
  inlet angle(bend_angle) outlet
  construction centerline guide := arc(center: turn_center, start: entry, end: exit)
  radius(bend_radius) guide
}
body := solid(annulus, along: guide)
