// A hollow duct that narrows as it bends: an inlet and an outlet lofted along a constrained arc,
// each section blended into the next while it turns about the bend's axis.
unit mm
use std
bend_radius := 30mm
inlet_half := 9mm
outlet_half := 6mm
wall := 2mm
bend_angle := 90deg

center := point hint(x: bend_radius, y: 0mm)
std.origin horizontal center
std.origin distance(bend_radius) center
outer := std.CenteredRectangle(center, w: 2 * inlet_half, h: 2 * inlet_half)
inner := std.CenteredRectangle(center, w: 2 * (inlet_half - wall), h: 2 * (inlet_half - wall))
inlet := face(outer.loop, holes: inner.loop)

// The guide is drawn in plan; each section stands perpendicular to its tangent at its end.
plan := plane(origin: std.origin, toward: std.front.toward, from: std.front, fold: 0deg)
in plan {
  turn_center := point hint(x: 0mm, y: 0mm)
  ground turn_center
  entry := point hint(x: bend_radius, y: 0mm)
  exit := point hint(x: bend_radius * cos(bend_angle), y: bend_radius * sin(bend_angle))
  construction centerline inlet_axis := line(turn_center, entry)
  construction centerline outlet_axis := line(turn_center, exit)
  horizontal inlet_axis
  inlet_axis angle(bend_angle) outlet_axis
  construction centerline guide := arc(center: turn_center, start: entry, end: exit)
  radius(bend_radius) guide
}

// The outlet's plane holds the bend's axis and the radius to the exit.
outlet_view := plane(origin: std.origin, toward: std.front.toward, u: (0, 1, 0), v: (0, 0, 1))
in outlet_view {
  out_center := point hint(x: bend_radius, y: 0mm)
  ground out_center
  out_outer := std.CenteredRectangle(out_center, w: 2 * outlet_half, h: 2 * outlet_half)
  out_inner := std.CenteredRectangle(out_center, w: 2 * (outlet_half - wall), h: 2 * (outlet_half - wall))
}
outlet := face(out_outer.loop, holes: out_inner.loop)
body := solid(inlet, outlet, along: guide)
