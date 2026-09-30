// A hollow duct that narrows as it bends: an inlet and an outlet lofted along a constrained arc,
// each section blended into the next while it turns about the bend's axis.
unit mm
use std
param bend_radius = 30mm
param inlet_half = 9mm
param outlet_half = 6mm
param wall = 2mm
param bend_angle = 90deg

point center hint(x: bend_radius, y: 0mm)
std.origin horizontal center
std.origin distance(bend_radius) center
outer: CenteredRectangle(center, w: 2 * inlet_half, h: 2 * inlet_half)
inner: CenteredRectangle(center, w: 2 * (inlet_half - wall), h: 2 * (inlet_half - wall))
face inlet(outer.loop, holes: inner.loop)

// The guide is drawn in plan; each section stands perpendicular to its tangent at its end.
plane plan(origin: std.origin, toward: std.front.toward, from: std.front, fold: 0deg)
in plan {
  point turn_center hint(x: 0mm, y: 0mm)
  ground turn_center
  point entry hint(x: bend_radius, y: 0mm)
  point exit hint(x: bend_radius * cos(bend_angle), y: bend_radius * sin(bend_angle))
  construction centerline line inlet_axis(turn_center, entry)
  construction centerline line outlet_axis(turn_center, exit)
  horizontal inlet_axis
  inlet_axis angle(bend_angle) outlet_axis
  construction centerline arc guide(center: turn_center, start: entry, end: exit)
  radius(bend_radius) guide
}

// The outlet's plane holds the bend's axis and the radius to the exit.
plane outlet_view(origin: std.origin, toward: std.front.toward, u: (0, 1, 0), v: (0, 0, 1))
in outlet_view {
  point out_center hint(x: bend_radius, y: 0mm)
  ground out_center
  out_outer: CenteredRectangle(out_center, w: 2 * outlet_half, h: 2 * outlet_half)
  out_inner: CenteredRectangle(out_center, w: 2 * (outlet_half - wall), h: 2 * (outlet_half - wall))
}
face outlet(out_outer.loop, holes: out_inner.loop)
solid body(inlet, outlet, along: guide)
