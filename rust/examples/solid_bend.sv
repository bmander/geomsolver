// A hollow duct that narrows as it bends: one hollow square section carried round a constrained
// arc, its half size running from the inlet's to the outlet's as it turns about the bend's axis.
unit mm
use std
use std.place (horizontal)
use std.shapes
use std.stations
bend_radius := 30mm
inlet_half := 9mm
outlet_half := 6mm
wall := 2mm
bend_angle := 90deg

component Section(c: point, size: Length, wall: Length) {
  private outer := std.shapes.CenteredRectangle(c, w: size, h: size)
  private inner := std.shapes.CenteredRectangle(c, w: size - 2 * wall, h: size - 2 * wall)
  profile := face(outer.loop, holes: inner.loop)
}

// The section at station `s`: square to the guide there, its half size the inlet's mixed with the
// outlet's.
component Bend(g: arc, up: axis, inlet: Length, outlet: Length, wall: Length, s: Scalar) {
  st := std.stations.OnArc(g, up: up, s: s)
  sec := Section(st.n.origin, size: 2 * (inlet + (outlet - inlet) * s), wall: wall) in st.n
}

// The guide is drawn in plan, the top plane.
in std.top {
  turn_center := point
  fix((0mm, 0mm)) turn_center
  entry := point hint((bend_radius, 0mm))
  exit := point hint((bend_radius * cos(bend_angle), bend_radius * sin(bend_angle)))
  construction centerline inlet_axis := line(turn_center, entry)
  construction centerline outlet_axis := line(turn_center, exit)
  horizontal inlet_axis
  inlet_axis angle(bend_angle) outlet_axis
  construction centerline guide := arc(center: turn_center, start: entry, end: exit)
  radius(bend_radius) guide
}
param s: Scalar over (0, 1)
body := sweep(Bend(guide, std.z, inlet: inlet_half, outlet: outlet_half, wall: wall).sec.profile over s)
