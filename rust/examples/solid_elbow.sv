// A hollow duct elbow: one hollow square section carried round a constrained arc, the station's
// plane through the bend's axis at every fraction of its turn.
unit mm
use std
use std.place (horizontal)
use std.shapes
use std.stations
bend_radius := 30mm
half_size := 9mm
wall := 2mm
bend_angle := 90deg

component Section(c: point, size: Length, wall: Length) {
  private outer := std.shapes.CenteredRectangle(c, w: size, h: size)
  private inner := std.shapes.CenteredRectangle(c, w: size - 2 * wall, h: size - 2 * wall)
  profile := face(outer.loop, holes: inner.loop)
}

// The section at station `s`: square to the guide there, its `v` along the bend's axis.
component Elbow(g: arc, up: axis, size: Length, wall: Length, s: Scalar) {
  st := std.stations.OnArc(g, up: up, s: s)
  sec := Section(st.n.origin, size: size, wall: wall) in st.n
}

// The guide is drawn in plan, the top plane.
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
param s: Scalar over (0, 1)
body := sweep(Elbow(guide, std.z, size: 2 * half_size, wall: wall).sec.profile over s)
