// L0 (#243): solid_elbow.sv as a running section: one hollow square carried round a constrained
// arc, the station plane through the bend's axis at every fraction of its turn.
// Closed form (Pappus): A = 18² − 14² = 128 mm², centroid 30 mm from the axis, a quarter turn:
// V = 128 · 30 · π/2 = 1920π mm³.
unit mm
use std
use std.shapes
use std.stations
bend_radius := 30mm
bend_angle := 90deg

component Section(c: point, size: Length, wall: Length) {
  private outer := std.shapes.CenteredRectangle(c, w: size, h: size)
  private inner := std.shapes.CenteredRectangle(c, w: size - 2 * wall, h: size - 2 * wall)
  profile := face(outer.loop, holes: inner.loop)
}

component Elbow(g: arc, up: axis, size: Length, wall: Length, s: Scalar) {
  st := std.stations.OnArc(g, up: up, s: s)
  sec := Section(st.n.origin, size: size, wall: wall) in st.n
}

in std.top {
  turn_center := point
  fix((0mm, 0mm)) turn_center
  entry := point hint((30mm, 0mm))
  exit := point hint((0mm, 30mm))
  inlet := line(turn_center, entry)
  outlet := line(turn_center, exit)
  horizontal inlet
  inlet angle(bend_angle) outlet
  guide := arc(center: turn_center, start: entry, end: exit)
  radius(bend_radius) guide
}
param s: Scalar over (0, 1)
body := sweep(Elbow(guide, std.z, size: 18mm, wall: 2mm).sec.profile over s)
