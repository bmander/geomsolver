// A hollow reducer: one hollow square section run along a dimensioned centerline, its size the
// inlet's mixed with the outlet's as the station runs. The edge `ab` at one station is `ab` at
// every other, so the two ends correspond by being one section.
unit mm
use std
length := 40mm
inlet_size := 24mm
outlet_size := 12mm
wall := 2mm

component Section(c: point, size: Length, wall: Length) {
  private outer := std.CenteredRectangle(c, w: size, h: size)
  private inner := std.CenteredRectangle(c, w: size - 2 * wall, h: size - 2 * wall)
  profile := face(outer.loop, holes: inner.loop)
}

// The section at station `s`: drawn in the plane square to the guide there, its size running
// from the inlet's to the outlet's.
component Reducer(g: line, up: axis, inlet: Length, outlet: Length, wall: Length, s: Scalar) {
  st := std.OnLine(g, up: up, s: s)
  sec := Section(st.n.origin, size: inlet + (outlet - inlet) * s, wall: wall) in st.n
}

in std.top {
  entry := point
  fix((0mm, 0mm)) entry
  exit := point hint((0mm, length))
  construction centerline guide := line(entry, exit)
  vertical guide
  length(length) guide
}
param s: Scalar over (0, 1)
body := sweep(Reducer(guide, std.z, inlet: inlet_size, outlet: outlet_size, wall: wall).sec.profile over s)
