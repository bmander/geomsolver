// L0 (#243): solid_loft.sv as a running section. The reducer's two ends are one section drawn
// once, its size running from the inlet's to the outlet's as the station runs along the guide.
// No second face, no pairing rule: the edge `ab` at one station is `ab` at every other.
// Closed form: area(s) = size² − (size − 2·wall)² is linear in size = 24 − 12s, so
// V = 40 · (8 · 18 − 16) = 5120 mm³, today's `solid_loft` volume.
unit mm
use std
length := 40mm

component Section(c: point, size: Length, wall: Length) {
  private outer := std.CenteredRectangle(c, w: size, h: size)
  private inner := std.CenteredRectangle(c, w: size - 2 * wall, h: size - 2 * wall)
  profile := face(outer.loop, holes: inner.loop)
}

// The section at station `s`: drawn in the plane square to the guide there, its size the inlet's
// mixed with the outlet's.
component Reducer(g: line, up: axis, inlet: Length, outlet: Length, wall: Length, s: Scalar) {
  st := std.OnLine(g, up: up, s: s)
  sec := Section(st.n.origin, size: inlet + (outlet - inlet) * s, wall: wall) in st.n
}

in std.top {
  entry := point
  fix((0mm, 0mm)) entry
  exit := point hint((0mm, 40mm))
  guide := line(entry, exit)
  vertical guide
  length(length) guide
}
body := sweep(Reducer(guide, std.z, inlet: 24mm, outlet: 12mm, wall: 2mm).sec.profile over s in (0, 1))
