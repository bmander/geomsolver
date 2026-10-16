// L0 (#243): a loft to a point. A 20 mm square shrinking to its centre over 30 mm: at s = 1 the
// whole section is one point, the apex (finding: a section may vanish at an end).
// Closed form: V = 20² · 30 / 3 = 4000 mm³.
unit mm
use std

component Shrinking(g: line, up: axis, size: Length, s: Scalar) {
  st := std.OnLine(g, up: up, s: s)
  sq := std.CenteredRectangle(st.n.origin, w: size * (1 - s), h: size * (1 - s)) in st.n
  f := face(sq.loop)
}

in std.top {
  o := point
  fix((0mm, 0mm)) o
  e := point
  fix((0mm, 30mm)) e
  guide := line(o, e)
}
pyramid := sweep(Shrinking(guide, std.z, size: 20mm).f over s in (0, 1))
