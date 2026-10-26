// A loft with no guide (#243 L7): three squares in space — 20 at height 0, 12 at 40 moved off
// the axis, 16 at 60 back on it — and a chain of lines through each corresponding corner. The
// station is the plane through three rails at `s` (`std.stations.Across`), and each corner stands
// where its rail meets it; the family runs the four chains in step, link by link.
// Closed form: between level sections each piece is a prismatoid,
// 40/6 · (400 + 4·256 + 144) + 20/6 · (144 + 4·196 + 256) = 14400 mm³.
// Edit any corner.
unit mm
use std
use std.stations

component Quad(ra: path, rb: path, rc: path, rd: path, s: Scalar) {
  st := std.stations.Across(ra, rb, rc, s: s)
  a := point in st.n
  b := point in st.n
  c := point in st.n
  d := point in st.n
  a coincident ra
  b coincident rb
  c coincident rc
  d coincident rd
  f := face(a, b, c, d, -> close)
}

a0 := point
b0 := point
c0 := point
d0 := point
fix((-10mm, -10mm, 0mm)) a0
fix((10mm, -10mm, 0mm)) b0
fix((10mm, 10mm, 0mm)) c0
fix((-10mm, 10mm, 0mm)) d0
a1 := point
b1 := point
c1 := point
d1 := point
fix((4mm, -1mm, 40mm)) a1
fix((16mm, -1mm, 40mm)) b1
fix((16mm, 11mm, 40mm)) c1
fix((4mm, 11mm, 40mm)) d1
a2 := point
b2 := point
c2 := point
d2 := point
fix((-8mm, -8mm, 60mm)) a2
fix((8mm, -8mm, 60mm)) b2
fix((8mm, 8mm, 60mm)) c2
fix((-8mm, 8mm, 60mm)) d2
ra := line(a0, a1) -> line(a1, a2)
rb := line(b0, b1) -> line(b1, b2)
rc := line(c0, c1) -> line(c1, c2)
rd := line(d0, d1) -> line(d1, d2)
param s: Scalar over (0, 2)
body := sweep(Quad(ra, rb, rc, rd).f over s)
