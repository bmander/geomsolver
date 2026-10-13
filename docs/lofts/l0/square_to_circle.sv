// L0 (#243): a square 20 mm across lofted to a circle of radius 10 over 30 mm. No pairing of a
// square's four lines with a circle's one edge is natural; drawn as one section it is plain: a
// rounded square whose corner radius runs from 0 to 10 while its flats run from 20 to 0. At s = 0
// the arcs are corners, at s = 1 the flats are points (finding: an edge may vanish at an end).
// Closed form: area(s) = 400(1−s)² + 800s(1−s) + 100πs², so V = 30 · (800 + 100π)/3
// = 8000 + 1000π mm³.
unit mm
use std (coords)
use lofts

component Transition(g: line, up: axis, half: Length, s: Scalar) {
  st := lofts.OnLine(g, up: up, s: s)
  in st.n {
    c0 := point
    c1 := point
    c2 := point
    c3 := point
    h := half * (1 - s)
    c0 coords(du: h, dv: h) st.n
    c1 coords(du: -h, dv: h) st.n
    c2 coords(du: -h, dv: -h) st.n
    c3 coords(du: h, dv: -h) st.n
    outline := (k0 := arc(center: c0)) -> tangent line -> tangent (k1 := arc(center: c1)) ->
      tangent line -> tangent (k2 := arc(center: c2)) -> tangent line ->
      tangent (k3 := arc(center: c3)) -> tangent line -> tangent close
    radius(half * s) k0
    k0 equal k1
    k1 equal k2
    k2 equal k3
  }
  f := face(outline)
}

in std.top {
  o := point
  fix((0mm, 0mm)) o
  e := point
  fix((0mm, 30mm)) e
  guide := line(o, e)
}
transition := solid(Transition(guide, std.z, half: 10mm).f over s in (0, 1))
