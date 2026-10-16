// L0 (#243): a square 20 mm across lofted to a circle of radius 10 over 30 mm. No pairing of a
// square's four lines with a circle's one edge is natural; drawn as one section it is plain: a
// rounded square whose corner radius runs from 0 to 10 while its flats run from 20 to 0. At s = 0
// the arcs are corners, at s = 1 the flats are points (finding: an edge may vanish at an end).
// Closed form: area(s) = 400(1−s)² + 800s(1−s) + 100πs², so V = 30 · (800 + 100π)/3
// = 8000 + 1000π mm³.
unit mm
use std (coords)

component Transition(g: line, up: axis, half: Length, s: Scalar) {
  st := std.OnLine(g, up: up, s: s)
  in st.n {
    h := half * (1 - s)
    r := half * s
    c0 := point
    c1 := point
    c2 := point
    c3 := point
    c0 coords(du: h, dv: h) st.n
    c1 coords(du: -h, dv: h) st.n
    c2 coords(du: -h, dv: -h) st.n
    c3 coords(du: h, dv: -h) st.n
    // the flats' ends, seeded where the rounded square has them (L1): unseeded, the drawn
    // station came out with every flat a point and each corner arc the long way round
    t1 := point hint((h, h + r))
    t2 := point hint((-h, h + r))
    l1 := point hint((-h - r, h))
    l2 := point hint((-h - r, -h))
    b1 := point hint((-h, -h - r))
    b2 := point hint((h, -h - r))
    r1 := point hint((h + r, -h))
    r2 := point hint((h + r, h))
    outline := (k0 := arc(center: c0) hint(r: r)) -> tangent (top := line(t1, t2)) ->
      tangent (k1 := arc(center: c1) hint(r: r)) -> tangent (left := line(l1, l2)) ->
      tangent (k2 := arc(center: c2) hint(r: r)) -> tangent (bottom := line(b1, b2)) ->
      tangent (k3 := arc(center: c3) hint(r: r)) -> tangent (right := line(r1, r2)) ->
      tangent close
    radius(r) k0
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
transition := sweep(Transition(guide, std.z, half: 10mm).f over s in (0, 1))
