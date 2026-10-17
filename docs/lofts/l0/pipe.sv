// L0 (#243): a 4 mm pipe along a U-bend, a chain of lines and tangent arcs in plan: 20 mm
// straight, a quarter turn left about R25, 20 mm, another quarter turn left about R25, 20 mm.
// A chain is one path: the station runs over it link by link (`s` from 0 to 5, link k at
// s ∈ [k, k + 1]), the frame square to whichever link it is on, so the joints need no gluing.
// One word, `std.On`, stands on any path (since L4, an arc's too).
// Closed form: a section square to its path with its centroid on it sweeps its area times the
// path's length: V = π·4² · (3·20 + 2·25π/2) = 16π(60 + 25π) mm³.
//
// (An arc of a chain runs counter-clockwise from its start, so a chain turns left at every arc:
// the S-bend L0 first drew, a right turn after a left, is not a chain until an arc can be walked
// end to start.)
unit mm
use std (horizontal, vertical)

component Tube(g: path, up: axis, r: Length, s: Scalar) {
  st := std.On(g, up: up, s: s)
  in st.n {
    k := circle(center: st.n.origin)
    radius(r) k
  }
  f := face(k)
}

in std.top {
  a := point
  b := point hint((20, 0))
  c1 := point hint((20, 25))
  d := point hint((45, 25))
  e := point hint((45, 45))
  c2 := point hint((20, 45))
  f := point hint((20, 70))
  g := point hint((0, 70))
  fix((0mm, 0mm)) a
  route := horizontal (l0 := line(a, b)) -> tangent (k1 := arc(center: c1)) ->
    tangent vertical (l2 := line(d, e)) -> tangent (k3 := arc(center: c2)) ->
    tangent horizontal (l4 := line(f, g))
  length(20mm) l0
  length(20mm) l2
  length(20mm) l4
  radius(25mm) k1
  radius(25mm) k3
}
param s: Scalar over (0, 5)
pipe := sweep(Tube(route, std.z, r: 4mm).f over s)
