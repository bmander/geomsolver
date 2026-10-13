// L0 (#243): a 4 mm pipe along an S-bend, a chain of lines and tangent arcs in plan: 20 mm
// straight, a quarter turn left about R25, 20 mm, a quarter turn right about R25, 20 mm.
// A chain is one path: the station runs over it link by link (`s` from 0 to 5, link k at
// s ∈ [k, k + 1]), the frame square to whichever link it is on, so the joints need no gluing.
// One word, `lofts.On`, stands on any path; see the README's findings on what that asks.
// Closed form: a section square to its path with its centroid on it sweeps its area times the
// path's length: V = π·4² · (3·20 + 2·25π/2) = 16π(60 + 25π) mm³.
unit mm
use std (horizontal, vertical)
use lofts
use pending

component Tube(g: path, up: axis, r: Length, s: Scalar) {
  st := pending.On(g, up: up, s: s)
  in st.n {
    k := circle(center: st.n.origin)
    radius(r) k
  }
  f := face(k)
}

in std.top {
  a := point
  fix((0mm, 0mm)) a
  c1 := point
  c2 := point
  route := horizontal (l0 := line(a, b)) -> tangent (k1 := arc(center: c1)) ->
    vertical (l2 := line) -> tangent (k3 := arc(center: c2)) -> horizontal (l4 := line)
  distance(20mm) l0
  distance(20mm) l2
  distance(20mm) l4
  radius(25mm) k1
  radius(25mm) k3
}
pipe := solid(Tube(route, std.z, r: 4mm).f over s in (0, 5))
