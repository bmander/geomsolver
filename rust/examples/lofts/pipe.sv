// A 4 mm pipe along a U-bend (#243 L4): a chain of lines and tangent arcs drawn in plan — a
// straight, a quarter turn left, a straight, another quarter turn left, a straight — is one path.
// The station runs over it link by link (`s` from 0 to 5, link k while s is in (k, k + 1)), its
// frame square to whichever link it stands on, so the joints need no gluing and each wall keeps
// one name round the whole run. One word, `std.On`, stands on a line, an arc, a spline or a
// traced curve alike.
// Closed form: a section square to its path with its centroid on it sweeps its area times the
// path's length (Pappus): V = π·r² · (3·straight + 2·bend·π/2), 16π(60 + 25π) mm³ as drawn.
// Edit `straight`, `bend` or `r`.
unit mm
use std (horizontal, vertical)

straight := 20mm
bend := 25mm
r := 4mm

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
  b := point hint((straight, 0))
  c1 := point hint((straight, bend))
  d := point hint((straight + bend, bend))
  e := point hint((straight + bend, bend + straight))
  c2 := point hint((straight, bend + straight))
  f := point hint((straight, 2 * bend + straight))
  g := point hint((0, 2 * bend + straight))
  fix((0mm, 0mm)) a
  // an arc of a chain runs counter-clockwise from its start, so the chain turns left at each
  route := horizontal (l0 := line(a, b)) -> tangent (k1 := arc(center: c1)) ->
    tangent vertical (l2 := line(d, e)) -> tangent (k3 := arc(center: c2)) ->
    tangent horizontal (l4 := line(f, g))
  length(straight) l0
  length(straight) l2
  length(straight) l4
  radius(bend) k1
  radius(bend) k3
}
pipe := sweep(Tube(route, std.z, r: r).f over s in (0, 5))
