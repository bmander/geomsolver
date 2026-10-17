// A line routed through space with no plane written (#363 S6): four straights and three bends,
// every point standing in space. An arc over points in space is drawn in the plane through its
// centre and its ends — minted, its `u` toward the start, its normal (start − centre) × (end −
// centre), the short way round — so each bend is an arc in a plane all the same: its radius, its
// tangencies, a chain's joints, a family along it. The first bend turns in plan, the second
// stands up, the third leans back over the run: three planes, none of them named.
//
// A chain drawn in space turns either way at an arc: each bend's plane faces as its points do.
// Closed form: a section square to its path with its centroid on it sweeps its area times the
// path's length, V = π·r² · (4·straight + 3·bend·π/2), 16π(80 + 30π) mm³ as drawn; the bends'
// planes are square to z, x and y in turn.
// Edit `straight`, `bend` or `r`.
unit mm
use std

straight := 20mm
bend := 20mm
r := 4mm

a := point hint((-straight, 0, 0))
b := point hint((0, 0, 0))
fix((0mm, 0mm, 0mm)) b
c1 := point hint((0, bend, 0))
fix((0mm, bend, 0mm)) c1
d := point hint((bend, bend, 0))
e := point hint((bend, bend + straight, 0))
c2 := point hint((bend, bend + straight, bend))
f := point hint((bend, 2 * bend + straight, bend))
g := point hint((bend, 2 * bend + straight, bend + straight))
c3 := point hint((0, 2 * bend + straight, bend + straight))
h := point hint((0, 2 * bend + straight, 2 * bend + straight))
i := point hint((-straight, 2 * bend + straight, 2 * bend + straight))
route := (l0 := line(a, b)) -> tangent (k1 := arc(center: c1)) ->
  tangent (l2 := line(d, e)) -> tangent (k3 := arc(center: c2)) ->
  tangent (l4 := line(f, g)) -> tangent (k5 := arc(center: c3)) ->
  tangent (l6 := line(h, i))
length(straight) l0
length(straight) l2
length(straight) l4
length(straight) l6
radius(bend) k3
radius(bend) k5
d coincident std.top
l2 parallel std.y
l4 parallel std.z
l6 parallel std.x

component Tube(g: path, up: axis, r: Length, s: Scalar) {
  st := std.On(g, up: up, s: s)
  in st.n {
    k := circle(center: st.n.origin)
    radius(r) k
  }
  f := face(k)
}

param s: Scalar over (0, 7)
tube := sweep(Tube(route, std.z, r: r).f over s)
