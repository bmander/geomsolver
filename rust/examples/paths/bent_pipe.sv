// A pipe bent in two planes (#363 S4): a straight, a quarter bend in plan, a straight, a quarter
// bend standing up in the plane x = bend, and a straight running up. The straight between the bends
// touches each bend in that bend's own plane — a line in space tangent to an arc at a point of
// it, its direction square to the radius there and in the arc's plane — so the five are one
// chain, and a family runs it link by link, `s` from 0 to 5.
//
// A station that states a rate is drawn on the first link alone: on each later link its frame is
// the one the last link's flow carried across the joint, so a section that is not round rides
// the whole pipe. Starting with `u` square to `z` on the first straight (+y), the frame turns a
// quarter about `z` round the first bend and a quarter about `x` round the second, and ends with
// the tangent +z, `u` −x and `v` −y.
// Closed form: a section square to its path with its centroid on it sweeps its area times the
// path's length, L = 3·straight + 2·bend·π/2: the round tube π·r²·L, 16π(60 + 20π) = 6174.20 mm³,
// and the bar, 3 wide along `u` and 2 along `v` standing 0.5 off the path, 6·(L − π/4) mm³ (its
// centroid runs half toward the first bend's centre and square to the second's plane). The bar's
// end cap stands at g − du·x − dv·y.
// Edit `straight`, `bend` or `r`.
unit mm
use std (coords)

straight := 20mm
bend := 20mm
r := 4mm

// the plane of the second bend: x = bend, its `u` along y and its `v` up
P2 := plane
fix(origin == (bend, 0mm, 0mm)) P2
fix(dir == (0, 1, 0)) P2.u
fix(dir == (0, 0, 1)) P2.v

a := point hint((-straight, 0, 0))
b := point in std.top hint((0, 0))
fix((0mm, 0mm)) b
c1 := point in std.top hint((0, bend))
fix((0mm, bend)) c1
d := point in std.top hint((bend, bend))
e := point in P2 hint((bend + straight, 0))
c2 := point in P2 hint((bend + straight, bend))
f := point in P2 hint((2 * bend + straight, bend))
g := point hint((bend, 2 * bend + straight, bend + straight))
route := (l0 := line(a, b)) -> tangent (k1 := arc(center: c1)) ->
  tangent (l2 := line(d, e)) -> tangent (k3 := arc(center: c2)) ->
  tangent (l4 := line(f, g))
length(straight) l0
length(straight) l2
length(straight) l4
radius(bend) k3
l4 parallel std.z

component Tube(g: path, up: axis, r: Length, s: Scalar) {
  st := std.On(g, up: up, s: s)
  in st.n {
    k := circle(center: st.n.origin)
    radius(r) k
  }
  f := face(k)
}

// a bar from `lo` to `hi` along the frame's `u`, 2 along its `v`
component Bar(g: path, up: axis, lo: Length, hi: Length, s: Scalar) {
  st := std.On(g, up: up, s: s)
  in st.n {
    a := point
    b := point
    c := point
    d := point
  }
  a coords(du: lo, dv: -1mm) st.n
  b coords(du: hi, dv: -1mm) st.n
  c coords(du: hi, dv: 1mm) st.n
  d coords(du: lo, dv: 1mm) st.n
  f := face(a, b, c, d, -> close)
}

pipe := sweep(Tube(route, std.z, r: r).f over s in (0, 5))
bar := sweep(Bar(route, std.z, lo: -1mm, hi: 2mm).f over s in (0, 5))
