// An S-bend in plan (#363 S4): a straight, a left turn, a straight, a right turn, a straight. An
// arc drawn in a plane runs counter-clockwise from its start, so a chain entering every link at
// its start could only turn left at one. A chain may also enter an arc at its end and walk it to
// its start: the right turn is written as the arc from `f` to `e`, counter-clockwise, and the
// chain enters it at `e`. It is drawn from there — its ends swapped and its turn reversed, the
// same arc — so the joints, the tangencies and the family along the chain all read it forward. A
// name still reads the points as written: `k3.start` is `f`, as the claims below say.
//
// The other way to say the same turn is `arc(center: c2, sense: cw)`, threaded from `e` like the
// first bend: an arc that says it runs clockwise from its start.
// Closed form: a section square to its path with its centroid on it sweeps its area times the
// path's length, V = π·r² · (3·straight + 2·bend·π/2), 16π(60 + 20π) mm³ as drawn, and the run
// ends at (2·straight + 2·bend, 2·bend + straight).
// Edit `straight`, `bend` or `r`.
unit mm
use std (horizontal, vertical)

straight := 20mm
bend := 20mm
r := 4mm

in std.top {
  a := point
  b := point hint((straight, 0))
  c1 := point hint((straight, bend))
  d := point hint((straight + bend, bend))
  e := point hint((straight + bend, bend + straight))
  c2 := point hint((straight + 2 * bend, bend + straight))
  f := point hint((straight + 2 * bend, 2 * bend + straight))
  g := point hint((2 * straight + 2 * bend, 2 * bend + straight))
  fix((0mm, 0mm)) a
  // the right turn, written from where it ends to where it starts: the chain enters it at `e`
  route := horizontal (l0 := line(a, b)) -> tangent (k1 := arc(center: c1)) ->
    tangent vertical (l2 := line(d, e)) -> tangent (k3 := arc(center: c2, start: f, end: e)) ->
    tangent horizontal (l4 := line(f, g))
  length(straight) l0
  length(straight) l2
  length(straight) l4
  radius(bend) k1
  radius(bend) k3
}
claim k3.start coincident f
claim k3.end coincident e

component Tube(g: path, up: axis, r: Length, s: Scalar) {
  st := std.On(g, up: up, s: s)
  in st.n {
    k := circle(center: st.n.origin)
    radius(r) k
  }
  f := face(k)
}

param s: Scalar over (0, 5)
tube := sweep(Tube(route, std.z, r: r).f over s)
