// A ribbon wound on a helix in Frenet's frame, beside one in the frame that does not twist
// (#363 S5). A curve traced in space has a centre of curvature where it bends: `g curvature(at:
// m) c` holds the point `c` there, the centre of the circle osculating the curve at `m`.
// `std.Frenet(g, s:)` turns its plane's `u` toward it — the principal normal, its sense the
// centre's and no seed's — and its `v` along the binormal, so the ribbon `facing` keeps its width
// pointed at the helix's axis all the way round, as a strip wound flat on a drum would.
// `std.On`'s frame is the rotation-minimising one: it turns only as the guide makes it, so the
// ribbon `minimal`, starting the same way, falls behind Frenet's at the helix's torsion, a turn of
// 2π·pitch/√((2π·r)² + pitch²) about the tangent for every turn of the helix.
//
// Frenet's frame has no centre to point at where the curve does not bend — a straight run, an
// inflection — and a station there is refused, never followed.
// Closed form: each ribbon sweeps its area along the helix's length,
// V = w·h · turns·√((2π·r)² + pitch²), 4 · 0.5 · 3·√((24π)² + 6²) mm³ as drawn; and `facing`'s
// edges are helices of radius r ± w/2, its end cap's corners where the closed form puts them.
// Edit `r`, `pitch`, `turns`, `w` or `h`.
unit mm
use std (coords)

r := 12mm
pitch := 6mm
turns := 3
w := 4mm
h := 0.5mm

o := point hint((0, 0, 0))
fix((0mm, 0mm, 0mm)) o
x1 := point hint((10, 0, 0))
fix((10mm, 0mm, 0mm)) x1
y1 := point hint((0, 10, 0))
fix((0mm, 10mm, 0mm)) y1
z1 := point hint((0, 0, 10))
fix((0mm, 0mm, 10mm)) z1
xl := line(o, x1)
yl := line(o, y1)
zl := line(o, z1)

component Helix(o: point, x: line, y: line, z: line, r: Length, pitch: Length, turns: Scalar,
                u: Scalar) {
  p := point hint((r * cos(360deg * turns * u), r * sin(360deg * turns * u), pitch * turns * u))
  o distance(r * cos(360deg * turns * u), along: x) p
  o distance(r * sin(360deg * turns * u), along: y) p
  o distance(pitch * turns * u, along: z) p
}

// a `w` × `h` rectangle centred in a plane, `w` along its `u`
component Strip(n: plane, w: Length, h: Length) {
  in n {
    a := point
    b := point
    c := point
    d := point
  }
  a coords(du: -w / 2, dv: -h / 2) n
  b coords(du: w / 2, dv: -h / 2) n
  c coords(du: w / 2, dv: h / 2) n
  d coords(du: -w / 2, dv: h / 2) n
}

component Facing(g: path, w: Length, h: Length, s: Scalar) {
  st := std.Frenet(g, s: s)
  sec := Strip(st.n, w: w, h: h)
  f := face(sec.a, sec.b, sec.c, sec.d, -> close)
}

component Minimal(g: path, up: axis, w: Length, h: Length, s: Scalar) {
  st := std.On(g, up: up, s: s)
  sec := Strip(st.n, w: w, h: h)
  f := face(sec.a, sec.b, sec.c, sec.d, -> close)
}

helix := Helix(o, xl, yl, zl, r: r, pitch: pitch, turns: turns).p over u in (0, 1)
facing := solid(Facing(helix, w: w, h: h).f over s in (0, 1))
minimal := solid(Minimal(helix, std.z, w: w, h: h).f over s in (0, 1))

// Frenet's frame where the helix is a third of the way up: its centre of curvature stands
// (r² + (pitch/2π)²)/r in from the curve, toward the axis
here := std.Frenet(helix, s: 0.3)
