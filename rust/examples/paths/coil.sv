// A coil of square wire wound on a helix drawn in space (#363 S1). The helix is a curve traced in
// space: a point placed by its three ordinates as `u` runs from 0 to 1, `turns` turns of radius
// `r` about `z` rising `pitch` a turn. Handed to a `path` formal it is a guide like a line or a
// spline: `std.On` stands a plane square to it where `u` is the family's `s`
// (`m coincident(t == s) g`, the trace's own formal), steps along it to read its tangent
// (`hint(at: m, along: g)`), and turns the plane only as the guide makes it: the frame is
// integrated from where the family stands drawn, its `u` square to `up` there and nowhere held
// to it after, so along a twisted guide the plane stays square to the wire.
//
// The same coil is `lofts/spring.sv` with the helix written into the station's body
// (`std.OnHelix`); here the helix is a curve of the drawing, and could be any.
// Closed form: a section square to its path with its centroid on it sweeps its area times the
// path's length: V = wire² · turns·√((2π·r)² + pitch²), 1.5² · 3·√((20π)² + 5²) mm³ as drawn.
// Edit `r`, `pitch`, `turns` or `wire`.
unit mm
use std (coords)

r := 10mm
pitch := 5mm
turns := 3
wire := 1.5mm

// the frame the helix is written in: three lines from a held origin
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

// a square of side `w` centred on the guide, square to it
component Wire(g: path, up: axis, w: Length, s: Scalar) {
  st := std.On(g, up: up, s: s)
  in st.n {
    a := point
    b := point
    c := point
    d := point
  }
  a coords(du: -w / 2, dv: -w / 2) st.n
  b coords(du: w / 2, dv: -w / 2) st.n
  c coords(du: w / 2, dv: w / 2) st.n
  d coords(du: -w / 2, dv: w / 2) st.n
  f := face(a, b, c, d, -> close)
}

helix := trace(Helix(o, xl, yl, zl, r: r, pitch: pitch, turns: turns).p over u in (0, 1))
param s: Scalar over (0, 1)
coil := sweep(Wire(helix, std.z, w: wire).f over s)
