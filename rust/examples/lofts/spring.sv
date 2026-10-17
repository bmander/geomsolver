// A coil spring (#243 L6): a round wire drawn square to a helix at every station.
// `std.OnHelix(base, r:, pitch:, turns:, s:)` is the plane square to the helix of radius `r`
// about `base`'s normal, its `u` out from the axis and its `v` up the axis leaned back by the
// lead angle, so a section drawn in it is carried along the helix.
// Closed form: a section square to its path with its centroid on it sweeps its area times the
// path's length: V = π·wire² · turns·√((2π·r)² + pitch²), π·1.5² · 4·√((20π)² + 5²) mm³ as drawn.
// Edit `r`, `wire`, `pitch` or `turns` (a negative pitch winds left-handed).
unit mm
use std

r := 10mm
wire := 1.5mm
pitch := 5mm
turns := 4

component Coil(base: plane, r: Length, wire: Length, pitch: Length, turns: Scalar, s: Scalar) {
  st := std.OnHelix(base, r: r, pitch: pitch, turns: turns, s: s)
  in st.n {
    w := circle(center: st.n.origin)
    radius(wire) w
  }
  f := face(w)
}

param s: Scalar over (0, 1)
spring := sweep(Coil(std.top, r: r, wire: wire, pitch: pitch, turns: turns).f over s)
