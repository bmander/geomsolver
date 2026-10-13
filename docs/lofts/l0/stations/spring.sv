// L0 (#243): a coil spring and a square thread, each a section carried along a screw.
// The spring's wire is drawn square to the helix at every station; the thread's square is drawn
// in the plane through the axis (an axial section), which the screw carries rigidly.
// Closed forms:
//   spring: wire r 1.5, coil r 10, pitch 5, 4 turns; a section square to its path with its centroid
//     on it sweeps its area times the path's length: V = π·1.5² · 4·√((20π)² + 5²) mm³.
//   thread: a 2 mm square at radius 9–11, pitch 4, 3 turns; an axial section under a screw sweeps
//     its area times its centroid's circle per turn (Pappus): V = 4 · 2π·10 · 3 = 240π mm³.
unit mm
use std (on_u)
use lofts

component Coil(ax: axis, base: plane, r: Length, wire: Length, pitch: Length, turns: Scalar,
               s: Scalar) {
  st := lofts.OnHelix(ax, base, r: r, pitch: pitch, turns: turns, s: s)
  in st.n {
    w := circle(center: st.n.origin)
    radius(wire) w
  }
  f := face(w)
}

component Thread(ax: axis, base: plane, r: Length, size: Length, pitch: Length, turns: Scalar,
                 s: Scalar) {
  st := lofts.Screwed(ax, base, pitch: pitch, turns: turns, s: s)
  in st.n {
    c := point
    c on_u(d: r) st.n
  }
  sq := std.CenteredRectangle(c, w: size, h: size) in st.n
  f := face(sq.loop)
}

spring := Coil(std.z, std.top, r: 10mm, wire: 1.5mm, pitch: 5mm, turns: 4, s: 0.5)
thread := Thread(std.z, std.top, r: 10mm, size: 2mm, pitch: 4mm, turns: 3, s: 0.5)
