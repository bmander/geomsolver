// L0 (#243): a smooth loft through three round sections: a bottle's body, shoulder and neck,
// radii 30, 28 and 12 at heights 0, 60 and 90, square to the axis at its base and its neck.
// Smooth is a rail: the spline through the three sections' rims (#243 L7), one span a pair of
// neighbouring sections, its tangents at the base and the neck along the axis — the end
// conditions. Each station's circle reaches the rail where its plane meets it, the rim point
// drawn in the station's plane and in the front plane at once (§6.7, 0.58). The rims in order
// are a chain, `rims`, which is also the ruled loft's rail: `solid(…rims…)` over the axis cut at
// the shoulder lofts the frusta between the sections.
// Closed form: none (the rail is a cubic); the volume is π∫r² dz, held to quadrature (L1, L7).
// L0 drew the rail by hand, a Bézier through the shoulder, and a cubic square to both ends reached
// a shoulder of 28 only by running past the neck and back, so L1 lowered it to 20; the spline
// through the rims, least bent between them, rises the whole way at 28 (L7).
unit mm
use std

component Body(g: line, up: axis, front: plane, rail: path, s: Scalar) {
  st := std.OnLine(g, up: up, s: s)
  rim := point in st.n, front
  rim coincident rail
  in st.n {
    k := circle(center: st.n.origin)
    rim coincident k
  }
  f := face(k)
}

in std.front {
  base := point
  fix((0mm, 0mm)) base
  top := point
  fix((0mm, 90mm)) top
  axis_line := line(base, top)
  r0 := point
  r1 := point
  r2 := point
  fix((30mm, 0mm)) r0
  fix((28mm, 60mm)) r1
  fix((12mm, 90mm)) r2
  rims := line(r0, r1) -> line(r1, r2)
  rail := spline(through: rims, start: axis_line, end: axis_line)
}
bottle := sweep(Body(axis_line, std.y, std.front, rail).f over s in (0, 1))
