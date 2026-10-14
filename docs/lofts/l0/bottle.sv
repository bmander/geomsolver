// L0 (#243): a smooth loft through three round sections: a bottle's body, shoulder and neck,
// radii 30, 28 and 12 at heights 0, 60 and 90, square to the axis at its base and its neck.
// Smooth is a rail: a spline in the front plane through the three sections' rims, its end
// tangents the end conditions. Each station's circle reaches the rail where its plane meets it,
// the rim point drawn in the station's plane and in the front plane at once (§6.7, 0.58).
// Closed form: none (the rail is a cubic); the volume is π∫r² dz, held to quadrature.
unit mm
use std (horizontal, vertical)
use lofts

component Body(g: line, up: axis, front: plane, rail: spline, s: Scalar) {
  st := lofts.OnLine(g, up: up, s: s)
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
  k0 := point hint((30mm, 30mm))
  k1 := point hint((32mm, 75mm))
  rail := spline(r0, k0, k1, r2)
  r1 coincident rail
  vertical (foot := line(r0, k0))
  length(30mm) foot
  vertical (lip := line(k1, r2))
}
bottle := solid(Body(axis_line, std.y, std.front, rail).f over s in (0, 1))
