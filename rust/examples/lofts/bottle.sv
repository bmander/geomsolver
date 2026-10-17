// A bottle through three round sections, ruled and smooth (#243 L7): body, shoulder and neck,
// radii 30, 28 and 12 at heights 0, 60 and 90. Several sections are joined by **rails**: one
// rail a corresponding corner, its places in order, and each station's rim stands where its rail
// meets the station's plane (`rim := point in st.n, front`, `rim coincident rail`).
//   ruled: the rail is the chain of lines through the rims, `rims`, and the guide the axis cut
//     at the shoulder, `axis`; a family runs every chain it is given in step, link by link, so
//     the loft is ruled piecewise in `s`, section k at s = k, and the shoulder is an edge;
//   smooth: the rail is the spline through its rims, `spline(through: rims2, start:, end:)`, the
//     cubic through them a span a link, its ends along the tangents written — here the axis, so
//     the wall leaves the base and enters the neck square to them.
// Closed forms: ruled, the sum of two frusta, π·60/3·(30² + 30·28 + 28²) + π·30/3·(28² + 28·12 +
// 12²) ≈ 198297.33 mm³; smooth, π∫x² dy along the rail, a quadrature exact for its degree.
// The smooth bottle is drawn `apart` along x from the ruled one, so the two are seen side by side.
// Edit any radius or height, or `apart`.
unit mm
use std

foot := 30mm
shoulder := 28mm
neck := 12mm
rise := 60mm
height := 90mm
apart := 80mm

component Ring(g: path, up: axis, front: plane, rail: path, s: Scalar) {
  st := std.On(g, up: up, s: s)
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
  waist := point
  top := point
  fix((0mm, 0mm)) base
  fix((0mm, rise)) waist
  fix((0mm, height)) top
  axis := line(base, waist) -> line(waist, top)
  r0 := point
  r1 := point
  r2 := point
  fix((foot, 0mm)) r0
  fix((shoulder, rise)) r1
  fix((neck, height)) r2
  rims := line(r0, r1) -> line(r1, r2)
}
// the smooth bottle's own axis and rims, `apart` along x
in std.front {
  base2 := point
  top2 := point
  fix((apart, 0mm)) base2
  fix((apart, height)) top2
  whole := line(base2, top2)
  q0 := point
  q1 := point
  q2 := point
  fix((apart + foot, 0mm)) q0
  fix((apart + shoulder, rise)) q1
  fix((apart + neck, height)) q2
  rims2 := line(q0, q1) -> line(q1, q2)
  rail := spline(through: rims2, start: whole, end: whole)
}
param s: Scalar over (0, 2)
ruled := sweep(Ring(axis, std.y, std.front, rims).f over s)
param s2: Scalar over (0, 1)
smooth := sweep(Ring(whole, std.y, std.front, rail, s: s2).f over s2)
