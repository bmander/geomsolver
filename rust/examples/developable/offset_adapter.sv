// An offset adapter: two parallel round ducts whose axes do not line up — a Ø100 rim at the origin
// and a Ø60 rim 100 above it and 30 over. The panel between them is an oblique cone: the plane
// touching the lower rim at one angle touches the upper one at the same angle, so every ruling
// pairs equal angles exactly and passes through the cone's apex.
//
// Closed form: the ruling from (50 cos θ, 50 sin θ, 0) to (30 + 30 cos θ, 30 sin θ, 100) passes
// through (75, 0, 250) for every θ — five halves of the way along it — so the apex is there; at θ =
// −40° the upper end is (52.981, −19.284, 100), the closed form the prototype's single ruling
// solved to. Laid flat, each rail keeps its length (100π and 60π) and each ruling its own.
unit mm
use std
use std.panels

r1 := 50mm
r2 := 30mm
rise := 100mm
over := 30mm

in std.top {
  c1 := point
  fix((0mm, 0mm)) c1
  lower := circle(center: c1)
  radius(r1) lower
  s := point hint((38.3, -32.1))                  // the seam, at −40° on the lower rim
  s coincident lower
}
seam_at := line(c1, s)
in std.top {
  x1 := point
  fix((10mm, 0mm)) x1
  xl := line(c1, x1)
}
xl angle(-40deg) seam_at
up := plane
fix(origin == (over, 0mm, rise)) up
fix(dir == (1, 0, 0)) up.u
fix(dir == (0, 1, 0)) up.v
in up {
  c2 := point
  fix((0mm, 0mm)) c2
  upper := circle(center: c2)
  radius(r2) upper
}

panel := std.panels.Developable(lower, upper)
skin := { q | q distance(<= 1.5mm, side: outside) panel }
adapter := solid(skin)

blank := plane
fix(origin == (0mm, 0mm, -150mm)) blank
fix(dir == (1, 0, 0)) blank.u
fix(dir == (0, 1, 0)) blank.v
flat := std.panels.Development(panel, s, d: 0mm) in blank
in blank {
  corner := point
  fix((0mm, 0mm)) corner
  x_edge := point
  fix((10mm, 0mm)) x_edge
  edge := line(corner, x_edge)
}
flat.f coincident corner
line(flat.f, flat.g) parallel edge
