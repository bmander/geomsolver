// A tilted adapter: the offset adapter's Ø60 rim turned 25° about the line through its centre
// along y — a reducer between a horizontal duct and one that leans. The panel is no cone now: the
// rulings are wherever one plane touches both rims, which is solved, not paired by angle.
//
// Which root: at a place on the lower rim, two planes through its tangent touch the upper rim —
// one where it runs through the plane the way the lower rim does (`sense: same`), and the crossing
// one, a bow-tie that cuts through itself. From the seam at −40° on the lower rim, (38.302,
// −32.139, 0), they touch the upper rim at (50.035, −20.281, 90.658) — the outer, the panel's —
// and at a point round the far side of it, the root a solve seeded near the far side falls into and
// `sense: same` excludes. The outer agrees with an independent root-find of det(a′, b′, b − a) = 0.
// Laid flat, each rail keeps its length (100π and 60π) and each ruling its own.
unit mm
use std

r1 := 50mm
r2 := 30mm
rise := 100mm
over := 30mm
tilt := 25deg

in std.top {
  c1 := point
  fix((0mm, 0mm)) c1
  lower := circle(center: c1)
  radius(r1) lower
  s := point hint((38.3, -32.1))                  // the seam, at −40° on the lower rim
  s coincident lower
  x1 := point
  fix((10mm, 0mm)) x1
  xl := line(c1, x1)
}
xl angle(-40deg) line(c1, s)
up := plane                                       // the upper rim's plane, turned 25° about y
fix(origin == (over, 0mm, rise)) up
fix(dir == (cos(tilt), 0, -sin(tilt))) up.u
fix(dir == (0, 1, 0)) up.v
in up {
  c2 := point
  fix((0mm, 0mm)) c2
  upper := circle(center: c2)
  radius(r2) upper
}

panel := std.Developable(lower, upper)
skin := { q | q distance(<= 1.5mm, side: outside) panel }
adapter := solid(skin)

blank := plane
fix(origin == (0mm, 0mm, -150mm)) blank
fix(dir == (1, 0, 0)) blank.u
fix(dir == (0, 1, 0)) blank.v
flat := std.Development(panel, s, d: 0mm) in blank
in blank {
  corner := point
  fix((0mm, 0mm)) corner
  x_edge := point
  fix((10mm, 0mm)) x_edge
  edge := line(corner, x_edge)
}
flat.f coincident corner
line(flat.f, flat.g) parallel edge
