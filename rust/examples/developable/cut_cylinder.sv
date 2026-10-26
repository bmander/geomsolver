// A cylinder cut by a tilted plane: a round duct r 40 mitred at 20°, its cut edge the ellipse the
// plane makes of the cylinder — `std.Ellipse` drawn in the tilted plane, a curve written as a
// formula and read as a path in space. The panel between the base circle and the ellipse is the
// cylinder itself: every ruling is vertical, the plane touching both rails along a generator.
//
// Laid flat from the generator through x = 40, the outside up, the pattern is the classic mitre
// template: a strip whose bottom edge is the base circle's 2π·40 = 251.327 unrolled straight and
// whose top edge is a cosine.
//
// Closed form: the ellipse's point at eccentric angle θ is (40 cos θ, 40 sin θ, h + 40 tan β cos θ)
// (semi-axes 40/cos β along the tilted plane's u and 40 along y, h = 80 the plane's height on the
// axis, β = 20°), so the generator there is h + 40 tan β · cos θ long, and laid flat at x = 40θ
// along the bottom edge the top edge is y = h + 40 tan β · cos(x/40). The pattern's area is
// 2π·40·h = 20106.2: the cosine's part integrates to nothing.
unit mm
use std

r := 40mm
h := 80mm
beta := 20deg

in std.top {
  c := point
  fix((0mm, 0mm)) c
  base := circle(center: c)
  radius(r) base
  s := point hint((40, 0))                        // the seam, on the base at x = 40
  s coincident base
  c level(up) s
}
cut := plane                                      // the mitre's plane, tilted β about y
fix(origin == (0mm, 0mm, h)) cut
fix(dir == (cos(beta), 0, sin(beta))) cut.u
fix(dir == (0, 1, 0)) cut.v
in cut {
  o := point
  fix((0mm, 0mm)) o
}
mitre := trace(std.Ellipse(o, a: r / cos(beta), b: r, tilt: 0deg).p over u in (0deg, 360deg))

panel := std.Developable(base, mitre)
skin := { q | q distance(<= 1mm, side: outside) panel }
duct := solid(skin)

blank := plane
fix(origin == (0mm, 0mm, -150mm)) blank
fix(dir == (1, 0, 0)) blank.u
fix(dir == (0, 1, 0)) blank.v
flat := std.Development(panel, s, d: 0mm) in blank
in blank {
  corner := point
  fix((0mm, 0mm)) corner
  y_edge := point
  fix((0mm, 10mm)) y_edge
  edge := line(corner, y_edge)
}
flat.f coincident corner                          // the seam's foot at the blank's corner,
line(flat.f, flat.g) parallel edge                // the seam up the blank's edge
