// A rectangle to a round: a 200 × 120 duct mouth (a closed chain) under a Ø100 duct 150 above it,
// joined by one developable sheet. A plane touching both rails along each ruling makes four planar
// triangles — along each straight side the rulings all meet at the one point of the round the
// plane through that side touches — and four fans, cones about the corners, where the plane turns
// about a corner between its two sides' directions, supporting it there. Nothing is a special
// case: the rulings are what `std.Developable`'s body says, and the corners fan because a plane
// touches a corner wherever it lies between its two one-sided tangents.
//
// The band is seamed at the middle of a long side, `s`, and laid flat from there, outside up: the
// development is the sheet-metal pattern of the transition, its long sides cut straight (each
// triangle's base, 200 or 120 long) and its rounded edge the circle's 100π.
//
// Closed forms: the pattern's edge along the rectangle is 2·(200 + 120) = 640 long, and along the
// round 100π = 314.159. Each triangle is planar — its rulings in one plane — with its apex where
// that plane touches the round: for a long side, the plane through the side's line at (y = −60, z
// = 0) touching the circle of radius 50 at z = 150 there at (0, −50, 150), so the triangle's
// height is √(10² + 150²) = 150.333 and its area 200 · 150.333 / 2 = 15033.3; for a short side,
// (100, 0, 0) to (50, 0, 150), height √(50² + 150²) = 158.114, area 120 · 158.114 / 2 = 9486.8.
unit mm
use std

w := 200mm
h := 120mm
r := 50mm
rise := 150mm

in std.top {
  a := point hint((-100, -60))
  b := point hint((100, -60))
  c := point hint((100, 60))
  d := point hint((-100, 60))
  mouth := horizontal (ab := line(a, b)) -> vertical (bc := line(b, c)) ->
           horizontal (cd := line(c, d)) -> vertical (da := line(d, a)) -> close
  length(w) ab
  length(h) bc
  o := point
  fix((0mm, 0mm)) o
  o midpoint line(a, c)
  s := point
  s midpoint ab                                   // the seam: the middle of a long side
}
duct := plane
fix(origin == (0mm, 0mm, rise)) duct
fix(dir == (1, 0, 0)) duct.u
fix(dir == (0, 1, 0)) duct.v
in duct {
  k := point
  fix((0mm, 0mm)) k
  round := circle(center: k)
  radius(r) round
}

panel := std.Developable(mouth, round)            // a set: the transition
skin := { q | q distance(<= 1.5mm, side: outside) panel }
transition := solid(skin)                         // the sheet, 1.5 thick

sheet := plane
fix(origin == (0mm, 0mm, -100mm)) sheet
fix(dir == (1, 0, 0)) sheet.u
fix(dir == (0, 1, 0)) sheet.v
flat := std.Development(panel, s, d: 0mm) in sheet   // its pattern, laid in the sheet
in sheet {
  corner := point
  fix((0mm, 0mm)) corner
  x_edge := point
  fix((10mm, 0mm)) x_edge
  datum := line(corner, x_edge)
}
flat.f coincident corner                          // the seam's end on the rectangle at the corner
line(flat.f, flat.g) perpendicular datum          // and the seam square to the sheet's edge
