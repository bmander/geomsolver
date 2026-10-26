// An offset square-to-round: a 240 × 240 duct (a closed chain) under a Ø160 round 220 above it,
// the round's centre 40 over and 30 back from the square's — the sheet-metal transition a duct
// run needs where two ducts do not line up. One developable sheet joins them: along each straight
// side a planar triangle, its rulings meeting at the one point of the round the plane through the
// side touches, and about each corner a fan, a cone, where the plane turns between the corner's
// two sides. With the round off centre the four triangles are four different heights and the
// fans four different cones: nothing about the pattern is symmetric, and nothing is special-cased.
//
// The sheet is galvanised steel 1 thick (`skin`), and its neutral layer — half a thickness out —
// is laid flat from the seam, the middle of the front side, outside up: the cut pattern, placed
// on its blank by two relations.
//
// Closed forms: the pattern's edge along the square is 4 · 240 = 960 long and along the round
// 160π = 502.655 (the layer half a thickness out runs a hair longer at the corners' fans, where
// it turns). The front triangle stands in the plane through the side y = −120, z = 0 that touches
// the round: tangent to the round of radius 80 about (40, 30) at the height 220, so its apex is at
// the round's point (40, 30 − 80, 220) = (40, −50, 220) and the triangle's height √(70² + 220²) =
// 230.868, its area 240 · 230.868 / 2 = 27704.2; the left one's apex is (40 − 80, 30, 220), its
// height √(80² + 220²) = 234.094; the back one's (40, 110, 220), √(10² + 220²) = 220.227; the
// right one's (120, 30, 220), √(0² + 220²) = 220 — that side's plane is upright, the round
// overhanging it to the very edge.
unit mm
use std
use std.panels

side := 240mm
r := 80mm
rise := 220mm
over := 40mm
back := 30mm
thick := 1mm

in std.top {
  a := point hint((-120, -120))
  b := point hint((120, -120))
  c := point hint((120, 120))
  d := point hint((-120, 120))
  mouth := horizontal (ab := line(a, b)) -> vertical (bc := line(b, c)) ->
           horizontal (cd := line(c, d)) -> vertical (da := line(d, a)) -> close
  length(side) ab
  length(side) bc
  o := point
  fix((0mm, 0mm)) o
  o midpoint line(a, c)
  s := point
  s midpoint ab                                   // the seam: the middle of the front side
}
duct := plane
fix(origin == (over, back, rise)) duct
fix(dir == (1, 0, 0)) duct.u
fix(dir == (0, 1, 0)) duct.v
in duct {
  k := point
  fix((0mm, 0mm)) k
  round := circle(center: k)
  radius(r) round
}

panel := std.panels.Developable(mouth, round)            // a set: the transition
skin := { q | q distance(<= thick, side: outside) panel }
transition := solid(skin)                         // the sheet, 1 thick

blank := plane
fix(origin == (0mm, 0mm, -300mm)) blank
fix(dir == (1, 0, 0)) blank.u
fix(dir == (0, 1, 0)) blank.v
in blank {
  k0 := point
  k1 := point
  k2 := point
  k3 := point
  fix((0mm, 0mm)) k0
  fix((800mm, 0mm)) k1
  fix((800mm, 800mm)) k2
  fix((0mm, 800mm)) k3
  bottom := line(k0, k1)
  right := line(k1, k2)
  top := line(k2, k3)
  left := line(k3, k0)
}
flat := std.panels.Development(panel, s, d: thick / 2) in blank   // its pattern, laid on the blank
// the seam's foot 130 along the blank's bottom edge and 60 up, the seam square to that edge
k0 distance(130mm, along: std.x) flat.f
k0 distance(60mm, along: std.y) flat.f
line(flat.f, flat.g) perpendicular bottom
