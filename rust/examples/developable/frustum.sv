// A frustum: the panel between two coaxial circles, radius 50 and radius 30, 100 apart — a
// conical duct reducer rolled from one flat blank. Every ruling of a cone is one of its
// generators, so the correspondence pairs the two rims at one angle, and the sheet laid flat is
// an annular sector about the image of the cone's apex.
//
// The sheet is the panel's band 2 thick on its outside (`skin`, the offset idiom), and its
// neutral layer — 1 out — is what is laid flat: a pattern is cut for the middle of the sheet, which
// bends without stretching.
//
// Closed forms: slant ℓ = √(20² + 100²) = 101.980; the panel laid flat is an annular sector of
// radii R₁ = 50ℓ/20 = 254.951 and R₂ = 30ℓ/20 = 152.971 about one centre, through 2π·20/ℓ =
// 1.23224 rad (70.60°). The layer d out along the outward normal is the cone moved d/sin α up its
// axis (tan α = 20/100, the half-angle): its sector has radii R + d·cot α = R + 5d, through the
// same angle. The band's volume is πℓ((r₁ + r₂)·t + t² cos α) for a band t thick: 52517.6 at t = 2.
unit mm
use std
use std.panels

r1 := 50mm
r2 := 30mm
rise := 100mm
t := 2mm

in std.top {
  c1 := point
  fix((0mm, 0mm)) c1
  lower := circle(center: c1)
  radius(r1) lower
  s := point hint((50, 0))                        // the seam, on the lower rim
  s coincident lower
  c1 level(up) s                                 // on the x axis
}
up := plane
fix(origin == (0mm, 0mm, rise)) up
fix(dir == (1, 0, 0)) up.u
fix(dir == (0, 1, 0)) up.v
in up {
  c2 := point
  fix((0mm, 0mm)) c2
  upper := circle(center: c2)
  radius(r2) upper
}

panel := std.panels.Developable(lower, upper)     // the panel: what a plane touches both rims along
skin := { q | q distance(<= t, side: outside) panel }
reducer := solid(skin)                            // the sheet, t thick, square at its edges

blank := plane
fix(origin == (0mm, 0mm, -150mm)) blank
fix(dir == (1, 0, 0)) blank.u
fix(dir == (0, 1, 0)) blank.v
flat := std.panels.Development(panel, s, d: t / 2) in blank   // its neutral layer, laid flat
in blank {
  corner := point
  fix((0mm, 0mm)) corner
  x_edge := point
  fix((10mm, 0mm)) x_edge
  edge := line(corner, x_edge)
}
flat.f coincident corner                          // the seam's end on the lower rim at the corner,
line(flat.f, flat.g) parallel edge                // the seam along the blank's edge
