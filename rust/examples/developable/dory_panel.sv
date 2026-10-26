// A dory's side panel: the plywood strake between the chine, where it meets the bottom, and the
// sheer, its top edge — two splines in space, bow to stern, the port side so the panel's outside
// is outboard. Only a developable surface comes off a flat sheet without stretching, so the panel
// is the one a plane touches along each ruling between the two. The sheer is longer than the
// chine at both ends (the stem rakes forward, the transom aft), so the regular rulings reach the
// sheer's ends after the chine's: there the chine's end points fan, the stem's cone at the bow
// and the transom's at the stern.
//
// Each rail lies in a plane of its own, `z = 120 + y/4` and `z = 600 + y/4`, parallel planes,
// and turns one way in it; a plane touching both touches them where their tangents are
// parallel, so the rulings run on without folding. (A rail with an S in it, seen along the
// rulings, has a ruling where the tangent plane stands still: there two families of planes
// touching both rails cross, and the panel is refused — E080 — rather than guessed.)
//
// Its neutral layer (the strake is 6 thick, the layer 3 out) is laid on a 2440 × 1220 sheet of
// plywood, drawn in its own plane, by two relations: the stem's top 40 in from the sheet's end,
// the seam's foot 120 above its long edge — and the seam ruling square to that edge.
// `dory_panel.svd` draws the sheet, the cut file.
//
// What it holds to: laid flat, each rail keeps its length and each ruling its own; the pattern's
// area is the panel's layer's; the stem's and the transom's fans start at the chine's end points;
// the pattern lies inside the sheet (`tests/developable.rs`).
unit mm
use std
use std.panels

thick := 6mm

// the chine and the sheer, each a spline over five control points in space, bow (x = 0) to stern
c0 := point hint((0, 0, 120))
c1 := point hint((470, -300, 45))
c2 := point hint((980, -420, 15))
c3 := point hint((1490, -390, 22.5))
c4 := point hint((1960, -320, 40))
fix((0mm, 0mm, 120mm)) c0
fix((470mm, -300mm, 45mm)) c1
fix((980mm, -420mm, 15mm)) c2
fix((1490mm, -390mm, 22.5mm)) c3
fix((1960mm, -320mm, 40mm)) c4
chine := spline(c0, c1, c2, c3, c4)
s0 := point hint((-85, 0, 600))
s1 := point hint((410, -420, 495))
s2 := point hint((980, -560, 460))
s3 := point hint((1550, -530, 467.5))
s4 := point hint((2025, -440, 490))
fix((-85mm, 0mm, 600mm)) s0
fix((410mm, -420mm, 495mm)) s1
fix((980mm, -560mm, 460mm)) s2
fix((1550mm, -530mm, 467.5mm)) s3
fix((2025mm, -440mm, 490mm)) s4
sheer := spline(s0, s1, s2, s3, s4)

// the seam, where the pattern is laid from: the chine's point 980 aft of the stem
seam := point hint((980, -420, 15))
seam coincident chine
c0 distance(980mm, along: std.x) seam

panel := std.panels.Developable(chine, sheer)
skin := { q | q distance(<= thick, side: outside) panel }
strake := solid(skin)

// the plywood sheet, in a plane of its own below the boat
plywood := plane
fix(origin == (0mm, 0mm, -500mm)) plywood
fix(dir == (1, 0, 0)) plywood.u
fix(dir == (0, 1, 0)) plywood.v
in plywood {
  k0 := point
  k1 := point
  k2 := point
  k3 := point
  fix((0mm, 0mm)) k0
  fix((2440mm, 0mm)) k1
  fix((2440mm, 1220mm)) k2
  fix((0mm, 1220mm)) k3
  long_edge := line(k0, k1)
  stern_edge := line(k1, k2)
  top_edge := line(k2, k3)
  bow_edge := line(k3, k0)
}
flat := std.panels.Development(panel, seam, d: thick / 2) in plywood
// the stem's top 40 in from the sheet's end, the seam's foot 120 above its long edge
k0 distance(40mm, along: std.x) flat.start.p2
k0 distance(120mm, along: std.y) flat.f
line(flat.f, flat.g) perpendicular long_edge
