// A dory's three panels, each a developable sheet of 6 mm plywood: the bottom between the two
// chines, and a side each side between its chine and its sheer — the hull a dory builder cuts from
// flat sheets and bends together over a few frames. (`dory_panel.sv` is the port side alone, its
// gates.)
//
// The rails: each chine is a spline over five points in the plane `z = 120 + |y|/4`, each sheer in
// `z = 600 + |y|/4`, mirror images port and starboard, each turning one way in its plane, so no
// panel folds. The sheers run past the chines at the stem and the transom, so each side fans from
// its chine's end points; the bottom's rulings run straight across, port chine to starboard, the
// chines' mirror images pairing (a plane holds a tangent and its mirror image along the line
// between them), and it is no wider than 80 at its bow, where it meets the stem.
//
// Which way each faces is the order of its rails: a panel's outside is `a′ × (b − a)`, so the
// bottom is `Developable(starboard chine, port chine)` — outside down — and each side runs from
// the chine on port and from the sheer on starboard — outside outboard. Laid flat, each is laid
// outside up: the two sides are mirror images, as a builder cuts them.
//
// Laid out for cutting on two sheets of 2440 × 1220 plywood drawn side by side in one plane, below
// the boat: the two sides nested on the first, bows at one end, the port's chine along the
// sheet's long edge and the starboard's along the other, their sheers meeting in the middle; the
// bottom alone on the second. `dory.svd` draws the hull and the cut file.
//
// What it holds to: each panel solves at DOF 0 with its rails; laid flat, each rail keeps its
// length (the bottom's chine edges, port and starboard, are one length); the two sides' patterns
// are mirror images, their rails' lengths equal; every pattern lies on its sheet.
unit mm
use std

thick := 6mm

// the chines, in the planes z = 120 + |y|/4, bow (x = 60) to stern (x = 1960)
pc0 := point hint((60, -40, 110))
pc1 := point hint((470, -300, 45))
pc2 := point hint((980, -420, 15))
pc3 := point hint((1490, -390, 22.5))
pc4 := point hint((1960, -320, 40))
fix((60mm, -40mm, 110mm)) pc0
fix((470mm, -300mm, 45mm)) pc1
fix((980mm, -420mm, 15mm)) pc2
fix((1490mm, -390mm, 22.5mm)) pc3
fix((1960mm, -320mm, 40mm)) pc4
port_chine := spline(pc0, pc1, pc2, pc3, pc4)
sc0 := point hint((60, 40, 110))
sc1 := point hint((470, 300, 45))
sc2 := point hint((980, 420, 15))
sc3 := point hint((1490, 390, 22.5))
sc4 := point hint((1960, 320, 40))
fix((60mm, 40mm, 110mm)) sc0
fix((470mm, 300mm, 45mm)) sc1
fix((980mm, 420mm, 15mm)) sc2
fix((1490mm, 390mm, 22.5mm)) sc3
fix((1960mm, 320mm, 40mm)) sc4
starboard_chine := spline(sc0, sc1, sc2, sc3, sc4)

// the sheers, in the planes z = 600 + |y|/4, from the stem's head (x = −85) to the transom's top
ps0 := point hint((-85, 0, 600))
ps1 := point hint((410, -420, 495))
ps2 := point hint((980, -560, 460))
ps3 := point hint((1550, -530, 467.5))
ps4 := point hint((2025, -440, 490))
fix((-85mm, 0mm, 600mm)) ps0
fix((410mm, -420mm, 495mm)) ps1
fix((980mm, -560mm, 460mm)) ps2
fix((1550mm, -530mm, 467.5mm)) ps3
fix((2025mm, -440mm, 490mm)) ps4
port_sheer := spline(ps0, ps1, ps2, ps3, ps4)
ss0 := point hint((-85, 0, 600))
ss1 := point hint((410, 420, 495))
ss2 := point hint((980, 560, 460))
ss3 := point hint((1550, 530, 467.5))
ss4 := point hint((2025, 440, 490))
fix((-85mm, 0mm, 600mm)) ss0
fix((410mm, 420mm, 495mm)) ss1
fix((980mm, 560mm, 460mm)) ss2
fix((1550mm, 530mm, 467.5mm)) ss3
fix((2025mm, 440mm, 490mm)) ss4
starboard_sheer := spline(ss0, ss1, ss2, ss3, ss4)

// the three panels, each a set, each sheet its band 6 thick on its outside
bottom := std.Developable(starboard_chine, port_chine)
port := std.Developable(port_chine, port_sheer)
starboard := std.Developable(starboard_sheer, starboard_chine)
bottom_skin := { q | q distance(<= thick, side: outside) bottom }
port_skin := { q | q distance(<= thick, side: outside) port }
starboard_skin := { q | q distance(<= thick, side: outside) starboard }
bottom_panel := solid(bottom_skin)
port_side := solid(port_skin)
starboard_side := solid(starboard_skin)

// where each is laid from: amidships, 980 aft of the stem, on the panel's first rail
bottom_seam := point hint((980, 420, 15))
bottom_seam coincident starboard_chine
sc0 distance(920mm, along: std.x) bottom_seam
port_seam := point hint((980, -420, 15))
port_seam coincident port_chine
pc0 distance(920mm, along: std.x) port_seam
starboard_seam := point hint((980, 560, 460))
starboard_seam coincident starboard_sheer
ss0 distance(1065mm, along: std.x) starboard_seam

// two sheets of plywood side by side in the shop floor's plane, below the boat
floor := plane
fix(origin == (0mm, 0mm, -800mm)) floor
fix(dir == (1, 0, 0)) floor.u
fix(dir == (0, 1, 0)) floor.v
in floor {
  // the first sheet, for the sides
  a0 := point
  a1 := point
  a2 := point
  a3 := point
  fix((0mm, 0mm)) a0
  fix((2440mm, 0mm)) a1
  fix((2440mm, 1220mm)) a2
  fix((0mm, 1220mm)) a3
  sides_edge := line(a0, a1)
  sides_end := line(a1, a2)
  sides_top := line(a2, a3)
  sides_start := line(a3, a0)
  // the second, beside it, for the bottom
  b0 := point
  b1 := point
  b2 := point
  b3 := point
  fix((0mm, 1400mm)) b0
  fix((2440mm, 1400mm)) b1
  fix((2440mm, 2620mm)) b2
  fix((0mm, 2620mm)) b3
  bottom_edge := line(b0, b1)
  bottom_end := line(b1, b2)
  bottom_top := line(b2, b3)
  bottom_start := line(b3, b0)
}

// each panel's neutral layer, 3 out, laid on its sheet by two distances and a bearing
port_flat := std.Development(port, port_seam, d: thick / 2) in floor
a0 distance(40mm, along: std.x) port_flat.start.p2      // the stem's head 40 in from the end
a0 distance(90mm, along: std.y) port_flat.f              // the seam's chine end 90 up
sides_edge angle(90deg) line(port_flat.f, port_flat.g)  // the seam square to the long edge, up
starboard_flat := std.Development(starboard, starboard_seam, d: thick / 2) in floor
a0 distance(40mm, along: std.x) starboard_flat.start.p1  // its stem's head 40 in from the end
a0 distance(1130mm, along: std.y) starboard_flat.g       // its seam's chine end 90 from the top
// its seam square to the long edge, sheer to chine up
sides_edge angle(90deg) line(starboard_flat.f, starboard_flat.g)
bottom_flat := std.Development(bottom, bottom_seam, d: thick / 2) in floor
b0 distance(60mm, along: std.x) bottom_flat.start.p1     // the bow 60 in from the sheet's end
b0 distance(160mm, along: std.y) bottom_flat.f           // the starboard chine 160 up
bottom_edge angle(90deg) line(bottom_flat.f, bottom_flat.g)  // its seam square across the sheet
