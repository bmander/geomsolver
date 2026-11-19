// A punt's hull of panels (#953): a closed shell — a flat bottom between two spline chines, a flat
// deck between the sheers, flat ends, and two developable sides, each the panel a plane touches
// along every ruling between its chine and its sheer — and the hull the solid it encloses,
// `solid({ p | p in shell.side })`, its faces named by its members (`hull.port`). Each sheer is
// its chine moved 40 out and 300 up, so each side's rulings all run along `(0, ±40, 300)`: a
// cylinder over its chine, curved, and laid flat without stretching.
//
// Kept within the water up to `T`, the immersed body is the hull in common with the half-space;
// its waterplane `immersed.water`.  Here `T` is stated and what the hull displaces there is claimed
// — 30 litres, less a hair, at `T` = 62.94mm, the root of `(40 / 300) · 1000 · T² + A T = V`, `A`
// the bottom's area.  To find the waterline from the displacement instead, write
// `param T: Length hint(100mm)` and state `volume(30000 * 1cm^3) immersed`: the solve moves the
// waterplane over the held panels (some forty seconds natively, its hull filled at every step's
// mesh), and a panel whose rails the solve moves is refused where the row is stated.
unit mm
use std
use std.panels
use std.sets

deck_plane := plane
fix(origin == (0mm, 0mm, 300mm)) deck_plane
fix(dir == (1, 0, 0)) deck_plane.u
fix(dir == (0, 1, 0)) deck_plane.v

// the chines and the bottom, in the top plane
in std.top {
  pc0 := point
  fix((0mm, -150mm)) pc0
  pc1 := point
  fix((250mm, -260mm)) pc1
  pc2 := point
  fix((500mm, -300mm)) pc2
  pc3 := point
  fix((750mm, -260mm)) pc3
  pc4 := point
  fix((1000mm, -150mm)) pc4
  sc0 := point
  fix((0mm, 150mm)) sc0
  sc1 := point
  fix((250mm, 260mm)) sc1
  sc2 := point
  fix((500mm, 300mm)) sc2
  sc3 := point
  fix((750mm, 260mm)) sc3
  sc4 := point
  fix((1000mm, 150mm)) sc4
  port_chine := spline(pc0, pc1, pc2, pc3, pc4)
  starboard_chine := spline(sc0, sc1, sc2, sc3, sc4)
  bow_foot := line(sc0, pc0)
  stern_foot := line(pc4, sc4)
  bottom := face(port_chine, stern_foot, starboard_chine, bow_foot)
}

// the sheers and the deck, 300 up
in deck_plane {
  ps0 := point
  fix((0mm, -190mm)) ps0
  ps1 := point
  fix((250mm, -300mm)) ps1
  ps2 := point
  fix((500mm, -340mm)) ps2
  ps3 := point
  fix((750mm, -300mm)) ps3
  ps4 := point
  fix((1000mm, -190mm)) ps4
  ss0 := point
  fix((0mm, 190mm)) ss0
  ss1 := point
  fix((250mm, 300mm)) ss1
  ss2 := point
  fix((500mm, 340mm)) ss2
  ss3 := point
  fix((750mm, 300mm)) ss3
  ss4 := point
  fix((1000mm, 190mm)) ss4
  port_sheer := spline(ps0, ps1, ps2, ps3, ps4)
  starboard_sheer := spline(ss0, ss1, ss2, ss3, ss4)
  bow_head := line(ss0, ps0)
  stern_head := line(ps4, ss4)
  deck := face(port_sheer, stern_head, starboard_sheer, bow_head)
}

// the ends
bow_plane := plane
fix(origin == (0mm, 0mm, 0mm)) bow_plane
fix(dir == (0, 1, 0)) bow_plane.u
fix(dir == (0, 0, 1)) bow_plane.v
in bow_plane {
  bow1 := point
  bow2 := point
  bow3 := point
  bow4 := point
  fix((-150mm, 0mm)) bow1
  fix((150mm, 0mm)) bow2
  fix((190mm, 300mm)) bow3
  fix((-190mm, 300mm)) bow4
  bow_a := line(bow1, bow2)
  bow_b := line(bow2, bow3)
  bow_c := line(bow3, bow4)
  bow_d := line(bow4, bow1)
  bow := face(bow_a, bow_b, bow_c, bow_d)
}
stern_plane := plane
fix(origin == (1000mm, 0mm, 0mm)) stern_plane
fix(dir == (0, 1, 0)) stern_plane.u
fix(dir == (0, 0, 1)) stern_plane.v
in stern_plane {
  stern1 := point
  stern2 := point
  stern3 := point
  stern4 := point
  fix((-150mm, 0mm)) stern1
  fix((150mm, 0mm)) stern2
  fix((190mm, 300mm)) stern3
  fix((-190mm, 300mm)) stern4
  stern_a := line(stern1, stern2)
  stern_b := line(stern2, stern3)
  stern_c := line(stern3, stern4)
  stern_d := line(stern4, stern1)
  stern := face(stern_a, stern_b, stern_c, stern_d)
}

// the sides, each outside outboard
port := std.panels.Developable(port_chine, port_sheer)
starboard := std.panels.Developable(starboard_sheer, starboard_chine)

shell := { q | q in bottom or q in port or q in starboard or q in deck or q in bow or q in stern }
hull := solid({ p | p in shell.side })

keel := point
fix((0mm, 0mm, 0mm)) keel
T := 62.94mm
water := std.sets.HalfSpace(keel, std.z, d: T)
immersed := solid({ p | p in hull; p in water })
claim volume(>= 29990 * 1cm^3) immersed
