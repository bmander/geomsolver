// A D-shaped boss drafted on a plate (#962): a round boss of radius 10 with a flat 4 off its
// centre, standing 12 on a 60 × 40 plate 5 thick, its sides leaning 3° in as it rises. The flat
// meets the round side at an angle at each end of it.
//
// Drafted, the flat is a plane leaning in and the round side a cone narrowing as it rises; where
// they meet the corner moves along a parabola, the curve the plane and the cone meet in, which the
// exact kernel builds. A D-shaped hole drafted the same way would grow as it goes, its corners
// moving out along such curves.
//
// Drawn on the top plane, the plate's face at z = 0, the boss standing up from it. Edit the draft
// or the flat's offset and apply (⌘↵).
unit mm
use std
use std.solids (union)

dr := 3deg   // the boss's draft

in std.top {
  p0 := point
  p1 := point
  p2 := point
  p3 := point
  fix((-30, -20)) p0
  fix((30, -20)) p1
  fix((30, 20)) p2
  fix((-30, 20)) p3
  (s := line(p0, p1)) -> (e := line(p1, p2)) -> (n := line(p2, p3)) -> (w := line(p3, p0)) -> close

  o := point
  fix((0, 0)) o
  // the flat's ends, on the circle of 10, 4 below its centre
  f1 := point hint((9.17, -4))
  f2 := point hint((-9.17, -4))
  fix(y == -4mm) f1
  fix(y == -4mm) f2
  flat := line(f2, f1)
  round_side := arc(center: o, start: f1, end: f2)
  radius(10mm) round_side
}
plate := solid(std.solids.Prism(face(s, e, n, w), from: -5mm, to: 0mm))
boss := solid(std.solids.Drafted(face(flat, round_side), from: 0mm, to: 12mm, draft: dr))
part := solid(plate)
boss union part
