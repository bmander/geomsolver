// A foot filled into a drafted casting beside its rounded corner (#962). The casting is 160 × 100
// with corners of 20, drafted 2° from its floor, so each side is a plane leaning in that goes on
// tangent into its corner's cone along a line: the south side's seam with the south-east corner
// stands at x = 60 all the way up. A foot 28 long, 15 tall and standing 20 out from the south side
// is drawn 2 short of that line — the tangent point the drawing dimensions, less a little — and
// its root is filled 5 against the casting.
//
// The ball stands against whichever face of the casting is nearest (std's sets read the side's
// smooth surface, `F.smooth`): along the foot's east side, 5 out from it, it stands past the
// seam's normal plane, so it touches the cone and not the side, all the way up — every distance
// in the side's plane is the same along the edge. The fill there is the ball rolled against the
// cone the edge's whole length, its centre on a hyperbola, its face fitted, cut off at the floor
// and at the foot's top: `solventc --step` writes it beside the prisms along the foot's other
// edges. Move the foot's east side to 54 and the ball touches the side alone: a prism. Write
// `chamfer(d: 5mm)` for the round and the flat is the side's line short of the seam's normal
// plane, the sum of distances from the cone and the foot beyond it, glued there. Edit the foot or
// the radius and apply (⌘↵).
unit mm
use std
use std.solids (union)
use std.rounds (round, chamfer)

in std.top {
  c0 := point
  c1 := point
  c2 := point
  c3 := point
  fix((-80, -50)) c0
  fix((80, -50)) c1
  fix((80, 50)) c2
  fix((-80, 50)) c3
  (south := line(c0, c1)) -> (se := round(r: 20mm)) (east := line(c1, c2)) -> (ne := round(r: 20mm)) (north := line(c2, c3)) -> (nw := round(r: 20mm)) (west := line(c3, c0)) -> (sw := round(r: 20mm)) close
  outline := face(south, se, east, ne, north, nw, west, sw)
  f0 := point
  f1 := point
  f2 := point
  f3 := point
  fix((30, -70)) f0
  fix((58, -70)) f1
  fix((58, -40)) f2
  fix((30, -40)) f3
  (fs := line(f0, f1)) -> (fe := line(f1, f2)) -> (fn := line(f2, f3)) -> (fw := line(f3, f0)) -> close
  footing := face(fs, fe, fn, fw)
}
casting := solid(std.solids.Drafted(outline, from: 0mm, to: 60mm, draft: 2deg))
foot := solid(std.solids.Prism(footing, from: 0mm, to: 15mm))
part := solid(casting)
foot union part

part.foot round(r: 5mm) part.casting
