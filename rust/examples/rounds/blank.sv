// A gear blank's face, defined as a set: the shell between a pitch cone grown by the addendum
// (`tip`) and shrunk by the dedendum, and between two spheres about the cone's apex — outside the
// toe's (`toe_side`), inside the heel's — with the tip's rims rounded where it meets each sphere.
//
// `tip` is `std.sets.Offset(pitch, d: …)`, the cone grown by the addendum: an offset cone. Rounding
// its corners with `std.rounds.Rounded(tip, heel, r: …)` is a ball rolled inside both, whose centre
// stands on the cone grown by the addendum less the radius and on the sphere shrunk by it. Every
// term is about the cone's axis, so the region is a meridian turned once, its rounds built exactly
// in that meridian (lowered): `solventc --step` writes two tori between the cones and spheres.
//
// Edit the addendum, the radii or the cone's half angle and apply (⌘↵). Swap a round for
// `std.rounds.Chamfered(tip, heel, la: 1mm, lb: 1mm)` and the flat beside the sphere bows with it,
// the sum of distances a parabola's arc in the meridian, built exactly too.
unit mm
use std
use std.sets
use std.rounds

apex := point hint((0, 0, 0))
fix((0, 0, 0)) apex
ze := point hint((0, 0, 1))
fix((0, 0, 1)) ze
axis_line := line(apex, ze)

pitch := std.sets.Cone(axis_line, half: 30deg)
tip := std.sets.Offset(pitch, d: 1.5mm)
heel := std.sets.Sphere(apex, r: 20)
toe_side := { q | q distance(>= 12mm) apex }

face := { p |
  p inside tip
  p distance(<= 2mm, side: inside) pitch
  p inside heel
  p inside toe_side
  p inside std.rounds.Rounded(tip, heel, r: 0.8mm)
  p inside std.rounds.Rounded(tip, toe_side, r: 0.8mm)
}
blank := solid(face)
