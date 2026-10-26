// A gusset placed by its volume (#510): a right-angled steel rib, 5mm thick, one leg along the
// floor 100mm long and the other up the wall, its top `b` free to slide up it, and
// `volume(30 * 1cm^3) rib` stated of the drawing.  The volume is a row over the rib's face, its
// area through its depth: the leg up the wall the solve finds is 2 V / (t a), 120mm.  Its mass is
// claimed beside it, judged on the solved drawing.  A volume is written as its number of a unit
// cubed: `30cm^3` would be the cube of 30 centimetres.
unit mm
use std
use std.place (vertical)

a := 100mm
t := 5mm
steel := material(E: 200GPa, nu: 0.3, rho: 7850kg / 1m^3)

in std.front {
  o := point
  f := point
  b := point hint((0, 80))
  fix((0mm, 0mm)) o
  fix((a, 0mm)) f
  o vertical b
  floor := line(o, f)
  slope := line(f, b)
  wall := line(b, o)
  web := face(floor, slope, wall)
}
rib := solid(web, from: 0mm, to: t) of steel
volume(30 * 1cm^3) rib
claim mass(<= 250g) rib
