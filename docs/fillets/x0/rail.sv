// X0 (#183): solid_fillet_rail.sv, a rod half sunk along a plate, both sides of it filled between
// the plate's top and the rod, each running out flush into the plate's ends. Does not parse yet.
unit mm
use std
in std.front {
  a := point
  b := point
  c := point
  d := point
  fix((0, 0)) a
  fix((60, 0)) b
  fix((60, 10)) c
  fix((0, 10)) d
  (ab := line(a, b)) -> (bc := line(b, c)) -> (cd := line(c, d)) -> (da := line(d, a)) -> close
  o := point
  fix((30, 10)) o
  k := circle(center: o) hint(r: 6)
  radius(6) k
}
plate := solid(face(ab, bc, cd, da), from: 0mm, to: 40mm)
rod := solid(face(k), from: 0mm, to: 40mm)
rail := solid(plate)
rod union rail

// Both sides at once: the rod meets the plate's top along two lines.
rail.rod round(2mm) rail.plate
