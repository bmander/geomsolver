// A rod half sunk along a plate, both sides of it filled where it stands on the plate's top, each
// running out flush into the plate's ends. The rod splits the plate's top in two, so `rail.plate`'s
// top is two faces, each meeting the rod along a line: a round over a face in pieces is a run along
// each edge it meets the other in. In a section square to the rod the plate's top is a line and
// the rod a circle, and the ball rolls between them. Built exactly: a prism along each side whose
// face is a cylinder.
//
// Edit `r` or the rod's radius and apply (⌘↵).
unit mm
use std (round)

r := 2mm   // the fill's radius

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
  radius(6mm) k
}
plate := solid(face(ab, bc, cd, da), from: 0mm, to: 40mm)
rod := solid(face(k), from: 0mm, to: 40mm)
rail := solid(plate)
rod union rail

// Both sides at once: the rod meets the plate's top along two lines.
rail.rod round(r: r) rail.plate
