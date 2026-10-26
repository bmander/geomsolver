// A groove along a plate: a rod's half sunk into the plate's top and cut away, its rims rounded
// all round — along the plate's top and at both ends. At each corner a run along the top meets a
// ring's stretch round the groove's end, each with a face on the groove's cylinder: the two are
// mitred, their rounds meeting in a curve that ends where both touch the groove — a point where
// the run's cylinder and the ring's torus touch, their meeting two branches crossing there, one of
// which runs into both faces.
//
// The plate is drawn on the front plane and stands 40 deep in front of it. Orbit (right-drag) to
// see the corners. Edit `r` or the groove's radius and apply (⌘↵).
unit mm
use std
use std.rounds (round)

r := 2mm   // the rims' radius

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
part := solid(plate)
rod cut part

// every edge where the groove meets the plate: its rims along the top and round both ends
part.rod round(r: r) part.plate
