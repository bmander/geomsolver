// A cast mounting pad: a boss standing on a plate, its root filled, two of the plate's near edges
// rounded off, and a bore drilled down through the boss and the plate after. Which rounds add
// material and which take it away is read off the geometry — the root is concave, the edges
// convex — so nothing says `union` or `cut`. The rounds read the faces the body has before the
// bore, which cuts through them after. Built exactly: a ring whose face is a torus at the root, a
// prism whose face is a cylinder along each edge, each running out flush into the plate's sides.
//
// Edit the radii and apply (⌘↵).
unit mm
use std (round)

root_r := 3mm   // the boss's root, filled
edge_r := 2mm   // the plate's two near edges, taken away

in std.front {
  t0 := point
  t1 := point
  t2 := point
  t3 := point
  fix((-30, -20)) t0
  fix((30, -20)) t1
  fix((30, 20)) t2
  fix((-30, 20)) t3
  (ab := line(t0, t1)) -> (bc := line(t1, t2)) -> (cd := line(t2, t3)) -> (da := line(t3, t0)) -> close
  outline := face(ab, bc, cd, da)
  c := point
  fix((0, 0)) c
  rim := circle(center: c) hint(r: 8)
  radius(8mm) rim
  hole := circle(center: c) hint(r: 4)
  radius(4mm) hole
}
plate := solid(outline, depth: 10mm)
boss := solid(face(rim), from: 0mm, to: 20mm)
bore := solid(face(hole), from: -10mm, to: 20mm)
pad := solid(plate)
boss union pad
bore cut pad

// The boss's root: every edge where a face the boss gave the pad meets one the plate gave it.
pad.boss round(r: root_r) pad.plate
// Two of the plate's near edges, each running out flush into the plate's sides.
pad.plate.near round(r: edge_r) pad.plate.da
pad.plate.near round(r: edge_r) pad.plate.bc
