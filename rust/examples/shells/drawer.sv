// A drawer: a box open through a side rather than a cap. The face left open is `drawer.north`,
// the side its section's `north` edge sweeps; the cavity's section is moved out through that
// edge, so it runs out of the north side, and its caps — the drawer's front and back — are moved
// in.
//
// Built exactly: 11520 less 36 × 8 × 22, 5184 mm³.
unit mm
use std
use std.rounds (shell)

wall := 2mm

in std.front {
  b0 := point
  b1 := point
  b2 := point
  b3 := point
  fix((0, 0)) b0
  fix((40, 0)) b1
  fix((40, 24)) b2
  fix((0, 24)) b3
  (south := line(b0, b1)) -> (east := line(b1, b2)) -> (north := line(b2, b3)) -> (west := line(b3, b0)) -> close
  outline := face(south, east, north, west)
}
drawer := solid(outline, depth: 12mm)

drawer.north shell(t: wall) drawer
