// X0 (#183): solid_fillet_block.sv, a block rounded all over and a pocket rounded inside. Where
// three rounded edges meet at a corner, the ball touching all three faces stands there and the
// corner is a patch of it: an eighth of a sphere at a square corner. Does not parse yet.
//
// The prefix form `round(r) X` is every edge among X's faces, as `horizontal l` is the prefix of
// `a horizontal b`. On a swept solid it makes the solid a body over its own sweep, as `union` or
// `cut` naming one does (§6.9, [0.31]), so the block needs no `pebble := solid(block)`.
unit mm
use std
in std.front {
  // a block 40 × 24 × 12
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

  // a plate 50 × 40 × 10 with a pocket 30 × 20 and 6 deep
  p0 := point
  p1 := point
  p2 := point
  p3 := point
  fix((50, 0)) p0
  fix((100, 0)) p1
  fix((100, 40)) p2
  fix((50, 40)) p3
  (ps := line(p0, p1)) -> (pe := line(p1, p2)) -> (pn := line(p2, p3)) -> (pw := line(p3, p0)) -> close
  top := face(ps, pe, pn, pw)
  k0 := point
  k1 := point
  k2 := point
  k3 := point
  fix((60, 10)) k0
  fix((90, 10)) k1
  fix((90, 30)) k2
  fix((60, 30)) k3
  (ks := line(k0, k1)) -> (ke := line(k1, k2)) -> (kn := line(k2, k3)) -> (kw := line(k3, k0)) -> close
  mouth := face(ks, ke, kn, kw)
}
block := solid(outline, depth: 12mm)
round(3mm) block

plate := solid(top, depth: 10mm)
pocket := solid(mouth, from: 1mm, to: -6mm)
tray := solid(plate)
pocket cut tray
// Every edge among the pocket's faces: its floor's four and its four upright corners, concave,
// spherical where three meet. Not the rim: there the pocket meets the plate.
round(2mm) tray.pocket
