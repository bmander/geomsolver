// A block rounded all over and a pocket rounded inside (issue #66, rung 3): where three of a
// fillet's edges meet at a corner between three faces, the ball touching all three stands there,
// and the corner is a patch of it — an eighth of a sphere at a square corner. Each edge's run stops
// at the section through the ball's centre, and the corner's patch ends in all three.
unit mm
use std
in std.front {

  // a block 40 × 24 × 12, every edge rounded to 3
  b0 := point
  b1 := point hint(x: 40, y: 0)
  b2 := point hint(x: 40, y: 24)
  b3 := point hint(x: 0, y: 24)
  fix(x == 0, y == 0) b0
  fix(x == 40, y == 0) b1
  fix(x == 40, y == 24) b2
  fix(x == 0, y == 24) b3
  (south := line(b0, b1)) -> (east := line(b1, b2)) -> (north := line(b2, b3)) -> (west := line(b3, b0)) -> close
  outline := face(south, east, north, west)
  block := solid(outline, depth: 12mm)
  pebble := solid(block)
  round := fillet(block, block, r: 3mm)
  round cut pebble

  // a plate 50 × 40 × 10 with a pocket 30 × 20 and 6 deep, its floor's edges and its corners
  // rounded inside to 2
  p0 := point hint(x: 50, y: 0)
  p1 := point hint(x: 100, y: 0)
  p2 := point hint(x: 100, y: 40)
  p3 := point hint(x: 50, y: 40)
  fix(x == 50, y == 0) p0
  fix(x == 100, y == 0) p1
  fix(x == 100, y == 40) p2
  fix(x == 50, y == 40) p3
  (ps := line(p0, p1)) -> (pe := line(p1, p2)) -> (pn := line(p2, p3)) -> (pw := line(p3, p0)) -> close
  top := face(ps, pe, pn, pw)
  k0 := point hint(x: 60, y: 10)
  k1 := point hint(x: 90, y: 10)
  k2 := point hint(x: 90, y: 30)
  k3 := point hint(x: 60, y: 30)
  fix(x == 60, y == 10) k0
  fix(x == 90, y == 10) k1
  fix(x == 90, y == 30) k2
  fix(x == 60, y == 30) k3
  (ks := line(k0, k1)) -> (ke := line(k1, k2)) -> (kn := line(k2, k3)) -> (kw := line(k3, k0)) -> close
  mouth := face(ks, ke, kn, kw)
  plate := solid(top, depth: 10mm)
  pocket := solid(mouth, from: 1mm, to: -6mm)
  cupped := solid(plate)
  pocket cut cupped
  tray := solid(cupped)
  inside := fillet(cupped.pocket, cupped.pocket, r: 2mm)
  inside union tray
}
