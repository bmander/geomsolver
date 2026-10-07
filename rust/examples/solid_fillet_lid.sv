// A block's top rim rounded alone, a pocket's floor rounded alone and a triangular prism's cap
// rounded alone (issue #66, rung 3): where two of a fillet's edges meet at a corner whose third
// edge is left sharp, each run carries on to the plane of the face beyond it and the two cross
// there, a mitre. The sharp edge turns the fillet's way — convex beside a convex fillet, concave
// beside a concave one — or the runs would part rather than cross.
unit mm
use std
in std.front {

  // a block 40 × 24 × 12, its near cap's rim rounded to 3, its upright edges sharp
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
  lid := solid(block)
  rim := fillet(block.near, block, r: 3mm)
  rim cut lid

  // a plate 50 × 40 × 10 with a pocket 30 × 20 and 6 deep, its floor's edges rounded inside to 2,
  // its corners sharp
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
  floor := fillet(cupped.pocket.far, cupped.pocket, r: 2mm)
  floor union tray

  // a triangular prism 10 deep, one corner obtuse, its near cap rounded to 2: at the obtuse corner
  // each run carries on past the vertex to the other's side; at the acute ones each crosses the
  // other's edge in the cap
  t0 := point hint(x: 110, y: 0)
  t1 := point hint(x: 150, y: 0)
  t2 := point hint(x: 105, y: 15)
  fix(x == 110, y == 0) t0
  fix(x == 150, y == 0) t1
  fix(x == 105, y == 15) t2
  (ta := line(t0, t1)) -> (tb := line(t1, t2)) -> (tc := line(t2, t0)) -> close
  tri := face(ta, tb, tc)
  prism := solid(tri, depth: 10mm)
  wedge := solid(prism)
  cap := fillet(prism.near, prism, r: 2mm)
  cap cut wedge
}
