// X0 (#183): solid_fillet_lid.sv, mitres: a block's top rim rounded alone, a pocket's floor
// rounded alone and a triangular prism's cap rounded alone. Where two rounded edges meet at a
// corner whose third edge is sharp, each run carries on to the face beyond it and the two cross,
// as OCCT does. Nothing here says "mitre": it is what the definition does at a sharp third edge
// (X2 checks that). Does not parse yet.
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

  // a triangle, one corner obtuse
  t0 := point
  t1 := point
  t2 := point
  fix((110, 0)) t0
  fix((150, 0)) t1
  fix((105, 15)) t2
  (ta := line(t0, t1)) -> (tb := line(t1, t2)) -> (tc := line(t2, t0)) -> close
  tri := face(ta, tb, tc)
}
// The block's near cap's rim: where the near cap meets the block's other faces. The upright edges
// are block against block too, but not against the near cap, so they stay sharp.
block := solid(outline, depth: 12mm)
block.near round(3mm) block

// The pocket's floor: where the floor meets the pocket's walls. The upright corners stay sharp.
plate := solid(top, depth: 10mm)
pocket := solid(mouth, from: 1mm, to: -6mm)
tray := solid(plate)
pocket cut tray
tray.pocket.far round(2mm) tray.pocket

// A triangular prism's near cap, 10 deep: at the obtuse corner each run carries on past the
// vertex to the other's side; at the acute ones each crosses the other's edge in the cap.
prism := solid(tri, depth: 10mm)
prism.near round(2mm) prism
