// What is cut from a body is shelled with it: the walls run round a pocket and through a bore.
//
// Within the tray, the cavity is the slab's cavity less everything within `wall` of the pocket:
// the cavity of a body less a cut is the body's cavity less the cut grown by the thickness. So a
// wall stands round the pocket's sides and under its floor, rounded within its floor's edges and
// corners as the pocket grown is. The tray is open at its top, through the pocket's rim.
//
// The plate is sealed, and its bore leaves a tube of wall round it.
//
// Both built exactly: the tray to 14720 less 38 × 38 × 9 less the pocket grown below the top (its
// offset rectangle's 392 + π over 4, its rounded floor's 320 + 18π + 2π/3); the plate to
// 8000 − 180π less 38 × 38 × 3 less 147π (a cylinder of radius 7 through the 3).
unit mm
use std
use std.rounds (shell)

wall := 1mm

in std.top {
  q0 := point
  q1 := point
  q2 := point
  q3 := point
  fix((0, 0)) q0
  fix((40, 0)) q1
  fix((40, 40)) q2
  fix((0, 40)) q3
  (qs := line(q0, q1)) -> (qe := line(q1, q2)) -> (qn := line(q2, q3)) -> (qw := line(q3, q0)) -> close
  slab := face(qs, qe, qn, qw)
  k0 := point
  k1 := point
  k2 := point
  k3 := point
  fix((10, 12)) k0
  fix((30, 12)) k1
  fix((30, 28)) k2
  fix((10, 28)) k3
  (ks := line(k0, k1)) -> (ke := line(k1, k2)) -> (kn := line(k2, k3)) -> (kw := line(k3, k0)) -> close
  opening := face(ks, ke, kn, kw)
  p0 := point
  p1 := point
  p2 := point
  p3 := point
  fix((80, -20)) p0
  fix((120, -20)) p1
  fix((120, 20)) p2
  fix((80, 20)) p3
  (ps := line(p0, p1)) -> (pe := line(p1, p2)) -> (pn := line(p2, p3)) -> (pw := line(p3, p0)) -> close
  square := face(ps, pe, pn, pw)
  bc := point
  fix((100, 0)) bc
  bore := radius(6mm) circle(center: bc)
}
tray := solid(slab, from: 0mm, to: 10mm)
pocket := solid(opening, from: 6mm, to: 11mm)
pocket cut tray
plate := solid(square, from: 0mm, to: 5mm)
drill := solid(face(bore), from: -1mm, to: 6mm)
drill cut plate

tray.near shell(t: wall) tray
shell(t: wall) plate
