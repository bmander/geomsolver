// A tray with a pocket cut into it, the pocket rounded inside. Every edge among the pocket's own
// faces is concave: where its floor meets its walls and where two walls meet. A ball rolled along
// such an edge, outside both faces, leaves material behind, so here `round` fills rather than
// cuts, and where three of those edges meet at a corner of the floor the fill is a ball's.
//
// `dish.pocket` names the faces the pocket gave the dish, as the dish stands: a face a Boolean ate
// names nothing. Edit `r` and apply (⌘↵). Write `dish.pocket round(r: r) dish.tray.near` instead
// to round the pocket's rim, where its walls meet the face it was cut into: those edges are
// convex, so that round cuts.
unit mm
use std (round)

r := 1.5mm   // the pocket's inside radius

in std.front {
  t0 := point
  t1 := point
  t2 := point
  t3 := point
  fix((0, 0)) t0
  fix((60, 0)) t1
  fix((60, 40)) t2
  fix((0, 40)) t3
  (ts := line(t0, t1)) -> (te := line(t1, t2)) -> (tn := line(t2, t3)) -> (tw := line(t3, t0)) -> close
  outline := face(ts, te, tn, tw)
  k0 := point
  k1 := point
  k2 := point
  k3 := point
  fix((15, 10)) k0
  fix((45, 10)) k1
  fix((45, 30)) k2
  fix((15, 30)) k3
  (ks := line(k0, k1)) -> (ke := line(k1, k2)) -> (kn := line(k2, k3)) -> (kw := line(k3, k0)) -> close
  opening := face(ks, ke, kn, kw)
}
tray := solid(outline, depth: 10mm)
pocket := solid(opening, from: -6mm, to: 1mm)
dish := solid(tray)
pocket cut dish

round(r: r) dish.pocket
