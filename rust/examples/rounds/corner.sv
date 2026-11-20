// A tray whose pocket's floor is rounded as tightly as its corners (#962): a pocket 60 × 40 cut
// 10 deep into a plate 120 × 80 × 20, its plan corners drawn round to 5, and the floor's edges
// rounded to 5 too.  Along each wall the ball rolls as ever; round each corner its centre stands
// on the corner's axis, so the round there is a quarter of its ball — the torus a smaller ball
// turns has closed onto the axis into a sphere.  The solid is the plate less the pocket, plus the
// round's section along the walls and a quarter of (a cylinder less a hemisphere) at each corner:
// 192000 − 10 (2400 − 25 (4 − π)) + 160 (1 − π/4) 25 + 125π/3 = 169203.909 mm³.
//
// Edit `r` and apply (⌘↵): under 5 each corner is a spindle torus; over 5 the ball cannot reach
// into the corner — it touches the walls either side of it and never the corner itself — and is
// refused, saying so.
unit mm
use std
use std.solids (cut)
use std.rounds (round)

corner := 5mm   // the pocket's plan corners
r := 5mm        // the floor's round

in std.top {
  a := point
  b := point
  c := point
  d := point
  fix((-60, -40)) a
  fix((60, -40)) b
  fix((60, 40)) c
  fix((-60, 40)) d
  outline := face(a, b, c, d, -> close)
  p := point
  q := point
  s := point
  t := point
  fix((-30, -20)) p
  fix((30, -20)) q
  fix((30, 20)) s
  fix((-30, 20)) t
  (south := line(p, q)) -> (se := round(r: corner)) (east := line(q, s)) -> (ne := round(r: corner)) (north := line(s, t)) -> (nw := round(r: corner)) (west := line(t, p)) -> (sw := round(r: corner)) close
  mouth := face(south, se, east, ne, north, nw, west, sw)
}
plate := solid(std.solids.Prism(outline, from: 0mm, to: 20mm))
pocket := solid(std.solids.Prism(mouth, from: 10mm, to: 21mm))
pocket cut plate

plate.pocket.far round(r: r) plate.pocket
