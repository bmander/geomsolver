// A rope given its span and how far it sags, not its length. Nothing states how long the rope
// is: it hangs from `a` and `b`, and its lowest point, `low`, is placed half way across and
// `30mm` down. That point is on the rope, so the drawing determines the length, and the solve
// finds it — the length a catenary needs to sag that far over that span.
//
// Edit the span or the sag and the rope hangs again at the length that takes. Where nothing
// fixes the length, as here a placed point does, the energy would settle it — and a hanging
// rope has no length it is stationary in (it only lowers its energy as it lengthens), which is
// why the catenary example states its own.
unit mm
use std (horizontal)

in std.front {
  a := point
  b := point hint((100, 0))
  low := point hint((50, -30))
  fix((0mm, 0mm)) a
  a horizontal b
  a distance(100mm, along: right) b
  a distance(50mm, along: right) low
  a distance(30mm, along: down) low
  rope := curve(a, b)
  low coincident rope
}
rope minimizes integral(p.y over p)
