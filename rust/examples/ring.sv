// A rope threaded through a ring that slides on a rod. The rope hangs from `a` and `b`, 160 long,
// and the rod runs below where it would hang: the ring holds the rope down to the rod, so the rope
// bends there in a corner — two catenaries meeting on the rod. Where along the rod is the rope's
// to choose, and nothing states it: `rope touches rod` says only that the rope is held to the line
// somewhere, with nothing along the line holding it, so the ring slides to where the rope's pull
// along the rod balances. It comes away from the rod at the angle it met it, as light leaves a
// mirror, and the report says the shape is a minimum.
//
// Drag `b` and the ring slides along the rod. Raise the rod until the rope would cross it, and
// the ring slides to the crossing, holding nothing: the rope hangs as if it were not there.
unit mm
use std

in std.front {
  a := point
  b := point hint((100, 0))
  fix((0mm, 0mm)) a
  r0 := point
  r1 := point
  fix((0mm, -68mm)) r0
  fix((100mm, -52mm)) r1
  rod := line(r0, r1)
  rope := curve(a, b)
  length(160mm) rope
  rope touches rod
}
rope minimizes integral(p.y over p)
