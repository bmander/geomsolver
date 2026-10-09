// A hanging rope, stated as what it is: a curve of fixed length between two points that makes
// its potential energy least. Not its formula, `a cosh((x - x0) / a) + c` — its principle.
//
// `curve(a, b)` is a free curve: its ends are the two points written, and its shape is the
// drawing's to find. `length(150mm) rope` fixes how long it is, and `rope minimizes` says what it
// does with the rest of its freedom: it hangs, lowering its height integrated along its length. The
// solve finds the shape where that energy is stationary among every shape of that length, and
// the report says the stationary shape is a minimum.
//
// `b` is left free: drag it and the rope re-hangs, its two freedoms the drawing's only ones. Change
// the length and it sags more or less. Nothing here knows what a catenary is; that is what the
// principle comes to.
unit mm
use std

in std.front {
  a := point
  b := point hint((100, 0))
  fix((0mm, 0mm)) a
  rope := curve(a, b)
  length(150mm) rope
}
rope minimizes integral(p.y over p)
