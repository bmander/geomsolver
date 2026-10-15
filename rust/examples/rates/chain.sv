// A hanging chain stated as a flow, beside the same chain stated by its principle.
//
// The flow says what holds at each point of the chain: the forces on the piece between its start
// and the point are a horizontal pull `c` and a vertical one `v + s` — the start's, grown by the
// weight of the `s` of chain below (a unit of weight a unit of length). The chain runs along the
// pull, at unit speed, so `s` is arc length:
//
//   rate(p over s) parallel tl              // along the pull...
//   rate(p over s) distance(1, along: tl)   // ...at unit speed
//
// `c` and `v` are left unbound, so they are unknowns of the drawing, and the chain's far end lands
// on `b` after its length: `b coincident(t == 150mm) chain`. That is a two-point problem, and the
// drawing solves it as it solves anything: the contact's gradient in `c` and `v` is the flow's
// sensitivity to them, carried through the integration (#339 D2). The solve shoots.
//
// `rope` is the same chain stated as what it does instead: a free curve of the same length between
// the same points that makes its potential energy least (`catenary.sv`). Nothing ties the two
// together; they come out the same curve.
//
// Closed form: c is the root of c sinh(50/c) = 75 (30.8236…), v = −75, and
// x = 50 + c asinh((s − 75)/c), y = √(c² + (s − 75)²) − √(c² + 75²).
unit mm
use std

component Chain(c: Length, v: Length, s: Length) {
  p := point
  q := point
  tl := line(p, q)
  p distance(c, along: right) q              // the pull at p: its run...
  p distance(v + s, along: up) q             // ...and its rise, the weight below added
  rate(p over s) parallel tl                 // the chain runs along the pull
  rate(p over s) distance(1, along: tl)      // at unit speed: s is arc length
}

in std.front {
  a := point
  b := point
  fix((0mm, 0mm)) a
  fix((100mm, 0mm)) b
  hang := Chain(c: hint(30mm), v: hint(-75mm), s: 0mm)
  hang.p coincident a                        // the home: the chain leaves a
  b coincident(t == 150mm) chain             // ...and reaches b after its length
  rope := curve(a, b)
  length(150mm) rope
}
chain := hang.p over s in (0mm, 150mm)
rope minimizes integral(p.y over p)
