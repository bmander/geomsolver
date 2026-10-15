// D0 (#339): the hanging chain as a stated flow, in the proposed spelling. Does not parse: it
// stops at `rate`.
//
// The forces on the chain between the home and `p`: a horizontal pull `c` and a vertical one that
// grows by the weight of the chain, `v + s` (s the arc length, the weight a unit a length). The
// chain runs along the pull, at unit speed. `c` and `v` are unbound formals of the drawn
// instance, so unknowns of the drawing; the far end landing on `b` settles them. That is a
// two-point problem the drawing solves by shooting: the contact at s = 150 has a Jacobian in `c`
// and `v` from the variational equations, as §9.10's free curve has in its ends.
//
// Closed form: today's free curve (`rust/examples/catenary.sv`, `tests/catenary.rs`), a to b,
// 150 long: c = 30.8236… (the root of c·sinh(50/c) = 75), v = −75, and
// x = 50 + c·asinh((s − 75)/c), y = √(c² + (s − 75)²) − √(c² + 75²).
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
}
chain := hang.p over s in (0mm, 150mm)
