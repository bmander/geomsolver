// A regular n-gon from one definition: a component whose body is a corner and a side, n times
// round a `ring` that ends mid-joint (§6.6, issue #38) — the trailing joint welds each copy's
// side onto the next copy's corner, and the wrap closes the loop.  `n` is a parameter, so
// `Ngon(n: 5, …)` and `Ngon(n: 12, …)` are one drawing rule at two counts.
//
// A `ring` (§12.3, issue #96): every corner is the first turned a step of `360°/n` about the
// circle's centre, so the sides are equal and the polygon winds once, convex, by construction —
// no chain of equalities with its one redundancy, no seeds walking the circle to choose the
// winding among the stars and zigzags equal chords would also allow.  One corner is solved for:
// on the circle, and the circle sized by one side.  Which way the polygon is turned about its hub
// is the first side's heading, `heading`: a formal, so the call states it, or leaves it out and the
// drawing keeps it free — as here, where it spins about its hub.

use std

component Ngon(n: Int, side: Length, heading: Angle) {
  // seeds track every parameter: the radius the side demands, not a number frozen at one size —
  // seeded at 30, the solve must inflate the figure by side/(2 sin(pi/n))/30 and runs out of
  // iterations near n = 185; seeded here, n runs to the flattener's statement cap — and the
  // corner where the first side leaves at `heading`, a quarter turn and half a step behind it
  r0 := side / (2 * sin(tau / (2 * n)))
  c := circle hint(r: r0)
  ring n about c.center {
    p := point hint(at: c, bearing: heading - 90deg - 180deg / n)
    p coincident c
    (s := line(p)) ->
  }
  // one side sized, and the radius follows — a dimensioned radius would let the sides collapse
  length(side) s[0]
  s[0] angle(heading) std.x
}

// the hub and the sides are reached by their names: `five.c`, `five.s[0]`; its first corner
// starts straight up from the hub, and the drawing is free to turn it
in std.front {
  five := Ngon(n: 5, side: 40, heading: hint(216deg))
  fix((0, 0)) five.c.center
}
