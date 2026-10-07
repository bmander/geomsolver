// A regular n-gon from one definition: a component whose body is a corner and a side, n times
// round a `ring` that ends mid-joint (§6.6, issue #38) — the trailing joint welds each copy's
// side onto the next copy's corner, and the wrap closes the loop.  `n` is a parameter, so
// `Ngon(n: 5, …)` and `Ngon(n: 12, …)` are one drawing rule at two counts.
//
// A `ring` (§12.3, issue #96): every corner is the first turned a step of `360°/n` about the
// circle's centre, so the sides are equal and the polygon winds once, convex, by construction —
// no chain of equalities with its one redundancy, no seeds walking the circle to choose the
// winding among the stars and zigzags equal chords would also allow.  One corner is solved for:
// on the circle, and the circle sized by one side.

use std

component Ngon(n: Int, side: Length) {
  // seeds track both parameters: the radius the side demands, not a number frozen at one size —
  // seeded at 30, the solve must inflate the figure by side/(2 sin(pi/n))/30 and runs out of
  // iterations near n = 185; seeded here, n runs to the flattener's statement cap
  r0 := side / (2 * sin(tau / (2 * n)))
  c := circle hint(r: r0)
  ring n about c.center {
    p := point hint(at: c, bearing: 90deg)
    p coincident c
    (s := line(p)) ->
  }
  // one side sized, and the radius follows — a dimensioned radius would let the sides collapse
  distance(side) s[0]
}

// the hub and the sides are reached by their names: `five.c`, `five.s[0]`
in std.front {
  five := Ngon(n: 5, side: 40)
  fix((0, 0)) five.c.center
}
