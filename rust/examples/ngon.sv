// A regular n-gon from one definition: a component whose body is a corner and a side, n times
// round a `cycle` that ends mid-joint (§6.6, issue #38) — the trailing joint welds each copy's
// side onto the next copy's corner, and the wrap closes the loop.  `n` is a parameter, so
// `Ngon(n: 5, …)` and `Ngon(n: 12, …)` are one drawing rule at two counts.
//
// The corners sit on a circle and consecutive sides are equal — pure relations, so the one
// redundancy a closed loop of equalities carries is a theorem the diagnosis notes as implied
// and never paints, where dimensioning every corner's angle would honestly read Over.  What
// the relations cannot say is the *winding*: equal chords of a circle fix each central angle's
// size and not its sign, so the collapsed polygon, the zigzags and every star satisfy them
// too.  Which of those this drawing is, is a branch, and a branch is chosen where a residual
// cannot state it — by the seeds, each corner seeded one step further round the circle, which
// is the statement "convex, once around".

component Ngon(n: Int, side: Length) {
  circle c hint(r: 30)
  cycle n as i {
    point p hint(x: 30 * cos(tau * i / n), y: 30 * sin(tau * i / n))
    p on c
    line s(p) -> equal
  }
  // one side sized, and the radius follows — a dimensioned radius would let the sides collapse
  s[0].p1 distance(side) s[0].p2
  // the hub and the first side, for a caller to hold
  port hub = c
  port first = s[0]
}

five: Ngon(n: 5, side: 40)
ground five.hub.center
