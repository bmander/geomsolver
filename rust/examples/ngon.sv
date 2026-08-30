// A regular n-gon from one definition: a component whose body is one line and one corner,
// n times round a `cycle` that ends mid-joint (§6.6, issue #38) — the trailing joint welds
// each copy's side onto the next's first link, and the wrap closes the loop.  `n` is a
// parameter, so `Ngon(n: 5, …)` and `Ngon(n: 12, …)` are one drawing rule at two counts.
//
// Round a closed loop, every corner stated is one statement too many: the last corner is the
// polygon's closure theorem.  The `equal` cycle is pure relations, so its redundancy is noted
// as *implied* and never painted; the `angle` cycle carries a dimension, so the diagnosis
// says Over — "remove one" — which is the honest reading of dimensioning all n corners of a
// figure that closes.

component Ngon(n: Int, side: Length) {
  cycle n {
    line s -> angle(360 / n) equal
  }
  s[0].p1 distance(side) s[0].p2
  // the first side, for a caller to hold: a port names what an instance may reach
  port first = s[0]
}

five: Ngon(n: 5, side: 40)
ground five.first.p1
