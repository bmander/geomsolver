// Staircases of levelled segments, and nothing else said about them.
//
// Every segment is either vertical or horizontal and *no length is given anywhere*, so each
// staircase is a long chain of free links that only knows which way each link points.  That makes
// it the drawing where a cost that goes with the *direction class* rather than with the geometry
// shows up: every levelled line in the document lands in the ground x-axis's class, so a
// decomposition that treats a shared class as an adjacency makes every staircase a neighbour of
// every other.  `copies` of them, side by side and sharing nothing, then say whether a drag costs
// the figure it moves or the whole document.
//
// The alternation is written as two runs rather than as one alternating run, because a staircase
// of `n` points has `floor(n / 2)` vertical links and `floor((n - 1) / 2)` horizontal ones — the
// counts differ by one when `n` is even, and a single run would need to ask which segment it was
// on.  Said this way each run states one direction and the arithmetic settles the parity.

param n = 32
param copies = 3

repeat copies as c {
  // the staircase's own points: up 5 on every odd step, along 3 on every even one
  repeat n as i {
    point p at (4 * n * c + 3 * floor(i / 2), 5 * floor((i + 1) / 2))
  }

  repeat floor(n / 2) as k {
    line v(p[2 * k], p[2 * k + 1])
    vertical(v)
  }
  repeat floor((n - 1) / 2) as k {
    line h(p[2 * k + 1], p[2 * k + 2])
    horizontal(h)
  }
}
