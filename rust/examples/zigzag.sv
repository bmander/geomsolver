// Staircases of level and plumb segments, with no lengths given anywhere.
//
// Every segment is either horizontal or vertical, and nothing says how long any of them is.  So
// each staircase is a long slack chain that knows only which way each step points, and several
// of them sit side by side, sharing nothing at all.
//
// This is a benchmark rather than a drawing of anything.  Dragging one point of one staircase
// ought to cost only that staircase: a document made of separate pieces should not get slower to
// edit just because it contains more pieces.  The trap it is built to catch is that every level
// segment in the file points the *same way*, so a solver that mistakes "shares a direction" for
// "is connected to" would decide the whole document is one figure and pay for all of it on every
// drag.
//
// The steps are written as two runs rather than one alternating run because a staircase of `n`
// points has `floor(n / 2)` vertical steps and `floor((n - 1) / 2)` horizontal ones — the counts
// differ when `n` is even, and written this way the arithmetic settles it instead of each step
// having to ask which kind it is.

param n = 32
param copies = 3

repeat copies as c {
  // the staircase's own points: up 5 on every odd step, along 3 on every even one
  repeat n as i {
    point p hint at (4 * n * c + 3 * floor(i / 2), 5 * floor((i + 1) / 2))
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
