// D0 (#339): the tractrix, in the proposed spelling. Does not parse: it stops at `rate`.
//
// A point dragged by a string of length `a` whose other end runs along a line. The body says
// what holds at every place — the string's end on the line, `a · u` along it, the string `a`
// long — and one rate row: the point moves only along the string. The body is one row short of
// square, and the rate row is that row.
//
// The home is the drawn instance (`u: 0`): what is said of it outside the body holds there alone,
// here that the string starts square to the line. `Dragged` says the same in its own body, for an
// instance written in place, which has no sheet to say it on.
//
// Closed form: x = a(u − tanh u), y = a sech u, the string's end at (a·u, 0). The home is the
// cusp, where the point's speed is zero: the rate rows are regular there all the same.
unit mm
use std

component Drag(o: point, a: Length, u: Scalar) {
  q := point
  p := point hint((0, 30))
  q level(up) o
  o distance(a * u, along: right) q        // the string's end runs along the line
  string := line(p, q)
  length(a) string
  rate(p over u) parallel string            // dragged: p moves only along the string
}

component Dragged(o: point, a: Length, u: Scalar) {
  d := Drag(o, a: a, u: u)
  initially {
    o distance(a, along: up) d.p            // at the home alone: the string square to the line
  }
}

in std.front {
  o := point
  fix((0mm, 0mm)) o
  drag := Drag(o, a: 30mm, u: 0)
  o distance(30mm, along: up) drag.p
}
tractrix := drag.p over u in (0, 3)
written := Dragged(o, a: 30mm).d.p over u in (-3, 3)     // from the home at the call's 0, both ways
