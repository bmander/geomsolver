// A tractrix: the path of a point dragged by a string of length `a` whose other end runs along a
// line. The body says what holds at every place, and one thing about how the point moves:
//
//   - the string's end `q` is on the line, `a · u` along it;
//   - the string is `a` long;
//   - `rate(p over u) parallel string`: the point is dragged, so it moves only along the string.
//
// That rate row is what a body with a rate adds (#339 D2). It is not a row at the home, so the
// instance drawn below (`drag`, at `u: 0`) is short one freedom there: the string's bearing. The
// sheet holds that freedom by starting the string square to the line. From that home the curve is
// integrated, and the home is a cusp, where the point's speed is zero: the rows that fix the rates
// still have full rank there.
//
// `Dragged` is the same body written in place. It has no sheet to say where it starts, so it says
// it itself, `initially { … }`, and it hands the running formal on as itself (`u: u`), so `Drag`'s
// rate is along the curve's formal. Its curve runs from its home at the call's `u: 0` both ways.
//
// Away from the home, a drawn copy of the body stands where the curve is (`there`). The string's
// length is one of its rows, and the rate row becomes a claim: the curve's tangent at its point
// runs along the string.
//
// Closed form: x = a(u − tanh u), y = a sech u, the string's end at (a·u, 0).
unit mm
use std

a := 30mm
u1 := 1.2

component Drag(o: point, a: Length, u: Scalar) {
  q := point
  p := point hint((0, 30))
  q level(up) o
  o distance(a * u, along: right) q          // the string's end runs along the line
  string := line(p, q)
  length(a) string
  rate(p over u) parallel string             // dragged: p moves only along the string
}

component Dragged(o: point, a: Length, u: Scalar) {
  d := Drag(o, a: a, u: u)
  initially {
    d.p level(right) o                       // at the home: the string square to the line
  }
}

in std.front {
  o := point
  fix((0mm, 0mm)) o
  drag := Drag(o, a: a, u: 0)
  drag.p level(right) o                      // the home: the string starts square to the line
}
tractrix := drag.p over u in (0, 3)
both_ways := Dragged(o, a: a, u: 0).d.p over u in (-3, 3)

in std.front {
  m := point hint((14, 16))
  m coincident(t == u1) tractrix             // the curve's point at u1...
  q1 := point hint((36, 0))
  q1 level(up) o
  o distance(a * u1, along: right) q1        // ...the string's end there...
  k := point hint((20, 10))
  tl := line(m, k)
  length(10mm) tl
  tractrix tangent(at: m) tl                 // ...and the curve's tangent there
  string1 := line(m, q1)
  claim length(a) string1                    // the body's row holds where the flow took it
  claim tl parallel string1                  // and its rate row: the tangent is the string
}
