// L0 (#243): the library words that cannot be written yet, apart from `lofts.sv` so that what
// they need (README, findings 5 and 8) does not stop the programs that do not use them.
use std (square)
use lofts

// The one word the three above stand in for: the plane square to any path `g` — a line, an arc, a
// spline, a traced curve, an envelope, or a named chain of them run link by link — where its
// parameter is `s`, its `v` along `up`. It asks two things of the language (README, findings 2
// and 5): `coincident(t == s)` on every kind of path, and the tangent at a point of every kind.
component On(g: path, up: axis, s: Scalar) {
  m := point
  m coincident(t == s) g
  q := point
  tl := line(m, q)
  g tangent(at: m) tl
  n := plane
  n.origin coincident m
  n square(up: up) tl
}

// Today's `solid(a, b, along: g)` as a word (L3): the section at `s` is `a`'s walk with every
// corner mixed with the corner of `b` written in its place, each read in its own plane's
// coordinates and drawn in the station's. Not writable yet (README, finding 8): it repeats over a
// face's walk, zips two walks, reads a corner's coordinates as a number outside a trace body, and
// would have to make an arc where the walk has an arc.
component Ruled(a: face, b: face, g: line, up: axis, s: Scalar) {
  st := lofts.OnLine(g, up: up, s: s)
  cycle (ea, eb) in (a, b) {
    in st.n {
      q := point
      q coords(du: (1 - s) * ea.p1.x + s * eb.p1.x, dv: (1 - s) * ea.p1.y + s * eb.p1.y) st.n
      e := line(q, next.q)
    }
  }
  f := face(e)
}
