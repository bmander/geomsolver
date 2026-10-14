// L0 (#243): the library word that cannot be written yet, apart from the programs so that what it
// needs (README, finding 8) does not stop the programs that do not use it. The station words it
// stood beside moved into `std` at L3, `On` over any path among them.
use std (coords)

// The retired `solid(a, b, along: g)` as a word (L8): the section at `s` is `a`'s walk with every
// corner mixed with the corner of `b` written in its place, each read in its own plane's
// coordinates and drawn in the station's. Not writable yet (README, finding 8): it repeats over a
// face's walk, zips two walks, reads a corner's coordinates as a number outside a trace body, and
// would have to make an arc where the walk has an arc.
component Ruled(a: face, b: face, g: line, up: axis, s: Scalar) {
  st := std.OnLine(g, up: up, s: s)
  cycle (ea, eb) in (a, b) {
    in st.n {
      q := point
      q coords(du: (1 - s) * ea.p1.x + s * eb.p1.x, dv: (1 - s) * ea.p1.y + s * eb.p1.y) st.n
      e := line(q, next.q)
    }
  }
  f := face(e)
}
