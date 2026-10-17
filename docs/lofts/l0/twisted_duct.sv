// L0 (#243): solid_twist.sv as a law. A 20 mm square duct turned a quarter as it runs 40 mm: the
// turn is a dimension of the section reading the station's `s`, so an eased turn is the same
// statement with another expression (`twist * (3 * s^2 - 2 * s^3)`), and a turn past a half is no
// different from a quarter. At L0 the twist was only the pairing of a square with a diamond.
// Closed form (Cavalieri): every section is the 20 mm square, so V = 20² · 40 = 16000 mm³ at any
// turn and under any law.
unit mm
use std (vertical)
length := 40mm

// The square at station `s`, drawn once below and carried whole onto the station by the
// similarity taking its centre `o` and `x` to the station's origin and `std.Twisted`'s point,
// turned `twist · s` (#243 L5, #357). An eased turn is the word given another fraction:
// `s: 3 * s^2 - 2 * s^3`.
component TwistedSquare(g: line, up: axis, prof: face, o: point, x: point, twist: Angle,
                        s: Scalar) {
  st := std.OnLine(g, up: up, s: s)
  tw := std.Twisted(st.n, twist: twist, reach: 10mm, s: s)
  f := prof mapped(o -> st.n.origin, x -> tw.t) in st.n
}

in std.front {
  o := point
  x := point
  fix((0mm, -50mm)) o
  fix((10mm, -50mm)) x
  a := point
  b := point
  c := point
  d := point
  fix((-10mm, -60mm)) a
  fix((10mm, -60mm)) b
  fix((10mm, -40mm)) c
  fix((-10mm, -40mm)) d
}
square := face(a, b, c, d, -> close)

in std.top {
  entry := point
  fix((0mm, 0mm)) entry
  exit := point hint((0mm, 40mm))
  guide := line(entry, exit)
  vertical guide
  length(length) guide
}
param s: Scalar over (0, 1)
duct := sweep(TwistedSquare(guide, std.z, square, o, x, twist: 90deg).f over s)
// Three and a half turns: no pairing can say this.
auger := sweep(TwistedSquare(guide, std.z, square, o, x, twist: 1260deg).f over s)
