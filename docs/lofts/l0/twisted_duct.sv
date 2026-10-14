// L0 (#243): solid_twist.sv as a law. A 20 mm square duct turned a quarter as it runs 40 mm: the
// turn is a dimension of the section reading the station's `s`, so an eased turn is the same
// statement with another expression (`twist * (3 * s^2 - 2 * s^3)`), and a turn past a half is no
// different from a quarter. At L0 the twist was only the pairing of a square with a diamond.
// Closed form (Cavalieri): every section is the 20 mm square, so V = 20² · 40 = 16000 mm³ at any
// turn and under any law.
unit mm
use std (coords)
length := 40mm

// The square at station `s`, drawn on the station's axes turned `twist · s` (`std.Twisted`, #243
// L5). An eased turn is the word given another fraction: `s: 3 * s^2 - 2 * s^3`.
component TwistedSquare(g: line, up: axis, half: Length, twist: Angle, s: Scalar) {
  st := std.OnLine(g, up: up, s: s)
  tw := std.Twisted(st.n, twist: twist, s: s)
  in st.n {
    a := point
    b := point
    c := point
    d := point
    a coords(du: -half, dv: -half) tw.axes
    b coords(du: half, dv: -half) tw.axes
    c coords(du: half, dv: half) tw.axes
    d coords(du: -half, dv: half) tw.axes
  }
  f := face(a, b, c, d, -> close)
}

in std.top {
  entry := point
  fix((0mm, 0mm)) entry
  exit := point hint((0mm, 40mm))
  guide := line(entry, exit)
  vertical guide
  length(length) guide
}
duct := solid(TwistedSquare(guide, std.z, half: 10mm, twist: 90deg).f over s in (0, 1))
// Three and a half turns: no pairing can say this.
auger := solid(TwistedSquare(guide, std.z, half: 10mm, twist: 1260deg).f over s in (0, 1))
