// L0 (#243): solid_twist.sv as a law. A 20 mm square duct turned a quarter as it runs 40 mm: the
// turn is a dimension in the section reading the station's `s`, so an eased turn is the same
// statement with another expression (`twist * (3 * s^2 - 2 * s^3)`), and a turn past a half is no
// different from a quarter. Today the twist is only the pairing of a square with a diamond.
// Closed form (Cavalieri): every section is the 20 mm square, so V = 20² · 40 = 16000 mm³ at any
// turn and under any law.
unit mm
use std
use std.place (on_u, coords)
use lofts
length := 40mm

// The square at station `s`, turned `twist · s` from the station's own `u`.
component TwistedSquare(g: line, up: axis, half: Length, twist: Angle, s: Scalar) {
  st := lofts.OnLine(g, up: up, s: s)
  in st.n {
    k := point
    k on_u(d: half) st.n
    t := point
    st.n.origin distance(half) t
    (square := line(st.n.origin, k)) angle(twist * s, sense: ccw) (turned := line(st.n.origin, t))
  }
  tw := std.Turned(st.n.origin, t) in st.n
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
duct := TwistedSquare(guide, std.z, half: 10mm, twist: 90deg, s: 0.5)
// Three and a half turns: no pairing can say this.
auger := TwistedSquare(guide, std.z, half: 10mm, twist: 1260deg, s: 0.5)
