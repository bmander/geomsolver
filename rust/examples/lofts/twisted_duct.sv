// A twisted duct (#243 L5): a 20 mm square run 40 mm along a line and turned as it runs. The
// turn is the station's own: `std.Twisted` gives axes in the station's plane turned `twist · s`,
// and the corners are drawn on them, so a turn of three and a half turns is one statement, and
// an eased turn is the same word given another fraction (`s: 3 * s^2 - 2 * s^3`).
// Closed form (Cavalieri): every section is the same square, so V = (2·half)² · length at any
// turn and under any law, 16000 mm³ as drawn — the duct, the auger and the eased one alike.
// Edit `length`, `half` or either twist.
unit mm
use std (coords, vertical)

length := 40mm
half := 10mm

// The square at station `s`, drawn on the station's axes turned `twist · s`.
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

// The same square turned on an eased law: slow at both ends, fastest half way.
component EasedSquare(g: line, up: axis, half: Length, twist: Angle, s: Scalar) {
  st := std.OnLine(g, up: up, s: s)
  tw := std.Twisted(st.n, twist: twist, s: 3 * s^2 - 2 * s^3)
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
  exit := point hint((0, length))
  guide := line(entry, exit)
  vertical guide
  length(length) guide
}
// a quarter turn
duct := solid(TwistedSquare(guide, std.z, half: half, twist: 90deg).f over s in (0, 1))
// three and a half turns: no pairing of two sections can say this
auger := solid(TwistedSquare(guide, std.z, half: half, twist: 1260deg).f over s in (0, 1))
// a quarter turn, eased in and out
eased := solid(EasedSquare(guide, std.z, half: half, twist: 90deg).f over s in (0, 1))
