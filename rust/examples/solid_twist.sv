// A square duct turned an eighth as it runs: a square at its start, a diamond at its end, each
// corner running straight from the square's to the diamond's as the station runs along the guide,
// so each side is a twisted ruled face, not a plane. A twist of the section itself — a turn by an
// angle read from `s`, of any size and under any law — is the same statement with the corners
// written against turned axes (`docs/lofts/l0/twisted_duct.sv`).
unit mm
use std (coords)
length := 40mm
half := 10mm
reach := 12mm

// The section at station `s`: each corner the square's mixed with the diamond's, in the plane
// square to the guide there.
component Twisted(g: line, up: axis, half: Length, reach: Length, s: Scalar) {
  st := std.OnLine(g, up: up, s: s)
  in st.n {
    a := point
    b := point
    c := point
    d := point
    a coords(du: -half * (1 - s), dv: -half + (half - reach) * s) st.n
    b coords(du: half + (reach - half) * s, dv: -half * (1 - s)) st.n
    c coords(du: half * (1 - s), dv: half + (reach - half) * s) st.n
    d coords(du: -half + (half - reach) * s, dv: half * (1 - s)) st.n
  }
  f := face(a, b, c, d, -> close)
}

in std.top {
  entry := point
  fix((0mm, 0mm)) entry
  exit := point hint((0mm, length))
  construction centerline guide := line(entry, exit)
  vertical guide
  length(length) guide
}
param s: Scalar over (0, 1)
body := sweep(Twisted(guide, std.z, half: half, reach: reach).f over s)
