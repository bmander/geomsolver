// A screw thread and an auger's flight (#243 L6): a section carried along a screw.
// `std.Helix(base, pitch:, turns:, s:)` turns `base`'s frame `360deg · turns · s` about its normal
// and lifts it `pitch · turns · s` along it. A section drawn in its `axial` plane, through the
// axis, sweeps a thread; one drawn in `n`, the base so carried, sweeps a twisted prism — a flight.
// Every direction is an ordinate in `base`'s frame, so nothing is held by a seed, and a negative
// pitch winds left-handed.
// Closed forms:
//   thread: a 2 mm square centred 10 from the axis, 3 turns at a pitch of 4; an axial section
//     under a screw sweeps its area times its centroid's circle a turn (Pappus):
//     4 · 2π·10 · 3 = 240π mm³;
//   flight: a 4 × 2 blade 4–8 mm from the axis, 2½ turns rising 12 a turn; a section square to
//     the axis under a screw sweeps its area times the rise (Cavalieri): 8 · 30 = 240 mm³.
// Edit `r`, `size`, `pitch` or `turns` (a negative pitch for a left-hand thread).
unit mm
use std (coords, on_u)

r := 10mm
size := 2mm
pitch := 4mm
turns := 3

component Thread(base: plane, r: Length, size: Length, pitch: Length, turns: Scalar, s: Scalar) {
  st := std.Helix(base, pitch: pitch, turns: turns, s: s)
  in st.axial {
    c := point
    c on_u(d: r) st.axial
  }
  sq := std.CenteredRectangle(c, w: size, h: size) in st.axial
  f := face(sq.loop)
}

component Flight(base: plane, off: Length, w: Length, h: Length, pitch: Length, turns: Scalar,
                 s: Scalar) {
  st := std.Helix(base, pitch: pitch, turns: turns, s: s)
  in st.n {
    a := point
    b := point
    c := point
    d := point
    a coords(du: off - w / 2, dv: -h / 2) st.n
    b coords(du: off + w / 2, dv: -h / 2) st.n
    c coords(du: off + w / 2, dv: h / 2) st.n
    d coords(du: off - w / 2, dv: h / 2) st.n
  }
  f := face(a, b, c, d, -> close)
}

thread := solid(Thread(std.top, r: r, size: size, pitch: pitch, turns: turns).f
                over s in (0, 1))
flight := solid(Flight(std.top, off: 6mm, w: 4mm, h: 2mm, pitch: 12mm, turns: 2.5).f
                over s in (0, 1))
