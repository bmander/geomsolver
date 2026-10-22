// A screw thread and an auger's flight (#243 L6): a section carried along a screw.
// The thread is the epic's helix (#391, #398): a frame turning about the top plane's normal and
// rising `pitch` a turn, whose value is `360deg · turns · s`, and the square drawn in its block
// on the front plane, through the axis — it stands where the value puts it, so swept as `s` runs
// it sweeps a thread. The flight is `std.Helix(base, pitch:, turns:, s:)`'s station, `base`
// carried by the same screw about its normal (#366): a section drawn in its `n`, square to the
// axis, sweeps a twisted prism. Each is the screw's sweep of its section, exactly, and a negative
// pitch winds left-handed.
// Closed forms:
//   thread: a 2 mm square centred 10 from the axis, 3 turns at a pitch of 4; an axial section
//     under a screw sweeps its area times its centroid's circle a turn (Pappus):
//     4 · 2π·10 · 3 = 240π mm³;
//   flight: a 4 × 2 blade 4–8 mm from the axis, 2½ turns rising 12 a turn; a section square to
//     the axis under a screw sweeps its area times the rise (Cavalieri): 8 · 30 = 240 mm³.
// Edit `r`, `size`, `pitch` or `turns` (a negative pitch for a left-hand thread).
unit mm
use std (coords, on_u, screws)

r := 10mm
size := 2mm
pitch := 4mm
turns := 3

component Flight(off: Length, w: Length, h: Length, pitch: Length, turns: Scalar, s: Scalar) {
  st := std.Helix(std.top, pitch: pitch, turns: turns, s: s)
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

param s: Scalar over (0, 1)

// the screw, at its value over `s`, and the square drawn in its block on the axial plane
screw := frame
screw screws(pitch) std.top.normal
fix(value == 360deg * turns * s) screw
in screw {
  in std.front {
    c := point
    c on_u(d: r) std.front
  }
  sq := std.CenteredRectangle(c, w: size, h: size) in std.front
  sec := face(sq.loop)
}
thread := sweep(sec over s)
flight := sweep(Flight(off: 6mm, w: 4mm, h: 2mm, pitch: 12mm, turns: 2.5).f over s)
