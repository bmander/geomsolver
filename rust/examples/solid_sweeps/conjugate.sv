// A wheel cut by a plus-shaped tool always in mesh with it, the conjugate a gear shaper would cut
// (#585): the plus turns about its stem while the wheel turns two thirds as fast the other way about
// a spindle 50 mm off, and over a whole period of the pair — 540°, after which the plus stands on
// itself again — everything its arms pass through, seen from the wheel, is gone. The tool is never
// clear of the blank, so no sheet ends; instead the region the plus's section sweeps in the plane
// is read exactly (its arms' contacts, their corners' paths), kept where it lies outside the plus
// at every pose, and the wheel is the blank less that region's prism (the planar subtraction
// class, docs/generating-sweeps.md).
unit mm
use std
use std.place (vertical, coords)
use std.motion (turns)

in std.front {
  construction stem := vertical line(std.origin, hint((0, 10)))
  length(10mm) stem
  wc := point hint((-50, 0))
  std.origin distance(-50mm, along: u) wc
  construction spindle := vertical line(wc, hint((-50, 10)))
  length(10mm) spindle
}
// the plus, four arms 30 long and 8 wide, and the wheel's rim 40 about the spindle
in std.top {
  repeat 4 as k {
    a := point hint((30 * cos(90deg * k) - 4 * sin(90deg * k), 30 * sin(90deg * k) + 4 * cos(90deg * k)))
    b := point hint((30 * cos(90deg * k) + 4 * sin(90deg * k), 30 * sin(90deg * k) - 4 * cos(90deg * k)))
    c := point hint((4 * cos(90deg * k + 45deg) * sqrt(2), 4 * sin(90deg * k + 45deg) * sqrt(2)))
    a coords(du: 30mm * cos(90deg * k) - 4mm * sin(90deg * k), dv: 30mm * sin(90deg * k) + 4mm * cos(90deg * k)) std.top
    b coords(du: 30mm * cos(90deg * k) + 4mm * sin(90deg * k), dv: 30mm * sin(90deg * k) - 4mm * cos(90deg * k)) std.top
    c coords(du: 4mm * sqrt(2) * cos(90deg * k - 45deg), dv: 4mm * sqrt(2) * sin(90deg * k - 45deg)) std.top
  }
  plus := face(c[0], b[0], a[0], c[1], b[1], a[1], c[2], b[2], a[2], c[3], b[3], a[3], -> close)
  wo := point
  wo coincident wc
  rim := circle(center: wo) hint(r: 40)
  radius(40mm) rim
}
construction star := solid(plus, from: -10mm, to: 0mm)

// the plus turning about its stem, the wheel two thirds as fast the other way about its spindle
tool_turn := frame
tool_turn turns(1) stem
wheel_turn := frame
wheel_turn turns(-4 / 6) spindle
blank := solid(face(rim), from: -8mm, to: -2mm)
wheel := solid(blank)
construction space := sweep(star over t in (0deg, 540deg), under: tool_turn, seen_from: wheel_turn)
space cut wheel
