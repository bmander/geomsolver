// What every part of the hub motor draws with.
//
// The motor is drawn in two views of one frame.  The end view is `std.front`, looking along the
// axle from the left (brake) side: x across, z up, the axle's centre at its origin.  The axial
// half-section is `std.side`, looking from +x: u along the axle (+y, toward the drive side), v
// radially up.  Every part is placed from two points, the front's origin and the side's, so a
// frame group carries them both:
//
//   front, side   the two planes
//   o, datum      the end view's centre, and a line from it along +x the patterns are turned from
//   so, ax        the section's origin on the axle's mid-plane, and the axle's centreline from it
//
// A face drawn in the end view sweeps along the front plane's normal, which points at the
// viewer, toward −y: a prism `from: h0, to: h1` spans y from −h1 to −h0.  The parts that stand
// either side of the mid-plane say so where they sweep.

use std (offset)

// A rectangle by its corners' ordinates from `o`, in the view it is drawn in: in the section, a
// run along the axle (`x0` to `x1`) and a band of radius (`y0` to `y1`) — a turned part's
// stations and diameters, which is how it is dimensioned.
component Box(o: point, x0: Length, y0: Length, x1: Length, y1: Length) {
  a := point
  b := point
  c := point
  d := point
  profile := (ab := line(a, b)) -> (bc := line(b, c)) -> (cd := line(c, d)) -> (da := line(d, a)) -> close
  a offset(dx: x0, dy: y0) o
  b offset(dx: x1, dy: y0) o
  c offset(dx: x1, dy: y1) o
  d offset(dx: x0, dy: y1) o
}

// A plain ring turned about the axle: a bearing, a spacer, a locknut, a winding's end turns.
// Drawn in the section, `s0` to `s1` along the axle and `r0` to `r1` from it.
component Revolved(f: group, s0: Length, s1: Length, r0: Length, r1: Length) {
  in f.side {
    sec := Box(f.so, x0: s0, y0: r0, x1: s1, y1: r1)
  }
  body := solid(sec.profile, about: f.ax)
}

// A circle of `n` holes of diameter `d` on radius `r` about the end view's centre, the first at
// `phase` from the datum: drawn once and turned (`ring`), so a pattern is one hole solved for.
component HoleCircle(f: group, n: Int, r: Length, d: Length, phase: Angle) {
  in f.front {
    ring n about f.o {
      c := point
      f.o distance(r) c
      hole := circle(center: c) hint(r: d / 2)
      radius(d / 2) hole
      private construction spoke := line(f.o, c)
    }
  }
  f.datum angle(phase) spoke[0]
}

// A point placed by its two ordinates from `o`: in the section, its station along the axle and
// its radius — one shoulder of a turned profile.
component At(o: point, dx: Length, dy: Length) {
  p := point
  p offset(dx: dx, dy: dy) o
}
