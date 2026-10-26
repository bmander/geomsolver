// What every part of the hub motor draws with.
//
// The motor is drawn in two views of one frame.  The end view is `std.front`, looking along the
// axle from the left (brake) side: x across, z up, the axle's centre at its origin.  The axial
// half-section is `std.side`, looking from +x: u along the axle (+y, toward the drive side), v
// radially up.  Every part is a function of the axle's centre and centreline in those views (a
// `Datums`), and draws in the planes they are drawn in (`in f.o.plane`, `in f.so.plane`): no view
// is handed to a part, and none is named in one.  Each reads the numbers it is made to from
// `motor.dims` as constants of its definition, the `use` at the top of its file listing them.
//
// A face drawn in the end view sweeps along the front plane's normal, which points at the
// viewer, toward −y: a prism `from: h0, to: h1` spans y from −h1 to −h0.  The parts that stand
// either side of the mid-plane say so where they sweep.

use std
use std.place (offset, horizontal)
use motor.dims

// What the motor is drawn about: in the end view, the axle's centre and a datum from it along
// +x the patterns are turned from; in the section, the axle's mid-plane on its centreline and
// the centreline itself, which every turned part turns about.
interface Datums {
  o: point
  datum: line
  so: point
  ax: line
}

// A rectangle by its corners' ordinates from `o`, in the view it is drawn in: in the section, a
// run along the axle (`x0` to `x1`) and a band of radius (`y0` to `y1`) — a turned part's
// stations and diameters, which is how it is dimensioned.
component Box(o: point, x0: Length, y0: Length, x1: Length, y1: Length) {
  a := point
  b := point
  c := point
  d := point
  profile := (ab := line(a, b)) -> (bc := line(b, c)) -> (cd := line(c, d)) ->
             (da := line(d, a)) -> close
  a offset(dx: x0, dy: y0) o
  b offset(dx: x1, dy: y0) o
  c offset(dx: x1, dy: y1) o
  d offset(dx: x0, dy: y1) o
}

// A point placed by its two ordinates from `o`: in the section, its station along the axle and
// its radius — one shoulder of a turned profile.
component At(o: point, dx: Length, dy: Length) {
  p := point
  p offset(dx: dx, dy: dy) o
}

// A plain ring turned about the axle: a bearing, a spacer, a locknut.  Drawn in the section of
// the frame it stands in, `s0` to `s1` along the axle and `r0` to `r1` from it, about its own
// axle's centreline, as the assembly draws the frame's: closed, so the left one of a pair is the
// right one mirrored in the mid-plane (`Revolved(…) mirrored(front)`, #578), one figure placed
// twice.
component Revolved(s0: Length, s1: Length, r0: Length, r1: Length) {
  in std.side {
    construction centerline ax := line(std.side.origin, hint((100, 0)))
    horizontal ax
    length(100mm) ax
    sec := Box(std.side.origin, x0: s0, y0: r0, x1: s1, y1: r1)
  }
  body := solid(sec.profile, about: ax)
}

// A circle of `n` holes of diameter `d` on radius `r` about the end view's centre, the first at
// `phase` from the datum: drawn once and turned (`ring`), so a pattern is one hole solved for.
component HoleCircle(f: Datums, n: Int, r: Length, d: Length, phase: Angle) {
  in f.o.plane {
    ring n about f.o {
      c := point
      f.o distance(r) c
      hole := circle(center: c) hint(r: d / 2)
      radius(d / 2) hole
      private construction radial := line(f.o, c)
    }
  }
  f.datum angle(phase) radial[0]
}

// Open this file to preview the plain rings on the right of the axle: the bearing on its
// shoulder, the spacer from its inner race to the step, and the flanged locknut.
preview {
  unit mm
  brg_out := motor.dims.brg_in + motor.dims.brg_w
  bearing := Revolved(s0: motor.dims.brg_in, s1: brg_out, r0: motor.dims.brg_id / 2,
                      r1: motor.dims.brg_od / 2)
  spacer := Revolved(s0: brg_out, s1: motor.dims.step_end, r0: motor.dims.d_seat / 2,
                     r1: motor.dims.spacer_r)
  nut := Revolved(s0: motor.dims.step_end, s1: motor.dims.old / 2,
                  r0: motor.dims.d_end / 2, r1: motor.dims.nut_r)
}
