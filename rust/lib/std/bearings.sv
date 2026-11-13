// Deep-groove ball bearings (ISO 15), drawn about their own axis, `std.z`, their mid-plane their
// frame's `std.top`, each with its ports: `od` (seated in a housing, the room it takes there),
// and its two sides `left` (−z) and `right` (+z), each a face whose normal runs out of the bearing.
unit mm
use std
use std.sets
use std.solids
use std.solids (cut, union)
use std.place (vertical, right_of)
use std.ports
use std.ports (seats_in, abuts, grooves)
use std.retaining

// A bearing's size, as the table gives it: bore `d`, outside `D`, width `B`, `balls` balls `dw`
// across.
interface BearingSize { d: Length, D: Length, B: Length, balls: Int, dw: Length }

// ISO 15, the thin series a bicycle's drive takes (6800, 6900), as far as it is used.
b6901 := {d: 12mm, D: 24mm, B: 6mm, balls: 10, dw: 3.97mm}
b6903 := {d: 17mm, D: 30mm, B: 7mm, balls: 12, dw: 3.97mm}
b6804 := {d: 20mm, D: 32mm, B: 7mm, balls: 14, dw: 3.5mm}
b6805 := {d: 25mm, D: 37mm, B: 7mm, balls: 16, dw: 3.5mm}

// A rectangle in the view `o` is drawn in, by its corners' ordinates from `o`: a turned part's
// section, a radius out (`x0` to `x1`) and a station along the axis (`y0` to `y1`).
component Section(o: point, x0: Length, y0: Length, x1: Length, y1: Length) {
  a := point
  b := point
  c := point
  d := point
  profile := (ab := line(a, b)) -> (bc := line(b, c)) -> (cd := line(c, d)) ->
             (da := line(d, a)) -> close
  o distance(x0, along: u) a
  o distance(y0, along: v) a
  o distance(x1, along: u) b
  o distance(y0, along: v) b
  o distance(x1, along: u) c
  o distance(y1, along: v) c
  o distance(x0, along: u) d
  o distance(y1, along: v) d
}

// A deep-groove ball bearing of size `s`, its balls on the pitch circle midway between bore and
// outside.  Each ring is its section turned about the axis, the ball's track grooved into it a
// little wider than the ball; the cage and seals are not drawn.
component Bearing(s: BearingSize) {
  d := s.d
  D := s.D
  B := s.B
  balls := s.balls
  dw := s.dw
  dm := (d + D) / 2
  shoulder := 0.35 * dw                // each ring's shoulder, either side of the pitch circle
  in std.front {
    up := point
    construction axis_line := line(std.origin, up)
    vertical axis_line
    length(10mm) axis_line
    std.origin distance(>= 0mm, along: v) up
    inner_sec := Section(std.origin, x0: d / 2, y0: -B / 2, x1: dm / 2 - shoulder, y1: B / 2)
    outer_sec := Section(std.origin, x0: dm / 2 + shoulder, y0: -B / 2, x1: D / 2, y1: B / 2)
    track_c := point
    track_c right_of(d: dm / 2) std.origin
    track := circle(center: track_c)
    radius(0.52 * dw) track
  }
  inner_ring := solid(std.solids.Revolution(inner_sec.profile, about: axis_line))
  outer_ring := solid(std.solids.Revolution(outer_sec.profile, about: axis_line))
  construction groove := solid(std.solids.Revolution(face(track), about: axis_line))
  groove cut inner_ring
  groove cut outer_ring
  // one ball on the pitch circle, a half disc turned about its diameter, then the rest turned
  // round the axis a ball pitch apart
  in std.front {
    c := point
    c right_of(d: dm / 2) std.origin
    n := point
    s_ := point
    c distance(dw / 2, along: v) n
    c distance(0mm, along: u) n
    c distance(0mm, along: u) s_
    c distance(<= 0mm, along: v) s_
    rim := arc(center: c, start: s_, end: n)
    construction across := line(n, s_)
  }
  construction ball := solid(std.solids.Revolution(face(rim, across), about: across))
  ball_set := solid(ball)
  repeat balls - 1 as i {
    construction placed := ball at(under: axis_line) (i + 1) * 360deg / balls
    placed union ball_set
  }
  // the ports: the room it takes in a housing, its outside's cylinder a millimetre past either
  // side, seated by its mid-plane; and its sides
  construction pocket := solid({ q | q in std.sets.Cylinder(axis_line, r: D / 2)
                                     std.origin distance(in: (-B / 2 - 1mm, B / 2 + 1mm),
                                                         along: axis_line) q })
  od := {at: std.top, pocket: pocket}
  down := axis
  down parallel(sense: opposite) std.z
  std.origin coincident down
  lf := std.ports.FrameOn(std.z, std.origin, d: -B / 2, toward: down, east: std.x)
  rf := std.ports.FrameOn(std.z, std.origin, d: B / 2, toward: std.z, east: std.x)
  left := {at: lf.at}
  right := {at: rf.at}
}

// A bearing held in its seat by a ring: seated by its outside in `seat`, the seat's axis running
// the way the bearing went in, and a ring abutting its trailing side, grooved into `on` — DIN 471
// on a shaft (`OnShaft`), DIN 472 in a bore (`InBore`).  A joint of joints (docs/ports-plan.md,
// P6): one statement where the joint was six.
component OnShaft(size: BearingSize, ring: std.retaining.RingSize, seat: std.ports.Seat,
                  on: std.ports.Grooved) {
  b := Bearing(size)
  b.od seats_in seat
  clip := std.retaining.Ring471(ring)
  clip.face abuts b.left
  clip.seat grooves on
  // what a parent reads of the bearing it holds
  bearing := {inner_ring: b.inner_ring, outer_ring: b.outer_ring, ball_set: b.ball_set}
}
component InBore(size: BearingSize, ring: std.retaining.RingSize, seat: std.ports.Seat,
                 on: std.ports.Grooved) {
  b := Bearing(size)
  b.od seats_in seat
  clip := std.retaining.Ring472(ring)
  clip.face abuts b.left
  clip.seat grooves on
  // what a parent reads of the bearing it holds
  bearing := {inner_ring: b.inner_ring, outer_ring: b.outer_ring, ball_set: b.ball_set}
}
