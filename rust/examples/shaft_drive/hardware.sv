// Bought-in hardware: deep-groove ball bearings and retaining rings, each drawn about its own
// axis, its frame's `std.z`, its mid-plane its frame's `std.top`, and given only numbers, so every
// bearing or ring of one size is one figure placed wherever it is seated.
unit mm
use std
use std.sets
use std.solids
use std.solids (cut, union)
use std.place (horizontal, vertical, left_of, right_of)

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

// A bearing's size, as the table gives it: bore `d`, outside `D`, width `B`, `balls` balls `dw`
// across.
interface BearingSize { d: Length, D: Length, B: Length, balls: Int, dw: Length }

// A deep-groove ball bearing (ISO 15: 6804, 6901, …) of size `s`, its balls on the pitch circle
// midway between bore and outside.  Each ring is its
// section turned about the axis, the ball's track grooved into it a little wider than the ball;
// the cage and seals are not drawn.
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
  // one ball on the pitch circle, a half disc turned about its diameter (README: a ball written as
  // `{ p | p distance(<= r) c }` meshes to nothing where some bearings are seated), then the rest
  // turned round the axis a ball pitch apart
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
}

// A retaining ring (DIN 471 on a shaft, DIN 472 in a bore): a flat ring from `r_in` to `r_out`,
// `t` thick, open `gap` across its ends toward `−std.x`, with an eye either side of the gap on the
// circle `r_eye`, each `eye` across with a hole `hole` across for the pliers.  On a shaft the eyes
// stand outside the ring (`r_eye` > `r_out`), in a bore inside it.
component RetainingRing(r_in: Length, r_out: Length, t: Length, gap: Angle, r_eye: Length,
                        eye: Length, hole: Length) {
  // the eyes stand clear of the ring's ends, turned this much past them
  eye_turn := asin(eye / 2 / r_eye)
  in std.top {
    o := point
    o coincident std.top.origin
    west := point
    west left_of(d: 10mm) o
    construction back := line(o, west)
    // the ring's ends, `gap / 2` either side of the back
    a_in := point
    a_out := point
    b_in := point
    b_out := point
    construction end_a := line(o, a_out)
    construction end_b := line(o, b_out)
    a_in coincident end_a
    b_in coincident end_b
    back angle(-gap / 2) end_a
    back angle(gap / 2) end_b
    outer := arc(center: o, start: b_out, end: a_out)
    inner := arc(center: o, start: b_in, end: a_in)
    radius(r_out) outer
    radius(r_in) inner
    o distance(>= 0mm, along: end_a) a_in
    o distance(>= 0mm, along: end_b) b_in
    // the eyes, beside the ends
    e_a := point
    e_b := point
    construction to_ea := line(o, e_a)
    construction to_eb := line(o, e_b)
    o distance(r_eye) e_a
    o distance(r_eye) e_b
    back angle(-gap / 2 - eye_turn) to_ea
    back angle(gap / 2 + eye_turn) to_eb
    eye_a := circle(center: e_a)
    eye_b := circle(center: e_b)
    radius(eye / 2) eye_a
    radius(eye / 2) eye_b
    side_a := line(a_out, a_in)
    side_b := line(b_in, b_out)
    hole_a := circle(center: e_a)
    hole_b := circle(center: e_b)
    radius(hole / 2) hole_a
    radius(hole / 2) hole_b
  }
  body := solid(std.solids.Prism(face(outer, side_a, inner, side_b), from: -t / 2, to: t / 2))
  construction eyes_a := solid(std.solids.Prism(face(eye_a, holes: hole_a), from: -t / 2, to: t / 2))
  construction eyes_b := solid(std.solids.Prism(face(eye_b, holes: hole_b), from: -t / 2, to: t / 2))
  eyes_a union body
  eyes_b union body
}

// A ring's size, as the table gives it: its groove's diameter, its thickness `t` and radial
// `band`, and its eyes' circle `r_eye`, size and hole.
interface RingSize { groove: Length, t: Length, band: Length, r_eye: Length, eye: Length, hole: Length }

// DIN 471, on a shaft: the ring from the groove outward, its eyes outside it.
component Ring471(s: RingSize) {
  ring := RetainingRing(r_in: s.groove / 2, r_out: s.groove / 2 + s.band, t: s.t, gap: 40deg,
    r_eye: s.r_eye, eye: s.eye, hole: s.hole)
}

// DIN 472, in a bore: the ring from the groove inward, its eyes inside it.
component Ring472(s: RingSize) {
  ring := RetainingRing(r_in: s.groove / 2 - s.band, r_out: s.groove / 2, t: s.t, gap: 40deg,
    r_eye: s.r_eye, eye: s.eye, hole: s.hole)
}

preview {
  b := Bearing({d: 20mm, D: 32mm, B: 7mm, balls: 14, dw: 3.5mm})
  clip := Ring471({groove: 19mm, t: 1.2mm, band: 2.6mm, r_eye: 13.4mm, eye: 3.4mm, hole: 1.5mm})
}
