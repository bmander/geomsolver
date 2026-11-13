// Retaining rings (DIN 471 on a shaft, DIN 472 in a bore), drawn about their own axis, `std.z`,
// their mid-plane their frame's `std.top`, each with its ports: `face` (−z, its normal out of the
// ring), which abuts what it holds, and `seat`, the groove it takes in what it is fitted to.
unit mm
use std
use std.sets
use std.solids
use std.solids (union)
use std.place (left_of)
use std.ports

// A ring's size, as the table gives it: its groove's diameter, its thickness `t` and radial
// `band`, and its eyes' circle `r_eye`, size and hole.
interface RingSize { groove: Length, t: Length, band: Length, r_eye: Length, eye: Length, hole: Length }

// DIN 471, for shafts of 17, 20 and 25 mm; DIN 472, for a 24 mm bore.
din471_17 := {groove: 16.2mm, t: 1mm, band: 2.3mm, r_eye: 11.6mm, eye: 3mm, hole: 1.2mm}
din471_20 := {groove: 19mm, t: 1.2mm, band: 2.6mm, r_eye: 13.4mm, eye: 3.4mm, hole: 1.5mm}
din471_25 := {groove: 23.9mm, t: 1.2mm, band: 3mm, r_eye: 16.4mm, eye: 3.6mm, hole: 1.5mm}
din472_24 := {groove: 25.2mm, t: 1.2mm, band: 2.6mm, r_eye: 9.6mm, eye: 3.4mm, hole: 1.5mm}

// A retaining ring: a flat ring from `r_in` to `r_out`, `t` thick, open `gap` across its ends
// toward −x, with an eye either side of the gap on the circle `r_eye`, each `eye` across with a
// hole `hole` across for the pliers.
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

// The ring's ports, about the axis `ax` it is drawn about: its groove, a band `t` wide from `r_in`
// out to `r_out`; its face, `t / 2` down the axis.
component RingPorts(t: Length, r_in: Length, r_out: Length) {
  in std.front {
    up := point
    construction ax := line(std.origin, up)
    std.origin distance(0mm, along: u) up
    std.origin distance(10mm, along: v) up
  }
  construction groove := solid({ q | q in std.sets.Cylinder(ax, r: r_out)
                                     q not in std.sets.Cylinder(ax, r: r_in)
                                     std.origin distance(in: (-t / 2, t / 2), along: ax) q })
  down := axis
  down parallel(sense: opposite) std.z
  std.origin coincident down
  ff := std.ports.FrameOn(std.z, std.origin, d: -t / 2, toward: down, east: std.x)
  face := {at: ff.at}
}

// DIN 471, on a shaft: the ring from the groove outward, its groove cut from the groove's
// diameter out past the shaft.
component Ring471(s: RingSize) {
  ring := RetainingRing(r_in: s.groove / 2, r_out: s.groove / 2 + s.band, t: s.t, gap: 40deg,
    r_eye: s.r_eye, eye: s.eye, hole: s.hole)
  p := RingPorts(t: s.t, r_in: s.groove / 2, r_out: s.groove / 2 + s.band)
  seat := {at: std.top, groove: p.groove}
  face := {at: p.face.at}
}

// DIN 472, in a bore: the ring from the groove inward, its groove cut from inside the bore out to
// the groove's diameter.
component Ring472(s: RingSize) {
  ring := RetainingRing(r_in: s.groove / 2 - s.band, r_out: s.groove / 2, t: s.t, gap: 40deg,
    r_eye: s.r_eye, eye: s.eye, hole: s.hole)
  p := RingPorts(t: s.t, r_in: s.groove / 2 - s.band, r_out: s.groove / 2)
  seat := {at: std.top, groove: p.groove}
  face := {at: p.face.at}
}
