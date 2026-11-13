// Ports (docs/ports-plan.md, [0.217]): the groups a part declares to say how it is joined, the
// interfaces that type them and the words that join them.  A port's frame is a plane — its origin
// on the part's axis, its normal along the axis — and a part that hands out a datum of its own in
// a port stands in a frame of its own, which a mate through the port places.
unit mm
use std
use std.solids (cut)

// Where a part is seated: the frame the seated part's own frame takes, and the body the seat is
// made in.
interface Seat { at: plane, host: solid }
// What is seated: the part's own frame, and the room it takes in the seat's body.
interface Seated { at: plane, pocket: solid }
// A flat face square to a part's axis: its frame's origin on the axis, its normal out of the part.
interface Flat { at: plane }
// A ring's seat: the groove it takes in what it is fitted to.
interface RingSeat { groove: solid }
// What a ring is fitted to: the body its groove is cut in.
interface Grooved { host: solid }

// A part seated: its frame the seat's, and the room it takes cut from the seat's body.
s: Seated seats_in h: Seat := {
  s.at.origin coincident h.at.origin
  s.at.normal parallel(sense: same) h.at.normal
  h.at.u angle(0deg, about: h.at.normal) s.at.u
  s.pocket cut h.host
}

// Two faces against each other: one point of the axis, the normals facing each other.
a: Flat abuts b: Flat := {
  a.at.origin coincident b.at.origin
  a.at.normal parallel(sense: opposite) b.at.normal
  b.at.u angle(0deg, about: b.at.normal) a.at.u
}

// A ring's groove cut in what it is fitted to.
r: RingSeat grooves j: Grooved := {
  r.groove cut j.host
}

// A frame on the axis `ax`, `d` along it from `from`, its normal along `toward` and its x `east`
// turned about it.
component FrameOn(ax: axis, from: point, d: Length, toward: axis, east: axis) {
  at := plane
  at.origin coincident ax
  from distance(d, along: ax) at.origin
  at.normal parallel(sense: same) toward
  at.v perpendicular at.u
  east angle(0deg, about: toward) at.u
}

// A seat in `host` on the axis `ax`, `d` along it from `from`, the seated part's axis running
// along `toward`: the port `seat`.
component Station(ax: axis, from: point, d: Length, toward: axis, east: axis, host: solid) {
  f := FrameOn(ax, from, d: d, toward: toward, east: east)
  seat := {at: f.at, host: host}
}
