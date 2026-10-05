// Two shafts that do not meet: the gear's axis drawn in the front plane, the pinion's in a second
// plane standing on an axis the solve turns, and the two related in space — a shaft angle and an
// offset, the two numbers a crossed-axis gear pair (a hypoid, a worm, a crossed helical) is
// specified by.
//
// The side plane is the page's y and an axis `t` held square to it, so it can turn only about y;
// where `t` points is not stated, so the solve answers for it, and its `hint` is only where it
// starts.  The pinion's axis is drawn level in its plane, from its origin and 60 long, which leaves
// it one freedom there: its height.  The two statements across the planes settle the two unknowns
// — `angle` between lines of different planes is the unsigned angle between their directions in
// space, and `distance` their common perpendicular.  DOF 0.
//
// Open the glass box (⌘B): the front plane stands upright with the gear's shaft in it, and at a
// shaft angle of 90° the side plane comes out level — the top plane — with the pinion's shaft
// lying in it, `offset` behind the gear's.  Set `shaft_angle` to 60deg and the side plane tilts to
// 30°; change `offset` and the pinion's shaft moves along the common perpendicular.  Delete the
// `distance` line and the report says DOF 1: the pinion may slide.

unit mm
use std
shaft_angle := 90deg
offset := 17.5mm

in std.front {
  gax := line
  fix(x == 0, y == 0) gax.p1
  fix(x == 0, y == 50) gax.p2
}

t := axis hint(x: 0.87, y: 0, z: 0.5)
t perpendicular std.y
side := plane(u: t, v: std.y)
side.origin coincident std.front     // on std.y already, as its axes pass through it
in side {
  pax := line(hint(x: 0, y: 10), hint(x: 60, y: 12))
  pax.p1 distance(0, along: u) side
  pax.p2 distance(60, along: u) side
  pax.p1 horizontal pax.p2
}

gax angle(shaft_angle) pax      // the shaft angle, in space
gax distance(offset) pax        // the offset: their common perpendicular, in space

// the shafts themselves, for the box: a cylinder about each axis, drawn as two rings and four rulings
gear_shaft := cylinder(axis: gax) hint(r: 6)
pinion_shaft := cylinder(axis: pax) hint(r: 4)
radius(6) gear_shaft
radius(4) pinion_shaft
