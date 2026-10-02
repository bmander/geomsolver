// Two shafts that do not meet: the gear's axis drawn in the front view, the pinion's in a second
// view folded from it, and the two related in space — a shaft angle and an offset, the two numbers
// a crossed-axis gear pair (a hypoid, a worm, a crossed helical) is specified by.
//
// The side view's fold is not stated.  `fold: beta` names nothing the document defines, so `beta`
// is a free variable (the CLI says W111) and the solve answers for it; the `hint` is only where it
// starts.  The pinion's axis is drawn level in its view, from its datum and 60 long, which leaves
// it one freedom there: its height.  The two statements across the views settle the two unknowns —
// `angle` between lines of different views is the unsigned angle between their directions in
// space, and `distance` their common perpendicular.  29 unknowns, 29 equations, DOF 0.
//
// Open the glass box (⌘B): the front view stands upright with the gear's shaft in it, the side view
// has folded flat under it — at a shaft angle of 90° the fold comes out at 0°, the top view — and
// the pinion's shaft lies in it, `offset` behind the gear's.  Set `shaft_angle` to 60deg and the
// side view tilts to 30°; change `offset` and the pinion's shaft moves along the common
// perpendicular.  Delete the `distance` line and the report says DOF 1: the pinion may slide.
// The datums' places on the sheet are held without a `ground`, since where a solved view's picture
// sits is presentation, not geometry.

unit mm
shaft_angle := 90deg
offset := 17.5mm

o := point hint(x: 0, y: 0)
t := point hint(x: 40, y: 0)
front := plane(origin: o, toward: t)
gax := line in front
fix(x == 0, y == 0) gax.p1
fix(x == 0, y == 50) gax.p2

o2 := point hint(x: 120, y: 0)
t2 := point hint(x: 160, y: 0)
side := plane(origin: o2, toward: t2, from: front, fold: beta) hint(fold: 30deg)
pax := line(hint(x: 120, y: 10), hint(x: 180, y: 12)) in side
pax.p1 distance(0, along: u) side
pax.p2 distance(60, along: u) side
pax.p1 horizontal pax.p2

gax angle(shaft_angle) pax      // the shaft angle, in space
gax distance(offset) pax        // the offset: their common perpendicular, in space

// the shafts themselves, for the box: a cylinder about each axis, drawn as two rings and four rulings
gear_shaft := cylinder(axis: gax) hint(r: 6)
pinion_shaft := cylinder(axis: pax) hint(r: 4)
radius(6) gear_shaft
radius(4) pinion_shaft
