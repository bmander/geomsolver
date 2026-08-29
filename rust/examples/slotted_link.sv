// The slot-and-two-holes shape of a link joining a pair of pins.
//
// The two long flanks are straight and the two ends are half-circles, and what says so is four
// tangencies: each end arc meets the top run and the bottom run smoothly, with no crease where
// they join.  Nothing states where the arcs go — being tangent to both runs at their own ends is
// enough to put them there.
//
// The holes sit exactly on the ends' centres because they are drawn *on the same points*.
// Sharing a point costs nothing at all, where saying "concentric" would be one more equation to
// solve; when two things really are in the same place, naming one point for both is the cheaper
// and truer way to say it.
//
// One overall length, one end radius shared by both ends, a radius for each hole, one levelled
// run and one pinned centre — and the shape is completely determined.

// The document's unit.  Without this line the drawing is in *drawing units* — a length with no
// name — and everything still dimension-checks; what the line buys is the right to write a
// length in the unit a person has in hand: `c1 distance(3 1/8"` is 79.375 here.) c2
unit mm

param length = 80
param r = 15
param hole_r = 6

point c1 hint(x: 0, y: 0)
point c2 hint(x: length, y: 0)

point t1 hint(x: 0, y: r)
point t2 hint(x: length, y: r)
line  top(t1, t2)

point b1 hint(x: length, y: 0 - r)
point b2 hint(x: 0, y: 0 - r)
line  bottom(b1, b2)

arc a_right(center: c2, start: b1, end: t2) hint(r: r)
arc a_left(center: c1, start: t1, end: b2) hint(r: r)

circle h1(center: c1) hint(r: hole_r)
circle h2(center: c2) hint(r: hole_r)

a_right tangent(at: start) bottom
a_right tangent(at: end) top
a_left tangent(at: start) top
a_left tangent(at: end) bottom

a_left equal a_right
radius(r) a_left
radius(hole_r) h1
radius(hole_r) h2

c1 distance(length) c2
horizontal top

ground c1
