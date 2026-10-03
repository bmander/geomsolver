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
// length in the unit a person has in hand: `c1 distance(3 1/8") c2` is 79.375 here.
unit mm

length := 80
r := 15
hole_r := 6

c1 := point
c2 := point hint(x: length, y: 0)

t1 := point hint(x: 0, y: r)
t2 := point hint(x: length, y: r)
b1 := point hint(x: length, y: -r)
b2 := point hint(x: 0, y: -r)

// round the outline counter-clockwise, the way an arc runs: each end leaves one flank and meets
// the other at a tangent joint
(bottom := line(b2, b1)) -> tangent
(a_right := arc(center: c2) hint(r: r)) -> tangent
horizontal (top := line(t2, t1)) -> tangent
(a_left := arc(center: c1) hint(r: r)) -> tangent close

h1 := radius(hole_r) circle(center: c1) hint(r: hole_r)
h2 := radius(hole_r) circle(center: c2) hint(r: hole_r)

a_left equal a_right
radius(r) a_left

c1 distance(length) c2

fix(x == 0, y == 0) c1
