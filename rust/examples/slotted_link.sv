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

length := 80
r := 15
hole_r := 6

c1 := point hint(x: 0, y: 0)
c2 := point hint(x: length, y: 0)

t1 := point hint(x: 0, y: r)
t2 := point hint(x: length, y: r)
top := line(t1, t2)

b1 := point hint(x: length, y: 0 - r)
b2 := point hint(x: 0, y: 0 - r)
bottom := line(b1, b2)

a_right := arc(center: c2, start: b1, end: t2) hint(r: r)
a_left := arc(center: c1, start: t1, end: b2) hint(r: r)

h1 := circle(center: c1) hint(r: hole_r)
h2 := circle(center: c2) hint(r: hole_r)

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
