// A freeform curve with a straight edge resting against it, and a bead riding along it.
//
// The curve is a B-spline: a smooth line steered by the run of control points beside it.  Drag
// one and the curve bends towards it, and the two things touching the curve have to cope.
//
// Touching a *curve* is not like touching a line.  There is no formula for which part of a curve
// is being touched, so each contact carries its own extra unknown recording how far along it
// sits.  That is what lets the contacts **slide**: move a control point and the straight edge
// stays tangent by finding a new place to touch, rather than dragging the curve around with it.
//
// It will slide a long way, too — including past the joins between the curve's spans, which
// changes which control points the contact actually depends on.  The drawing does not notice,
// which is the point of the case.
//
// The bead is held a fixed distance from the anchor above the curve, so it has somewhere to be
// and the curve cannot simply shrug it off.  `8.666667` is where the straight edge starts
// looking for its contact — a starting guess, not a statement.

point k0 hint(x: 0, y: 26)
point k1 hint(x: 20, y: 0)
point k2 hint(x: 40, y: 26)
point k3 hint(x: 60, y: 0)
point k4 hint(x: 80, y: 26)
point k5 hint(x: 100, y: 0)
point k6 hint(x: 120, y: 26)

// `cam` rather than `curve`: a statement now begins with a *name* and a name that is also an
// element keyword could not lead one.
spline cam(k0, k1, k2, k3, k4, k5, k6)

// the follower: a level face resting against the curve, touching wherever it must
point f1 hint(x: 0, y: 8.666667)
point f2 hint(x: 120, y: 8.666667)
line  face(f1, f2)
horizontal face
cam tangent face

// and a point riding on the curve, held off a grounded anchor above it
point rider hint(x: 60, y: 8.666667)
point anchor hint(x: 60, y: 68.666667)
rider on cam
anchor distance(60) rider

ground k0
ground anchor
