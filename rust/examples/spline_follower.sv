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

point k0 hint at (0, 26)
point k1 hint at (20, 0)
point k2 hint at (40, 26)
point k3 hint at (60, 0)
point k4 hint at (80, 26)
point k5 hint at (100, 0)
point k6 hint at (120, 26)

spline curve(k0, k1, k2, k3, k4, k5, k6)

// the follower: a level face resting against the curve, touching wherever it must
point f1 hint at (0, 8.666667)
point f2 hint at (120, 8.666667)
line  face(f1, f2)
horizontal(face)
spline_tangent_line(curve, face)

// and a point riding on the curve, held off a grounded anchor above it
point rider hint at (60, 8.666667)
point anchor hint at (60, 68.666667)
point_on_spline(rider, curve)
distance(anchor, rider) == 60

ground(k0)
ground(anchor)
