// A cubic B-spline with a straight follower held against it, and a point riding along it.
//
// This is the case for the two things a curve has that no implicit primitive does.  The tangency
// **owns the parameter it touches at**, so dragging a control point slides the contact along the
// curve instead of breaking it — and it slides *past knots*, which changes which control points
// the constraint's columns name and quietly recompiles the plan underneath.  The rider is held at
// a distance from a grounded anchor above the curve, so it has somewhere to be and the curve
// cannot simply shrug it off.
//
// Both halves start where they already belong, so opening the case shows the curve as it is drawn
// rather than the nearest configuration to it.  The follower sits where the curve already dips
// lowest — an interior dip rather than an end, so the contact has curve on both sides of it to
// slide along — and `8.666667` is where that dip is, which is a *seed* and not a statement.

point k0 at (0, 26)
point k1 at (20, 0)
point k2 at (40, 26)
point k3 at (60, 0)
point k4 at (80, 26)
point k5 at (100, 0)
point k6 at (120, 26)

spline curve(k0, k1, k2, k3, k4, k5, k6)

// the follower: a level face resting against the curve, touching wherever it must
point f1 at (0, 8.666667)
point f2 at (120, 8.666667)
line  face(f1, f2)
horizontal(face)
spline_tangent_line(curve, face)

// and a point riding on the curve, held off a grounded anchor above it
point rider at (60, 8.666667)
point anchor at (60, 68.666667)
point_on_spline(rider, curve)
distance(anchor, rider) == 60

ground(k0)
ground(anchor)
