// An obround slot with a hole at each end — the shape a link between two pins actually is.
//
// The two flanks are straight and the two ends are semicircular, and what says so is four
// tangencies: each end arc meets the top and the bottom run at its own endpoints.  The holes are
// concentric with the ends because they are drawn on the *same centre points*, which is a shared
// point rather than a constraint — the cheapest way to say concentric there is.
//
// One length, one end radius (shared by the other end), one hole radius each, one levelled run
// and one grounded centre: fully constrained.

param length = 80
param r = 15
param hole_r = 6

point c1 at (0, 0)
point c2 at (length, 0)

point t1 at (0, r)
point t2 at (length, r)
line  top(t1, t2)

point b1 at (length, 0 - r)
point b2 at (0, 0 - r)
line  bottom(b1, b2)

arc a_right(center: c2, start: b1, end: t2, r: r)
arc a_left(center: c1, start: t1, end: b2, r: r)

circle h1(center: c1, r: hole_r)
circle h2(center: c2, r: hole_r)

tangent_arc_line(a_right, bottom, at: start)
tangent_arc_line(a_right, top,    at: end)
tangent_arc_line(a_left,  top,    at: start)
tangent_arc_line(a_left,  bottom, at: end)

equal_radius(a_left, a_right)
radius(a_left) == r
radius(h1) == hole_r
radius(h2) == hole_r

distance(c1, c2) == length
horizontal(top)

ground(c1)
