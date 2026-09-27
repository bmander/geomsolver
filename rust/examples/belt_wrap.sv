// An open belt over two pulleys, placed by how far it wraps.
//
// The belt is one closed chain: a straight run along the bottom, round the big pulley, a straight
// run back along the top, round the small one, and closed.  Every joint is `-> tangent`, so each
// run leaves one pulley and meets the next without a crease, and the arcs name only their
// centres: where each wrap starts and ends is where the runs either side of it touch.
//
// Nothing says how far apart the pulleys are.  What is stated instead is the length of belt in
// contact with the big pulley — `length(wrap) big`, an arc's length along itself, its radius
// times its sweep.  An open belt wraps the larger pulley by more than half a turn, and the more
// so the closer the two are, so that one number places the second pulley: move it and the
// centre distance follows.  The wrap on the small pulley is whatever is left over.
//
// `length` is measured the way the arc runs, counter-clockwise from its start to its end, which
// is why the chain goes round counter-clockwise: the big wrap runs from the bottom run to the
// top one round the far side.

param r = 10        // the small pulley
param R = 25        // the big one
param wrap = 90     // belt in contact with the big pulley

point c1 hint(x: 0, y: 0)
point c2 hint(x: 66, y: 0)

// where the runs touch the pulleys: seeds only, for the side each run passes on
point sb hint(x: -2, y: -10)
point bb hint(x: 60, y: -24)
point bt hint(x: 60, y: 24)
point st hint(x: -2, y: 10)

line bottom(sb, bb) -> tangent
arc big(center: c2) hint(r: R) -> tangent
line top(bt, st) -> tangent
arc small(center: c1) hint(r: r) -> tangent close

radius(r) small
radius(R) big
length(wrap) big

c1 horizontal c2
ground c1
