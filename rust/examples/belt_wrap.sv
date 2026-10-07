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

use std (horizontal)

r := 10        // the small pulley
R := 25        // the big one
wrap := 90     // belt in contact with the big pulley

in std.front {
  c1 := point
  c2 := point hint((66, 0))

  // each run's ends are seeded for the side it passes on; the chain threads them into the arcs
  (bottom := line(hint((-2, -10)), hint((60, -24)))) -> tangent
  radius(R) (big := arc(center: c2) hint(r: R)) -> tangent
  (top := line(hint((60, 24)), hint((-2, 10)))) -> tangent
  radius(r) (small := arc(center: c1) hint(r: r)) -> tangent close

  length(wrap) big

  c1 horizontal c2
  fix((0, 0)) c1
}
