// How to draw a straight line.  Every rod in the Peaucellier–Lipkin cell of 1864 swings about a
// pivot, so nothing here is straight, yet `pen` traces an exact line: the kite inverts `b` about
// `o`, and the inversion of a circle through the centre of inversion is a line.  The document
// states none of that — the last statement *claims* it, and the diagnosis proves it.

use std

// the crank's angle, a number that runs over the stretch the pen is traced on
param t: Angle over (60deg, 115deg)

// the three lengths the machine is built from; the pen draws the line x = (arm² − side²) / (2·crank)
param arm := 100       // the long arms, o–c and o–d
param side := 60       // the four sides of the kite, b–c–pen–d
param crank := 40      // the crank q–b, and the orbit its pin rides

// The cell, once: a linkage posed at crank angle u, eight numbers against eight statements.
// The ccw/cw lines pick the elbows — a seed repeating a predicate is the weaker of two
// statements of one fact — and the seeds only start the solve near them.  Drawn below at `t`, so
// the drawing stands at `t = 60°` and is followed from there as `t` runs.
component Cell(orbit: circle, datum: line, arm: Length, side: Length, u: Angle) {
  b := point   hint((50.4, 38.6))
  c := point   hint((30.4, 95.2))
  d := point   hint((99.9, 4.8))
  pen := point hint((80.0, 61.4))
  swing := line(datum.p2, b)
  b coincident orbit                         // the crank pin on its circle...
  datum angle(u) swing               // ...posed at bearing u; directed, so this side up
  oc := line(datum.p1, c)
  od := line(datum.p1, d)
  datum.p1 distance(arm) c           // the two long arms,
  datum.p1 distance(arm) d
  (bc := line(b, c)) -> (cp := line(c, pen)) -> (pd := line(pen, d)) -> (db := line(d, b)) -> close
  b distance(side) c                 // and the kite closed round b and pen
  b distance(side) d
  pen distance(side) c
  pen distance(side) d
  ccw(datum.p1, b, c)                // c left of the arm, d right,
  cw(datum.p1, b, d)
  ccw(c, d, pen)                     // and pen on the far side of the kite from b
}

// The fixed frame: the datum the crank angle is read from, and the orbit.  `o coincident orbit` is
// the theorem's whole hypothesis — the pin's circle passes through the centre of inversion — and
// it places `q` too, so no dimension between the pivots is ever stated.
in std.front {
  o := point
  q := point hint((crank, 0))
  datum := line(o, q)
  orbit := circle(center: q) hint(r: crank)

  horizontal datum
  radius(crank) orbit
  o coincident orbit
  fix((0, 0)) o

  // the machine itself, its crank at `t`
  cell := Cell(orbit, datum, arm: arm, side: side, u: t)
}

// Where the pen goes: the drawn cell's own `pen`, as `t` runs.  The drawing's pose is where the
// trace is anchored, so it needs no seeds of its own.  `rail` runs through two
// pinned points of the path, and the claim says it is vertical — which nothing above states.
// The diagnosis reports it a theorem; delete it and the drawing is unchanged, which is what a
// claim promises.
path := trace(cell.pen over t)

in std.front {
  g1 := point hint((80, 51))
  g2 := point hint((80, 114))
  g1 coincident(t == 65) path
  g2 coincident(t == 110) path
  rail := line(g1, g2)

  claim vertical rail
}

// Diagnosed: DOF 0 and `1 DOF over t` — the crank — and the claim a theorem.  Followed from 60° to
// 115°, the cell folds and stretches to carry the pen along the line it cannot leave.
