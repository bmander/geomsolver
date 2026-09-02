// How to draw a straight line.  Every rod in the Peaucellier–Lipkin cell of 1864 swings about a
// pivot, so nothing here is straight, yet `pen` traces an exact line: the kite inverts `b` about
// `o`, and the inversion of a circle through the centre of inversion is a line.  The document
// states none of that — the last statement *claims* it, and the diagnosis proves it.

// the three lengths the machine is built from; the pen draws the line x = (arm² − side²) / (2·crank)
param arm = 100       // the long arms, o–c and o–d
param side = 60       // the four sides of the kite, b–c–pen–d
param crank = 40      // the crank q–b, and the orbit its pin rides

// The cell, once: a linkage posed at crank angle u, eight numbers against eight statements.
// The ccw/cw lines pick the elbows — a seed repeating a predicate is the weaker of two
// statements of one fact — and the seeds only start the solve near them.  Drawn below with `u`
// left unbound, so the crank is the drawing's one freedom; traced below that, where the same
// component is asked where `pen` goes as `u` runs.
component Cell(orbit: circle, datum: line, arm: Length, side: Length, u: Angle) {
  point b   hint(x: 50.4, y: 38.6)
  point c   hint(x: 30.4, y: 95.2)
  point d   hint(x: 99.9, y: 4.8)
  point pen hint(x: 80.0, y: 61.4)
  line swing(datum.p2, b)
  b on orbit                         // the crank pin on its circle...
  datum angle(u) swing               // ...posed at bearing u; directed, so this side up
  line oc(datum.p1, c)
  line od(datum.p1, d)
  datum.p1 distance(arm) c           // the two long arms,
  datum.p1 distance(arm) d
  line bc(b, c) -> line cp(c, pen) -> line pd(pen, d) -> line db(d, b) -> close
  b distance(side) c                 // and the kite closed round b and pen
  b distance(side) d
  pen distance(side) c
  pen distance(side) d
  ccw(datum.p1, b, c)                // c left of the arm, d right,
  cw(datum.p1, b, d)
  ccw(c, d, pen)                     // and pen on the far side of the kite from b
}

// The fixed frame: the datum the crank angle is read from, and the orbit.  `o on orbit` is the
// theorem's whole hypothesis — the pin's circle passes through the centre of inversion — and it
// places `q` too, so no dimension between the pivots is ever stated.
point o hint(x: 0, y: 0)
point q hint(x: crank, y: 0)
line datum(o, q) class construction
circle orbit(center: q) hint(r: crank) class construction

horizontal datum
radius(crank) orbit
o on orbit
ground o

// the machine itself, at one pose — `u` unbound, so the crank angle is an unknown of the
// drawing and the pen may be dragged
cell: Cell(orbit, datum, arm: arm, side: side)

// Where the pen goes: the drawn cell's own `pen`, as its crank angle runs.  The drawing's pose
// is where the trace is anchored, so it needs no seeds of its own.  `rail` runs through two
// pinned points of the path, and the claim says it is vertical — which nothing above states.
// The diagnosis reports it a theorem; delete it and the drawing is unchanged, which is what a
// claim promises.
curve path = cell.pen over u in (60, 115)

point g1 hint(x: 80, y: 51)
point g2 hint(x: 80, y: 114)
g1 on(u == 65) path
g2 on(u == 110) path
line rail(g1, g2) class construction

claim vertical rail

// Diagnosed: dof 1, Under — the crank — and the claim a theorem.  Drag `cell.pen`; the cell
// folds and stretches to carry it along the line it cannot leave.
