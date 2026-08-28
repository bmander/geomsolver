// How to draw a straight line.  Every rod in the Peaucellier–Lipkin cell of 1864 swings about a
// pivot, so nothing here is straight, yet `pen` traces an exact line: the kite inverts `b` about
// `o`, and the inversion of a circle through the centre of inversion is a line.  The document
// states none of that — the last statement *claims* it, and the diagnosis proves it.

// the three lengths the machine is built from; the pen draws the line x = (arm² − side²) / (2·crank)
param arm = 100       // the long arms, o–c and o–d
param side = 60       // the four sides of the kite, b–c–pen–d
param crank = 40      // the crank q–b, and the orbit its pin rides

// Where the pen goes, as a locus: a scratch linkage posed at crank angle u, eight numbers against
// eight statements.  Only `b` is seeded — the ccw/cw lines pick the elbows, and a seed repeating a
// predicate is the weaker of two statements of one fact.
curve cell(orbit: circle, f: frame, arm: Length, side: Length)(u) =
  trace p from (90) where {
    line datum(f.origin, f.toward)
    point b hint at orbit bearing (u + f.angle)   // `u` is measured from the datum, so the seed is too
    point c
    point d
    point p
    line swing(f.toward, b)
    point_on_circle(b, orbit)          // the crank pin on its circle...
    angle(datum, swing) == u           // ...posed at bearing u; directed, so this side up
    distance(f.origin, c) == arm       // the two long arms,
    distance(f.origin, d) == arm
    distance(b, c) == side             // and the kite closed round b and p
    distance(b, d) == side
    distance(p, c) == side
    distance(p, d) == side
    ccw(f.origin, b, c)                // c left of the arm, d right,
    cw(f.origin, b, d)
    ccw(c, d, p)                       // and p on the far side of the kite from b
  }

// The fixed frame; `f` is the datum as something measurable, which is where the seed above gets
// its bearing.  `point_on_circle(o, orbit)` is the theorem's whole hypothesis — the pin's circle
// passes through the centre of inversion — and it places `q` too, so no dimension between the
// pivots is ever stated.
point o hint(x: 0, y: 0)
point q hint(x: crank, y: 0)
line datum(o, q) class construction
frame f(origin: o, toward: q) class construction
circle orbit(center: q) hint(r: crank) class construction

horizontal(datum)
radius(orbit) == crank
point_on_circle(o, orbit)
ground(o)

// the machine itself, at one pose
point b   hint(x: 50.4, y: 38.6)
point c   hint(x: 30.4, y: 95.2)
point d   hint(x: 99.9, y: 4.8)
point pen hint(x: 80.0, y: 61.4)

line swing(q, b)
point_on_circle(b, orbit)

line oc(o, c)
line od(o, d)
oc equal od
distance(o, c) == arm

line bc(b, c) to line cp(c, pen) to line pd(pen, d) to line db(d, b) to close
bc equal cp equal pd equal db
distance(b, c) == side

ccw(o, b, c)                           // the same mirror choices the trace makes,
cw(o, b, d)                            // so the drawn cell is the traced one
ccw(c, d, pen)

// `rail` runs through two pinned points of the pen's path, and the claim says it is vertical —
// which nothing above states.  The diagnosis reports it a theorem; delete it and the drawing is
// unchanged, which is what a claim promises.
curve path = cell(orbit, f, arm: arm, side: side) over (60, 115)

point g1 hint(x: 80, y: 51)
point g2 hint(x: 80, y: 114)
point_on_curve(g1, path, u == 65)
point_on_curve(g2, path, u == 110)
line rail(g1, g2) class construction

claim vertical(rail)

// Diagnosed: dof 1, Under — the crank — and the claim a theorem.  Drag `pen`; the cell folds and
// stretches to carry it along the line it cannot leave.
