// How to draw a straight line.
//
// Every rod in this drawing swings about a pivot, so every point of it moves on a circle —
// there is no straight edge here, no rail, nothing straight to copy from.  Yet the pen draws
// an exact straight line.  This is the Peaucellier–Lipkin cell of 1864, the first mechanism
// ever to do it, and the reason is inversive geometry: the kite of four equal rods holds `pen`
// on the ray from `o` through `b` with  ob · o-pen  constant, so `pen` is the *inversion* of
// `b` — and `b` rides a circle that passes through the centre of inversion, and the inversion
// of a circle through its own centre is a straight line.
//
// The document states none of that.  It states what a machinist would: two long arms the same
// length, four short rods the same length, a crank pin riding a circle — and the one condition
// the whole theorem hangs on, said as an incidence: `point_on_circle(o, orbit)`, the pin's
// circle passing exactly through the fixed centre.  The curve family then traces where the pen
// goes as the crank turns, stated the same way over a scratch copy of the linkage, with no
// formula anywhere.
//
// The punchline is the last statement in the file: `claim vertical(rail)`.  A claim (§9.7) is
// a relation stated as *expected to add no rank* — it is never solved for, so it cannot pull
// the drawing toward itself, and the diagnosis judges it instead: a theorem when the rest of
// the document already implies it, `violated` when it is simply untrue, `consuming` when only
// the pose happens to satisfy it.  Here the rail is the line through two points of the traced
// path, the claim says it is vertical, and nothing in the document supports that — except that
// it is *true*: the diagnosis reports the claim a **theorem**.  Edit any number in the file —
// the arms, the sides, the orbit — and the rail moves, but it never tilts.  The drawing has
// discovered what Kelvin, shown the cell, called the most beautiful thing he had ever seen:
// straightness manufactured out of turning.
//
// One degree of freedom is left on purpose — the crank.  Drag `pen` and the whole cell folds
// and stretches to carry it along the line it cannot leave.

// the three lengths the machine is built from; the pen draws the vertical line
// x = (arm² − side²) / (2 · crank) — but nothing below says so
param arm = 100       // the long arms, o–c and o–d
param side = 60       // the four sides of the kite, b–c–pen–d
param crank = 40      // the crank q–b, and so the orbit its pin rides

// where the pen must go, as a locus: a scratch copy of the linkage, posed at crank angle u.
// Eight numbers to find (b, c, d, p), eight statements — the same eight a machinist would
// check with a ruler.  Each of the doctrine's three instruments does the one job it is for.
// An `angle` is directed, so which side of the frame the crank swings is in the residual
// itself and no predicate has to say so; the ccw/cw lines pick which of the mirror poses
// the elbows take, read once at the home angle and carried round by continuity.  The
// seeds say only where to start looking: `b` at the edge of its own circle, and the elbows
// split to either side of the arm — without that split, c and d start as one point, and a
// solve that begins symmetric stays symmetric, folded flat.  (The bearing u/2 is no
// derivation, just the inscribed angle: `o` is *on* the orbit, so the pin seen from `o` moves
// at half the rate the crank turns it.)  And p itself needs no guess at all, because "sixty
// from c and sixty from d, on the far side" meets in exactly one place.
curve cell(o: point, q: point, datum: line, orbit: circle, arm: Length, side: Length)(u) =
  trace p from (90) where {
    point b at orbit bearing (u)
    point c at (o.x + arm * cos(u / 2 + 35), o.y + arm * sin(u / 2 + 35))
    point d at (o.x + arm * cos(u / 2 - 35), o.y + arm * sin(u / 2 - 35))
    point p
    line swing(q, b)
    point_on_circle(b, orbit)          // the crank pin on its circle...
    angle(datum, swing) == u           // ...posed at bearing u — directed, so this side up,
    distance(o, c) == arm              // the two long arms,
    distance(o, d) == arm
    distance(b, c) == side             // and the kite closed round b and p
    distance(b, d) == side
    distance(p, c) == side
    distance(p, d) == side
    ccw(o, b, c)                       // c on the left of the arm...
    cw(o, b, d)                        // ...d on the right
    ccw(c, d, p)                       // and p on the far side of the kite from b
  }

// the frame: the fixed centre, the crank pivot beside it, and the orbit the pin rides.
// `point_on_circle(o, orbit)` is the theorem's whole hypothesis — the pin's circle passes
// through the centre of inversion — and it is an incidence, not a number: it also places `q`,
// exactly one orbit-radius from `o`, so no dimension between the pivots is ever stated.
point o hint at (0, 0)
point q hint at (crank, 0)
line datum(o, q) construction
circle orbit(center: q, r: crank) construction

horizontal(datum)
radius(orbit) == crank
point_on_circle(o, orbit)
ground(o)

// the machine itself, at one pose
point b   hint at (50.4, 38.6)
point c   hint at (30.4, 95.2)
point d   hint at (99.9, 4.8)
point pen hint at (80.0, 61.4)

line swing(q, b)
point_on_circle(b, orbit)

line oc(o, c)
line od(o, d)
oc equal od
distance(o, c) == arm

line bc(b, c) to line cp(c, pen) to line pd(pen, d) to line db(d, b) to close
bc equal cp equal pd equal db
distance(b, c) == side

ccw(o, b, c)                           // the same mirror choices as the trace makes,
cw(o, b, d)                            // so the drawn cell is the traced one
ccw(c, d, pen)

// the pen's path, and the claim the linkage has been converging on for a hundred and sixty
// years.  `rail` is the line through two pinned points of the trace, and the claim states the
// theorem outright.  Delete it and the drawing is the same — that is what a claim promises,
// and what the diagnosis checks.
curve path = cell(o, q, datum, orbit, arm: arm, side: side) over (60, 115)

point g1 hint at (80, 51)
point g2 hint at (80, 114)
point_on_curve(g1, path, u == 65)
point_on_curve(g2, path, u == 110)
line rail(g1, g2) construction

claim vertical(rail)

// Diagnosed: dof 1, Under — the crank — and the claim a theorem.
