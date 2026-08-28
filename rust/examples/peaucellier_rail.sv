// The same 1864 straight-line linkage as `peaucellier.sv`, proved without a curve.  There the
// pen's path is traced as a locus and two points of it are claimed level; here the pen is simply
// joined to a grounded point, and the claim is that the join is vertical.  The price is that this
// version has to be *told* where the line is, where its sibling discovers it.

// the three lengths the machine is built from
param arm = 100       // the long arms, o–c and o–d
param side = 60       // the four sides of the kite, b–c–pen–d
param crank = 40      // the crank q–b, and the orbit its pin rides

// The fixed frame.  `o on orbit` is the theorem's whole hypothesis — the pin's
// circle passes through the centre of inversion — and it places `q` too, so no dimension between
// the pivots is ever stated.
point o hint(x: 0, y: 0)
point q hint(x: crank, y: 0)
line datum(o, q) class construction
circle orbit(center: q) hint(r: crank) class construction

horizontal datum
radius(crank) orbit
o on orbit
ground o

// the machine itself, at one pose; the crank is the one freedom left
point b   hint(x: 50.4, y: 38.6)
point c   hint(x: 30.4, y: 95.2)
point d   hint(x: 99.9, y: 4.8)
point pen hint(x: 80.0, y: 61.4)

line swing(q, b)
b on orbit

line oc(o, c)
line od(o, d)
oc equal od
o distance(arm) c

line bc(b, c) to line cp(c, pen) to line pd(pen, d) to line db(d, b) to close
bc equal cp equal pd equal db
b distance(side) c

ccw(o, b, c)                           // c left of the arm, d right,
cw(o, b, d)
ccw(c, d, pen)                         // and the pen on the far side of the kite from b

// The whole proof, in four statements.  A claim is judged on whether stating it would cost a
// freedom, so a theorem here does not mean "the pen happens to be at 80" — it means saying so
// takes nothing from the crank, which is to say the pen's x never changes as the crank turns.
point anchor hint(x: 80, y: 0)
ground anchor
line rail(anchor, pen) class construction
claim vertical rail

// Diagnosed: dof 1, Under — the crank — and the claim a theorem.  Move the anchor and the claim
// is refuted rather than quietly passing, which is what makes it a test; but edit `arm` or `side`
// and it is refuted too, because the anchor still says 80.  The sibling that never states the
// number survives that edit, and this one does not — which is the cost of writing the answer down.
