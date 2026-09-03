// A belt over two pulleys — and a drawing that looks looser than it is.
//
// Each end of the belt is held on its pulley, and the belt is tangent to each.  Said that way,
// the figure appears to have two ways left to move that it does not really have: to a first
// approximation the contact point can slide along the belt, so the usual test reports a freedom.
// Move it any distance at all, though, and the drawing stops fitting together.  The freedom
// exists only in the limit — it is real for an infinitely small step and false for every actual
// one.
//
// No amount of care with the first approximation can tell those apart, because at the solution
// they look identical.  So the check is to try it: nudge the drawing along each apparent
// freedom, solve again, and see whether it stays where it was put or walks straight back.  Here
// it walks back, and the figure is correctly called rigid.
//
// The app no longer *writes* belts this way — naming the point a tangency happens at avoids the
// whole difficulty — but a document may still say it, and older ones do, which is why the case
// is kept.
//
// `side: right` says which way round the belt runs.  It is written out because a document that
// leaves it out gets a fixed default rather than a look at the drawing.

point c1 hint(x: 0, y: 0)
point c2 hint(x: 50, y: 0)

circle k1(center: c1) hint(r: 10)
circle k2(center: c2) hint(r: 10)

point p hint(x: 0, y: 10)
point q hint(x: 50, y: 10)
line  belt(p, q)

radius(10) k1
radius(10) k2
p on k1
q on k2
belt tangent(side: right) k1
belt tangent(side: right) k2

ground c1
ground c2
