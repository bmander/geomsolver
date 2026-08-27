// The Pythagorean theorem, drawn instead of written.
//
// A square of side `a + b` holds four copies of a right-angled triangle with legs `a` and `b`,
// one in each corner, each turned a quarter-turn from the last.  What they leave in the middle
// is a second square, standing on their hypotenuses.  Compare the big square with the four
// triangles plus the small one and you get `(a + b)² = 4·ab/2 + c²`, which reduces to
// `c² = a² + b²`.
//
// The drawing gives `a` and `b` **once**, as named numbers on the first triangle; every other
// leg reads those names rather than repeating a number.  So editing either one moves all four
// triangles together, and the figure stays a proof rather than becoming a picture of one.
//
// The inner square's side is then *claimed* to be `hypot(a, b)` — and that is the theorem.  A
// claim (§9.7) is judged, never solved for: the figure is built entirely from the legs, and the
// diagnosis checks the hypotenuse against it and reports the claim a theorem — true, and adding
// nothing the construction does not already say.  Change `a` or `b` and it stays so, which is
// the part worth watching.

// the legs, as numbers the drawing is placed from; `a` and `b` below are the dimensions that
// state them, and every other leg reads those names rather than these
param la = 30
param lb = 40
param s = la + lb

point O hint at (0, 0)
point E hint at (s, 0)
point F hint at (s, s)
point G hint at (0, s)

line bottom(O, E)
line right(E, F)
line top(F, G)
line left(G, O)

perpendicular(bottom, right)
perpendicular(right, top)
perpendicular(top, left)
horizontal(bottom)
equal_length(bottom, left)

// one point on each side, `a` along from the corner it follows going round
point P1 hint at (la, 0)
point P2 hint at (s, la)
point P3 hint at (lb, s)
point P4 hint at (0, lb)

point_on_line(P1, bottom)
point_on_line(P2, right)
point_on_line(P3, top)
point_on_line(P4, left)

// the two legs, named here and read everywhere else
distance(O, P1) == a = la
distance(P1, E) == b = lb
distance(E, P2) == a
distance(F, P3) == a
distance(G, P4) == a

// the hypotenuses, which are the inner square
line h1(P1, P2)
line h2(P2, P3)
line h3(P3, P4)
line h4(P4, P1)

// the theorem, stated as a claim: judged against the figure, never imposed on it
claim distance(P1, P2) == c = hypot(a, b)
ground(O)
