// The Pythagorean theorem, drawn.
//
// A square of side `a + b` holds four copies of the right triangle with legs `a` and `b`, one in
// each corner and each turned a quarter from the last.  What they leave in the middle is a square
// on their hypotenuses, so `(a + b)² = 4 · ab/2 + c²`, which is `c² = a² + b²`.
//
// The drawing states `a` and `b` **once**, as named dimensions on the first triangle, and every
// other leg reads the name — so editing either number moves all four triangles together.  The
// inner square's side is then dimensioned `c = hypot(a, b)`, and that is the theorem: it is an
// equation the figure *already satisfies*, so the diagnosis reports it as redundant and
// consistent rather than as a conflict, and it stays that way when `a` or `b` is edited.

// the legs, as numbers the drawing is placed from; `a` and `b` below are the dimensions that
// state them, and every other leg reads those names rather than these
param la = 30
param lb = 40
param s = la + lb

point O at (0, 0)
point E at (s, 0)
point F at (s, s)
point G at (0, s)

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
point P1 at (la, 0)
point P2 at (s, la)
point P3 at (lb, s)
point P4 at (0, lb)

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

// true without being imposed
distance(P1, P2) == c = hypot(a, b)
ground(O)
