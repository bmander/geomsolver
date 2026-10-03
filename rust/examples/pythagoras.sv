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
la := 30
lb := 40
s := la + lb

O := point
E := point hint(x: s, y: 0)
F := point hint(x: s, y: s)
G := point hint(x: 0, y: s)

horizontal (bottom := line(O, E)) -> perpendicular
(right := line(E, F)) -> perpendicular
(top := line(F, G)) -> perpendicular
(left := line(G, O)) -> close
bottom equal left

// one point on each side, `a` along from the corner it follows going round
P1 := point hint(x: la, y: 0)
P2 := point hint(x: s, y: la)
P3 := point hint(x: lb, y: s)
P4 := point hint(x: 0, y: lb)

P1 on bottom
P2 on right
P3 on top
P4 on left

// the two legs, named here and read everywhere else
O distance(a := la) P1
P1 distance(b := lb) E
E distance(a) P2
F distance(a) P3
G distance(a) P4

// the hypotenuses, which are the inner square
(h1 := line(P1, P2)) -> (h2 := line(P2, P3)) -> (h3 := line(P3, P4)) -> (h4 := line(P4, P1)) -> close

// the theorem, stated as a claim: judged against the figure, never imposed on it
claim P1 distance(c := hypot(a, b)) P2
fix(x == 0, y == 0) O
