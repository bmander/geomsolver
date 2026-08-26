// Structurally fine, geometrically impossible.
//
// Three points, three lengths, one of every pair — the count a triangle wants, and the matching
// says so.  But 1 + 1 < 10, and the triangle inequality is not something a degree-of-freedom
// count can see: `b` and `c` would have to meet a unit from `a` and a unit from each other while
// `a` and `b` stand ten apart, and no position does that.  So the structure is well-constrained
// and the solve has nowhere to go, which is the whole point of the case.

point a hint at (0, 0)
point b hint at (10, 0)
point c hint at (5, 5)

line ab(a, b)

distance(a, b) == 10
distance(b, c) == 1
distance(a, c) == 1
horizontal(ab)

ground(a)
