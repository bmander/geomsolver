// The three altitudes of a triangle meet at a point — and nothing here says so.
//
// An altitude is a line from a corner, square to the opposite side.  Draw all three of them and
// they always pass through a single point.  That is a theorem about every triangle, not a
// coincidence of this one, and this drawing does not state it: `P` is simply told to lie on all
// three lines.
//
// Two of those three statements are enough to place `P`.  The third is therefore saying nothing
// new — but only because the theorem happens to be true.  Counting cannot see that: it is one
// more statement about one more line, and the books balance.  Finding it takes actually moving
// the triangle and solving again, to see that the third statement is still redundant at a
// different shape rather than just at this one.
//
// Three degrees of freedom are left deliberately — the foot of each altitude may slide along its
// line — so the triangle can be dragged about while the three lines go on meeting.

point A hint at (0, 0)
point B hint at (40, 0)
point C hint at (15, 30)

line ab(A, B)
line bc(B, C)
line ca(C, A)

// each altitude runs from a vertex to a foot that is free to slide along it
point QA hint at (15, 5)
point QB hint at (20, 10)
point QC hint at (15, -5)

line alt_a(A, QA)
line alt_b(B, QB)
line alt_c(C, QC)

perpendicular(alt_a, bc)
perpendicular(alt_b, ca)
perpendicular(alt_c, ab)

// two of these place P; the third is the theorem
point P hint at (15, 8)
point_on_line(P, alt_a)
point_on_line(P, alt_b)
point_on_line(P, alt_c)

ground(A)
ground(B)
ground(C)
