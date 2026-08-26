// The three altitudes of a triangle meet in a point — and the drawing does not say so.
//
// The triangle is nailed down, and each altitude is a line from a vertex made perpendicular to
// the opposite side.  `P` is then put on all three.  Two of those incidences place it; the third
// is the theorem, and it is *true without being imposed* — which is exactly what the structural
// count cannot see, since to a matching the third incidence looks like one more equation for one
// more unknown.  Only the witness configuration finds it: jiggle the triangle, re-solve, and the
// third row is still dependent, at every pose rather than at this one.
//
// Three DOF are left on purpose (the altitude feet slide along their lines), so the case can be
// dragged while the concurrency holds.

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
