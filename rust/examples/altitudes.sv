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

use std

in std.front {
  A := point
  B := point
  C := point

  (ab := line(A, B)) -> (bc := line(B, C)) -> (ca := line(C, A)) -> close

  // Each altitude runs from a vertex to a foot that is free to slide along it.  The feet are
  // written *in the lines* rather than declared above them: nothing else in this file says their
  // names, and a name earns its place when something says it twice.  They are still there to
  // constrain and to drag — `alt_a.p2` is the point, and that dotted path is what it is called.
  alt_a := line(A, hint((15, 5)))
  alt_b := line(B, hint((20, 10)))
  alt_c := line(C, hint((15, -5)))

  alt_a perpendicular bc
  alt_b perpendicular ca
  alt_c perpendicular ab

  // two of these place P; the third is the theorem
  P := point hint((15, 8))
  P coincident alt_a
  P coincident alt_b
  P coincident alt_c

  fix((0, 0)) A
  fix((40, 0)) B
  fix((15, 30)) C
}
