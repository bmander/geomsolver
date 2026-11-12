// Joins: the points on a segment from some point of one set to some point of another (§6.21,
// [0.212]). The join of convex sets is their hull, so one word builds a polytope from what it is
// the join of: a tetrahedron of two of its opposite edges, an octahedron of a square and the axis
// through it, a prismatoid of a square and a smaller one turned a quarter above it. A set is
// given by name or written in place, and a join is a set like any, so joins nest. Each solid is
// built exactly, as the kernel's hull of the corners its sets are the hulls of.
unit mm
use std
use std.sets

component Join(A: set, B: set) := { p | some q in A, r in B: p in std.sets.Segment(q, r) }

// the regular tetrahedron: the join of two of its opposite edges
a := point
b := point
c := point
d := point
fix((3, 3, 3)) a
fix((3, -3, -3)) b
fix((-3, 3, -3)) c
fix((-3, -3, 3)) d
tetrahedron := solid(Join(std.sets.Segment(a, b), std.sets.Segment(c, d)))

// a square standing on its corners, itself the join of two of its sides, and the axis through it
s0 := point
s1 := point
s2 := point
s3 := point
fix((40, 0, 0)) s0
fix((30, 10, 0)) s1
fix((20, 0, 0)) s2
fix((30, -10, 0)) s3
top := point
bottom := point
fix((30, 0, 10)) top
fix((30, 0, -10)) bottom
square := Join(std.sets.Segment(s0, s1), std.sets.Segment(s2, s3))
octahedron := solid(Join(square, std.sets.Segment(top, bottom)))

// the same square, as the floor of a prismatoid whose roof is a smaller square turned a quarter
// from it, 12 above: the roof written in place as a set of its own
r0 := point
r1 := point
r2 := point
r3 := point
fix((34, 4, 12)) r0
fix((26, 4, 12)) r1
fix((26, -4, 12)) r2
fix((34, -4, 12)) r3
prismatoid := solid(Join(square, { x | some u in std.sets.Segment(r0, r1),
  v in std.sets.Segment(r2, r3): x in std.sets.Segment(u, v) }))
