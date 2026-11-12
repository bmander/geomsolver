// A regular tetrahedron grown by 1 — brep-bench's offset/tetrahedron-grown — written as the bench
// describes it: conv{a, b, c, d} ⊕ B₁, the points within 1 of the hull of the four corners.
// The hull is the join of two opposite edges (§6.21, [0.212]); the points within 1 of it are
// those a ball of 1 about some point of it reaches. Built exactly: each face moved 1 out, each
// edge a cylinder's sector, each corner a ball's, its volume V + S + 3e(π − acos ⅓) + 4π/3.
unit mm
use std
use std.sets

a := point
b := point
c := point
d := point
fix((3, 3, 3)) a
fix((3, -3, -3)) b
fix((-3, 3, -3)) c
fix((-3, -3, 3)) d
construction hull := solid({ p | some q in std.sets.Segment(a, b), r in std.sets.Segment(c, d):
  p in std.sets.Segment(q, r) })
grown := solid({ p | some q in hull: p distance(<= 1mm) q })
