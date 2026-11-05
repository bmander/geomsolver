// `std.solids`: the solids a drawing makes from its faces, each a set (#737).  A solid is
// `solid(S)`, the points in a set `S`, and nothing else: a prism, a drafted prism, a
// countersink and a revolution are words written here over the language's own words, and the
// evaluator recognises what their sets say — by their statements, never by these names — so the
// same set written by hand is built the same way, exactly, its faces named alike.
//
// The faces of a solid are named by what bounds them: a section's sides by the edges they are
// swept from; of two ordinates along its normal the greater bounds `near` and the lesser `far`,
// and one alone (a countersink's) `near` where it bounds from above and `far` from below; a turn
// short of a whole one is capped `start` where it begins and `end` where it ends.

// -- the body words (#783 P3) ---------------------------------------------------------------------
//
// A body is the points of its stock, or of anything in `union` with it, that are in everything it
// keeps in `common` and in nothing `cut` from it — `(stock ∪ unions) ∖ cuts ∩ commons`, a formula
// over sets, never an order of statements, so `bore cut body` may stand fifty lines above `body`.
// What is in union is a disjunct of the body's set and what is cut or kept a conjunct, said where
// the body is not declared (`B or S`, `B and S`): the language's primitive, which these name.  So a
// pocket eats a boss in union with the same body; a part that keeps the boss names the pocketed
// shell and puts the boss on that.  A word that reads the body's own faces (a round's) acts on the
// body its other terms make.
//
//   use std.solids (cut)
//   bore cut body
x union B := B or { p | p in x }
x cut B := B and { p | p not in x }
x common B := B and { p | p in x }


// A prism: the points the section occupies slid some way along its normal, between the ordinates
// `from` and `to` along its normal from that plane — `from: -30mm, to: 0mm` the material behind
// the face the view shows.  `near` is its cap at `to`, `far` at `from`, and each side is named by
// the edge it is swept from (`block.ab`).
//
//   use std.solids
//   block := solid(std.solids.Prism(sec, from: -30mm, to: 0mm))
component Prism(section: face, from: Length, to: Length) := { p |
  some h: p in section at(along: n) h
  p distance(>= from, along: n) section
  p distance(<= to, along: n) section
}

// A revolution: the points the section occupies turned about the line `about` (in its plane)
// through some angle from `from` to `to` — a whole turn by default, `to: 90deg` a quarter, `from:
// -90deg, to: 0deg` a quarter the other way — right-handed about the line's own direction.  A
// turn short of a whole one is capped `start` where it begins and `end` where it ends, and each
// side is named by the edge it is turned from.
//
//   use std.solids
//   ring := solid(std.solids.Revolution(sec, about: ax))
component Revolution(section: face, about: line, from: Angle = 0deg, to: Angle = 360deg) := { p |
  some t in (from, to): p in section at(under: about) t
}

// A drafted prism: `Prism`'s points, its section at each ordinate its own with every edge moved
// into the material by the ordinate's size times `tan(draft)` — the sides leaning away from the
// section's plane, the parting plane, on both sides of it, as a forging's come out of its die; a
// negative draft leans them the other way.  Its faces are named as a prism's, a side crossing the
// parting plane two pieces of one name.
//
//   rod := solid(std.solids.Drafted(sec, from: -4mm, to: 4mm, draft: 3deg))
component Drafted(section: face, from: Length, to: Length, draft: Angle) := { p |
  p distance(>= from, along: n) section
  p distance(<= to, along: n) section
  beyond := { q | not some h: q in section at(along: n) h }
  within := { q | some h: q in section at(along: n) h }
  distance(p, beyond) - distance(p, within) >= tan(draft) * distance(p, section)
}

// A countersink: the cone under the circle `rim` whose sides meet at the included angle `angle`,
// as a drawing calls one out (`⌵ Ø6 × 90°`) — the points behind the face the view shows, each
// section the rim moved in by its ordinate's size times `tan(angle / 2)`, to the cone's point.  It
// is `Drafted`'s set with no far ordinate: the lean closes the rim at the apex, so nothing says how
// deep it reaches, and a hole cut with it is sunk to wherever the cone meets the hole.  The sides
// lean from the rim's own plane, so draw `rim` where the sink opens: in a plane lying on the face
// it is cut from (a sketch on a face).  `near` is its cap, the rim's disc, and its side is named
// by the rim's edge.
//
//   sink := solid(std.solids.Countersink(face(rim), angle: 90deg))
//   sink cut strap
component Countersink(rim: face, angle: Angle) := { p |
  p distance(<= 0mm, along: n) rim
  beyond := { q | not some h: q in rim at(along: n) h }
  within := { q | some h: q in rim at(along: n) h }
  distance(p, beyond) - distance(p, within) >= tan(angle / 2) * distance(p, rim)
}

// A prism through everything: the points the section occupies slid any way along its normal,
// unbounded along it.  It is a cutter, naming no body: cut from bodies (`pin cut body`) or
// kept in common with them, it reaches through each one's material both ways, however the drawing
// moves it; its sides are named by the section's edges.
//
//   pin := solid(std.solids.Through(hole))
//   pin cut body
component Through(section: face) := { p | some h: p in section at(along: n) h }
