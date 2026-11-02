// `std.solids`: the solids a drawing makes from its faces, each a set (#737).  A solid is
// `solid(S)`, the points inside a set `S`, and nothing else: a prism, a drafted prism and a
// revolution are words written here over the language's own words, and the evaluator recognises
// what their sets say — by their statements, never by these names — so the same set written by
// hand is built the same way, exactly, its faces named alike.
//
// The faces of a solid are named by what bounds them: a section's sides by the edges they are
// swept from; of two ordinates along its normal the greater bounds `near` and the lesser `far`;
// a turn short of a whole one is capped `start` where it begins and `end` where it ends.

// A prism: the points whose foot on the section's plane lies within it, between the ordinates
// `from` and `to` along its normal from that plane — `from: -30mm, to: 0mm` the material behind
// the face the view shows.  `near` is its cap at `to`, `far` at `from`, and each side is named by
// the edge it is swept from (`block.ab`).
//
//   use std.solids
//   block := solid(std.solids.Prism(sec, from: -30mm, to: 0mm))
component Prism(section: face, from: Length, to: Length) := { p |
  p inside section
  p distance(>= from, along: n) section
  p distance(<= to, along: n) section
}
