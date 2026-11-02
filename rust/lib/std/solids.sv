// `std.solids`: the solids a drawing makes from its faces, each a set (#737).  A solid is
// `solid(S)`, the points inside a set `S`, and nothing else: a prism, a drafted prism and a
// revolution are words written here over the language's own words, and the evaluator recognises
// what their sets say — by their statements, never by these names — so the same set written by
// hand is built the same way, exactly, its faces named alike.
//
// The faces of a solid are named by what bounds them: a section's sides by the edges they are
// swept from; of two ordinates along its normal the greater bounds `near` and the lesser `far`;
// a turn short of a whole one is capped `start` where it begins and `end` where it ends.

use std.motion (turns)

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

// A revolution: the points the section occupies turned about the line `about` (in its plane)
// through some angle from `from` to `to` — a whole turn by default, `to: 90deg` a quarter, `from:
// -90deg, to: 0deg` a quarter the other way — right-handed about the line's own direction.  A
// turn short of a whole one is capped `start` where it begins and `end` where it ends, and each
// side is named by the edge it is turned from.
//
//   use std.solids
//   ring := solid(std.solids.Revolution(sec, about: ax))
component Revolution(section: face, about: line, from: Angle = 0deg, to: Angle = 360deg) := { p |
  turn := frame
  turn turns(ratio: 1) about
  p coincident(under: turn, over t in (from, to)) section
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
  beyond := { q | q outside section }
  within := { q | q inside section }
  distance(p, beyond) - distance(p, within) >= tan(draft) * distance(p, section)
}

// A prism through everything: the points whose foot on the section's plane lies within it,
// unbounded along its normal.  It is a cutter: cut from a body (`pin cut body`) or kept in common
// with one, it reaches through the body's material both ways, however the drawing moves it; its
// sides are named by the section's edges.
//
//   pin := solid(std.solids.Through(hole))
//   pin cut body
component Through(section: face) := { p | p inside section }
