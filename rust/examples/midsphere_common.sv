// A cube trimmed by its midsphere: the solid common to a cube of side 10 about the origin and the
// ball about the origin that touches each of the cube's twelve edges at its middle.  The ball's
// radius is not stated: it goes through a corner of the cube's middle section, which is the
// middle of an edge the section is swept along, and the solve finds 5√2.
//
// Each face of the cube cuts the sphere in the circle inscribed in that face, and the circles of
// neighbouring faces touch at the middle of the edge between them: the result is six discs and
// eight three-sided patches of the sphere, two of each meeting at a point at every edge's middle.
// Its volume is the ball less six caps, π(1250 − 2000√2/3) = 965.069.
unit mm
use std
use std.sets

side := 10mm   // the cube's side

in std.front {
  a := point hint((-5, -5))
  b := point hint((5, -5))
  c := point hint((5, 5))
  d := point hint((-5, 5))
  section := horizontal (ab := line(a, b)) -> vertical (bc := line(b, c)) ->
             horizontal (cd := line(c, d)) -> vertical (da := line(d, a)) -> close
  length(side) ab
  length(side) bc
  diagonal := line(a, c)
  std.origin midpoint diagonal
  cube := solid(section, from: -side / 2, to: side / 2)
}

ball := std.sets.Sphere(std.origin, r: hint(7mm))
a coincident ball

body := solid(ball)
cube common body
