// An impeller's vane drafted on its disc (#962): a flat vane 70 long and 4 thick standing 30 on
// a disc 3 deep, its sides leaning 2° in as it rises, as it comes out of a die parted at the disc's
// face. Each end is an arc of 4.25 bulging half a millimetre past the sides' ends, meeting them at
// an angle, not tangent.
//
// Drafted, each straight side is a plane leaning in and each end a cone about its arc's centre,
// narrowing as it rises. Where a side meets an end the corner moves along the curve the plane and
// the cone meet in, a parabola (the moved line and the moved circle meet where the corner's
// distance to the circle's centre and to the line, moved, keep their sum): the exact kernel builds
// that edge, and `solventc --step` writes it as the conic it is.
//
// Drawn on the top plane, the disc's face at z = 0, the vane standing up from it. Orbit
// (right-drag) to see a corner's edge curve in toward the tip. Edit the draft and apply (⌘↵).
unit mm
use std
use std.solids (union)

dr := 2deg   // the vane's draft

in std.top {
  o := point
  fix((0, 0)) o
  rim := circle(center: o) hint(r: 100)
  radius(100mm) rim

  // the vane's sides' ends, and its ends' centres: each end bulges 0.5 past them
  a := point
  b := point
  c := point
  d := point
  fix((20, -2)) a
  fix((90, -2)) b
  fix((90, 2)) c
  fix((20, 2)) d
  ct := point
  cr := point
  fix((86.25, 0)) ct
  fix((23.75, 0)) cr
  south := line(a, b)
  tip := arc(center: ct, start: b, end: c)
  north := line(c, d)
  root := arc(center: cr, start: d, end: a)
}
disc := solid(std.solids.Prism(face(rim), from: -3mm, to: 0mm))
vane := solid(std.solids.Drafted(face(south, tip, north, root), from: 0mm, to: 30mm, draft: dr))
impeller := solid(disc)
vane union impeller
