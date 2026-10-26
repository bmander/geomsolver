// A pipe tee, solid: a branch standing square on a main pipe, the crotch between them filled. The
// two cylinders meet in a saddle, a curve in space no line or circle of the drawing carries, and
// the round needs none: it is the material a ball of radius `r` rolled round the saddle outside
// both pipes cannot reach, read off the two faces as they stand.
//
// The main pipe is drawn on the side plane and runs along x; the branch is drawn on the top plane
// and rises along z from the main pipe's axis. Only the saddle is rounded: the two cylinders'
// surfaces meet again below the main pipe's axis, where the body has no edge, and the round makes
// nothing there. Orbit (right-drag) to see the crotch. Edit `r` or a pipe's radius and apply (⌘↵).
unit mm
use std
use std.rounds (round)

r := 2mm   // the crotch's radius

in std.side {
  c1 := point
  fix((0, 0)) c1
  main_k := circle(center: c1) hint(r: 10)
  radius(10mm) main_k
}
in std.top {
  c2 := point
  fix((0, 0)) c2
  stem_k := circle(center: c2) hint(r: 6)
  radius(6mm) stem_k
}
main := solid(face(main_k), from: -30mm, to: 30mm)
stem := solid(face(stem_k), from: 0mm, to: 25mm)
tee := solid(main)
stem union tee

tee.stem round(r: r) tee.main
