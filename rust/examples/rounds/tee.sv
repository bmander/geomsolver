// A pipe tee, solid: a branch standing square on a main pipe, the crotch filled. No line or
// circle carries where the two meet, so the ball is rolled along the loop they meet in, traced:
// its centre where the two pipes' surfaces grown by the radius meet, its arc between where it
// touches each swept round the loop. Nothing in the source says so — the round is the same
// word as anywhere else. Built exactly: the canal the ball's arc sweeps, fitted as a B-spline
// surface.
//
// Edit `r` or the branch's radius and apply (⌘↵).
unit mm
use std (round)

r := 2mm   // the crotch's radius, filled

in std.side {
  c1 := point
  main_k := circle(center: c1) hint(r: 10)
}
fix((0, 0)) c1
radius(10mm) main_k
in std.top {
  c2 := point
  stem_k := circle(center: c2) hint(r: 6)
}
fix((0, 0)) c2
radius(6mm) stem_k
main := solid(face(main_k), from: -30mm, to: 30mm)
stem := solid(face(stem_k), from: 0mm, to: 25mm)
tee := solid(main)
stem union tee
// Concave, all the way round: filled.
tee.stem round(r: r) tee.main
