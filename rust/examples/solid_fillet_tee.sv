// A pipe tee, solid (issue #66, rung 2): a branch standing square on a main pipe, the ball rolled
// round the crotch where they meet. No line or circle carries that meeting: the ball's centre runs
// where the two cylinders offset by its radius meet, traced, and the fillet's face is the canal it
// sweeps, fitted as a B-spline surface.
unit mm
use std
in std.side {
  c1 := point
  main_k := circle(center: c1) hint(r: 10)
}
fix(x == 0, y == 0) c1
radius(10) main_k
in std.top {
  c2 := point
  stem_k := circle(center: c2) hint(r: 6)
}
fix(x == 0, y == 0) c2
radius(6) stem_k
main := solid(face(main_k), from: -30mm, to: 30mm)
stem := solid(face(stem_k), from: 0mm, to: 25mm)
tee := solid(main)
stem union tee
crotch := fillet(stem, main, r: 2mm)
crotch union tee
