// A solid pipe with a bore drilled square into its wall (issue #66, rung 2), the bore's rim rounded
// off: a convex edge on a traced saddle, the ball's material taken away. The fillet reads `drilled`,
// the pipe with its bore, named, since a fillet rounds the edges of what a body adds, not cuts.
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
  bore_k := circle(center: c2) hint(r: 6)
}
fix(x == 0, y == 0) c2
radius(6) bore_k
main := solid(face(main_k), from: -30mm, to: 30mm)
bore := solid(face(bore_k), from: 0mm, to: 25mm)
drilled := solid(main)
bore cut drilled
pipe := solid(drilled)
rim := fillet(drilled.main, drilled.bore, r: 1mm)
rim cut pipe
