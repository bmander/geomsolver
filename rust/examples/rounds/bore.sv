// A solid pipe with a bore drilled square into its wall, the bore's rim rounded off. The rim is
// where the bore's wall meets the pipe's: a saddle no line or circle carries, so the ball is
// rolled along it, traced, against the pipe as the bore leaves it — the round reads the faces the
// cut makes, on the one body that is drilled, with no intermediate named. Built exactly: the
// canal the ball's arc sweeps, fitted.
//
// Edit `r` or the bore's radius and apply (⌘↵).
unit mm
use std (round)

r := 1mm   // the rim's radius, taken away

in std.side {
  c1 := point
  main_k := circle(center: c1) hint(r: 10)
}
fix((0, 0)) c1
radius(10mm) main_k
in std.top {
  c2 := point
  bore_k := circle(center: c2) hint(r: 6)
}
fix((0, 0)) c2
radius(6mm) bore_k
main := solid(face(main_k), from: -30mm, to: 30mm)
bore := solid(face(bore_k), from: 0mm, to: 25mm)
pipe := solid(main)
bore cut pipe

// Convex, on a saddle: taken away.
pipe.main round(r: r) pipe.bore
