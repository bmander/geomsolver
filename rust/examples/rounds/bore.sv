// A solid pipe with a bore drilled square into its wall, the bore's rim rounded off. The rim is
// where the cut's wall meets the pipe's: a saddle, convex, so the round takes material away. The
// round is said of the one body that is drilled, by the faces the cut gave it (`pipe.bore`): the
// bore's wall here, its floor meeting no face of the pipe's. No intermediate is named, since a
// round reads its body's faces as the terms that make them leave them, and acts after.
//
// The pipe is drawn on the side plane and runs along x; the bore is drawn on the top plane and
// goes down along z to the pipe's axis. Orbit (right-drag) to see the rim. Edit `r` and apply
// (⌘↵), or round the bore's floor into its wall too: `round(r: 1mm) pipe.bore`, concave, so
// filled.
unit mm
use std
use std.rounds (round)

r := 1mm   // the rim's radius

in std.side {
  c1 := point
  fix((0, 0)) c1
  main_k := circle(center: c1) hint(r: 10)
  radius(10mm) main_k
}
in std.top {
  c2 := point
  fix((0, 0)) c2
  bore_k := circle(center: c2) hint(r: 6)
  radius(6mm) bore_k
}
main := solid(face(main_k), from: -30mm, to: 30mm)
bore := solid(face(bore_k), from: 0mm, to: 25mm)
pipe := solid(main)
bore cut pipe

pipe.main round(r: r) pipe.bore
