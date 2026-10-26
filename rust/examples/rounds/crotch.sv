// A pipe tee's crotch chamfered: a branch pipe of radius 6 standing square on a main pipe of
// radius 10, their saddle cut by a flat of legs `d` on both walls — `chamfer`, not `round`. No
// line or circle carries the saddle, so the flat — where the two walls' distances, each over the
// other's leg, sum to one — is sectioned square to the saddle between its legs (each wall met with
// the other grown by its leg) and fitted, its strips on both walls built as a rolled round's are.
//
// The main pipe is drawn on the side plane, the branch on the top plane. Orbit (right-drag) to see
// the flat. Edit `d` and apply (⌘↵), or round it instead (`use std.rounds (round)`, `round(r: d)`).
unit mm
use std
use std.rounds (chamfer)

d := 1mm   // the flat's legs

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

tee.stem chamfer(d: d) tee.main
