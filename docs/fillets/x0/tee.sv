// X0 (#183): solid_fillet_tee.sv, a pipe tee, solid: a branch standing square on a main pipe, the
// crotch filled. No line or circle carries the meeting; the engine traces it (a lowering, L7), but
// nothing in the source says so. Does not parse yet.
unit mm
use std
in std.side {
  c1 := point
  main_k := circle(center: c1) hint(r: 10)
}
fix((0, 0)) c1
radius(10) main_k
in std.top {
  c2 := point
  stem_k := circle(center: c2) hint(r: 6)
}
fix((0, 0)) c2
radius(6) stem_k
main := solid(face(main_k), from: -30mm, to: 30mm)
stem := solid(face(stem_k), from: 0mm, to: 25mm)
tee := solid(main)
stem union tee

tee.stem round(2mm) tee.main
