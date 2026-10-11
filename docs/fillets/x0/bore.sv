// X0 (#183): solid_fillet_bore.sv, a solid pipe with a bore drilled square into its wall, the
// bore's rim rounded off. Does not parse yet.
//
// The stack named an intermediate (`drilled`, then `pipe := solid(drilled)`), since its fillet
// read only the edges a body adds, not those it cuts. Under the ordering proposed here (a round
// comes after every term that makes a face it names, and before every other), the rim is said on
// the one body that is drilled.
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
  bore_k := circle(center: c2) hint(r: 6)
}
fix((0, 0)) c2
radius(6) bore_k
main := solid(face(main_k), from: -30mm, to: 30mm)
bore := solid(face(bore_k), from: 0mm, to: 25mm)
pipe := solid(main)
bore cut pipe

// Convex, on a saddle: taken away.
pipe.main round(1mm) pipe.bore
