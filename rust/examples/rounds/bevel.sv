// A box's top rim chamfered unequally: each of the four edges round its top face cut by a flat 1
// across the top and 2 down the side (brep-bench's chamfer/box-top-asymmetric). The library's
// `chamfer(d: …)` cuts the same distance along both faces; its flat, `std.rounds.Flat(a, b, la: …,
// lb: …)`, takes a leg on each, so a word of two legs is a few lines written here: for each face
// `a` of A and `b` of B that meet, the flat `across` on `a` and `down` on `b`, cut from the body.
//
// The box runs from the origin to (20, 10, 6), drawn on the top plane and standing 6 up from it,
// so its near face is the top. The flat of the edge at y = 10, z = 6 is the plane through the lines
// y = 9, z = 6 and y = 10, z = 4, and the other three likewise; neighbouring flats meet on the line
// from the box's corner at z = 4 to the top's corner, (20, 10, 4) to (19, 9, 6). Built exactly:
// ten planes, 3428/3 mm³ (20·10·4 + ∫₀² (20 − s)(10 − s) ds) and 584 + 56√5 mm². Swap `across`
// and `down` to cut 2 across the top and 1 down the sides.
unit mm
use std
use std.rounds

across := 1mm   // the flat's leg across the top
down := 2mm     // and down each side

A bevel(across, down) B := {
  repeat a in A {
    repeat b in B {
      when b != a and b meets a {
        flat := std.rounds.Flat(a, b, la: across, lb: down)
        taken := solid(flat)
        taken cut A.body
      }
    }
  }
}

in std.top {
  b0 := point
  b1 := point
  b2 := point
  b3 := point
  fix((0, 0)) b0
  fix((20, 0)) b1
  fix((20, 10)) b2
  fix((0, 10)) b3
  (south := line(b0, b1)) -> (east := line(b1, b2)) -> (north := line(b2, b3)) -> (west := line(b3, b0)) -> close
  outline := face(south, east, north, west)
}
box := solid(outline, from: 0mm, to: 6mm)

box.near bevel(across: across, down: down) box
