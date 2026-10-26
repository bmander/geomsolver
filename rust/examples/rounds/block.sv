// A block with its near rim rounded and its far rim chamfered, each by one word of the library:
// `A round(r: …) B` rounds every edge where a face of A meets a face of B, `A chamfer(d: …) B`
// cuts each with a flat. Neither is a feature of the kernel. Both are written in Solvent, in
// `rust/lib/std/rounds.sv`, over faces read as sets (`block.near` is the points of that face), the
// distance from a set, a bound on a sum of measurements, where an edge ends, and repetition over a
// solid's faces. What each word makes is a region the field reads point by point, cut from the
// block — and, being sets the lowering table recognises, each is built exactly too (a prism whose
// face is a cylinder, or a plane), so the block exports by `solventc --step`.
//
// The block is drawn on the front plane and stands 12 deep behind it: its near face is the one
// you see square on. Orbit (right-drag) to see both rims. Edit `r` or `d` and apply (⌘↵). Write `round(r: r) block` instead to round every edge, its
// corners where three runs meet spherical, and `chamfer(d: d) block` to cut every edge flat.
unit mm
use std
use std.rounds (round, chamfer)

r := 2mm   // the near rim's radius
d := 1mm   // how far the far rim's flat runs along each face

in std.front {
  b0 := point
  b1 := point
  b2 := point
  b3 := point
  fix((0, 0)) b0
  fix((40, 0)) b1
  fix((40, 24)) b2
  fix((0, 24)) b3
  (south := line(b0, b1)) -> (east := line(b1, b2)) -> (north := line(b2, b3)) -> (west := line(b3, b0)) -> close
  outline := face(south, east, north, west)
}
block := solid(outline, depth: 12mm)

block.near round(r: r) block
block.far chamfer(d: d) block
