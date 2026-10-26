// A chamfer that spills onto a round along a straight edge (#422). A block's far face has its
// north rim rounded 3, so the face goes on tangent into the round at z = 21. A rib stands on the
// far face beside that edge, its north wall at z = 19.5, and its foot is chamfered 2 against the
// far face: the leg on the face reaches 21.5, past the seam, so it lands on the round.
//
// The flat stands against whichever face of the chain is nearest (std's sets read the face's
// smooth surface, `F.smooth`): over the far face it is the line where the distances from the face
// and the wall sum to 2; past the round's normal at the seam it is where the distances from the
// round and the wall do — a conic's arc. Swept along the edge that section is a prism, so the
// part is built exactly: `solventc --step` writes a plane and an extrusion of the conic for the
// flat. Write `round(r: 2mm)` for the chamfer and the ball beside the wall touches the round, not
// the face. Edit the rib's wall or the leg and apply (⌘↵).
unit mm
use std
use std.rounds (round, chamfer)

d := 2mm   // the chamfer's legs

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
  r0 := point
  r1 := point
  r2 := point
  r3 := point
  fix((10, 15)) r0
  fix((30, 15)) r1
  fix((30, 19.5)) r2
  fix((10, 19.5)) r3
  (rs := line(r0, r1)) -> (re := line(r1, r2)) -> (rn := line(r2, r3)) -> (rw := line(r3, r0)) -> close
  ribbed := face(rs, re, rn, rw)
}
block := solid(outline, depth: 12mm)
block.north round(r: 3mm) block.far

rib := solid(ribbed, from: -18mm, to: -12mm)
part := solid(block)
rib union part

part.rib.rn chamfer(d: d) part.block.far
