// A round written in the drawing itself, not taken from the library: what `std.sv` says, said
// again here in a few lines, to show it is only the language.
//
// `Rolled(a, b, r)` is the material a ball of radius `r` rolled along the convex edge where faces
// `a` and `b` meet cannot reach. It is a set: the points `p`
//   - within both faces' sides, at an edge whose material angle is under a half turn;
//   - at least `r` from `centres`, the set of places the ball's centre may stand (within both
//     faces' sides and at least `r` from each face's surface);
//   - nearer the edge (`crease`, the points on both faces) than wherever else the two surfaces
//     meet short of where the edge ends (`meeting`), so only the corner by the edge is taken;
//   - and within where the edge ends (`crease.ends`), so the run is carried on past each end and
//     cut off by the faces it ends on.
// `A rolled(r) B` is a word that makes solids: for each face `a` of A and `b` of B it makes that
// set a solid and cuts it from the body the faces are of.
//
// Here it rounds the one edge where the block's near face meets its south face, by 3mm. Edit `r`,
// or name other faces (`block.near rolled(r: r) block` rounds the near rim, mitred at the
// corners).
unit mm
use std

r := 3mm

component Rolled(a: face, b: face, r: Length) := { p |
  p inside a
  p inside b
  angle(a, b, at: p) <= 180deg
  centres := { c | c inside a; c distance(>= r) a.surface; c inside b; c distance(>= r) b.surface }
  crease := { q | q coincident a; q coincident b }
  meeting := { q | q coincident a.surface; q coincident b.surface; q inside crease.ends }
  distance(p, centres) >= r
  distance(p, crease) <= distance(p, meeting)
  p inside crease.ends
}

A rolled(r) B := {
  repeat a in A {
    repeat b in B {
      when b != a {
        run := Rolled(a, b, r: r)
        taken := solid(run)
        taken cut A.body
      }
    }
  }
}

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

block.near rolled(r: r) block.south
