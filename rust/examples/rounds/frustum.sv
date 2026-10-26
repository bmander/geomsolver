// Half a frustum, rounded all over: a cone's frustum, radius 10 at its foot to 6 at its top, 12
// high, its half behind the plane through its axis cut away. Where that plane meets the cone it
// meets it along two generators: straight edges, but in a section square to one the cone is no
// circle, so the ball is rolled along it, its centre on the plane grown by `r` and the cone shrunk
// by it — a hyperbola. The rims are rings, the cut's edges along its foot and top straight runs,
// and where three meet a corner's cell over a plane, the cone and a plane — its section with each
// rolled run the plane of the ball's arc, through its centre and where it touches both faces —
// each rolled run cut short there and glued to it.
//
// The section is drawn on the front plane and turned about the up axis; the cut stands on the top
// plane. Orbit (right-drag) to see the corners. Edit `r` and apply (⌘↵), or round the generators
// alone (`half.cutter round(r: r) half.cone.bc`).
unit mm
use std
use std.rounds (round)

r := 1mm   // every edge's radius

in std.front {
  a := point
  b := point
  c := point
  d := point
  fix((0, 0)) a
  fix((10, 0)) b
  fix((6, 12)) c
  fix((0, 12)) d
  (ab := line(a, b)) -> (bc := line(b, c)) -> (cd := line(c, d)) -> (da := line(d, a)) -> close
}
in std.top {
  e := point
  f := point
  g := point
  h := point
  fix((-20, -20)) e
  fix((20, -20)) f
  fix((20, 0)) g
  fix((-20, 0)) h
  (ef := line(e, f)) -> (fg := line(f, g)) -> (gh := line(g, h)) -> (he := line(h, e)) -> close
}
cone := solid(face(ab, bc, cd, da), about: std.z)
cutter := solid(face(ef, fg, gh, he), from: -1mm, to: 13mm)
half := solid(cone)
cutter cut half

round(r: r) half
