// A boss rounded into a frame whose inside edge is already rounded (#962): an L-shaped frame,
// floor and wall 10 thick, its inside edge filled 2; a round boss of 6 standing across that edge,
// its axis 2 off the wall, so it meets the fill's axis; and the boss's foot rounded 2 into the
// frame, against the floor, the fill's face and the wall.
//
// Against the fill's concave face a ball of the fill's own radius has its centre on the fill's
// axis alone, so where the boss crosses the fill the ball stands still, a radius off the boss, one
// each side; it rounds the stretch where the boss crosses the fill with a patch of its sphere, from
// the section through the stretch's end on the floor's seam to the one on the wall's.  The half
// ring along the floor and the runs up the wall end in those same sections, so the round is one
// piece all the way round the boss's foot.  Built exactly: 37286.086 mm³ — two spheres, a torus,
// cylinders and planes.
//
// Edit `r` and apply (⌘↵): over 2 the ball never touches the fill and is refused, saying both
// radii.  Move the boss off the fill's axis (`fix((14, -20)) o`): the ball along the floor stops
// short of where the boss meets the fill, stands still, and its patch is in two, the floor's part
// and the fill's; nearer the wall (`fix((11, -20)) o`) the floor's ball rolls on over the fill to
// where it stands, a stretch of torus.
unit mm
use std
use std.solids (union)
use std.rounds (round)

R := 2mm   // the frame's inside edge
r := 2mm   // the boss into the frame

in std.front {
  p0 := point
  p1 := point
  p2 := point
  p3 := point
  p4 := point
  p5 := point
  fix((0, 0)) p0
  fix((60, 0)) p1
  fix((60, 10)) p2
  fix((10, 10)) p3
  fix((10, 40)) p4
  fix((0, 40)) p5
  (base := line(p0, p1)) -> (end := line(p1, p2)) -> (floor := line(p2, p3)) -> (wall := line(p3, p4)) -> (cap := line(p4, p5)) -> (back := line(p5, p0)) -> close
  section := face(base, end, floor, wall, cap, back)
}
in std.top {
  o := point
  fix((12, -20)) o
  rim := circle(center: o) hint(r: 6)
  radius(6mm) rim
}
frame := solid(std.solids.Prism(section, from: 0mm, to: 40mm))
filled := frame.floor round(r: R) frame.wall

part := solid(frame)
boss := solid(std.solids.Prism(face(rim), from: 5mm, to: 25mm))
boss union part
part.boss round(r: r) part.frame
