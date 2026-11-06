// A ring plunged at a lean into a block, stopping inside it: a torus, its tube's middle 3 mm from
// its axis and the tube 1 mm thick, its axis leaning 35° from upright, pushed straight down 8 mm
// from clear above the block until it is wholly under the block's top. A slide leaning to a ring's
// axis touches it along two closed curves, where the tube's normal is square to the push — the
// outer and the inner, each smooth where each level's two points of it meet, at its top and bottom
// — and each is carried down into the cylinder over its outline seen along the push: the outer
// outline is the wall of the hole the ring bores, the inner the wall of the core its own hole
// leaves standing. No contact walk builds it (the push ends in the block); the kernel splits the
// block at once by the ring where it stops and those two cylinders, met in closed form, and keeps
// each cell the ring, carried back along the push, misses (the closed class, #805). Leaning further
// than `tan² β = (R − r)/r` (here 54.7°) the inner outline would cusp, and that is refused by name.
// Every line down the push meets the ring once or not at all, so the block loses, over the
// annulus between the two outlines, each line's depth to the ring's far side: by the ring's
// symmetry about its centre that is the centre's depth, 4 mm, times the annulus's area, and half
// the ring's volume, `π² R r²`.
unit mm
use std
use std.solids (cut)
use std.place (coords)
use std.motion (slides)

lean := 35deg        // how far the ring's axis leans from upright

in std.top {
  private b0 := point
  private b1 := point
  private b2 := point
  private b3 := point
  b0 coords(du: -8mm, dv: -8mm) std.top
  b1 coords(du: 8mm, dv: -8mm) std.top
  b2 coords(du: 8mm, dv: 8mm) std.top
  b3 coords(du: -8mm, dv: 8mm) std.top
  private s0 := line(b0, b1)
  private s1 := line(b1, b2)
  private s2 := line(b2, b3)
  private s3 := line(b3, b0)
}
block := solid(std.solids.Prism(face(s0, s1, s2, s3), from: -8mm, to: 0mm))

// the ring, drawn in a plane through its axis, its centre 4 mm above the block's top
private across := axis
fix(dir == (cos(lean), 0, -sin(lean))) across
private along := axis
fix(dir == (sin(lean), 0, cos(lean))) along
bench := plane(u: across, v: along)
fix(origin == (0, 0, 4)) bench
in bench {
  private a0 := point
  private a1 := point
  a0 coords(du: 0mm, dv: -1mm) bench
  a1 coords(du: 0mm, dv: 1mm) bench
  private ring_axis := line(a0, a1)
  private tc := point
  tc coords(du: 3mm, dv: 0mm) bench
  private tube := circle(center: tc) hint(r: 1)
  radius(1mm) tube
}
construction ring := solid(std.solids.Revolution(face(tube), about: ring_axis))

// the push: 8 mm straight down, the ring's centre ending 4 mm under the block's top
private down := axis
fix(dir == (0, 0, -1)) down
ram := frame
ram slides(8mm) down
construction bore := solid({ p | some t in (0deg, 360deg): p in ring at(under: ram) t })
bore cut block
