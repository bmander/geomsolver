// A ring pushed into a block at so steep a lean its inner outline folds: `leaning_ring.sv`'s torus
// — its tube's middle 3 mm from its axis, the tube 1 mm thick — its axis leaning 65° from upright,
// past `tan² β = (R − r)/r` (54.7°), pushed straight down 8 mm from clear above the block until it
// is wholly under the block's top. Its inner contact, carried down the push, no longer sweeps a
// cylinder over a smooth outline: seen along the push the inner outline cusps four times, a
// swallowtail between each two, and the sheet the inner contact sweeps folds along each cusp's path.
// No class reads a candidate for it (the closed class refuses the fold by name, `Folded`); the
// contact's road carries each face's contact exactly down the push, cut into stretches clear of the
// folds, sets the ring where it starts and where it stops beside them, gives every meeting by
// construction and keeps, of the folded sheet, what bounds the sweep (#880 T1). The block loses what
// every line down the push loses — the ring's intervals on it pushed down 8 mm, their union, clipped
// to the block — which brep-bench's `ring-pushed-in-past-its-fold` integrates line by line.
unit mm
use std
use std.solids (cut)
use std.place (coords)
use std.motion (slides)

lean := 65deg        // how far the ring's axis leans from upright, past its fold

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
