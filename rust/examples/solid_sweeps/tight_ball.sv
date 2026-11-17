// A ball screwed round a helix tighter than itself, hollowing a block: a ball of radius 2 mm, its
// centre 1.5 mm from an upright axis, screwed about it two turns, rising 3 mm a turn — brep-bench's
// `ball-screwed-round-a-tight-helix` — inside a block that holds all of it. Its centre's helix bends
// at a radius of `(ρ² + c²)/ρ`, about 1.65 mm, less than the ball's, so the tube about it folds: its
// inner wall turns back on itself along two cuspidal edges, and the two turns overlap. Its tube
// meets itself, so no canal builds it; the closed class refuses the fold by name (`Folded`); the
// contact's road carries the ball's contact exactly round the helix, cut into stretches clear of
// the folds, sets the ball where it starts and where it stops beside it, gives every meeting by
// construction and keeps, of the folded sheet, what bounds the sweep (#880 T1). The block loses the
// whole of what the ball sweeps — inside it, a cavity — which each circle about the axis covers
// as far as the arcs the ball's poses lay on it run together.
unit mm
use std
use std.solids (cut)
use std.place (coords)
use std.motion (screws)

off_axis := 1.5mm    // the ball's centre from the axis
ball := 2mm          // the ball's radius
lead := 3mm          // how far the screw climbs a turn

// the ball, its centre on the axis's foot plane, drawn as a half disc turned about its diameter
in std.front {
  construction centerline ax := line(std.origin, hint((0, 1)))
  fix((0, 1)) ax.p2
  private centre := point
  centre coords(du: off_axis, dv: 0mm) std.front
  private bottom := point
  private top := point
  bottom coords(du: off_axis, dv: -ball) std.front
  top coords(du: off_axis, dv: ball) std.front
  private diameter := line(bottom, top)
  private meridian := arc(center: centre, start: bottom, end: top) hint(r: 2)
  radius(ball) meridian
}
construction cutter := solid(std.solids.Revolution(face(meridian, diameter), about: diameter))

// the block about the axis, wide and tall enough to hold the whole sweep
in std.top {
  private b0 := point
  private b1 := point
  private b2 := point
  private b3 := point
  fix((-5, -5)) b0
  fix((5, -5)) b1
  fix((5, 5)) b2
  fix((-5, 5)) b3
  private s0 := line(b0, b1)
  private s1 := line(b1, b2)
  private s2 := line(b2, b3)
  private s3 := line(b3, b0)
}
block := solid(std.solids.Prism(face(s0, s1, s2, s3), from: -4mm, to: 10mm))

// the screw about the axis, two turns
chase := frame
chase screws(lead) ax
construction hollow := solid({ p | some t in (0deg, 720deg): p in cutter at(under: chase) t })
hollow cut block
