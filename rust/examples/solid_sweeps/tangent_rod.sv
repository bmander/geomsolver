// A rod turned about an axis it lies across, hollowing a block: a rod of radius 1 mm and 6 mm
// long, its axis level and running along the way it turns, 5 mm out from an upright axis, turned a
// quarter turn about that axis inside a block that holds all of it. Its contact under the turn is
// its top and bottom lines, where its normal is upright, and the circle across its middle, where the
// turn's velocity runs along its axis. Seen in the half-plane about the axis, each line runs in to
// its middle and back out again — its distance from the axis least where it lies square to the
// radius — so its image there turns back, and the sheet it sweeps folds along that point's path, a
// circle of radius 5 mm. The closed class reads the rod's side curved past its candidates at the
// fold, and the contact's road builds it (#880 T1c): under a turn each sheet lies on its meridian's
// surface of revolution, so the fold's two leaves lie on one surface — here the plane each line turns
// in — and the trim keeps them once, as it keeps once the sheets the rod's two end rims sweep, whose
// images about the axis are one curve. The block loses, on each circle about the axis, the arcs the
// rod lies across at the start carried round a quarter turn: 6π + 5π²/2 + 9π/2 mm³ and the arcs'
// far ends' share, ∫ (c² + 9)·atan(3/c) − 3c dz with c = 5 + √(1 − z²) over z in (−1, 1).
unit mm
use std
use std.solids (cut)
use std.place (coords)
use std.motion (turns)

out := 5mm           // from the axis to the rod's axis
rod := 1mm           // the rod's radius
half := 3mm          // half the rod's length

in std.front {
  construction centerline ax := line(std.origin, hint((0, 1)))
  fix((0, 1)) ax.p2
}

// the rod, drawn in the level plane through its axis, which runs along the turn's way
private along := axis
fix(dir == (0, 1, 0)) along
private up := axis
fix(dir == (0, 0, 1)) up
bench := plane(u: along, v: up)
fix(origin == (5, 0, 0)) bench
in bench {
  private a0 := point
  private a1 := point
  private a2 := point
  private a3 := point
  a0 coords(du: -half, dv: 0mm) bench
  a1 coords(du: half, dv: 0mm) bench
  a2 coords(du: half, dv: rod) bench
  a3 coords(du: -half, dv: rod) bench
  private rod_axis := line(a0, a1)
  private far := line(a1, a2)
  private side := line(a2, a3)
  private near := line(a3, a0)
}
construction bar := solid(std.solids.Revolution(face(rod_axis, far, side, near), about: rod_axis))

// the block about the axis, wide and tall enough to hold the whole sweep
in std.top {
  private b0 := point
  private b1 := point
  private b2 := point
  private b3 := point
  fix((-8, -8)) b0
  fix((8, -8)) b1
  fix((8, 8)) b2
  fix((-8, 8)) b3
  private s0 := line(b0, b1)
  private s1 := line(b1, b2)
  private s2 := line(b2, b3)
  private s3 := line(b3, b0)
}
block := solid(std.solids.Prism(face(s0, s1, s2, s3), from: -2mm, to: 2mm))

// the turn about the upright axis, a quarter turn
swinging := frame
swinging turns(1) ax
construction hollow := solid({ p | some t in (0deg, 90deg): p in bar at(under: swinging) t })
hollow cut block
