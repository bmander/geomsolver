// A ball swung partway round an axis, half sunk in a block: everything a ball passes through is
// the tube about its centre's path — here a stretch of a torus — with half a ball at each end, so
// the groove it leaves is built exactly (the canal, `solid::canal`). Pappus gives the tube, so the
// groove is half of π r² · R θ + 4/3 π r³: 323.03 mm³ at these numbers. Keep `ball` below
// `swing_r`, or the tube would meet itself about the axis.
unit mm
use std
use std.place (coords)
use std.motion (turns)

swing_r := 12mm      // from the axis to the ball's centre
ball := 3mm          // the ball's radius
swing := 90deg       // how far round it turns

// the ball, its centre on the block's top face, drawn as a half disc turned about its diameter
in std.front {
  construction centerline ax := line(std.origin, hint((0, 1)))
  fix((0, 1)) ax.p2
  private centre := point
  centre coords(du: swing_r, dv: 0mm) std.front
  private bottom := point
  private top := point
  bottom coords(du: swing_r, dv: -ball) std.front
  top coords(du: swing_r, dv: ball) std.front
  private diameter := line(bottom, top)
  private meridian := arc(center: centre, start: bottom, end: top) hint(r: 3)
  radius(ball) meridian
}
construction cutter := solid(face(meridian, diameter), about: diameter)

// the block below the top face, wide enough to hold the whole groove
in std.top {
  private b0 := point
  private b1 := point
  private b2 := point
  private b3 := point
  fix((-6, -6)) b0
  fix((18, -6)) b1
  fix((18, 18)) b2
  fix((-6, 18)) b3
  private s0 := line(b0, b1)
  private s1 := line(b1, b2)
  private s2 := line(b2, b3)
  private s3 := line(b3, b0)
}
block := solid(face(s0, s1, s2, s3), from: -8mm, to: 0mm)

// the turn about the upright axis, and the groove the ball sweeps over `swing` of it
swinging := frame
swinging turns(1) ax
construction groove := sweep(cutter over t in (0deg, swing), under: swinging)
groove cut block
