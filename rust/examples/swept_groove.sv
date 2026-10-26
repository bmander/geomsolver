// A curved groove: a ball-end cutter swung through an arc about an upright axis, cut into the top
// of a block. The groove is everything the ball passes through, and a ball half sunk in the top
// face leaves a round-bottomed channel. What a ball sweeps is the tube about its centre's path —
// here a stretch of a torus with the ball at each end, where the cutter stops inside the block —
// so the part is built exactly. ⌘B shows it in the glass box. Keep `arc_r + ball_r` inside the
// block's half width.
unit mm
use std
use std.place (below, right_of)
use std.shapes
use std.motion (turns)

block_w := 40mm      // the block's width and depth
block_h := 12mm      // its height
ball_r := 4mm        // the cutter's ball
arc_r := 14mm        // from the swing's axis to the ball's centre
swing := 150deg      // how far the cutter swings

// A sphere of radius `r` about `center`: a half disc turned about its upright diameter.
component Sphere(center: point, r: Length) {
  private bottom := point hint((center.x, center.y - r))
  private top := point hint((center.x, center.y + r))
  private diameter := line(bottom, top)
  // on the diameter, and the arc's ends both `r` from it: so its middle, said once
  center coincident diameter
  vertical diameter
  private meridian := arc(center: center, start: bottom, end: top) hint(r: r)
  radius(r) meridian
  body := solid(face(meridian, diameter), about: diameter)
}

// The block: a rectangle below the page's origin, run through the page as deep as it is wide,
// so its top face is level with the origin.
in std.front {
  private mid := point hint((0, -block_h / 2))
  mid below(d: block_h / 2) std.origin
  private outline := std.shapes.CenteredRectangle(mid, w: block_w, h: block_h)
  construction block := solid(outline.loop, from: -block_w / 2, to: block_w / 2)

  // The cutter: a ball with its centre on the top face, swung about the upright axis.
  construction centerline spindle := line(std.origin, hint((0, 1)))
  fix((0, 1)) spindle.p2
  private centre := point hint((arc_r, 0))
  centre right_of(d: arc_r) std.origin
  private ball := Sphere(centre, r: ball_r)
}
swing_about := frame
swing_about turns(1) spindle
construction groove := sweep(ball.body over t in (-swing / 2, swing / 2), under: swing_about)

part := solid(block)
groove cut part
