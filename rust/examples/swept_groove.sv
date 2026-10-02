// A curved groove: a ball-end cutter swung through an arc about an upright axis, cut into the top
// of a block. The groove is everything the ball passes through, and a ball half sunk in the top
// face leaves a round-bottomed channel. Change a number and apply (⌘↵) to watch it refine again;
// ⌘B shows it in the glass box. Keep `arc_r + ball_r` inside the block's half width.
unit mm
use std

block_w := 40mm      // the block's width and depth
block_h := 12mm      // its height
ball_r := 4mm        // the cutter's ball
arc_r := 14mm        // from the swing's axis to the ball's centre
swing := 150deg      // how far the cutter swings

// A sphere of radius `r` about `center`: a half disc turned about its upright diameter.
component Sphere(center: point, r: Length) {
  private bottom := point hint(x: center.x, y: center.y - r)
  private top := point hint(x: center.x, y: center.y + r)
  private diameter := line(bottom, top)
  center midpoint diameter
  vertical diameter
  private meridian := arc(center: center, start: bottom, end: top) hint(r: r)
  radius(r) meridian
  body := solid(face(meridian, diameter), about: diameter)
}

// The block: a rectangle below the page's origin, run through the page as deep as it is wide,
// so its top face is level with the origin.
private mid := point hint(x: 0, y: -block_h / 2)
std.origin vertical mid
std.origin distance(block_h / 2, along: down) mid
private outline := std.CenteredRectangle(mid, w: block_w, h: block_h)
construction block := solid(outline.loop, from: -block_w / 2, to: block_w / 2)

// The cutter: a ball with its centre on the top face, swung about the upright axis.
construction centerline spindle := line(std.origin, std.up.toward)
private centre := point hint(x: arc_r, y: 0)
std.origin horizontal centre
std.origin distance(arc_r, along: right) centre
private ball := Sphere(centre, r: ball_r)
swing_about := motion(about: spindle)
construction groove := solid(ball.body, under: swing_about, from: -swing / 2, to: swing / 2)

part := solid(block)
groove cut part
