// A ring with a dimple pressed into its top, off to one side: a ball cut from a torus. The two meet
// in one small closed curve that crosses neither solid's seam, so no edge leads to it; the Rust
// kernel finds it by searching the faces themselves, and refuses rather than guesses where it
// cannot tell. Open the glass box (⌘B) to see it. Raise `sunk` to deepen the dimple; at 0mm the
// ball only rests on the ring, and that touch is refused, not built.
unit mm
use std

ring_r := 30mm     // from the spindle to the middle of the tube
tube_r := 10mm     // the tube's radius
ball_r := 2mm      // the ball's radius
sunk := 1mm        // how far the ball's lowest point is pressed below the ring's top
around := 45deg    // where round the ring, from the page

// The ring: the tube's section in the front plane, turned about the upright spindle.
in std.front {
  construction centerline spindle := line(std.origin, hint((0, 1)))
  fix((0, 1)) spindle.p2
  tube := point hint((ring_r, 0mm))
  std.origin horizontal tube
  std.origin distance(ring_r, along: right) tube
  section := circle(center: tube) hint(r: tube_r)
  radius(tube_r) section
}
ring := solid(face(section), about: spindle)

// The ball, drawn in a plane standing off the front as far as it stands round the ring. It is half a
// disc turned about a level diameter: the upper half, so the ball's seam stays above the ring.
beside := plane
fix(origin == (0mm, -ring_r * sin(around), 0mm)) beside
fix(dir == (1, 0, 0)) beside.u
fix(dir == (0, 0, 1)) beside.v
in beside {
  c := point hint((ring_r * cos(around), tube_r + ball_r - sunk))
  beside.origin distance(ring_r * cos(around), along: right) c
  beside.origin distance(tube_r + ball_r - sunk, along: up) c
  near := point hint((ring_r * cos(around) - ball_r, tube_r + ball_r - sunk))
  far := point hint((ring_r * cos(around) + ball_r, tube_r + ball_r - sunk))
  diameter := line(near, far)
  c midpoint diameter
  horizontal diameter
  meridian := arc(center: c, start: far, end: near) hint(r: ball_r)
  radius(ball_r) meridian
  ball := solid(face(meridian, diameter), about: diameter)
}

part := solid(ring)
ball cut part
