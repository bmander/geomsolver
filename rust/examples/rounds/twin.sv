// A twin bar: two rods of different sizes laid side by side, overlapping, and made one. Their
// walls meet in two straight seams, each a concave edge between two curved faces; one statement
// fills both. In the section square to the seams both walls are circles, and the ball of radius
// `r` stands where the two circles, each grown by `r`, meet: the fill is the corner between two
// arcs that the ball cannot reach, a prism along the bar.
//
// The section is drawn on the front plane and stands 40 deep in front of it. Orbit (right-drag)
// to see the seams filled. Edit `r` or a rod's radius and apply (⌘↵).
unit mm
use std
use std.rounds (round)

r := 2mm   // the seams' radius

in std.front {
  c1 := point
  c2 := point
  fix((0, 0)) c1
  fix((14, 0)) c2
  big := circle(center: c1) hint(r: 10)
  small := circle(center: c2) hint(r: 8)
  radius(10mm) big
  radius(8mm) small
}
left := solid(face(big), from: 0mm, to: 40mm)
right := solid(face(small), from: 0mm, to: 40mm)
bar := solid(left)
right union bar

// the two walls, where they meet: both seams
bar.right.small round(r: r) bar.left.big
