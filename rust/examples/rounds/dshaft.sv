// A D-shaped shaft: a round of 10 with a flat 6 from its axis, 20 long, rounded all over. Its
// edges are of every kind a round meets here: the flat's edges along each cap run into the round
// and are carried on into it, kept within it; the rims round the round are stretches of rings,
// carried on past the flat and kept within it; the seams where the flat meets the round are runs
// beside a cylinder. Where the flat, the round and a cap meet, the ball touching all three rounds
// the corner: its cell has a patch of the round among its faces.
//
// Drawn on the top plane and standing 20 up from it. Orbit (right-drag) to see the corners. Edit
// `r` and apply (⌘↵), or round the flat's top edge alone (`shaft.near round(r: r) shaft.flat`).
unit mm
use std (round)

r := 2mm   // the radius of every edge

in std.top {
  o := point
  t := point
  s := point
  fix((0, 0)) o
  fix((6, 8)) t
  fix((6, -8)) s
  rim := arc(center: o, start: t, end: s) hint(r: 10)
  flat := line(s, t)
}
shaft := solid(face(rim, flat), from: 0mm, to: 20mm)

round(r: r) shaft
