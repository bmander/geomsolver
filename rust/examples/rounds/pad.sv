// A cast mounting pad: a boss standing on a plate, its root filled, two of the plate's edges
// rounded off, and a bore drilled down through the boss. Three statements say the rounds, and none
// says which way: the boss's root is concave, so its round fills; the plate's edges are convex, so
// theirs cut. The bore is cut after, and its wall is no face either round names, so neither round
// reaches it.
//
// The plate is drawn about the front plane's origin, its near face (the one the boss stands on)
// the one you see square on; `da` and `bc` are its left and right sides, `ab` and `cd` its bottom
// and top. Each side's round runs the plate's whole height and is cut off flush by the bottom and
// top faces it ends on. Edit a radius and apply (⌘↵), or round the top edge too:
// `pad.plate.near round(r: edge_radius) pad.plate.cd`, mitred where it meets the side's.
unit mm
use std (round)

width := 60mm
depth := 40mm
thickness := 10mm
boss_radius := 8mm
boss_height := 20mm
bore_radius := 4mm
root_radius := 3mm   // the boss's root, filled
edge_radius := 2mm   // the plate's two side edges, cut

outline := std.CenteredRectangle(std.origin, w: width, h: depth) in std.front
in std.front {
  rim := circle(center: std.origin) hint(r: 8)
  radius(boss_radius) rim
  hole := circle(center: std.origin) hint(r: 4)
  radius(bore_radius) hole
}

plate := solid(outline.loop, depth: thickness)
boss := solid(face(rim), from: 0mm, to: boss_height)
bore := solid(face(hole), from: -thickness, to: boss_height)
pad := solid(plate)
boss union pad
bore cut pad

// every edge where a face the boss gave the pad meets one the plate gave it: the root's circle
pad.boss round(r: root_radius) pad.plate
// the near face's edge along each side
pad.plate.near round(r: edge_radius) pad.plate.da
pad.plate.near round(r: edge_radius) pad.plate.bc
