// X0 (#183): solid_fillet.sv, the cast mounting pad, with its rounds said as statements about the
// body rather than as nouns. Does not parse yet: `A round(r) B` over face paths is proposed.
//
// A boss standing on a plate, its root filled, two of the plate's top edges rounded off, and a
// bore drilled down through the boss. Which rounds add material and which take it away is read
// off the geometry: the root is concave, the top edges convex. Nothing says `union` or `cut`.
unit mm
use std
width := 60mm
depth := 40mm
thickness := 10mm
boss_radius := 8mm
boss_height := 20mm
bore_radius := 4mm
root_radius := 3mm
edge_radius := 2mm

component Rectangle(center: point, w: Length, h: Length) {
  a := point hint(x: center.x - w / 2, y: center.y - h / 2)
  b := point hint(x: center.x + w / 2, y: center.y - h / 2)
  c := point hint(x: center.x + w / 2, y: center.y + h / 2)
  d := point hint(x: center.x - w / 2, y: center.y + h / 2)
  profile := horizontal (ab := line(a, b)) -> vertical (bc := line(b, c)) ->
            horizontal (cd := line(c, d)) -> vertical (da := line(d, a)) -> close
  distance(w) ab
  distance(h) bc
  diagonal := line(a, c)
  center midpoint diagonal
}

outline := Rectangle(std.front.origin, w: width, h: depth) in std.front
in std.front {
  rim := circle(center: std.front.origin) hint(r: boss_radius)
  radius(boss_radius) rim
  hole := circle(center: std.front.origin) hint(r: bore_radius)
  radius(bore_radius) hole
}

plate := solid(outline.profile, depth: thickness)
boss := solid(face(rim), from: 0mm, to: boss_height)
bore := solid(face(hole), from: -thickness, to: boss_height)
pad := solid(plate)
boss union pad
bore cut pad

// The boss's root: every edge where a face the boss gave the pad meets one the plate gave it.
// Named, so its faces are `pad.root.round`, `pad.root.a`, `pad.root.b`.
root := pad.boss round(root_radius) pad.plate
// Two of the plate's top edges, each running out flush into the plate's sides.
pad.plate.near round(edge_radius) pad.plate.da
pad.plate.near round(edge_radius) pad.plate.bc
