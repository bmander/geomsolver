// A cast mounting pad (issue #66, rung 1): a boss standing on a plate, the ball rolled round its
// root filling the corner, two of the plate's top edges rounded off, and a bore drilled down
// through the boss after it is filleted. A fillet is a noun, the rolling ball's material along
// the edges where two solids or faces meet; a body adds it at a concave edge with `union` and
// takes it away at a convex one with `cut`.
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

outline := Rectangle(std.origin, w: width, h: depth)
rim := circle(center: std.origin) hint(r: boss_radius)
radius(boss_radius) rim
hole := circle(center: std.origin) hint(r: bore_radius)
radius(bore_radius) hole

plate := solid(outline.profile, depth: thickness)
boss := solid(face(rim), from: 0mm, to: boss_height)
bore := solid(face(hole), from: -thickness, to: boss_height)
pad := solid(plate)
boss union pad

// the boss's root: every edge where a face of the boss meets a face of the plate, a ring
root := fillet(boss, plate, r: root_radius)
root union pad
// two of the plate's top edges, each a straight round ending flush in the plate's sides
left := fillet(plate.near, plate.da, r: edge_radius)
right := fillet(plate.near, plate.bc, r: edge_radius)
left cut pad
right cut pad
bore cut pad
