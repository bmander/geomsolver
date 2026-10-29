// Corners of other angles. A hex box open at its top: its walls' inner faces meet at 120°, the
// hexagon moved in by `wall` a hexagon `wall / sin 60°` smaller to its corners. A wedge sealed:
// its section a right triangle, moved in by `wall` a triangle of the same shape about the same
// incentre, scaled by (ρ − wall) / ρ, ρ the inradius.
//
// Both built exactly: the hex box to its hexagon less the smaller one over 13; the wedge to 8000
// less the scaled triangle over 16.
unit mm
use std
use std.rounds (shell)
use std.shapes

wall := 2mm

in std.top {
  h := point
  fix((0, 0)) h
  hx := point
  fix((10, 0)) hx
  ref := line(h, hx)
  hexes := std.shapes.Polygon(h, ref, n: 6, r: 20mm, phase: 0deg)
  hex := face(hexes.v[0], hexes.v[1], hexes.v[2], hexes.v[3], hexes.v[4], hexes.v[5], -> close)
}
in std.front {
  w0 := point
  w1 := point
  w2 := point
  fix((50, 0)) w0
  fix((90, 0)) w1
  fix((50, 20)) w2
  (ws := line(w0, w1)) -> (wh := line(w1, w2)) -> (wv := line(w2, w0)) -> close
  triangle := face(ws, wh, wv)
}
box := solid(hex, from: 0mm, to: 15mm)
wedge := solid(triangle, depth: 20mm)

box.near shell(t: wall) box
shell(t: wall) wedge
