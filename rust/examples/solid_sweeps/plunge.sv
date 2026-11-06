// A bar plunged aslant into a plate, stopping inside it: a bar of square section stood on a corner,
// lying along the plate below it, pushed up 8 mm along a line leaning toward its length, so it
// ends halfway through. No contact walk builds it: its silhouette turns at the bar's vertices onto
// its caps, and the push ends inside the plate, so where it stops is the bar's own faces at the
// end pose. The kernel builds it on one arrangement (the closed class, #721): the plate split at
// once by every face the sweep's boundary can lie on — each face of the bar where the push enters
// it at the start and leaves it at the end, each silhouette edge swept — and each cell kept where
// the bar, carried back along the push, misses it. At each `y` across the bar its section is the
// rectangle 8 long and `2 (1 − |y|)` high, swept along the push into a hexagon of which the plate
// keeps `0 ≤ z ≤ 4`; the plate loses that, integrated across the bar.
unit mm
use std
use std.solids (cut)
use std.place (coords)
use std.motion (slides)

length := 20mm       // the plate: its length and width, and its thickness
width := 10mm
thick := 4mm

in std.top {
  private b0 := point
  private b1 := point
  private b2 := point
  private b3 := point
  b0 coords(du: 0mm, dv: -width / 2) std.top
  b1 coords(du: length, dv: -width / 2) std.top
  b2 coords(du: length, dv: width / 2) std.top
  b3 coords(du: 0mm, dv: width / 2) std.top
  private near_side := line(b0, b1)
  private far_end := line(b1, b2)
  private far_side := line(b2, b3)
  private near_end := line(b3, b0)
}
plate := solid(std.solids.Prism(face(near_side, far_end, far_side, near_end), from: 0mm, to: thick))

// the bar: its section a square stood on a corner, 1 mm from its middle to each corner, its middle
// 6 mm under the plate, and its length along x from 6 mm to 14 mm
in std.side {
  private c0 := point
  private c1 := point
  private c2 := point
  private c3 := point
  c0 coords(du: 0mm, dv: -7mm) std.side
  c1 coords(du: 1mm, dv: -6mm) std.side
  c2 coords(du: 0mm, dv: -5mm) std.side
  c3 coords(du: -1mm, dv: -6mm) std.side
  private lower := line(c0, c1)
  private upper := line(c1, c2)
  private outer := line(c2, c3)
  private under := line(c3, c0)
}
construction bar := solid(std.solids.Prism(face(lower, upper, outer, under), from: 6mm, to: 14mm))

// the push: 8 mm up a line leaning 0.3 toward x
private up := axis
fix(dir == (0.3, 0, 0.9539392014169457)) up
ram := frame
ram slides(8mm) up
construction pocket := solid({ p | some t in (0deg, 360deg): p in bar at(under: ram) t })
pocket cut plate
