// An arched window through a wall panel (#962): two jambs standing on a sill, and an arch of
// R97.5 springing from their heads, each corner where a jamb meets the arch rounded at R12.  The
// opening is written as its corners and its arch, and each rounded corner as a joint `-> round(r:
// …)` between a line and an arc: the round mints its arc, a tangency at each end and its branch —
// inside the corner, so here inside the arch's circle as well — and the solver finds where it
// touches the jamb and the arch.  No trigonometry, no coordinate of a tangent point: the springing
// points stay corners of the drawing, on the jamb and on the arch's circle, and dimensioned there.
unit mm
use std
use std.solids (cut)

width := 150        // between the jambs
rise := 120         // from the sill to the springing, where the arch meets the jambs
arch_r := 97.5      // the arch's radius
fillet := 12        // where a jamb meets the arch
wall := 300         // the panel, square
thickness := 20

in std.front {
  w0 := point
  w1 := point
  w2 := point
  w3 := point
  fix((-wall / 2, -60)) w0
  fix((wall / 2, -60)) w1
  fix((wall / 2, wall - 60)) w2
  fix((-wall / 2, wall - 60)) w3
  (base := line(w0, w1)) -> (east := line(w1, w2)) -> (head := line(w2, w3)) ->
    (west := line(w3, w0)) -> close
  panel_face := face(base, east, head, west)

  sill_l := point
  sill_r := point
  spring_r := point
  spring_l := point
  fix((-width / 2, 0)) sill_l
  fix((width / 2, 0)) sill_r
  fix((width / 2, rise)) spring_r
  fix((-width / 2, rise)) spring_l
  // the arch's centre below the springing, on the window's centre line
  centre := point hint((0, rise - 60))
  (sill := line(sill_l, sill_r)) -> (jamb_r := line(sill_r, spring_r)) ->
    (round_r := round(r: fillet)) (arch := arc(center: centre, start: spring_r, end: spring_l)) ->
    (round_l := round(r: fillet)) (jamb_l := line(spring_l, sill_l)) -> close
  radius(arch_r) arch
  opening := face(sill, jamb_r, round_r, arch, round_l, jamb_l)
}

panel := solid(std.solids.Prism(panel_face, from: 0, to: thickness))
light := solid(std.solids.Prism(opening, from: -1, to: thickness + 1))
light cut panel
