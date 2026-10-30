// A square thread cut with a single point: a rectangular bar pointing at a rod's axis, its flat tip
// `tip` from the axis, screwed along the rod `lead` a turn from clear below it to clear above, as a
// lathe cuts a thread. The bar is a prism standing across the rod, its length square to the plane
// through the axis its section is drawn in, and a screw moves each of its rulings' points as an
// affine field moves them, so its contact is linear along each ruling: one point a ruling, fixed
// on the bar (the constant-twist class, a prism's sides, #684 M). The tip touches along its middle
// ruling's crossing, the line where it is nearest the axis, and the corners where it meets the
// flanks along their rulings, ever farther out along them as their fans' normals turn toward the
// flanks', which the screw slides along and never touches. The thread's root is the tip's
// cylinder; its flanks are the corners' paths, so the groove widens outward, by the turn a corner
// point makes to come round to the tip's plane: `lead × acos(tip / r) / π` at radius r.
unit mm
use std
use std.place (coords)
use std.motion (screws)

rod_r := 5mm         // the rod: its radius and length
rod_l := 20mm
tip := 4mm           // how far from the rod's axis the bar's tip runs
width := 2mm         // the bar's width along the axis
back := 12mm         // how far from the axis its other end lies
reach := 8mm         // how far it stands either side of the plane its section is drawn in
lead := 6mm          // how far the screw climbs a turn

in std.front {
  construction centerline ax := line(std.origin, hint((0, 1)))
  fix((0, 1)) ax.p2
  private r0 := point
  private r1 := point
  private r2 := point
  private r3 := point
  fix((0, 0)) r0
  r1 coords(du: rod_r, dv: 0mm) std.front
  r2 coords(du: rod_r, dv: rod_l) std.front
  r3 coords(du: 0mm, dv: rod_l) std.front
  private rod_axis := line(r3, r0)
  private rod_end := line(r0, r1)
  private rod_wall := line(r1, r2)
  private rod_top := line(r2, r3)

  // the bar's section, in the plane through the rod's axis: its tip, its flanks, its back
  private b0 := point
  private b1 := point
  private b2 := point
  private b3 := point
  b0 coords(du: tip, dv: -width / 2) std.front
  b1 coords(du: back, dv: -width / 2) std.front
  b2 coords(du: back, dv: width / 2) std.front
  b3 coords(du: tip, dv: width / 2) std.front
  private lower := line(b0, b1)
  private end := line(b1, b2)
  private upper := line(b2, b3)
  private nose := line(b3, b0)
}
rod := solid(face(rod_axis, rod_end, rod_wall, rod_top), about: rod_axis)
construction bar := solid(face(lower, end, upper, nose), from: -reach, to: reach)

// the screw about the rod's axis, and the roll that carries the bar from a millimetre under the
// rod to a millimetre over it
cut := frame
cut screws(lead) ax
start := -360deg * (width / 2 + 1mm) / lead
finish := 360deg * (rod_l + width / 2 + 1mm) / lead
construction thread := sweep(bar over t in (start, finish), under: cut)
thread cut rod
