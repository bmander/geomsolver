// A rectangular bar screwed along a rod, cutting a spiral groove in it as a lathe's tool cuts a
// thread: the bar points at the rod's axis, its flat tip `tip` from it, and stands across the rod,
// its length square to the plane through the axis its section is drawn in.  A screw moves each of
// its rulings' points as an affine field moves them, so its contact is linear along each ruling:
// one point a ruling, fixed on the bar (the constant-twist class, a prism's sides, #684 M). The
// tip touches along its middle ruling's crossing, the line where it is nearest the axis, and the
// corners where it meets the flanks along their rulings, ever farther out along them as their
// fans' normals turn toward the flanks', which the screw slides along and never touches. The
// groove's root is the tip's cylinder; its walls are the corners' paths, so it widens outward, by
// the turn a corner point makes to come round to the tip's plane: `lead × acos(tip / r) / π` at
// radius r.  No roll is stated: the screw carries the bar through the rod from clear below it to
// clear above, however long the rod (#271).
unit mm
use std
use std.solids
use std.sets
use std.place (coords, above)
use std.motion (screws)

rod_r := 5mm         // the rod: its radius and length
rod_l := 20mm
tip := 4mm           // how far from the rod's axis the bar's tip runs
width := 2mm         // the bar's width along the axis
back := 12mm         // how far from the axis its other end lies
reach := 8mm         // how far it stands either side of the plane its section is drawn in
lead := 6mm          // how far the screw climbs a turn

in std.front {
  // the rod's axis, up from its foot at the origin, as long as the rod
  construction centerline ax := line(std.origin)
  ax.p2 above(d: rod_l) std.origin
  // the bar's section is dimensioned from the axis and the rod's foot: along the axis, then out
  // from it
  private along := std.Turned(ax.p1, ax.p2)
  private b0 := point
  private b1 := point
  private b2 := point
  private b3 := point
  b0 coords(du: -width / 2, dv: tip) along.axes
  b1 coords(du: -width / 2, dv: back) along.axes
  b2 coords(du: width / 2, dv: back) along.axes
  b3 coords(du: width / 2, dv: tip) along.axes
  private lower := line(b0, b1)
  private end := line(b1, b2)
  private upper := line(b2, b3)
  private nose := line(b3, b0)
}
// the rod: within `rod_r` of its axis, between its two ends
rod := solid({ p | p in std.sets.Cylinder(ax, r: rod_r)
                   p not in std.sets.HalfSpace(ax.p1, ax, d: 0mm)
                   p in std.sets.HalfSpace(ax.p2, ax, d: 0mm) })
construction bar := solid(std.solids.Prism(face(lower, end, upper, nose), from: -reach, to: reach))

// the screw about the rod's axis, carrying the bar through it
cut := frame
cut screws(lead) ax
construction thread := solid({ p | some t: p in bar at(under: cut) t })
thread cut rod
