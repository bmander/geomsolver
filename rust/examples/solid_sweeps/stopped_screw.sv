// A square bar screwed partway along a rod, stopping inside it: `square_thread.sv`'s bar, its tip
// 4 mm from the axis, 2 mm wide along it, screwed from clear below the rod to halfway up it, as a
// lathe's thread stops short of a shoulder. Where it stops is the bar's own faces at the end pose,
// which no contact walk reads. The kernel builds it on one arrangement (the closed class, #721):
// the rod split at once by every face the sweep's boundary can lie on — the bar's faces at both
// end poses, its edges along the axis and its sides' contact lines screwed into cylinders, its
// level edges into helicoidal sheets — and each cell kept where the bar, screwed back, misses it.
// A point at radius `ρ` and height `z` is cut where some roll `θ` holds `|z − k θ| ≤ 1` (`k` the
// lead a radian) and its angle within `acos(4 / ρ)` of the bar's plane: the circle there loses
// `|Θ| + 2 acos(4 / ρ)`, `Θ` those rolls of the sweep, integrated over the rod.
unit mm
use std
use std.solids (cut)
use std.place (coords)
use std.motion (screws)

rod_r := 5mm         // the rod: its radius and length
rod_l := 20mm
tip := 4mm           // how far from the rod's axis the bar's tip runs
width := 2mm         // the bar's width along the axis
back := 12mm         // how far from the axis its other end lies
reach := 8mm         // how far it stands either side of the plane its section is drawn in
lead := 6mm          // how far the screw climbs a turn
stop := 10mm         // how high up the rod the bar's middle stops

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
rod := solid(std.solids.Revolution(face(rod_axis, rod_end, rod_wall, rod_top), about: rod_axis))
construction bar := solid(std.solids.Prism(face(lower, end, upper, nose), from: -reach, to: reach))

// the screw about the rod's axis, from a millimetre under the rod to `stop` up it
feed := frame
feed screws(lead) ax
start := -360deg * (width / 2 + 1mm) / lead
finish := 360deg * stop / lead
construction thread := solid({ p | some t in (start, finish): p in bar at(under: feed) t })
thread cut rod
