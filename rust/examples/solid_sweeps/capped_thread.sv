// A square thread cut by a bar too short to stand past the rod: `square_thread.sv`'s bar, standing
// only 2.5 either side of the plane its section is drawn in, so its ends pass through the rod. A
// screw moves each of its rulings' points as an affine field moves them, so it touches at one point
// a ruling (the constant-twist class, a prism's sides, #684 M): along its tip's middle, and out along
// the corners where the tip meets its flanks as their fans' normals turn toward the flanks'. Short,
// those corners' points of contact reach the bar's ends before the flanks do: a vertex of the prism
// at each, where the corner meets an end's rims. Every normal of the cone between the three faces
// meeting there is the bar's, and those square to the screw stand at the vertex, so the contact goes
// on round it onto the end's rim and along it (#721 G1). The groove's walls meet the rims' sweeps
// in sharp edges along the vertices' helices. A point of the bar at radius r turns at most
// `acos(4 / r)` from the tip's plane, or `asin(2.5 / r)` where its end stops it first, so the groove
// at radius r is the bar's width and the lead's share of twice the lesser, along the axis. The rod
// keeps 1303.9950 mm³ of its 1570.7963.
unit mm
use std
use std.solids (cut)
use std.sets
use std.place (coords, above)
use std.motion (screws)

rod_r := 5mm         // the rod: its radius and length
rod_l := 20mm
tip := 4mm           // how far from the rod's axis the bar's tip runs
width := 2mm         // the bar's width along the axis
back := 12mm         // how far from the axis its other end lies
reach := 2.5mm       // how far it stands either side of the plane its section is drawn in
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
feed := frame
feed screws(lead) ax
construction thread := solid({ p | some t: p in bar at(under: feed) t })
thread cut rod
