// A thread chased with a pointed tool: a V-bit, a cone ground to a 60° point on a round shank,
// lying across a rod with its point a millimetre into it, screwed along the rod 6 mm a turn from
// clear below it to clear above. A screw moves every point of the bit the same way at every time,
// so the groove's wall is the bit's characteristic carried along the helix. On each ring of the
// cone that characteristic is two points, either side of the plane through the rod's axis and the
// bit's, and they meet at the point, where the cone meets its own axis: there every normal of the
// cone round the apex is the bit's, and the characteristic turns from one to the other. The walk
// runs in along one, turns at the apex and runs back out along the other, and the groove's wall is
// two sheets meeting in a sharp edge along the apex's helix, the thread's root
// (docs/generating-sweeps.md, the constant-twist class, an apex on the tool's axis, #721 G5). Its
// section square to the rod's axis is the same at every height, turned, so the rod loses that
// section's area through its 20 mm: in helical coordinates a point is removed where its helix
// meets the bit, each radius losing the angle its slice of the bit spans along the helix.
unit mm
use std
use std.solids (cut)
use std.place (coords)
use std.motion (screws)

rod_r := 5mm         // the rod: its radius and length
rod_l := 20mm
depth := 1mm         // the bit: how far its point reaches into the rod, its point's half angle,
half := 30deg        // its shank's radius and length
shank := 3mm
reach := 6mm
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
}
rod := solid(std.solids.Revolution(face(rod_axis, rod_end, rod_wall, rod_top), about: rod_axis))

// the bit, drawn in the plane through its axis and the rod's: its axis along x, out from the rod,
// its point `depth` within the rod's wall at the rod's foot
private out := axis
fix(dir == (0, 0, 1)) out
private along := axis
fix(dir == (1, 0, 0)) along
bench := plane(u: out, v: along)
fix(origin == (rod_r - depth, 0, 0)) bench
in bench {
  private tip := point
  private lip := point
  private top := point
  private back := point
  tip coords(du: 0mm, dv: 0mm) bench
  lip coords(du: shank, dv: shank / tan(half)) bench
  top coords(du: shank, dv: reach + shank / tan(half)) bench
  back coords(du: 0mm, dv: reach + shank / tan(half)) bench
  private cone := line(tip, lip)
  private side := line(lip, top)
  private end := line(top, back)
  private spindle := line(back, tip)
}
construction vbit := solid(std.solids.Revolution(face(cone, side, end, spindle), about: spindle))

// the screw about the rod's axis, and the roll that carries the bit from 4 below the rod's foot,
// clear of it, to 4 above its top
chase := frame
chase screws(lead) ax
start := -360deg * 4mm / lead
finish := 360deg * (rod_l + 4mm) / lead
construction thread := solid({ p | some t in (start, finish): p in vbit at(under: chase) t })
thread cut rod
