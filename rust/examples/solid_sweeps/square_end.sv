// A bar whose corner runs to its end: `flat_end.sv`'s bar, its section turned 30° in its plane, so
// a corner of it, not a side, stands nearest the axis. Turned across the block, the bar touches
// along that corner's line within the block, round the corner's fan (docs/generating-sweeps.md, the
// constant-twist class, a prism's sides, #684 M), the point of contact running out along the
// corner as the fan's normal turns, until it reaches the bar's far end: a vertex of the prism,
// where the corner meets the end's two rims. Every normal of the cone between the three faces
// meeting there is the bar's, and those square to the turn stand at the vertex, so the contact goes
// on round the vertex onto the rim whose own contact leaves it there, and on along that rim
// (#721 G1). The groove's wall is the corner's line turned, and it meets the rim's sweep in a sharp
// edge along the vertex's circle. The block keeps 46.2641 mm³ of its 83.7758: what the bar's shadow
// covers is removed, each point at radius r and height z where some point of the bar stands, read
// slice by slice.
unit mm
use std
use std.solids (cut, common)
use std.place (coords)
use std.motion (turns)

inner := 8mm         // the block: a ring sector about z, its radii, half height and angle
outer := 12mm
half := 1mm
sector := 60deg
reach := 10mm        // the bar: how far from the axis the middle of its end's inner rim stands,
tilt := 20deg        // how far it leans out from upright, its section turned `twist` in its plane,
twist := 30deg       // its section's depth from the inner side to the outer and its width across,
depth := 3mm         // and how far it stands up from its end
width := 4mm
length := 3mm

// the bar, drawn in the plane of its end, which faces down `tilt` toward the axis
private across := axis
fix(dir == (cos(tilt), 0, -sin(tilt))) across
private along := axis
fix(dir == (0, 1, 0)) along
holder := plane(u: across, v: along)
fix(origin == (reach, 0, 0)) holder
in holder {
  private b0 := point
  private b1 := point
  private b2 := point
  private b3 := point
  b0 coords(du: width / 2 * sin(twist), dv: -width / 2 * cos(twist)) holder
  b1 coords(du: depth * cos(twist) + width / 2 * sin(twist), dv: depth * sin(twist) - width / 2 * cos(twist)) holder
  b2 coords(du: depth * cos(twist) - width / 2 * sin(twist), dv: depth * sin(twist) + width / 2 * cos(twist)) holder
  b3 coords(du: -width / 2 * sin(twist), dv: width / 2 * cos(twist)) holder
  private lower := line(b0, b1)
  private outer_side := line(b1, b2)
  private upper := line(b2, b3)
  private inner_side := line(b3, b0)
}
construction bar := solid(std.solids.Prism(face(lower, outer_side, upper, inner_side), from: 0mm, to: length))

// the block: a ring about z in common with a wedge `sector` round from x
in std.front {
  construction centerline ax := line(std.origin, hint((0, 1)))
  fix((0, 1)) ax.p2
  private q0 := point
  private q1 := point
  private q2 := point
  private q3 := point
  q0 coords(du: inner, dv: -half) std.front
  q1 coords(du: outer, dv: -half) std.front
  q2 coords(du: outer, dv: half) std.front
  q3 coords(du: inner, dv: half) std.front
  private wall_in := line(q3, q0)
  private underside := line(q0, q1)
  private wall_out := line(q1, q2)
  private topside := line(q2, q3)
}
block := solid(std.solids.Revolution(face(wall_in, underside, wall_out, topside), about: ax))
in std.top {
  private w0 := point
  private w1 := point
  private w2 := point
  fix((0, 0)) w0
  w1 coords(du: 2 * outer, dv: 0mm) std.top
  w2 coords(du: 2 * outer * cos(sector), dv: 2 * outer * sin(sector)) std.top
  private wa := line(w0, w1)
  private wb := line(w1, w2)
  private wc := line(w2, w0)
}
construction wedge := solid(std.solids.Prism(face(wa, wb, wc), from: -2 * half, to: 2 * half))
wedge common block

// the turn about the block's axis, and the floor and wall it sweeps
swing := frame
swing turns(1) ax
construction groove := solid({ p | some t in (-140deg, 140deg): p in bar at(under: swing) t })
groove cut block
