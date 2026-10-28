// A wheel slid along a rod: a torus whose rim reaches into the rod, slid up it from clear below to
// clear above. A slide moves every point of the wheel the same way at every time, so the groove's
// wall is where the wheel's characteristic (the points of it moving along its own surface) is
// carried: here, square to the slide, the wheel's tube's own circle, extruded up the rod (the
// constant-twist class, #684). The groove's section is the lens the rod's disc and the tube's share,
// carried the rod's length: 77.04 mm³ taken from 1570.80. Tilt the wheel (`tilt`) and the wall is
// no circle of it, but still built exactly.
unit mm
use std
use std.place (coords)
use std.motion (slides)

rod_r := 5mm         // the rod: its radius and length
rod_l := 20mm
ring := 6mm          // the wheel: from its axis to its tube's centre, and the tube's radius
tube := 2mm
reach := 5.5mm       // how far from the rod's axis the tube's centre runs
tilt := 0deg         // how far the wheel's axis tilts up from square to the rod

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
rod := solid(face(rod_axis, rod_end, rod_wall, rod_top), about: rod_axis)

// the wheel, drawn in a plane through its axis, which runs across the rod below it
private across := axis
fix(dir == (1, 0, 0)) across
private along := axis
fix(dir == (0, cos(tilt), sin(tilt))) along
bench := plane(u: across, v: along)
fix(origin == (0, 0, -9)) bench
in bench {
  private a0 := point
  private a1 := point
  a0 coords(du: reach + ring, dv: -1mm) bench
  a1 coords(du: reach + ring, dv: 1mm) bench
  private wheel_axis := line(a0, a1)
  private tc := point
  tc coords(du: reach, dv: 0mm) bench
  private rim := circle(center: tc) hint(r: 2)
  radius(tube) rim
}
construction wheel := solid(face(rim), about: wheel_axis)

// the slide up the rod's axis, 40 mm a turn of the roll, and the groove it leaves
feed := frame
feed slides(40mm) ax
construction groove := sweep(wheel over t in (0deg, 360deg), under: feed)
groove cut rod
