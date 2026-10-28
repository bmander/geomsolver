// A wheel screwed along a rod: the torus of `slide.sv` carried along a helix about the rod's axis,
// `lead` a turn, from clear below the rod to clear above, cutting a helical groove. A screw moves
// every point of the wheel the same way at every time, as a slide does, so the groove's wall is
// the wheel's characteristic carried along the helix — read once on the wheel, never traced (the
// constant-twist class, #64). Its section square to the axis is the same at every height, turned:
// the groove takes that section's area from every millimetre of the rod.
unit mm
use std
use std.place (coords)
use std.motion (screws)

rod_r := 5mm         // the rod: its radius and length
rod_l := 20mm
ring := 6mm          // the wheel: from its axis to its tube's centre, and the tube's radius
tube := 2mm
reach := 5.5mm       // how far from the rod's axis the tube's centre runs
lead := 30mm         // how far the screw climbs a turn
// the wheel's axis leans from square to the rod by the helix's angle where the tube's centre runs,
// so the wheel lies along the groove, as a grinding wheel is set (leaned the other way it crosses
// the groove, its characteristic turns back on its rings in the rod, and it is refused, S1)
helix := atan(2 * pi * reach / lead)

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

// the wheel, drawn in a plane through its axis, which leans across the rod at its foot
private across := axis
fix(dir == (1, 0, 0)) across
private along := axis
fix(dir == (0, cos(helix), -sin(helix))) along
bench := plane(u: across, v: along)
fix(origin == (0, 0, 0)) bench
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

// the screw about the rod's axis, and the roll that carries the wheel from below the rod, its
// whole size (`ring + tube`) and a millimetre under the foot, to as far above the top
grind := frame
grind screws(lead) ax
start := -360deg * (ring + tube + 1mm) / lead
finish := 360deg * (rod_l + ring + tube + 1mm) / lead
construction groove := sweep(wheel over t in (start, finish), under: grind)
groove cut rod
