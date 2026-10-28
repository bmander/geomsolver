// A wheel of two beads slid along a rod: two tori on one axis, overlapping, united into one cutter
// and slid up the rod from clear below to clear above, as `slide.sv`'s wheel is. Where the beads
// meet, the cutter's profile turns inward, a crease no one tool's characteristic runs over; but a
// sweep of a union is the union of its members' sweeps, under the same motion over the same roll,
// so the groove is each bead's groove, each the slide's own and built exactly (#684 C). Square to
// the slide each bead's shadow is its tube's circle, and the groove's section is the part of the
// rod's disc either shadow covers (the beads' two lenses less what they share), carried the rod's
// length: 84.35 mm³ taken from 1570.80.
unit mm
use std
use std.place (coords)
use std.motion (slides)

rod_r := 5mm         // the rod: its radius and length
rod_l := 20mm
ring := 6mm          // the beads: from the wheel's axis to each tube's centre
tube := 2mm          // the first bead's tube, and the second's, its centre along the axis by `step`
tube2 := 1.5mm
step := 1.5mm
reach := 5.5mm       // how far from the rod's axis the tubes' centres run

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
fix(dir == (0, 1, 0)) along
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
  private tc2 := point
  tc2 coords(du: reach, dv: step) bench
  private rim2 := circle(center: tc2) hint(r: 1.5)
  radius(tube2) rim2
}
construction bead := solid(face(rim), about: wheel_axis)
construction bead2 := solid(face(rim2), about: wheel_axis)
construction wheel := solid(bead)
bead2 union wheel

// the slide up the rod's axis, 40 mm a turn of the roll, and the groove it leaves
feed := frame
feed slides(40mm) ax
construction groove := sweep(wheel over t in (0deg, 360deg), under: feed)
groove cut rod
