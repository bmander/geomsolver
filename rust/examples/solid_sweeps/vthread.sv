// A V wheel screwed along a rod: a ring whose rim is a sharp 60° V, carried along a helix about the
// rod's axis, `lead` a turn, from clear below the rod to clear above, cutting a helical V groove,
// as a thread is ground. The wheel's two flanks meet at its rim in a convex corner, and each
// flank's characteristic meets the rim at its own point, so the groove's wall is the flanks'
// characteristics joined round the rim between them, where every normal of the corner's fan is
// the wheel's: the rim's own sweep, tangent to both (the constant-twist class, a screw with sharp
// edges, #684 E). The groove's section square to the axis is the same at every height, turned:
// the groove takes that section's area from every millimetre of the rod.
unit mm
use std
use std.place (coords)
use std.motion (screws)

rod_r := 5mm         // the rod: its radius and length
rod_l := 20mm
wheel := 10mm        // the wheel: from its axis to its rim
tip := 4mm           // how far from the rod's axis the rim runs
depth := 2.5mm       // how deep the V is, from the rim to the wheel's bore
flank := 30deg       // each flank's angle from the wheel's plane
lead := 30mm         // how far the screw climbs a turn
// the wheel's axis leans from square to the rod by the helix's angle half a millimetre inside the
// rod, so the wheel lies along the groove, as a grinding wheel is set
helix := atan(2 * pi * (tip + 0.5mm) / lead)

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

// the wheel, drawn in a plane through its axis, which leans across the rod at its foot: the V's
// tip toward the rod, its bore a cylinder `depth` behind it
private across := axis
fix(dir == (1, 0, 0)) across
private along := axis
fix(dir == (0, cos(helix), -sin(helix))) along
bench := plane(u: across, v: along)
fix(origin == (0, 0, 0)) bench
in bench {
  private a0 := point
  private a1 := point
  a0 coords(du: tip + wheel, dv: -1mm) bench
  a1 coords(du: tip + wheel, dv: 1mm) bench
  private wheel_axis := line(a0, a1)
  private v0 := point
  private v1 := point
  private v2 := point
  v0 coords(du: tip, dv: 0mm) bench
  v1 coords(du: tip + depth, dv: -depth * tan(flank)) bench
  v2 coords(du: tip + depth, dv: depth * tan(flank)) bench
  private lower := line(v0, v1)
  private bore := line(v1, v2)
  private upper := line(v2, v0)
}
construction vwheel := solid(face(lower, bore, upper), about: wheel_axis)

// the screw about the rod's axis, and the roll that carries the wheel from below the rod, its
// whole size and a millimetre under the foot, to as far above the top
grind := frame
grind screws(lead) ax
start := -360deg * (wheel + 2mm) / lead
finish := 360deg * (rod_l + wheel + 2mm) / lead
construction groove := sweep(vwheel over t in (start, finish), under: grind)
groove cut rod
