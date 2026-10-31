// A V wheel turned once round a rod, as a rod turns in a lathe against a form tool: the wheel's
// sharp 60° rim dips `tip` from the rod's axis, its own axis leaning `lean` from upright across the
// turn, and the turn carries it a whole turn about the rod's axis, starting and ending inside the
// rod. A turn's sweep depends only on its path, so a whole turn sweeps a solid of revolution: the
// groove's wall is the wheel's characteristic (each flank's, joined round the rim's fan) laid in a
// half-plane through the axis and revolved all the way round, closing on itself (the
// constant-twist class, a whole turn, #684 N). No end of the roll need be clear of the rod: the
// wheel at either lies within what the turn sweeps, and a roll of more than a turn sweeps the
// same. The groove takes the wheel's shadow in a half-plane through the axis, turned once round
// (Pappus).
unit mm
use std
use std.place (coords)
use std.motion (turns)

rod_r := 5mm         // the rod: its radius and length
rod_l := 20mm
wheel := 10mm        // the wheel: from its axis to its rim
tip := 4mm           // how far from the rod's axis the rim dips
depth := 2.5mm       // how deep the V is, from the rim to the wheel's bore
flank := 30deg       // each flank's angle from the wheel's plane
lean := 20deg        // how far the wheel's axis leans from upright, across the turn: upright, both
                     // flanks meet the rim at one point, the wall creased along its circle (#684 V)

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

// the wheel, drawn in a plane through its axis at the rod's middle height, the axis leaning across
// the turn: the V's tip toward the rod, its bore a cylinder `depth` behind it
private across := axis
fix(dir == (1, 0, 0)) across
private along := axis
fix(dir == (0, sin(lean), cos(lean))) along
bench := plane(u: across, v: along)
fix(origin == (0, 0, 10)) bench
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

// one whole turn about the rod's axis, from where the wheel is drawn
lathe := frame
lathe turns(1) ax
construction groove := sweep(vwheel over t in (0deg, 360deg), under: lathe)
groove cut rod
