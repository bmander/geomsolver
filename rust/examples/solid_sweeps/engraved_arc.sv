// An arc engraved with a pointed tool: a V-bit, a cone ground to a 60° point on a round shank,
// leaning off upright across the turn, its point half a millimetre into a curved block's top,
// turned about the block's axis from clear of one end to clear of the other. A turn moves every
// point of the bit the same way at every time, so the groove's wall is the bit's characteristic
// laid in a half-plane through the axis and revolved. On each ring of the cone that characteristic
// is two points, and they meet at the point, where the cone meets its own axis: every normal of the
// cone round the apex is the bit's there, and the walk turns at it from one to the other. The wall
// is two surfaces of revolution meeting in a sharp edge along the circle the apex turns on, the
// groove's root (docs/generating-sweeps.md, the constant-twist class, an apex on the tool's axis,
// #721 G5). The bit passes wholly across the block, so each circle about the axis that meets it
// loses the block's whole angle: the block loses its 60° of the bit's shadow in a half-plane
// through the axis turned, `π/6 ∫ (ρ_max² − ρ_min²) dz` over the heights the point reaches.
unit mm
use std
use std.solids (cut, common)
use std.place (coords)
use std.motion (turns)

inner := 8mm         // the block: a ring sector about z, its radii, half height and angle
outer := 12mm
half := 1mm
sector := 60deg
reach := 10mm        // the bit: how far its point stands from the block's axis, how deep it
depth := 0.5mm       // reaches into the block's top, its point's half angle, its shank's radius,
point := 30deg       // and how far its axis leans from upright, across the turn
shank := 1.5mm
lean := 25deg
from := -140deg      // the roll, from clear of the block to clear of it
to := 140deg

// the bit: a cone ground to a 60° point on a round shank, drawn in a plane through its axis,
// which leans `lean` across the turn, its point `depth` into the block's top at `reach` from its axis
private across := axis
fix(dir == (1, 0, 0)) across
private along := axis
fix(dir == (0, sin(lean), cos(lean))) along
holder := plane(u: across, v: along)
fix(origin == (reach, 0, half - depth)) holder
in holder {
  private tip := point
  private lip := point
  private top := point
  private back := point
  tip coords(du: 0mm, dv: 0mm) holder
  lip coords(du: shank, dv: shank / tan(point)) holder
  top coords(du: shank, dv: 6mm) holder
  back coords(du: 0mm, dv: 6mm) holder
  private cone := line(tip, lip)
  private side := line(lip, top)
  private end := line(top, back)
  private spindle := line(back, tip)
}
construction bit := solid(std.solids.Revolution(face(cone, side, end, spindle), about: spindle))

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

// the turn about the block's axis, and the groove it sweeps
swing := frame
swing turns(1) ax
construction groove := solid({ p | some t in (from, to): p in bit at(under: swing) t })
groove cut block
