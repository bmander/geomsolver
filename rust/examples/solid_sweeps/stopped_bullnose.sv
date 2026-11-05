// A bull-nose mill turned partway into a ring sector, stopping inside it: `stopped_drum.sv`'s drum,
// radius 1 mm and 1 mm tall, its bottom corner rounded to 0.3 mm, swung about the axis from well
// clear of the sector to halfway across its 60°. The round is a torus about the mill's own axis,
// along the turn's: it touches round its two meridians in the plane through both axes, turned into
// tori about the turn's axis, and round its bottom ring, kept in its plane; at each end pose the
// mill's round touches each meridian's torus round that meridian and crosses it beside it, and
// where the meridian's top meets the side's line three surfaces touch at a point. The kernel builds
// it on one arrangement (the closed class, #721) and keeps each cell the mill, turned back, misses.
// Each level of the mill is a disc of radius `R(z)` about its axis, 10 mm out, so each circle about
// the turn's axis meets it within `a(ρ, z) = acos((ρ² + 10² − R²)/(20ρ))` of its plane, and the turn
// covers the sector's angles from 0 to `a + 30°`: the ring loses `∫∫ ρ (a + π/6) dρ dz`.
unit mm
use std
use std.solids
use std.place (coords)
use std.motion (turns)

// the ring sector: radii 8 and 12, heights ±1, 60° round from x
in std.front {
  construction centerline ax := line(std.origin, hint((0, 1)))
  fix((0, 1)) ax.p2
  private q0 := point
  private q1 := point
  private q2 := point
  private q3 := point
  q0 coords(du: 8mm, dv: -1mm) std.front
  q1 coords(du: 12mm, dv: -1mm) std.front
  q2 coords(du: 12mm, dv: 1mm) std.front
  q3 coords(du: 8mm, dv: 1mm) std.front
  private wall_in := line(q3, q0)
  private underside := line(q0, q1)
  private wall_out := line(q1, q2)
  private topside := line(q2, q3)
}
ring := solid(std.solids.Revolution(face(wall_in, underside, wall_out, topside), about: ax))
in std.top {
  private w0 := point
  private w1 := point
  private w2 := point
  fix((0, 0)) w0
  w1 coords(du: 24mm, dv: 0mm) std.top
  w2 coords(du: 24mm * cos(60deg), dv: 24mm * sin(60deg)) std.top
  private wa := line(w0, w1)
  private wb := line(w1, w2)
  private wc := line(w2, w0)
}
construction wedge := solid(std.solids.Prism(face(wa, wb, wc), from: -2mm, to: 2mm))
wedge common ring
block := solid(ring)

// the mill: radius 1 mm and 1 mm tall about an upright axis 10 mm out from the turn's, its
// middle level with the ring's, its bottom corner rounded to 0.3 mm
private across := axis
fix(dir == (1, 0, 0)) across
private up := axis
fix(dir == (0, 0, 1)) up
bench := plane(u: across, v: up)
fix(origin == (10, 0, -0.5)) bench
in bench {
  private e0 := point
  private e1 := point
  private e2 := point
  private e3 := point
  e0 coords(du: 0mm, dv: 0mm) bench
  e1 coords(du: 0.7mm, dv: 0mm) bench
  e2 coords(du: 1mm, dv: 1mm) bench
  e3 coords(du: 0mm, dv: 1mm) bench
  private k := point
  k coords(du: 0.7mm, dv: 0.3mm) bench
  private e4 := point
  e4 coords(du: 1mm, dv: 0.3mm) bench
  private bottom := line(e0, e1)
  private corner := arc(center: k, start: e1, end: e4) hint(r: 0.3)
  radius(0.3mm) corner
  private side := line(e4, e2)
  private top := line(e2, e3)
  private spindle := line(e3, e0)
}
construction mill := solid(std.solids.Revolution(face(bottom, corner, side, top, spindle), about: spindle))

// the swing: from 140° short of the sector to 30° into it
swing := frame
swing turns(1) ax
construction notch := solid({ p | some t in (-140deg, 30deg): p in mill at(under: swing) t })
notch cut block
