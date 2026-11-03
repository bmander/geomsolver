// A drum turned partway into a ring sector, stopping inside it: a short round drum, radius 1 mm
// and 1 mm tall, its axis upright 10 mm out from the turn's, swung about the axis from well clear
// of the sector to halfway across its 60°. Where it stops is its own round side at the end pose,
// which no contact walk reads. The kernel builds it on one arrangement (the closed class, #721):
// the ring split at once by every face the sweep's boundary can lie on — the drum at both end
// poses, its side's two lines in the plane through both axes turned (the cylinders of radii 9 and
// 11 about the axis), its flat ends kept in their own planes — and each cell kept where the drum,
// turned back, misses it. Each circle about the axis meets the drum in the angles within
// `a(ρ) = acos((ρ² + 10² − 1)/(20ρ))` of its plane, so the turn covers the sector's angles from 0
// to `a + 30°`: the ring loses `∫ ρ (a(ρ) + π/6) dρ` from 9 to 11, through the drum's 1 mm.
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

// the drum: radius 1 mm and 1 mm tall about an upright axis 10 mm out from the turn's, its
// middle level with the ring's
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
  e1 coords(du: 1mm, dv: 0mm) bench
  e2 coords(du: 1mm, dv: 1mm) bench
  e3 coords(du: 0mm, dv: 1mm) bench
  private bottom := line(e0, e1)
  private side := line(e1, e2)
  private top := line(e2, e3)
  private spindle := line(e3, e0)
}
construction drum := solid(std.solids.Revolution(face(bottom, side, top, spindle), about: spindle))

// the swing: from 140° short of the sector to 30° into it
swing := frame
swing turns(1) ax
construction notch := solid({ p | p inside(under: swing, over t in (-140deg, 30deg)) drum })
notch cut block
