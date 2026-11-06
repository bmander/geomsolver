// A bar turned partway into a ring sector, stopping inside it: a small upright bar, 1 mm square
// and 1.2 mm tall, 10 mm out from the axis, swung about the axis from well clear of the sector to
// halfway across its 60°. Where it stops is its own faces at the end pose, which no contact walk
// reads. The kernel builds it on one arrangement (the closed class, #721): the ring split at once
// by every face the sweep's boundary can lie on — the bar's faces at both end poses, its edges
// turned (cylinders about the axis and planes square to it), its sides' contact lines turned — and
// each cell kept where the bar, turned back, misses it. Each circle about the axis meets the bar
// in the angles within `a(ρ) = asin(min(½, √(ρ² − 10²)) / ρ)` of its plane, so the turn covers the
// sector's angles from 0 to `a + 30°`: the ring loses `1.2 ∫ ρ (a(ρ) + π/6) dρ` from 10 to
// `√(11² + ½²)`.
unit mm
use std
use std.solids (cut, common)
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

// the bar: x from 10 mm to 11 mm, y within half a millimetre, z within 0.6 mm
in std.front {
  private c0 := point
  private c1 := point
  private c2 := point
  private c3 := point
  c0 coords(du: 10mm, dv: -0.6mm) std.front
  c1 coords(du: 11mm, dv: -0.6mm) std.front
  c2 coords(du: 11mm, dv: 0.6mm) std.front
  c3 coords(du: 10mm, dv: 0.6mm) std.front
  private lower := line(c0, c1)
  private outer := line(c1, c2)
  private upper := line(c2, c3)
  private inner := line(c3, c0)
}
construction bar := solid(std.solids.Prism(face(lower, outer, upper, inner), from: -0.5mm, to: 0.5mm))

// the swing: from 140° short of the sector to 30° into it
swing := frame
swing turns(1) ax
construction notch := solid({ p | some t in (-140deg, 30deg): p in bar at(under: swing) t })
notch cut block
