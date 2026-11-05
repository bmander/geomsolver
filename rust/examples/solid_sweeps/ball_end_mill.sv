// A ball-end mill screwed across a ring sector: an end mill whose end is a half ball, radius 1 mm,
// its side 3 mm tall above the ball's centre, its axis upright 10 mm out from the screw's, swung
// about the axis from well clear of the sector to well past it while it rises 3 mm a turn. The
// ball sweeps the pipe about its centre's helix, which touches the cylinder the side screws into
// along the helix of the ball's outer equator. The kernel builds it on one arrangement (the closed
// class, #721, #780 C1): the ring split at once by the mill at both end poses, the side's two lines
// in the plane through both axes screwed into the cylinders of radii 9 and 11 about the axis, and
// the ball's great circle square to its centre's velocity screwed into the pipe — and each cell
// kept where the mill, screwed back, misses it. In helical coordinates `(ρ, ψ, ζ = z − kψ)`,
// `k = 3/2π`, the screw is a shift in `ψ`, so a point of the ring is removed where the helix through
// it meets the mill: its side's where `ζ` is within `[−k a, 3 + k a]`, `a(ρ) = acos((ρ² + 10² −
// 1)/(20ρ))`, and its ball's where `|ζ| ≤ M(ρ)`, the most `kφ + √(20ρ (cos φ − cos a))` reaches over
// `|φ| ≤ a`. Each circle of the sector from its floor up to where its helix leaves the ring is
// removed, and the ring loses `π/3 (40 + ∫ ρ M dρ)` from 9 to 11: 58.356 mm³.
unit mm
use std
use std.solids
use std.place (coords)
use std.motion (screws)

lead := 3mm          // how far the swing climbs a turn

// the ring sector: radii 8 and 12, heights ±2, 30° either side of x
in std.front {
  construction centerline ax := line(std.origin, hint((0, 1)))
  fix((0, 1)) ax.p2
  private q0 := point
  private q1 := point
  private q2 := point
  private q3 := point
  q0 coords(du: 8mm, dv: -2mm) std.front
  q1 coords(du: 12mm, dv: -2mm) std.front
  q2 coords(du: 12mm, dv: 2mm) std.front
  q3 coords(du: 8mm, dv: 2mm) std.front
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
  w1 coords(du: 24mm * cos(30deg), dv: -24mm * sin(30deg)) std.top
  w2 coords(du: 24mm * cos(30deg), dv: 24mm * sin(30deg)) std.top
  private wa := line(w0, w1)
  private wb := line(w1, w2)
  private wc := line(w2, w0)
}
construction wedge := solid(std.solids.Prism(face(wa, wb, wc), from: -3mm, to: 3mm))
wedge common ring
block := solid(ring)

// the mill: a half ball of radius 1 mm about a point 10 mm out from the screw's axis, level with
// the ring's middle, under a side 3 mm tall, drawn as its half section turned about its axis
private across := axis
fix(dir == (1, 0, 0)) across
private up := axis
fix(dir == (0, 0, 1)) up
bench := plane(u: across, v: up)
fix(origin == (10, 0, 0)) bench
in bench {
  private centre := point
  private tip := point
  private rim := point
  private shoulder := point
  private crown := point
  centre coords(du: 0mm, dv: 0mm) bench
  tip coords(du: 0mm, dv: -1mm) bench
  rim coords(du: 1mm, dv: 0mm) bench
  shoulder coords(du: 1mm, dv: 3mm) bench
  crown coords(du: 0mm, dv: 3mm) bench
  private end := arc(center: centre, start: tip, end: rim) hint(r: 1)
  radius(1mm) end
  private side := line(rim, shoulder)
  private top := line(shoulder, crown)
  private spindle := line(crown, tip)
}
construction mill := solid(std.solids.Revolution(face(end, side, top, spindle), about: spindle))

// the swing: from 60° short of the sector's middle to 60° past it, `lead` a turn: the mill, 6°
// either side of its axis, passes wholly across the sector's 60°
swing := frame
swing screws(lead) ax
construction groove := solid({ p | some t in (-60deg, 60deg): p in mill at(under: swing) t })
groove cut block
