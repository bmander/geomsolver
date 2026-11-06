// A flat-end mill leaning across a turn, its sharp rims digging past its ends (#780 L2): a drum of
// radius 1 mm and 3 mm long, its ends square to its axis, leaning 25° from upright 5 mm out from
// the turn's axis and 1.2 mm beside the plane through it, swung 100° about the axis wholly inside a
// ring. Its side touches along two curves, each turned into a surface of revolution held exactly;
// its flat ends' contact lines miss them; each rim is a sharp circle whose fan holds a normal square
// to the turn's velocity along an arc of it, and turned it sweeps the ring the circle sweeps, its
// meridian an ellipse in `(ρ² + z², z)`. At each end pose the end's plane meets that ring in its rim
// and a second circle, the side in its rim and a loop crossing it where the side's contact reaches
// the rim. The kernel builds it on one arrangement (the closed class, #721) and keeps each cell the
// mill, turned back, misses. Each level of the mill is convex and clear of the axis, so the turn
// sweeps each slice's area and its arc of every circle carried 100° on: the ring loses the mill's
// own `3π` and `100/360` of `∫ π (ρ_max² − ρ_min²) dz`, the volume it sweeps turned whole.
unit mm
use std
use std.solids
use std.place (coords)
use std.motion (turns)

// the ring: radii 3 and 8, heights ±2.5, whole round the axis
in std.front {
  construction centerline ax := line(std.origin, hint((0, 1)))
  fix((0, 1)) ax.p2
  private q0 := point
  private q1 := point
  private q2 := point
  private q3 := point
  q0 coords(du: 3mm, dv: -2.5mm) std.front
  q1 coords(du: 8mm, dv: -2.5mm) std.front
  q2 coords(du: 8mm, dv: 2.5mm) std.front
  q3 coords(du: 3mm, dv: 2.5mm) std.front
  private wall_in := line(q3, q0)
  private underside := line(q0, q1)
  private wall_out := line(q1, q2)
  private topside := line(q2, q3)
}
block := solid(std.solids.Revolution(face(wall_in, underside, wall_out, topside), about: ax))

// the mill: radius 1 mm and 3 mm long about an axis leaning 25° from upright in the plane
// y = 1.2 mm, its middle at (5, 1.2, 0)
private across := axis
fix(dir == (cos(25deg), 0, -sin(25deg))) across
private up := axis
fix(dir == (sin(25deg), 0, cos(25deg))) up
bench := plane(u: across, v: up)
fix(origin == (5 - 1.5 * sin(25deg), 1.2, -1.5 * cos(25deg))) bench
in bench {
  private e0 := point
  private e1 := point
  private e2 := point
  private e3 := point
  e0 coords(du: 0mm, dv: 0mm) bench
  e1 coords(du: 1mm, dv: 0mm) bench
  e2 coords(du: 1mm, dv: 3mm) bench
  e3 coords(du: 0mm, dv: 3mm) bench
  private bottom := line(e0, e1)
  private side := line(e1, e2)
  private top := line(e2, e3)
  private spindle := line(e3, e0)
}
construction mill := solid(std.solids.Revolution(face(bottom, side, top, spindle), about: spindle))

// the swing: 100° about the axis, wholly inside the ring
swing := frame
swing turns(1) ax
construction channel := solid({ p | some t in (0deg, 100deg): p in mill at(under: swing) t })
channel cut block
