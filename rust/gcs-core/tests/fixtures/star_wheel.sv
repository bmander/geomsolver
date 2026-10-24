// #615: the radiator bot's measured star handle, undrafted, swept through a wheel blank; a
// clearance claim between the cut wheel and the star at the drawing's pose.
unit mm
use std
use std (vertical, turns)

interface StarSizes {
  span: Length, waist: Length, h: Length
  tip_r: Length, fillet_r: Length, draft: Angle
  rim_r: Length, edge_r: Length
  stem_d: Length
}
handle_dims := {
  span: 74.2mm, waist: 25.8mm, h: 16.0mm, tip_r: 3.25mm, fillet_r: 4mm, draft: 8deg,
  rim_r: 0.6mm, edge_r: 0.4mm, stem_d: 7mm,
}

component StarOutline(top: plane, s: StarSizes) {
  in top {
    hub := point
    hub coincident top.origin
    ring 4 about hub {
      tip_c := point hint((s.span / 2 - s.tip_r, 0))
      fillet_c := point hint(((s.waist / 2 + s.fillet_r) / sqrt(2),
                              (s.waist / 2 + s.fillet_r) / sqrt(2)))
      apex := point hint((s.span / 2, 0))
      hub distance(s.waist / 2 + s.fillet_r) fillet_c
      line(fillet_c, tip_c) equal line(fillet_c, next.tip_c)
      ccw(hub, tip_c, fillet_c)
      apex coincident(t == s.span / (s.span - 2 * s.tip_r)) line(hub, tip_c)
      p1 := point hint((tip_c.x, s.tip_r))
      p2 := point hint((fillet_c.x - 0.2 * s.fillet_r, fillet_c.y - 0.98 * s.fillet_r))
      p3 := point hint((fillet_c.x - 0.98 * s.fillet_r, fillet_c.y - 0.2 * s.fillet_r))
      p4 := point hint((s.tip_r, tip_c.x))
      tip_half := arc(center: tip_c, start: apex, end: p1)
      side := line(p1, p2)
      fillet := arc(center: fillet_c, start: p3, end: p2)
      next_side := line(p3, p4)
      next_tip_half := arc(center: next.tip_c, start: p4, end: next.apex)
      radius(s.fillet_r) fillet
      side tangent(at: p1) tip_half
      side tangent(at: p2) fillet
      next_side tangent(at: p1) fillet
      next_side tangent(at: p2) next_tip_half
      ccw(p1, p2, tip_c)
      cw(p1, p2, fillet_c)
      ccw(p3, p4, next.tip_c)
      cw(p3, p4, fillet_c)
    }
    hub distance(s.span / 2 - s.tip_r, along: u) tip_c[0]
    hub level(v) tip_c[0]
    outline := face(tip_half[0], side[0], fillet[0], next_side[0], next_tip_half[0],
                    tip_half[1], side[1], fillet[1], next_side[1], next_tip_half[1],
                    tip_half[2], side[2], fillet[2], next_side[2], next_tip_half[2],
                    tip_half[3], side[3], fillet[3], next_side[3], next_tip_half[3])
  }
}

in std.front {
  construction stem := vertical line(std.origin, hint((0, 10)))
  length(10mm) stem
  wc := point hint((-60, 0))
  std.origin distance(-60mm, along: u) wc
  construction spindle := vertical line(wc, hint((-60, 10)))
  length(10mm) spindle
}
top := plane
fix(origin == (0mm, 0mm, -1.1mm)) top
fix(dir == (1, 0, 0)) top.u
fix(dir == (0, 1, 0)) top.v
plan := StarOutline(top, s: handle_dims)
star := solid(plan.outline, from: -16mm, to: 0mm)
in std.top {
  wo := point
  wo coincident wc
  rim := radius(46.1mm) circle(center: wo)
}
handle_turn := frame
handle_turn turns(1) stem
wheel_turn := frame
wheel_turn turns(-4 / 6) spindle
space := sweep(star over t in (0deg, 540deg), under: handle_turn, seen_from: wheel_turn)
wheel := solid(face(rim), from: -15.6mm, to: -0.6mm)
space cut wheel
claim wheel clear(0.1mm) star
