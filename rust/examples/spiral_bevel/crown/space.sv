// Step 4, the gear's space cutter: the stretch between two neighbouring mate teeth,
// each active flank closed far from the working blank. It is the outer mate within
// the inner one indexed a crown pitch round. The outer section is closed at a cap
// in N standing |CO| + reach from the cutter's axis, clear of every blank point.
use std
use design
use views
use pitch.gear
use pitch.trace
use crown.thickness
use crown.section
use crown.rounding
use crown.tooth
use crown.mate

// The cap: in the pitch plane, the point `reach` past the apex on the ray from C
// through it, carried onto the trace normal beyond C, and its image in N; drawn
// there along the cutter's `axis`.
component CutterReach(p: plane, n: plane, apex: point, normal: line, axis: line, reach: Length) {
  in p {
    point beyond hint(x: apex.x + (apex.x - normal.p1.x) * reach
        / sqrt((apex.x - normal.p1.x)^2 + (apex.y - normal.p1.y)^2),
      y: apex.y + (apex.y - normal.p1.y) * reach
        / sqrt((apex.x - normal.p1.x)^2 + (apex.y - normal.p1.y)^2))
    point radial hint(x: normal.p1.x + (normal.p1.x - normal.p2.x)
        * sqrt((beyond.x - normal.p1.x)^2 + (beyond.y - normal.p1.y)^2)
        / sqrt((normal.p2.x - normal.p1.x)^2 + (normal.p2.y - normal.p1.y)^2),
      y: normal.p1.y + (normal.p1.y - normal.p2.y)
        * sqrt((beyond.x - normal.p1.x)^2 + (beyond.y - normal.p1.y)^2)
        / sqrt((normal.p2.x - normal.p1.x)^2 + (normal.p2.y - normal.p1.y)^2))
    private line to_apex(normal.p1, apex)
    private line past(apex, beyond)
    private line reach_line(normal.p1, beyond)
    private line cap_radius(normal.p1, radial)
  }
  to_apex angle(0deg) past
  distance(reach) past
  normal angle(180deg) cap_radius
  cap_radius equal reach_line
  in n {
    point foot hint(x: (radial.x * (normal.p2.x - normal.p1.x) + radial.y * (normal.p2.y - normal.p1.y))
      / sqrt((normal.p2.x - normal.p1.x)^2 + (normal.p2.y - normal.p1.y)^2), y: 0)
    point end hint(x: foot.x, y: foot.y + axis.p2.y - axis.p1.y)
    line cap(foot, end)
  }
  foot on p
  radial project foot
  axis angle(0deg) cap
  cap equal axis
}

// One active rounded flank, closed at `cap` along its base and tip. The walk enters
// the flank from whichever end meets its round.
component FlankSection(base: line, flank: line, corner: arc, tip: line, cap: line) {
  private point base_end hint(x: cap.p1.x, y: base.p1.y)
  private point tip_end hint(x: cap.p1.x, y: tip.p1.y)
  base_end on base
  base_end on cap
  tip_end on tip
  tip_end on cap
  face profile(base_end, flank, corner, tip_end, -> close)
}

// The space between the outer mate section's inner flank and the inner mate
// section's outer flank, one crown pitch round: the outer crown within its
// indexed neighbour. Drawn in N (`in n`), about the mate's `axis`.
component ComplementarySpace(axis: line, outside: group, inside: group, indexing: motion,
                             cap: line) {
  private outer: FlankSection(outside.base, outside.inner, outside.inner_round, outside.tip, cap)
  private inner: FlankSection(inside.base, inside.outer, inside.outer_round, inside.tip, axis)
  private construction solid outer_crown(outer.profile, about: axis)
  private construction solid inner_crown(inner.profile, about: axis)
  private construction solid neighbor(inner_crown, under: indexing, at: 0deg)
  solid body(outer_crown)
  neighbor bound body
}

preview {
  unit mm
  pitch: PitchView(std.front)
  gear: GearCone(pitch.view, g.view, hypoid_design)
  g: FoldedView(pitch.view, gear.generator)
  trace: ToothTrace(pitch.view, gear.generator, hypoid_design)
  thickness: CrownThickness(pitch.view, gear.generator, trace.normal, hypoid_design)
  n: FoldedView(pitch.view, trace.normal)
  param nm = hypoid_design.module * cos(hypoid_design.spiral)
  tooth: CrownTooth(pitch.view, trace.normal, thickness.inner_pitch, thickness.outer_pitch,
    hypoid_design, normal_module: nm) in n.view
  mate: CrownMate(tooth, hypoid_design, normal_module: nm) in n.view
  reach: CutterReach(pitch.view, n.view, gear.O, trace.normal, mate.axis,
    reach: hypoid_design.space_reach)
  motion crown_neighbor(about: gear.crown_axis,
    phase: -4 * length(thickness.ahead) / radius(thickness.ahead) * 1rad)
  space: ComplementarySpace(mate.axis, mate.outer, mate.inner, crown_neighbor, reach.cap) in n.view
}
