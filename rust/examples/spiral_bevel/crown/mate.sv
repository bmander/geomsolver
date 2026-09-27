// Step 4, the gear's generator: the crown tooth's mate. Each of its sections lies
// on the tooth's own flank lines, one tooth's width outward or inward along the
// pitch line, with its own tip and roundings; it is drawn tip down in N, so it
// walks its edges the other way round. The shift is stated once, on the tooth, and
// the two crowns are complementary by construction.
use std
use design
use views
use pitch.gear
use pitch.trace
use crown.thickness
use crown.section
use crown.rounding
use crown.tooth

// A mate section between pitch points `lp` (inner) and `rp` (outer), its inner flank
// along `inner_along` and its outer flank along `outer_along`, both through its pitch
// points: the tooth's outer and inner flanks, carried a tooth's width.
component MateSection(lp: point, rp: point, inner_along: line, outer_along: line,
                      design: group) {
  // Seeds only, as RackSection's with the flank angles exchanged and the tip down.
  param nm = design.normal_module
  param inner_pressure = design.pressure - design.shift
  param outer_pressure = design.pressure + design.shift
  param th = design.dedendum * nm
  param tr = design.rounding * nm
  param bd = design.base * nm
  param jo = th - tr * (1 - sin(outer_pressure))
  param ji = th - tr * (1 - sin(inner_pressure))
  param ro = jo * tan(outer_pressure) + tr * cos(outer_pressure)
  param ri = ji * tan(inner_pressure) + tr * cos(inner_pressure)
  private point bi hint(x: lp.x - (rp.x - lp.x) / abs(rp.x - lp.x) * bd * tan(inner_pressure),
                        y: lp.y + (rp.x - lp.x) / abs(rp.x - lp.x) * bd)
  private point bo hint(x: rp.x + (rp.x - lp.x) / abs(rp.x - lp.x) * bd * tan(outer_pressure),
                        y: bi.y)
  private point ij hint(x: lp.x + (rp.x - lp.x) / abs(rp.x - lp.x) * ji * tan(inner_pressure),
                        y: lp.y - (rp.x - lp.x) / abs(rp.x - lp.x) * ji)
  private point it hint(x: lp.x + (rp.x - lp.x) / abs(rp.x - lp.x) * ri,
                        y: lp.y - (rp.x - lp.x) / abs(rp.x - lp.x) * th)
  private point ot hint(x: rp.x - (rp.x - lp.x) / abs(rp.x - lp.x) * ro, y: it.y)
  private point oj hint(x: rp.x - (rp.x - lp.x) / abs(rp.x - lp.x) * jo * tan(outer_pressure),
                        y: lp.y - (rp.x - lp.x) / abs(rp.x - lp.x) * jo)
  private point ci hint(x: it.x, y: lp.y - (rp.x - lp.x) / abs(rp.x - lp.x) * (th - tr))
  private point co hint(x: ot.x, y: ci.y)
  construction line pitch(lp, rp)
  profile = line base(bo, bi) -> line inner(bi, ij) -> tangent
            arc inner_round(center: ci) hint(r: tr) -> tangent line tip(it, ot) -> tangent
            arc outer_round(center: co) hint(r: tr) -> tangent line outer(oj, bo) -> close
  inner_along angle(180deg) inner
  outer_along angle(180deg) outer
  lp on inner
  rp on outer
  rounding: TipRounding(pitch, base, tip, inner_round, outer_round, design)
}

// The mate of `tooth` (a CrownTooth), revolved about the cutter's axis turned tip down.
component CrownMate(tooth: group, design: group) {
  point outer_far hint(x: 2 * tooth.rack.pitch.p2.x - tooth.rack.pitch.p1.x,
                       y: tooth.rack.pitch.p2.y)
  point inner_far hint(x: 2 * tooth.rack.pitch.p1.x - tooth.rack.pitch.p2.x,
                       y: tooth.rack.pitch.p1.y)
  point bottom hint(x: tooth.axis.p1.x, y: 2 * tooth.axis.p1.y - tooth.axis.p2.y)
  construction line outer_span(tooth.rack.pitch.p1, outer_far)
  construction line inner_span(inner_far, tooth.rack.pitch.p2)
  tooth.rack.pitch.p2 midpoint outer_span
  tooth.rack.pitch.p1 midpoint inner_span
  outer: MateSection(tooth.rack.pitch.p2, outer_far, tooth.rack.outer, tooth.rack.inner, design)
  inner: MateSection(inner_far, tooth.rack.pitch.p1, tooth.rack.outer, tooth.rack.inner, design)
  line axis(tooth.axis.p1, bottom)
  tooth.axis angle(180deg) axis
  axis equal tooth.axis
  construction solid outer_crown(outer.profile, about: axis)
  construction solid inner_crown(inner.profile, about: axis)
}

preview {
  unit mm
  pitch: PitchView(std.front, span: hypoid_design.cutter_radius)
  gear: GearCone(pitch.view, g.view, hypoid_design)
  g: FoldedView(pitch.view, gear.generator, span: hypoid_design.cutter_radius)
  trace: ToothTrace(pitch.view, gear.generator, hypoid_design)
  thickness: CrownThickness(pitch.view, gear.generator, trace.normal, hypoid_design)
  n: FoldedView(pitch.view, trace.normal, span: hypoid_design.cutter_radius)
  tooth: CrownTooth(pitch.view, trace.normal, thickness.inner_pitch, thickness.outer_pitch,
    hypoid_design) in n.view
  mate: CrownMate(tooth, hypoid_design) in n.view
}
