// The hypoid layout, step by step (docs/spiral-bevel-layout-plan.md): the pitch
// cones and the tooth trace through the mean point M, the members' blanks, the
// crown tooth and its mate in the normal section, and the generating motions.
// Every view is folded square to the pitch plane about a line through M.
use std
use design
use views
use pitch.gear
use pitch.trace
use pitch.pinion
use blank.member
use crown.thickness
use crown.tooth
use crown.mate
use crown.space
use generation

component HypoidLayout(front: plane, design: group) {
  pitch: PitchView(front, span: design.cutter_radius)
  gear: GearCone(pitch.view, g.view, design)
  g: FoldedView(pitch.view, gear.generator, span: design.cutter_radius)
  trace: ToothTrace(pitch.view, gear.generator, design)
  pinion: PinionCone(pitch.view, q.view, gear.generator, trace.foot, gear.axis, design)
  q: FoldedView(pitch.view, pinion.hinge, span: design.cutter_radius)
  // The pinion's virtual axis stands at the angle at M in the gear's triangle, the
  // bevel pinion's pitch angle.
  gear.to_foot angle(pinion_angle) gear.to_apex
  pinion.virtual_line angle(pinion_angle) pinion.virtual_axis
  // The blanks, in the axial views; the gear's cones on the generator opposite M.
  gear_blank: MemberLimits(gear.pitch_line, gear.back_cone, gear.axis, design) in g.view
  pinion_blank: MemberLimits(pinion.pitch_line, pinion.pitch_line, pinion.axis, design) in q.view
  // The crown, in the normal section.
  thickness: CrownThickness(pitch.view, gear.generator, trace.normal, design)
  n: FoldedView(pitch.view, trace.normal, span: design.cutter_radius)
  tooth: CrownTooth(pitch.view, trace.normal, thickness.inner_pitch, thickness.outer_pitch,
    design) in n.view
  mate: CrownMate(tooth, design) in n.view
  reach: CutterReach(pitch.view, n.view, gear.O, trace.normal, mate.axis,
    reach: design.space_reach)
  generation: Generation(gear, pinion, thickness)
  construction gear_space: ComplementarySpace(mate.axis, mate.outer, mate.inner,
    generation.crown_neighbor, reach.cap) in n.view
  group pinion_design(heel: pinion_blank.heel.wall.solid, toe: pinion_blank.toe.wall.solid,
    tip: pinion_blank.tip.wall.solid, root: pinion_blank.root.wall.solid,
    back: pinion_blank.back.wall.solid,
    generation: generation.pinion_generation, indexing: generation.pinion_index)
  group gear_design(heel: gear_blank.heel.wall.solid, toe: gear_blank.toe.wall.solid,
    tip: gear_blank.tip.wall.solid, root: gear_blank.root.wall.solid,
    back: gear_blank.back.wall.solid,
    generation: generation.gear_generation, indexing: generation.gear_index)
}

preview {
  unit mm
  layout: HypoidLayout(std.front, hypoid_design)
}
