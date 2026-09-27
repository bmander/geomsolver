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

// `normal_module` is left unbound by the caller: the trace measures it, and every
// depth reads it.
component HypoidLayout(front: plane, design: group, normal_module: Length) {
  pitch: PitchView(front)
  gear: GearCone(pitch.view, g.view, design)
  g: FoldedView(pitch.view, gear.generator)
  trace: ToothTrace(pitch.view, gear.generator, design, normal_module: normal_module)
  pinion: PinionCone(pitch.view, q.view, gear.generator, trace.foot, design)
  q: FoldedView(pitch.view, pinion.hinge)
  // The pinion's pitch angle is the angle at M in the gear's triangle.
  gear.to_apex angle(pinion_angle, sense: cw) gear.to_foot
  pinion.pitch_line angle(pinion_angle) pinion.axis
  // The blanks, in the axial views; the gear's cones on the generator opposite M.
  gear_blank: MemberLimits(gear.pitch_line, gear.back_cone, gear.axis, design,
    normal_module: normal_module, outward: left, inward: right) in g.view
  pinion_blank: MemberLimits(pinion.pitch_line, pinion.pitch_line, pinion.axis, design,
    normal_module: normal_module, outward: right, inward: left) in q.view
  // The crown, in the normal section.
  thickness: CrownThickness(pitch.view, gear.generator, trace.normal, design)
  n: FoldedView(pitch.view, trace.normal)
  tooth: CrownTooth(pitch.view, trace.normal, thickness.inner_pitch, thickness.outer_pitch,
    design, normal_module: normal_module) in n.view
  mate: CrownMate(tooth, design, normal_module: normal_module) in n.view
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
