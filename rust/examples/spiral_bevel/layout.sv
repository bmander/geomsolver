// The hypoid layout, steps 2 to 5 (docs/spiral-bevel-layout-plan.md): the pitch cones and the
// tooth trace through the mean point M, the members' blanks in their axial views, the crown
// tooth and its mate in the normal section, and the generating motions.
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
use crown.reach
use crown.space
use crown.relief
use generation

// A caller leaves `normal_module` unbound: the trace constructs it, and every depth reads it.
component HypoidLayout(front: plane, design: group, normal_module: Length) {
  pitch := views.PitchView(front, span: design.cutter_radius)
  gear := pitch.gear.GearCone(pitch.view, g.view, design)
  g := views.FoldedView(pitch.view, gear.generator, span: design.cutter_radius)
  trace := pitch.trace.ToothTrace(pitch.view, gear.generator, design,
    normal_module: normal_module)
  pinion := pitch.pinion.PinionCone(pitch.view, q.view, gear, trace.foot, design)
  q := views.FoldedView(pitch.view, pinion.hinge, span: design.cutter_radius)

  // The blanks, in the axial views; the gear's cones on the generator across its axis.
  gear_blank := blank.member.MemberLimits(gear.pitch_line, gear.opposite, gear.axis, design,
    normal_module: normal_module) in g.view
  pinion_blank := blank.member.MemberLimits(pinion.pitch_line, pinion.pitch_line, pinion.axis,
    design, normal_module: normal_module) in q.view

  // The crown, in the normal section.
  thickness := crown.thickness.CrownThickness(pitch.view, gear.generator, trace.normal, design)
  n := views.FoldedView(pitch.view, trace.normal, span: design.cutter_radius)
  tooth := crown.tooth.CrownTooth(pitch.view, trace.normal, thickness.inner_pitch,
    thickness.outer_pitch, design, normal_module: normal_module) in n.view
  mate := crown.mate.CrownMate(tooth, design, normal_module: normal_module) in n.view
  reach := crown.reach.CutterReach(pitch.view, n.view, gear.O, trace.normal, mate.axis,
    reach: design.space_reach)
  generation := generation.Generation(gear, pinion, thickness)
  construction gear_space := crown.space.ComplementarySpace(mate.axis, mate.outer, mate.inner,
    generation.crown_neighbor, reach.cap) in n.view
  repeat design.relieved {
    construction tooth_relief := crown.relief.ToothRelief(tooth, reach.cap, design,
      normal_module: normal_module) in n.view
    construction space_relief := crown.relief.SpaceRelief(mate.axis, mate.outer, mate.inner,
      generation.crown_neighbor, reach.cap, design, normal_module: normal_module) in n.view
  }

  // What each member is cut from and how its cutter moves.
  pinion_design := {heel: pinion_blank.heel.wall.solid, toe: pinion_blank.toe.wall.solid,
    tip: pinion_blank.tip.wall.solid, root: pinion_blank.root.wall.solid,
    back: pinion_blank.back.wall.solid,
    generation: generation.pinion_generation, indexing: generation.pinion_index}
  gear_design := {heel: gear_blank.heel.wall.solid, toe: gear_blank.toe.wall.solid,
    tip: gear_blank.tip.wall.solid, root: gear_blank.root.wall.solid,
    back: gear_blank.back.wall.solid,
    generation: generation.gear_generation, indexing: generation.gear_index}
}

preview {
  unit mm
  layout := HypoidLayout(std.front, design.hypoid_design)
}
