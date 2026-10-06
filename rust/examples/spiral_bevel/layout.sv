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

// The layout on the pitch plane `p` (`std.top`). The normal module is the trace's: it constructs
// it, and every depth reads it.
component HypoidLayout(p: plane, design: group) {
  gear := pitch.gear.GearCone(p, g.view, design)
  g := views.FoldedView(p, gear.generator)
  trace := pitch.trace.ToothTrace(p, gear.generator, design)
  pinion := pitch.pinion.PinionCone(p, q.view, gear, trace.foot, design)
  q := views.FoldedView(p, pinion.hinge)

  // The blanks, in the axial views; the gear's cones on the generator across its axis.
  gear_blank := blank.member.MemberLimits(gear.pitch_line, gear.opposite, gear.ax, design,
    normal_module: trace.normal_module) in g.view
  pinion_blank := blank.member.MemberLimits(pinion.pitch_line, pinion.pitch_line, pinion.ax,
    design, normal_module: trace.normal_module) in q.view

  // The crown, in the normal section.
  thickness := crown.thickness.CrownThickness(p, gear.generator, trace.normal, design)
  n := views.FoldedView(p, trace.normal)
  tooth := crown.tooth.CrownTooth(p, trace.normal, thickness.inner_pitch,
    thickness.outer_pitch, design, normal_module: trace.normal_module) in n.view
  mate := crown.mate.CrownMate(tooth, design, normal_module: trace.normal_module) in n.view
  reach := crown.reach.CutterReach(p, n.view, gear.O, trace.normal, mate.ax,
    reach: design.space_reach)
  generation := generation.Generation(gear, pinion, thickness)
  construction gear_space := crown.space.ComplementarySpace(mate.ax, mate.outer, mate.inner,
    generation.crown_neighbor, reach.cap) in n.view
  repeat design.relieved {
    construction tooth_relief := crown.relief.ToothRelief(tooth, reach.cap, design,
      normal_module: trace.normal_module) in n.view
    construction space_relief := crown.relief.SpaceRelief(mate.ax, mate.outer, mate.inner,
      generation.crown_neighbor, reach.cap, design, normal_module: trace.normal_module) in n.view
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
  layout := HypoidLayout(std.top, design.hypoid_design)
}
