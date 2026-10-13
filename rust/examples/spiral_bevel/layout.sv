// The hypoid layout, steps 2 to 5: the pitch cones and the tooth trace through the mean point M,
// the members' blanks, the crown tooth and its mate as regions about the cutter's axis, and the
// generating motions.
use std
use design
use views
use pitch.gear
use pitch.trace
use pitch.pinion
use blank.member
use crown.thickness
use crown.cutter
use crown.tooth
use crown.mate
use crown.reach
use crown.relief
use generation

// The layout on the pitch plane `p` (`std.top`). The normal module is the trace's: a caller
// leaves `normal_module` unbound, the trace constructs it, and every depth reads it.
component HypoidLayout(p: plane, design: group, normal_module: Length) {
  gear := pitch.gear.GearCone(p, design)
  trace := pitch.trace.ToothTrace(p, gear.generator, design, normal_module: normal_module)
  pinion := pitch.pinion.PinionCone(p, gear, trace.foot, design)

  // The blanks, regions about the members' axes.
  gear_blank := blank.member.MemberBlank(gear.generator, gear.ax, design,
    normal_module: normal_module)
  pinion_blank := blank.member.MemberBlank(pinion.pitch_line, pinion.ax, design,
    normal_module: normal_module)

  // The crown, about the cutter's axis: the pinion's generator, the crown tooth, and the gear's,
  // its space cutter, on the flank cones they share.
  thickness := crown.thickness.CrownThickness(p, gear.generator, trace.normal, design)
  cutter := crown.cutter.Cutter(gear.crown_axis, trace.normal, thickness.inner_pitch,
    thickness.outer_pitch, design, normal_module: normal_module)
  tooth := crown.tooth.Tooth(cutter.inner_flank, cutter.outer_flank, cutter.tooth_base,
    cutter.tooth_tip, grow: design.backlash / 4, r: design.rounding * normal_module)
  reach := crown.reach.CutterReach(p, gear.O, trace.normal, cutter.ax, reach: design.space_reach)
  generation := generation.Generation(gear, pinion, thickness)
  construction gear_space := crown.mate.Space(cutter.outer_flank, cutter.inner_flank,
    cutter.mate_tip, cutter.mate_base, reach.cap, generation.crown_neighbor,
    grow: design.backlash / 4, r: design.rounding * normal_module)
  when design.tip_relief > 0mm {
    // the toppings' kinks, in the normal section N along the trace normal C -> M
    n := views.FoldedView(p, trace.normal)
    construction tooth_relief := crown.relief.ToothRelief(cutter, tooth, n.view,
      thickness.inner_pitch, thickness.outer_pitch, design, normal_module: normal_module)
    construction space_relief := crown.relief.SpaceRelief(cutter, gear_space, reach.cap,
      generation.crown_neighbor, n.view, thickness.inner_pitch, thickness.outer_pitch, design,
      normal_module: normal_module)
  }

  // What each member is cut from and how its cutter moves.
  pinion_design := {blank: pinion_blank.material,
    generation: generation.pinion_generation, indexing: generation.pinion_index}
  gear_design := {blank: gear_blank.material,
    generation: generation.gear_generation, indexing: generation.gear_index}
}

preview {
  unit mm
  layout := HypoidLayout(std.top, design.hypoid_design)
}
