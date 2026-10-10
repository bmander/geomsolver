// Step 5, the members: each blank less one continuous generating sweep of its crown at every
// tooth index, less its tip and end reliefs where the design has them.
use std
use design
use layout
use blank.ends

// A member: its blank less `tool` swept under its generating roll and indexed at every tooth.
component GeneratedMember(design: group, tool: solid, teeth: Int, roll_limit: Angle) {
  body := solid(design.blank)
  private construction removal := solid(tool, under: design.generation,
    from: -roll_limit, to: roll_limit)
  repeat teeth as i {
    private construction indexed := solid(removal, under: design.indexing,
      at: i * 360deg / teeth)
    indexed cut body
  }
}

// The tip relief (crown/relief.sv): a second `tool` cut from a member's `body` the same way.
component ReliefCut(body: solid, design: group, tool: solid, teeth: Int, roll_limit: Angle) {
  private construction removal := solid(tool, under: design.generation,
    from: -roll_limit, to: roll_limit)
  repeat teeth as i {
    private construction indexed := solid(removal, under: design.indexing,
      at: i * 360deg / teeth)
    indexed cut body
  }
}

component HypoidPair(p: plane, design: group) {
  private reference := layout.HypoidLayout(p, design)
  pinion := GeneratedMember(reference.pinion_design, reference.tooth.crown,
    teeth: design.pinion_teeth, roll_limit: design.pinion_roll)
  // The gear rolls slower against the crown, so further, to carry the cutter clear of its
  // blank at both limits.
  gear := GeneratedMember(reference.gear_design, reference.gear_space.body,
    teeth: design.gear_teeth, roll_limit: design.gear_roll)
  repeat design.relieved {
    pinion_relief := ReliefCut(pinion.body, reference.pinion_design,
      reference.tooth_relief[0].body, teeth: design.pinion_teeth, roll_limit: design.pinion_roll)
    gear_relief := ReliefCut(gear.body, reference.gear_design,
      reference.space_relief[0].body, teeth: design.gear_teeth, roll_limit: design.gear_roll)
  }
  repeat design.ends_relieved {
    pinion_ends := blank.ends.EndCut(pinion.body, reference.pinion_blank.toe_end[0].ring,
      reference.pinion_blank.heel_end[0].ring)
    gear_ends := blank.ends.EndCut(gear.body, reference.gear_blank.toe_end[0].ring,
      reference.gear_blank.heel_end[0].ring)
  }
}

preview {
  unit mm
  pair := HypoidPair(std.top, design.hypoid_design)
}
