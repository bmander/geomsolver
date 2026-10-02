// Step 5, the members: each blank less one continuous generating sweep of its
// crown at every tooth index.
use std
use design
use layout
use blank.member

component GeneratedMember(design: group, tool: solid, teeth: Int, roll_limit: Angle) {
  // The blank: the heel sphere within the tip cone, less the toe and the back.
  body := solid(design.heel)
  blank := MemberBlank(body, design)

  private construction removal := solid(tool, under: design.generation,
    from: -roll_limit, to: roll_limit)
  repeat teeth as i {
    private construction indexed := solid(removal, under: design.indexing,
      at: i * 360deg / teeth)
    indexed cut body
  }
}

// The tip relief (crown/relief.sv): a second cut on a member's `body`, its own sweep of
// the relief `tool` under the member's generating motion and at every tooth index.
component ReliefCut(body: solid, design: group, tool: solid, teeth: Int, roll_limit: Angle) {
  private construction removal := solid(tool, under: design.generation,
    from: -roll_limit, to: roll_limit)
  repeat teeth as i {
    private construction indexed := solid(removal, under: design.indexing,
      at: i * 360deg / teeth)
    indexed cut body
  }
}

// The pair: each member's blank less its generator, swept at every tooth, and less its
// tip relief and its end relief where the design has them.
component HypoidPair(front: plane, design: group) {
  private reference := HypoidLayout(front, design)
  pinion := GeneratedMember(reference.pinion_design, reference.tooth.crown,
    teeth: design.pinion_teeth, roll_limit: design.pinion_roll)
  // The gear rolls slower against the crown, so its roll is longer to carry the
  // cutter clear of its blank at both limits.
  gear := GeneratedMember(reference.gear_design, reference.gear_space.body,
    teeth: design.gear_teeth, roll_limit: design.gear_roll)
  repeat design.relieved {
    pinion_relief := ReliefCut(pinion.body, reference.pinion_design,
      reference.tooth_relief[0].body, teeth: design.pinion_teeth, roll_limit: design.pinion_roll)
    gear_relief := ReliefCut(gear.body, reference.gear_design,
      reference.space_relief[0].body, teeth: design.gear_teeth, roll_limit: design.gear_roll)
  }
  repeat design.ends_relieved {
    pinion_ends := EndCut(pinion.body, reference.pinion_blank.toe_end[0].ring,
      reference.pinion_blank.heel_end[0].ring)
    gear_ends := EndCut(gear.body, reference.gear_blank.toe_end[0].ring,
      reference.gear_blank.heel_end[0].ring)
  }
}

preview {
  unit mm
  pair := HypoidPair(std.front, hypoid_design)
}
