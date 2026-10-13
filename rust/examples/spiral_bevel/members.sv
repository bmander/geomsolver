// Step 5, the members: each blank less one continuous generating sweep of its crown at every
// tooth index, less its tip relief's topping swept the same way where the design has one.
use std
use design
use layout

// A cut generated in `body`: `tool` swept under its member's generating roll from -`roll_limit`
// to `roll_limit` and indexed at every tooth. `design` says how the member's cutter moves.
component Generated(body: solid, design: group, tool: solid, teeth: Int, roll_limit: Angle) {
  construction removal := solid(tool, under: design.generation,
    from: -roll_limit, to: roll_limit)
  repeat teeth as i {
    private construction indexed := solid(removal, under: design.indexing,
      at: i * 360deg / teeth)
    indexed cut body
  }
}

// A member: its blank less `tool`'s generated cut.
component GeneratedMember(design: group, tool: solid, teeth: Int, roll_limit: Angle) {
  body := solid(design.blank)
  cut := Generated(body, design, tool, teeth: teeth, roll_limit: roll_limit)
}

component HypoidPair(p: plane, design: group) {
  private reference := layout.HypoidLayout(p, design)
  pinion := GeneratedMember(reference.pinion_design, reference.tooth.crown,
    teeth: design.pinion_teeth, roll_limit: design.pinion_roll)
  // The gear rolls slower against the crown, so further, to carry the cutter clear of its
  // blank at both limits.
  gear := GeneratedMember(reference.gear_design, reference.gear_space.body,
    teeth: design.gear_teeth, roll_limit: design.gear_roll)
  // The tip relief: each member's topping (layout.sv) cut from it the same way.
  when design.tip_relief > 0mm {
    pinion_relief := Generated(pinion.body, reference.pinion_design,
      reference.pinion_topping.crown, teeth: design.pinion_teeth,
      roll_limit: design.pinion_roll)
    gear_relief := Generated(gear.body, reference.gear_design, reference.gear_topping.body,
      teeth: design.gear_teeth, roll_limit: design.gear_roll)
  }
}

preview {
  unit mm
  pair := HypoidPair(std.top, design.hypoid_design)
}
