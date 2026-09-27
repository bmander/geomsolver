// Step 5, the members: each blank less one continuous generating sweep of its
// crown at every tooth index.
use std
use design
use layout
use blank.member

component GeneratedMember(design: group, tool: solid, teeth: Int, roll_limit: Angle) {
  // The blank: the heel sphere within the tip cone, less the toe and the back.
  solid body(design.heel)
  blank: MemberBlank(body, design)

  private construction solid removal(tool, under: design.generation,
    from: -roll_limit, to: roll_limit)
  repeat teeth as i {
    private construction solid indexed(removal, under: design.indexing,
      at: i * 360deg / teeth)
    indexed cut body
  }
}

// The pair: each member's blank less its generator, swept at every tooth.
component HypoidPair(front: plane, design: group) {
  private reference: HypoidLayout(front, design)
  pinion: GeneratedMember(reference.pinion_design, reference.tooth.crown,
    teeth: design.pinion_teeth, roll_limit: design.pinion_roll)
  // The gear rolls slower against the crown, so its roll is longer to carry the
  // cutter clear of its blank at both limits.
  gear: GeneratedMember(reference.gear_design, reference.gear_space.body,
    teeth: design.gear_teeth, roll_limit: design.gear_roll)
}

preview {
  unit mm
  pair: HypoidPair(std.front, hypoid_design)
}
