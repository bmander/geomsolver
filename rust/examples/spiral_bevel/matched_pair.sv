use paired_references

component GeneratedMember(design: group, tool: solid, teeth: Int, roll_limit: Angle) {
  // The blank: the heel sphere within the tip cone, less the toe and the back.
  solid body(design.heel)
  design.tip bound body
  design.toe cut body
  design.back cut body

  private construction solid removal(tool, under: design.generation,
    from: -roll_limit, to: roll_limit)
  repeat teeth as i {
    private construction solid indexed(removal, under: design.indexing,
      at: i * 360deg / teeth)
    indexed cut body
  }
}

// Mathematical zero-backlash, 90-degree common-crown pair.
component MatchedPair(front: plane, pinion_teeth: Int, gear_teeth: Int, mean_module: Length,
                      offset_angle: Angle, spiral_angle: Angle, pressure_angle: Angle) {
  private reference: MatchedReferences(front, pinion_teeth: pinion_teeth,
    gear_teeth: gear_teeth, mean_module: mean_module, offset_angle: offset_angle,
    spiral_angle: spiral_angle, pressure_angle: pressure_angle)
  pinion: GeneratedMember(reference.pinion_design, reference.pinion_crown,
    teeth: pinion_teeth, roll_limit: 35deg)
  // The generating roll must carry each cutter clear of its blank at both
  // limits; the gear rolls slower against the crown and still overlaps its
  // blank at 35 degrees, so its roll is declared longer.
  gear: GeneratedMember(reference.gear_design, reference.gear_space.body,
    teeth: gear_teeth, roll_limit: 45deg)
}
