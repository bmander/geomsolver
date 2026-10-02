// Step 1, the requirements: the configuration and the proportions that follow from it. The
// mean cone distance sizes only the cutter, the face width and the gear space cutter's reach;
// every position in the layout follows from constraints. Depths are in normal modules, the
// module seen square to the tooth trace, which nothing states: the trace constructs it
// (pitch/trace.sv).
use configuration

// The crown's tooth count, 2R / m.
crown_teeth := sqrt(configuration.pinion_teeth^2 + configuration.gear_teeth^2)
mean_cone_distance := configuration.mean_module * crown_teeth / 2

hypoid_design := group(
  pinion_teeth: configuration.pinion_teeth, gear_teeth: configuration.gear_teeth,
  crown_teeth: crown_teeth, module: configuration.mean_module,
  cone_distance: mean_cone_distance,
  shaft: configuration.shaft_angle, offset: configuration.offset,
  spiral: configuration.spiral_angle, pressure: 20deg, shift: configuration.pressure_shift,
  cutter_radius: 0.8 * mean_cone_distance, face_width: 0.2 * mean_cone_distance,
  // depths, in normal modules
  addendum: 1, dedendum: 1.25, base: 2, rounding: 0.3, back: 4,
  // how far each member rolls, clear of its blank at both limits, and the space cutter reaches
  pinion_roll: 35deg, gear_roll: 45deg,
  space_reach: 1.1 * mean_cone_distance + configuration.mean_module,
  // the allowances, each with a switch, 1 where it is not zero
  backlash: configuration.backlash, lashed: min(ceil(configuration.backlash / 1mm), 1),
  tip_relief: configuration.tip_relief, relief_angle: 30deg,
  relieved: min(ceil(configuration.tip_relief / 1mm), 1),
  end_relief: configuration.end_relief,
  ends_relieved: min(ceil(configuration.end_relief / 1mm), 1))

preview {
  unit mm
}
