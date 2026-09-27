// Step 1: the requirements, as numbers a designer states. Every position in the
// layout follows from constraints over these. The mean cone distance is arithmetic
// on the tooth counts, and only sizes the cutter, the face width and the gear space
// cutter's reach (the heel sphere and a module) in proportion to the pair. The
// normal module, the module seen square to the tooth trace, is the one number with a
// cosine in it: the depths are stated in it, and the trace checks it (pitch/trace.sv).
use configuration

param mean_cone_distance = mean_module * sqrt(pinion_teeth^2 + gear_teeth^2) / 2

// Depths are in normal modules.
group hypoid_design(pinion_teeth: pinion_teeth, gear_teeth: gear_teeth,
  module: mean_module, normal_module: mean_module * cos(spiral_angle),
  shaft: shaft_angle, offset: offset, spiral: spiral_angle,
  pressure: 20deg, shift: pressure_shift,
  cutter_radius: 0.8 * mean_cone_distance, face_width: 0.2 * mean_cone_distance,
  addendum: 1, dedendum: 1.25, base: 2, rounding: 0.3, back: 4,
  pinion_roll: 35deg, gear_roll: 45deg,
  space_reach: 1.1 * mean_cone_distance + mean_module)

preview {
  unit mm
}
