// Step 5, generation. Every roll shares the crown's angle, and each member turns
// at the crown's tooth count over its own, 2R / m over N, so both stay conjugate
// through the one crown: read off the gear's triangle after the solve, its
// hypotenuse R over the gear's pitch radius, and over its short leg, N_p m / 2, for
// the pinion. The pinion's own cone, solved off the bevel, rolls at another ratio.
// Indexing is one member angle; the crown's neighbour is one crown pitch round,
// four quarter pitches of the mean pitch circle.
use std
use design
use views
use pitch.gear
use pitch.trace
use pitch.pinion
use crown.thickness

// `gear` is a GearCone, `pinion` a PinionCone and `thickness` a CrownThickness.
component Generation(gear: group, pinion: group, thickness: group) {
  motion crown_roll(about: gear.crown_axis)
  motion pinion_roll(about: pinion.axis,
    ratio: distance(gear.apex, gear.mean) / distance(gear.apex, gear.foot))
  motion gear_roll(about: gear.axis,
    ratio: -distance(gear.apex, gear.mean) / distance(gear.mean, gear.axis))
  motion pinion_generation(crown_roll, relative_to: pinion_roll)
  motion gear_generation(crown_roll, relative_to: gear_roll)
  motion pinion_index(about: pinion.axis)
  motion gear_index(about: gear.axis)
  motion crown_neighbor(about: gear.crown_axis,
    phase: -4 * length(thickness.ahead) / radius(thickness.ahead) * 1rad)
}

preview {
  unit mm
  pitch: PitchView(std.front, span: hypoid_design.cutter_radius)
  gear: GearCone(pitch.view, g.view, hypoid_design)
  g: FoldedView(pitch.view, gear.generator, span: hypoid_design.cutter_radius)
  trace: ToothTrace(pitch.view, gear.generator, hypoid_design)
  pinion: PinionCone(pitch.view, q.view, gear, trace.foot, hypoid_design)
  q: FoldedView(pitch.view, pinion.hinge, span: hypoid_design.cutter_radius)
  thickness: CrownThickness(pitch.view, gear.generator, trace.normal, hypoid_design)
  generation: Generation(gear, pinion, thickness)
}
