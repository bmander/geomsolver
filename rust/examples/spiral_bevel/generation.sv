// Step 5, generation. Every roll shares the crown's angle, and each member turns
// at the ratio its pitch cone rolls on the crown: its cone distance over its pitch
// radius at M, read off the pitch triangles after the solve. Indexing is one member
// angle; the crown's neighbour is one crown pitch round, four quarter pitches of
// the mean pitch circle.
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
    ratio: distance(pinion.apex, pinion.mean) / distance(pinion.mean, pinion.axis))
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
  pitch: PitchView(std.front)
  gear: GearCone(pitch.view, g.view, hypoid_design)
  g: FoldedView(pitch.view, gear.generator)
  trace: ToothTrace(pitch.view, gear.generator, hypoid_design)
  pinion: PinionCone(pitch.view, q.view, gear.generator, trace.foot, hypoid_design)
  q: FoldedView(pitch.view, pinion.hinge)
  gear.to_apex angle(pinion_angle, sense: cw) gear.to_foot
  pinion.pitch_line angle(pinion_angle) pinion.axis
  thickness: CrownThickness(pitch.view, gear.generator, trace.normal, hypoid_design)
  generation: Generation(gear, pinion, thickness)
}
