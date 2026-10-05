// Step 5, generation. Every roll shares the crown's angle, and each member turns at the crown's
// tooth count over its own, N_c / N with N_c = 2R / m, so both stay conjugate through the one
// crown. The ratios are read off the gear's triangle after the solve: its hypotenuse R over the
// gear's pitch radius, and over the short leg N_p m / 2 for the pinion, whose own cone, solved
// off the bevel, would roll at another. Indexing turns a member; the crown's neighbour is one
// crown pitch round, four quarter pitches of the mean pitch circle.
use std
use design
use views
use pitch.gear
use pitch.trace
use pitch.pinion
use crown.thickness

// `gear` is a GearCone, `pinion` a PinionCone and `thickness` a CrownThickness.
component Generation(gear: group, pinion: group, thickness: group) {
  crown_roll := motion(about: gear.crown_axis)
  pinion_roll := motion(about: pinion.ax,
    ratio: distance(gear.apex, gear.mean) / distance(gear.apex, gear.foot))
  gear_roll := motion(about: gear.ax,
    ratio: -distance(gear.apex, gear.mean) / distance(gear.mean, gear.ax))
  pinion_generation := motion(crown_roll, relative_to: pinion_roll)
  gear_generation := motion(crown_roll, relative_to: gear_roll)
  pinion_index := motion(about: pinion.ax)
  gear_index := motion(about: gear.ax)
  crown_neighbor := motion(about: gear.crown_axis,
    phase: -4 * length(thickness.ahead) / radius(thickness.ahead) * 1rad)
}

preview {
  unit mm
  pitch := views.PitchView(std.front, span: design.hypoid_design.cutter_radius)
  gear := pitch.gear.GearCone(pitch.view, g.view, design.hypoid_design)
  g := views.FoldedView(pitch.view, gear.generator, pitch.down, span: design.hypoid_design.cutter_radius)
  trace := pitch.trace.ToothTrace(pitch.view, gear.generator, design.hypoid_design)
  pinion := pitch.pinion.PinionCone(pitch.view, q.view, gear, trace.foot, design.hypoid_design)
  q := views.FoldedView(pitch.view, pinion.hinge, pitch.down, span: design.hypoid_design.cutter_radius)
  thickness := crown.thickness.CrownThickness(pitch.view, gear.generator, trace.normal,
    design.hypoid_design)
  in std.front {
    generation := Generation(gear, pinion, thickness)
  }
}
