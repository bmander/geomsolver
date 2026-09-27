// Step 2, the pinion's pitch cone. In the pitch plane its apex A is where the line
// through M at the offset angle from MO meets the square from O to the trace's
// heading, so O and A fall on one point of the heading: R cos(spiral) is
// A's cone distance times cos(spiral + offset), the equal normal pitch, with no
// cosine written. At no offset A is O. In Q, the view square to the pitch plane
// along M -> A, the axis leaves A at the pinion's pitch angle, which the caller
// ties to the gear's (layout.sv).
use std
use design
use views
use pitch.gear
use pitch.trace

component PinionCone(p: plane, q: plane, generator: line, foot: line, design: group) {
  // Seeds only.
  param ct = sqrt(design.pinion_teeth^2 + design.gear_teeth^2)
  param r = design.module * ct / 2
  param rp = r * cos(design.spiral) / cos(design.spiral + design.offset)
  param ex = r * cos(design.offset)
  in p {
    point A hint(x: r - rp * cos(design.offset), y: rp * sin(design.offset))
    line hinge(generator.p2, A)
  }
  generator angle(180deg - design.offset) hinge
  A on foot
  in q {
    point apex hint(x: rp - ex, y: 0)
    point mean hint(x: -ex, y: 0)
    point tip hint(x: rp - ex - rp * design.gear_teeth / ct, y: -rp * design.pinion_teeth / ct)
    line pitch_line(apex, mean)
    line axis(apex, tip)
  }
  apex on p
  mean on p
  A project apex
  generator.p2 project mean
  axis equal pitch_line
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
}
