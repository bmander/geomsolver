// Step 2, the tooth trace, in the pitch plane. The cutter centre C stands at the
// cutter radius from M, with MC at 90deg - spiral to MO; the trace is the circle
// about C through M, and its heading, the tangent at M, meets the square dropped
// from O at H. The normal module is the module seen square to the trace, which the
// claim checks: the distance from MC of a point one module from M along the
// generator.
use std
use design
use views
use pitch.gear

component ToothTrace(p: plane, generator: line, design: group) {
  // Seeds only.
  param r = design.module * sqrt(design.pinion_teeth^2 + design.gear_teeth^2) / 2
  param rc = design.cutter_radius
  in p {
    point C hint(x: r - rc * sin(design.spiral), y: rc * cos(design.spiral))
    point H hint(x: r * sin(design.spiral)^2, y: -r * cos(design.spiral) * sin(design.spiral))
    private point K hint(x: r - design.module, y: 0)
    line normal(C, generator.p2)
    line heading(generator.p2, H)
    line foot(generator.p1, H)
    circle trace(center: C) hint(r: rc)
  }
  generator.p2 distance(design.cutter_radius) C
  generator angle(90deg - design.spiral, sense: cw) normal
  generator.p2 on trace
  heading perpendicular normal
  foot perpendicular heading
  K on generator
  generator.p2 distance(design.module) K
  claim K distance(design.normal_module) normal
}

preview {
  unit mm
  pitch: PitchView(std.front, span: hypoid_design.cutter_radius)
  gear: GearCone(pitch.view, g.view, hypoid_design)
  g: FoldedView(pitch.view, gear.generator, span: hypoid_design.cutter_radius)
  trace: ToothTrace(pitch.view, gear.generator, hypoid_design)
}
