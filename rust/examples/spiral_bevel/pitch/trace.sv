// Step 2, the tooth trace, in the pitch plane. The cutter centre C stands at the
// cutter radius from M, with MC at 90deg - spiral to MO; the trace is the circle
// about C through M, and its heading, the tangent at M, meets the square dropped
// from O at H. The normal module is the module seen square to the trace: the
// distance from MC of K, the point one module from M along the generator. A caller
// leaves `normal_module` unbound, and this constructs it.
use std
use design
use views
use pitch.gear

component ToothTrace(p: plane, generator: line, design: group, normal_module: Length) {
  // Seeds only. C's is exact, and kept so: the normal view N is folded along MC and the
  // crown sections are seeded in it, so a rough C leaves their seeds far from where their
  // pitch points solve, and the whole-system solve can then stall just under the
  // interactive acceptance, a section's narrow tip collapsing. H's is rough: behind M, on
  // its right.
  r := design.module * sqrt(design.pinion_teeth^2 + design.gear_teeth^2) / 2
  rc := design.cutter_radius
  in p {
    C := point hint(x: r - rc * sin(design.spiral), y: rc * cos(design.spiral))
    H := point hint(x: r / 4, y: -r / 2)
    K := point hint(x: r - design.module, y: 0)
    normal := line(C, generator.p2)
    heading := line(generator.p2, H)
    foot := line(generator.p1, H)
    trace := circle(center: C) hint(r: rc)
  }
  generator.p2 distance(design.cutter_radius) C
  generator angle(90deg - design.spiral, sense: cw) normal
  generator.p2 on trace
  heading perpendicular normal
  foot perpendicular heading
  K on generator
  generator.p2 distance(design.module) K
  K distance(normal_module) normal
}

preview {
  unit mm
  pitch := PitchView(std.front, span: hypoid_design.cutter_radius)
  gear := GearCone(pitch.view, g.view, hypoid_design)
  g := FoldedView(pitch.view, gear.generator, span: hypoid_design.cutter_radius)
  trace := ToothTrace(pitch.view, gear.generator, hypoid_design)
}
