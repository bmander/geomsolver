// Step 2, the tooth trace, in the pitch plane. The cutter centre C stands the cutter radius
// from M, MC at 90deg less the spiral angle to MO; the trace is the circle about C through M,
// and its heading, the tangent at M, meets the square dropped from O at H. The normal module
// is the module seen square to the trace: how far from MC stands K, one module from M along
// the generator. A caller leaves `normal_module` unbound, and this constructs it.
use std
use design
use views
use pitch.gear

component ToothTrace(p: plane, generator: line, design: group, normal_module: Length) {
  // Seeds. C's is exact, and must be: N folds along MC and the crown sections are seeded in
  // it, so a rough C seeds them far from where they solve, and the whole solve can stall just
  // short of acceptance, a section's narrow tip collapsing. H's is rough: behind M, on its
  // right.
  r := design.cone_distance
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
  pitch := views.PitchView(std.front, span: design.hypoid_design.cutter_radius)
  gear := pitch.gear.GearCone(pitch.view, g.view, design.hypoid_design)
  g := views.FoldedView(pitch.view, gear.generator, span: design.hypoid_design.cutter_radius)
  trace := ToothTrace(pitch.view, gear.generator, design.hypoid_design)
}
