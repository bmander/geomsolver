// Step 4, the crown tooth's thickness, in the pitch plane. Two points on the mean
// pitch circle a quarter of the crown's circular pitch either side of M: an arc
// of length pi * module / 4 about the apex. The inner and outer trace circles
// about the cutter centre C pass through them, so the tooth is their radial gap,
// and its pitch points are where they cross the trace normal beyond C.
use std
use design
use views
use pitch.gear
use pitch.trace

// `generator` runs from the gear apex O to M, `normal` from C to M.
component CrownThickness(p: plane, generator: line, normal: line, design: group) {
  // Seeds only: the closed forms, so the sections drawn on the pitch points start solved.
  param ct = sqrt(design.pinion_teeth^2 + design.gear_teeth^2)
  param r = design.module * ct / 2
  param rc = design.cutter_radius
  param q = 90deg / ct
  param cx = r - rc * sin(design.spiral)
  param cy = rc * cos(design.spiral)
  param ri = sqrt((r * cos(q) - cx)^2 + (r * sin(q) - cy)^2)
  param ro = sqrt((r * cos(q) - cx)^2 + (r * sin(q) + cy)^2)
  in p {
    point ahead_end hint(x: r * cos(q), y: r * sin(q))
    point behind_end hint(x: r * cos(q), y: -r * sin(q))
    arc ahead(center: generator.p1, start: generator.p2, end: ahead_end) hint(r: r)
    arc behind(center: generator.p1, start: behind_end, end: generator.p2) hint(r: r)
    circle inner(center: normal.p1) hint(r: ri)
    circle outer(center: normal.p1) hint(r: ro)
    point inner_pitch hint(x: normal.p1.x + (normal.p1.x - normal.p2.x) * ri / rc,
                           y: normal.p1.y + (normal.p1.y - normal.p2.y) * ri / rc)
    point outer_pitch hint(x: normal.p1.x + (normal.p1.x - normal.p2.x) * ro / rc,
                           y: normal.p1.y + (normal.p1.y - normal.p2.y) * ro / rc)
    line to_inner(normal.p1, inner_pitch)
    line to_outer(normal.p1, outer_pitch)
  }
  length(pi * design.module / 4) ahead
  length(pi * design.module / 4) behind
  ahead_end on inner
  behind_end on outer
  normal angle(180deg) to_inner
  normal angle(180deg) to_outer
  inner_pitch on inner
  outer_pitch on outer
}

preview {
  unit mm
  pitch: PitchView(std.front, span: hypoid_design.cutter_radius)
  gear: GearCone(pitch.view, g.view, hypoid_design)
  g: FoldedView(pitch.view, gear.generator, span: hypoid_design.cutter_radius)
  trace: ToothTrace(pitch.view, gear.generator, hypoid_design)
  thickness: CrownThickness(pitch.view, gear.generator, trace.normal, hypoid_design)
}
