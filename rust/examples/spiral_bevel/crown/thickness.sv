// Step 4, the crown tooth's thickness, in the pitch plane. Two points on the mean pitch circle
// a quarter of the crown's circular pitch either side of M, arcs of length pi m / 4 about the
// apex; the inner and outer trace circles about the cutter centre C pass through them, so the
// tooth is their radial gap, and its pitch points are where they cross the trace normal beyond
// C.
use std
use design
use views
use pitch.gear
use pitch.trace

// `generator` runs from the gear apex O to M, `normal` from C to M.
component CrownThickness(p: plane, generator: line, normal: line, design: group) {
  // Seeds only, rough: the quarter pitch's ends straight across from M, and the trace circles
  // seven tenths of a module either side of the cutter radius, the pitch points on them beyond
  // C.
  r := design.cone_distance
  rc := design.cutter_radius
  ri := rc - 0.7 * design.module
  ro := rc + 0.7 * design.module
  in p {
    ahead_end := point hint(x: r, y: pi * design.module / 4)
    behind_end := point hint(x: r, y: -pi * design.module / 4)
    ahead := arc(center: generator.p1, start: generator.p2, end: ahead_end) hint(r: r)
    behind := arc(center: generator.p1, start: behind_end, end: generator.p2) hint(r: r)
    inner := circle(center: normal.p1) hint(r: ri)
    outer := circle(center: normal.p1) hint(r: ro)
    inner_pitch := point hint(at: normal.p1, toward: normal.p2, by: -ri / rc)
    outer_pitch := point hint(at: normal.p1, toward: normal.p2, by: -ro / rc)
    to_inner := line(normal.p1, inner_pitch)
    to_outer := line(normal.p1, outer_pitch)
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
  pitch := views.PitchView(std.front, span: design.hypoid_design.cutter_radius)
  gear := pitch.gear.GearCone(pitch.view, g.view, design.hypoid_design)
  g := views.FoldedView(pitch.view, gear.generator, span: design.hypoid_design.cutter_radius)
  trace := pitch.trace.ToothTrace(pitch.view, gear.generator, design.hypoid_design)
  thickness := CrownThickness(pitch.view, gear.generator, trace.normal, design.hypoid_design)
}
