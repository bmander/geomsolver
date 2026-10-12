// Step 4, the face mill the crown is cut by: its axis through the cutter centre C, square to the
// pitch plane along the crown's axis, and the two flank cones about it that the crown tooth and
// the gear's space share, each through its pitch point and leaning by the pressure angle split by
// the shift — the inner one widening toward the tooth's tip, the outer one narrowing — and the
// levels along it the crowns' tips and bases stand at: the tooth's tip up the axis from the pitch
// plane and its base down, its mate's the other way round. Every face of the crown is a set about
// this axis (§6.21): its flanks these cones, its depths these levels, its tips rounded where they
// meet a flank; each region's meridian turned once.
use std
use design
use views
use pitch.gear
use pitch.trace
use crown.thickness

// `normal` runs from C to the mean point in the pitch plane; `crown_axis` is the crown's, square
// to the pitch plane; `inner` and `outer` are the pitch points on the trace normal beyond C.
component Cutter(crown_axis: line, normal: line, inner: point, outer: point, design: group,
                 normal_module: Length) {
  // Seed: the cutter's radius up the crown's axis (as long as the cone distance) from C.
  top := point hint(at: normal.p1, along: crown_axis,
                    by: design.cutter_radius / design.cone_distance)
  ax := line(normal.p1, top)
  ax parallel crown_axis
  normal.p1 distance(design.cutter_radius, along: crown_axis) top
  inner_flank := std.ConeThrough(ax, inner, lean: design.pressure + design.shift)
  outer_flank := std.ConeThrough(ax, outer, lean: design.shift - design.pressure)
  tooth_tip := std.Level(normal.p1, ax, d: design.dedendum * normal_module)
  tooth_base := std.Level(normal.p1, ax, d: -design.base * normal_module)
  mate_tip := std.Level(normal.p1, ax, d: -design.dedendum * normal_module)
  mate_base := std.Level(normal.p1, ax, d: design.base * normal_module)
}

preview {
  unit mm
  gear := pitch.gear.GearCone(std.top, g.view, design.hypoid_design)
  g := views.FoldedView(std.top, gear.generator)
  trace := pitch.trace.ToothTrace(std.top, gear.generator, design.hypoid_design)
  thickness := crown.thickness.CrownThickness(std.top, gear.generator, trace.normal,
    design.hypoid_design)
  cutter := Cutter(gear.crown_axis, trace.normal, thickness.inner_pitch, thickness.outer_pitch,
    design.hypoid_design, normal_module: trace.normal_module)
}
