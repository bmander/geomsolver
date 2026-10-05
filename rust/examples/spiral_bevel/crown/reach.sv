// Step 4, how far the gear's space cutter reaches: a cap in N standing |CO| + reach from the
// cutter's axis, clear of every blank point. crown/space.sv closes the cutter's sections there.
use std
use design
use views
use pitch.gear
use pitch.trace
use crown.thickness
use crown.tooth
use crown.mate

// The cap: in the pitch plane, the point `reach` past the apex on the ray from C through it,
// carried onto the trace normal beyond C; and its image in N, drawn there along the cutter's
// `axis`.
component CutterReach(p: plane, n: plane, apex: point, normal: line, axis: line, reach: Length) {
  in p {
    // Seeds, rough: as far beyond the apex as C is short of it, and twice |CM| beyond C.
    beyond := point hint(at: apex, toward: normal.p1, by: -1)
    radial := point hint(at: normal.p1, toward: normal.p2, by: -2)
    private to_apex := line(normal.p1, apex)
    private past := line(apex, beyond)
    private reach_line := line(normal.p1, beyond)
    private cap_radius := line(normal.p1, radial)
  }
  to_apex angle(0deg) past
  distance(reach) past
  normal angle(180deg) cap_radius
  cap_radius equal reach_line
  in n {
    // Seeds: the image of `radial`, and the axis's run along from it.
    foot := point hint(at: radial)
    end := point hint(at: foot, along: axis)
    cap := line(foot, end)
  }
  foot coincident p
  radial project foot
  axis angle(0deg) cap
  cap equal axis
}

preview {
  unit mm
  pitch := views.PitchView(std.front, span: design.hypoid_design.cutter_radius)
  gear := pitch.gear.GearCone(pitch.view, g.view, design.hypoid_design)
  g := views.FoldedView(pitch.view, gear.generator, pitch.down, span: design.hypoid_design.cutter_radius)
  trace := pitch.trace.ToothTrace(pitch.view, gear.generator, design.hypoid_design)
  thickness := crown.thickness.CrownThickness(pitch.view, gear.generator, trace.normal,
    design.hypoid_design)
  n := views.FoldedView(pitch.view, trace.normal, pitch.down, span: design.hypoid_design.cutter_radius)
  tooth := crown.tooth.CrownTooth(pitch.view, trace.normal, thickness.inner_pitch,
    thickness.outer_pitch, design.hypoid_design) in n.view
  mate := crown.mate.CrownMate(tooth, design.hypoid_design) in n.view
  // Alone, each component's normal module is its own unknown: the trace constructs it.
  in std.front {
    trace.K distance(tooth.normal_module) trace.normal
    trace.K distance(mate.normal_module) trace.normal
  }
  reach := CutterReach(pitch.view, n.view, gear.O, trace.normal, mate.axis,
    reach: design.hypoid_design.space_reach)
}
