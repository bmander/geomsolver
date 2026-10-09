// Step 4, how far the gear's space cutter reaches: a cap in N standing |CO| + reach from the
// cutter's axis, clear of every blank point. crown/space.sv closes the cutter's sections there.
use std (turned)
use design
use views
use pitch.gear
use pitch.trace
use crown.thickness
use crown.tooth
use crown.mate

// The cap: in the pitch plane, the point `reach` past the apex on the axis from C through it,
// carried onto the trace normal beyond C, and drawn in N along the cutter's `axis` from there.
component CutterReach(p: plane, n: plane, apex: point, normal: line, ax: line, reach: Length) {
  // Seeds, rough: as far beyond the apex as C is short of it, and twice |CM| beyond C. The
  // radial's end lies on the line where P and N meet, so it is drawn in both.
  radial := point hint(at: normal.p1, toward: normal.p2, by: -2) in p, n
  in p {
    beyond := point hint(at: apex, toward: normal.p1, by: -1)
    private to_apex := line(normal.p1, apex)
    private past := line(apex, beyond)
    private reach_line := line(normal.p1, beyond)
    private cap_radius := line(normal.p1, radial)
  }
  to_apex angle(0deg) past
  distance(reach) past
  radial coincident normal
  normal.p1 distance(<= 0, along: normal) radial
  cap_radius equal reach_line
  in n {
    // Seed: the axis's run along from `radial`.
    end := point hint(at: radial, along: ax)
    cap := line(radial, end)
  }
  cap turned(theta: 0deg) ax
}

preview {
  unit mm
  gear := pitch.gear.GearCone(std.top, g.view, design.hypoid_design)
  g := views.FoldedView(std.top, gear.generator)
  trace := pitch.trace.ToothTrace(std.top, gear.generator, design.hypoid_design)
  thickness := crown.thickness.CrownThickness(std.top, gear.generator, trace.normal,
    design.hypoid_design)
  n := views.FoldedView(std.top, trace.normal)
  tooth := crown.tooth.CrownTooth(std.top, trace.normal, thickness.inner_pitch,
    thickness.outer_pitch, design.hypoid_design, normal_module: trace.normal_module) in n.view
  mate := crown.mate.CrownMate(tooth, design.hypoid_design,
    normal_module: trace.normal_module) in n.view
  reach := CutterReach(std.top, n.view, gear.O, trace.normal, mate.ax,
    reach: design.hypoid_design.space_reach)
}
