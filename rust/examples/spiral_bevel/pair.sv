// The layout alone with its analytic faces, for the generating-system checks
// (tests/envelope/paired.rs). gears.sv is the pair to export.
unit mm
use std
use design
use layout
use blank.limits
use verification

pair := layout.HypoidLayout(std.top, design.hypoid_design)
// each member's limits drawn as revolved sections in its axial view, for the checks' walls
pinion_limits := blank.limits.MemberLimits(pair.pinion.pitch_line, pair.pinion.pitch_line,
  pair.pinion.ax, design.hypoid_design, normal_module: pair.normal_module) in pair.q.view
gear_limits := blank.limits.MemberLimits(pair.gear.pitch_line, pair.gear.opposite, pair.gear.ax,
  design.hypoid_design, normal_module: pair.normal_module) in pair.g.view
in std.front {
  faces := verification.ReferenceFaces(pair, pinion_limits, gear_limits)
}
