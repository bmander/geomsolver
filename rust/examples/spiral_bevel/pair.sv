// The layout alone (layout.sv), for the checks that read it: the pitch cones, the trace, the
// crown tooth and its mates, the motions, and each member's blank as its region
// (tests/region_blank.rs, tests/hypoid_layout.rs). gears.sv is the pair to export; the checks of
// its generated faces (tests/envelope/paired.rs) read the faces its built members leave.
unit mm
use std
use design
use layout

pair := layout.HypoidLayout(std.top, design.hypoid_design)
