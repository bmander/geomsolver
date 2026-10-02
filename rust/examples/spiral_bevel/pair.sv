// The layout alone with its analytic faces, for the generating-system checks
// (tests/envelope/paired.rs). gears.sv is the pair to export.
unit mm
use std
use design
use layout
use verification

pair := layout.HypoidLayout(std.front, design.hypoid_design)
faces := verification.ReferenceFaces(pair)
