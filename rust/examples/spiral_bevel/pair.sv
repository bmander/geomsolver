// The pair's layout and its analytic faces, read by the generating-system checks
// (tests/envelope/paired.rs). gears.sv is the export entry point.
unit mm
use std
use design
use layout
use verification

pair := layout.HypoidLayout(std.front, design.hypoid_design)
faces := verification.ReferenceFaces(pair)
