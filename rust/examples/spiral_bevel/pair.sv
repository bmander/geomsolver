// The pair's layout and its analytic faces, read by the generating-system checks
// (tests/envelope/paired.rs). gears.sv is the export entry point.
unit mm
use std
use design
use layout
use verification

pair: HypoidLayout(std.front, hypoid_design)
faces: ReferenceFaces(pair)
