// The reference geometry and its analytic faces, read by the generating-system
// checks (tests/envelope/paired.rs). gears.sv is the export entry point.
unit mm
use std
use configuration
use paired_references
use verification

pair: MatchedReferences(std.front, pinion_teeth: pinion_teeth, gear_teeth: gear_teeth,
                        mean_module: mean_module, spiral_angle: 35deg, pressure_angle: 20deg)
faces: ReferenceFaces(pair)
