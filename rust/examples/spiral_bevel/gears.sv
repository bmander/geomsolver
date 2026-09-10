// Both members as ordinary solids. `solventc --stl` or `--step` exports either
// body; see README.md.
unit mm
use std
use configuration
use matched_pair

pair: MatchedPair(std.front, pinion_teeth: pinion_teeth, gear_teeth: gear_teeth,
  mean_module: mean_module, axis_offset: axis_offset,
  spiral_angle: 35deg, pressure_angle: 20deg)
