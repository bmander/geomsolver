// Both members as ordinary solids. Sweep-boundary export is still being integrated.
unit mm
use std
use configuration
use matched_pair

param datum_span = mean_module * hypot(pinion_teeth, gear_teeth) / 2
private point back_origin
private point back_direction hint(x: datum_span)
back_origin coincident std.origin
back_direction distance(datum_span, along: u) std.front
back_direction distance(0mm, along: v) std.front
plane back(origin: back_origin, toward: back_direction, u: (1, 0, 0), v: (0, 0, -1))
pair: MatchedPair(std.front, back, pinion_teeth: pinion_teeth, gear_teeth: gear_teeth,
  mean_module: mean_module, spiral_angle: 35deg, pressure_angle: 20deg)
