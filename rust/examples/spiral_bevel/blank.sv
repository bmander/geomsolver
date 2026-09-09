// A finite spherical/conical rim, shared by the generating model and CAD export.
use std
use boundaries

component BevelBlank(f: plane, pitch_angle: Angle,
                    mean_distance: Length, normal_module: Length) {
  in f {
    private point shaft_end
    shaft_end distance(0mm, along: u) f
    shaft_end distance(mean_distance, along: v) f
    construction centerline line shaft(f.origin, shaft_end)
    private toe: SphericalBoundary(f, size: 0.9 * mean_distance)
    private heel: SphericalBoundary(f, size: 1.1 * mean_distance)
    private tip: ConeBoundary(f, shaft, axis_bearing: 90deg, half_angle: pitch_angle,
      normal_offset: normal_module, near: 0.5 * mean_distance, far: 1.5 * mean_distance)
    private back: ConeBoundary(f, shaft, axis_bearing: 90deg, half_angle: pitch_angle,
      normal_offset: -4 * normal_module, near: 0.5 * mean_distance, far: 1.5 * mean_distance)
  }
  private construction solid outside_tip(heel.wall.solid)
  tip.wall.solid cut outside_tip
  solid body(heel.wall.solid)
  outside_tip cut body
  toe.wall.solid cut body
  back.wall.solid cut body
}

preview {
  unit mm
  param pinion_teeth = 24
  param gear_teeth = 48
  param mean_module = 2mm
  param mean_distance = mean_module * hypot(pinion_teeth, gear_teeth) / 2
  blank: BevelBlank(std.front, pitch_angle: atan2(pinion_teeth, gear_teeth),
    mean_distance: mean_distance, normal_module: mean_module * cos(35deg))
}
