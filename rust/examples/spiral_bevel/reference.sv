// The normal section of a reference crown tooth. Its straight flanks and circular
// tip transitions become cones and tori when revolved. This is construction geometry
// for the bevel-pair design, not a specification of a physical cutting tool.
use std

// `section` bundles width, tip_height, base_depth, outer_pressure, inner_pressure
// and tip_radius; the pitch radius is the one number that differs between
// sections of one pair. Each flank has its own pressure angle, since a hypoid's
// offset works the two sides of a tooth at different effective angles.
component RoundedRackSection(f: plane, pitch_radius: Length, section: group) {
  param width = section.width
  param tip_height = section.tip_height
  param base_depth = section.base_depth
  param outer_pressure = section.outer_pressure
  param inner_pressure = section.inner_pressure
  param tip_radius = section.tip_radius
  private construction line datum(f.origin, f.toward)
  private point center hint(x: pitch_radius, y: 0)
  center distance(pitch_radius, along: u) f
  center distance(0mm, along: v) f

  private point lp hint(x: pitch_radius - width / 2, y: 0)
  private point rp hint(x: pitch_radius + width / 2, y: 0)
  private construction line pitch(lp, rp)
  center midpoint pitch
  pitch parallel datum
  distance(width) pitch

  // Seeds choose the minor fillets. Tangency, radii, angles and heights determine them.
  param join_height_right = tip_height - tip_radius * (1 - sin(outer_pressure))
  param join_height_left = tip_height - tip_radius * (1 - sin(inner_pressure))
  param join_right = pitch_radius + width / 2 - join_height_right * tan(outer_pressure)
  param join_left = pitch_radius - width / 2 + join_height_left * tan(inner_pressure)
  param top_right = join_right - tip_radius * cos(outer_pressure)
  param top_left = join_left + tip_radius * cos(inner_pressure)
  private point bl hint(x: pitch_radius - width / 2 - base_depth * tan(inner_pressure), y: -base_depth)
  private point br hint(x: pitch_radius + width / 2 + base_depth * tan(outer_pressure), y: -base_depth)
  private point rj hint(x: join_right, y: join_height_right)
  private point rt hint(x: top_right, y: tip_height)
  private point lt hint(x: top_left, y: tip_height)
  private point lj hint(x: join_left, y: join_height_left)
  private point cr hint(x: top_right, y: tip_height - tip_radius)
  private point cl hint(x: top_left, y: tip_height - tip_radius)

  profile = line base(bl, br) -> line outer(br, rj) -> tangent
            arc outer_round(center: cr) -> tangent line tip(rt, lt) -> tangent
            arc inner_round(center: cl) -> tangent line inner(lj, bl) -> close
  base parallel datum
  tip parallel datum
  base angle(90deg + outer_pressure) outer
  base angle(270deg - inner_pressure) inner
  lp on inner
  rp on outer
  center distance(base_depth, side: left) base
  center distance(tip_height, side: left) tip
  radius(tip_radius) outer_round
  outer_round equal inner_round
}

preview {
  unit mm
  param pinion_teeth = 24
  param gear_teeth = 48
  param mean_module = 2mm
  param mean_cone_distance = mean_module * hypot(pinion_teeth, gear_teeth) / 2
  param reference_radius = 0.8 * mean_cone_distance
  // Preview proportions only; the matched pair derives its section width from the
  // crown traces (paired_references.sv). These are not manufacturing allowances.
  group section(width: 1.3 * mean_module, tip_height: mean_module,
                base_depth: mean_module, outer_pressure: 20deg, inner_pressure: 20deg,
                tip_radius: 0.3 * mean_module)
  rack: RoundedRackSection(std.front, pitch_radius: reference_radius, section: section)
  construction centerline line axis(std.origin, std.up.toward)
  solid crown(rack.profile, about: axis)
  surface outer(crown, rack.outer)
  surface outer_round(crown, rack.outer_round)
  surface inner(crown, rack.inner)
  surface inner_round(crown, rack.inner_round)
  surface tip(crown, rack.tip)
}
