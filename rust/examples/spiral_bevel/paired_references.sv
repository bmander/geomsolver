// Mathematical reference geometry for a zero-backlash, 90-degree matched pair:
// the blank boundaries, the crown sections and the motions that generate each
// member. These are construction solids, not the finished gears or machining
// tools. verification.sv declares the analytic faces the checks read.
use std
use reference
use boundaries
use cutters

// A derived plane needs datum points of its own: a point belongs to one view.
// The direction stands a positive, size-scaled span along u, which selects the
// plane's rotor branch.
component PlaneDatum(f: plane, span: Length) {
  point origin hint(at: f.origin)
  point direction hint(x: span)
  origin coincident f.origin
  direction distance(span, along: u) f
  direction distance(0mm, along: v) f
}

component MatchedReferences(front: plane, pinion_teeth: Int, gear_teeth: Int, mean_module: Length,
                            offset_angle: Angle, spiral_angle: Angle, pressure_angle: Angle) {
  param pinion_angle = atan2(pinion_teeth, gear_teeth)
  param gear_angle = atan2(gear_teeth, pinion_teeth)
  param crown_teeth = hypot(pinion_teeth, gear_teeth)
  param mean_distance = mean_module * crown_teeth / 2
  param trace_radius = 0.8 * mean_distance
  param trace_angle = spiral_angle - 90deg
  param center_x = mean_distance - trace_radius * cos(trace_angle)
  param center_y = -trace_radius * sin(trace_angle)
  // The two traces intersect the mean pitch circle exactly half a crown pitch apart.
  param quarter_pitch = 90deg / crown_teeth
  param pitch_x = mean_distance * cos(quarter_pitch) - center_x
  param pitch_y = mean_distance * sin(quarter_pitch)
  param inner_radius = hypot(pitch_x, pitch_y - center_y)
  param outer_radius = hypot(pitch_x, -pitch_y - center_y)
  param section_radius = (outer_radius + inner_radius) / 2
  param section_width = outer_radius - inner_radius
  param pinion_pitch_radius = center_x + section_radius
  param normal_module = mean_module * cos(spiral_angle)
  param root_depth = 1.25 * normal_module
  param transition_radius = 0.3 * normal_module
  param base_depth = 2 * normal_module
  param face_width = 0.2 * mean_distance
  param toe_distance = mean_distance - face_width / 2
  param heel_distance = mean_distance + face_width / 2
  param addendum = normal_module
  param rim_back_depth = 4 * normal_module

  // The back plane is the front with its normal reversed.
  private back_datum: PlaneDatum(front, span: mean_distance)
  private plane back(origin: back_datum.origin, toward: back_datum.direction,
                     u: (1, 0, 0), v: (0, 0, -1))
  group cone_span(near: 0.5 * mean_distance, far: 1.5 * mean_distance)
  group section(width: section_width, tip_height: root_depth, base_depth: base_depth,
                pressure: pressure_angle, tip_radius: transition_radius)

  in front {
    private construction centerline line front_axis(front.origin, hint(y: mean_distance))
    front_axis.p2 distance(0mm, along: u) front
    front_axis.p2 distance(mean_distance, along: v) front
    construction centerline line gear_axis(front.origin,
      hint(x: mean_distance * cos(gear_angle), y: -mean_distance * sin(gear_angle)))
    gear_axis.p2 distance(mean_distance * cos(gear_angle), along: u) front
    gear_axis.p2 distance(-mean_distance * sin(gear_angle), along: v) front
  }
  // A hypoid pair slides the pinion around the crown by the offset angle: its
  // axis turns about the crown's normal at the mean point, so the pinion sees
  // the tooth trace at spiral_angle + offset_angle, and it is enlarged so the
  // tooth-count ratio still holds there. Its apex leaves the gear's, and the two
  // axes stand about mean_distance * sin(offset_angle) apart. At zero this is
  // the bevel member with the common apex. Its pitch cone keeps the bevel
  // pitch angle: it touches the crown plane along the generator through the
  // mean point, so the pair rolls there with hypoid sliding along the tooth.
  param pinion_spiral = spiral_angle + offset_angle
  param pinion_distance = mean_distance * cos(spiral_angle) / cos(pinion_spiral)
  param pinion_toe_distance = pinion_distance - face_width / 2
  param pinion_heel_distance = pinion_distance + face_width / 2
  // The pinion's plane is the front turned about the crown axis through the
  // mean point. An explicit basis stands through the origin, so the turned plane
  // is stood off to pass through that point, and the apex sits along u from the
  // datum by what the enlargement and the turn leave.
  param pinion_apex_u = mean_distance * cos(offset_angle) - pinion_distance
  private turned_datum: PlaneDatum(front, span: mean_distance)
  private plane turned(origin: turned_datum.origin, toward: turned_datum.direction,
                       u: (cos(offset_angle), -sin(offset_angle), 0), v: (0, 0, 1))
  private pinion_datum: PlaneDatum(front, span: mean_distance)
  private plane pinion_plane(origin: pinion_datum.origin, toward: pinion_datum.direction,
                             from: turned, offset: -mean_distance * sin(offset_angle))
  group pinion_cone_span(near: 0.5 * pinion_distance, far: 1.5 * pinion_distance)
  in pinion_plane {
    private point pinion_apex hint(x: pinion_apex_u)
    pinion_apex distance(pinion_apex_u, along: u) pinion_plane
    pinion_apex distance(0mm, along: v) pinion_plane
    construction centerline line pinion_axis(pinion_apex,
      hint(x: pinion_apex_u + pinion_distance * cos(pinion_angle), y: pinion_distance * sin(pinion_angle)))
    pinion_apex distance(pinion_distance * cos(pinion_angle), along: x) pinion_axis.p2
    pinion_apex distance(pinion_distance * sin(pinion_angle), along: y) pinion_axis.p2
    pinion_tip_boundary: ConeBoundary(pinion_plane, pinion_apex, pinion_axis, axis_bearing: pinion_angle,
      half_angle: pinion_angle, normal_offset: addendum, span: pinion_cone_span)
    pinion_root_boundary: ConeBoundary(pinion_plane, pinion_apex, pinion_axis, axis_bearing: pinion_angle,
      half_angle: pinion_angle, normal_offset: -root_depth, span: pinion_cone_span)
    pinion_back_boundary: ConeBoundary(pinion_plane, pinion_apex, pinion_axis, axis_bearing: pinion_angle,
      half_angle: pinion_angle, normal_offset: -rim_back_depth, span: pinion_cone_span)
    // The pinion's ends are spheres about its own apex, poles along the crown axis.
    pinion_toe: SphericalBoundary(pinion_plane, pinion_apex, size: pinion_toe_distance)
    pinion_heel: SphericalBoundary(pinion_plane, pinion_apex, size: pinion_heel_distance)
  }
  in front {
    gear_tip_boundary: ConeBoundary(front, front.origin, gear_axis, axis_bearing: -gear_angle,
      half_angle: gear_angle, normal_offset: addendum, span: cone_span)
    gear_root_boundary: ConeBoundary(front, front.origin, gear_axis, axis_bearing: -gear_angle,
      half_angle: gear_angle, normal_offset: -root_depth, span: cone_span)
    gear_back_boundary: ConeBoundary(front, front.origin, gear_axis, axis_bearing: -gear_angle,
      half_angle: gear_angle, normal_offset: -rim_back_depth, span: cone_span)
  }
  // The spheres are drawn in the back plane so their revolution poles stand
  // perpendicular to both shaft axes, outside both tip cones. A pole inside a
  // cone's Boolean region leaves the kernel an open shell.
  in back {
    toe: SphericalBoundary(back, back_datum.origin, size: toe_distance)
    heel: SphericalBoundary(back, back_datum.origin, size: heel_distance)
  }

  // The crown traces have their own center, away from the pitch-cone apex. The
  // offset planes locate that center in depth; ordinate constraints locate it in x.
  // Each mate exchanges inner and outer and reverses height, so the two active
  // cones coincide with the pinion references with opposite boundary orientation.
  private crown_front_datum: PlaneDatum(front, span: mean_distance)
  private plane crown_front(origin: crown_front_datum.origin, toward: crown_front_datum.direction,
                            from: front, offset: -center_y)
  in crown_front {
    private construction centerline line crown_front_axis(hint(x: center_x),
                                                          hint(x: center_x, y: mean_distance))
    crown_front_axis.p1 distance(center_x, along: u) crown_front
    crown_front_axis.p1 distance(0mm, along: v) crown_front
    crown_front_axis.p2 distance(center_x, along: u) crown_front
    crown_front_axis.p2 distance(mean_distance, along: v) crown_front
    pinion: RoundedRackSection(crown_front, pitch_radius: pinion_pitch_radius, section: section)
  }
  private crown_back_datum: PlaneDatum(back, span: mean_distance)
  private plane crown_back(origin: crown_back_datum.origin, toward: crown_back_datum.direction,
                           from: back, offset: center_y)
  in crown_back {
    private construction centerline line crown_back_axis(hint(x: center_x),
                                                         hint(x: center_x, y: mean_distance))
    crown_back_axis.p1 distance(center_x, along: u) crown_back
    crown_back_axis.p1 distance(0mm, along: v) crown_back
    crown_back_axis.p2 distance(center_x, along: u) crown_back
    crown_back_axis.p2 distance(mean_distance, along: v) crown_back
    gear_outer: RoundedRackSection(crown_back, pitch_radius: pinion_pitch_radius + section_width,
      section: section)
    gear_inner: RoundedRackSection(crown_back, pitch_radius: pinion_pitch_radius - section_width,
      section: section)
  }
  // All rotations share the crown roll angle. The pitch cones roll without slip.
  motion crown_roll(about: front_axis)
  motion pinion_roll(about: pinion_axis, ratio: 1 / sin(pinion_angle))
  motion gear_roll(about: gear_axis, ratio: -1 / sin(gear_angle))
  motion pinion_generation(crown_roll, relative_to: pinion_roll)
  motion gear_generation(crown_roll, relative_to: gear_roll)

  construction solid pinion_crown(pinion.profile, about: crown_front_axis)
  construction solid gear_outer_crown(gear_outer.profile, about: crown_back_axis)
  construction solid gear_inner_crown(gear_inner.profile, about: crown_back_axis)
  // Public construction inputs for the solid components. Indexing is one member
  // angle; generating roll has the pitch-cone ratio and is a different motion.
  // The root cone bounds only the tooth regions verification.sv declares.
  motion pinion_index(about: pinion_axis)
  motion gear_index(about: gear_axis)
  motion crown_neighbor(about: front_axis, phase: -360deg / crown_teeth)
  group pinion_design(heel: pinion_heel.wall.solid, toe: pinion_toe.wall.solid,
    tip: pinion_tip_boundary.wall.solid, root: pinion_root_boundary.wall.solid,
    back: pinion_back_boundary.wall.solid,
    generation: pinion_generation, indexing: pinion_index)
  group gear_design(heel: heel.wall.solid, toe: toe.wall.solid,
    tip: gear_tip_boundary.wall.solid, root: gear_root_boundary.wall.solid,
    back: gear_back_boundary.wall.solid,
    generation: gear_generation, indexing: gear_index)
  // Inverse generating and neighbor motions rotate about the common apex.
  // For any blank point, radial distance from the offset crown axis is at most
  // |crown center| + heel radius. The extra module keeps this artificial cap clear.
  construction gear_space: ComplementarySpace(crown_back, crown_back_axis,
    gear_outer, gear_inner, crown_neighbor, radial_start: center_x,
    radial_end: center_x + hypot(center_x, center_y) + heel_distance + mean_module)
}
