// Mathematical reference surfaces for a zero-backlash, 90-degree matched pair.
// These are construction solids, not the finished gears or machining tools.
use std
use reference
use boundaries
use cutters

component MatchedReferences(front: plane, back: plane,
                            pinion_teeth: Int, gear_teeth: Int, mean_module: Length,
                            spiral_angle: Angle, pressure_angle: Angle) {
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
  param normal_module = mean_module * cos(spiral_angle)
  param root_depth = 1.25 * normal_module
  param transition_radius = 0.3 * normal_module
  param base_depth = 2 * normal_module
  param face_width = 0.2 * mean_distance
  param toe_distance = mean_distance - face_width / 2
  param heel_distance = mean_distance + face_width / 2
  param addendum = normal_module
  param rim_back_depth = 4 * normal_module

  in front {
    private point front_axis_end hint(y: mean_distance)
    front_axis_end distance(0mm, along: u) front
    front_axis_end distance(mean_distance, along: v) front
    private construction centerline line front_axis(front.origin, front_axis_end)
    private point pinion_axis_end
    pinion_axis_end distance(mean_distance * cos(pinion_angle), along: u) front
    pinion_axis_end distance(mean_distance * sin(pinion_angle), along: v) front
    private construction centerline line pinion_axis(front.origin, pinion_axis_end)
    private point gear_axis_end
    gear_axis_end distance(mean_distance * cos(gear_angle), along: u) front
    gear_axis_end distance(-mean_distance * sin(gear_angle), along: v) front
    private construction centerline line gear_axis(front.origin, gear_axis_end)
  }
  // The spheres are drawn in the back plane so their revolution poles stand
  // perpendicular to both shaft axes, outside both tip cones. A pole inside a
  // cone's Boolean region leaves the kernel an open shell.
  in back {
    toe: SphericalBoundary(back, size: toe_distance)
    heel: SphericalBoundary(back, size: heel_distance)
  }
  in front {
    pinion_tip_boundary: ConeBoundary(front, pinion_axis, axis_bearing: pinion_angle,
      half_angle: pinion_angle, normal_offset: addendum,
      near: 0.5 * mean_distance, far: 1.5 * mean_distance)
    pinion_root_boundary: ConeBoundary(front, pinion_axis, axis_bearing: pinion_angle,
      half_angle: pinion_angle, normal_offset: -root_depth,
      near: 0.5 * mean_distance, far: 1.5 * mean_distance)
    pinion_back_boundary: ConeBoundary(front, pinion_axis, axis_bearing: pinion_angle,
      half_angle: pinion_angle, normal_offset: -rim_back_depth,
      near: 0.5 * mean_distance, far: 1.5 * mean_distance)
    gear_tip_boundary: ConeBoundary(front, gear_axis, axis_bearing: -gear_angle,
      half_angle: gear_angle, normal_offset: addendum,
      near: 0.5 * mean_distance, far: 1.5 * mean_distance)
    gear_root_boundary: ConeBoundary(front, gear_axis, axis_bearing: -gear_angle,
      half_angle: gear_angle, normal_offset: -root_depth,
      near: 0.5 * mean_distance, far: 1.5 * mean_distance)
    gear_back_boundary: ConeBoundary(front, gear_axis, axis_bearing: -gear_angle,
      half_angle: gear_angle, normal_offset: -rim_back_depth,
      near: 0.5 * mean_distance, far: 1.5 * mean_distance)
  }
  pinion_tip_ends: EndCircles(pinion_tip_boundary.wall, toe.wall, heel.wall)
  pinion_root_ends: EndCircles(pinion_root_boundary.wall, toe.wall, heel.wall)
  pinion_back_ends: EndCircles(pinion_back_boundary.wall, toe.wall, heel.wall)
  gear_tip_ends: EndCircles(gear_tip_boundary.wall, toe.wall, heel.wall)
  gear_root_ends: EndCircles(gear_root_boundary.wall, toe.wall, heel.wall)
  gear_back_ends: EndCircles(gear_back_boundary.wall, toe.wall, heel.wall)

  // The crown traces have their own center, away from the pitch-cone apex. The
  // offset planes locate that center in depth; ordinate constraints locate it in x.
  // A positive, size-scaled datum selects the private plane rotor branch.
  private point crown_front_origin
  private point crown_front_direction hint(x: mean_distance)
  crown_front_origin coincident front.origin
  crown_front_direction distance(mean_distance, along: u) front
  crown_front_direction distance(0mm, along: v) front
  private plane crown_front(origin: crown_front_origin, toward: crown_front_direction,
                            from: front, offset: -center_y)
  in crown_front {
    private point crown_front_start
    private point crown_front_end
    crown_front_start distance(center_x, along: u) crown_front
    crown_front_start distance(0mm, along: v) crown_front
    crown_front_end distance(center_x, along: u) crown_front
    crown_front_end distance(mean_distance, along: v) crown_front
    private construction centerline line crown_front_axis(crown_front_start, crown_front_end)
    pinion: RoundedRackSection(crown_front, pitch_radius: center_x + section_radius,
      width: section_width, tip_height: root_depth, base_depth: base_depth,
      pressure: pressure_angle, tip_radius: transition_radius)
  }
  // Each mate exchanges inner and outer and reverses height. Thus the two active
  // cones coincide with the pinion references with opposite boundary orientation.
  private point crown_back_origin
  private point crown_back_direction hint(x: mean_distance)
  crown_back_origin coincident back.origin
  crown_back_direction distance(mean_distance, along: u) back
  crown_back_direction distance(0mm, along: v) back
  private plane crown_back(origin: crown_back_origin, toward: crown_back_direction,
                           from: back, offset: center_y)
  in crown_back {
    private point crown_back_start
    private point crown_back_end
    crown_back_start distance(center_x, along: u) crown_back
    crown_back_start distance(0mm, along: v) crown_back
    crown_back_end distance(center_x, along: u) crown_back
    crown_back_end distance(mean_distance, along: v) crown_back
    private construction centerline line crown_back_axis(crown_back_start, crown_back_end)
    gear_outer: RoundedRackSection(crown_back, pitch_radius: center_x + section_radius + section_width,
      width: section_width, tip_height: root_depth, base_depth: base_depth,
      pressure: pressure_angle, tip_radius: transition_radius)
    gear_inner: RoundedRackSection(crown_back, pitch_radius: center_x + section_radius - section_width,
      width: section_width, tip_height: root_depth, base_depth: base_depth,
      pressure: pressure_angle, tip_radius: transition_radius)
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
  motion pinion_index(about: pinion_axis)
  motion gear_index(about: gear_axis)
  motion crown_neighbor(about: front_axis, phase: -360deg / crown_teeth)
  group pinion_design(heel: heel.wall.solid, toe: toe.wall.solid,
    tip: pinion_tip_boundary.wall.solid, back: pinion_back_boundary.wall.solid,
    generation: pinion_generation, indexing: pinion_index)
  group gear_design(heel: heel.wall.solid, toe: toe.wall.solid,
    tip: gear_tip_boundary.wall.solid, back: gear_back_boundary.wall.solid,
    generation: gear_generation, indexing: gear_index)
  // Inverse generating and neighbor motions rotate about the common apex.
  // For any blank point, radial distance from the offset crown axis is at most
  // |crown center| + heel radius. The extra module keeps this artificial cap clear.
  construction gear_space: ComplementarySpace(crown_back, crown_back_axis,
    gear_outer, gear_inner, crown_neighbor, radial_start: center_x,
    radial_end: center_x + hypot(center_x, center_y) + heel_distance + mean_module)
  // Select the crown semicircle containing the intended tooth trace. Opposed view
  // axes express that same world-space half with opposite angular coordinates.
  surface pinion_outer(pinion_crown, pinion.outer, from: 180deg, to: 360deg)
  surface pinion_outer_round(pinion_crown, pinion.outer_round, from: 180deg, to: 360deg)
  surface pinion_inner(pinion_crown, pinion.inner, from: 180deg, to: 360deg)
  surface pinion_inner_round(pinion_crown, pinion.inner_round, from: 180deg, to: 360deg)
  surface pinion_tip(pinion_crown, pinion.tip, from: 180deg, to: 360deg)
  surface gear_outer_inner(gear_outer_crown, gear_outer.inner, from: 0deg, to: 180deg)
  surface gear_outer_inner_round(gear_outer_crown, gear_outer.inner_round, from: 0deg, to: 180deg)
  surface gear_outer_tip(gear_outer_crown, gear_outer.tip, from: 0deg, to: 180deg)
  surface gear_inner_outer(gear_inner_crown, gear_inner.outer, from: 0deg, to: 180deg)
  surface gear_inner_outer_round(gear_inner_crown, gear_inner.outer_round, from: 0deg, to: 180deg)
  surface gear_inner_tip(gear_inner_crown, gear_inner.tip, from: 0deg, to: 180deg)

  // Implicit generated surfaces. The face and material boundaries are defined separately.
  envelope pinion_outer_envelope(pinion_outer, under: pinion_generation,
    from: -35deg, to: 35deg)
  envelope pinion_outer_round_envelope(pinion_outer_round, under: pinion_generation,
    from: -35deg, to: 35deg)
  envelope pinion_inner_envelope(pinion_inner, under: pinion_generation,
    from: -35deg, to: 35deg)
  envelope pinion_inner_round_envelope(pinion_inner_round, under: pinion_generation,
    from: -35deg, to: 35deg)
  envelope pinion_tip_envelope(pinion_tip, under: pinion_generation,
    from: -35deg, to: 35deg)
  envelope gear_outer_inner_envelope(gear_outer_inner, under: gear_generation,
    from: -35deg, to: 35deg)
  envelope gear_outer_inner_round_envelope(gear_outer_inner_round, under: gear_generation,
    from: -35deg, to: 35deg)
  envelope gear_outer_tip_envelope(gear_outer_tip, under: gear_generation,
    from: -35deg, to: 35deg)
  envelope gear_inner_outer_envelope(gear_inner_outer, under: gear_generation,
    from: -35deg, to: 35deg)
  envelope gear_inner_outer_round_envelope(gear_inner_outer_round, under: gear_generation,
    from: -35deg, to: 35deg)
  envelope gear_inner_tip_envelope(gear_inner_tip, under: gear_generation,
    from: -35deg, to: 35deg)

  // Each tooth surface states its material region through ordinary components.
  pinion_outer_region: ToothRegion(pinion_outer_envelope,
    tip: pinion_tip_boundary.wall.solid, root: pinion_root_boundary.wall.solid,
    toe: toe.wall.solid, heel: heel.wall.solid)
  pinion_outer_round_region: ToothRegion(pinion_outer_round_envelope,
    tip: pinion_tip_boundary.wall.solid, root: pinion_root_boundary.wall.solid,
    toe: toe.wall.solid, heel: heel.wall.solid)
  pinion_inner_region: ToothRegion(pinion_inner_envelope,
    tip: pinion_tip_boundary.wall.solid, root: pinion_root_boundary.wall.solid,
    toe: toe.wall.solid, heel: heel.wall.solid)
  pinion_inner_round_region: ToothRegion(pinion_inner_round_envelope,
    tip: pinion_tip_boundary.wall.solid, root: pinion_root_boundary.wall.solid,
    toe: toe.wall.solid, heel: heel.wall.solid)
  pinion_tip_region: ToothRegion(pinion_tip_envelope,
    tip: pinion_tip_boundary.wall.solid, root: pinion_root_boundary.wall.solid,
    toe: toe.wall.solid, heel: heel.wall.solid)
  gear_outer_inner_region: ToothRegion(gear_outer_inner_envelope,
    tip: gear_tip_boundary.wall.solid, root: gear_root_boundary.wall.solid,
    toe: toe.wall.solid, heel: heel.wall.solid)
  gear_outer_inner_round_region: ToothRegion(gear_outer_inner_round_envelope,
    tip: gear_tip_boundary.wall.solid, root: gear_root_boundary.wall.solid,
    toe: toe.wall.solid, heel: heel.wall.solid)
  gear_outer_tip_region: ToothRegion(gear_outer_tip_envelope,
    tip: gear_tip_boundary.wall.solid, root: gear_root_boundary.wall.solid,
    toe: toe.wall.solid, heel: heel.wall.solid)
  gear_inner_outer_region: ToothRegion(gear_inner_outer_envelope,
    tip: gear_tip_boundary.wall.solid, root: gear_root_boundary.wall.solid,
    toe: toe.wall.solid, heel: heel.wall.solid)
  gear_inner_outer_round_region: ToothRegion(gear_inner_outer_round_envelope,
    tip: gear_tip_boundary.wall.solid, root: gear_root_boundary.wall.solid,
    toe: toe.wall.solid, heel: heel.wall.solid)
  gear_inner_tip_region: ToothRegion(gear_inner_tip_envelope,
    tip: gear_tip_boundary.wall.solid, root: gear_root_boundary.wall.solid,
    toe: toe.wall.solid, heel: heel.wall.solid)

  // Shared characteristics at the tangent generating-profile vertices. Both faces
  // refer to one curve, including their common toe/heel and material limits.
  seam pinion_outer_join(pinion_outer_region.bounded, pinion_outer_round_region.bounded)
  seam pinion_outer_root_join(pinion_outer_round_region.bounded, pinion_tip_region.bounded)
  seam pinion_inner_join(pinion_inner_region.bounded, pinion_inner_round_region.bounded)
  seam pinion_inner_root_join(pinion_inner_round_region.bounded, pinion_tip_region.bounded)
  seam gear_outer_inner_join(gear_outer_inner_region.bounded, gear_outer_inner_round_region.bounded)
  seam gear_outer_inner_root_join(gear_outer_inner_round_region.bounded, gear_outer_tip_region.bounded)
  seam gear_inner_outer_join(gear_inner_outer_region.bounded, gear_inner_outer_round_region.bounded)
  seam gear_inner_outer_root_join(gear_inner_outer_round_region.bounded, gear_inner_tip_region.bounded)

  // Intersections with finite analytic boundaries belong to the model as well.
  // These seams retain the generated face's material conditions and source chart.
  seam pinion_outer_tip_edge(pinion_outer_region.bounded, pinion_tip_boundary.wall)
  seam pinion_outer_toe_edge(pinion_outer_region.bounded, toe.wall)
  seam pinion_outer_heel_edge(pinion_outer_region.bounded, heel.wall)
  seam pinion_outer_round_toe_edge(pinion_outer_round_region.bounded, toe.wall)
  seam pinion_outer_round_heel_edge(pinion_outer_round_region.bounded, heel.wall)
  seam pinion_inner_tip_edge(pinion_inner_region.bounded, pinion_tip_boundary.wall)
  seam pinion_inner_toe_edge(pinion_inner_region.bounded, toe.wall)
  seam pinion_inner_heel_edge(pinion_inner_region.bounded, heel.wall)
  seam pinion_inner_round_toe_edge(pinion_inner_round_region.bounded, toe.wall)
  seam pinion_inner_round_heel_edge(pinion_inner_round_region.bounded, heel.wall)
  seam gear_outer_inner_tip_edge(gear_outer_inner_region.bounded, gear_tip_boundary.wall)
  seam gear_outer_inner_toe_edge(gear_outer_inner_region.bounded, toe.wall)
  seam gear_outer_inner_heel_edge(gear_outer_inner_region.bounded, heel.wall)
  seam gear_outer_inner_round_toe_edge(gear_outer_inner_round_region.bounded, toe.wall)
  seam gear_outer_inner_round_heel_edge(gear_outer_inner_round_region.bounded, heel.wall)
  seam gear_inner_outer_tip_edge(gear_inner_outer_region.bounded, gear_tip_boundary.wall)
  seam gear_inner_outer_toe_edge(gear_inner_outer_region.bounded, toe.wall)
  seam gear_inner_outer_heel_edge(gear_inner_outer_region.bounded, heel.wall)
  seam gear_inner_outer_round_toe_edge(gear_inner_outer_round_region.bounded, toe.wall)
  seam gear_inner_outer_round_heel_edge(gear_inner_outer_round_region.bounded, heel.wall)

  // Shared corner identities. The adjacent face loops can reuse these vertices.
  // The two supported forms meet finite boundary seams or a generating junction.
  vertex pinion_outer_tip_toe(pinion_outer_tip_edge, pinion_outer_toe_edge)
  vertex pinion_outer_join_toe(pinion_outer_join, pinion_outer_toe_edge)
  vertex pinion_outer_root_toe(pinion_outer_root_join, pinion_outer_round_toe_edge)
  vertex pinion_outer_tip_heel(pinion_outer_tip_edge, pinion_outer_heel_edge)
  vertex pinion_outer_join_heel(pinion_outer_join, pinion_outer_heel_edge)
  vertex pinion_outer_root_heel(pinion_outer_root_join, pinion_outer_round_heel_edge)
  vertex pinion_inner_tip_toe(pinion_inner_tip_edge, pinion_inner_toe_edge)
  vertex pinion_inner_join_toe(pinion_inner_join, pinion_inner_toe_edge)
  vertex pinion_inner_root_toe(pinion_inner_root_join, pinion_inner_round_toe_edge)
  vertex pinion_inner_tip_heel(pinion_inner_tip_edge, pinion_inner_heel_edge)
  vertex pinion_inner_join_heel(pinion_inner_join, pinion_inner_heel_edge)
  vertex pinion_inner_root_heel(pinion_inner_root_join, pinion_inner_round_heel_edge)
  vertex gear_outer_inner_tip_toe(gear_outer_inner_tip_edge, gear_outer_inner_toe_edge)
  vertex gear_outer_inner_join_toe(gear_outer_inner_join, gear_outer_inner_toe_edge)
  vertex gear_outer_inner_root_toe(gear_outer_inner_root_join, gear_outer_inner_round_toe_edge)
  vertex gear_outer_inner_tip_heel(gear_outer_inner_tip_edge, gear_outer_inner_heel_edge)
  vertex gear_outer_inner_join_heel(gear_outer_inner_join, gear_outer_inner_heel_edge)
  vertex gear_outer_inner_root_heel(gear_outer_inner_root_join, gear_outer_inner_round_heel_edge)
  vertex gear_inner_outer_tip_toe(gear_inner_outer_tip_edge, gear_inner_outer_toe_edge)
  vertex gear_inner_outer_join_toe(gear_inner_outer_join, gear_inner_outer_toe_edge)
  vertex gear_inner_outer_root_toe(gear_inner_outer_root_join, gear_inner_outer_round_toe_edge)
  vertex gear_inner_outer_tip_heel(gear_inner_outer_tip_edge, gear_inner_outer_heel_edge)
  vertex gear_inner_outer_join_heel(gear_inner_outer_join, gear_inner_outer_heel_edge)
  vertex gear_inner_outer_root_heel(gear_inner_outer_root_join, gear_inner_outer_round_heel_edge)
  // Finite boundary extents share corner identity and use axial sections.
  edge pinion_outer_tip_span(pinion_outer_tip_edge, from: pinion_outer_tip_toe,
      to: pinion_outer_tip_heel, along: pinion_axis)
  edge pinion_outer_join_span(pinion_outer_join, from: pinion_outer_join_toe,
      to: pinion_outer_join_heel, along: pinion_axis)
  edge pinion_outer_root_span(pinion_outer_root_join, from: pinion_outer_root_toe,
      to: pinion_outer_root_heel, along: pinion_axis)
  edge pinion_outer_toe_span(pinion_outer_toe_edge, from: pinion_outer_join_toe,
      to: pinion_outer_tip_toe, along: pinion_axis)
  edge pinion_outer_round_toe_span(pinion_outer_round_toe_edge, from: pinion_outer_root_toe,
      to: pinion_outer_join_toe, along: pinion_axis)
  edge pinion_outer_heel_span(pinion_outer_heel_edge, from: pinion_outer_join_heel,
      to: pinion_outer_tip_heel, along: pinion_axis)
  edge pinion_outer_round_heel_span(pinion_outer_round_heel_edge, from: pinion_outer_root_heel,
      to: pinion_outer_join_heel, along: pinion_axis)
  edge pinion_inner_tip_span(pinion_inner_tip_edge, from: pinion_inner_tip_toe,
      to: pinion_inner_tip_heel, along: pinion_axis)
  edge pinion_inner_join_span(pinion_inner_join, from: pinion_inner_join_toe,
      to: pinion_inner_join_heel, along: pinion_axis)
  edge pinion_inner_root_span(pinion_inner_root_join, from: pinion_inner_root_toe,
      to: pinion_inner_root_heel, along: pinion_axis)
  edge pinion_inner_toe_span(pinion_inner_toe_edge, from: pinion_inner_join_toe,
      to: pinion_inner_tip_toe, along: pinion_axis)
  edge pinion_inner_round_toe_span(pinion_inner_round_toe_edge, from: pinion_inner_root_toe,
      to: pinion_inner_join_toe, along: pinion_axis)
  edge pinion_inner_heel_span(pinion_inner_heel_edge, from: pinion_inner_join_heel,
      to: pinion_inner_tip_heel, along: pinion_axis)
  edge pinion_inner_round_heel_span(pinion_inner_round_heel_edge, from: pinion_inner_root_heel,
      to: pinion_inner_join_heel, along: pinion_axis)
  edge gear_outer_inner_tip_span(gear_outer_inner_tip_edge, from: gear_outer_inner_tip_toe,
      to: gear_outer_inner_tip_heel, along: gear_axis)
  edge gear_outer_inner_join_span(gear_outer_inner_join, from: gear_outer_inner_join_toe,
      to: gear_outer_inner_join_heel, along: gear_axis)
  edge gear_outer_inner_root_span(gear_outer_inner_root_join, from: gear_outer_inner_root_toe,
      to: gear_outer_inner_root_heel, along: gear_axis)
  edge gear_outer_inner_toe_span(gear_outer_inner_toe_edge, from: gear_outer_inner_join_toe,
      to: gear_outer_inner_tip_toe, along: gear_axis)
  edge gear_outer_inner_round_toe_span(gear_outer_inner_round_toe_edge, from: gear_outer_inner_root_toe,
      to: gear_outer_inner_join_toe, along: gear_axis)
  edge gear_outer_inner_heel_span(gear_outer_inner_heel_edge, from: gear_outer_inner_join_heel,
      to: gear_outer_inner_tip_heel, along: gear_axis)
  edge gear_outer_inner_round_heel_span(gear_outer_inner_round_heel_edge, from: gear_outer_inner_root_heel,
      to: gear_outer_inner_join_heel, along: gear_axis)
  edge gear_inner_outer_tip_span(gear_inner_outer_tip_edge, from: gear_inner_outer_tip_toe,
      to: gear_inner_outer_tip_heel, along: gear_axis)
  edge gear_inner_outer_join_span(gear_inner_outer_join, from: gear_inner_outer_join_toe,
      to: gear_inner_outer_join_heel, along: gear_axis)
  edge gear_inner_outer_root_span(gear_inner_outer_root_join, from: gear_inner_outer_root_toe,
      to: gear_inner_outer_root_heel, along: gear_axis)
  edge gear_inner_outer_toe_span(gear_inner_outer_toe_edge, from: gear_inner_outer_join_toe,
      to: gear_inner_outer_tip_toe, along: gear_axis)
  edge gear_inner_outer_round_toe_span(gear_inner_outer_round_toe_edge, from: gear_inner_outer_root_toe,
      to: gear_inner_outer_join_toe, along: gear_axis)
  edge gear_inner_outer_heel_span(gear_inner_outer_heel_edge, from: gear_inner_outer_join_heel,
      to: gear_inner_outer_tip_heel, along: gear_axis)
  edge gear_inner_outer_round_heel_span(gear_inner_outer_round_heel_edge, from: gear_inner_outer_root_heel,
      to: gear_inner_outer_join_heel, along: gear_axis)
  pinion_outer_faces: ToothSideFaces(pinion_outer_region.bounded, pinion_outer_round_region.bounded,
      pinion_outer_tip_span, pinion_outer_join_span, pinion_outer_root_span,
      pinion_outer_toe_span, pinion_outer_heel_span, pinion_outer_round_toe_span, pinion_outer_round_heel_span)
  pinion_inner_faces: ToothSideFaces(pinion_inner_region.bounded, pinion_inner_round_region.bounded,
      pinion_inner_tip_span, pinion_inner_join_span, pinion_inner_root_span,
      pinion_inner_toe_span, pinion_inner_heel_span, pinion_inner_round_toe_span, pinion_inner_round_heel_span)
  gear_outer_inner_faces: ToothSideFaces(gear_outer_inner_region.bounded, gear_outer_inner_round_region.bounded,
      gear_outer_inner_tip_span, gear_outer_inner_join_span, gear_outer_inner_root_span,
      gear_outer_inner_toe_span, gear_outer_inner_heel_span, gear_outer_inner_round_toe_span, gear_outer_inner_round_heel_span)
  gear_inner_outer_faces: ToothSideFaces(gear_inner_outer_region.bounded, gear_inner_outer_round_region.bounded,
      gear_inner_outer_tip_span, gear_inner_outer_join_span, gear_inner_outer_root_span,
      gear_inner_outer_toe_span, gear_inner_outer_heel_span, gear_inner_outer_round_toe_span, gear_inner_outer_round_heel_span)
}

preview {
  unit mm
  param pinion_teeth = 24
  param gear_teeth = 48
  param mean_module = 2mm
  param datum_span = mean_module * hypot(pinion_teeth, gear_teeth) / 2
  private point back_origin
  private point back_direction hint(x: datum_span)
  back_origin coincident std.origin
  back_direction distance(datum_span, along: u) std.front
  back_direction distance(0mm, along: v) std.front
  plane back(origin: back_origin, toward: back_direction, u: (1, 0, 0), v: (0, 0, -1))
  pair: MatchedReferences(std.front, back, pinion_teeth: pinion_teeth, gear_teeth: gear_teeth,
                          mean_module: mean_module, spiral_angle: 35deg, pressure_angle: 20deg)
}
