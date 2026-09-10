// The analytic faces of the reference pair: each generated flank as an envelope
// of its crown surface, trimmed to its member's limits, with the seams, corners
// and edges its face loop is built from. The generating-system checks
// (tests/envelope/paired.rs) read these; the exported bodies do not.
use boundaries

component ReferenceFaces(refs: group) {
  // The generating roll the envelopes are verified over. The removal sweeps in
  // matched_pair.sv declare their own, longer for the gear.
  param roll_span = 35deg
  pinion_tip_ends: EndCircles(refs.pinion_tip_boundary.wall, refs.toe.wall, refs.heel.wall)
  pinion_root_ends: EndCircles(refs.pinion_root_boundary.wall, refs.toe.wall, refs.heel.wall)
  pinion_back_ends: EndCircles(refs.pinion_back_boundary.wall, refs.toe.wall, refs.heel.wall)
  gear_tip_ends: EndCircles(refs.gear_tip_boundary.wall, refs.toe.wall, refs.heel.wall)
  gear_root_ends: EndCircles(refs.gear_root_boundary.wall, refs.toe.wall, refs.heel.wall)
  gear_back_ends: EndCircles(refs.gear_back_boundary.wall, refs.toe.wall, refs.heel.wall)

  // Select the crown semicircle containing the intended tooth trace. Opposed view
  // axes express that same world-space half with opposite angular coordinates.
  surface pinion_outer(refs.pinion_crown, refs.pinion.outer, from: 180deg, to: 360deg)
  surface pinion_outer_round(refs.pinion_crown, refs.pinion.outer_round, from: 180deg, to: 360deg)
  surface pinion_inner(refs.pinion_crown, refs.pinion.inner, from: 180deg, to: 360deg)
  surface pinion_inner_round(refs.pinion_crown, refs.pinion.inner_round, from: 180deg, to: 360deg)
  surface pinion_tip(refs.pinion_crown, refs.pinion.tip, from: 180deg, to: 360deg)
  surface gear_outer_inner(refs.gear_outer_crown, refs.gear_outer.inner, from: 0deg, to: 180deg)
  surface gear_outer_inner_round(refs.gear_outer_crown, refs.gear_outer.inner_round, from: 0deg, to: 180deg)
  surface gear_outer_tip(refs.gear_outer_crown, refs.gear_outer.tip, from: 0deg, to: 180deg)
  surface gear_inner_outer(refs.gear_inner_crown, refs.gear_inner.outer, from: 0deg, to: 180deg)
  surface gear_inner_outer_round(refs.gear_inner_crown, refs.gear_inner.outer_round, from: 0deg, to: 180deg)
  surface gear_inner_tip(refs.gear_inner_crown, refs.gear_inner.tip, from: 0deg, to: 180deg)

  // Implicit generated surfaces. The face and material boundaries are defined separately.
  envelope pinion_outer_envelope(pinion_outer, under: refs.pinion_generation,
    from: -roll_span, to: roll_span)
  envelope pinion_outer_round_envelope(pinion_outer_round, under: refs.pinion_generation,
    from: -roll_span, to: roll_span)
  envelope pinion_inner_envelope(pinion_inner, under: refs.pinion_generation,
    from: -roll_span, to: roll_span)
  envelope pinion_inner_round_envelope(pinion_inner_round, under: refs.pinion_generation,
    from: -roll_span, to: roll_span)
  envelope pinion_tip_envelope(pinion_tip, under: refs.pinion_generation,
    from: -roll_span, to: roll_span)
  envelope gear_outer_inner_envelope(gear_outer_inner, under: refs.gear_generation,
    from: -roll_span, to: roll_span)
  envelope gear_outer_inner_round_envelope(gear_outer_inner_round, under: refs.gear_generation,
    from: -roll_span, to: roll_span)
  envelope gear_outer_tip_envelope(gear_outer_tip, under: refs.gear_generation,
    from: -roll_span, to: roll_span)
  envelope gear_inner_outer_envelope(gear_inner_outer, under: refs.gear_generation,
    from: -roll_span, to: roll_span)
  envelope gear_inner_outer_round_envelope(gear_inner_outer_round, under: refs.gear_generation,
    from: -roll_span, to: roll_span)
  envelope gear_inner_tip_envelope(gear_inner_tip, under: refs.gear_generation,
    from: -roll_span, to: roll_span)

  // Each tooth surface states its material region through ordinary components.
  pinion_outer_region: ToothRegion(pinion_outer_envelope, limits: refs.pinion_design)
  pinion_outer_round_region: ToothRegion(pinion_outer_round_envelope, limits: refs.pinion_design)
  pinion_inner_region: ToothRegion(pinion_inner_envelope, limits: refs.pinion_design)
  pinion_inner_round_region: ToothRegion(pinion_inner_round_envelope, limits: refs.pinion_design)
  pinion_tip_region: ToothRegion(pinion_tip_envelope, limits: refs.pinion_design)
  gear_outer_inner_region: ToothRegion(gear_outer_inner_envelope, limits: refs.gear_design)
  gear_outer_inner_round_region: ToothRegion(gear_outer_inner_round_envelope, limits: refs.gear_design)
  gear_outer_tip_region: ToothRegion(gear_outer_tip_envelope, limits: refs.gear_design)
  gear_inner_outer_region: ToothRegion(gear_inner_outer_envelope, limits: refs.gear_design)
  gear_inner_outer_round_region: ToothRegion(gear_inner_outer_round_envelope, limits: refs.gear_design)
  gear_inner_tip_region: ToothRegion(gear_inner_tip_envelope, limits: refs.gear_design)

  // Shared characteristics at the tangent generating-profile vertices. Both faces
  // refer to one curve, including their common refs.toe/refs.heel and material limits.
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
  seam pinion_outer_tip_edge(pinion_outer_region.bounded, refs.pinion_tip_boundary.wall)
  seam pinion_outer_toe_edge(pinion_outer_region.bounded, refs.toe.wall)
  seam pinion_outer_heel_edge(pinion_outer_region.bounded, refs.heel.wall)
  seam pinion_outer_round_toe_edge(pinion_outer_round_region.bounded, refs.toe.wall)
  seam pinion_outer_round_heel_edge(pinion_outer_round_region.bounded, refs.heel.wall)
  seam pinion_inner_tip_edge(pinion_inner_region.bounded, refs.pinion_tip_boundary.wall)
  seam pinion_inner_toe_edge(pinion_inner_region.bounded, refs.toe.wall)
  seam pinion_inner_heel_edge(pinion_inner_region.bounded, refs.heel.wall)
  seam pinion_inner_round_toe_edge(pinion_inner_round_region.bounded, refs.toe.wall)
  seam pinion_inner_round_heel_edge(pinion_inner_round_region.bounded, refs.heel.wall)
  seam gear_outer_inner_tip_edge(gear_outer_inner_region.bounded, refs.gear_tip_boundary.wall)
  seam gear_outer_inner_toe_edge(gear_outer_inner_region.bounded, refs.toe.wall)
  seam gear_outer_inner_heel_edge(gear_outer_inner_region.bounded, refs.heel.wall)
  seam gear_outer_inner_round_toe_edge(gear_outer_inner_round_region.bounded, refs.toe.wall)
  seam gear_outer_inner_round_heel_edge(gear_outer_inner_round_region.bounded, refs.heel.wall)
  seam gear_inner_outer_tip_edge(gear_inner_outer_region.bounded, refs.gear_tip_boundary.wall)
  seam gear_inner_outer_toe_edge(gear_inner_outer_region.bounded, refs.toe.wall)
  seam gear_inner_outer_heel_edge(gear_inner_outer_region.bounded, refs.heel.wall)
  seam gear_inner_outer_round_toe_edge(gear_inner_outer_round_region.bounded, refs.toe.wall)
  seam gear_inner_outer_round_heel_edge(gear_inner_outer_round_region.bounded, refs.heel.wall)

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
      to: pinion_outer_tip_heel, along: refs.pinion_axis)
  edge pinion_outer_join_span(pinion_outer_join, from: pinion_outer_join_toe,
      to: pinion_outer_join_heel, along: refs.pinion_axis)
  edge pinion_outer_root_span(pinion_outer_root_join, from: pinion_outer_root_toe,
      to: pinion_outer_root_heel, along: refs.pinion_axis)
  edge pinion_outer_toe_span(pinion_outer_toe_edge, from: pinion_outer_join_toe,
      to: pinion_outer_tip_toe, along: refs.pinion_axis)
  edge pinion_outer_round_toe_span(pinion_outer_round_toe_edge, from: pinion_outer_root_toe,
      to: pinion_outer_join_toe, along: refs.pinion_axis)
  edge pinion_outer_heel_span(pinion_outer_heel_edge, from: pinion_outer_join_heel,
      to: pinion_outer_tip_heel, along: refs.pinion_axis)
  edge pinion_outer_round_heel_span(pinion_outer_round_heel_edge, from: pinion_outer_root_heel,
      to: pinion_outer_join_heel, along: refs.pinion_axis)
  edge pinion_inner_tip_span(pinion_inner_tip_edge, from: pinion_inner_tip_toe,
      to: pinion_inner_tip_heel, along: refs.pinion_axis)
  edge pinion_inner_join_span(pinion_inner_join, from: pinion_inner_join_toe,
      to: pinion_inner_join_heel, along: refs.pinion_axis)
  edge pinion_inner_root_span(pinion_inner_root_join, from: pinion_inner_root_toe,
      to: pinion_inner_root_heel, along: refs.pinion_axis)
  edge pinion_inner_toe_span(pinion_inner_toe_edge, from: pinion_inner_join_toe,
      to: pinion_inner_tip_toe, along: refs.pinion_axis)
  edge pinion_inner_round_toe_span(pinion_inner_round_toe_edge, from: pinion_inner_root_toe,
      to: pinion_inner_join_toe, along: refs.pinion_axis)
  edge pinion_inner_heel_span(pinion_inner_heel_edge, from: pinion_inner_join_heel,
      to: pinion_inner_tip_heel, along: refs.pinion_axis)
  edge pinion_inner_round_heel_span(pinion_inner_round_heel_edge, from: pinion_inner_root_heel,
      to: pinion_inner_join_heel, along: refs.pinion_axis)
  edge gear_outer_inner_tip_span(gear_outer_inner_tip_edge, from: gear_outer_inner_tip_toe,
      to: gear_outer_inner_tip_heel, along: refs.gear_axis)
  edge gear_outer_inner_join_span(gear_outer_inner_join, from: gear_outer_inner_join_toe,
      to: gear_outer_inner_join_heel, along: refs.gear_axis)
  edge gear_outer_inner_root_span(gear_outer_inner_root_join, from: gear_outer_inner_root_toe,
      to: gear_outer_inner_root_heel, along: refs.gear_axis)
  edge gear_outer_inner_toe_span(gear_outer_inner_toe_edge, from: gear_outer_inner_join_toe,
      to: gear_outer_inner_tip_toe, along: refs.gear_axis)
  edge gear_outer_inner_round_toe_span(gear_outer_inner_round_toe_edge, from: gear_outer_inner_root_toe,
      to: gear_outer_inner_join_toe, along: refs.gear_axis)
  edge gear_outer_inner_heel_span(gear_outer_inner_heel_edge, from: gear_outer_inner_join_heel,
      to: gear_outer_inner_tip_heel, along: refs.gear_axis)
  edge gear_outer_inner_round_heel_span(gear_outer_inner_round_heel_edge, from: gear_outer_inner_root_heel,
      to: gear_outer_inner_join_heel, along: refs.gear_axis)
  edge gear_inner_outer_tip_span(gear_inner_outer_tip_edge, from: gear_inner_outer_tip_toe,
      to: gear_inner_outer_tip_heel, along: refs.gear_axis)
  edge gear_inner_outer_join_span(gear_inner_outer_join, from: gear_inner_outer_join_toe,
      to: gear_inner_outer_join_heel, along: refs.gear_axis)
  edge gear_inner_outer_root_span(gear_inner_outer_root_join, from: gear_inner_outer_root_toe,
      to: gear_inner_outer_root_heel, along: refs.gear_axis)
  edge gear_inner_outer_toe_span(gear_inner_outer_toe_edge, from: gear_inner_outer_join_toe,
      to: gear_inner_outer_tip_toe, along: refs.gear_axis)
  edge gear_inner_outer_round_toe_span(gear_inner_outer_round_toe_edge, from: gear_inner_outer_root_toe,
      to: gear_inner_outer_join_toe, along: refs.gear_axis)
  edge gear_inner_outer_heel_span(gear_inner_outer_heel_edge, from: gear_inner_outer_join_heel,
      to: gear_inner_outer_tip_heel, along: refs.gear_axis)
  edge gear_inner_outer_round_heel_span(gear_inner_outer_round_heel_edge, from: gear_inner_outer_root_heel,
      to: gear_inner_outer_join_heel, along: refs.gear_axis)
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
