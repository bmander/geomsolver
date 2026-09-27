// The analytic faces of the reference pair: each generated flank as an envelope
// of its crown surface, trimmed to its member's limits, with the seams, corners
// and edges its face loop is built from. The generating-system checks
// (tests/envelope/paired.rs) read these; the exported bodies do not.
use boundaries

component ReferenceFaces(refs: group) {
  // The generating roll the envelopes are verified over. The removal sweeps in
  // matched_pair.sv declare their own, longer for the gear.
  param roll_span = 35deg
  pinion_tip_ends: EndCircles(refs.pinion_tip_boundary.wall, refs.pinion_toe.wall, refs.pinion_heel.wall)
  pinion_root_ends: EndCircles(refs.pinion_root_boundary.wall, refs.pinion_toe.wall, refs.pinion_heel.wall)
  pinion_back_ends: EndCircles(refs.pinion_back_boundary.wall, refs.pinion_toe.wall, refs.pinion_heel.wall)
  gear_tip_ends: EndCircles(refs.gear_tip_boundary.wall, refs.toe.wall, refs.heel.wall)
  gear_root_ends: EndCircles(refs.gear_root_boundary.wall, refs.toe.wall, refs.heel.wall)
  gear_back_ends: EndCircles(refs.gear_back_boundary.wall, refs.toe.wall, refs.heel.wall)

  // One generated flank per edge of each rack section's profile: the edge's crown
  // surface, on the semicircle holding the intended tooth trace (opposed view axes
  // express that same world-space half with opposite angular coordinates); its
  // implicit envelope under the generating roll; and the envelope's material
  // region, through an ordinary component. The face and material boundaries are
  // defined separately. Copy k holds the profile's k-th edge, in traversal order.
  repeat e in refs.pinion.profile {
    surface pinion(refs.pinion_crown, e, from: 180deg, to: 360deg)
    envelope pinion_envelope(pinion, under: refs.pinion_generation,
      from: -roll_span, to: roll_span)
    pinion_region: ToothRegion(pinion_envelope, limits: refs.pinion_design)
  }
  repeat e in refs.gear_outer.profile {
    surface gear_outer(refs.gear_outer_crown, e, from: 0deg, to: 180deg)
    envelope gear_outer_envelope(gear_outer, under: refs.gear_generation,
      from: -roll_span, to: roll_span)
    gear_outer_region: ToothRegion(gear_outer_envelope, limits: refs.gear_design)
  }
  repeat e in refs.gear_inner.profile {
    surface gear_inner(refs.gear_inner_crown, e, from: 0deg, to: 180deg)
    envelope gear_inner_envelope(gear_inner, under: refs.gear_generation,
      from: -roll_span, to: roll_span)
    gear_inner_region: ToothRegion(gear_inner_envelope, limits: refs.gear_design)
  }
  // The flanks the checks read, by where each stands in the profile
  // (RoundedRackSection: base, outer, outer_round, tip, inner_round, inner). The
  // base's, and a gear section's far side's, are made with the rest and read by
  // nothing below.
  param outer = 1
  param outer_round = 2
  param tip = 3
  param inner_round = 4
  param inner = 5

  // Shared characteristics at the tangent generating-profile vertices. Both faces
  // refer to one curve, including their common refs.toe/refs.heel and material limits.
  seam pinion_outer_join(pinion_region[outer].bounded, pinion_region[outer_round].bounded)
  seam pinion_outer_root_join(pinion_region[outer_round].bounded, pinion_region[tip].bounded)
  seam pinion_inner_join(pinion_region[inner].bounded, pinion_region[inner_round].bounded)
  seam pinion_inner_root_join(pinion_region[inner_round].bounded, pinion_region[tip].bounded)
  seam gear_outer_inner_join(gear_outer_region[inner].bounded, gear_outer_region[inner_round].bounded)
  seam gear_outer_inner_root_join(gear_outer_region[inner_round].bounded, gear_outer_region[tip].bounded)
  seam gear_inner_outer_join(gear_inner_region[outer].bounded, gear_inner_region[outer_round].bounded)
  seam gear_inner_outer_root_join(gear_inner_region[outer_round].bounded, gear_inner_region[tip].bounded)

  // Intersections with finite analytic boundaries belong to the model as well.
  // These seams retain the generated face's material conditions and source chart.
  seam pinion_outer_tip_edge(pinion_region[outer].bounded, refs.pinion_tip_boundary.wall)
  seam pinion_outer_toe_edge(pinion_region[outer].bounded, refs.pinion_toe.wall)
  seam pinion_outer_heel_edge(pinion_region[outer].bounded, refs.pinion_heel.wall)
  seam pinion_outer_round_toe_edge(pinion_region[outer_round].bounded, refs.pinion_toe.wall)
  seam pinion_outer_round_heel_edge(pinion_region[outer_round].bounded, refs.pinion_heel.wall)
  seam pinion_inner_tip_edge(pinion_region[inner].bounded, refs.pinion_tip_boundary.wall)
  seam pinion_inner_toe_edge(pinion_region[inner].bounded, refs.pinion_toe.wall)
  seam pinion_inner_heel_edge(pinion_region[inner].bounded, refs.pinion_heel.wall)
  seam pinion_inner_round_toe_edge(pinion_region[inner_round].bounded, refs.pinion_toe.wall)
  seam pinion_inner_round_heel_edge(pinion_region[inner_round].bounded, refs.pinion_heel.wall)
  seam gear_outer_inner_tip_edge(gear_outer_region[inner].bounded, refs.gear_tip_boundary.wall)
  seam gear_outer_inner_toe_edge(gear_outer_region[inner].bounded, refs.toe.wall)
  seam gear_outer_inner_heel_edge(gear_outer_region[inner].bounded, refs.heel.wall)
  seam gear_outer_inner_round_toe_edge(gear_outer_region[inner_round].bounded, refs.toe.wall)
  seam gear_outer_inner_round_heel_edge(gear_outer_region[inner_round].bounded, refs.heel.wall)
  seam gear_inner_outer_tip_edge(gear_inner_region[outer].bounded, refs.gear_tip_boundary.wall)
  seam gear_inner_outer_toe_edge(gear_inner_region[outer].bounded, refs.toe.wall)
  seam gear_inner_outer_heel_edge(gear_inner_region[outer].bounded, refs.heel.wall)
  seam gear_inner_outer_round_toe_edge(gear_inner_region[outer_round].bounded, refs.toe.wall)
  seam gear_inner_outer_round_heel_edge(gear_inner_region[outer_round].bounded, refs.heel.wall)

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
  pinion_outer_faces: ToothSideFaces(pinion_region[outer].bounded, pinion_region[outer_round].bounded,
      pinion_outer_tip_span, pinion_outer_join_span, pinion_outer_root_span,
      pinion_outer_toe_span, pinion_outer_heel_span, pinion_outer_round_toe_span, pinion_outer_round_heel_span)
  pinion_inner_faces: ToothSideFaces(pinion_region[inner].bounded, pinion_region[inner_round].bounded,
      pinion_inner_tip_span, pinion_inner_join_span, pinion_inner_root_span,
      pinion_inner_toe_span, pinion_inner_heel_span, pinion_inner_round_toe_span, pinion_inner_round_heel_span)
  gear_outer_inner_faces: ToothSideFaces(gear_outer_region[inner].bounded, gear_outer_region[inner_round].bounded,
      gear_outer_inner_tip_span, gear_outer_inner_join_span, gear_outer_inner_root_span,
      gear_outer_inner_toe_span, gear_outer_inner_heel_span, gear_outer_inner_round_toe_span, gear_outer_inner_round_heel_span)
  gear_inner_outer_faces: ToothSideFaces(gear_inner_region[outer].bounded, gear_inner_region[outer_round].bounded,
      gear_inner_outer_tip_span, gear_inner_outer_join_span, gear_inner_outer_root_span,
      gear_inner_outer_toe_span, gear_inner_outer_heel_span, gear_inner_outer_round_toe_span, gear_inner_outer_round_heel_span)
}
