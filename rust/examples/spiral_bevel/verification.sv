// The analytic faces of the pair's layout (layout.sv): each generated flank as an
// envelope of its crown surface, trimmed to its member's limits, with the seams,
// corners and edges its face loop is built from. The generating-system checks
// (tests/envelope/paired.rs) read these through pair.sv; the exported bodies do not.

// A surface region is the common part of these material-side conditions. Its generating
// envelope keeps its own parameter domain; branch selection and solid closure are separate.
component ToothRegion(source: envelope, limits: group) {
  patch bounded(source, inside: limits.tip, inside: limits.heel,
                outside: limits.root, outside: limits.toe)
}

// Exact end intersections share their supporting surfaces with the tooth boundaries.
component EndCircles(wall: surface, toe: surface, heel: surface) {
  seam near(wall, toe)
  seam far(wall, heel)
}

// The working flank and its root transition share one finite join edge.
// Loop order makes their uses of that edge opposite.
component ToothSideFaces(flank: patch, fillet: patch,
                         tip: edge, join: edge, root: edge,
                         toe: edge, heel: edge, round_toe: edge, round_heel: edge) {
  face working(toe, tip, heel, join, on: flank)
  face transition(round_toe, join, round_heel, root, on: fillet)
}

// `refs` is a HypoidLayout.
component ReferenceFaces(refs: group) {
  // The generating roll the envelopes are verified over. The removal sweeps in
  // members.sv declare their own, longer for the gear.
  param roll_span = 35deg
  pinion_tip_ends: EndCircles(refs.pinion_blank.tip.wall, refs.pinion_blank.toe.wall, refs.pinion_blank.heel.wall)
  pinion_root_ends: EndCircles(refs.pinion_blank.root.wall, refs.pinion_blank.toe.wall, refs.pinion_blank.heel.wall)
  pinion_back_ends: EndCircles(refs.pinion_blank.back.wall, refs.pinion_blank.toe.wall, refs.pinion_blank.heel.wall)
  gear_tip_ends: EndCircles(refs.gear_blank.tip.wall, refs.gear_blank.toe.wall, refs.gear_blank.heel.wall)
  gear_root_ends: EndCircles(refs.gear_blank.root.wall, refs.gear_blank.toe.wall, refs.gear_blank.heel.wall)
  gear_back_ends: EndCircles(refs.gear_blank.back.wall, refs.gear_blank.toe.wall, refs.gear_blank.heel.wall)

  // One generated flank per edge of each crown section's profile: the edge's crown
  // surface, on the semicircle about the cutter's axis holding the tooth trace (every
  // section is drawn in the normal view through M, so the trace stands at 180deg);
  // its implicit envelope under the generating roll; and the envelope's material
  // region, through an ordinary component. The face and material boundaries are
  // defined separately. Copy k holds the profile's k-th edge, in traversal order.
  repeat e in refs.tooth.rack.profile {
    surface pinion(refs.tooth.crown, e, from: 90deg, to: 270deg)
    envelope pinion_envelope(pinion, under: refs.generation.pinion_generation,
      from: -roll_span, to: roll_span)
    pinion_region: ToothRegion(pinion_envelope, limits: refs.pinion_design)
  }
  repeat e in refs.mate.outer.profile {
    surface gear_outer(refs.mate.outer_crown, e, from: 90deg, to: 270deg)
    envelope gear_outer_envelope(gear_outer, under: refs.generation.gear_generation,
      from: -roll_span, to: roll_span)
    gear_outer_region: ToothRegion(gear_outer_envelope, limits: refs.gear_design)
  }
  repeat e in refs.mate.inner.profile {
    surface gear_inner(refs.mate.inner_crown, e, from: 90deg, to: 270deg)
    envelope gear_inner_envelope(gear_inner, under: refs.generation.gear_generation,
      from: -roll_span, to: roll_span)
    gear_inner_region: ToothRegion(gear_inner_envelope, limits: refs.gear_design)
  }
  // The flanks the checks read, by where each stands in its profile: the tooth's
  // (RackSection: base, outer, outer_round, tip, inner_round, inner) and its mate's,
  // which walks the other way round (MateSection: base, inner, inner_round, tip,
  // outer_round, outer). The bases, and each mate section's far side, are made with
  // the rest and read by nothing below.
  param outer = 1
  param outer_round = 2
  param tip = 3
  param inner_round = 4
  param inner = 5
  param mate_inner = 1
  param mate_inner_round = 2
  param mate_outer_round = 4
  param mate_outer = 5

  // Shared characteristics at the tangent generating-profile vertices. Both faces
  // refer to one curve, including their common toe and heel and material limits.
  seam pinion_outer_join(pinion_region[outer].bounded, pinion_region[outer_round].bounded)
  seam pinion_outer_root_join(pinion_region[outer_round].bounded, pinion_region[tip].bounded)
  seam pinion_inner_join(pinion_region[inner].bounded, pinion_region[inner_round].bounded)
  seam pinion_inner_root_join(pinion_region[inner_round].bounded, pinion_region[tip].bounded)
  seam gear_outer_inner_join(gear_outer_region[mate_inner].bounded, gear_outer_region[mate_inner_round].bounded)
  seam gear_outer_inner_root_join(gear_outer_region[mate_inner_round].bounded, gear_outer_region[tip].bounded)
  seam gear_inner_outer_join(gear_inner_region[mate_outer].bounded, gear_inner_region[mate_outer_round].bounded)
  seam gear_inner_outer_root_join(gear_inner_region[mate_outer_round].bounded, gear_inner_region[tip].bounded)

  // Intersections with finite analytic boundaries belong to the model as well.
  // These seams retain the generated face's material conditions and source chart.
  seam pinion_outer_tip_edge(pinion_region[outer].bounded, refs.pinion_blank.tip.wall)
  seam pinion_outer_toe_edge(pinion_region[outer].bounded, refs.pinion_blank.toe.wall)
  seam pinion_outer_heel_edge(pinion_region[outer].bounded, refs.pinion_blank.heel.wall)
  seam pinion_outer_round_toe_edge(pinion_region[outer_round].bounded, refs.pinion_blank.toe.wall)
  seam pinion_outer_round_heel_edge(pinion_region[outer_round].bounded, refs.pinion_blank.heel.wall)
  seam pinion_inner_tip_edge(pinion_region[inner].bounded, refs.pinion_blank.tip.wall)
  seam pinion_inner_toe_edge(pinion_region[inner].bounded, refs.pinion_blank.toe.wall)
  seam pinion_inner_heel_edge(pinion_region[inner].bounded, refs.pinion_blank.heel.wall)
  seam pinion_inner_round_toe_edge(pinion_region[inner_round].bounded, refs.pinion_blank.toe.wall)
  seam pinion_inner_round_heel_edge(pinion_region[inner_round].bounded, refs.pinion_blank.heel.wall)
  seam gear_outer_inner_tip_edge(gear_outer_region[mate_inner].bounded, refs.gear_blank.tip.wall)
  seam gear_outer_inner_toe_edge(gear_outer_region[mate_inner].bounded, refs.gear_blank.toe.wall)
  seam gear_outer_inner_heel_edge(gear_outer_region[mate_inner].bounded, refs.gear_blank.heel.wall)
  seam gear_outer_inner_round_toe_edge(gear_outer_region[mate_inner_round].bounded, refs.gear_blank.toe.wall)
  seam gear_outer_inner_round_heel_edge(gear_outer_region[mate_inner_round].bounded, refs.gear_blank.heel.wall)
  seam gear_inner_outer_tip_edge(gear_inner_region[mate_outer].bounded, refs.gear_blank.tip.wall)
  seam gear_inner_outer_toe_edge(gear_inner_region[mate_outer].bounded, refs.gear_blank.toe.wall)
  seam gear_inner_outer_heel_edge(gear_inner_region[mate_outer].bounded, refs.gear_blank.heel.wall)
  seam gear_inner_outer_round_toe_edge(gear_inner_region[mate_outer_round].bounded, refs.gear_blank.toe.wall)
  seam gear_inner_outer_round_heel_edge(gear_inner_region[mate_outer_round].bounded, refs.gear_blank.heel.wall)

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
      to: pinion_outer_tip_heel, along: refs.pinion.axis)
  edge pinion_outer_join_span(pinion_outer_join, from: pinion_outer_join_toe,
      to: pinion_outer_join_heel, along: refs.pinion.axis)
  edge pinion_outer_root_span(pinion_outer_root_join, from: pinion_outer_root_toe,
      to: pinion_outer_root_heel, along: refs.pinion.axis)
  edge pinion_outer_toe_span(pinion_outer_toe_edge, from: pinion_outer_join_toe,
      to: pinion_outer_tip_toe, along: refs.pinion.axis)
  edge pinion_outer_round_toe_span(pinion_outer_round_toe_edge, from: pinion_outer_root_toe,
      to: pinion_outer_join_toe, along: refs.pinion.axis)
  edge pinion_outer_heel_span(pinion_outer_heel_edge, from: pinion_outer_join_heel,
      to: pinion_outer_tip_heel, along: refs.pinion.axis)
  edge pinion_outer_round_heel_span(pinion_outer_round_heel_edge, from: pinion_outer_root_heel,
      to: pinion_outer_join_heel, along: refs.pinion.axis)
  edge pinion_inner_tip_span(pinion_inner_tip_edge, from: pinion_inner_tip_toe,
      to: pinion_inner_tip_heel, along: refs.pinion.axis)
  edge pinion_inner_join_span(pinion_inner_join, from: pinion_inner_join_toe,
      to: pinion_inner_join_heel, along: refs.pinion.axis)
  edge pinion_inner_root_span(pinion_inner_root_join, from: pinion_inner_root_toe,
      to: pinion_inner_root_heel, along: refs.pinion.axis)
  edge pinion_inner_toe_span(pinion_inner_toe_edge, from: pinion_inner_join_toe,
      to: pinion_inner_tip_toe, along: refs.pinion.axis)
  edge pinion_inner_round_toe_span(pinion_inner_round_toe_edge, from: pinion_inner_root_toe,
      to: pinion_inner_join_toe, along: refs.pinion.axis)
  edge pinion_inner_heel_span(pinion_inner_heel_edge, from: pinion_inner_join_heel,
      to: pinion_inner_tip_heel, along: refs.pinion.axis)
  edge pinion_inner_round_heel_span(pinion_inner_round_heel_edge, from: pinion_inner_root_heel,
      to: pinion_inner_join_heel, along: refs.pinion.axis)
  edge gear_outer_inner_tip_span(gear_outer_inner_tip_edge, from: gear_outer_inner_tip_toe,
      to: gear_outer_inner_tip_heel, along: refs.gear.axis)
  edge gear_outer_inner_join_span(gear_outer_inner_join, from: gear_outer_inner_join_toe,
      to: gear_outer_inner_join_heel, along: refs.gear.axis)
  edge gear_outer_inner_root_span(gear_outer_inner_root_join, from: gear_outer_inner_root_toe,
      to: gear_outer_inner_root_heel, along: refs.gear.axis)
  edge gear_outer_inner_toe_span(gear_outer_inner_toe_edge, from: gear_outer_inner_join_toe,
      to: gear_outer_inner_tip_toe, along: refs.gear.axis)
  edge gear_outer_inner_round_toe_span(gear_outer_inner_round_toe_edge, from: gear_outer_inner_root_toe,
      to: gear_outer_inner_join_toe, along: refs.gear.axis)
  edge gear_outer_inner_heel_span(gear_outer_inner_heel_edge, from: gear_outer_inner_join_heel,
      to: gear_outer_inner_tip_heel, along: refs.gear.axis)
  edge gear_outer_inner_round_heel_span(gear_outer_inner_round_heel_edge, from: gear_outer_inner_root_heel,
      to: gear_outer_inner_join_heel, along: refs.gear.axis)
  edge gear_inner_outer_tip_span(gear_inner_outer_tip_edge, from: gear_inner_outer_tip_toe,
      to: gear_inner_outer_tip_heel, along: refs.gear.axis)
  edge gear_inner_outer_join_span(gear_inner_outer_join, from: gear_inner_outer_join_toe,
      to: gear_inner_outer_join_heel, along: refs.gear.axis)
  edge gear_inner_outer_root_span(gear_inner_outer_root_join, from: gear_inner_outer_root_toe,
      to: gear_inner_outer_root_heel, along: refs.gear.axis)
  edge gear_inner_outer_toe_span(gear_inner_outer_toe_edge, from: gear_inner_outer_join_toe,
      to: gear_inner_outer_tip_toe, along: refs.gear.axis)
  edge gear_inner_outer_round_toe_span(gear_inner_outer_round_toe_edge, from: gear_inner_outer_root_toe,
      to: gear_inner_outer_join_toe, along: refs.gear.axis)
  edge gear_inner_outer_heel_span(gear_inner_outer_heel_edge, from: gear_inner_outer_join_heel,
      to: gear_inner_outer_tip_heel, along: refs.gear.axis)
  edge gear_inner_outer_round_heel_span(gear_inner_outer_round_heel_edge, from: gear_inner_outer_root_heel,
      to: gear_inner_outer_join_heel, along: refs.gear.axis)
  pinion_outer_faces: ToothSideFaces(pinion_region[outer].bounded, pinion_region[outer_round].bounded,
      pinion_outer_tip_span, pinion_outer_join_span, pinion_outer_root_span,
      pinion_outer_toe_span, pinion_outer_heel_span, pinion_outer_round_toe_span, pinion_outer_round_heel_span)
  pinion_inner_faces: ToothSideFaces(pinion_region[inner].bounded, pinion_region[inner_round].bounded,
      pinion_inner_tip_span, pinion_inner_join_span, pinion_inner_root_span,
      pinion_inner_toe_span, pinion_inner_heel_span, pinion_inner_round_toe_span, pinion_inner_round_heel_span)
  gear_outer_inner_faces: ToothSideFaces(gear_outer_region[mate_inner].bounded, gear_outer_region[mate_inner_round].bounded,
      gear_outer_inner_tip_span, gear_outer_inner_join_span, gear_outer_inner_root_span,
      gear_outer_inner_toe_span, gear_outer_inner_heel_span, gear_outer_inner_round_toe_span, gear_outer_inner_round_heel_span)
  gear_inner_outer_faces: ToothSideFaces(gear_inner_region[mate_outer].bounded, gear_inner_region[mate_outer_round].bounded,
      gear_inner_outer_tip_span, gear_inner_outer_join_span, gear_inner_outer_root_span,
      gear_inner_outer_toe_span, gear_inner_outer_heel_span, gear_inner_outer_round_toe_span, gear_inner_outer_round_heel_span)
}
