// The spiral bevel's crown tooth section as it was drawn before the crown became a region about
// the cutter's axis (#214): a six-edge chain revolved about a line, its tip's corners rounded by
// tangent arcs. Kept as a drawn revolution the envelope, field and fillet checks
// read; the example's crown is `spiral_bevel/hypoid.sv`'s `Tooth`.
//
// Straight flanks at the pressure angle split by the shift, a base and a tip at their depths,
// and the tip roundings; revolved about the cutter's axis the flanks are cones and the roundings
// tori. Each flank stands a quarter of the backlash outside the line through its pitch point.
use std

component RackSection(lp: point, rp: point, design: group, normal_module: Length) {
  // Seeds only, rough, in pitch widths (the `pitch` line's run, half a normal pitch, so a
  // normal module is about 2 / pi of one). Each flank runs from its pitch point, square to the
  // pitch line and leaning by its pressure angle, to the base's depth and to its round's; each
  // round's centre stands a rounding inside its flank's end, and the tip's end a rounding above.
  outer_angle := design.pressure - design.shift
  inner_angle := design.pressure + design.shift
  base_depth := design.base * 2 / pi
  join_depth := (design.dedendum - design.rounding) * 2 / pi
  corner := design.rounding * 2 / pi
  construction pitch := line(lp, rp)
  private bi := point hint(at: lp, along: pitch, by: base_depth / cos(inner_angle),
                           turn: -90deg - inner_angle)
  private bo := point hint(at: rp, along: pitch, by: base_depth / cos(outer_angle),
                           turn: outer_angle - 90deg)
  private oj := point hint(at: rp, along: pitch, by: join_depth / cos(outer_angle),
                           turn: 90deg + outer_angle)
  private ij := point hint(at: lp, along: pitch, by: join_depth / cos(inner_angle),
                           turn: 90deg - inner_angle)
  private co := point hint(at: oj, along: pitch, by: -corner)
  private ci := point hint(at: ij, along: pitch, by: corner)
  private ot := point hint(at: co, along: pitch, by: corner, turn: 90deg)
  private it := point hint(at: ci, along: pitch, by: corner, turn: 90deg)
  profile := (base := line(bi, bo)) -> (outer := line(bo, oj)) -> tangent
             (outer_round := arc(center: co)) -> tangent (tip := line(ot, it)) -> tangent
             (inner_round := arc(center: ci)) -> tangent (inner := line(ij, bi)) -> close
  base angle(90deg + design.pressure - design.shift) outer
  base angle(270deg - design.pressure - design.shift) inner
  // Each flank a quarter of the backlash outside its shared line.
  lp distance(design.backlash / 4, side: left) inner
  rp distance(design.backlash / 4, side: left) outer
  // The tip is a land: the rounds meeting in a point satisfy the section's rows too, and are
  // not the section (§9.6). A hundredth of a module; the narrowest land, the configured
  // hypoid's, is a sixth.
  length(>= 0.01 * design.module) tip
  // The base and the tip at their depths, and the tip's two rounds of one radius.
  base parallel pitch
  base angle(180deg) tip
  pitch.p1 distance(design.base * normal_module, side: left) base
  pitch.p1 distance(design.dedendum * normal_module, side: left) tip
  radius(design.rounding * normal_module) outer_round
  outer_round equal inner_round
}

preview {
  unit mm
  // The 24:48 pair's crown at module 2, symmetric, revolved about the page's y axis at eight
  // tenths of the cone distance.
  pitch_radius := 0.8 * 2mm * hypot(24, 48) / 2
  proportions := {pressure: 20deg, shift: 0deg, module: 2mm,
                       base: 1, dedendum: 1, rounding: 0.3, backlash: 0mm}
  in std.front {
    lp := point
    rp := point
    fix((pitch_radius - 1.3mm, 0)) lp
    fix((pitch_radius + 1.3mm, 0)) rp
    rack := RackSection(lp, rp, proportions, normal_module: 2mm)
    construction centerline ax := line(std.origin)
    fix((0, 1)) ax.p2
  }
  crown := solid(rack.profile, about: ax)
  outer := surface(crown, rack.outer)
  outer_round := surface(crown, rack.outer_round)
  inner := surface(crown, rack.inner)
  inner_round := surface(crown, rack.inner_round)
  tip := surface(crown, rack.tip)
}
