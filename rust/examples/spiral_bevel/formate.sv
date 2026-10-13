// The hypoid pair of `hypoid.sv` with no generating crown in its pinion: a Formate pair. The gear
// is formed, not generated — every flank of its tooth spaces is one of the face mill's cones,
// copied round its axis with no roll — so its tooth is the ring between those cones, standing in
// the gear. The pinion is its blank less that tooth swept under the mesh itself: the gear turning
// about its shaft as the pinion turns about its own, the ratio their tooth counts. Each pinion
// flank is then the envelope of a gear flank under the motion the pair runs at, conjugate to it
// at any offset, and nothing but the two members and their shafts enters it. The layout, the
// blanks and the face mill are `hypoid.sv`'s own steps, imported. Step 1 of #283; see README.md.
unit mm
use std
use hypoid (GearCone, ToothTrace, PinionCone, Blank, FaceMill, Topping, Member, Generated)

// -- the pair as a designer states it ----------------------------------------------------------
// As `hypoid.sv` states it: the tooth counts, the module at M, the shafts, the cutter's pressure
// shift and spiral, and the allowances, each zero for the conjugate, sharp-edged pair. The shift
// is still the number tuned for the crown pair; the generated pinion takes it without undercut.
param pinion_teeth := 24
param gear_teeth := 48
param mean_module := 2mm
param shaft_angle := 90deg
param offset := 25mm
param pressure_shift := 12.5deg
param spiral_angle := 25deg
param backlash := 0.05mm
param tip_relief := 0.2mm
param end_relief := 0.2mm
// The pair on the pitch plane P, `std.top`: `pair.pinion.body` and `pair.gear.body` are the
// members.
pair := FormatePair(std.top, std.y, std.z, pinion_teeth: pinion_teeth, gear_teeth: gear_teeth,
  mean_module: mean_module, shaft_angle: shaft_angle, offset: offset,
  pressure_shift: pressure_shift, spiral_angle: spiral_angle, backlash: backlash,
  tip_relief: tip_relief, end_relief: end_relief)

// -- the pair ----------------------------------------------------------------------------------
// The layout, the blanks and the face mill are `HypoidPair`'s, step for step; what differs is how
// each member is cut. The unknowns no point holds are the same three formals: `mn`, `rp` and `pa`.
component FormatePair(P: plane, across: axis, up: axis, pinion_teeth: Int, gear_teeth: Int,
    mean_module: Length, shaft_angle: Angle, offset: Length, pressure_shift: Angle,
    spiral_angle: Angle, backlash: Length, tip_relief: Length, end_relief: Length, mn: Length,
    rp: Length, pa: Angle) {
  R := mean_module * sqrt(pinion_teeth^2 + gear_teeth^2) / 2
  pressure := 20deg
  addendum := mn
  dedendum := 1.25 * mn
  base := 2 * mn
  rounding := 0.3 * mn

  // The pitch cones and the tooth trace.
  gear_cone := GearCone(P, across, pinion_teeth: pinion_teeth, gear_teeth: gear_teeth,
    mean_module: mean_module)
  trace := ToothTrace(P, gear_cone, R: R, spiral_angle: spiral_angle, mean_module: mean_module,
    mn: mn)
  pinion_cone := PinionCone(P, up, gear_cone, trace, shaft_angle: shaft_angle, offset: offset,
    rp: rp, pa: pa)

  // The blanks.
  blank_design := {width: 0.2 * R, addendum: addendum, back: 4 * mn, relief: end_relief}
  gear_blank := Blank(gear_cone.generator, gear_cone.ax, blank_design)
  pinion_blank := Blank(pinion_cone.pitch_line, pinion_cone.ax, blank_design)

  // The face mill: its axis through C and its two flank cones through the pitch points.
  mill := FaceMill(P, gear_cone, trace, R: R, mean_module: mean_module, pressure: pressure,
    pressure_shift: pressure_shift, dedendum: dedendum, base: base)

  // The motions: the gear's pitch round its axis, which indexes it, and the mesh — the gear
  // turning at the pinion's tooth count over its own against the pinion's turn, the gear seen
  // from the pinion. Neither reads a cone distance: the ratio is the teeth.
  gear_index := motion(about: gear_cone.ax)
  gear_next := motion(about: gear_cone.ax, phase: 360deg / gear_teeth)
  pinion_index := motion(about: pinion_cone.ax)
  pinion_turn := motion(about: pinion_cone.ax)
  gear_turn := motion(about: gear_cone.ax, ratio: -pinion_teeth / gear_teeth)
  mesh := motion(gear_turn, relative_to: pinion_turn)

  // The gear's tooth, as the face mill forms it: the ring about the cutter's axis between the
  // inner and outer flank cones, up to the root clearance past the gear's tip and down past its
  // root, where the pinion's tip never reaches. The gear's teeth are this ring copied round the
  // gear's axis, each flank a quarter of the backlash inside it; the pinion's cutter is the ring
  // itself, each flank grown by the pinion's quarter.
  gear_tooth := GearTooth(mill.inner_flank, mill.outer_flank, mill.tooth_base, mill.tooth_tip,
    grow: backlash / 4, r: rounding)

  // The gear, formed: a space is what lies between one tooth's outer flank and the next tooth's
  // inner flank, that tooth a gear pitch round (`gear_next`), above the cutter's tip at the gear's
  // root; the gear is its blank less the space copied at every tooth, with no roll.
  clear_below := std.HalfSpace(trace.C, mill.ax, d: -4 * base)
  clear_above := std.HalfSpace(trace.C, mill.ax, d: 4 * base)
  construction gear_space := Space(mill.outer_flank, mill.inner_flank, mill.mate_tip,
    mill.mate_base, clear_below, clear_above, mill.reach, gear_next, grow: backlash / 4,
    r: rounding)
  gear := Formed(gear_blank.material, gear_space.body, gear_index, n: gear_teeth)

  // The pinion, generated by the gear: its blank less the gear's tooth swept under the mesh
  // through it, at every pinion tooth.
  pinion := Member(pinion_blank.material, gear_tooth.body, mesh, pinion_index, n: pinion_teeth)

  // The tip relief, as `hypoid.sv` draws it in the normal section N: the pinion's tip edges cut
  // by the gear tooth's flanks turned out beyond a kink near the gear's root, swept under the
  // mesh; the gear's by its space's sides turned out near the space's floor, copied at every
  // tooth.
  when tip_relief > 0mm {
    N := plane(u: trace.normal, v: mill.ax)
    depth := addendum - tip_relief
    inner_lean := pressure + pressure_shift + 30deg
    outer_lean := pressure_shift - pressure - 30deg
    pinion_inner := Topping(gear_tooth.inner, mill.ax, N, mill.to_inner, height: -depth,
      lean: inner_lean)
    pinion_outer := Topping(gear_tooth.outer, mill.ax, N, mill.to_outer, height: -depth,
      lean: outer_lean)
    pinion_top := std.HalfSpace(trace.C, mill.ax, d: -addendum / 2)
    construction pinion_topping := GearTooth(pinion_inner.cone, pinion_outer.cone,
      mill.tooth_base, pinion_top, grow: 0mm, r: rounding)
    gear_outer := Topping(gear_space.convex, mill.ax, N, mill.to_outer, height: depth,
      lean: outer_lean)
    gear_inner := Topping(gear_space.concave, mill.ax, N, mill.to_inner, height: depth,
      lean: inner_lean)
    gear_top := std.HalfSpace(trace.C, mill.ax, d: addendum / 2)
    construction gear_topping := Space(gear_outer.cone, gear_inner.cone, gear_top,
      mill.mate_base, clear_below, clear_above, mill.reach, gear_next, grow: 0mm, r: rounding)
    pinion_relief := Generated(pinion.body, pinion_topping.body, mesh, pinion_index,
      n: pinion_teeth)
    gear_relief := Copied(gear.body, gear_topping.body, gear_index, n: gear_teeth)
  }
}

// -- the gear's tooth and space ----------------------------------------------------------------
// The gear's tooth about the cutter's axis, tip up: beyond the flank cone `a` and within `b`,
// each grown by `grow`, above the level `lo` and below `hi`, its tip's edges rounded by `r`; and
// its solid, `body`. Each set names the face it bounds.
component GearTooth(a: set, b: set, lo: set, hi: set, grow: Length, r: Length) {
  inner := { q | q distance(<= grow, side: inside) a }
  outer := std.Offset(b, d: grow)
  base := { q | q outside lo }
  tip := { q | q inside hi }
  inner_round := std.Rounded(inner, tip, r: r)
  outer_round := std.Rounded(outer, tip, r: r)
  region := { p | p inside inner; p inside outer; p inside base; p inside tip
                  p inside inner_round; p inside outer_round }
  construction body := solid(region)
}

// A gear space, tip down: beyond the outer flank cone `a` of the tooth behind it, within the cap,
// above the cutter's tip `lo` and below `hi`, that floor's edge rounded by `r` — and within the
// inner flank cone `b` of the tooth ahead, that tooth `step` round the gear's axis, each cone grown
// by `grow` (into the gear's teeth). The floor is one cutter's: the tooth ahead bounds the space by
// its flank alone, between the levels `below` and `above` clear of the blank, so no second floor
// meets the first at the shallow angle a gear pitch tilts it by, and that flank meets the floor
// unrounded. Each set names the face it cuts in the gear: `convex`, a tooth's convex flank;
// `concave`, the next tooth's concave one.
component Space(a: set, b: set, lo: set, hi: set, below: set, above: set, cap: set, step: motion,
    grow: Length, r: Length) {
  convex := { q | q distance(<= grow, side: inside) a }
  concave := std.Offset(b, d: grow)
  tip := { q | q outside lo }
  base := { q | q inside hi }
  convex_round := std.Rounded(convex, tip, r: r)
  near := { p | p inside convex; p inside cap; p inside tip; p inside base; p inside convex_round }
  far := { p | p inside concave; p inside cap; p outside below; p inside above }
  private construction behind := solid(near)
  private construction flank := solid(far)
  private construction ahead := solid(flank, under: step, at: 0deg)
  body := solid(behind)
  ahead common body
}

// -- forming -----------------------------------------------------------------------------------
// A formed cut: `tool` copied into `body` by `index` at each of `n` teeth, with no roll. A formed
// member is its blank less its cutter so copied.
component Copied(body: solid, tool: solid, index: motion, n: Int) {
  repeat n as i {
    private construction indexed := solid(tool, under: index, at: i * 360deg / n)
    indexed cut body
  }
}
component Formed(blank: solid, tool: solid, index: motion, n: Int) {
  body := solid(blank)
  cut := Copied(body, tool, index, n: n)
}
