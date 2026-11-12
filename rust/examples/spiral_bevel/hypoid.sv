// A spiral bevel or hypoid pair: a 24-tooth pinion and a 48-tooth gear, laid out as a gear
// designer lays one out — the pitch cones and the tooth trace through the mean point M on the
// pitch plane P — then cut by a generating crown: each member is its blank less the crown rolled
// through it at every tooth. Positions follow from lines, angles, incidence and the stated
// numbers, with no `sin` or `cos`, and every root is chosen by a word (a bound, a sense, an
// ordinate's sign), not a seed: only a blade's apex is started, on the side its cone opens to
// (§9.6, #217). The face mill's pressure shift is found, not stated: where the flanks the two
// blades cut stand equally far from undercut (`Balanced`, #283, #904). See README.md.
unit mm
use std
use std.place (skew)
use std.sets
use std.rounds
use std.motion (turns)
use std.solids (cut, common)

// -- the pair as a designer states it ----------------------------------------------------------
param pinion_teeth := 24
param gear_teeth := 48
param mean_module := 2mm
// The shafts: their angle, and their offset (the common perpendicular's length). No offset is a
// bevel pair with a common apex; any other, a hypoid.
param shaft_angle := 90deg
param offset := 25mm
// The crown's spiral angle at M.
param spiral_angle := 25deg
// The allowances, each zero for the conjugate, sharp-edged pair: the normal backlash (a quarter
// off each crown flank), the tip relief (how far down the tooth the tips' flank edges are cut) and
// the end relief (the tips' toe and heel edges chamfered this far each way).
param backlash := 0.05mm
param tip_relief := 0.2mm
param end_relief := 0.2mm
// The face mill's radius: a cutter is chosen, not found — the flanks' margins against undercut
// grow with it, so no balance of them sets it (#904).
param cutter_radius := 75.6122mm
// The members are `pair.pinion.body` and `pair.gear.body`. The pressure shift is only seeded
// (`hint`): the seed says where the search begins, never where it ends.
pair := HypoidPair(pinion_teeth: pinion_teeth, gear_teeth: gear_teeth,
  mean_module: mean_module, shaft_angle: shaft_angle, offset: offset,
  spiral_angle: spiral_angle, backlash: backlash, tip_relief: tip_relief, end_relief: end_relief,
  pressure_shift: hint(10deg), cutter_radius: cutter_radius, least: hint(0.5))
// The shift is found where the flanks nearest undercut, one cut by each blade, are equally far
// from it: each flank's margin is its least area factor (`regular`), and the two that tie are each
// held to one unknown, `pair.least`. A hypoid's are the pinion's flank the inner blade cuts and the
// gear's the outer cuts; with too little offset to protect the pinion's other flank, that flank,
// the pinion's outer, is the one that ties with its inner (`pair.margins.pinion_outer`, cut by
// `pinion_cut`, in place of the gear's). Every flank is held from its fold, so a tie that leaves another folded is a bound
// broken, never an undercut member.
pair.margins.pinion_inner regular(pair.least, by: pair.margins.pinion_cut) within pair.margins.pinion
pair.margins.gear_outer regular(pair.least, by: pair.margins.gear_outer_cut) within pair.margins.gear

// -- the pair ----------------------------------------------------------------------------------
// The steps a designer takes, each a component below, on the pitch plane P — the `std.top` of the
// frame the pair stands in — with `across`, its `std.y` (the gear's axis stands square to it), and
// `up`, its `std.z`. The formals the call leaves out are unknowns: the normal module `mn`, which
// the trace sets; the pinion's pitch radius at M, `rp`, and the bevel pinion's pitch angle, `pa`,
// which the pinion's cone sets. The face mill's `pressure_shift` and `cutter_radius` are the
// call's: stated, or left out and found by what the caller states over `margins` — with
// `least`, the margin the flanks that bind are held to, which the pair itself reads nowhere.
component HypoidPair(pinion_teeth: Int, gear_teeth: Int, mean_module: Length, shaft_angle: Angle,
    offset: Length, spiral_angle: Angle, backlash: Length, tip_relief: Length, end_relief: Length,
    mn: Length, rp: Length, pa: Angle, pressure_shift: Angle, cutter_radius: Length,
    least: Scalar) {
  // The proportions: the crown's tooth count (the hypotenuse of the gear's triangle, in teeth),
  // the mean cone distance R, and the depths in normal modules.
  crown_teeth := sqrt(pinion_teeth^2 + gear_teeth^2)
  R := mean_module * crown_teeth / 2
  pressure := 20deg
  addendum := mn
  dedendum := 1.25 * mn
  base := 2 * mn
  rounding := 0.3 * mn

  // The pitch cones and the tooth trace.
  gear_cone := GearCone(std.top, std.y, pinion_teeth: pinion_teeth, gear_teeth: gear_teeth,
    mean_module: mean_module)
  trace := ToothTrace(std.top, gear_cone, spiral_angle: spiral_angle,
    cutter_radius: cutter_radius)
  // The normal module `mn` is the module seen square to the trace: O stands R / m modules from M,
  // so `mn` R / m from the trace's normal.
  gear_cone.O distance(mn * R / mean_module) trace.normal
  pinion_cone := PinionCone(std.top, std.z, gear_cone, trace, shaft_angle: shaft_angle,
    offset: offset, rp: rp, pa: pa)
  // The bevel pinion's pitch radius `rp` is M's distance from the pinion's axis, and its pitch
  // angle `pa` the gear's triangle's at M.
  gear_cone.M distance(rp) pinion_cone.foot
  gear_cone.to_apex angle(pa) gear_cone.to_foot

  // The blanks.
  blank_design := {width: 0.2 * R, addendum: addendum, back: 4 * mn, relief: end_relief}
  gear_blank := Blank(gear_cone.generator, gear_cone.ax, blank_design)
  pinion_blank := Blank(pinion_cone.pitch_line, pinion_cone.ax, blank_design)

  // The face mill.
  mill := FaceMill(std.top, gear_cone, trace, R: R, mean_module: mean_module, pressure: pressure,
    pressure_shift: pressure_shift, cutter_radius: cutter_radius, dedendum: dedendum, base: base)

  // The rolls. The crown turns about its axis at O and meshes with each member tooth for tooth,
  // so each member turns at the crown's tooth count over its own; a member's cut is the crown's
  // roll seen from the member's. An index frame turns a member a tooth at a time, and the crown's
  // neighbour is the crown read one crown pitch round.
  crown_roll := frame
  crown_roll turns(1) gear_cone.crown_ax
  pinion_roll := frame
  pinion_roll turns(crown_teeth / pinion_teeth) pinion_cone.ax
  gear_roll := frame
  gear_roll turns(-crown_teeth / gear_teeth) gear_cone.ax
  pinion_index := frame
  pinion_index turns(1) pinion_cone.ax
  gear_index := frame
  gear_index turns(1) gear_cone.ax
  crown_pitch := -360deg / crown_teeth

  // The crowns, and the members they generate.
  tooth := Tooth(mill.inner_flank, mill.outer_flank, mill.tooth_base, mill.tooth_tip,
    grow: backlash / 4, r: rounding)
  construction gear_space := Space(mill.outer_flank, mill.inner_flank, mill.mate_tip,
    mill.mate_base, mill.reach, crown_roll, pitch: crown_pitch, grow: backlash / 4, r: rounding)
  pinion := Member(pinion_blank.material, tooth.crown, crown_roll, pinion_roll, pinion_index,
    n: pinion_teeth)
  gear := Member(gear_blank.material, gear_space.body, crown_roll, gear_roll, gear_index,
    n: gear_teeth)
  // What a drawing that stands on the pair reads of each member: its axis and the foot of M on it,
  // its blank's back cone and the blank itself, and the member.
  gear_member := {ax: gear_cone.ax, foot: gear_cone.foot, back: gear_blank.back,
    material: gear_blank.material, body: gear.body}
  pinion_member := {ax: pinion_cone.ax, foot: pinion_cone.foot, back: pinion_blank.back,
    material: pinion_blank.material, body: pinion.body}

  // The face mill's settings, read: each flank's margin against undercut, the four the two
  // blades cut on the two members. What chooses the mill is the caller's: its settings stated, or
  // relations over these margins in their place, as this document's balance is.
  balance := Balanced(mill.inner_flank, mill.outer_flank, mill.tooth_base, mill.tooth_tip,
    mill.mate_tip, mill.mate_base, mill.reach, crown_roll, pinion_roll, gear_roll,
    pinion_cone.pitch_line, pinion_cone.ax, gear_cone.generator, gear_cone.ax, blank_design,
    pitch: crown_pitch, r: rounding)
  margins := {pinion_inner: balance.pinion_inner, pinion_outer: balance.pinion_outer,
    gear_inner: balance.gear_inner, gear_outer: balance.gear_outer,
    pinion_cut: balance.cuts.pinion, gear_outer_cut: balance.cuts.gear_outer,
    gear_inner_cut: balance.cuts.gear_inner, pinion: balance.blanks.pinion,
    gear: balance.blanks.gear}

  // The tip relief: a topping beside each crown flank, kinked in the normal section N (square to
  // P on the trace normal, through the cutter's axis) `tip_relief` short of the addendum, and cut
  // by its own sweep.  N's u runs along the trace normal away from M (`to_inner`), up the cutter's
  // axis its v: the way the toppings were cut in it.
  when tip_relief > 0mm {
    N := plane(u: mill.to_inner, v: mill.ax)
    depth := addendum - tip_relief
    inner_half := pressure + pressure_shift + 30deg
    outer_half := pressure - pressure_shift + 30deg
    pinion_inner := Topping(tooth.inner, mill.ax, N, mill.to_inner, height: -depth,
      half: inner_half, opens: 1)
    pinion_outer := Topping(tooth.outer, mill.ax, N, mill.to_outer, height: -depth,
      half: outer_half, opens: -1)
    pinion_top := std.sets.HalfSpace(trace.C, mill.ax, d: -addendum / 2)
    construction pinion_topping := Tooth(pinion_inner.cone, pinion_outer.cone, mill.tooth_base,
      pinion_top, grow: 0mm, r: rounding)
    gear_outer := Topping(gear_space.inner, mill.ax, N, mill.to_outer, height: depth,
      half: outer_half, opens: -1)
    gear_inner := Topping(gear_space.outer, mill.ax, N, mill.to_inner, height: depth,
      half: inner_half, opens: 1)
    gear_top := std.sets.HalfSpace(trace.C, mill.ax, d: addendum / 2)
    construction gear_topping := Space(gear_outer.cone, gear_inner.cone, gear_top,
      mill.mate_base, mill.reach, crown_roll, pitch: crown_pitch, grow: 0mm, r: rounding)
    pinion_relief := Generated(pinion.body, pinion_topping.crown, crown_roll, pinion_roll,
      pinion_index, n: pinion_teeth)
    gear_relief := Generated(gear.body, gear_topping.body, crown_roll, gear_roll, gear_index,
      n: gear_teeth)
  }
}

// -- the pitch cones and the tooth trace -------------------------------------------------------
// What the steps read of one another: the gear's cone, the trace, and the proportions a blank is
// cut to. A step is handed the instance and reads only these.
interface Gear {
  O: point, M: point, generator: line, ax: line, to_apex: line, to_foot: line, crown_ax: line
}
interface Trace { C: point, normal: line, to_heading: line }
interface BlankDesign { width: Length, addendum: Length, back: Length, relief: Length }

// The gear's pitch cone: apex O at P's origin, M on P's u axis. The right triangle O–M–F stands
// square to P on the generator O → M, F (`foot`) the foot of M on the gear's axis `ax`, below P
// and level with O along `across`; its legs are the two pitch radii at M, so the pitch angle and
// the cone distance follow. The crown's axis stands up from O, square to P.
component GearCone(P: plane, across: axis, pinion_teeth: Int, gear_teeth: Int,
    mean_module: Length) {
  O := point in P
  M := point in P
  generator := line(O, M)
  O coincident P.origin
  M level(v) P
  M distance(>= 0, along: u) P
  foot := point
  to_apex := line(M, O)
  to_foot := line(M, foot)
  ax := line(O, foot)
  O level(across) foot
  foot distance(<= 0mm, along: n) P
  to_foot perpendicular ax
  M distance(gear_teeth * mean_module / 2) foot
  O distance(pinion_teeth * mean_module / 2) foot
  crown_ax := line(O)
  fix((0, 0, 1)) crown_ax.p2
}

// The tooth trace: the circle about the cutter centre C through M, its normal C → M at 90° less
// the spiral angle to the generator. H is where the heading at M, square to the normal, meets the
// line from O parallel to it.
component ToothTrace(P: plane, gear: Gear, spiral_angle: Angle, cutter_radius: Length) {
  C := point in P
  H := point in P
  normal := line(C, gear.M)
  heading := line(gear.M, H)
  to_heading := line(gear.O, H)
  gear.M distance(cutter_radius) C
  gear.generator angle(90deg - spiral_angle, sense: cw) normal
  heading perpendicular normal
  to_heading parallel normal
}

// The pinion's cone, solved against the gear's: apex A in P, on the offset's side of O and behind
// M; its axis `ax` square to P on the generator A → M (square to the level line `square`, its foot
// above P); the shafts their angle and offset apart. The equal normal pitch sizes it with no
// cosine: V, where A → M crosses O → H, keeps |MV| cos(spiral + offset angle) = R cos(spiral), and
// is the apex of a virtual bevel pinion at the pitch angle `pa` and the pitch radius `rp` at M —
// what the pair reads them as, the gear's triangle's angle and M's distance from the pinion's axis.
// With no offset A and V are O, and the two axes one.
component PinionCone(P: plane, up: axis, gear: Gear, trace: Trace, shaft_angle: Angle,
    offset: Length, rp: Length, pa: Angle) {
  A := point in P
  V := point in P
  pitch_line := line(A, gear.M)
  V coincident pitch_line
  V coincident trace.to_heading
  A distance(>= 0, along: v) P
  A distance(>= 0, along: gear.generator) gear.M
  foot := point
  virtual_foot := point
  ax := line(A, foot)
  virtual_ax := line(V, virtual_foot)
  square := axis
  square perpendicular pitch_line
  square perpendicular up
  ax perpendicular square
  virtual_ax perpendicular square
  foot distance(>= 0mm, along: n) P
  virtual_foot distance(>= 0mm, along: n) P
  gear.M level(ax) foot
  gear.M level(virtual_ax) virtual_foot
  gear.M distance(rp) virtual_foot
  gear.ax skew(theta: shaft_angle, e: offset) ax
  pitch_line angle(pa) virtual_ax
}

// -- the blanks --------------------------------------------------------------------------------
// A member's blank, one region: inside its pitch cone grown by the addendum and its heel sphere,
// outside its toe sphere and its pitch cone shrunk by the back depth. The spheres are about the
// apex `rim.p1`, half the face width either side of M (`rim.p2`); the end relief chamfers the
// tip's toe and heel corners. Its faces are named by their sets: `material.tip`, `.heel`, `.toe`
// and `.back`.
component Blank(rim: line, ax: line, design: BlankDesign) {
  toe := std.sets.Ball(rim.p1)
  heel := std.sets.Ball(rim.p1)
  rim.p2 distance(design.width / 2) toe
  rim.p2 distance(design.width / 2) { q | q not in heel }
  pitch := std.sets.Cone(ax)
  rim.p2 in boundary(pitch)
  tip := { q | q distance(<= design.addendum) pitch }
  back := { q | q distance(<= design.back) { r | r not in pitch } }
  beyond_toe := { q | q not in toe }
  region := { p | p in tip; p in heel; p not in toe; p in back
                  p in std.rounds.Chamfered(tip, beyond_toe, la: design.relief,
                                                lb: design.relief)
                  p in std.rounds.Chamfered(tip, heel, la: design.relief, lb: design.relief) }
  material := solid(region)
}

// -- the generating crown ----------------------------------------------------------------------
// The face mill. The crown tooth's thickness: the points a quarter crown pitch either side of M on
// the mean pitch circle (the arcs `ahead` and `behind` about O); the inner and outer trace circles
// about C pass through them and cross the trace normal beyond C at the pitch points. The mill's
// axis `ax` stands up from C, the cutter's radius long; each flank cone passes through a pitch
// point, its half the pressure angle plus or minus the shift, the inner opening up the axis and
// the outer down it. The half-spaces are where the crowns' tips and bases stand; `reach` bounds
// the gear's space cutter, a cylinder about the axis just beyond the blanks.
component FaceMill(P: plane, gear: Gear, trace: Trace, R: Length, mean_module: Length,
    pressure: Angle, pressure_shift: Angle, cutter_radius: Length, dedendum: Length,
    base: Length) {
  ahead := arc(center: gear.O, start: gear.M) in P
  behind := arc(center: gear.O, end: gear.M) in P
  inner_pitch := point in P
  outer_pitch := point in P
  inner_trace := arc(center: trace.C, start: ahead.end, end: inner_pitch)
  outer_trace := arc(center: trace.C, start: behind.start, end: outer_pitch)
  to_inner := line(trace.C, inner_pitch)
  to_outer := line(trace.C, outer_pitch)
  length(pi * mean_module / 4) ahead
  length(pi * mean_module / 4) behind
  trace.normal angle(180deg) to_inner
  trace.normal angle(180deg) to_outer
  ax := line(trace.C)
  ax parallel gear.crown_ax
  trace.C distance(cutter_radius, along: gear.crown_ax) ax.p2
  inner_flank := Blade(ax, inner_pitch, half: pressure + pressure_shift, opens: 1)
  outer_flank := Blade(ax, outer_pitch, half: pressure - pressure_shift, opens: -1)
  tooth_tip := std.sets.HalfSpace(trace.C, ax, d: dedendum)
  tooth_base := std.sets.HalfSpace(trace.C, ax, d: -base)
  mate_tip := std.sets.HalfSpace(trace.C, ax, d: -dedendum)
  mate_base := std.sets.HalfSpace(trace.C, ax, d: base)
  cap := point in P
  to_cap := line(trace.C, gear.O)
  cap coincident to_cap
  gear.O distance(1.1 * R + mean_module, along: to_cap) cap
  reach := std.sets.Cylinder(ax)
  cap in boundary(reach)
}

// The crown tooth, the pinion's generator: beyond the flank cone `a` and within `b`, each grown by
// `grow` (a quarter of the backlash: the envelope of an offset is the offset of the envelope),
// above `lo` and below `hi`, its tip's edges rounded by `r`. Each set names the face it bounds.
component Tooth(a: set, b: set, lo: set, hi: set, grow: Length, r: Length) {
  inner := { q | q distance(<= grow) { r | r not in a } }
  outer := { q | q distance(<= grow) b }
  base := { q | q not in lo }
  tip := { q | q in hi }
  inner_round := std.rounds.Rounded(inner, tip, r: r)
  outer_round := std.rounds.Rounded(outer, tip, r: r)
  region := { p | p in inner; p in outer; p in base; p in tip
                  p in inner_round; p in outer_round }
  construction crown := solid(region)
}
// The crown space, the gear's generator: the tooth's mate on the same cones, tip down, so the two
// crowns are complementary by construction. It is where the outer mate (`outer_crown`, within the
// cap) overlaps the inner mate a crown pitch round (`neighbor`: `inner_crown` read `pitch` along
// `step`).
component Space(a: set, b: set, lo: set, hi: set, cap: set, step: frame, pitch: Angle,
    grow: Length, r: Length) {
  inner := { q | q distance(<= grow) { r | r not in a } }
  outer := { q | q distance(<= grow) b }
  tip := { q | q not in lo }
  base := { q | q in hi }
  inner_round := std.rounds.Rounded(inner, tip, r: r)
  outer_round := std.rounds.Rounded(outer, tip, r: r)
  near := { p | p in inner; p in cap; p in tip; p in base; p in inner_round }
  far := { p | p in outer; p in tip; p in base; p in outer_round }
  construction outer_crown := solid(near)
  construction inner_crown := solid(far)
  private construction neighbor := inner_crown at(under: step) pitch
  body := solid(outer_crown)
  neighbor common body
}

// -- generation --------------------------------------------------------------------------------
// A cut generated in `body`: `tool` swept under the roll `under` seen from `seen_from`, over the
// whole roll during which it meets the body (found once the layout is solved), and indexed by
// `index` at each of `n` teeth. A member is its blank less its crown's cut.
component Generated(body: solid, tool: solid, under: frame, seen_from: frame, index: frame,
    n: Int) {
  construction removal := solid({ p | some t: p in tool at(under: under, seen_from: seen_from) t })
  repeat n as i {
    private construction indexed := removal at(under: index) i * 360deg / n
    indexed cut body
  }
}
component Member(blank: solid, tool: solid, under: frame, seen_from: frame, index: frame,
    n: Int) {
  body := solid(blank)
  cut := Generated(body, tool, under, seen_from, index, n: n)
}

// -- the tip relief ----------------------------------------------------------------------------
// A topping beside one crown flank: a kink on the flank (as grown) in the normal section `n`,
// `height` along the axis and beyond it as the flank's pitch point is (`toward`), and the blade
// through it, turned by the relief angle so the cut widens toward the base. It is cut by its own
// sweep: a concave corner inside one sweep would trim two envelopes against each other
// (generating-sweep row T2).
component Topping(flank: set, ax: line, n: plane, toward: line, height: Length, half: Angle,
    opens: Scalar) {
  kink := point in n
  kink in boundary(flank)
  ax.p1 distance(height, along: ax) kink
  ax.p1 distance(>= 0mm, along: toward) kink
  cone := Blade(ax, kink, half: half, opens: opens)
}

// A blade's cone about the cutter's axis `about`, through `q`: `std.sets.Cone`'s about `.spine`
// (from the apex to `q`'s foot on `about`), `half` the angle between the axis and every generator,
// opening along `about` (`opens: 1`) or against it (`opens: -1`). It is `std.sets.ConeThrough` with
// that side said rather than read off the sign of a lean, so a half that reads an unknown
// (`pressure + pressure_shift`) stays a number times it plus a number. The apex is seeded on the
// side away from the opening.
component Blade(about: line, q: point, half: Angle, opens: Scalar) := { p |
  apex := point hint(at: about.p1, along: about, by: -opens)
  private foot := point hint(at: about.p1)
  apex coincident about
  private construction through := line(q, apex)
  about angle(90deg + opens * (90deg - half)) through
  foot coincident about
  private construction across := line(foot, q)
  across perpendicular about
  construction spine := line(apex, foot)
  private construction g := line(apex, p)
  spine angle(<= half) g
}

// -- the face mill's settings, found -------------------------------------------------------------
// Each member's flanks are the envelopes the crown's two blades generate (§6.15.3), and a flank's
// margin against undercut is its least area factor where the crown cuts it within its blank
// (`regular`): 1 rolling without sliding, nothing where it folds. The pressure shift steepens the
// inner blade and flattens the outer, so it raises the margins of the flanks the inner blade cuts
// and lowers those the outer cuts. The four flanks are read on the conjugate crowns, ungrown (a
// backlash's offset face is not its blade's envelope), each held from its fold; what ties two of
// them is the caller's: each held to one unknown, the margins and the crowns and blanks they are
// read by handed out as `pair.margins`.
component Balanced(inner: set, outer: set, tooth_base: set, tooth_tip: set, mate_tip: set,
    mate_base: set, reach: set, crown_roll: frame, pinion_roll: frame, gear_roll: frame,
    pinion_rim: line, pinion_ax: line, gear_rim: line, gear_ax: line, design: BlankDesign,
    pitch: Angle, r: Length) {
  // the blanks as cut before their end relief, which `regular` cannot read
  sharp := {width: design.width, addendum: design.addendum, back: design.back, relief: 0mm}
  construction pinion_blank := Blank(pinion_rim, pinion_ax, sharp)
  construction gear_blank := Blank(gear_rim, gear_ax, sharp)
  construction tooth := Tooth(inner, outer, tooth_base, tooth_tip, grow: 0mm, r: r)
  construction space := Space(outer, inner, mate_tip, mate_base, reach, crown_roll, pitch: pitch,
    grow: 0mm, r: r)
  pinion_inner := envelope(boundary(inner) over t in (-30deg, 30deg), under: crown_roll,
                           seen_from: pinion_roll)
  pinion_outer := envelope(boundary(outer) over t in (-30deg, 30deg), under: crown_roll,
                           seen_from: pinion_roll)
  gear_outer := envelope(boundary(outer) over t in (-30deg, 30deg), under: crown_roll,
                         seen_from: gear_roll)
  gear_inner := envelope(boundary(inner) over t in (-30deg, 30deg), under: crown_roll,
                         seen_from: gear_roll)
  pinion_inner regular(by: tooth.crown) within pinion_blank.material
  pinion_outer regular(by: tooth.crown) within pinion_blank.material
  gear_outer regular(by: space.outer_crown) within gear_blank.material
  gear_inner regular(by: space.inner_crown) within gear_blank.material
  // what a caller tying two of them reads them by, as this component's own: the crowns that cut
  // the flanks, and the blanks they are read within
  cuts := {pinion: tooth.crown, gear_outer: space.outer_crown, gear_inner: space.inner_crown}
  blanks := {pinion: pinion_blank.material, gear: gear_blank.material}
}
