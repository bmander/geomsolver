// A spiral bevel or hypoid pair, whole: a 24-tooth pinion and a 48-tooth gear laid out the way a
// gear designer lays a hypoid out — the pitch cones and the tooth trace through one design point,
// the mean point M on the pitch plane P — then the generating crown as regions about its cutter's
// axis, and each member its blank less its crown rolled through it at every tooth. Every position
// follows from lines, angles, incidence and the stated numbers; nothing is computed with `sin` or
// `cos`. No point is seeded but a blade's apex, started on the side its cone opens to as
// `std.ConeThrough`'s is: every root is chosen by a word — a bound (`inside`, `outside`,
// `distance(>= 0, …)`), a sense, an ordinate's sign — and the layout solves from them alone
// (§9.6, #217). The face mill's two settings are not stated either: the pressure shift and the
// cutter's radius are unknowns, found where the pinion and the gear stand equally far from
// undercut, at M and at the toe (`Balanced`, #283). The pair is one component, `HypoidPair`, over
// the steps below it. See README.md.
unit mm
use std (skew)

// -- the pair as a designer states it ----------------------------------------------------------
param pinion_teeth := 24
param gear_teeth := 48
param mean_module := 2mm
// The shafts: the angle between them, and the offset, their common perpendicular's length. No
// offset is a bevel pair with a common apex; any other, a hypoid.
param shaft_angle := 90deg
param offset := 25mm
// The crown's spiral angle at M; the pinion's adds the offset angle, which the layout solves for.
param spiral_angle := 25deg
// The allowances, each zero for the conjugate, sharp-edged pair: the normal backlash, a quarter
// off each crown flank; the tip relief, the tips' flank edges cut this far down the tooth; the
// end relief, the tips' toe and heel edges chamfered this far each way.
param backlash := 0.05mm
param tip_relief := 0.2mm
param end_relief := 0.2mm
// The pair on the pitch plane P, `std.top`, with `std.y` (the gear's axis stands square to it)
// and `std.z`, up from P: `pair.pinion.body` and `pair.gear.body` are the members. The face mill's
// pressure shift and radius are left to the solve, which starts them at the seeds (`hint`) and
// finds them from the balance; a seed says where the search begins, never where it ends.
pair := HypoidPair(std.top, std.y, std.z, pinion_teeth: pinion_teeth, gear_teeth: gear_teeth,
  mean_module: mean_module, shaft_angle: shaft_angle, offset: offset,
  spiral_angle: spiral_angle, backlash: backlash, tip_relief: tip_relief, end_relief: end_relief,
  pressure_shift: hint(10deg), cutter_radius: hint(80mm))

// -- the pair ----------------------------------------------------------------------------------
// The steps a designer takes, each a component below: the pitch cones and the tooth trace through
// M, the blanks, the generating crown, and the generation. The unknowns no point holds are the
// formals the call leaves out: the normal module `mn`, the module seen square to the tooth trace,
// which the trace constructs; the pinion's pitch radius at M, `rp`, which the equal normal pitch
// sets; the bevel pinion's pitch angle, `pa`, read off the gear's triangle; and the face mill's
// two settings, `pressure_shift` (added to the crown's inner flank pressure angle and taken from
// its outer's, so the crown tooth leans) and `cutter_radius`, which the balance below finds.
component HypoidPair(P: plane, across: axis, up: axis, pinion_teeth: Int, gear_teeth: Int,
    mean_module: Length, shaft_angle: Angle, offset: Length, spiral_angle: Angle,
    backlash: Length, tip_relief: Length, end_relief: Length, mn: Length, rp: Length, pa: Angle,
    pressure_shift: Angle, cutter_radius: Length) {
  // The proportions. The crown's tooth count, the hypotenuse of the gear's triangle in teeth, and
  // the mean cone distance R, that count times half the module, which sizes the face width
  // (0.2 R) and the gear space cutter's reach; the depths are in normal modules.
  crown_teeth := sqrt(pinion_teeth^2 + gear_teeth^2)
  R := mean_module * crown_teeth / 2
  pressure := 20deg
  addendum := mn
  dedendum := 1.25 * mn
  base := 2 * mn
  rounding := 0.3 * mn

  // The pitch cones and the tooth trace.
  gear_cone := GearCone(P, across, pinion_teeth: pinion_teeth, gear_teeth: gear_teeth,
    mean_module: mean_module)
  trace := ToothTrace(P, gear_cone, R: R, spiral_angle: spiral_angle, mean_module: mean_module,
    mn: mn, cutter_radius: cutter_radius)
  pinion_cone := PinionCone(P, up, gear_cone, trace, shaft_angle: shaft_angle, offset: offset,
    rp: rp, pa: pa)

  // The blanks.
  blank_design := {width: 0.2 * R, addendum: addendum, back: 4 * mn, relief: end_relief}
  gear_blank := Blank(gear_cone.generator, gear_cone.ax, blank_design)
  pinion_blank := Blank(pinion_cone.pitch_line, pinion_cone.ax, blank_design)

  // The generating crown: the face mill, and the tooth and the gear's space about its axis.
  mill := FaceMill(P, gear_cone, trace, R: R, mean_module: mean_module, pressure: pressure,
    pressure_shift: pressure_shift, cutter_radius: cutter_radius, dedendum: dedendum, base: base)

  // Generation. Every roll shares the crown's angle, about its axis square to P at O, and the
  // crown meshes with each member tooth for tooth: each turns at the crown's tooth count over its
  // own. With the gear the crown rolls on its pitch cone; with the pinion, whose own cone, solved
  // off the bevel, would roll at another, it keeps the pair's ratio, so pinion and gear generated
  // by one crown run at their tooth counts. Nothing is read off the solved drawing, so the solve
  // holds a contact to what either generates (`Balanced`). Indexing turns a member; the crown's
  // neighbour is one crown pitch round, four quarter pitches of the mean pitch circle.
  crown_roll := motion(about: gear_cone.crown_ax)
  pinion_roll := motion(about: pinion_cone.ax, ratio: crown_teeth / pinion_teeth)
  gear_roll := motion(about: gear_cone.ax, ratio: -crown_teeth / gear_teeth)
  pinion_generation := motion(crown_roll, relative_to: pinion_roll)
  gear_generation := motion(crown_roll, relative_to: gear_roll)
  pinion_index := motion(about: pinion_cone.ax)
  gear_index := motion(about: gear_cone.ax)
  crown_neighbor := motion(about: gear_cone.crown_ax,
    phase: -4 * length(mill.ahead) / radius(mill.ahead) * 1rad)

  tooth := Tooth(mill.inner_flank, mill.outer_flank, mill.tooth_base, mill.tooth_tip,
    grow: backlash / 4, r: rounding)
  construction gear_space := Space(mill.outer_flank, mill.inner_flank, mill.mate_tip,
    mill.mate_base, mill.reach, crown_neighbor, grow: backlash / 4, r: rounding)
  pinion := Member(pinion_blank.material, tooth.crown, pinion_generation, pinion_index,
    n: pinion_teeth)
  gear := Member(gear_blank.material, gear_space.body, gear_generation, gear_index, n: gear_teeth)

  // The face mill's two settings, found: the pinion and the gear equally far from undercut, in the
  // normal section through M and in the one through the toe of the trace.
  balance := Balanced(P, gear_cone, trace, mill.ax, mill.inner_flank, mill.outer_flank,
    pinion_generation, gear_generation, R: R, face: blank_design.width, depth: addendum,
    cutter_radius: cutter_radius)

  // The tip relief: a topping beside each crown flank, the kinks drawn in the normal section N
  // (square to P on the trace normal, through the cutter's axis) a depth `tip_relief` short of
  // the addendum, cut by its own sweep the same way (`Topping`).
  when tip_relief > 0mm {
    N := plane(u: trace.normal, v: mill.ax)
    depth := addendum - tip_relief
    inner_half := pressure + pressure_shift + 30deg
    outer_half := pressure - pressure_shift + 30deg
    pinion_inner := Topping(tooth.inner, mill.ax, N, mill.to_inner, height: -depth,
      half: inner_half, opening: 180deg - inner_half)
    pinion_outer := Topping(tooth.outer, mill.ax, N, mill.to_outer, height: -depth,
      half: outer_half, opening: outer_half)
    pinion_top := std.HalfSpace(trace.C, mill.ax, d: -addendum / 2)
    construction pinion_topping := Tooth(pinion_inner.cone, pinion_outer.cone, mill.tooth_base,
      pinion_top, grow: 0mm, r: rounding)
    gear_outer := Topping(gear_space.inner, mill.ax, N, mill.to_outer, height: depth,
      half: outer_half, opening: outer_half)
    gear_inner := Topping(gear_space.outer, mill.ax, N, mill.to_inner, height: depth,
      half: inner_half, opening: 180deg - inner_half)
    gear_top := std.HalfSpace(trace.C, mill.ax, d: addendum / 2)
    construction gear_topping := Space(gear_outer.cone, gear_inner.cone, gear_top,
      mill.mate_base, mill.reach, crown_neighbor, grow: 0mm, r: rounding)
    pinion_relief := Generated(pinion.body, pinion_topping.crown, pinion_generation,
      pinion_index, n: pinion_teeth)
    gear_relief := Generated(gear.body, gear_topping.body, gear_generation, gear_index,
      n: gear_teeth)
  }
}

// -- the pitch cones and the tooth trace -------------------------------------------------------
// What the steps below read of one another, each an interface: the gear's cone as the trace, the
// pinion's cone and the face mill read it, the trace as the pinion's cone and the mill read it,
// and the proportions a blank is cut to. A step is handed the instance, and reads only these.
interface Gear {
  O: point, M: point, generator: line, ax: line, to_apex: line, to_foot: line, crown_ax: line
}
interface Trace { C: point, normal: line, to_heading: line }
interface BlankDesign { width: Length, addendum: Length, back: Length, relief: Length }

// The gear's pitch cone: its apex O at P's origin and M on P's u axis beyond it; the gear's right
// triangle O–M–F stands square to P on the generator O → M, F (`foot`) the foot of M on the gear's
// axis `ax` below P, level with O along `across`, its legs the two pitch radii at M. The pitch angle and the
// cone distance follow; nothing states them. The crown's axis stands up square to P at O (P is
// `std.top`, at the world's origin).
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
  foot inside P
  to_foot perpendicular ax
  M distance(gear_teeth * mean_module / 2) foot
  O distance(pinion_teeth * mean_module / 2) foot
  crown_ax := line(O)
  fix((0, 0, 1)) crown_ax.p2
}

// The tooth trace: the circle about the cutter centre C through M, the cutter's radius from it,
// MC (`normal`) at 90° less the spiral to MO; its heading at M, square to MC, meets the line from
// O along MC at H (said as parallel, not as square to the heading, which a heading of no length
// would be). The normal module `mn` is the module seen square to the trace: how far from MC a
// point one module from M toward O stands, so O, R / m modules from M, stands R / m times as far.
component ToothTrace(P: plane, gear: Gear, R: Length, spiral_angle: Angle, mean_module: Length,
    mn: Length, cutter_radius: Length) {
  C := point in P
  H := point in P
  normal := line(C, gear.M)
  heading := line(gear.M, H)
  to_heading := line(gear.O, H)
  gear.M distance(cutter_radius) C
  gear.generator angle(90deg - spiral_angle, sense: cw) normal
  heading perpendicular normal
  to_heading parallel normal
  gear.O distance(mn * R / mean_module) normal
}

// The pinion's cone, solved against the gear's: its apex A in P on the offset's hand of O and
// behind M, its axis `ax` square to P on its generator A → M (each axis square to the level line
// `square`, running from the apex to the foot of M on it, above P, against `up`), the shafts
// their angle and offset apart. The equal normal pitch sizes it, with no cosine written: V, where
// MA crosses the square from O to the heading, keeps |MV| cos(spiral + offset angle) = R
// cos(spiral); a virtual axis leaves V at the bevel pinion's pitch angle `pa`, the angle at M in
// the gear's triangle, and the pinion's pitch radius `rp` is one distance from M both axes keep.
// With no offset, A and V are O and the two axes one.
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
  foot outside P
  virtual_foot outside P
  gear.M level(ax) foot
  gear.M level(virtual_ax) virtual_foot
  gear.M distance(rp) foot
  gear.M distance(rp) virtual_foot
  gear.ax skew(theta: shaft_angle, e: offset) ax
  gear.to_apex angle(pa) gear.to_foot
  pitch_line angle(pa) virtual_ax
}

// -- the blanks --------------------------------------------------------------------------------
// A member's blank: within its pitch cone grown by the addendum and its heel sphere, outside its
// toe sphere and its pitch cone shrunk by the back depth, one region and its solid. The spheres
// are about the apex at the start of the generator `rim` (apex → M), the toe half the face width
// short of M and the heel half the face width beyond it. The end relief chamfers the tip's toe
// and heel corners `relief` each way (`std.Chamfered`); a relief of nothing cuts nothing. Each
// set the region uses names the face it bounds: `material.tip`, `.heel`, `.toe` and `.back`.
component Blank(rim: line, ax: line, design: BlankDesign) {
  toe := std.Sphere(rim.p1)
  heel := std.Sphere(rim.p1)
  rim.p2 distance(design.width / 2, side: outside) toe
  rim.p2 distance(design.width / 2, side: inside) heel
  pitch := std.Cone(ax)
  rim.p2 coincident pitch
  tip := std.Offset(pitch, d: design.addendum)
  back := { q | q distance(<= design.back, side: inside) pitch }
  beyond_toe := { q | q outside toe }
  region := { p | p inside tip; p inside heel; p outside toe; p inside back
                  p inside std.Chamfered(tip, beyond_toe, la: design.relief, lb: design.relief)
                  p inside std.Chamfered(tip, heel, la: design.relief, lb: design.relief) }
  material := solid(region)
}

// -- the generating crown ----------------------------------------------------------------------
// The face mill. The crown tooth's thickness: two points of the mean pitch circle a quarter crown
// pitch either side of M (the arcs `ahead` and `behind` about O); the inner and outer trace
// circles about C pass through them, and its pitch points are where they cross the trace normal
// beyond C. The mill's axis `ax` stands up through C, square to P, as long as the cutter's radius;
// the flank cones about it pass through the pitch points leaning by the pressure angle split by
// the shift — the inner opening up the axis, the outer down it (`Blade`) — and the levels along
// it are where the crowns' tips and bases stand. `reach`, how far the gear's space cutter
// reaches: the cylinder about the axis through the point a little over the cone distance past O
// on the line from C through it, clear of every blank point.
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
  inner_flank := Blade(ax, inner_pitch, half: pressure + pressure_shift,
    opening: 180deg - pressure - pressure_shift)
  outer_flank := Blade(ax, outer_pitch, half: pressure - pressure_shift,
    opening: pressure - pressure_shift)
  tooth_tip := std.HalfSpace(trace.C, ax, d: dedendum)
  tooth_base := std.HalfSpace(trace.C, ax, d: -base)
  mate_tip := std.HalfSpace(trace.C, ax, d: -dedendum)
  mate_base := std.HalfSpace(trace.C, ax, d: base)
  cap := point in P
  to_cap := line(trace.C, gear.O)
  cap coincident to_cap
  gear.O distance(1.1 * R + mean_module, along: to_cap) cap
  reach := std.Cylinder(ax)
  cap coincident reach
}

// A crown tooth about a cutter's axis, the pinion's generator: beyond the flank cone `a` and
// within `b`, each grown by `grow` (a quarter of the backlash: the envelope of an offset surface
// is the offset of the envelope), above the level `lo` and below `hi`, its tip's edges rounded by
// `r`; and its solid, `crown`. Each set names the face it bounds.
component Tooth(a: set, b: set, lo: set, hi: set, grow: Length, r: Length) {
  inner := { q | q distance(<= grow, side: inside) a }
  outer := std.Offset(b, d: grow)
  base := { q | q outside lo }
  tip := { q | q inside hi }
  inner_round := std.Rounded(inner, tip, r: r)
  outer_round := std.Rounded(outer, tip, r: r)
  region := { p | p inside inner; p inside outer; p inside base; p inside tip
                  p inside inner_round; p inside outer_round }
  construction crown := solid(region)
}
// A crown space, tip down, the gear's generator: the tooth's mate on the same cones, so the two
// crowns are complementary by construction — beyond `a` and within the cap, within `b` turned by
// `step`, each cone grown by `grow`, above `lo` and below `hi`, its tips' edges rounded. The
// outer mate is `outer_crown`, the inner `inner_crown`, turned `neighbor`.
component Space(a: set, b: set, lo: set, hi: set, cap: set, step: motion, grow: Length, r: Length) {
  inner := { q | q distance(<= grow, side: inside) a }
  outer := std.Offset(b, d: grow)
  tip := { q | q outside lo }
  base := { q | q inside hi }
  inner_round := std.Rounded(inner, tip, r: r)
  outer_round := std.Rounded(outer, tip, r: r)
  near := { p | p inside inner; p inside cap; p inside tip; p inside base; p inside inner_round }
  far := { p | p inside outer; p inside tip; p inside base; p inside outer_round }
  private construction outer_crown := solid(near)
  private construction inner_crown := solid(far)
  private construction neighbor := solid(inner_crown, under: step, at: 0deg)
  body := solid(outer_crown)
  neighbor common body
}

// -- generation --------------------------------------------------------------------------------
// A cut generated in `body`: `tool` swept under the generating roll `under` through `body` — the
// whole roll about 0 during which it meets the body's material, clear of it at both ends, found
// once the layout is solved — indexed by `index` at each of `n` teeth. A member is its blank less
// its crown's cut.
component Generated(body: solid, tool: solid, under: motion, index: motion, n: Int) {
  construction removal := solid(tool, under: under, through: body)
  repeat n as i {
    private construction indexed := solid(removal, under: index, at: i * 360deg / n)
    indexed cut body
  }
}
component Member(blank: solid, tool: solid, under: motion, index: motion, n: Int) {
  body := solid(blank)
  cut := Generated(body, tool, under, index, n: n)
}

// -- the tip relief ----------------------------------------------------------------------------
// A member's tip edges are generated by its crown's flanks near their base, the addendum from the
// pitch line; a topping leaves each flank `tip_relief` short of that, at a kink on the flank as
// grown in the normal section `n`, beyond the axis as its pitch point is (along `toward`, the
// cutter centre to it), turned the relief angle (30°) so the cut widens toward the base, and
// rounds into a top inside the crown: a blade through the kink (`Blade`, its `half` and
// `opening` as there). Its own sweep, cut the same way: a concave corner inside one sweep would
// trim two envelopes against each other (generating-sweep row T2).
component Topping(flank: set, ax: line, n: plane, toward: line, height: Length, half: Angle,
    opening: Angle) {
  kink := point in n
  kink coincident flank.surface
  ax.p1 distance(height, along: ax) kink
  ax.p1 distance(>= 0mm, along: toward) kink
  cone := Blade(ax, kink, half: half, opening: opening)
}

// A blade's cone about the cutter's axis `about` through `q`: `std.Cone`'s about the line
// `.spine`, from the apex to `q`'s foot on `about`, `half` the angle between the axis and every
// generator. The apex is on the axis where the generator from `q` to it makes `opening` with
// `about` — the supplement of the half where the cone opens along `about`, the half itself where
// it opens against it — seeded on that side of the axis's start. `std.ConeThrough` with the side
// the cone opens to said, so a half that reads an unknown (`pressure + pressure_shift`) is a
// number times it plus a number. A point on it, a line touching it, `inside`, `outside` and an
// offset read as a cone's.
component Blade(about: line, q: point, half: Angle, opening: Angle) := { p |
  apex := point hint(at: about.p1, along: about, by: cos(opening) / abs(cos(opening)))
  private foot := point hint(at: about.p1)
  apex coincident about
  private construction through := line(q, apex)
  about angle(opening) through
  foot coincident about
  private construction across := line(foot, q)
  across perpendicular about
  construction spine := line(apex, foot)
  private construction g := line(apex, p)
  spine angle(half) g
}

// -- the face mill's settings, found -------------------------------------------------------------
// The pressure shift leans the crown tooth: it steepens the flank that generates the pinion's
// concave side (the inner cone) and flattens the one that generates the gear's (the outer), so
// it trades the pinion's margin against undercut for the gear's. The cutter's radius sets how
// those margins run along the face. Neither is stated: each member's flank is the envelope its
// crown flank generates, held to by the solve (§6.15.3), and the two settings are found where
// the members stand equally far from undercut — in the normal section N through M, and in the
// one through the toe of the trace, half the face width short of M, where both run nearest it.
// Two settings, two balances. The toe's balance holds at more than one radius; the cutter is no
// smaller than the cone distance R, where both members stand farther from undercut along the
// whole face, and the seeds say which. The solve tears the two numbers out of the block that
// holds them, solving the rest for each try (solver.md).
component Balanced(P: plane, gear: Gear, trace: Trace, ax: line, inner: set,
    outer: set, pinion_generation: motion, gear_generation: motion, R: Length, face: Length,
    depth: Length, cutter_radius: Length) {
  gear.M distance(>= R) trace.C
  pinion_flank := envelope(inner, under: pinion_generation, from: -30deg, to: 30deg)
  gear_flank := envelope(outer, under: gear_generation, from: -30deg, to: 30deg)
  // the normal section through M, square to P on the trace normal, through the cutter's axis
  N := plane(u: trace.normal, v: ax)
  mean := Margins(N, gear.M, trace.normal, ax, trace.C, pinion_flank, gear_flank, depth: depth)
  // the toe of the trace: on the cutter's circle through M, half the face width nearer O, the
  // root within a face width of M
  toe := point in P
  trace.C distance(cutter_radius) toe
  gear.O distance(R - face / 2) toe
  gear.M distance(<= face) toe
  construction to_toe := line(trace.C, toe)
  T := plane(u: to_toe, v: ax)
  at_toe := Margins(T, toe, to_toe, ax, trace.C, pinion_flank, gear_flank, depth: depth)
  mean.pinion_profile equal mean.gear_profile
  at_toe.pinion_profile equal at_toe.gear_profile
}

// The members' margins against undercut in the section `s` through the cutter's axis `ax` and
// the trace's point `at`: on each member's flank nearest undercut — the pinion's generated by the
// inner cone, the gear's by the outer — the point where its mate's tip reaches, `depth` from P
// (the start of its active profile), on its side of the trace (`toward`, from the cutter's
// centre `c` out through `at`) and within two depths of `at`; and the circle osculating the
// section there, whose radius is how far the profile is from the cusp undercut would make.
component Margins(s: plane, at: point, toward: line, ax: line, c: point, pinion: envelope,
    gear: envelope, depth: Length) {
  pinion_start := point
  gear_start := point
  pinion_start coincident s
  gear_start coincident s
  pinion_start coincident pinion
  gear_start coincident gear
  c distance(depth, along: ax) pinion_start
  c distance(-depth, along: ax) gear_start
  at distance(<= 0mm, along: toward) pinion_start
  at distance(>= 0mm, along: toward) gear_start
  at distance(<= 2 * depth) pinion_start
  at distance(<= 2 * depth) gear_start
  in s {
    pinion_profile := circle
    gear_profile := circle
  }
  pinion curvature(at: pinion_start) pinion_profile
  gear curvature(at: gear_start) gear_profile
}
