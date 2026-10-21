// The hypoid pair stated as its meshing problem: step 5 of #283 (#345, S4). The pair is
// `formate.sv`'s — the gear formed by a face mill, the pinion its blank less the gear's tooth swept
// under the mesh — with the gear's concave flank eased off the face mill's cone, so the pair
// touches at a point and runs with a transmission error. Nothing here states the ease. The
// designer states the contact — the mean point it is drawn at, the pressure angle of its
// common normal, the angle its path makes, the lag at the start of the phase, and that the pinion
// does not fold within its blank — and the ease is found: the profile radius of the gear's
// concave blade and the radius it is cut at. See README.md.
unit mm
use std (reciprocal)
use hypoid (GearCone, ToothTrace, PinionCone, Blank, FaceMill, Tooth, Member, Gear)
use formate (Formed)

// -- the pair as a designer states it ----------------------------------------------------------
// The layout is `formate.sv`'s: the tooth counts, the module at M, the shafts and the spiral. The
// contact is the one the face mill's inner blades make, the gear's concave flank and the pinion's
// convex, about the mean point:
// - `pressure`, how far its common normal leans from the plane square to the face mill's axis;
// - `path_angle`, the angle the path of contact makes with the tooth trace's heading at M, the
//   path running outward along the trace normal;
// - `lag`, the transmission error at the start of the phase, half a pinion pitch before the pair
//   is drawn: how far the pinion stands behind the ratio (negative) when the flanks touch, in
//   degrees of the pinion, as a meshing's lag is.
// The backlash is all the gear's: each flank of its spaces stands half of it off the face mill,
// so the pinion's cutter is the face mill's own tooth and the solve reads the pinion's undercut.
param pinion_teeth := 24
param gear_teeth := 48
param mean_module := 2mm
param shaft_angle := 90deg
param offset := 25mm
param spiral_angle := 25deg
param pressure := 30deg
param path_angle := 70deg
param lag := -0.03
param backlash := 0.05mm
// The pair on the pitch plane P, `std.top`: `pair.pinion.body` and `pair.gear.body` are the
// members. The ease's two curvatures are left to the solve, which starts them at the seeds.
pair := MeshingPair(std.top, std.y, std.z, pinion_teeth: pinion_teeth, gear_teeth: gear_teeth,
  mean_module: mean_module, shaft_angle: shaft_angle, offset: offset,
  spiral_angle: spiral_angle, pressure: pressure, path_angle: path_angle, lag: lag,
  backlash: backlash, profile_ease: hint(-0.01), lengthwise_ease: hint(-0.001))

// -- the pair ----------------------------------------------------------------------------------
// The layout, the blanks, the face mill and the pinion are `formate.sv`'s. The unknowns no point
// holds are its three formals (`mn`, `rp`, `pa`) and the ease's two curvatures, per millimetre:
// `profile_ease` across the tooth trace and `lengthwise_ease` along it (seeded: the search
// starts there).
component MeshingPair(P: plane, across: axis, up: axis, pinion_teeth: Int, gear_teeth: Int,
    mean_module: Length, shaft_angle: Angle, offset: Length, spiral_angle: Angle,
    pressure: Angle, path_angle: Angle, lag: Scalar, backlash: Length, mn: Length, rp: Length,
    pa: Angle, profile_ease: Scalar, lengthwise_ease: Scalar) {
  R := mean_module * sqrt(pinion_teeth^2 + gear_teeth^2) / 2
  dedendum := 1.25 * mn
  base := 2 * mn
  rounding := 0.3 * mn

  // The pitch cones, the tooth trace and the blanks, as `formate.sv` lays them out.
  gear_cone := GearCone(P, across, pinion_teeth: pinion_teeth, gear_teeth: gear_teeth,
    mean_module: mean_module)
  trace := ToothTrace(P, gear_cone, R: R, spiral_angle: spiral_angle, mean_module: mean_module,
    mn: mn, cutter_radius: 0.8 * R)
  pinion_cone := PinionCone(P, up, gear_cone, trace, shaft_angle: shaft_angle, offset: offset,
    rp: rp, pa: pa)
  blank_design := {width: 0.2 * R, addendum: mn, back: 4 * mn, relief: 0mm}
  gear_blank := Blank(gear_cone.generator, gear_cone.ax, blank_design)
  pinion_blank := Blank(pinion_cone.pitch_line, pinion_cone.ax, blank_design)

  // The face mill. Its inner blades lean the stated pressure angle: a cone's normal leans from
  // the plane square to its axis by the cone's half angle, so the pressure shift is the stated
  // angle less the face mill's 20°.
  mill := FaceMill(P, gear_cone, trace, R: R, mean_module: mean_module, pressure: 20deg,
    pressure_shift: pressure - 20deg, cutter_radius: 0.8 * R, dedendum: dedendum, base: base)

  // The motions, as `formate.sv`'s: the mesh is the gear's turn seen from the pinion's
  // (`under: gear_turn, seen_from: pinion_turn`); and the pair seen from the gear, the drive: the
  // pinion's turn seen from the gear's (`under: pinion_turn, seen_from: gear_turn`), the phase the
  // pinion's.
  gear_index := frame
  std.Turn(gear_index, about: gear_cone.ax, ratio: 1)
  pinion_turn := frame
  std.Turn(pinion_turn, about: pinion_cone.ax, ratio: 1)
  gear_turn := frame
  std.Turn(gear_turn, about: gear_cone.ax, ratio: -pinion_teeth / gear_teeth)

  // The pinion, generated by the gear's tooth as `formate.sv` generates it — the ring between the
  // face mill's cones, nothing grown — so each pinion flank is conjugate to a face mill cone.
  gear_tooth := Tooth(mill.inner_flank, mill.outer_flank, mill.tooth_base, mill.tooth_tip,
    grow: 0mm, r: rounding)
  pinion := Member(pinion_blank.material, gear_tooth.crown, gear_turn, pinion_turn, pinion_turn,
    n: pinion_teeth)

  // The mean contact T: on the face mill's inner cone in the normal section through M, where the
  // cone's normal is a line of action of the mesh.
  mean := MeanContact(gear_cone, trace.heading, mill.ax, mill.inner_flank, gear_turn,
    pinion_turn)

  // -- the contact as the designer states it ----------------------------------------------------
  // The pinion's convex flank, the envelope of the face mill's inner cone under the mesh, and its
  // meshing with the gear's concave flank: the cone eased about T, along the tooth trace and
  // across it, by the two curvatures. Conjugate, the flanks would touch along a line; eased, at a
  // point, T where the pair is drawn, the phase 0, and the pinion lags the relief elsewhere.
  pinion_flank := envelope(mill.inner_flank over t in (-60deg, 60deg), under: gear_turn,
                           seen_from: pinion_turn)
  contact := meshing(mill.inner_flank, pinion_flank over t in (-7.5deg, 7.5deg),
    under: pinion_turn, seen_from: gear_turn, ease: mean.T, along: trace.heading,
    profile: profile_ease / 1mm, lengthwise: lengthwise_ease / 1mm)
  Q := point hint(at: mean.T)
  Q coincident(t == 0deg) contact
  // the path of contact there — the contact point's rate along the phase — at the stated angle
  // to the trace's heading
  ahead := point hint((53.5, -8.6, -2.3))
  path := line(Q, ahead)
  rate(Q over t) parallel path
  Q distance(10mm) ahead
  Q distance(>= 0mm, along: trace.normal) ahead
  path angle(path_angle) trace.heading
  // the transmission error: the lag at the start of the phase
  start := point hint(at: mean.T)
  start coincident(t == -7.5deg, lag == lag) contact
  // no undercut: the pinion's convex flank does not fold where the gear's tooth cuts it
  pinion_flank regular(by: gear_tooth.crown) within pinion_blank.material

  // -- the gear, cut to the ease ----------------------------------------------------------------
  // The gear's concave blade, curved and set off the face mill's to cut the eased flank, half the
  // backlash into the tooth; and the gear formed as `formate.sv` forms it, its convex flanks by
  // the face mill's outer cone half the backlash in.
  blade := EasedBlade(mean, mill.ax, mill.inner_flank, trace.heading, profile: profile_ease,
    lengthwise: lengthwise_ease, half: pressure, grow: backlash / 2, reach: 4 * base)
  construction gear_space := Space(mill.outer_flank, blade.cutter, mill.mate_tip,
    mill.mate_base, mill.reach, gear_index, pitch: 360deg / gear_teeth, grow: backlash / 2,
    r: rounding)
  gear := Formed(gear_blank.material, gear_space.body, gear_index, n: gear_teeth)
}

// -- the mean contact --------------------------------------------------------------------------
// What the eased blade reads of the mean contact.
interface Contact { T: point, normal: line, generator: line }

// The point T where the pair touches at the phase it is drawn: on the face mill's cone `flank`,
// in the normal section through M — square to the trace's `heading`, so through the trace normal
// and the mill's axis `ax` — where the cone's normal is a line of action of the mesh: a force
// along it does no work on the members' relative motion, so two flanks touching there with that
// normal touch as they turn. The mesh is `mesh` seen from `seen_from`.
// `normal` runs from T to the mill's axis; `generator`, the cone's line from its apex through T.
component MeanContact(gear: Gear, heading: line, ax: line, flank: set, mesh: frame,
    seen_from: frame) {
  T := point hint((53.2, 1, 0.6))
  gear.M level(heading) T
  T coincident flank
  to_axis := point hint((35.5, 38.9, 24.8))
  to_axis coincident ax
  normal := line(T, to_axis)
  generator := line(flank.apex, T)
  normal perpendicular generator
  normal reciprocal(seen_from: seen_from) mesh
}

// -- the eased blade ---------------------------------------------------------------------------
// The face mill's inner blade cut to the ease about T: an arc in N tangent to the cone's generator
// at T, turned about an axis parallel to the mill's in N. A surface of revolution through T with
// the cone's normal there, its two curvatures at T are the cone's changed by the ease's (each
// `profile` or `lengthwise` per millimetre, along the cone's normal toward the axis as the
// meshing reads it): across the trace, the arc's — bent from the generator by the ease's
// parabola at `bent`, 2 mm along it; along the trace, the circle the axis turns T on — through
// `eased`, the cone's point 2 mm along the trace at T's height moved by the parabola, as far
// again as the normal leans from the circle's plane (`half`). `cutter`: what lies between the
// arc, `grow` beyond T into the tooth, and a millimetre off the axis, `reach` below T to `reach`
// above.
component EasedBlade(mean: Contact, ax: line, flank: set, heading: line, profile: Scalar,
    lengthwise: Scalar, half: Angle, grow: Length, reach: Length) {
  // N, the normal section: through T and the mill's axis, standing on `radial`, from T to its
  // foot on the axis, and `upright`, up the axis from there — lines seeded here, since a plane
  // over the mill's axis itself starts its axis where the solve leaves it stale
  foot := point hint((35.5, 38.9, 0.6))
  foot coincident ax
  foot level(ax) mean.T
  radial := line(mean.T, foot)
  above := point hint((35.5, 38.9, 10.6))
  above coincident ax
  foot distance(10mm, along: ax) above
  upright := line(foot, above)
  N := plane(u: radial, v: upright)
  // across the trace: the arc's centre on the normal beyond T, as far from `bent` as from T
  bent := point in N hint(at: mean.T)
  mean.T distance(2mm, along: mean.generator) bent
  mean.T distance(2mm * profile, along: mean.normal) bent
  centre := point in N hint(at: mean.T, along: mean.normal, by: -1)
  to_centre := line(mean.T, centre)
  to_centre perpendicular mean.generator
  mean.T distance(<= 0mm, along: mean.normal) centre
  line(centre, mean.T) equal line(centre, bent)
  // along the trace: the axis through `turn_centre`, level with T and as far from `eased`
  on_cone := point hint(at: mean.T)
  on_cone coincident flank
  on_cone level(ax) mean.T
  mean.T distance(2mm, along: heading) on_cone
  eased := point hint(at: mean.T)
  eased level(ax) mean.T
  on_cone level(heading) eased
  on_cone distance(2mm * lengthwise / cos(half), along: radial) eased
  turn_centre := point in N hint(at: mean.T, along: radial, by: 1.15)
  turn_centre level(ax) mean.T
  line(turn_centre, mean.T) equal line(turn_centre, eased)
  // the blade grown into the tooth: the arc about the same centre through `formed`
  formed := point in N hint(at: mean.T)
  mean.T level(mean.generator) formed
  mean.T distance(-grow, along: mean.normal) formed
  low := point in N hint(at: mean.T, along: mean.generator, by: -0.2)
  high := point in N hint(at: mean.T, along: mean.generator, by: 0.2)
  turn_centre distance(-reach, along: ax) low
  turn_centre distance(reach, along: ax) high
  line(centre, low) equal line(centre, formed)
  profile_arc := arc(center: centre, start: low, end: high) in N
  bottom := point in N hint(at: turn_centre, along: ax, by: -0.37)
  top := point in N hint(at: turn_centre, along: ax, by: 0.37)
  turn_centre distance(-1mm, along: radial) bottom
  turn_centre distance(-1mm, along: radial) top
  turn_centre distance(-reach, along: ax) bottom
  turn_centre distance(reach, along: ax) top
  lower := line(low, bottom)
  inner := line(bottom, top)
  upper := line(top, high)
  start := point in N hint(at: turn_centre, along: ax, by: -0.37)
  finish := point in N hint(at: turn_centre, along: ax, by: 0.37)
  turn_centre level(radial) start
  turn_centre level(radial) finish
  turn_centre distance(-reach, along: ax) start
  turn_centre distance(reach, along: ax) finish
  turned_about := line(start, finish)
  construction cutter := solid(face(profile_arc, lower, inner, upper), about: turned_about)
}

// -- the gear's space --------------------------------------------------------------------------
// A gear space, tip down, as `formate.sv`'s `Space` cuts it: beyond the outer flank cone `a` of
// the tooth behind it, grown by `grow`, within the cap, above the cutter's tip `lo` and below
// `hi`, that floor's edge rounded by `r` — and within the eased blade `flank` of the tooth ahead,
// that tooth `step` read at `pitch`.
component Space(a: set, flank: solid, lo: set, hi: set, cap: set, step: frame, pitch: Angle,
    grow: Length, r: Length) {
  convex := { q | q distance(<= grow, side: inside) a }
  tip := { q | q outside lo }
  base := { q | q inside hi }
  convex_round := std.Rounded(convex, tip, r: r)
  near := { p | p inside convex; p inside cap; p inside tip; p inside base; p inside convex_round }
  private construction behind := solid(near)
  private construction ahead := flank at(under: step) pitch
  body := solid(behind)
  ahead common body
}
