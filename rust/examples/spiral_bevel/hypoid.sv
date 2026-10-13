// A spiral bevel or hypoid pair, whole: a 24-tooth pinion and a 48-tooth gear laid out the way a
// gear designer lays a hypoid out — the pitch cones and the tooth trace through one design point,
// the mean point M on the pitch plane P — then the generating crown as regions about its cutter's
// axis, and each member its blank less its crown rolled through it at every tooth. Every position
// follows from lines, angles, incidence and the stated numbers; nothing is computed with `sin` or
// `cos`. No point is seeded: every root is chosen by a word — a bound (`inside`, `outside`,
// `distance(>= 0, …)`), a sense, an ordinate's sign — and the layout solves from them alone
// (§9.6, #217). See README.md.
unit mm
use std (skew)

// -- step 1, the pair as a designer states it --------------------------------------------------
param pinion_teeth := 24
param gear_teeth := 48
param mean_module := 2mm
// The shafts: the angle between them, and the offset, their common perpendicular's length. No
// offset is a bevel pair with a common apex; any other, a hypoid.
param shaft_angle := 90deg
param offset := 25mm
// Added to the crown's inner flank pressure angle and taken from its outer's: an offset works a
// tooth's two sides at different effective pressure angles, and a hypoid balances them.
param pressure_shift := 12.5deg
// The crown's spiral angle at M; the pinion's adds the offset angle, which the layout solves for.
param spiral_angle := 25deg
// The allowances, each zero for the conjugate, sharp-edged pair: the normal backlash, a quarter
// off each crown flank; the tip relief, the tips' flank edges cut this far down the tooth; the
// end relief, the tips' toe and heel edges chamfered this far each way.
param backlash := 0.05mm
param tip_relief := 0.2mm
param end_relief := 0.2mm
// How far each member rolls through its crown, clear of its blank at both limits.
param pinion_roll_limit := 35deg
param gear_roll_limit := 45deg

// The proportions. The mean cone distance R (the crown's tooth count times half the module) sizes
// only the cutter (0.8 R), the face width (0.2 R) and the gear space cutter's reach.
R := mean_module * sqrt(pinion_teeth^2 + gear_teeth^2) / 2
pressure := 20deg
// The unknowns no point holds: the normal module, the module seen square to the tooth trace,
// which the trace constructs; the pinion's pitch radius at M, which the equal normal pitch sets;
// and the bevel pinion's pitch angle, read off the gear's triangle.
param mn: Length
param rp: Length
param pa: Angle
// The depths, in normal modules.
addendum := mn
dedendum := 1.25 * mn
base := 2 * mn
rounding := 0.3 * mn

// -- step 2, the pitch cones and the tooth trace -----------------------------------------------
// The gear apex O at P's origin and M on its u axis beyond it; the gear's right triangle O–M–F
// stands square to P on the generator O → M, F the foot of M on the gear's axis below P, its legs
// the two pitch radii at M. The pitch angle and the cone distance follow; nothing states them.
O := point in std.top
M := point in std.top
generator := line(O, M)
O coincident std.top.origin
M level(v) std.top
M distance(>= 0, along: u) std.top
foot := point
to_apex := line(M, O)
to_foot := line(M, foot)
gear_ax := line(O, foot)
O level(std.y) foot
foot inside std.top
to_foot perpendicular gear_ax
M distance(gear_teeth * mean_module / 2) foot
O distance(pinion_teeth * mean_module / 2) foot
// The crown's axis, up square to P at O (P is `std.top`, at the world's origin).
crown_ax := line(O)
fix((0, 0, 1)) crown_ax.p2

// The tooth trace: the circle about the cutter centre C through M, the cutter's radius 0.8 R, MC
// at 90° less the spiral to MO; its heading at M, square to MC, meets the line from O along MC at
// H (said as parallel, not as square to the heading, which a heading of no length would be). The
// normal module is the module seen square to the trace: how far from MC a point one module from
// M toward O stands, so O, R / m modules from M, stands R / m times as far.
C := point in std.top
H := point in std.top
normal := line(C, M)
heading := line(M, H)
to_heading := line(O, H)
M distance(0.8 * R) C
generator angle(90deg - spiral_angle, sense: cw) normal
heading perpendicular normal
to_heading parallel normal
O distance(mn * R / mean_module) normal

// The pinion's cone, solved against the gear's: its apex A in P on the offset's hand of O and
// behind M, its axis square to P on its generator A → M (each axis square to the level line
// `square`, running from the apex to the foot of M on it, above P), the shafts their angle and
// offset apart. The equal normal pitch sizes it, with no cosine written: V, where MA crosses the
// square from O to the heading, keeps |MV| cos(spiral + offset angle) = R cos(spiral); a virtual
// axis leaves V at the bevel pinion's pitch angle, the angle at M in the gear's triangle, and the
// pinion's pitch radius is one distance from M both axes keep. With no offset, A and V are O and
// the two axes one.
A := point in std.top
V := point in std.top
pitch_line := line(A, M)
V coincident pitch_line
V coincident to_heading
A distance(>= 0, along: v) std.top
A distance(>= 0, along: generator) M
pinion_foot := point
virtual_foot := point
pinion_ax := line(A, pinion_foot)
virtual_ax := line(V, virtual_foot)
square := axis
square perpendicular pitch_line
square perpendicular std.z
pinion_ax perpendicular square
virtual_ax perpendicular square
pinion_foot outside std.top
virtual_foot outside std.top
M level(pinion_ax) pinion_foot
M level(virtual_ax) virtual_foot
M distance(rp) pinion_foot
M distance(rp) virtual_foot
gear_ax skew(theta: shaft_angle, e: offset) pinion_ax
to_apex angle(pa) to_foot
pitch_line angle(pa) virtual_ax

// -- step 3, the blanks ------------------------------------------------------------------------
// A member's blank: within its pitch cone grown by the addendum and its heel sphere, outside its
// toe sphere and its pitch cone shrunk by the back depth, one region and its solid. The spheres
// are about the apex at the start of the generator `rim` (apex → M), the toe half the face width
// short of M and the heel half the face width beyond it. The end relief chamfers the tip's toe
// and heel corners `relief` each way (`std.Chamfered`); a relief of nothing cuts nothing. Each
// set the region uses names the face it bounds: `material.tip`, `.heel`, `.toe` and `.back`.
component Blank(rim: line, ax: line, design: group) {
  toe := std.Sphere(rim.p1)
  heel := std.Sphere(rim.p1)
  rim.p2 distance(design.width / 2, side: outside) toe
  rim.p2 distance(design.width / 2, side: inside) heel
  pitch := std.Cone(ax)
  rim.p2 coincident pitch
  tip := std.Offset(pitch, d: design.addendum)
  back := { q | q outside(design.back) pitch }
  beyond_toe := { q | q outside toe }
  region := { p | p inside tip; p inside heel; p outside toe; p inside back
                  p inside std.Chamfered(tip, beyond_toe, la: design.relief, lb: design.relief)
                  p inside std.Chamfered(tip, heel, la: design.relief, lb: design.relief) }
  material := solid(region)
}
blank_design := {width: 0.2 * R, addendum: addendum, back: 4 * mn, relief: end_relief}
gear_blank := Blank(generator, gear_ax, blank_design)
pinion_blank := Blank(pitch_line, pinion_ax, blank_design)

// -- step 4, the generating crown --------------------------------------------------------------
// The crown tooth's thickness: two points of the mean pitch circle a quarter crown pitch either
// side of M; the inner and outer trace circles about C pass through them, and its pitch points
// are where they cross the trace normal beyond C. The face mill: its axis up through C, square to
// P, the flank cones about it through the pitch points leaning by the pressure angle split by the
// shift, and the levels the crowns' tips and bases stand at.
ahead := arc(center: O, start: M) in std.top
behind := arc(center: O, end: M) in std.top
inner_pitch := point in std.top
outer_pitch := point in std.top
inner_trace := arc(center: C, start: ahead.end, end: inner_pitch)
outer_trace := arc(center: C, start: behind.start, end: outer_pitch)
to_inner := line(C, inner_pitch)
to_outer := line(C, outer_pitch)
length(pi * mean_module / 4) ahead
length(pi * mean_module / 4) behind
normal angle(180deg) to_inner
normal angle(180deg) to_outer
cut_ax := line(C)
cut_ax parallel crown_ax
C distance(0.8 * R, along: crown_ax) cut_ax.p2
inner_flank := std.ConeThrough(cut_ax, inner_pitch, lean: pressure + pressure_shift)
outer_flank := std.ConeThrough(cut_ax, outer_pitch, lean: pressure_shift - pressure)
tooth_tip := std.Level(C, cut_ax, d: dedendum)
tooth_base := std.Level(C, cut_ax, d: -base)
mate_tip := std.Level(C, cut_ax, d: -dedendum)
mate_base := std.Level(C, cut_ax, d: base)
// How far the gear's space cutter reaches: the cylinder about the cutter's axis through the
// point a little over the cone distance past O on the line from C through it, clear of every
// blank point.
cap := point in std.top
to_cap := line(C, O)
cap coincident to_cap
O distance(1.1 * R + mean_module, along: to_cap) cap
reach := std.Cylinder(cut_ax)
cap coincident reach

// A crown tooth about a cutter's axis, the pinion's generator: beyond the flank cone `a` and
// within `b`, each grown by `grow` (a quarter of the backlash: the envelope of an offset surface
// is the offset of the envelope), above the level `lo` and below `hi`, its tip's edges rounded by
// `r`; and its solid, `crown`. Each set names the face it bounds.
component Tooth(a: set, b: set, lo: set, hi: set, grow: Length, r: Length) {
  inner := { q | q outside(grow) a }
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
  inner := { q | q outside(grow) a }
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
  neighbor bound body
}

// -- step 5, generation ------------------------------------------------------------------------
// Every roll shares the crown's angle, about its axis square to P at O, and each member turns at
// the crown's tooth count over its own, read off the gear's triangle after the solve: its
// hypotenuse R over the gear's pitch radius, and over the short leg for the pinion, whose own
// cone, solved off the bevel, would roll at another. Indexing turns a member; the crown's
// neighbour is one crown pitch round, four quarter pitches of the mean pitch circle.
crown_roll := motion(about: crown_ax)
pinion_roll := motion(about: pinion_ax, ratio: distance(O, M) / distance(O, foot))
gear_roll := motion(about: gear_ax, ratio: -distance(O, M) / distance(M, gear_ax))
pinion_generation := motion(crown_roll, relative_to: pinion_roll)
gear_generation := motion(crown_roll, relative_to: gear_roll)
pinion_index := motion(about: pinion_ax)
gear_index := motion(about: gear_ax)
crown_neighbor := motion(about: crown_ax, phase: -4 * length(ahead) / radius(ahead) * 1rad)

tooth := Tooth(inner_flank, outer_flank, tooth_base, tooth_tip, grow: backlash / 4, r: rounding)
construction gear_space := Space(outer_flank, inner_flank, mate_tip, mate_base, reach,
  crown_neighbor, grow: backlash / 4, r: rounding)

// A cut generated in `body`: `tool` swept under the generating roll `under` from -`roll` to
// `roll`, indexed by `index` at each of `n` teeth. A member is its blank less its crown's cut.
component Generated(body: solid, tool: solid, under: motion, index: motion, n: Int, roll: Angle) {
  construction removal := solid(tool, under: under, from: -roll, to: roll)
  repeat n as i {
    private construction indexed := solid(removal, under: index, at: i * 360deg / n)
    indexed cut body
  }
}
component Member(blank: solid, tool: solid, under: motion, index: motion, n: Int, roll: Angle) {
  body := solid(blank)
  cut := Generated(body, tool, under, index, n: n, roll: roll)
}
pinion := Member(pinion_blank.material, tooth.crown, pinion_generation, pinion_index,
  n: pinion_teeth, roll: pinion_roll_limit)
gear := Member(gear_blank.material, gear_space.body, gear_generation, gear_index,
  n: gear_teeth, roll: gear_roll_limit)

// -- the tip relief ----------------------------------------------------------------------------
// A member's tip edges are generated by its crown's flanks near their base, the addendum from the
// pitch line; a topping leaves each flank `tip_relief` short of that, at a kink on the flank as
// grown in the normal section N (square to P on the trace normal, through the cutter's axis),
// beyond C as its pitch point is (along `toward`, C to it), turned the relief angle (30°) so the
// cut widens toward the base, and rounds into a top inside the crown. Its own sweep, cut the same
// way: a concave corner inside one sweep would trim two envelopes against each other
// (generating-sweep row T2).
component Topping(flank: set, ax: line, n: plane, toward: line, height: Length, lean: Angle) {
  kink := point in n
  kink coincident flank.surface
  ax.p1 distance(height, along: ax) kink
  ax.p1 distance(>= 0mm, along: toward) kink
  cone := std.ConeThrough(ax, kink, lean: lean)
}
when tip_relief > 0mm {
  N := plane(u: normal, v: cut_ax)
  depth := addendum - tip_relief
  inner_lean := pressure + pressure_shift + 30deg
  outer_lean := pressure_shift - pressure - 30deg
  pinion_inner := Topping(tooth.inner, cut_ax, N, to_inner, height: -depth, lean: inner_lean)
  pinion_outer := Topping(tooth.outer, cut_ax, N, to_outer, height: -depth, lean: outer_lean)
  pinion_top := std.Level(C, cut_ax, d: -addendum / 2)
  construction pinion_topping := Tooth(pinion_inner.cone, pinion_outer.cone, tooth_base,
    pinion_top, grow: 0mm, r: rounding)
  gear_outer := Topping(gear_space.inner, cut_ax, N, to_outer, height: depth, lean: outer_lean)
  gear_inner := Topping(gear_space.outer, cut_ax, N, to_inner, height: depth, lean: inner_lean)
  gear_top := std.Level(C, cut_ax, d: addendum / 2)
  construction gear_topping := Space(gear_outer.cone, gear_inner.cone, gear_top, mate_base,
    reach, crown_neighbor, grow: 0mm, r: rounding)
  pinion_relief := Generated(pinion.body, pinion_topping.crown, pinion_generation,
    pinion_index, n: pinion_teeth, roll: pinion_roll_limit)
  gear_relief := Generated(gear.body, gear_topping.body, gear_generation, gear_index,
    n: gear_teeth, roll: gear_roll_limit)
}
