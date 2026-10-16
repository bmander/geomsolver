// Contact analysis at the mean point: step 3 of #283. A pair is not cut to be conjugate. The pinion
// is cut by a cutter larger than the gear's, so the two flanks touch at a point and not along a
// line, and at a modified roll, so the gear runs a little behind or ahead of the ratio. Where they
// touch, and how far the gear lags, is not stated anywhere here. It is the meshing of the two
// flanks: at each phase of the pinion, the turn of the gear at which the flanks share a point and
// a normal.
//
// The mesh is `mean_point.sv`'s, the hypoid's at its mean point reduced to its kinematics: crossed
// helical gears, so the answer has a closed form to be read against. The gear is formed (its flank
// is the face mill's cone, its blades leaning the pressure angle less the tilt, the lean
// `mean_point.sv` solves for). The pinion's cutter is that cone grown about M, and the two cones
// share the generator through M and their normal along it and nothing else. So the bearing runs
// along that generator, and the pinion, cut at a roll `roll` times the mesh's, is conjugate to the
// gear at that ratio: the gear lags `(roll − 1)` times the phase, exactly (`tests/meshing.rs`).
unit mm
use std

param pinion_teeth := 24
param gear_teeth := 48
param module := 2mm
param pressure := 20deg
param tilt := 5deg
param cutter_radius := 40mm
// the pinion's cutter, grown about M; and its roll, against the mesh's
param grown := 1.25
param roll := 1.005

rg := module * gear_teeth / 2
rp := module * pinion_teeth / 2

// -- the mesh ------------------------------------------------------------------------------------
g0 := point hint((0, 0, 0))
g1 := point hint((0, 0, 10))
fix((0, 0, 0)) g0
fix((0mm, 0mm, 10mm)) g1
gear_ax := line(g0, g1)
p0 := point hint((0, 72, 0))
p1 := point hint((10, 72, 0))
fix((0mm, rg + rp, 0mm)) p0
fix((10mm, rg + rp, 0mm)) p1
pinion_ax := line(p0, p1)
gear_turn := frame
std.Turn(gear_turn, about: gear_ax, ratio: -pinion_teeth / gear_teeth)
pinion_turn := frame
std.Turn(pinion_turn, about: pinion_ax, ratio: 1)
// the mesh is the gear's turn seen from the pinion's: `under: gear_turn, seen_from: pinion_turn`

// -- the gear: formed by the face mill -----------------------------------------------------------
M := point hint((0, 48, 0))
fix((0mm, rg, 0mm)) M
toward := point hint((30, 48, -30))
fix((30mm, rg, -30mm)) toward
across := line(M, toward)
C := point hint((28, 48, -28))
C coincident across
M distance(cutter_radius) C
normal_plane := plane(u: across, v: std.y)
top := point hint((27, 58, -27))
top coincident normal_plane
C distance(10mm) top
mill_ax := line(C, top)
mill_ax angle(tilt) std.y
apex := point hint((35, -96, -35))
apex coincident mill_ax
spine := line(apex, C)
flank := std.Cone(spine, half: pressure - tilt)
M coincident flank

// -- the pinion: cut by a larger cutter at a modified roll ---------------------------------------
generator := line(M, apex)
cut_apex := point hint((44, -132, -44))
cut_apex coincident(t == grown) generator
cut_top := point hint((44, -122, -44))
cut_spine := line(cut_apex, cut_top)
cut_spine parallel spine
cut_apex distance(10mm) cut_top
cutter := std.Cone(cut_spine, half: pressure - tilt)
cutting_turn := frame
std.Turn(cutting_turn, about: gear_ax, ratio: -roll * pinion_teeth / gear_teeth)
pinion_flank := envelope(cutter over t in (-40deg, 40deg), under: cutting_turn,
                         seen_from: pinion_turn)

// -- the contact ---------------------------------------------------------------------------------
// The two flanks turning with their members; Q, where they touch at the phase the pair is drawn
// at. The trace over the phase is the bearing and the transmission error.
bearing := meshing(pinion_flank, flank over t in (-15deg, 15deg), under: gear_turn,
                   seen_from: pinion_turn)
Q := point hint((0, 48, 0))
Q coincident(t == 0deg) bearing
