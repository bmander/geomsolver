// Phillips' spatial involute gearing: step 4a of #283 (#326). A pair on skew shafts is one line
// of action L. A force along L does no work on the members' relative motion when their speeds
// along it match, and then each member's flank is the involute helicoid a plane square to L
// sweeps as it slides along L at that speed, seen from the member: its rack. The two flanks touch
// where L meets the rack, with the rack's normal, at the ratio's roll: no transmission error, and
// a point contact, since each member's plane of action — the plane through L along its shaft — is
// its own.
//
// The shafts are `mean_point.sv`'s, square to one another and the two pitch radii apart. L runs
// through the pitch point P on their common perpendicular, leaning the pressure angle from the
// pitch plane, and it is **reciprocal** to the mesh: a force along it does no work on the gear's
// motion seen from the pinion, so the two move as fast along it. That settles its helix angle.
// The rack slides along L at that speed, `sigma`, and is reciprocal to each member's cutting in
// turn — it moves along L as the member does — which the claims check (`tests/phillips.rs` reads
// the pair against the closed form).
unit mm
use std (reciprocal, turns, slides)

param pinion_teeth := 24
param gear_teeth := 48
param module := 2mm
param pressure := 20deg

rg := module * gear_teeth / 2
rp := module * pinion_teeth / 2
q := pinion_teeth / gear_teeth
// the speed along L per radian of the pinion's phase: the members' pitch speeds, `rp` square to
// the pinion's shaft and `q rg` square to the gear's, seen along L
sigma := q * rg * rp * cos(pressure) / sqrt(rp^2 + (q * rg)^2)

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
gear_turn turns(-q) gear_ax
pinion_turn := frame
pinion_turn turns(1) pinion_ax
// the mesh is the gear's turn seen from the pinion's: `under: gear_turn, seen_from: pinion_turn`

// -- the line of action --------------------------------------------------------------------------
P := point hint((0, 48, 0))
fix((0mm, rg, 0mm)) P
ahead := point hint((7, 51, -7))
L := line(P, ahead)
P distance(10mm) ahead
P distance(10mm * sin(pressure), along: std.y) ahead
L reciprocal(seen_from: pinion_turn) gear_turn
// the mesh's instantaneous screw axis, on the common perpendicular, and its pitch: `screw.isa`,
// running the way `toward` does, and `screw.pitch`, the advance a radian of the mesh's roll — both
// read by rates of points the mesh carries (std.sv)
toward := axis                               // the way it runs: back along x, as the mesh turns
fix(dir == (-1, 0, 0)) toward
screw := std.Screw(toward, gear_turn, seen_from: pinion_turn)

// -- the rack and the flanks ---------------------------------------------------------------------
// The plane square to L through P, sliding along L at the speed both members have along it; each
// member's flank is what it sweeps, seen from that member.
rack := std.HalfSpace(P, L, d: 0mm)
slide := frame
slide slides(2 * pi * sigma) L
pinion_flank := envelope(rack over t in (-40deg, 40deg), under: slide, seen_from: pinion_turn)
gear_flank := envelope(rack over t in (-40deg, 40deg), under: slide, seen_from: gear_turn)
claim L reciprocal(seen_from: pinion_turn) slide
claim L reciprocal(seen_from: gear_turn) slide

// -- the contact ---------------------------------------------------------------------------------
bearing := meshing(pinion_flank, gear_flank over t in (-15deg, 15deg), under: gear_turn,
                   seen_from: pinion_turn)
Q := point hint((0, 48, 0))
Q coincident(t == 0deg) bearing
