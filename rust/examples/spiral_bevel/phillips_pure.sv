// Phillips' pair in the pure form: step 4a of #283 (#326). `phillips.sv` makes each flank with a
// rack, a plane sliding along the line of action L. Here nothing generates anything: each flank is
// the contact element as its member sees it. The element is a line of the member's plane of action
// — the plane through L along its shaft — moving along L as fast as both members move along it:
// at phase `phi` it is `s` across L from the point `sigma phi` along it. The member, turned `phi`,
// sees it carried back: `X coincident(under: turn, t == -phi) x`. Over `phi` and `s` that is a
// surface traced in space, and it is the member's involute helicoid — the very flank the rack
// sweeps (`tests/phillips.rs`), with no rack in its definition.
unit mm
use std (reciprocal, turns)

param pinion_teeth := 24
param gear_teeth := 48
param module := 2mm
param pressure := 20deg

rg := module * gear_teeth / 2
rp := module * pinion_teeth / 2
q := pinion_teeth / gear_teeth
// the speed along L per radian of the pinion's phase, as `phillips.sv` states it
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

// -- the line of action, as `phillips.sv` holds it -----------------------------------------------
P := point hint((0, 48, 0))
fix((0mm, rg, 0mm)) P
ahead := point hint((7, 51, -7))
L := line(P, ahead)
P distance(10mm) ahead
P distance(10mm * sin(pressure), along: std.y) ahead
L reciprocal(seen_from: pinion_turn) gear_turn

// -- each member's plane of action ---------------------------------------------------------------
// Through L along the member's shaft: its normal square to both, and a line in it across L.
pn := point hint((0, 48, 10))
pin_normal := line(P, pn)
pin_normal perpendicular L
pin_normal perpendicular pinion_ax
P distance(10mm) pn
pa := point hint((-7, 51, -7))
pin_across := line(P, pa)
pin_across perpendicular L
pin_across perpendicular pin_normal
P distance(10mm) pa
gn := point hint((10, 48, 0))
gear_normal := line(P, gn)
gear_normal perpendicular L
gear_normal perpendicular gear_ax
P distance(10mm) gn
ga := point hint((0, 45, 10))
gear_across := line(P, ga)
gear_across perpendicular L
gear_across perpendicular gear_normal
P distance(10mm) ga

// -- the flanks, as their members see the contact -------------------------------------------------
// The element at phase `phi`, `s` across L in the plane of action, and the member's view of it:
// carried back by the member's turn.
component Element(P: point, L: line, across: line, normal: line, m: frame, v: Length,
    phi: Angle, s: Length) {
  x := point hint((0, 48, 0))
  P distance(v * phi / 1rad, along: L) x
  P distance(s, along: across) x
  P level(normal) x
  X := point hint((0, 48, 0))
  X coincident(under: m, t == -phi) x
}
run := 40deg
w := 15mm
pinion_flank := trace(Element(P, L, pin_across, pin_normal, pinion_turn,
                              v: sigma).X over (phi in (-run, run), s in (-w, w)))
gear_flank := trace(Element(P, L, gear_across, gear_normal, gear_turn,
                            v: sigma).X over (phi in (-run, run), s in (-w, w)))

// -- the contact ---------------------------------------------------------------------------------
bearing := meshing(pinion_flank, gear_flank over t in (-15deg, 15deg), under: gear_turn,
                   seen_from: pinion_turn)
Q := point hint((0, 48, 0))
Q coincident(t == 0deg) bearing
