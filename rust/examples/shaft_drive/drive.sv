// A chainless bicycle drive in a road frame: a shaft sealed inside the drive-side chainstay, a
// step-up gearbox inside the bottom bracket's shell and a step-down gearbox inside the drive-side
// dropout, every shaft in its bearings and every bearing held by its retaining ring.  See README.md
// for the design and what solventc could not yet draw of it, and `dims.sv` for the numbers.
//
// The world is the bicycle's: `std.x` forward, `std.y` to the rider's left, `std.z` up, the
// bottom bracket's centre at the origin, the frame's mid-plane `y = 0`.  The drive side is the
// right, toward −y.
unit mm
use std
use std.sets
use std.solids
use std.solids (cut, union)
use std.rounds (round)
use std.place (seats, right_of)
use spiral_bevel.hypoid (HypoidPair)
use planetary
use frame
use std.ports (seats_in, abuts, grooves)
use std.bearings
use std.retaining
use dims (cs_length, bb_drop, pinion_y, ring_y, pinion_teeth, gear_teeth, bevel_module,
          bevel_angle, bevel_backlash, bevel_tip_relief, bevel_end_relief, bevel_cutter_radius,
          shell_od, shell_id, case_left, case_right, end_wall, diaphragm, cup, boss_r, boss_ahead,
          boss_behind, wall, web, pinion_bearing, sleeve_r, hub_r, hub_bore, hub_span, hub_bearings,
          flange_r, flanges, spokes, rear_case_r, rear_case, axle_d, old, shaft_od, shaft_id,
          tube_od, tube_id, spindle_d, spindle_bore, crank_throw, crank_face)

// -- the frame's datums --------------------------------------------------------------------------
// The bottom bracket's axis is the world's `std.y`.  The rear axle stands `cs_length` from it,
// `bb_drop` above it, behind; its axis runs along `std.y` through its centre.
in std.front {
  axle_c := point
  std.origin distance(bb_drop, along: v) axle_c
  std.origin distance(cs_length) axle_c
  std.origin distance(<= 0mm, along: u) axle_c
  construction centres := line(std.origin, axle_c)
  mid := point
  mid midpoint centres
}
axle := axis
axle parallel(sense: same) std.y
axle_c coincident axle
// the two axes as drawn lines, for ordinates along them and the sets about them
y_end := point
y_end coincident std.y
std.origin distance(10mm, along: std.y) y_end
construction bb_line := line(std.origin, y_end)
axle_end := point
axle_end coincident axle
axle_c distance(10mm, along: std.y) axle_end
construction axle_line := line(axle_c, axle_end)
drive_side := axis
drive_side parallel(sense: opposite) std.y
std.origin coincident drive_side

// -- the spiral bevel pairs ---------------------------------------------------------------------
// One pair, laid out and cut once, standing twice.  At the bottom bracket its gear rides the sun's
// sleeve, its axis the bracket's, and drives the pinion, whose axis is the shaft's: it runs back
// along the line of centres seen from the side, splaying `splay` outward on the drive side seen
// from above, the pair's shafts meeting at `bevel_angle`.  The pinion's mean point stands
// `pinion_y` out on the drive side.
//
// At the hub the same pair stands turned end for end: half a turn about `flip`, the line square to
// the axles and to the shaft through the shaft's point over the middle of the line of centres.
// The turn takes the bracket's axis to a line along the axle's, through the axle's centre, and the
// shaft's axis to itself, run the other way: so the hub's gear turns on the axle and the shaft's
// two pinions share one axis by construction.  The shaft's pinion drives the hub's gear.
//
// The pair is placed by what it does (#904 A5): its frame's origin on the bracket's axis, its
// gear's axis along it, its pinion's axis square to `flip` and running back, the pinion's mean
// point at `pinion_y`.  With no offset the face mill's shift is found where the pinion's two
// flanks stand equally far from undercut (`hypoid.sv`).
pair := HypoidPair(pinion_teeth: pinion_teeth, gear_teeth: gear_teeth,
  mean_module: bevel_module, shaft_angle: bevel_angle, offset: 0mm, spiral_angle: 25deg,
  backlash: bevel_backlash, tip_relief: bevel_tip_relief, end_relief: bevel_end_relief,
  pressure_shift: hint(10deg), cutter_radius: bevel_cutter_radius, least: hint(0.5))
pair.margins.pinion_inner regular(pair.least, by: pair.margins.pinion_cut) within pair.margins.pinion
pair.margins.pinion_outer regular(pair.least, by: pair.margins.pinion_cut) within pair.margins.pinion
flip := axis
flip perpendicular std.y
flip perpendicular centres
pair.std.origin coincident std.y
pair.gear_member.ax parallel(sense: same) std.y
pair.pinion_member.ax perpendicular flip
pair.pinion_member.ax.p1 distance(<= 0mm, along: std.x) pair.pinion_member.ax.p2
std.origin distance(pinion_y, along: std.y) pair.pinion_member.foot
flip_c := point
flip_c coincident pair.pinion_member.ax
mid distance(0mm, along: centres) flip_c
flip_c coincident flip

// The pinion's mean point at the hub, on the shaft's axis.
hub_foot := pair.pinion_member.foot at(under: flip) 180deg
claim hub_foot coincident pair.pinion_member.ax

// Each member is made one part with a web filling its core: the points deeper inside its pitch
// cone than its blank's back, in a band along its axis about the foot of its mean point (as
// `hypoid_drive/drive.sv` hubs its members).  The bracket's gear's web is bored to sit on the sun's
// sleeve, the hub's on the hub's shell, each pinion's to take the shaft's end.  The hub's gear
// and pinion are made where the bracket's stand, and turned to the hub.
gear_web := { q | q not in pair.gear_member.back
                  q not in std.sets.Cylinder(bb_line, r: sleeve_r)
                  pair.gear_member.foot distance(in: (web.x, web.y), along: bb_line) q }
hub_web := { q | q not in pair.gear_member.back
                 q not in std.sets.Cylinder(bb_line, r: hub_r)
                 pair.gear_member.foot distance(in: (web.x, web.y), along: bb_line) q }
pinion_web := { q | q not in pair.pinion_member.back
                    q not in std.sets.Cylinder(pair.pinion_member.ax, r: shaft_od / 2)
                    pair.pinion_member.foot distance(in: (web.x, web.y), along: pair.pinion_member.ax) q }
construction gear_web_solid := solid(gear_web)
construction hub_web_solid := solid(hub_web)
construction pinion_web_solid := solid(pinion_web)
bb_gear := solid(pair.gear_member.body)
gear_web_solid union bb_gear
construction hub_gear_here := solid(pair.gear_member.body)
hub_web_solid union hub_gear_here
construction pinion_part := solid(pair.pinion_member.body)
pinion_web_solid union pinion_part
bb_pinion := solid(pinion_part)
hub_gear := hub_gear_here at(under: flip) 180deg
hub_pinion := pinion_part at(under: flip) 180deg

// -- the planetary -------------------------------------------------------------------------------
// Its mid-plane `ring_y` out on the non-drive side, its axis the bracket's, toward the drive side.
bb_mid := point
bb_mid coincident std.y
std.origin distance(ring_y, along: std.y) bb_mid
box := planetary.Planetary()
box seats(turn: 0deg, up: drive_side, east: std.x) bb_mid

// A point `d` along the line `ax` from `from`: where a crank stands on the spindle.
component At(ax: line, from: point, d: Length) {
  p := point
  p coincident ax
  from distance(d, along: ax) p
}

// -- the shaft ---------------------------------------------------------------------------------
// A tube from pinion to pinion, its ends in the members' webs, grooved for the rings that hold its
// bearings.
shaft := solid({ q | q in std.sets.Cylinder(pair.pinion_member.ax, r: shaft_od / 2)
                     q not in std.sets.Cylinder(pair.pinion_member.ax, r: shaft_id / 2)
                     pair.pinion_member.foot distance(>= 0mm, along: pair.pinion_member.ax) q
                     hub_foot distance(<= 0mm, along: pair.pinion_member.ax) q })

// -- the crank spindle and the cranks -------------------------------------------------------------
spindle := solid({ q | q in std.sets.Cylinder(bb_line, r: spindle_d / 2)
                       q not in std.sets.Cylinder(bb_line, r: spindle_bore / 2)
                       std.origin distance(in: (-crank_face - 11mm, crank_face + 11mm), along: bb_line) q })
// One crank, placed twice on the spindle's ends: the right one forward, the left turned half a turn
// about the spindle, behind.  Each stands on the spindle's axis at its outer face.
right_end := At(bb_line, std.origin, d: -crank_face - 11mm)
left_end := At(bb_line, std.origin, d: crank_face + 11mm)
down := axis
down parallel(sense: opposite) std.z
std.origin coincident down
right_crank := Crank(throw: crank_throw, boss: 17mm, eye: 10mm, bore: spindle_d)
right_crank seats(turn: 0deg, up: std.z, east: std.x) right_end.p
left_crank := Crank(throw: crank_throw, boss: 17mm, eye: 10mm, bore: spindle_d)
left_crank seats(turn: 180deg, up: down, east: std.x) left_end.p

// -- the frame -------------------------------------------------------------------------------------
// The tubes, stays and the non-drive dropout (`frame.sv`), and about the drive: the bracket's shell,
// the pinion's housing where it leaves the shell, the chainstay round the shaft, the drive-side
// dropout's gear case round the hub's gear and its pinion's housing, the hub's turned end for end.
// Each is a cylinder about the axis it houses, a revolution built exactly; the frame is their union
// less their bores, then the webs that seat the bearings.
tubes := frame.Frame(std.origin, axle_c)
case_outer_set := { q | q in std.sets.Cylinder(bb_line, r: shell_od / 2)
    std.origin distance(in: (case_right, case_left), along: bb_line) q }
construction case_outer := solid(case_outer_set)
case_bore_set := { q | q in std.sets.Cylinder(bb_line, r: shell_id / 2)
    std.origin distance(in: (case_right + end_wall, case_left + 1mm), along: bb_line) q }
construction case_bore := solid(case_bore_set)
boss_set := { q | q in std.sets.Cylinder(pair.pinion_member.ax, r: boss_r)
    pair.pinion_member.foot distance(in: (-boss_ahead, boss_behind), along: pair.pinion_member.ax) q }
construction boss := solid(boss_set)
boss_bore_set := { q | q in std.sets.Cylinder(pair.pinion_member.ax, r: boss_r - wall)
    pair.pinion_member.foot distance(in: (-boss_ahead - 1mm, boss_behind - wall), along: pair.pinion_member.ax) q }
construction boss_bore := solid(boss_bore_set)
stay_set := { q | q in std.sets.Cylinder(pair.pinion_member.ax, r: tube_od / 2)
    pair.pinion_member.foot distance(>= boss_behind - 1mm, along: pair.pinion_member.ax) q
    hub_foot distance(<= 1mm - boss_behind, along: pair.pinion_member.ax) q }
construction stay := solid(stay_set)
stay_bore_set := { q | q in std.sets.Cylinder(pair.pinion_member.ax, r: tube_id / 2)
    pair.pinion_member.foot distance(>= 0mm, along: pair.pinion_member.ax) q
    hub_foot distance(<= 0mm, along: pair.pinion_member.ax) q }
construction stay_bore := solid(stay_bore_set)
dropout_set := { q | q in std.sets.Cylinder(axle_line, r: rear_case_r)
    axle_c distance(in: (rear_case.x, rear_case.y), along: axle_line) q }
construction dropout := solid(dropout_set)
dropout_bore_set := { q | q in std.sets.Cylinder(axle_line, r: rear_case_r - wall)
    axle_c distance(in: (rear_case.x + wall, rear_case.y - wall), along: axle_line) q }
construction dropout_bore := solid(dropout_bore_set)
// the hub's shell runs out through the case's inner wall; the axle through its outer wall
hub_way_set := { q | q in std.sets.Cylinder(axle_line, r: hub_r + 1mm)
    axle_c distance(in: (rear_case.x + wall - 1mm, rear_case.y + 1mm), along: axle_line) q }
construction hub_way := solid(hub_way_set)
axle_way_set := { q | q in std.sets.Cylinder(axle_line, r: axle_d / 2)
    axle_c distance(in: (rear_case.x - 1mm, rear_case.x + wall + 1mm), along: axle_line) q }
construction axle_way := solid(axle_way_set)
construction hub_boss := boss at(under: flip) 180deg
construction hub_boss_bore := boss_bore at(under: flip) 180deg

shell := solid(case_outer)
tubes.body union shell
boss union shell
stay union shell
dropout union shell
hub_boss union shell
case_bore cut shell
boss_bore cut shell
stay_bore cut shell
dropout_bore cut shell
hub_way cut shell
axle_way cut shell
hub_boss_bore cut shell

// The webs that seat the bearings, added inside the bored shell: the 6805 round the sun's sleeve,
// and each pinion's 6903, the hub's turned from the bracket's.
diaphragm_set := { q | q in std.sets.Cylinder(bb_line, r: shell_id / 2)
    std.origin distance(in: (diaphragm.x, diaphragm.y), along: bb_line) q }
construction diaphragm_ring := solid(diaphragm_set)
pinion_seat_set := { q | q in std.sets.Cylinder(pair.pinion_member.ax, r: boss_r - wall + 0.5mm)
    pair.pinion_member.foot distance(in: (pinion_bearing.x, pinion_bearing.y), along: pair.pinion_member.ax) q }
construction pinion_seat := solid(pinion_seat_set)
construction hub_pinion_seat := pinion_seat at(under: flip) 180deg
frame := solid(shell)
diaphragm_ring union frame
pinion_seat union frame
hub_pinion_seat union frame
// Where the chainstay leaves each pinion's housing, the frame is rounded.  (Where the housing
// leaves the shell it is not: README, what solventc lacks.)
frame.shell.stay round(r: 3mm) frame.shell.boss
frame.shell.stay round(r: 3mm) frame.shell.hub_boss

// The left bearing cup, pressed into the shell's open end: a sleeve seating the left 6804, and a
// flange against the shell's face.
cup_set := { q | q in std.sets.Cylinder(bb_line, r: shell_id / 2)
    std.origin distance(in: (cup.x, case_left), along: bb_line) q }
cup_flange_set := { q | q in std.sets.Cylinder(bb_line, r: shell_od / 2)
    q not in std.sets.Cylinder(bb_line, r: std.bearings.b6804.D / 2 - 1.5mm)
    std.origin distance(in: (case_left, cup.y), along: bb_line) q }
bearing_cup := solid(cup_set)
construction cup_flange := solid(cup_flange_set)
cup_flange union bearing_cup

// -- the hub and the thru-axle ---------------------------------------------------------------------
// A shell on the axle, its drive-side end carrying the hub's gear in the dropout's case, two spoke
// flanges drilled for `spokes` spokes each, a 6901 seated in each end and held by a ring.
hub := solid({ q | q in std.sets.Cylinder(axle_line, r: hub_r)
                   q not in std.sets.Cylinder(axle_line, r: hub_bore)
                   axle_c distance(in: (hub_span.x, hub_span.y), along: axle_line) q })
flange_right_set := { q | q in std.sets.Cylinder(axle_line, r: flange_r)
    q not in std.sets.Cylinder(axle_line, r: hub_bore)
    axle_c distance(in: (flanges.x, flanges.x + 3mm), along: axle_line) q }
construction flange_right := solid(flange_right_set)
flange_left_set := { q | q in std.sets.Cylinder(axle_line, r: flange_r)
    q not in std.sets.Cylinder(axle_line, r: hub_bore)
    axle_c distance(in: (flanges.y, flanges.y + 3mm), along: axle_line) q }
construction flange_left := solid(flange_left_set)
flange_right union hub
flange_left union hub
right_hub_seat_set := { q | q in std.sets.Cylinder(axle_line, r: std.bearings.b6901.D / 2)
    axle_c distance(in: (hub_span.x - 1mm, hub_bearings.x), along: axle_line) q }
construction right_hub_seat := solid(right_hub_seat_set)
left_hub_seat_set := { q | q in std.sets.Cylinder(axle_line, r: std.bearings.b6901.D / 2)
    axle_c distance(in: (hub_bearings.y, hub_span.y + 1mm), along: axle_line) q }
construction left_hub_seat := solid(left_hub_seat_set)
right_hub_seat cut hub
left_hub_seat cut hub
in std.front {
  ring spokes about axle_c {
    sc := point
    axle_c distance(flange_r - 4mm) sc
    spoke_hole := circle(center: sc)
    radius(1.3mm) spoke_hole
    construction drill := solid(std.solids.Through(face(spoke_hole)))
  }
  axle_c distance(0mm, along: v) sc[0]
  axle_c distance(>= 0mm, along: u) sc[0]
}
repeat spokes as i {
  drill[i] cut hub
}
thru_axle := solid({ q | q in std.sets.Cylinder(axle_line, r: axle_d / 2)
                         axle_c distance(in: (-old / 2 - 1mm, old / 2 + 3mm), along: axle_line) q })
axle_head_set := { q | q in std.sets.Cylinder(axle_line, r: 9mm)
    axle_c distance(in: (old / 2, old / 2 + 3mm), along: axle_line) q }
construction axle_head := solid(axle_head_set)
axle_head union thru_axle

// -- bearings and retaining rings ------------------------------------------------------------------
// Each bearing is seated in a station of the body that carries it, the station's axis running the
// way the bearing went in, and held by a ring against its trailing side (`std.bearings.OnShaft`,
// `InBore`): the room it takes is cut from the body, the ring's groove from what it is fitted to.
// A station is where the bearing's mid-plane stands on its axis.
spindle_j := {host: spindle}
shaft_j := {host: shaft}
hub_j := {host: hub}
rear := axis
rear parallel(sense: opposite) pair.pinion_member.ax
pair.pinion_member.foot coincident rear

right_st := std.ports.Station(bb_line, std.origin, d: case_right + end_wall / 2, toward: drive_side,
  east: std.x, host: shell)
right_bb := std.bearings.OnShaft(std.bearings.b6804, ring: std.retaining.din471_20,
  seat: right_st.seat, on: spindle_j)
left_st := std.ports.Station(bb_line, std.origin, d: cup.x + 1mm + std.bearings.b6804.B / 2,
  toward: std.y, east: std.x, host: bearing_cup)
left_bb := std.bearings.OnShaft(std.bearings.b6804, ring: std.retaining.din471_20,
  seat: left_st.seat, on: spindle_j)
sleeve_st := std.ports.Station(bb_line, std.origin, d: (diaphragm.x + diaphragm.y) / 2,
  toward: drive_side, east: std.x, host: diaphragm_ring)
sleeve_bb := std.bearings.OnShaft(std.bearings.b6805, ring: std.retaining.din471_25,
  seat: sleeve_st.seat, on: box.sleeve)
bb_pinion_st := std.ports.Station(pair.pinion_member.ax, pair.pinion_member.foot,
  d: (pinion_bearing.x + pinion_bearing.y) / 2, toward: rear, east: flip, host: pinion_seat)
bb_pinion_bb := std.bearings.OnShaft(std.bearings.b6903, ring: std.retaining.din471_17,
  seat: bb_pinion_st.seat, on: shaft_j)
hub_pinion_st := std.ports.Station(pair.pinion_member.ax, hub_foot,
  d: -(pinion_bearing.x + pinion_bearing.y) / 2, toward: pair.pinion_member.ax, east: flip,
  host: hub_pinion_seat)
hub_pinion_bb := std.bearings.OnShaft(std.bearings.b6903, ring: std.retaining.din471_17,
  seat: hub_pinion_st.seat, on: shaft_j)
right_hub_st := std.ports.Station(axle_line, axle_c, d: hub_bearings.x, toward: axle, east: std.x,
  host: hub)
right_hub_bb := std.bearings.InBore(std.bearings.b6901, ring: std.retaining.din472_24,
  seat: right_hub_st.seat, on: hub_j)
left_hub_st := std.ports.Station(axle_line, axle_c, d: hub_bearings.y, toward: drive_side,
  east: std.x, host: hub)
left_hub_bb := std.bearings.InBore(std.bearings.b6901, ring: std.retaining.din472_24,
  seat: left_hub_st.seat, on: hub_j)

// -- claims ----------------------------------------------------------------------------------------
// The shaft runs clear inside the chainstay, and every turning part clear of the frame.
claim shaft clear(3mm) frame
claim box.carrier clear(0.5mm) frame
claim bb_gear clear(0.5mm) frame
claim hub clear(0.5mm) frame
claim hub_gear clear(0.5mm) frame
// The pinions clear what turns beside them: the sun's sleeve, the spindle's right bearing, and
// the hub.
claim box.turns.sun clear(0.5mm) bb_pinion
claim right_bb.bearing.outer_ring clear(0.5mm) bb_pinion
claim hub clear(0.5mm) hub_pinion
// The cranks clear the frame.
claim right_crank.body clear(2mm) frame
claim left_crank.body clear(2mm) frame

// -- the parts -------------------------------------------------------------------------------------
// A crank arm as it is drawn face on, in its frame's front: a boss about the spindle and an eye
// `throw` along `std.x`, joined by the two lines tangent to both, bored for the spindle and the
// pedal's 9/16" thread; 11 mm thick, in front of the front.
component Crank(throw: Length, boss: Length, eye: Length, bore: Length) {
  in std.front {
    c := point
    c right_of(d: throw) std.origin
    a := point hint((0mm, boss))
    b := point hint((0mm, -boss))
    d := point hint((throw, eye))
    e := point hint((throw, -eye))
    lower := line(b, e)
    small := arc(center: c, start: e, end: d)
    upper := line(d, a)
    big := arc(center: std.origin, start: a, end: b)
    radius(boss) big
    radius(eye) small
    // the outer tangents: each tangent point on its own side of the line of centres
    ccw(std.origin, c, a)
    ccw(std.origin, c, d)
    cw(std.origin, c, b)
    cw(std.origin, c, e)
    lower tangent(at: p1) big
    lower tangent(at: p2) small
    upper tangent(at: p1) small
    upper tangent(at: p2) big
    spindle_hole := circle(center: std.origin)
    radius(bore / 2) spindle_hole
    pedal_hole := circle(center: c)
    radius(0.5625in / 2) pedal_hole
  }
  body := solid(std.solids.Prism(face(lower, small, upper, big, holes: spindle_hole, pedal_hole),
    from: -11mm, to: 0mm))
}
