// A chainless bicycle drive: a shaft sealed inside the drive-side chainstay, a step-up gearbox
// inside an oversized bottom bracket and a step-down gearbox inside the rear dropout.  See
// README.md for the design and what solventc could not yet draw of it, and `dims.sv` for the
// numbers and their checks.
//
// The world is the bicycle's: `std.x` forward, `std.y` to the rider's left, `std.z` up, the
// bottom bracket's centre at the origin.  The drive side is the right, toward −y.
unit mm
use std
use std.sets
use std.solids
use std.solids (cut, union)
use std.rounds (round)
use std.place (seats, right_of)
use spiral_bevel.hypoid (HypoidPair)
use planetary
use dims (cs_length, bb_drop, shaft_y, ring_y, scale, pinion_teeth, gear_teeth, hyp_module,
          hyp_offset, hyp_backlash, hyp_tip_relief, hyp_end_relief, shell_od, shell_id,
          case_left, case_right, end_wall, bearing_od, sleeve_bearing_od, diaphragm, boss_r,
          boss_ahead, boss_behind, wall, web, sleeve_r, hub_r, hub_bore, hub_span, flange_r,
          flanges, spokes, rear_case_r, rear_case, shaft_od, shaft_id, tube_od, tube_id,
          spindle_d, spindle_bore, crank_throw)

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
// the two axes as drawn lines, for ordinates along them
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

// -- the hypoid pairs ----------------------------------------------------------------------------
// One pair, laid out and cut once, standing twice.  At the bottom bracket its gear rides the sun's
// sleeve, its axis the bracket's, and drives the pinion, whose axis lies in the shaft's plane
// `shaft_y` out on the drive side and runs back along the line of centres.  The pair's offset puts
// that axis `hyp_offset` below the line of centres, along its normal.
//
// At the hub the same pair stands turned end for end: half a turn about `flip`, the normal to the
// line of centres through its middle, in the shaft's plane.  The turn takes the bracket's axis to
// the axle's and the shaft's axis to itself, run the other way, so the hub's gear turns on the
// axle and the shaft's two pinions share one axis by construction.  The shaft's pinion drives the
// hub's gear.
flip_c := point
mid distance(shaft_y, along: std.y) flip_c
mid distance(0mm, along: std.x) flip_c
mid distance(0mm, along: std.z) flip_c
flip := axis
flip perpendicular std.y
flip perpendicular centres
flip_c coincident flip

// The pair is laid out in its own pitch-plane frame, `place`, which stands where the four claims
// below hold: its gear's axis the bracket's, its pinion's square to `flip`, in the shaft's plane.
// Those are what should place it; stated as relations, they leave the pair's layout to converge
// from an attitude it does not (README: what solventc lacks), so the frame is held at the
// attitude they give, read from the pair's own solve — the pinion's axis runs (38.628, -18.108,
// 19.314) from its apex in the pair's frame — and claimed.
place := frame
fix(dir == (-0.870151820509, 0.447213595500, -0.206968135870)) place.u
fix(dir == (0.231397410491, 0, -0.972859310701)) place.v
fix(origin == (0mm, -34.184481591980mm, 0mm)) place
in place {
  pair := HypoidPair(pinion_teeth: pinion_teeth, gear_teeth: gear_teeth,
    mean_module: hyp_module, shaft_angle: 90deg, offset: hyp_offset, spiral_angle: 25deg,
    backlash: hyp_backlash, tip_relief: hyp_tip_relief, end_relief: hyp_end_relief,
    pressure_shift: hint(10deg), cutter_radius: hint(80mm * scale))
}
claim pair.gear_member.foot coincident std.y
claim pair.gear_member.ax parallel(sense: same) std.y
claim pair.pinion_member.ax perpendicular flip
claim std.origin distance(shaft_y, along: std.y) pair.pinion_member.foot

// The pinion's mean point at the hub.
hub_foot := pair.pinion_member.foot at(under: flip) 180deg
claim hub_foot coincident pair.pinion_member.ax

// Each member is made one part with a web filling its core: the points deeper inside its pitch
// cone than its blank's back, in a band along its axis about the foot of its mean point (as
// `hypoid_drive/drive.sv` hubs its members).  The gear's web is bored to sit on the sun's sleeve,
// the pinion's to take the shaft's end.
gear_web := { q | q not in pair.gear_member.back
                  q not in std.sets.Cylinder(bb_line, r: sleeve_r)
                  pair.gear_member.foot distance(in: (web.x, web.y), along: bb_line) q }
construction gear_web_solid := solid(gear_web)
pinion_web := { q | q not in pair.pinion_member.back
                    q not in std.sets.Cylinder(pair.pinion_member.ax, r: shaft_od / 2)
                    pair.pinion_member.foot distance(in: (web.x, web.y), along: pair.pinion_member.ax) q }
construction pinion_web_solid := solid(pinion_web)
construction gear_part := solid(pair.gear_member.body)
gear_web_solid union gear_part
construction pinion_part := solid(pair.pinion_member.body)
pinion_web_solid union pinion_part
bb_gear := solid(gear_part)
bb_pinion := solid(pinion_part)
hub_gear := gear_part at(under: flip) 180deg
hub_pinion := pinion_part at(under: flip) 180deg

// -- the planetary -------------------------------------------------------------------------------
// Its mid-plane `ring_y` out on the non-drive side, its axis the bracket's, toward the drive side.
bb_mid := point
bb_mid coincident std.y
std.origin distance(ring_y, along: std.y) bb_mid
box := planetary.Planetary()
box seats(turn: 0deg, up: drive_side, east: std.x) bb_mid

// -- the shaft ---------------------------------------------------------------------------------
// A tube from pinion to pinion, its ends in the members' webs.
shaft := solid({ q | q in std.sets.Cylinder(pair.pinion_member.ax, r: shaft_od / 2)
                     q not in std.sets.Cylinder(pair.pinion_member.ax, r: shaft_id / 2)
                     pair.pinion_member.foot distance(>= 0mm, along: pair.pinion_member.ax) q
                     hub_foot distance(<= 0mm, along: pair.pinion_member.ax) q })

// -- the crank spindle -----------------------------------------------------------------------------
spindle := solid({ q | q in std.sets.Cylinder(bb_line, r: spindle_d / 2)
                       q not in std.sets.Cylinder(bb_line, r: spindle_bore / 2)
                       std.origin distance(in: (case_right - 14mm, case_left + 18mm), along: bb_line) q })

// -- the cranks -----------------------------------------------------------------------------------
// One crank, placed twice on the spindle's ends: the right one forward, the left turned half a turn
// about the spindle, behind.  Each stands on the spindle's axis at its outer face.
right_end := point
right_end coincident std.y
std.origin distance(case_right - 14mm, along: std.y) right_end
left_end := point
left_end coincident std.y
std.origin distance(case_left + 7mm, along: std.y) left_end
down := axis
down parallel(sense: opposite) std.z
std.origin coincident down
right_crank := Crank(throw: crank_throw, boss: 17mm, eye: 10mm, bore: spindle_d)
right_crank seats(turn: 0deg, up: std.z, east: std.x) right_end
left_crank := Crank(throw: crank_throw, boss: 17mm, eye: 10mm, bore: spindle_d)
left_crank seats(turn: 180deg, up: down, east: std.x) left_end

// -- the frame: the bracket's case, the pinion's housing, the chainstay and the dropout's case ------
// Each is a cylinder about the axis it houses, so each is a revolution built exactly; the frame is
// their union less their bores.
case_outer_set := { q | q in std.sets.Cylinder(bb_line, r: shell_od / 2)
    std.origin distance(in: (case_right, case_left), along: bb_line) q }
construction case_outer := solid(case_outer_set)
case_bore_set := { q | q in std.sets.Cylinder(bb_line, r: shell_id / 2)
    std.origin distance(in: (case_right + end_wall, case_left + 1mm), along: bb_line) q }
construction case_bore := solid(case_bore_set)
right_seat_set := { q | q in std.sets.Cylinder(bb_line, r: bearing_od / 2)
    std.origin distance(in: (case_right - 1mm, case_right + end_wall + 1mm), along: bb_line) q }
construction right_seat := solid(right_seat_set)
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
    std.origin distance(in: (rear_case.x, rear_case.y), along: bb_line) q }
construction dropout := solid(dropout_set)
dropout_bore_set := { q | q in std.sets.Cylinder(axle_line, r: rear_case_r - wall)
    std.origin distance(in: (rear_case.x + wall, rear_case.y - wall), along: bb_line) q }
construction dropout_bore := solid(dropout_bore_set)
dropout_seat_set := { q | q in std.sets.Cylinder(axle_line, r: hub_r + 1mm)
    std.origin distance(in: (rear_case.x - 1mm, rear_case.y + 1mm), along: bb_line) q }
construction dropout_seat := solid(dropout_seat_set)
construction hub_boss := boss at(under: flip) 180deg
construction hub_boss_bore := boss_bore at(under: flip) 180deg

shell := solid(case_outer)
boss union shell
stay union shell
dropout union shell
hub_boss union shell
case_bore cut shell
right_seat cut shell
boss_bore cut shell
stay_bore cut shell
dropout_bore cut shell
dropout_seat cut shell
hub_boss_bore cut shell

// The web that holds the sleeve's bearing, added inside the bored shell.
diaphragm_ring_set := { q | q in std.sets.Cylinder(bb_line, r: shell_id / 2)
    q not in std.sets.Cylinder(bb_line, r: sleeve_bearing_od / 2)
    std.origin distance(in: (diaphragm.x, diaphragm.y), along: bb_line) q }
construction diaphragm_ring := solid(diaphragm_ring_set)
frame := solid(shell)
diaphragm_ring union frame
// Where the chainstay leaves each pinion's housing, the frame is rounded.  (Where the housing
// leaves the bracket's case it is not: README, what solventc lacks.)
frame.shell.stay round(r: 3mm) frame.shell.boss
frame.shell.stay round(r: 3mm) frame.shell.hub_boss

// -- the hub -------------------------------------------------------------------------------------
// A shell on the axle, its drive-side end carrying the hub's gear in the dropout's case, and two
// spoke flanges, each drilled for `spokes` spokes.
hub := solid({ q | q in std.sets.Cylinder(axle_line, r: hub_r)
                   q not in std.sets.Cylinder(axle_line, r: hub_bore)
                   std.origin distance(in: (hub_span.x, hub_span.y), along: bb_line) q })
flange_right_set := { q | q in std.sets.Cylinder(axle_line, r: flange_r)
    q not in std.sets.Cylinder(axle_line, r: hub_bore)
    std.origin distance(in: (flanges.x - 3mm, flanges.x), along: bb_line) q }
construction flange_right := solid(flange_right_set)
flange_left_set := { q | q in std.sets.Cylinder(axle_line, r: flange_r)
    q not in std.sets.Cylinder(axle_line, r: hub_bore)
    std.origin distance(in: (flanges.y, flanges.y + 3mm), along: bb_line) q }
construction flange_left := solid(flange_left_set)
flange_right union hub
flange_left union hub
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

// -- claims ----------------------------------------------------------------------------------------
// The shaft runs clear inside the chainstay, and every turning part clear of the frame.
claim shaft clear(3mm) frame
claim box.carrier clear(0.5mm) frame
claim bb_gear clear(0.5mm) frame
claim hub clear(0.5mm) frame
claim hub_gear clear(0.5mm) frame
// The pinions clear what turns beside them: the sun's sleeve, and the hub.
claim box.turns.sun clear(0.5mm) bb_pinion
claim hub clear(0.5mm) hub_pinion

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
