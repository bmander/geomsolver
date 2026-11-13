// The shaft drive's numbers, stated once: a chainless bicycle transmission whose shaft runs
// sealed inside the drive-side chainstay, its step-up gearbox inside an oversized bottom bracket
// and its step-down gearbox inside the rear dropout.
//
// **Why the shaft spins fast.**  A drive shaft is a torsion spring between the cranks and the
// wheel: under a pedal stroke it winds up, and gives the energy back late and out of phase.  Its
// wind-up at a given torque is `T L / (G J)`, and the torque it carries is the crank's divided by
// how much faster than the crank it turns.  So the bottom bracket steps the speed up by `k`
// before the shaft and the hub steps it down again: the shaft carries `1/k` of the crank torque,
// winds up `1/k` as far, and that angle is seen at the crank `1/k` again.  The crank feels a
// spring `k²` stiffer than the same shaft driven at crank speed (`wind_gain` below, 36).
//
// **The train.**
// 1. At the bottom bracket, a helical planetary: the crank spindle drives the carrier, the ring
//    is held in the shell, the sun is the output, `1 + Zr / Zs` = 3 times as fast.
// 2. On the sun's sleeve, a hypoid gear (48) drives a hypoid pinion (24) on the shaft: twice
//    again.  The shaft turns 6 times as fast as the cranks.
// 3. In the rear dropout, the same hypoid pair the other way round: the shaft's pinion (24)
//    drives the hub's gear (48), half as fast.  The wheel turns 3 times as fast as the cranks,
//    a 48 × 16 single speed.
//
// Every figure that carries a unit says it, so the arithmetic is checked; claims over the figures
// are judged before the solve.
unit mm

// -- the rider and the wheel --------------------------------------------------------------------
cadence := 90rpm                       // a brisk cruising cadence
crank_cruise := 50Nm                   // a seated cruising effort (≈ 140 W at 90 rpm with the two legs alternating)
crank_peak := 180Nm                    // a standing sprint: 1060 N on a 170 mm crank
roll_out := 2.168m                     // a 700 × 35C tyre's rolling circumference

// -- the planetary at the bottom bracket ---------------------------------------------------------
Zs := 30                               // sun
Zp := 15                               // planets
Zr := 60                               // ring: Zs + 2 Zp
planets := 3                           // (Zs + Zr) / planets = 30: they assemble a third of a turn apart
mn := 1mm                              // normal module
alpha := 20deg                         // normal pressure angle
beta := 20deg                          // helix angle: sun right-handed, planets and ring left-handed
pw := 14mm                             // face width
ha := 0.8mm                            // addendum: a stub tooth, so the rack's 1 mm cut leaves 0.2 mm of tip clearance
hf := 1mm                              // dedendum (the rack's tip)
backlash := 0.04mm                     // normal backlash at each mesh
edge := 0.3mm                          // the chamfer that breaks every tooth's end

mt := mn / cos(beta)                   // transverse module, 1.064 mm
alpha_t := atan(tan(alpha) / cos(beta))  // transverse pressure angle, 21.17°
rs := Zs * mt / 2                      // pitch radii: 15.96 mm
rpl := Zp * mt / 2                     //              7.98 mm
rr := Zr * mt / 2                      //              31.93 mm
a := rs + rpl                          // the planets' centre distance, 23.94 mm
planet_tip := rpl + ha                 // 8.78 mm

step_planetary := 1 + Zr / Zs          // carrier in, sun out, ring held: 3

// -- the spiral bevel pairs ---------------------------------------------------------------------
// The pair of `spiral_bevel/hypoid.sv`, 24 and 48 teeth, with no offset: a spiral bevel pair, its
// axes meeting, which wastes none of its work in the lengthwise sliding a hypoid's offset brings.
// Its every length is taken at `scale`: a mean module of 1.25 mm, the gear 67 mm across.  Its
// shafts meet at `bevel_angle`, not square: the drive-side chainstay splays `splay` outward toward
// the hub, as a frame's chainstays do, and the shaft runs along it.
scale := 0.625
pinion_teeth := 24
gear_teeth := 48
splay := 2deg                          // the shaft's lean outward, toward the hub, seen from above
bevel_angle := 90deg + splay           // between each gear's axis and its pinion's
bevel_module := 2mm * scale
bevel_backlash := 0.05mm * scale
bevel_tip_relief := 0.2mm * scale
bevel_end_relief := 0.2mm * scale
bevel_cutter_radius := 75.6122mm * scale  // the face mill: hypoid.sv's, chosen, not found
step_bevel := gear_teeth / pinion_teeth  // 2, up at the bottom bracket and down at the hub

// -- what the train does ---------------------------------------------------------------------------
k := step_planetary * step_bevel       // the shaft turns 6 times as fast as the cranks
overall := k / step_bevel              // the wheel turns 3 times as fast: a 48 × 16 equivalent
shaft_speed := cadence * k             // 540 rpm
wheel_speed := cadence * overall       // 270 rpm
road_speed := wheel_speed / turn * roll_out   // 9.76 m/s: 35.1 km/h at 90 rpm
shaft_cruise := crank_cruise / k       // 8.3 N·m in the shaft cruising
shaft_peak := crank_peak / k           // 30 N·m in the shaft sprinting

// The planetary's shares of the crank's torque: the sun carries Zs / (Zs + Zr) of it, the ring
// reacts the rest into the shell.  Each of the three planets takes a third of the sun's
// tangential force.
sun_peak := crank_peak * Zs / (Zs + Zr)          // 60 N·m
mesh_force := sun_peak / rs / planets            // 1250 N at the sun's pitch circle, peak
// Lewis's bending stress at the planet's root, its form factor at the planet's virtual count
// (Zp / cos³β ≈ 18): an estimate, to be checked to ISO 6336 before tooling.
lewis_y := 0.31
sigma_planet := mesh_force / (pw * mn * lewis_y)  // ≈ 290 MPa at the sprint peak
case_steel := 450MPa                   // allowable bending, case-hardened 18CrNiMo7-6, reversed

// -- the shaft and its tube --------------------------------------------------------------------------
shaft_od := 17mm                       // a cold-drawn 25CrMo4 tube, 17 × 1.5: it runs in a 6903 at each end
shaft_id := 14mm
shaft_G := 80000MPa                    // shear modulus of steel
cs_length := 430mm                     // the chainstay: bottom-bracket centre to rear axle
bb_drop := 70mm                        // the axle stands this far above the bottom bracket
pinion_y := -32mm                      // the bracket's pinion's mean point, out on the drive side
J := pi * (shaft_od^4 - shaft_id^4) / 32           // 4430 mm⁴
shaft_tau := shaft_peak * (shaft_od / 2) / J       // 58 MPa at the sprint peak
// The wind-up the crank feels at the sprint peak, the shaft driven at k times crank speed, and
// the same shaft driven 1:1 at the crank's speed and torque.
wind := shaft_peak * cs_length / (shaft_G * J) * 1rad / k        // 0.35° at the crank
wind_direct := crank_peak * cs_length / (shaft_G * J) * 1rad     // 12.5°
wind_gain := wind_direct / wind                                  // k² = 36
tube_od := 30mm                        // the drive-side chainstay round the shaft, 30 × 2
tube_id := 26mm

// -- the frame: a 56 cm road frame in 7005 aluminium ------------------------------------------------
seat_angle := 73.5deg
seat_length := 540mm                   // the bracket's centre to the seat tube's top
stack := 575mm                         // the head tube's top over the bracket
reach := 385mm                         //                    and ahead of it
head_angle := 72deg
head_length := 160mm
cluster := 30mm                        // the top tube and seat stays meet the seat tube this far below its top
stay_spread := 14mm                    // the seat stays stand this far either side of the seat tube there
old := 142mm                           // over the dropouts: a 12 mm thru-axle
dropout_t := 7mm
dropout_r := 32mm                      // the non-drive dropout's plate about the axle
stay_short := 26mm                     // the stays end this far short of the axle's centre, on the dropouts
ns_stay_y := 38mm                      // the non-drive chainstay's foot on the shell
head_od := 50mm
head_id := 44mm
down_od := 44mm
down_id := 41mm
seat_od := 34.9mm
seat_id := 31.6mm
top_od := 35mm
top_id := 32mm
stay_od := 16mm
stay_id := 13mm
ns_stay_od := 22mm
ns_stay_id := 19mm

// -- the bottom bracket: a 100 mm shell, centred on the frame -----------------------------------------
shell_od := 82mm                       // the gearbox's case
shell_id := 72mm                       // the ring gear is pressed into it
case_left := 50mm                      // its faces, either side of the mid-plane: the left one open for
case_right := -50mm                    // the bearing cup, the right one closed round the right bearing
end_wall := 7mm                        // the right face's wall, a 6804's width
bearing_od := 32mm                     // the spindle's 6804s, 20 × 32 × 7
sleeve_bearing_od := 37mm              // the sun's sleeve's 6805, 25 × 37 × 7
diaphragm := (-9.5mm, -2.5mm)          // the web in the shell that holds the 6805, between the
                                       // planetary and the bevel gear
cup := (42mm, 52mm)                    // the left bearing cup, pressed into the shell's open end
boss_r := 28mm                         // the pinion's housing: a cylinder about the shaft's axis,
                                       // round a bevel pinion 46.8 mm across
boss_ahead := 12mm                     // from this far ahead of the pinion's mean point: short of the
                                       // hub, whose axle the shaft's axis meets
boss_behind := 24mm                    // to this far behind it, past the pinion's bearing
wall := 2.5mm
web := (-1.5mm, 4mm)                   // a bevel member's web: its band along the axis from the foot
                                       // of its mean point, filling its core out to the blank's back
pinion_bearing := (10mm, 17mm)         // each pinion's 6903 (17 × 30 × 7), behind its mean point
crank_throw := 170mm
crank_face := 62mm                     // each crank's inner face, either side of the mid-plane:
                                       // clear of the pinion's housing
ring_od := 72mm
spindle_d := 20mm                      // the crank spindle, hollow 20 × 14, 4340 at 40 HRC
spindle_bore := 14mm
sun_bore := 21mm                       // the sun runs on the spindle on a 0.5 mm-wall bronze bush
sleeve_r := 12.5mm                     // the sun's sleeve, 25 across: it runs in a 6805 (25 × 37 × 7)
sleeve_to := 34mm                      // and reaches past the bevel gear it carries
sleeve_groove_z := ring_y - diaphragm.y - 0.8mm  // its 6805's ring's groove, along the planetary's axis
spindle_tau := crank_peak * (spindle_d / 2) / (pi * (spindle_d^4 - spindle_bore^4) / 32)  // 151 MPa
planet_bore := 7mm                     // the planets run on 5 mm pins in 1 mm-wall bushes
pin_d := 5mm
ring_y := 12mm                         // the planetary's mid-plane, out toward the non-drive side
plate_t := 3mm                         // the carrier's plates
plate_r := 29mm
gap := 0.5mm                           // between a planet's face and a plate
bridge_w := 16mm                       // the carrier's bridges between the planets, across
bridge_r := 18.5mm                     // their inside, clear of the sun's tips

// -- the hub ---------------------------------------------------------------------------------------
hub_r := 15mm                          // the hub's shell, 30 across over a 6901 at each end
hub_bore := 8mm                        // between the bearings
hub_seat := 12mm                       // the 6901s' seats, 24 across
hub_span := (-67mm, 63mm)
hub_bearings := (-61mm, 58mm)          // each 6901's mid-plane
flange_r := 28mm
flanges := (-14mm, 33mm)               // the spoke flanges' inner faces, 3 mm thick: the right one clear
                                       // of the hub's pinion housing
spokes := 16                           // a side, 2.6 mm holes on a 48 mm circle
rear_case_r := 40mm                    // the drive-side dropout's gear case about the axle
rear_case := (-71mm, -20mm)            // round the hub's gear and pinion
axle_d := 12mm                         // the thru-axle

// -- the bearings and retaining rings (ISO 15, DIN 471 / 472) ----------------------------------------
b6804 := {d: 20mm, D: 32mm, B: 7mm, balls: 14, dw: 3.5mm}
b6805 := {d: 25mm, D: 37mm, B: 7mm, balls: 16, dw: 3.5mm}
b6903 := {d: 17mm, D: 30mm, B: 7mm, balls: 12, dw: 3.97mm}
b6901 := {d: 12mm, D: 24mm, B: 6mm, balls: 10, dw: 3.97mm}
// a ring's groove's diameter, its thickness, its band, and its eyes' circle, size and hole
r471_20 := {groove: 19mm, t: 1.2mm, band: 2.6mm, r_eye: 13.4mm, eye: 3.4mm, hole: 1.5mm}
r471_25 := {groove: 23.9mm, t: 1.2mm, band: 3mm, r_eye: 16.4mm, eye: 3.6mm, hole: 1.5mm}
r471_17 := {groove: 16.2mm, t: 1mm, band: 2.3mm, r_eye: 11.6mm, eye: 3mm, hole: 1.2mm}
r472_24 := {groove: 25.2mm, t: 1.2mm, band: 2.6mm, r_eye: 9.6mm, eye: 3.4mm, hole: 1.5mm}

// -- claims over the design table --------------------------------------------------------------------
claim Zr == Zs + 2 * Zp
claim (Zs + Zr) / planets == floor((Zs + Zr) / planets)
// The rack cuts no undercut: its tip stays above the interference point, rp sin² αt inside the
// pitch circle, on the smallest gear.
claim hf <= rpl * sin(alpha_t)^2
// The contact ratios: how many tooth pairs share the load on average, across the face (the overlap
// ratio, the helix's gift) and round the profile (transverse: sun–planet, and planet–ring,
// internal).  A helical mesh runs quiet where their sum stays above two.
eps_beta := pw * sin(beta) / (pi * mn)                                       // 1.52
base_pitch := pi * mt * cos(alpha_t)
reach_s := sqrt((rs + ha)^2 - (rs * cos(alpha_t))^2)       // each tip's reach along the line of action
reach_p := sqrt((rpl + ha)^2 - (rpl * cos(alpha_t))^2)
reach_r := sqrt((rr - ha)^2 - (rr * cos(alpha_t))^2)
eps_sp := (reach_s + reach_p - a * sin(alpha_t)) / base_pitch            // 1.19
eps_pr := (reach_p - reach_r + (rr - rpl) * sin(alpha_t)) / base_pitch   // 1.36
claim eps_beta >= 1
claim eps_sp >= 1.1 and eps_pr >= 1.1
claim eps_sp + eps_beta >= 2.2 and eps_pr + eps_beta >= 2.2
claim sigma_planet <= case_steel
claim shaft_tau <= 200MPa
claim spindle_tau <= 300MPa
claim wind <= 0.5deg
