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

// -- the hypoid pairs ------------------------------------------------------------------------------
// The pair of `spiral_bevel/hypoid.sv`, 24 and 48 teeth, its every length taken at `scale`: a mean
// module of 1.25 mm, the gear 67 mm across, the shafts 15.6 mm apart.  The pair is laid out and
// cut there exactly as at its own 2 mm (README: what solventc lacks).
scale := 0.625
pinion_teeth := 24
gear_teeth := 48
hyp_module := 2mm * scale
hyp_offset := 25mm * scale
hyp_backlash := 0.05mm * scale
hyp_tip_relief := 0.2mm * scale
hyp_end_relief := 0.2mm * scale
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
shaft_od := 16mm                       // a cold-drawn 25CrMo4 tube, 16 × 1.5
shaft_id := 13mm
shaft_G := 80000MPa                    // shear modulus of steel
cs_length := 430mm                     // the chainstay: bottom-bracket centre to rear axle
bb_drop := 70mm                        // the axle stands this far above the bottom bracket
shaft_y := -40mm                       // the shaft's plane, out from the frame's mid-plane on the drive
                                       // side: clear of the spokes from a hub flange at -11 mm
J := pi * (shaft_od^4 - shaft_id^4) / 32           // 3630 mm⁴
shaft_tau := shaft_peak * (shaft_od / 2) / J       // 66 MPa at the sprint peak
// The wind-up the crank feels at the sprint peak, the shaft driven at k times crank speed, and
// the same shaft driven 1:1 at the crank's speed and torque.
wind := shaft_peak * cs_length / (shaft_G * J) * 1rad / k        // 0.42° at the crank
wind_direct := crank_peak * cs_length / (shaft_G * J) * 1rad     // 15.3°
wind_gain := wind_direct / wind                                  // k² = 36
tube_od := 30mm                        // the chainstay, 7005 aluminium, 30 × 2
tube_id := 26mm

// -- the bottom bracket -------------------------------------------------------------------------------
shell_od := 82mm                       // the shell: an oversized bottom bracket, the gearbox's case
shell_id := 72mm                       // the ring gear is pressed into it
case_left := 47mm                      // the shell's faces: the left (+y) one open for the bearing cup,
case_right := -76mm                    // the right one closed round the right-hand bearing, outboard of
                                       // the hypoid pinion
end_wall := 4mm
bearing_od := 32mm                     // the spindle's 6804s, 20 × 32 × 7
sleeve_bearing_od := 37mm              // the sun's sleeve's 6805, 25 × 37 × 7
diaphragm := (-9mm, -2mm)              // the web in the shell that holds the 6805, between the
                                       // planetary and the hypoid gear
boss_r := 28mm                         // the pinion's housing: a cylinder about the shaft's axis,
                                       // round a hypoid pinion 46.8 mm across
boss_ahead := 22mm                     // from this far ahead of the pinion's mean point
boss_behind := 30mm                    // to this far behind it
wall := 2.5mm
web := (-1.5mm, 4mm)                   // a hypoid member's web: its band along the axis from the foot
                                       // of its mean point, filling its core out to the blank's back
hub_r := 12.5mm                        // the hub's shell at its gear: the sun's sleeve's radius, so the
                                       // hub's gear and web are the bracket's turned end for end
hub_bore := 9mm                        // over a 12 mm thru-axle's bearings
hub_span := (-66mm, 42mm)
flange_r := 28mm
flanges := (-8mm, 34mm)                // the spoke flanges' inner faces, 3 mm thick
spokes := 16                           // a side, 2.6 mm holes on a 48 mm circle
rear_case_r := 40mm                    // the dropout's gear case about the axle
rear_case := (-71mm, -12mm)            // inboard of the hub's pinion, which reaches -16.5 mm
crank_throw := 170mm
ring_od := 72mm
spindle_d := 20mm                      // the crank spindle, hollow 20 × 14, 4340 at 40 HRC
spindle_bore := 14mm
sun_bore := 21mm                       // the sun runs on the spindle on a 0.5 mm-wall bronze bush
sleeve_r := 12.5mm                     // the sun's sleeve, 25 across: it runs in a 6805 (25 × 37 × 7)
sleeve_to := 34mm                      // and reaches past the hypoid gear it carries
spindle_tau := crank_peak * (spindle_d / 2) / (pi * (spindle_d^4 - spindle_bore^4) / 32)  // 151 MPa
planet_bore := 7mm                     // the planets run on 5 mm pins in 1 mm-wall bushes
pin_d := 5mm
ring_y := 10mm                         // the planetary's mid-plane, out from the frame's toward the
                                       // non-drive side (+y)
plate_t := 3mm                         // the carrier's plates
plate_r := 29mm
gap := 0.5mm                           // between a planet's face and a plate
bridge_w := 16mm                       // the carrier's bridges between the planets, across
bridge_r := 18.5mm                     // their inside, clear of the sun's tips

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
