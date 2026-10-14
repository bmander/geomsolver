// The hub motor's numbers, stated once: a 1 hp (746 W) direct-drive, outer-rotor, brushless
// permanent-magnet motor laced into a 26" rear wheel and run from a 48 V pack.
//
// **What the motor must do.**  746 W at the wheel at 32 km/h is a cruising load on a 26 × 2.1"
// tyre (rolling circumference 2.09 m): 255 rpm and 27.9 N·m.  Everything electromagnetic below
// follows from that torque and the voltage, and everything mechanical from where the parts must
// stand on a 135 mm rear axle.
//
// **The machine.**  The magnets turn and the copper stands still: the stator is pressed on the
// axle, which is clamped in the dropouts, and the magnets are bonded inside a steel ring that is
// itself the hub shell, so the rotor's own radius is the lever.  46 poles over 51 slots — a
// fractional-slot concentrated winding, 17 coils a phase wound one to a tooth, gcd(51, 23) = 1 so
// the cogging is the 2346th harmonic of a turn and nearly nothing.  Torque comes from the shear
// in the air gap: 27.9 N·m on a 103.1 mm gap radius and a 35 mm stack is 11.9 kPa, inside the
// 10–20 kPa an air-cooled machine sustains.
//
// The language has two dimensions, length and angle, so the electrical figures are plain
// numbers in SI units, each with its unit in the comment; the lengths and angles carry theirs.
// A component receives this table as `dims`; `motor_dims` at the bottom is that group.

// -- the duty -------------------------------------------------------------------------------
power := 745.7          // W, shaft output: one mechanical horsepower
volts := 48             // V, the pack's nominal
rpm := 255              // rev/min at 32 km/h on a 2.09 m tyre
omega := rpm * 2 * pi / 60          // rad/s: 26.7
torque := power / omega             // N·m: 27.9
efficiency := 0.86      // at the rated point; copper and iron losses ~120 W
amps := power / (volts * efficiency)  // A from the pack at rated load: 18.1
// The controller: 48 V, 25 A continuous, 40 A peak, sinusoidal (FOC), three hall sensors.

// -- the electromagnetic sizing --------------------------------------------------------------
poles := 46             // 23 pole pairs
slots := 51             // 17 teeth a phase
pole_pitch := 360deg / poles        // 7.83°
slot_pitch := 360deg / slots        // 7.06°
freq := poles / 2 * rpm / 60        // Hz electrical at the rated speed: 97.8
// The winding: concentrated coils, one per tooth, phases A-a-A … in the 17-coil groups a 51/46
// star of slots gives; star-connected.  Winding factor 0.94 (pitch 0.988, spread 0.955).
// Back-EMF at the rated speed is 4.44 · f · N · k_w · Φ = 16.5 V rms a phase (28.5 V rms line,
// 40.3 V peak) under the pack's 48 V, leaving the controller its headroom; the magnet flux a pole
// is Φ = B_g · α · τ_p · L = 0.85 T · 0.80 · 14.1 mm · 35 mm = 0.335 mWb.
turns := 7              // turns a tooth: 119 a phase
strands := 5            // in hand, of 0.80 mm grade-2 enamelled copper (2.51 mm² a turn)
wire_d := 0.80mm
// 15.1 A rms a phase at rated torque is 6.0 A/mm² in the copper; a coil side is 17.6 mm² of
// copper in half a slot of 81 mm² (the slot drawn in `motor.stator`), a fill of 0.43 — wound
// by needle winder over 0.25 mm slot liners.

// -- the gap ----------------------------------------------------------------------------------
Rs := 102.5mm           // the stator's outside: the teeth's tips
gap := 0.6mm            // the air gap, radially
hm := 3.0mm             // magnet thickness, radially: N42SH sintered NdFeB, nickel plated
Rm := Rs + gap          // the magnets' inside face: 103.1
Rr := Rm + hm           // the rotor ring's bore, where the magnets are bonded: 106.1
alpha := 0.80           // pole arc over pole pitch, for a near-sinusoidal gap field
magnet_arc := alpha * pole_pitch    // 6.26°
L := 35mm               // the stack: 100 sheets of 0.35 mm M270-35A, bonded

// -- the stator lamination --------------------------------------------------------------------
// One slot is drawn: a tooth either side of it, parallel-sided, and shoes closing it to a slot
// opening `bo` at the tips.  At 0.85 T in the gap the teeth carry 1.6 T; the yoke's 0.6 T is
// thicker than flux asks, for the press fit on the carrier.
wt := 6.6mm             // tooth width
bo := 2.6mm             // the slot's opening between shoes
hs := 1.2mm             // the shoe's lip at the tip
hw := 2.0mm             // the shoe's taper below the lip, where the slot wedge sits
Ry := 84.5mm            // the slots' bottom: 18 mm deep
Rb := 76.5mm            // the stator's bore, on the carrier's rim: an 8 mm yoke
Rc := Rs - hs - hw      // where the coils stop under the shoes: 99.3
overhang := 6mm         // the end turns, axially, past each face of the stack

// -- the rotor ring: the back iron and the hub shell in one ------------------------------------
// 1018 steel tube, bored for the magnets.  Half a pole's flux at 1.5 T needs 3.2 mm; 6 mm (0.8 T)
// carries the cover screws' threads and stiffens the shell against the spokes' pull.
back := 6mm
Ro := Rr + back         // 112.1
half := 26mm            // half the ring's length: the stack, the end turns, 2.5 mm to the covers
screw_r := Rr + back / 2            // the cover screws' circle, mid-wall: 109.1
screws := 12            // M4 a side, 10 deep
tap4 := 3.3mm           // M4's tap drill
tap_depth := 10mm
clear4 := 4.5mm         // and its clearance in the cover

// -- the carrier: the stator's aluminium spider ------------------------------------------------
// 6061-T6, keyed to the axle, the stack pressed on its rim (H7/s6) and bonded.
hub_r := 20mm           // its hub's outside
hub_half := 22mm        // half its hub's length
web_half := 4mm         // half its web's thickness
rim_r := 70mm           // its rim's inside
windows := 6            // lightening windows through the web, for air and the phase wires
window_d := 28mm
window_r := 45mm        // their circle's radius

// -- the axle ---------------------------------------------------------------------------------
// 4140 steel, hardened and ground at the seats.  Stepped down from the carrier's seat to the
// bearings' and again to the dropouts', where 10 mm flats take the torque into the frame (use a
// torque arm: a start from rest can ask three times the rated torque).
old := 135mm            // over locknuts: a 135 mm rear dropout spacing
axle_half := 95mm       // to the end of the thread
d_core := 24mm          // the carrier's seat
d_seat := 20mm          // the bearings' seats and the spacers'
d_end := 14mm           // M14 × 1.5 in the dropouts
flats := 10mm           // across the flats
flats_from := 62mm      // the flats start, outboard of the locknut's thread
step_seat := 24mm       // the shoulder the bearings stand on: the carrier's seat ends here
step_end := 60mm        // the step down to the thread
cable_d := 8mm          // the phase and hall wires leave down the left end
cable_to := 10mm        // the cable bore stops this short of the mid-plane
exit_at := 14mm         // and turns out radially here, left of the web
exit_d := 7mm
key_w := 8mm            // DIN 6885 A 8 × 7 × 32: the carrier's torque into the axle
key_h := 7mm
key_l := 32mm
key_t1 := 4mm           // the axle's keyway depth
key_t2 := 3.3mm         // the carrier's

// -- the bearings: 6004-2RS, both covers ------------------------------------------------------
brg_id := 20mm
brg_od := 42mm
brg_w := 12mm
brg_in := step_seat     // each stands on the axle's shoulder

// -- the side covers --------------------------------------------------------------------------
// A356-T6 castings, machined: a spigot pilots each in the ring's bore, twelve M4 cap screws
// pull it onto the ring's end, a bearing sits in its boss against a lip, and its rim is the
// spoke flange.  The left carries the disc brake's ISO 6-bolt boss, the right the freewheel's
// thread.
web := 5mm              // the cover's web
spigot := 3mm           // how far the spigot enters the ring
spigot_r := Rm          // its inside: clear of the magnets' ends
boss_r := 27mm          // the bearing boss's outside
lip := 2mm              // the lip outboard of the bearing
lip_r := 14mm           // the lip's bore, clear of the spacer
pocket_in := brg_in - 1mm       // the boss's inner face, a millimetre inboard of the bearing
flange_r := 121mm       // the spoke flange's outside
flange_t := 3mm         // its thickness
spokes := 18            // a flange: 36 spokes, laced three-cross
spoke_r := 116.5mm      // the spoke holes' circle (233 mm PCD)
spoke_d := 2.6mm        // for 2.0/1.8/2.0 mm (14 g) spokes
// Disc side: an ISO 6-bolt boss, M5 on a 44 mm circle, its face 15.1 mm inboard of the
// dropout's face, as a 160 mm rotor and a post-mount caliper expect.
disc_face := old / 2 - 15.1mm   // 52.4 from the mid-plane
disc_r := 29mm          // the boss's outside
disc_bolts := 6
disc_bcd := 44mm
tap5 := 4.2mm           // M5's tap drill
disc_depth := 10mm
// Drive side: a BSA freewheel thread, 1.375" × 24 TPI, 16 mm long, for a 7-speed screw-on
// freewheel; the thread is cut, not drawn.
fw_r := 1.375" / 2
fw_len := 16mm

// -- the axle's spacers and locknuts ----------------------------------------------------------
spacer_r := 13mm        // a steel tube on the seat, from the bearing's inner race to the step
nut_r := 14mm           // the flanged locknut, its face at the dropout's
end_turn_in := Ry - 0.5mm       // the end turns' inside, just under the slots' bottom

// -- the hall sensors: SS41, in the mouths of slots 0, 17 and 34 --------------------------------
hall_w := 1.5mm         // across the slot
hall_r0 := Rc + 0.2mm   // radially, over the coils' wedge line
hall_r1 := Rs - 0.3mm   // and under the tips, 0.9 from the magnets
hall_l := 4mm           // along the axle, set into the stack's left end

motor_dims := {
  Rs: Rs, gap: gap, hm: hm, Rm: Rm, Rr: Rr, Ro: Ro, L: L, half: half, theta: 0deg,
  poles: poles, slots: slots, pole_pitch: pole_pitch, slot_pitch: slot_pitch,
  magnet_arc: magnet_arc,
  wt: wt, bo: bo, hs: hs, hw: hw, Ry: Ry, Rb: Rb, Rc: Rc, overhang: overhang,
  end_turn_in: end_turn_in, hall_w: hall_w, hall_r0: hall_r0, hall_r1: hall_r1, hall_l: hall_l,
  screw_r: screw_r, screws: screws, tap4: tap4, tap_depth: tap_depth, clear4: clear4,
  hub_r: hub_r, hub_half: hub_half, web_half: web_half, rim_r: rim_r, windows: windows,
  window_d: window_d, window_r: window_r,
  axle_half: axle_half, d_core: d_core, d_seat: d_seat, d_end: d_end, flats: flats,
  flats_from: flats_from, step_seat: step_seat, step_end: step_end, cable_d: cable_d,
  cable_to: cable_to, exit_at: exit_at, exit_d: exit_d,
  key_w: key_w, key_h: key_h, key_l: key_l, key_t1: key_t1, key_t2: key_t2,
  brg_id: brg_id, brg_od: brg_od, brg_w: brg_w, brg_in: brg_in,
  web: web, spigot: spigot, spigot_r: spigot_r, boss_r: boss_r, lip: lip, lip_r: lip_r,
  pocket_in: pocket_in, flange_r: flange_r, flange_t: flange_t, spokes: spokes,
  spoke_r: spoke_r, spoke_d: spoke_d,
  disc_face: disc_face, disc_r: disc_r, disc_bolts: disc_bolts, disc_bcd: disc_bcd,
  tap5: tap5, disc_depth: disc_depth, fw_r: fw_r, fw_len: fw_len,
  old: old, spacer_r: spacer_r, nut_r: nut_r
}
