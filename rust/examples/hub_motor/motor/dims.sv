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
// fractional-slot concentrated winding, 17 coils a phase wound one to a tooth, and the cogging
// has lcm(51, 46) = 2346 periods a turn, so it is nearly nothing.  Torque comes from the shear
// in the air gap: 27.9 N·m on a 103.1 mm gap radius and a 35 mm stack is 11.9 kPa, inside the
// 10–20 kPa an air-cooled machine sustains.
//
// **Every figure carries its unit** (spec §3.3, [0.98]): the duty and the electromagnetic sizing
// are values in watts, volts, amperes, teslas and webers, and the arithmetic between them is
// checked as a length's is, so `torque := power / omega` is a torque because it comes out one.
// The magnetic circuit is worked last, from the lengths it is drawn with: change `wt` or `back`
// and the teeth's or the ring's flux density follows.  No figure reaches a constraint; the
// comments say what each comes to.
//
// Every number here is a constant of each definition whose file `use`s it: a part names the ones
// it reads, `use motor.dims (Rs, L)`, and reads them bare, with nothing passed at a call.  What
// only the assembly reads (the bearings' bore, the covers' bosses, the spacers and locknuts) it
// reads the same way.

// -- the duty -------------------------------------------------------------------------------
power := 745.7W         // shaft output: one mechanical horsepower
volts := 48V            // the pack's nominal
cruise := 32km / 3600s  // 32 km/h
roll_out := 2.09m       // a 26 × 2.1" tyre's rolling circumference
speed := cruise / roll_out * turn   // 255 rpm
omega := speed / 1rad               // 26.7 rad/s: over a radian, a speed is a rate
torque := power / omega             // 27.9 N·m
efficiency := 0.86      // at the rated point
loss := power / efficiency - power  // 121 W, copper and iron
amps := power / (volts * efficiency)  // 18.1 A from the pack at rated load
// The controller: 48 V, 25 A continuous, 40 A peak, sinusoidal (FOC), three hall sensors.

// -- the electromagnetic sizing --------------------------------------------------------------
poles := 46             // 23 pole pairs
slots := 51             // 17 teeth a phase
pole_pitch := 360deg / poles        // 7.83°
slot_pitch := 360deg / slots        // 7.06°
freq := poles / 2 * speed / turn    // 97.8 Hz electrical at the rated speed
// The winding: concentrated coils, one per tooth, phases A-a-A … in the 17-coil groups a 51/46
// star of slots gives; star-connected.  A coil spans one slot, nine tenths of a pole.
k_pitch := sin(90deg * slot_pitch / pole_pitch)     // 0.988
k_spread := 0.955       // the spread of a phase's coils over its belts
k_w := k_pitch * k_spread           // the winding factor: 0.944
turns := 7              // turns a tooth
phase_turns := turns * slots / 3    // 119 a phase
strands := 5            // in hand, of 0.80 mm grade-2 enamelled copper
wire_d := 0.80mm
// Wound by needle winder over 0.25 mm slot liners.

// -- the gap ----------------------------------------------------------------------------------
Rs := 102.5mm           // the stator's outside: the teeth's tips
gap := 0.6mm            // the air gap, radially
hm := 3.0mm             // magnet thickness, radially: N42SH sintered NdFeB, nickel plated
Rm := Rs + gap          // the magnets' inside face: 103.1
Rr := Rm + hm           // the rotor ring's bore, where the magnets are bonded: 106.1
alpha := 0.80           // pole arc over pole pitch, for a near-sinusoidal gap field
magnet_arc := alpha * pole_pitch    // 6.26°
L := 35mm               // the stack: 100 sheets of 0.35 mm M270-35A, bonded
theta := 0deg           // the rotor's angle in the drawing: a north pole's centre on the datum

// -- the stator lamination --------------------------------------------------------------------
// One slot is drawn: a tooth either side of it, parallel-sided, and shoes closing it to a slot
// opening `bo` at the tips.  The yoke is thicker than flux asks, for the press fit on the
// carrier (`B_yoke` below).
wt := 6.6mm             // tooth width
bo := 2.6mm             // the slot's opening between shoes
hs := 1.2mm             // the shoe's lip at the tip
hw := 2.0mm             // the shoe's taper below the lip, where the slot wedge sits
Ry := 84.5mm            // the slots' bottom: 18 mm deep
Rb := 76.5mm            // the stator's bore, on the carrier's rim: an 8 mm yoke
Rc := Rs - hs - hw      // where the coils stop under the shoes: 99.3
overhang := 6mm         // the end turns, axially, past each face of the stack
end_turn_in := Ry - 0.5mm       // the end turns' inside, just under the slots' bottom

// -- the rotor ring: the back iron and the hub shell in one ------------------------------------
// 1018 steel tube, bored for the magnets.  Flux asks 3.2 mm of it (`back_min` below); 6 mm
// carries the cover screws' threads and stiffens the shell against the spokes' pull.
back := 6mm
Ro := Rr + back         // 112.1
end_clear := 2.5mm      // from the end turns to the covers
half := L / 2 + overhang + end_clear    // half the ring's length: 26
screw_r := Rr + back / 2            // the cover screws' circle, mid-wall: 109.1
screws := 12            // M4 a side, 10 deep
tap4 := 3.3mm           // M4's tap drill
tap_depth := 10mm
clear4 := 4.5mm         // and its clearance in the cover

// -- the magnetic circuit, from the lengths above ----------------------------------------------
// The gap's flux density is an estimate from 3 mm of N42SH over a 0.6 mm gap: confirm it with a
// 2D FEA before tooling.  Everything after it is arithmetic the units check.
B_g := 0.85T
tau_p := Rm * pole_pitch / 1rad     // the pole pitch along the magnets' face: 14.1 mm
flux := B_g * alpha * tau_p * L     // a pole's: 0.335 mWb
B_tooth := B_g * (Rs * slot_pitch / 1rad) / wt  // a slot pitch's flux through a tooth: 1.63 T
B_yoke := flux / 2 / ((Ry - Rb) * L)            // 0.60 T
B_back := flux / 2 / (back * L)                 // in the rotor ring: 0.80 T
back_min := flux / 2 / (1.5T * L)               // the ring that would carry it at 1.5 T: 3.2 mm
// The back-EMF at the rated speed, 4.44 · f · N · k_w · Φ, is under the pack's 48 V at its
// peak, which leaves the controller its headroom.
emf := pi * sqrt(2) * freq * phase_turns * k_w * flux   // 16.4 V rms a phase
emf_line := emf * sqrt(3)                        // 28.3 V rms line to line
emf_peak := emf_line * sqrt(2)                   // 40.1 V
phase_amps := power / (3 * emf)     // 15.2 A rms a phase at rated torque
copper := strands * pi / 4 * wire_d^2           // 2.51 mm² a turn
density := phase_amps / copper      // 6.0 A/mm² in the copper
coil_side := turns * copper         // 17.6 mm²
slot_area := 81mm * 1mm             // the slot drawn in `motor.stator`, within the stack
fill := 2 * coil_side / slot_area   // 0.43
shear := torque / (2 * pi * Rm^2 * L)   // the air gap's shear: 11.9 kPa, air-cooled territory

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
fw_to := brg_in + brg_w + lip + fw_len  // the thread's end from the mid-plane: 54

// -- the axle's spacers and locknuts ----------------------------------------------------------
spacer_r := 13mm        // a steel tube on the seat, from the bearing's inner race to the step
nut_r := 14mm           // the flanged locknut, its face at the dropout's

// -- the hall sensors: SS41, in the mouths of slots 0, 17 and 34 --------------------------------
hall_w := 1.5mm         // across the slot
hall_r0 := Rc + 0.2mm   // radially, over the coils' wedge line
hall_r1 := Rs - 0.3mm   // and under the tips, 0.9 from the magnets
hall_l := 4mm           // along the axle, set into the stack's left end
