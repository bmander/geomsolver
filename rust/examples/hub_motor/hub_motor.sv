// A 1 hp electric bicycle hub motor: direct drive, outer rotor, 46 poles on 51 slots, 48 V.
//
// The wheel turns on the motor, and the motor is the hub.  The axle is clamped in the frame's
// dropouts and never turns; the stator — 51 wound teeth on an aluminium carrier keyed to the
// axle — stands on it, and around it the rotor turns on two bearings: a steel ring that is the
// magnets' back iron and the hub's shell at once, its two cast covers carrying the bearings,
// the spoke flanges, the brake disc on the left and the freewheel on the right.  The rating,
// and the electromagnetic sizing that meets it, is `motor.dims`: 746 W and 27.9 N·m at 255
// rpm, 32 km/h on a 26" wheel, from a 48 V pack at 18 A.
//
// **Two views of one frame** (`motor.parts.Datums`).  The end view, `std.front`, is the
// electromagnetic section: the slots, the magnets, the ring, the spoke and screw circles, the
// carrier's windows, the key.  The half-section along the axle, `std.side`, is every turned
// profile — axle, carrier, covers, bearings, spacers, locknuts, end turns — each a stepped
// outline turned about the axle's centreline.  Every solid is a sweep of a face drawn in one of
// them; a hole is a face drawn where it is seen round, swept through the part it bores.
//
// **The parts** (each its own module, each a definition: a function of the frame it is drawn about and
// what it is mated to, reading the table, `motor.dims`, as constants of its own definition):
//
//   motor.axle      Axle      4140, Ø24 / Ø20 / M14 × 1.5 with 10 mm flats; the key; the cable port
//   motor.carrier   Carrier   6061-T6 spider: hub, web with six windows, rim under the stack
//   motor.stator    Stator    51-slot M270-35A lamination stack, 35 mm; the windings
//   motor.rotor     Rotor     1018 ring, Ø212.2 bore, 52 long, tapped M4 × 24; 46 N42SH magnets
//   motor.cover     Cover     A356-T6 side covers: spigot, bearing boss, 18-hole spoke flange,
//                             ISO 6-bolt boss (left) or BSA freewheel thread (right)
//   motor.parts     Revolved  6004-2RS bearings, steel spacers, flanged locknuts
//
// Bought in: two 6004-2RS bearings; 24 M4 × 12 socket cap screws (12.9, threadlocked); a DIN
// 6885 A 8 × 7 × 32 key; two M14 × 1.5 axle nuts and a torque arm; three SS41 hall sensors
// (drawn, in the stator); a 3 × 2.5 mm² + 5 × 0.3 mm² cable.
//
// The drawing has no freedom: the rotor's angle is its formal, given the table's `theta`, a north
// pole on the datum.  The claims at the end are the fits the assembly depends on — the gap, the
// copper clear of the covers, the stator clear of everything that turns — judged on the solids.

unit mm
use std (horizontal)
use motor.dims (brg_id, brg_in, brg_od, brg_w, d_end, d_seat, disc_face, disc_r, fw_r, fw_to, old,
                spokes, step_end)
use motor.parts
use motor.axle
use motor.carrier
use motor.stator
use motor.rotor
use motor.cover

// -- the frame: the end view's datum and the axle's centreline in the section ------------------
in std.front {
  construction centerline datum := line(std.origin, hint((150, 0)))
  horizontal datum
  length(150mm) datum
}
in std.side {
  construction centerline ax := line(std.side.origin, hint((100, 0)))
  horizontal ax
  length(100mm) ax
}
// what every part is drawn about: each draws in the planes these are drawn in
frame := {o: std.origin, datum: datum, so: std.side.origin, ax: ax}

// -- what stands still: the axle, the carrier keyed to it, the stator pressed on the carrier ---
// (the axle's cable port is drawn in the plan, about its origin)
axle := motor.axle.Axle(frame, std.top.origin)
carrier := motor.carrier.Carrier(frame, exit: axle.exit)
stator := motor.stator.Stator(frame)

// -- what turns: the ring and its magnets, and the covers screwed to its ends -------------------
rotor := motor.rotor.Rotor(frame, theta: motor.dims.theta)
// each cover drawn as the right-hand one, and the brake side's mirrored in the mid-plane (#578)
left := motor.cover.Cover(boss_to: disc_face, boss_r: disc_r, spoke_phase: 0deg,
                          disc: 1) mirrored(front)
right := motor.cover.Cover(boss_to: fw_to, boss_r: fw_r, spoke_phase: 180deg / spokes, disc: 0)

// -- between them: the bearings on the axle's shoulders, and what clamps their inner races -----
// Each bearing stands on a shoulder of the carrier's seat; a spacer tube runs from its inner
// race to the step down to the thread, and the flanged locknut on the thread clamps the two
// against the shoulder, its face where the dropout's is, 67.5 from the mid-plane.  Each on the
// left is the right one mirrored in the mid-plane: three figures, each placed twice.
brg_out := brg_in + brg_w
bearing_l := motor.parts.Revolved(s0: brg_in, s1: brg_out, r0: brg_id / 2,
                                  r1: brg_od / 2) mirrored(front)
bearing_r := motor.parts.Revolved(s0: brg_in, s1: brg_out, r0: brg_id / 2, r1: brg_od / 2)
spacer_l := motor.parts.Revolved(s0: brg_out, s1: step_end, r0: d_seat / 2,
                                 r1: motor.dims.spacer_r) mirrored(front)
spacer_r := motor.parts.Revolved(s0: brg_out, s1: step_end, r0: d_seat / 2, r1: motor.dims.spacer_r)
nut_l := motor.parts.Revolved(s0: step_end, s1: old / 2, r0: d_end / 2,
                              r1: motor.dims.nut_r) mirrored(front)
nut_r := motor.parts.Revolved(s0: step_end, s1: old / 2, r0: d_end / 2, r1: motor.dims.nut_r)

// -- the fits -------------------------------------------------------------------------------
claim stator.lamination clear(0.5mm) rotor.magnets      // the air gap, 0.6 drawn
claim stator.windings clear(2mm) left.body               // the end turns, 2.5 from each cover
claim stator.windings clear(2mm) right.body
claim stator.lamination clear(5mm) left.body
claim stator.halls clear(0.5mm) rotor.magnets           // the hall sensors, under the gap
claim stator.halls clear(0.4mm) stator.lamination       // and between the shoes
claim carrier.body clear(0.5mm) left.body               // the carrier's hub, short of the bosses
claim carrier.body clear(0.5mm) right.body
claim carrier.body clear(1mm) bearing_l.body
claim carrier.body clear(1mm) bearing_r.body
claim spacer_l.body clear(0.5mm) left.body              // the spacers turn with the axle, not
claim spacer_r.body clear(0.5mm) right.body             // the covers: they clear the lips
// the covers' clearance holes, drawn by each cover from the table, on the ring's tapped holes
repeat motor.dims.screws as i {
  claim left.clear[i].center coincident rotor.screw[i].center
  claim right.clear[i].center coincident rotor.screw[i].center
}
