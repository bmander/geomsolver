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
// **Two views of one frame** (`motor.parts`).  The end view, `std.front`, is the
// electromagnetic section: the slots, the magnets, the ring, the spoke and screw circles, the
// carrier's windows, the key.  The half-section along the axle, `std.side`, is every turned
// profile — axle, carrier, covers, bearings, spacers, locknuts, end turns — each a stepped
// outline turned about the axle's centreline.  Every solid is a sweep of a face drawn in one of
// them; a hole is a face drawn where it is seen round, swept through the part it bores.
//
// **The parts** (each its own module, each a component handed the frame and the table):
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
// The drawing has no freedom: the rotor's angle is the table's `theta`, a north pole on the
// datum.  The claims at the end are the fits the assembly depends on — the gap, the copper
// clear of the covers, the stator clear of everything that turns — judged on the solids.

unit mm
use std (horizontal)
use motor.dims
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
frame := {front: std.front, side: std.side, top: std.top, o: std.origin, datum: datum,
          so: std.side.origin, ax: ax, to: std.top.origin}

// -- what stands still: the axle, the carrier keyed to it, the stator pressed on the carrier ---
axle := motor.axle.Axle(frame, dims: motor.dims.motor_dims)
carrier := motor.carrier.Carrier(frame, dims: motor.dims.motor_dims, key: axle.key,
                                 exit: axle.exit)
stator := motor.stator.Stator(frame, dims: motor.dims.motor_dims)

// -- what turns: the ring and its magnets, and the covers screwed to its ends -------------------
rotor := motor.rotor.Rotor(frame, dims: motor.dims.motor_dims)
left := motor.cover.Cover(frame, dims: motor.dims.motor_dims, screws: rotor.screw, sg: -1,
                          boss_to: motor.dims.disc_face, boss_r: motor.dims.disc_r,
                          spoke_phase: 0deg, disc: 1)
right := motor.cover.Cover(frame, dims: motor.dims.motor_dims, screws: rotor.screw, sg: 1,
                           boss_to: motor.dims.brg_in + motor.dims.brg_w + motor.dims.lip +
                                    motor.dims.fw_len,
                           boss_r: motor.dims.fw_r, spoke_phase: 180deg / motor.dims.spokes,
                           disc: 0)

// -- between them: the bearings on the axle's shoulders, and what clamps their inner races -----
// Each bearing stands on a shoulder of the carrier's seat; a spacer tube runs from its inner
// race to the step down to the thread, and the flanged locknut on the thread clamps the two
// against the shoulder, its face where the dropout's is, 67.5 from the mid-plane.
brg_in := motor.dims.brg_in
brg_out := motor.dims.brg_in + motor.dims.brg_w
bearing_l := motor.parts.Revolved(frame, s0: -brg_out, s1: -brg_in, r0: motor.dims.brg_id / 2,
                                  r1: motor.dims.brg_od / 2)
bearing_r := motor.parts.Revolved(frame, s0: brg_in, s1: brg_out, r0: motor.dims.brg_id / 2,
                                  r1: motor.dims.brg_od / 2)
spacer_l := motor.parts.Revolved(frame, s0: -motor.dims.step_end, s1: -brg_out,
                                 r0: motor.dims.d_seat / 2, r1: motor.dims.spacer_r)
spacer_r := motor.parts.Revolved(frame, s0: brg_out, s1: motor.dims.step_end,
                                 r0: motor.dims.d_seat / 2, r1: motor.dims.spacer_r)
nut_l := motor.parts.Revolved(frame, s0: -motor.dims.old / 2, s1: -motor.dims.step_end,
                              r0: motor.dims.d_end / 2, r1: motor.dims.nut_r)
nut_r := motor.parts.Revolved(frame, s0: motor.dims.step_end, s1: motor.dims.old / 2,
                              r0: motor.dims.d_end / 2, r1: motor.dims.nut_r)

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
