// The hardware store, as a module: what the common fasteners and fittings measure, each stated
// once under a name a drawing can read (§14.4).
//
// `use hardware` puts the table in scope, so a dimension table writes `param boltaf =
// hexbolt14_af` and a pocket is drawn to the bolt that will sit in it rather than to a number
// somebody remembered.  Sizes are in millimetres whatever the part is sold as; the name says the
// nominal size the way the bin label does — `14` is 1/4", `516` is 5/16", `832` is #8-32.
// Alongside the numbers are the few figures a drawing wants of them: a nut or a bolt's head
// face on is `std`'s `Hex`; a washer, a bearing and a pin's head are the circles below.

use std

// -- bolts, nuts and washers, 1/4"-20 ----------------------------------------------------------
param hexbolt14_d = 6.35mm          // the shank
param hexbolt14_af = 11.1mm         // the head, across flats (7/16")
param hexbolt14_ac = 12.8mm         // and across corners
param hexbolt14_h = 4.4mm           // the head's height
param hexbolt14_thread = 19mm       // a partially threaded bolt's thread, from the tip (3/4")
param nut14_af = 11.1mm             // a 1/4"-20 hex nut, across flats
param nut14_h = 5.6mm               // its height
param nylock14_h = 7.6mm            // a nylon-insert nut's
param washer14_id = 7.1mm           // a 1/4" SAE flat washer
param washer14_od = 15.9mm
param washer14_t = 1.6mm
param clearance14 = 6.6mm           // the hole a 1/4" shank turns in
param fit14 = 6.4mm                 // the hole that locates one without turning: drill it 1/4"

// -- #8-32 -------------------------------------------------------------------------------------
param screw832_d = 4.2mm
param screw832_clearance = 4.4mm    // the hole it passes through
param nut832_af = 8.7mm             // an #8-32 hex nut (11/32")
param nut832_ac = 10mm
param nut832_t = 3.2mm

// -- pins and rod ------------------------------------------------------------------------------
param clevis14_d = 6.35mm           // a 1/4" clevis pin
param clevis14_head_d = 9.7mm
param clevis14_head_t = 2.3mm
param clevis14_grip_114 = 25mm      // under the head to the cotter hole, the 1-1/4" pin
param rod516_d = 7.94mm             // 5/16" steel rod: a press fit in a 608 bearing
param rod316_d = 4.76mm             // 3/16" steel rod

// -- bearings ----------------------------------------------------------------------------------
param brg608_id = 8mm               // a 608 skateboard bearing
param brg608_od = 22mm
param brg608_w = 7mm

// -- O-rings, AS568 dash numbers: the bore and the section --------------------------------------
param oring010_id = 6.07mm          // 1/4" × 1/16"
param oring010_cs = 1.78mm
param oring014_id = 12.42mm         // 1/2" × 1/16"
param oring014_cs = 1.78mm
param oring112_id = 12.37mm         // 1/2" × 3/32"
param oring112_cs = 2.62mm
// A groove for a ring in a bore: the ring's section squeezed this much is a moving seal that
// holds; the groove is this much wider than the section so the ring can roll rather than drag.
param oring_squeeze = 0.12
param oring_groove_w = 1.35

// -- pipe fittings -----------------------------------------------------------------------------
param npt14_cpl_af = 15.9mm         // a 1/4" NPT brass coupling, across flats (5/8")
param npt14_cpl_l = 28.6mm          // and long (1-1/8")
param npt14_drill = 11.1mm          // the tap drill for 1/4" NPT (7/16")
param mplug_body_d = 12mm           // an industrial ("M-style") quick-release plug, 1/4" NPT
param mplug_body_l = 14mm
param mplug_nose_d = 7mm
param mplug_nose_l = 16mm

// -- how they are drawn ------------------------------------------------------------------------
// A washer or a bearing face on: two circles about `c`.
component Ring(c: point, id: Length, od: Length) {
  circle outer(center: c) hint(r: od / 2)
  radius(od / 2) outer
  circle inner(center: c) hint(r: id / 2)
  radius(id / 2) inner
}

// A nut or a bolt's head face on: the hex, with the bore through it.
component Nut(c: point, ref: line, af: Length, bore: Length, phase: Angle) {
  hex: Hex(c, ref, af: af, phase: phase)
  circle hole(center: c) hint(r: bore / 2)
  radius(bore / 2) hole
}
