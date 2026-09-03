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

// **A groove for an O-ring in a bore** (§6.9) — a feature that carries its own rule (issue #48,
// item 5).
//
// The rule is the part an LLM gets wrong, and it is one line of arithmetic nobody should be
// writing twice: a moving seal wants 10–20% squeeze on the ring's section, so the groove's
// bottom is the bore less twice the squeezed section, and the groove is a third wider than the
// section so the ring can roll rather than drag.  `hardware` states both numbers
// (`oring_squeeze`, `oring_groove_w`); this reads them.  A design then says *a groove for a
// #014* and the arithmetic is the library's.
//
//   use std
//   use hardware
//   g: Groove(body: pis, f: swing, o: crown, ax: axis, ac: across, dir: dir,
//             r: D / 2, z: -groove, cs: oring014_cs)
//
// `body` is the solid the groove is cut out of, and the statement inside is what does it: a
// component may contribute a `through` to a body it was handed, because the body rule is a set
// and not a sequence.  The groove is turned about the bore's own axis, so what is written here
// is its section: `w` wide at `z` down the axis, from the bore out to the squeezed diameter.
component Groove(body: solid, f: plane, o: point, ax: line, ac: line, dir: Angle,
                 r: Length, z: Length, cs: Length) {
  param rb = r - (1 - oring_squeeze) * cs   // the groove's bottom, off the axis
  param w = oring_groove_w * cs             // and how wide it is along the axis
  in f {
    g0: Loc(o, ax, ac, dir: dir, u: z, v: rb)
    g1: Loc(o, ax, ac, dir: dir, u: z, v: r)
    g2: Loc(o, ax, ac, dir: dir, u: z - w, v: r)
    g3: Loc(o, ax, ac, dir: dir, u: z - w, v: rb)
    line e0(g0.p, g1.p) class hidden -> line e1(g1.p, g2.p) class hidden ->
      line e2(g2.p, g3.p) class hidden -> line e3(g3.p, g0.p) class gone -> close
    face gf(e0, e1, e2, e3)
    claim g0.p distance(w) g3.p class detail
    claim g0.p distance(rb) ax class detail
  }
  solid groove(gf, about: ax)
  groove through body
}
