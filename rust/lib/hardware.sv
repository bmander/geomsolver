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

hexbolt14_d := 6.35mm          // the shank
hexbolt14_af := 11.1mm         // the head, across flats (7/16")
hexbolt14_ac := 12.8mm         // and across corners
hexbolt14_h := 4.4mm           // the head's height
hexbolt14_thread := 19mm       // a partially threaded bolt's thread, from the tip (3/4")
nut14_af := 11.1mm             // a 1/4"-20 hex nut, across flats
nut14_h := 5.6mm               // its height
nylock14_h := 7.6mm            // a nylon-insert nut's
washer14_id := 7.1mm           // a 1/4" SAE flat washer
washer14_od := 15.9mm
washer14_t := 1.6mm
clearance14 := 6.6mm           // the hole a 1/4" shank turns in
fit14 := 6.4mm                 // the hole that locates one without turning: drill it 1/4"

// -- #8-32 -------------------------------------------------------------------------------------
screw832_d := 4.2mm
screw832_clearance := 4.4mm    // the hole it passes through
nut832_af := 8.7mm             // an #8-32 hex nut (11/32")
nut832_ac := 10mm
nut832_t := 3.2mm

// -- pins and rod ------------------------------------------------------------------------------
clevis14_d := 6.35mm           // a 1/4" clevis pin
clevis14_head_d := 9.7mm
clevis14_head_t := 2.3mm
clevis14_grip_114 := 25mm      // under the head to the cotter hole, the 1-1/4" pin
rod516_d := 7.94mm             // 5/16" steel rod: a press fit in a 608 bearing
rod316_d := 4.76mm             // 3/16" steel rod

// -- bearings ----------------------------------------------------------------------------------
brg608_id := 8mm               // a 608 skateboard bearing
brg608_od := 22mm
brg608_w := 7mm

// -- O-rings, AS568 dash numbers: the bore and the section --------------------------------------
oring010_id := 6.07mm          // 1/4" × 1/16"
oring010_cs := 1.78mm
oring014_id := 12.42mm         // 1/2" × 1/16"
oring014_cs := 1.78mm
oring112_id := 12.37mm         // 1/2" × 3/32"
oring112_cs := 2.62mm
// A groove for a ring in a bore: the ring's section squeezed this much is a moving seal that
// holds; the groove is this much wider than the section so the ring can roll rather than drag.
oring_squeeze := 0.12
oring_groove_w := 1.35

// -- pipe fittings -----------------------------------------------------------------------------
npt14_cpl_af := 15.9mm         // a 1/4" NPT brass coupling, across flats (5/8")
npt14_cpl_l := 28.6mm          // and long (1-1/8")
npt14_drill := 11.1mm          // the tap drill for 1/4" NPT (7/16")
mplug_body_d := 12mm           // an industrial ("M-style") quick-release plug, 1/4" NPT
mplug_body_l := 14mm
mplug_nose_d := 7mm
mplug_nose_l := 16mm

// -- how they are drawn ------------------------------------------------------------------------
// A washer or a bearing face on: two circles about `c`.
component Ring(c: point, id: Length, od: Length) {
  outer := circle(center: c) hint(r: od / 2)
  radius(od / 2) outer
  inner := circle(center: c) hint(r: id / 2)
  radius(id / 2) inner
}

// A nut or a bolt's head face on: the hex, with the bore through it.
component Nut(c: point, ref: line, af: Length, bore: Length, phase: Angle) {
  hex := std.Hex(c, ref, af: af, phase: phase)
  hole := circle(center: c) hint(r: bore / 2)
  radius(bore / 2) hole
}

// **A groove for an O-ring in a bore** (§6.9) — a feature that carries its own rule (issue #48,
// item 5).
//
// The rule is the part an LLM gets wrong, and it is one line of arithmetic nobody should be
// writing twice: a moving seal wants 10–20% squeeze on the ring's section, so the groove's
// bottom is the bore less twice the squeezed section, and the groove is a third wider than the
// section so the ring can roll rather than drag.  `hardware` states both numbers
// (`oring_squeeze`, `oring_groove_w`) in `seal_dims`, which the caller supplies.  A design then says *a groove for a
// #014* and the arithmetic is the library's.
//
//   use std
//   use hardware
//   g: Groove(body: pis, f: axis_datum, r: D / 2,
//             z: -groove, cs: oring014_cs, seal: seal_dims) in swing
//
// `body` is the solid the groove is cut out of, and the statement inside is what does it: a
// component may contribute a `through` to a body it was handed, because the body rule is a set
// and not a sequence.  The groove is turned about the bore's own axis, so what is written here
// is its section: `w` wide at `z` down the axis, from the bore out to the squeezed diameter.
component Groove(body: solid, f: plane,
                 r: Length, z: Length, cs: Length, seal: group) {
  ax := line(f.origin, f.toward)
  rb := r - (1 - seal.squeeze) * cs   // the groove's bottom, off the axis
  w := seal.width_factor * cs             // and how wide it is along the axis
  g0 := point hint(x: f.origin.x + (z) * f.c - (rb) * f.s,
                    y: f.origin.y + (z) * f.s + (rb) * f.c)
  g1 := point hint(x: f.origin.x + (z) * f.c - (r) * f.s,
                    y: f.origin.y + (z) * f.s + (r) * f.c)
  g2 := point hint(x: f.origin.x + (z - w) * f.c - (r) * f.s,
                    y: f.origin.y + (z - w) * f.s + (r) * f.c)
  g3 := point hint(x: f.origin.x + (z - w) * f.c - (rb) * f.s,
                    y: f.origin.y + (z - w) * f.s + (rb) * f.c)
  (e0 := line(g0, g1))  -> (e1 := line(g1, g2))  ->
    (e2 := line(g2, g3))  -> (e3 := line(g3, g0))  -> close
  gf := face(e0, e1, e2, e3)
  e0 perpendicular ax
  e1 parallel ax
  e2 perpendicular ax
  e3 parallel ax
  distance(r - rb) e0
  distance(w) e1
  g0 distance(rb, side: left) ax
  g0 distance(z, along: u) f
  claim g0 distance(w) g3
  claim g0 distance(rb) ax
  groove := solid(gf, about: ax)
  groove cut body
}

// Caller-selected seal proportions, passed explicitly to Groove.
seal_dims := {squeeze: oring_squeeze, width_factor: oring_groove_w}

// A circular through-hole pattern. The polygon supplies constrained centers, with
// its edges private to this component. n >= 3, as for Polygon.
component BoltPattern(body: solid, center: point, ref: line,
                      n: Int, pitch_r: Length, hole_r: Length, phase: Angle) {
  private construction layout := std.Polygon(center, ref, n: n, r: pitch_r, phase: phase)
  repeat n as i {
    hole := radius(hole_r) circle(center: layout.v[i])
    private drill := solid(face(hole), through: body)
    drill cut body
  }
}
