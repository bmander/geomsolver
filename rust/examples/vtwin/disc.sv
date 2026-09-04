// The crank disc: **one section, and the solid it is a section of** (§6.9).
//
// Seen along the axis — which is how it is designed — the disc is a rim, its bore for the 5/16"
// shaft, the hole for the clevis pin `R` out, the pocket in its back the pin's head sits in, and
// the #8-32 set screw square to the crank arm, in a hole from the rim to the bore with its nut
// trapped in a pocket.  Every one of those is that circle swept `tdisc` along the axis, or a
// circle swept through it, so the thickness is stated once and the views that show it are asked
// for rather than drawn.  (The nut's *pocket* is the one feature that is not part of the body:
// it is a hex prism about a radial line, which no sweep the language has can make — see
// `parts.Grub`.)  The assembly's crank
// instances it turned to the pin; the part sheet instances it once, the pin at the top.  The
// crank pin is a 1/4" × 1-1/4" clevis pin: its head in the pocket behind, the two rod eyes on
// its shank with a washer against the disc and one under the hairpin cotter, so the pin is
// captured between its head and the cotter and nothing is threaded into plastic.

use std
use vtwin.dims
use vtwin.parts

component Disc(swing: plane, o: point, pin: point, arm: line, dir: Angle) {
  in swing {
    circle rim(center: o) hint(r: rdisc)
    radius(rdisc) rim class shown at (-2.1, 32)
    circle bore(center: o) hint(r: dhub / 2) class hidden detail
    radius(dhub / 2) bore
    circle ph(center: pin) hint(r: pinclr / 2) class hidden detail
    radius(pinclr / 2) ph
    circle pkt(center: pin) hint(r: pinpocketd / 2) class hidden detail
    radius(pinpocketd / 2) pkt
    // the set screw, square to the arm so its pocket stays clear of the pin's
    point se hint(x: o.x + rdisc * cos(dir - 90deg), y: o.y + rdisc * sin(dir - 90deg))
    line ssa(o, se) class gone
    o distance(rdisc) se
    arm angle(90deg, sense: cw) ssa
    gs: Grub(o, ssa, arm, dir: dir - 90deg, rin: dhub / 2, rout: rdisc)
    claim radius(dhub / 2) bore class detail at (2.4, 12)
    claim radius(pinclr / 2) ph class detail at (0.8, 10)
    claim radius(pinpocketd / 2) pkt class detail at (1.4, 14)
    // what the solid is swept from: each circle is a loop by itself
    face rim_f(rim)
    face bore_f(bore)
    face ph_f(ph)
    face pkt_f(pkt)
  }

  // -- the solid: the section's faces swept, and the body their one rule (§6.9) ----------------
  // **The section is the disc's mid-plane**, and it is the set screw that says so: its hole is a
  // turn about a line lying in this plane, so the hole comes out centred on the plane whatever
  // else is written, and the screw runs down the middle of the thickness.  Drawn from the back
  // face instead, half the hole would have been in fresh air.  So the material is half a
  // thickness either way, and the pin's head sits in a pocket `pinpocket` deep in the back —
  // the face toward the plate, which a view from the right sees on its own right.
  solid plate(rim_f, from: -tdisc / 2, to: tdisc / 2)
  solid hub(bore_f, from: -tdisc / 2, to: tdisc / 2)
  solid pinhole(ph_f, from: -tdisc / 2, to: tdisc / 2)
  solid pinpkt(pkt_f, from: -tdisc / 2, to: -tdisc / 2 + pinpocket)
  solid body(plate)
  hub through body
  pinhole through body
  pinpkt through body
  gs.bore through body
}
