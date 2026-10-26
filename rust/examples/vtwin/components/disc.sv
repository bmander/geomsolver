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
use std.place (vertical, on_u)
use components.dims (R, dhub, pinclr, pinhead, pinpocket, rdisc, tdisc)
use components.parts

// f is the disc's own frame: origin on the crank axis, u toward the pin.  The disc is drawn in it,
// so every number below is the part's own and the frame is all that turns.
component Disc(f: group) {
  pinpocketd := pinhead + 0.5mm   // diametral clearance around the clevis head
  in f.axes {
    // the clevis pin's centre, the throw `R` along the arm
    pin := point hint((R, 0mm))
    pin on_u(d: R) f.axes
    rim := circle(center: f.axes.origin) hint(r: rdisc)
    radius(rdisc) rim
    bore := circle(center: f.axes.origin) hint(r: dhub / 2)
    radius(dhub / 2) bore
    ph := circle(center: pin) hint(r: pinclr / 2)
    radius(pinclr / 2) ph
    pkt := circle(center: pin) hint(r: pinpocketd / 2)
    radius(pinpocketd / 2) pkt
    // the set screw, square to the arm so its pocket stays clear of the pin's
    se := point hint((0mm, -rdisc))
    ssa := line(f.axes.origin, se)
    reference := line(f.axes.origin, pin)
    se coincident rim
    ssa perpendicular reference
    cw(f.axes.origin, pin, se)
    screw_axes := std.Turned(f.axes.origin, se)
    gs := components.parts.Grub(screw_axes, rin: dhub / 2, rout: rdisc)
    claim radius(dhub / 2) bore
    claim radius(pinclr / 2) ph
    claim radius(pinpocketd / 2) pkt
  }

  // -- the solid: the section's faces swept, and the body their one rule (§6.9) ----------------
  // **The section is the disc's mid-plane**, and it is the set screw that says so: its hole is a
  // turn about a line lying in this plane, so the hole comes out centred on the plane whatever
  // else is written, and the screw runs down the middle of the thickness.  Drawn from the back
  // face instead, half the hole would have been in fresh air.  So the material is half a
  // thickness either way, and the pin's head sits in a pocket `pinpocket` deep in the back —
  // the face toward the plate, which a view from the right sees on its own right.
  // Each circle is a loop by itself.
  plate := solid(face(rim), from: -tdisc / 2, to: tdisc / 2)
  hub := solid(face(bore), from: -tdisc / 2, to: tdisc / 2)
  pinhole := solid(face(ph), from: -tdisc / 2, to: tdisc / 2)
  pinpkt := solid(face(pkt), from: -tdisc / 2, to: -tdisc / 2 + pinpocket)
  body := solid(plate)
  hub cut body
  pinhole cut body
  pinpkt cut body
  gs.bore cut body
}

// Open this file to preview the disc with the crank pin at the top.
// ../disc.svd arranges three projections of this preview.
preview {
  unit mm
  in std.front {
    pin := point hint(at: std.up, (components.dims.R, 0mm))
    std.origin vertical pin
    std.origin distance(components.dims.R) pin
    disc_axes := std.Turned(std.origin, pin)
  }
  disc := Disc(disc_axes)
}
