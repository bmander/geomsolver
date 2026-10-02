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
use components.dims
use components.parts

component Disc(f: plane, dims: group) {
  pinpocketd := dims.pinhead + 0.5mm   // diametral clearance around the clevis head
  rim := circle(center: f.origin) hint(r: dims.rdisc)
  radius(dims.rdisc) rim
  bore := circle(center: f.origin) hint(r: dims.dhub / 2)
  radius(dims.dhub / 2) bore
  ph := circle(center: f.toward) hint(r: dims.pinclr / 2)
  radius(dims.pinclr / 2) ph
  pkt := circle(center: f.toward) hint(r: pinpocketd / 2)
  radius(pinpocketd / 2) pkt
  // the set screw, square to the arm so its pocket stays clear of the pin's
  se := point hint(x: f.origin.x + (0mm) * f.c - (-dims.rdisc) * f.s,
                    y: f.origin.y + (0mm) * f.s + (-dims.rdisc) * f.c)
  ssa := line(f.origin, se)
  reference := line(f.origin, f.toward)
  se on rim
  ssa perpendicular reference
  screw_axes := plane(origin: f.origin, toward: se)
  gs := Grub(screw_axes, rin: dims.dhub / 2, rout: dims.rdisc, dims: dims)
  claim radius(dims.dhub / 2) bore
  claim radius(dims.pinclr / 2) ph
  claim radius(pinpocketd / 2) pkt

  // -- the solid: the section's faces swept, and the body their one rule (§6.9) ----------------
  // **The section is the disc's mid-plane**, and it is the set screw that says so: its hole is a
  // turn about a line lying in this plane, so the hole comes out centred on the plane whatever
  // else is written, and the screw runs down the middle of the thickness.  Drawn from the back
  // face instead, half the hole would have been in fresh air.  So the material is half a
  // thickness either way, and the pin's head sits in a pocket `pinpocket` deep in the back —
  // the face toward the plate, which a view from the right sees on its own right.
  // Each circle is a loop by itself.
  plate := solid(face(rim), from: -dims.tdisc / 2, to: dims.tdisc / 2)
  hub := solid(face(bore), from: -dims.tdisc / 2, to: dims.tdisc / 2)
  pinhole := solid(face(ph), from: -dims.tdisc / 2, to: dims.tdisc / 2)
  pinpkt := solid(face(pkt), from: -dims.tdisc / 2, to: -dims.tdisc / 2 + dims.pinpocket)
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
  pin := point hint(x: std.up.origin.x + (R) * std.up.c - (0mm) * std.up.s,
                    y: std.up.origin.y + (R) * std.up.s + (0mm) * std.up.c)
  std.origin vertical pin
  std.origin distance(R) pin
  disc_axes := plane(origin: std.origin, toward: pin)
  disc := Disc(disc_axes, dims: vtwin_dims)
}
