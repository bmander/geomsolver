// The flywheel: **one section, and the solid it is a section of** (§6.9).  A plain disc on the
// shaft behind the bearing boss, bored for the 5/16" rod, with a #8-32 set screw in a trapped
// nut, the same arrangement as the crank disc's (`Grub`).  File a flat on the shaft for it.
//
// The rim and the bore are that circle swept `wfw` along the shaft, and the views that show the
// thickness are asked for rather than drawn — so the one number that used to appear in three
// places appears in one.  As on the disc, the section is the flywheel's **mid-plane**, because
// the set screw's hole is a turn about a line lying in it.  (The nut's pocket is not part of the
// body: see `parts.Grub`.)  It is drawn on its own sheet only: in the assembly it stands behind
// the plate, and the side view there draws its outline.

use std
use components.dims
use components.parts

component Flywheel(f: plane, dims: group) {
  rim := circle(center: f.origin) hint(r: dims.rfw)
  radius(dims.rfw) rim
  bore := circle(center: f.origin) hint(r: dims.dhub / 2)
  radius(dims.dhub / 2) bore
  se := point hint(x: f.origin.x + (0mm) * f.c - (-dims.rfw) * f.s,
                    y: f.origin.y + (0mm) * f.s + (-dims.rfw) * f.c)
  ssa := line(f.origin, se)
  reference := line(f.origin, f.toward)
  se on rim
  ssa perpendicular reference
  screw_axes := plane(origin: f.origin, toward: se)
  gs := components.parts.Grub(screw_axes, rin: dims.dhub / 2, rout: dims.rfw, dims: dims)

  // -- the solid: the section's faces swept, and the body their one rule (§6.9) ----------------
  plate := solid(face(rim), from: -dims.wfw / 2, to: dims.wfw / 2)
  hub := solid(face(bore), from: -dims.wfw / 2, to: dims.wfw / 2)
  body := solid(plate)
  hub cut body
  gs.bore cut body
}

// Open this file to preview the flywheel; ../flywheel.svd arranges its three projections.
preview {
  unit mm
  fw := Flywheel(std.up, dims: components.dims.vtwin_dims)
}
