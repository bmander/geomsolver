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
use vtwin.dims
use vtwin.parts

component Flywheel(front: plane, o: point, ref: line) {
  in front {
    circle rim(center: o) hint(r: rfw)
    radius(rfw) rim class detail at (-2.1, 44)
    circle bore(center: o) hint(r: dhub / 2) class hidden detail
    radius(dhub / 2) bore class detail at (2.4, 12)
    point se hint(x: o.x + rfw, y: o.y)
    line ssa(o, se) class gone
    o distance(rfw) se
    ref angle(90deg, sense: cw) ssa
    gs: Grub(o, ssa, ref, dir: 0deg, rin: dhub / 2, rout: rfw)
  }

  // -- the solid: the section's faces swept, and the body their one rule (§6.9) ----------------
  solid plate(face(rim), from: -wfw / 2, to: wfw / 2)
  solid hub(face(bore), from: -wfw / 2, to: wfw / 2)
  solid body(plate)
  hub cut body
  gs.bore cut body
}
