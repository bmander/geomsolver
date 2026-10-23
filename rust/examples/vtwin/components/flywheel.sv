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
use components.dims (dhub, rfw, wfw)
use components.parts

// f is the flywheel's own frame: origin on the shaft, u along the set screw's reference.  The
// flywheel is drawn in it, so every number below is the part's own.
component Flywheel(f: group) {
  in f.axes {
    rim := circle(center: f.axes.origin) hint(r: rfw)
    radius(rfw) rim
    bore := circle(center: f.axes.origin) hint(r: dhub / 2)
    radius(dhub / 2) bore
    se := point hint((0mm, -rfw))
    ssa := line(f.axes.origin, se)
    se coincident rim
    ssa perpendicular f.axes.u
    se distance(>= 0, along: v) f.axes.origin   // square to `u`, on its right
    screw_axes := std.Turned(f.axes.origin, se)
    gs := components.parts.Grub(screw_axes, rin: dhub / 2, rout: rfw)
  }

  // -- the solid: the section's faces swept, and the body their one rule (§6.9) ----------------
  plate := solid(face(rim), from: -wfw / 2, to: wfw / 2)
  hub := solid(face(bore), from: -wfw / 2, to: wfw / 2)
  body := solid(plate)
  hub cut body
  gs.bore cut body
}

// Open this file to preview the flywheel; ../flywheel.svd arranges its three projections.
preview {
  unit mm
  in std.front {
    up := point
    fix((0, 40)) up
    axes := std.Turned(std.origin, up)
  }
  fw := Flywheel(axes)
}
