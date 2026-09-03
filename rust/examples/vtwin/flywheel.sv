// The flywheel, designed in one place: three views (§6.7).  A plain disc on the shaft behind
// the bearing boss, bored for the 5/16" rod, with a #8-32 set screw in a trapped nut, the same
// arrangement as the crank disc's (`Grub`).  File a flat on the shaft for it.  It is drawn on
// its own sheet only: in the assembly it stands behind the plate, and the side view there draws
// its outline.

use std
use vtwin.dims
use vtwin.parts

component Flywheel(front: plane, side: plane, top: plane, o: point, ref: line,
                   o_s: point, o_t: point, draw_side: Int, draw_top: Int) {
  in front {
    circle rim(center: o) hint(r: rfw)
    radius(rfw) rim class detail at (-2.1, 44)
    circle bore(center: o) hint(r: dhub / 2) class hidden detail
    radius(dhub / 2) bore class detail at (2.4, 12)
    point se hint(x: o.x + rfw, y: o.y)
    line ssa(o, se) class gone
    o distance(rfw) se
    ref angle(-90deg) ssa
    gs: Grub(o, ssa, ref, 0deg, rin: dhub / 2, rout: rfw)
  }
  repeat draw_side {
    in side {
      body: Box(o_s, x0: -wfw, y0: -rfw, x1: 0mm, y1: rfw)
      bore_s: Box(o_s, x0: -wfw, y0: -dhub / 2, x1: 0mm, y1: dhub / 2) class hidden
      gc: At(o_s, dx: -wfw / 2, dy: 0mm)
      circle grub_s(center: gc.p) hint(r: grub / 2) class hidden
      radius(grub / 2) grub_s
      gv: At(gc.p, dx: 0mm, dy: 5mm)
      line gref(gc.p, gv.p) class gone
      nut_s: Hex(gc.p, gref, af: nutaf, phase: 0deg) class hidden
      claim body.a distance(wfw) body.b class detail at (0, -6)
    }
  }
  repeat draw_top {
    in top {
      body_t: Box(o_t, x0: -rfw, y0: -wfw, x1: rfw, y1: 0mm)
      bore_t: Box(o_t, x0: -dhub / 2, y0: -wfw, x1: dhub / 2, y1: 0mm) class hidden
      grub_t: Box(o_t, x0: dhub / 2, y0: -wfw / 2 - grub / 2, x1: rfw, y1: -wfw / 2 + grub / 2) class hidden
      nut_t: Box(o_t, x0: dhub / 2 + nutin, y0: -wfw / 2 - nutac / 2, x1: dhub / 2 + nutin + nutT, y1: -wfw / 2 + nutac / 2) class hidden
    }
  }
}
