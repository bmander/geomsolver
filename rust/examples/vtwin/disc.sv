// The crank disc, designed in one place: three views (§6.7).
//
// Along the axis: the disc, its bore for the 5/16" shaft, the hole for the clevis pin `R` out,
// the pocket in its back the pin's head sits in, and the #8-32 set screw square to the crank
// arm, in a hole from the rim to the bore with its nut trapped in a pocket.  From the side and
// from above: its thickness, and the pocket and the screw edge on.  The assembly's crank
// instances it turned to the pin; the part sheet instances it once, the pin at the top.  The
// crank pin is a 1/4" × 1-1/4" clevis pin: its head in the pocket behind, the two rod eyes on
// its shank with a washer against the disc and one under the hairpin cotter, so the pin is
// captured between its head and the cotter and nothing is threaded into plastic.

use std
use vtwin.dims
use vtwin.parts

component Disc(swing: plane, side: plane, top: plane, o: point, pin: point, arm: line,
               dir: Angle, o_s: point, o_t: point, draw_side: Int, draw_top: Int) {
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
    arm angle(-90deg) ssa
    gs: Grub(o, ssa, arm, dir: dir - 90deg, rin: dhub / 2, rout: rdisc)
    claim radius(dhub / 2) bore class detail at (2.4, 12)
    claim radius(pinclr / 2) ph class detail at (0.8, 10)
    claim radius(pinpocketd / 2) pkt class detail at (1.4, 14)
  }

  // -- from the side: the back face toward the plate at `o_s`, the front to the left ----------
  repeat draw_side {
    in side {
      point pin_s hint(x: o_s.x - tdisc / 2, y: o_s.y + R)
      o_s distance(-tdisc / 2, along: x) pin_s
      body: Box(o_s, x0: -tdisc, y0: -rdisc, x1: 0mm, y1: rdisc)
      bore_s: Box(o_s, x0: -tdisc, y0: -dhub / 2, x1: 0mm, y1: dhub / 2) class hidden
      ph_s: Box(pin_s, x0: -tdisc / 2, y0: -pinclr / 2, x1: tdisc / 2, y1: pinclr / 2) class hidden
      pkt_s: Box(pin_s, x0: tdisc / 2 - pinpocket, y0: -pinpocketd / 2, x1: tdisc / 2, y1: pinpocketd / 2) class hidden
      // the set screw end on at mid-thickness, its nut's hex face behind it
      gc: At(o_s, dx: -tdisc / 2, dy: 0mm)
      circle grub_s(center: gc.p) hint(r: grub / 2) class hidden
      radius(grub / 2) grub_s
      gv: At(gc.p, dx: 0mm, dy: 5mm)
      line gref(gc.p, gv.p) class gone
      nut_s: Hex(gc.p, gref, af: nutaf, phase: 0deg) class hidden
      claim body.a distance(tdisc) body.b class detail at (0, -6)
      claim pkt_s.a distance(pinpocket) pkt_s.b class detail at (0, 8)
    }
    pin project pin_s
  }

  // -- from above: the screw's hole and the nut's pocket, seen along the pin -------------------
  repeat draw_top {
    in top {
      body_t: Box(o_t, x0: -rdisc, y0: -tdisc, x1: rdisc, y1: 0mm)
      bore_t: Box(o_t, x0: -dhub / 2, y0: -tdisc, x1: dhub / 2, y1: 0mm) class hidden
      grub_t: Box(o_t, x0: dhub / 2, y0: -tdisc / 2 - grub / 2, x1: rdisc, y1: -tdisc / 2 + grub / 2) class hidden
      nut_t: Box(o_t, x0: dhub / 2 + nutin, y0: -tdisc / 2 - nutac / 2, x1: dhub / 2 + nutin + nutT, y1: -tdisc / 2 + nutac / 2) class hidden
    }
  }
}
