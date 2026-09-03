// The side view: the engine seen from its right, across the crank axis.  The plate edge on with
// the foot behind it; the crank train stacked along the shaft — the disc and the pin in front
// of the plate with the two rods side by side on the pin, the two bearings in the boss behind
// it, the flywheel behind that; a pivot stud with its spring and nut; the cylinders, each a slab
// as deep as its body; and the inlet boss with the coupling, the passage, the throttle barrel
// and the plenum in section.
//
// Page-x here is depth: the plate's front face is the origin's own x, and what stands in front
// of it — the cylinders, the disc — lies to the left, toward the view it is projected from.
// Every height the front view designs is projected, never restated.

use vtwin.dims
use vtwin.parts

component SideView(o: point) {
  param mid = tp / 2

  // -- the plate and the foot, the plate's top and bottom the front view's ---------------------
  point ptop hint(x: o.x + mid, y: o.y + fy1)
  o distance(mid, along: x) ptop
  point pbot hint(x: o.x + mid, y: o.y + fy0)
  o distance(mid, along: x) pbot
  plate: Slab(o, x0: 0mm, x1: tp, top: ptop, bottom: pbot)
  foot: Box(o, x0: tp, y0: fy0, x1: tp + footd, y1: fy0 + footh)
  claim plate.a distance(tp) plate.b class shown
  claim foot.a distance(footd) foot.b class shown

  // -- the crank train along the shaft ---------------------------------------------------------
  bboss: Box(o, x0: tp, y0: -(rbrg + 3mm), x1: tp + boss, y1: rbrg + 3mm)
  brg1: Box(o, x0: tp + 1mm, y0: -rbrg, x1: tp + 1mm + wbrg, y1: rbrg) class hidden
  brg2: Box(o, x0: tp + 1mm + wbrg, y0: -rbrg, x1: tp + 1mm + 2 * wbrg, y1: rbrg) class hidden
  shaft: Box(o, x0: -(zdisc + tdisc - 3mm), y0: -rshaft, x1: zfw + wfw + 2mm, y1: rshaft)
  disc: Box(o, x0: -(zdisc + tdisc), y0: -rdisc, x1: -zdisc, y1: rdisc)
  flywheel: Box(o, x0: zfw, y0: -rfw, x1: zfw + wfw, y1: rfw)
  claim flywheel.a distance(wfw) flywheel.b class shown
  // the pin at the height the front view puts it, the two eyes on it
  point pin_s hint(x: o.x, y: o.y + R * cos(theta0))
  o distance(0, along: x) pin_s
  pin: Box(pin_s, x0: -(zB + rw / 2 + 2mm), y0: -rpin, x1: -(zdisc + 2mm), y1: rpin)
  eyeA: Box(pin_s, x0: -(zA + rw / 2), y0: -reye, x1: -(zA - rw / 2), y1: reye)
  eyeB: Box(pin_s, x0: -(zB + rw / 2), y0: -reye, x1: -(zB - rw / 2), y1: reye)
  claim eyeA.a distance(rw) eyeA.b class shown

  // -- a pivot stud: threaded into the cylinder's face wall, through the plate, the spring and
  // the nut behind pressing the cylinder's face to the plate.  Both pivots stand at one height
  // here, so one stands for both. ---------------------------------------------------------------
  point pv hint(x: o.x, y: o.y + H * cos(alphaR))
  o distance(0, along: x) pv
  stud: Box(pv, x0: -(fwA - 1mm), y0: -rstud, x1: tp + 18mm, y1: rstud) class hidden
  repeat 7 as i {
    zz: At(pv, dx: tp + 2mm * i, dy: 4.5mm * (1 - 2 * (i - 2 * floor(i / 2))))
  }
  repeat 6 as i {
    line coil(zz[i].p, zz[i + 1].p) class thin
  }
  nut: Box(pv, x0: tp + 12mm, y0: -5.5mm, x1: tp + 18mm, y1: 5.5mm)

  // -- the cylinders: bank B nearest, bank A behind it, each between the heights of its highest
  // and lowest corner in the front view ---------------------------------------------------------
  point cyB_top hint(x: o.x - tcylB / 2, y: o.y + 60mm)
  point cyB_bot hint(x: o.x - tcylB / 2, y: o.y + 5mm)
  point cyA_top hint(x: o.x - tcylA / 2, y: o.y + 60mm)
  point cyA_bot hint(x: o.x - tcylA / 2, y: o.y + 5mm)
  o distance(-tcylB / 2, along: x) cyB_top
  o distance(-tcylB / 2, along: x) cyB_bot
  o distance(-tcylA / 2, along: x) cyA_top
  o distance(-tcylA / 2, along: x) cyA_bot
  cylB: Slab(o, x0: -tcylB, x1: 0mm, top: cyB_top, bottom: cyB_bot)
  cylA: Slab(o, x0: -tcylA, x1: 0mm, top: cyA_top, bottom: cyA_bot) class hidden
  claim cylB.a distance(tcylB) cylB.b class shown

  // -- the inlet, in section on the plate's mid-plane ------------------------------------------
  boss: Box(o, x0: mid - bossz / 2, y0: fy1, x1: mid + bossz / 2, y1: bossh)
  cpl_in: Box(o, x0: mid - cpl / 2, y0: bossh - cplin, x1: mid + cpl / 2, y1: bossh) class hidden
  cpl_out: Box(o, x0: mid - cpl / 2, y0: bossh, x1: mid + cpl / 2, y1: bossh - cplin + cpll)
  passage: Box(o, x0: mid - wch / 2, y0: rpl + wch / 2, x1: mid + wch / 2, y1: bossh - cplin) class hidden
  plenum: Box(o, x0: mid - wch / 2, y0: rpl - wch / 2, x1: mid + wch / 2, y1: rpl + wch / 2) class hidden
  point T_s hint(x: o.x + mid - bossz / 2 - 2mm, y: o.y + Ty)
  o distance(mid - bossz / 2 - 2mm, along: x) T_s
  barrel: Box(T_s, x0: 2mm, y0: -rbar, x1: 2mm + bossz, y1: rbar) class hidden
  hub: Box(T_s, x0: -2mm, y0: -4mm, x1: 2mm, y1: 4mm)
  point tip_s hint(x: o.x + mid - bossz / 2 - 2mm, y: o.y + Ty + lev)
  o distance(mid - bossz / 2 - 2mm, along: x) tip_s
  lever: Slab(o, x0: mid - bossz / 2 - 4mm, x1: mid - bossz / 2, top: tip_s, bottom: T_s) class lever
  claim boss.a distance(bossz) boss.b class shown
}
