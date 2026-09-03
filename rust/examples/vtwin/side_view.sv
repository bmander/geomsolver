// The side view: what the assembly adds beyond its parts, seen from the engine's right, across
// the crank axis.  The plate, the foot and the inlet are the frame part's own side view; here is
// the crank train stacked along the shaft — the disc with the clevis pin's head pocketed in its
// back, the two rod eyes on the pin's shank with a washer against the disc and one under the
// hairpin cotter, the two bearings in the boss behind the plate, the flywheel behind that; a
// pivot bolt's stack — its head trapped in the cylinder's face wall, the shank through the plate, the spring,
// the washer and the nut behind; and the cylinders, each a slab as deep as its body.
//
// Page-x here is depth: the plate's front face is the origin's own x, and what stands in front
// of it — the cylinders, the disc — lies to the left, toward the view it is projected from.
// Every height the front view designs is projected, never restated.

use vtwin.dims
use vtwin.parts

component SideView(o: point) {
  param mid = tp / 2

  // -- the crank train along the shaft ---------------------------------------------------------
  brg1: Box(o, x0: tp + boss - brgpocket, y0: -rbrg, x1: tp + boss - brgpocket + wbrg, y1: rbrg) class hidden
  brg2: Box(o, x0: tp + boss - brgpocket + wbrg, y0: -rbrg, x1: tp + boss - brgpocket + 2 * wbrg, y1: rbrg) class hidden
  shaft: Box(o, x0: -(zdisc + tdisc - 3mm), y0: -rshaft, x1: zfw + wfw + 2mm, y1: rshaft)
  disc: Box(o, x0: -(zdisc + tdisc), y0: -rdisc, x1: -zdisc, y1: rdisc)
  flywheel: Box(o, x0: zfw, y0: -rfw, x1: zfw + wfw, y1: rfw)
  claim flywheel.a distance(wfw) flywheel.b class shown
  // the clevis pin at the height the front view puts it: its head in the disc's back, the two
  // eyes on its shank, a washer each side of them, the cotter outboard
  point pin_s hint(x: o.x, y: o.y + R * cos(theta0))
  o distance(0, along: x) pin_s
  head: Box(pin_s, x0: -(zdisc + pinpocket), y0: -pinhead / 2, x1: -(zdisc + pinpocket - pinheadH), y1: pinhead / 2) class hidden
  pin: Box(pin_s, x0: -(zdisc + pinpocket + pingrip + 7mm), y0: -rpin, x1: -(zdisc + pinpocket), y1: rpin)
  w1: Box(pin_s, x0: -(zA - rw / 2), y0: -reye, x1: -(zA - rw / 2 - wsh), y1: reye) class thin
  eyeA: Box(pin_s, x0: -(zA + rw / 2), y0: -reye, x1: -(zA - rw / 2), y1: reye)
  eyeB: Box(pin_s, x0: -(zB + rw / 2), y0: -reye, x1: -(zB - rw / 2), y1: reye)
  w2: Box(pin_s, x0: -(zB + rw / 2 + wsh), y0: -reye, x1: -(zB + rw / 2), y1: reye) class thin
  cotter: Box(pin_s, x0: -(zdisc + pinpocket + pingrip + 1mm), y0: -reye, x1: -(zdisc + pinpocket + pingrip), y1: reye) class thin
  claim eyeA.a distance(rw) eyeA.b class shown

  // -- a pivot bolt's stack, at the pivots' one height ---------------------------------------
  point pv hint(x: o.x, y: o.y + H * cos(alphaR))
  o distance(0, along: x) pv
  bhead: Box(pv, x0: -(trapz + boltH), y0: -boltaf / 2, x1: -trapz, y1: boltaf / 2) class hidden
  bshank: Box(pv, x0: -trapz, y0: -rstud, x1: tp + spring + wsh + nutH + 2mm, y1: rstud) class hidden
  repeat 7 as i {
    zz: At(pv, dx: tp + spring / 6 * i, dy: 4.5mm * (1 - 2 * (i - 2 * floor(i / 2))))
  }
  repeat 6 as i {
    line coil(zz[i].p, zz[i + 1].p) class thin
  }
  wsh_s: Box(pv, x0: tp + spring, y0: -6.35mm, x1: tp + spring + wsh, y1: 6.35mm) class thin
  nut: Box(pv, x0: tp + spring + wsh, y0: -boltaf / 2, x1: tp + spring + wsh + nutH, y1: boltaf / 2)

  // -- the cylinders: bank B nearest, bank A behind it, each between the heights of its highest
  // and lowest corner in the front view; and each one's bore, which is what shows the two
  // cylinders are two parts — B's face wall is a rod thicker than A's, so its bore stands a rod
  // further from the plate, over rod B where it rides the pin beside rod A -------------------
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
  point boB_top hint(x: o.x - zB, y: o.y + 55mm)
  point boB_bot hint(x: o.x - zB, y: o.y + 8mm)
  point boA_top hint(x: o.x - zA, y: o.y + 55mm)
  point boA_bot hint(x: o.x - zA, y: o.y + 8mm)
  o distance(-zB, along: x) boB_top
  o distance(-zB, along: x) boB_bot
  o distance(-zA, along: x) boA_top
  o distance(-zA, along: x) boA_bot
  boreB: Slab(o, x0: -(fwB + D), x1: -fwB, top: boB_top, bottom: boB_bot) class hidden
  boreA: Slab(o, x0: -(fwA + D), x1: -fwA, top: boA_top, bottom: boA_bot) class hidden
  claim cylB.a distance(tcylB) cylB.b class shown
  claim boreA.b distance(fwA, along: x) cylA.b class shown at (0, -6)
  claim boreB.b distance(fwB, along: x) cylB.b class shown at (0, 6)
}
