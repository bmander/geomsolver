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

use components.dims
use components.parts

component SideView(o: point, dims: group) {
  param tcylA = dims.fwA + dims.D + dims.wall
  param tcylB = dims.fwB + dims.D + dims.wall
  param mid = dims.tp / 2

  // -- the crank train along the shaft ---------------------------------------------------------
  brg1: Box(o, x0: dims.tp + dims.boss - dims.brgpocket, y0: -dims.rbrg, x1: dims.tp + dims.boss - dims.brgpocket + dims.wbrg, y1: dims.rbrg)
  brg2: Box(o, x0: dims.tp + dims.boss - dims.brgpocket + dims.wbrg, y0: -dims.rbrg, x1: dims.tp + dims.boss - dims.brgpocket + 2 * dims.wbrg, y1: dims.rbrg)
  shaft: Box(o, x0: -(dims.zdisc + dims.tdisc - 3mm), y0: -dims.rshaft, x1: dims.zfw + dims.wfw + 2mm, y1: dims.rshaft)
  disc: Box(o, x0: -(dims.zdisc + dims.tdisc), y0: -dims.rdisc, x1: -dims.zdisc, y1: dims.rdisc)
  flywheel: Box(o, x0: dims.zfw, y0: -dims.rfw, x1: dims.zfw + dims.wfw, y1: dims.rfw)
  claim flywheel.a distance(dims.wfw) flywheel.b
  // the clevis pin at the height the front view puts it: its head in the disc's back, the two
  // eyes on its shank, a washer each side of them, the cotter outboard
  point pin_s hint(x: o.x, y: o.y + dims.R * cos(dims.theta0))
  o distance(0, along: x) pin_s
  head: Box(pin_s, x0: -(dims.zdisc + dims.pinpocket), y0: -dims.pinhead / 2, x1: -(dims.zdisc + dims.pinpocket - dims.pinheadH), y1: dims.pinhead / 2)
  pin: Box(pin_s, x0: -(dims.zdisc + dims.pinpocket + dims.pingrip + 7mm), y0: -dims.rpin, x1: -(dims.zdisc + dims.pinpocket), y1: dims.rpin)
  w1: Box(pin_s, x0: -(dims.zA - dims.rw / 2), y0: -dims.reye, x1: -(dims.zA - dims.rw / 2 - dims.wsh), y1: dims.reye)
  eyeA: Box(pin_s, x0: -(dims.zA + dims.rw / 2), y0: -dims.reye, x1: -(dims.zA - dims.rw / 2), y1: dims.reye)
  eyeB: Box(pin_s, x0: -(dims.zB + dims.rw / 2), y0: -dims.reye, x1: -(dims.zB - dims.rw / 2), y1: dims.reye)
  w2: Box(pin_s, x0: -(dims.zB + dims.rw / 2 + dims.wsh), y0: -dims.reye, x1: -(dims.zB + dims.rw / 2), y1: dims.reye)
  cotter: Box(pin_s, x0: -(dims.zdisc + dims.pinpocket + dims.pingrip + 1mm), y0: -dims.reye, x1: -(dims.zdisc + dims.pinpocket + dims.pingrip), y1: dims.reye)
  claim eyeA.a distance(dims.rw) eyeA.b

  // -- a pivot bolt's stack, at the pivots' one height ---------------------------------------
  point pv hint(x: o.x, y: o.y + dims.H * cos(dims.alphaR))
  o distance(0, along: x) pv
  bhead: Box(pv, x0: -(dims.trapz + dims.boltH), y0: -dims.boltaf / 2, x1: -dims.trapz, y1: dims.boltaf / 2)
  bshank: Box(pv, x0: -dims.trapz, y0: -dims.rstud, x1: dims.tp + dims.spring + dims.wsh + dims.nutH + 2mm, y1: dims.rstud)
  repeat 7 as i {
    zz: At(pv, dx: dims.tp + dims.spring / 6 * i, dy: 4.5mm * (1 - 2 * (i - 2 * floor(i / 2))))
  }
  repeat 6 as i {
    line coil(zz[i].p, zz[i + 1].p)
  }
  wsh_s: Box(pv, x0: dims.tp + dims.spring, y0: -6.35mm, x1: dims.tp + dims.spring + dims.wsh, y1: 6.35mm)
  nut: Box(pv, x0: dims.tp + dims.spring + dims.wsh, y0: -dims.boltaf / 2, x1: dims.tp + dims.spring + dims.wsh + dims.nutH, y1: dims.boltaf / 2)

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
  cylA: Slab(o, x0: -tcylA, x1: 0mm, top: cyA_top, bottom: cyA_bot)
  point boB_top hint(x: o.x - dims.zB, y: o.y + 55mm)
  point boB_bot hint(x: o.x - dims.zB, y: o.y + 8mm)
  point boA_top hint(x: o.x - dims.zA, y: o.y + 55mm)
  point boA_bot hint(x: o.x - dims.zA, y: o.y + 8mm)
  o distance(-dims.zB, along: x) boB_top
  o distance(-dims.zB, along: x) boB_bot
  o distance(-dims.zA, along: x) boA_top
  o distance(-dims.zA, along: x) boA_bot
  boreB: Slab(o, x0: -(dims.fwB + dims.D), x1: -dims.fwB, top: boB_top, bottom: boB_bot)
  boreA: Slab(o, x0: -(dims.fwA + dims.D), x1: -dims.fwA, top: boA_top, bottom: boA_bot)
  claim cylB.a distance(tcylB) cylB.b
  claim boreA.b distance(dims.fwA, along: x) cylA.b
  claim boreB.b distance(dims.fwB, along: x) cylB.b
}

// Open this file to preview the hardware stack. Heights supplied by the assembly's
// projections remain free here, starting at the component's hints.
preview {
  unit mm
  SideView(std.origin, dims: vtwin_dims)
}
