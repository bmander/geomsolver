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
// Every height the front view designs is projected, never restated: the part is given the points
// whose heights it shows, and ties each of its own to one, so nothing outside places it.

use std (vertical)
use components.dims (D, H, R, alphaR, boltH, boltaf, boss, brgpocket, fwA, fwB, nutH, pingrip,
                     pinhead, pinheadH, pinpocket, rbrg, rdisc, reye, rfw, rpin, rshaft, rstud, rw,
                     spring, tdisc, tp, trapz, wall, wbrg, wfw, wsh, zA, zB, zdisc, zfw)
use components.parts

// What the side view reads of a cylinder the front view designs: the highest and lowest corners
// of its outline, and of its bore.
interface Span { top: point, bottom: point, bore_top: point, bore_bottom: point }

// `o` is the crank axis, edge on; `crank_pin` and `pivot` the crank pin and a cylinder's pivot in
// the view along the axis, and `near` and `far` the two cylinders there, bank B's nearest.  It is
// drawn in the plane `o` is drawn in.
part SideView(o: point, crank_pin: point, pivot: point, near: Span, far: Span) {
  tcylA := fwA + D + wall
  tcylB := fwB + D + wall
  mid := tp / 2

  in o.plane {
    // -- the crank train along the shaft ---------------------------------------------------------
    brg1 := components.parts.Box(o, x0: tp + boss - brgpocket, y0: -rbrg, x1: tp + boss - brgpocket + wbrg, y1: rbrg)
    brg2 := components.parts.Box(o, x0: tp + boss - brgpocket + wbrg, y0: -rbrg, x1: tp + boss - brgpocket + 2 * wbrg, y1: rbrg)
    shaft := components.parts.Box(o, x0: -(zdisc + tdisc - 3mm), y0: -rshaft, x1: zfw + wfw + 2mm, y1: rshaft)
    disc := components.parts.Box(o, x0: -(zdisc + tdisc), y0: -rdisc, x1: -zdisc, y1: rdisc)
    flywheel := components.parts.Box(o, x0: zfw, y0: -rfw, x1: zfw + wfw, y1: rfw)
    claim flywheel.a distance(wfw) flywheel.b
    // the clevis pin at the height the front view puts it: its head in the disc's back, the two
    // eyes on its shank, a washer each side of them, the cotter outboard
    pin_s := point hint((o.x, o.y + R))
    o vertical pin_s
    crank_pin project pin_s
    head := components.parts.Box(pin_s, x0: -(zdisc + pinpocket), y0: -pinhead / 2, x1: -(zdisc + pinpocket - pinheadH), y1: pinhead / 2)
    pin := components.parts.Box(pin_s, x0: -(zdisc + pinpocket + pingrip + 7mm), y0: -rpin, x1: -(zdisc + pinpocket), y1: rpin)
    w1 := components.parts.Box(pin_s, x0: -(zA - rw / 2), y0: -reye, x1: -(zA - rw / 2 - wsh), y1: reye)
    eyeA := components.parts.Box(pin_s, x0: -(zA + rw / 2), y0: -reye, x1: -(zA - rw / 2), y1: reye)
    eyeB := components.parts.Box(pin_s, x0: -(zB + rw / 2), y0: -reye, x1: -(zB - rw / 2), y1: reye)
    w2 := components.parts.Box(pin_s, x0: -(zB + rw / 2 + wsh), y0: -reye, x1: -(zB + rw / 2), y1: reye)
    cotter := components.parts.Box(pin_s, x0: -(zdisc + pinpocket + pingrip + 1mm), y0: -reye, x1: -(zdisc + pinpocket + pingrip), y1: reye)
    claim eyeA.a distance(rw) eyeA.b

    // -- a pivot bolt's stack, at the pivots' one height ---------------------------------------
    pv := point hint((o.x, o.y + H * cos(alphaR)))
    o vertical pv
    pivot project pv
    bhead := components.parts.Box(pv, x0: -(trapz + boltH), y0: -boltaf / 2, x1: -trapz, y1: boltaf / 2)
    bshank := components.parts.Box(pv, x0: -trapz, y0: -rstud, x1: tp + spring + wsh + nutH + 2mm, y1: rstud)
    repeat 7 as i {
      zz := components.parts.At(pv, dx: tp + spring / 6 * i, dy: 4.5mm * (1 - 2 * (i - 2 * floor(i / 2))))
    }
    repeat 6 as i {
      coil := line(zz[i].p, zz[i + 1].p)
    }
    wsh_s := components.parts.Box(pv, x0: tp + spring, y0: -6.35mm, x1: tp + spring + wsh, y1: 6.35mm)
    nut := components.parts.Box(pv, x0: tp + spring + wsh, y0: -boltaf / 2, x1: tp + spring + wsh + nutH, y1: boltaf / 2)

    // -- the cylinders: bank B nearest, bank A behind it, each between the heights of its highest
    // and lowest corner in the front view; and each one's bore, which is what shows the two
    // cylinders are two parts — B's face wall is a rod thicker than A's, so its bore stands a rod
    // further from the plate, over rod B where it rides the pin beside rod A -------------------
    cyB_top := point hint((o.x - tcylB / 2, o.y + 60mm))
    cyB_bot := point hint((o.x - tcylB / 2, o.y + 5mm))
    cyA_top := point hint((o.x - tcylA / 2, o.y + 60mm))
    cyA_bot := point hint((o.x - tcylA / 2, o.y + 5mm))
    o distance(-tcylB / 2, along: u) cyB_top
    o distance(-tcylB / 2, along: u) cyB_bot
    o distance(-tcylA / 2, along: u) cyA_top
    o distance(-tcylA / 2, along: u) cyA_bot
    near.top project cyB_top
    near.bottom project cyB_bot
    far.top project cyA_top
    far.bottom project cyA_bot
    cylB := components.parts.Slab(o, x0: -tcylB, x1: 0mm, top: cyB_top, bottom: cyB_bot)
    cylA := components.parts.Slab(o, x0: -tcylA, x1: 0mm, top: cyA_top, bottom: cyA_bot)
    boB_top := point hint((o.x - zB, o.y + 55mm))
    boB_bot := point hint((o.x - zB, o.y + 8mm))
    boA_top := point hint((o.x - zA, o.y + 55mm))
    boA_bot := point hint((o.x - zA, o.y + 8mm))
    o distance(-zB, along: u) boB_top
    o distance(-zB, along: u) boB_bot
    o distance(-zA, along: u) boA_top
    o distance(-zA, along: u) boA_bot
    near.bore_top project boB_top
    near.bore_bottom project boB_bot
    far.bore_top project boA_top
    far.bore_bottom project boA_bot
    boreB := components.parts.Slab(o, x0: -(fwB + D), x1: -fwB, top: boB_top, bottom: boB_bot)
    boreA := components.parts.Slab(o, x0: -(fwA + D), x1: -fwA, top: boA_top, bottom: boA_bot)
    claim cylB.a distance(tcylB) cylB.b
    claim boreA.b distance(fwA, along: u) cylA.b
    claim boreB.b distance(fwB, along: u) cylB.b
  }
}

// Open this file to preview the hardware stack, the heights it is given standing in for the
// assembly's at `t = 0`, the pin straight up.
preview {
  unit mm
  in std.front {
    pin := point
    fix((0, 10)) pin
    pivot := point
    fix((0, 21.21)) pivot
    b_top := point
    fix((-2.3, 58.7)) b_top
    b_bottom := point
    fix((-5.1, 3.6)) b_bottom
    b_bore_top := point
    fix((-6.6, 50.8)) b_bore_top
    b_bore_bottom := point
    fix((-7.3, 8.3)) b_bore_bottom
    a_top := point
    fix((2.3, 58.7)) a_top
    a_bottom := point
    fix((5.1, 3.6)) a_bottom
    a_bore_top := point
    fix((6.6, 50.8)) a_bore_top
    a_bore_bottom := point
    fix((7.3, 8.3)) a_bore_bottom
  }
  near := {top: b_top, bottom: b_bottom, bore_top: b_bore_top, bore_bottom: b_bore_bottom}
  far := {top: a_top, bottom: a_bottom, bore_top: a_bore_top, bore_bottom: a_bore_bottom}
  side := SideView(std.side.origin, pin, pivot, near, far)
}
