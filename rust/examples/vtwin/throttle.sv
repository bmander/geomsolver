// The throttle barrel: **one section, and the solid it is a section of** (§6.9).
//
// The section is drawn along the barrel's own axis — the barrel, the cross-hole `phi` out of
// line with the passage, the hub and the lever on the front — and every one of those is that
// section swept along the axis, so the lengths that used to be written twice more from the side
// and from above are written once here.  The axis runs *through* this plane rather than lying in
// it, which decides the shape of every statement below: the barrel, the hub, the lever and the
// grooves are prisms along it, and the cross-hole, whose axis lies in the plane, is a turn.
//
// An O-ring groove is a ring less its core, so it is written as the two solids it is made of
// with the ring named — `groove0` — exactly as §6.9 says a design that needs an order should:
// there are two things there, and naming the intermediate is the honest way to say so.
//
// Along its axis: the hub proud of the boss's face, the body through the boss and `tback` past
// it, the cross-hole at the boss's mid-plane, an O-ring groove either side of it sealing the
// barrel in its bore, and a third groove behind the boss for the O-ring that retains it (a soft
// circlip: it stands proud of the barrel and bears on the boss's back).  The assembly
// draws it in the boss, turned to `phi`; the part sheet draws it upright, full open.  Its
// outline is `class barrel`, so a sheet showing it inside the boss dashes it and its own sheet
// draws it solid.  Print it lever down, or on its side.

use vtwin.dims
use vtwin.parts

component Throttle(front: plane, c: point, ref: line, phi: Angle) {
  param hd = sqrt(rbar^2 - (dhole / 2)^2)

  in front {
    circle barrel(center: c) hint(r: rbar) class barrel
    radius(rbar) barrel
    point tip hint(x: c.x + lev * sin(phi), y: c.y + lev * cos(phi))
    line lever(c, tip) class lever
    c distance(lev) tip
    ref angle(phi, sense: cw) lever
    circle knob(center: tip) hint(r: 2.5mm) class lever
    radius(2.5) knob
    circle hub(center: c) hint(r: hubr) class lever
    radius(hubr) hub
    claim lever angle(phi) ref class shown at (0.3, 14)
    // the cross-hole: two chords of the barrel, half a hole either side of the lever's line
    point e0 hint(x: c.x - dhole / 2 * cos(phi) + hd * sin(phi), y: c.y + dhole / 2 * sin(phi) + hd * cos(phi))
    point e1 hint(x: c.x - dhole / 2 * cos(phi) - hd * sin(phi), y: c.y + dhole / 2 * sin(phi) - hd * cos(phi))
    point e2 hint(x: c.x + dhole / 2 * cos(phi) + hd * sin(phi), y: c.y - dhole / 2 * sin(phi) + hd * cos(phi))
    point e3 hint(x: c.x + dhole / 2 * cos(phi) - hd * sin(phi), y: c.y - dhole / 2 * sin(phi) - hd * cos(phi))
    e0 on barrel
    e1 on barrel
    e2 on barrel
    e3 on barrel
    e0 distance(dhole / 2, side: left) lever
    e1 distance(dhole / 2, side: left) lever
    e2 distance(dhole / 2, side: right) lever
    e3 distance(dhole / 2, side: right) lever
    line h0(e0, e1) class barrel
    line h1(e2, e3) class barrel
    claim radius(rbar) barrel class detail at (-2.4, 12)
    claim e0 distance(dhole) e2 class detail at (0, 9)
    claim c distance(lev) tip class detail at (0, 8)
    claim radius(hubr) hub class detail at (2.6, 9)

    // -- what the solid is made of --------------------------------------------------------
    // **The lever's own frame**: its line, and one square to it through the barrel's centre.
    // `Loc` places a point by two distances, so the pair it is given must be perpendicular —
    // `ref` is the sheet's own axis and is only square to the lever at full open.  That second
    // line is also the *cross-hole's axis*, since the hole runs across the lever: `h0` and `h1`
    // above are two chords parallel to the lever, half a hole either side of it.
    point cx hint(x: c.x + rbar * cos(phi), y: c.y - rbar * sin(phi))
    line hax(c, cx) class gone
    hax perpendicular lever
    c distance(rbar) cx
    // the lever, `levw` wide — the same number as its thickness — so `lever` stays the
    // centreline the angle is measured on and these two flanks are what the material is
    lv0: Loc(c, lever, hax, dir: 90deg - phi, u: 0mm, v: levw / 2)
    lv1: Loc(c, lever, hax, dir: 90deg - phi, u: lev, v: levw / 2)
    lv2: Loc(c, lever, hax, dir: 90deg - phi, u: lev, v: -levw / 2)
    lv3: Loc(c, lever, hax, dir: 90deg - phi, u: 0mm, v: -levw / 2)
    line lv_a(lv0.p, lv1.p) class lever
    line lv_c(lv2.p, lv3.p) class lever
    // the hole: half of its section, on one side of the axis it is turned about
    x0: Loc(c, hax, lever, dir: -phi, u: -rbar, v: 0mm)
    x1: Loc(c, hax, lever, dir: -phi, u: -rbar, v: dhole / 2)
    x2: Loc(c, hax, lever, dir: -phi, u: rbar, v: dhole / 2)
    x3: Loc(c, hax, lever, dir: -phi, u: rbar, v: 0mm)
    // an O-ring groove's core, the one circle the three of them share
    circle core(center: c) hint(r: torgb / 2) class hidden
    radius(torgb / 2) core
    face barrel_f(barrel)
    face core_f(core)
  }

  // -- the solid: the section's faces swept, and the body their one rule (§6.9) ----------------
  // The axis runs from the hub, proud of the boss's face, back through the boss and `tback`
  // past it; the cross-hole sits at the boss's mid-plane, which is this section's own zero and
  // is where a turn about an in-plane line puts it with nothing said.
  param zback = -(bossz / 2 + tback)
  param zkeep = -(bossz / 2 + tretain)
  solid barrel_s(barrel_f, from: zback, to: bossz / 2)
  solid hub_s(face(hub), from: bossz / 2, to: bossz / 2 + levw)
  solid arm(face(lv_a, lv2.p, lv_c, lv0.p), from: bossz / 2, to: bossz / 2 + levw)
  solid knob_s(face(knob), from: bossz / 2, to: bossz / 2 + levw)
  solid cross(face(x0.p, x1.p, x2.p, x3.p, -> close), about: hax)
  // each seal groove: the ring the barrel would have there, less the core that is left standing
  solid ring0(barrel_f, from: torz - torw / 2, to: torz + torw / 2)
  solid ring1(barrel_f, from: -torz - torw / 2, to: -torz + torw / 2)
  solid ring2(barrel_f, from: zkeep - torw / 2, to: zkeep + torw / 2)
  solid core0(core_f, from: torz - torw / 2, to: torz + torw / 2)
  solid core1(core_f, from: -torz - torw / 2, to: -torz + torw / 2)
  solid core2(core_f, from: zkeep - torw / 2, to: zkeep + torw / 2)
  solid groove0(ring0)
  core0 through groove0
  solid groove1(ring1)
  core1 through groove1
  solid groove2(ring2)
  core2 through groove2

  solid body(barrel_s)
  hub_s on body
  arm on body
  knob_s on body
  cross through body
  groove0 through body
  groove1 through body
  groove2 through body
}
