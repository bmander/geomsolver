// The throttle's turned stock: one longitudinal half-profile, revolved about its axis.
// The hub and three O-ring grooves are steps in that profile. The lever and knob are
// added at the front, and the transverse passage is cut through the barrel afterward.
// The end-on sketch remains the assembly drawing and carries the part's dimensions.

use components.dims
use components.parts

component Throttle(front: plane, c: point, ref: line, phi: Angle) {
  plane lever_axes(origin: c, toward: tip)
  plane hole_axes(origin: c, toward: cx)
  param hd = sqrt(rbar^2 - (dhole / 2)^2)

  in front {
    circle barrel(center: c) hint(r: rbar)
    radius(rbar) barrel
    point tip hint(x: c.x + lev * sin(phi), y: c.y + lev * cos(phi))
    line lever(c, tip)
    c distance(lev) tip
    ref angle(phi, sense: cw) lever
    circle knob(center: tip) hint(r: 2.5mm)
    radius(2.5) knob
    circle hub(center: c) hint(r: hubr)
    radius(hubr) hub
    claim lever angle(phi) ref
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
    line h0(e0, e1)
    line h1(e2, e3)
    claim radius(rbar) barrel
    claim e0 distance(dhole) e2
    claim c distance(lev) tip
    claim radius(hubr) hub

    // -- what the solid is made of --------------------------------------------------------
    // **The lever's own frame**: its line, and one square to it through the barrel's centre.
    // `Loc` places a point by two distances, so the pair it is given must be perpendicular —
    // `ref` is the sheet's own axis and is only square to the lever at full open.  That second
    // line is also the *cross-hole's axis*, since the hole runs across the lever: `h0` and `h1`
    // above are two chords parallel to the lever, half a hole either side of it.
    point cx hint(x: c.x + rbar * cos(phi), y: c.y - rbar * sin(phi))
    line hax(c, cx)
    hax perpendicular lever
    c distance(rbar) cx
    // the lever, `levw` wide — the same number as its thickness — so `lever` stays the
    // centreline the angle is measured on and these two flanks are what the material is
    lv0: Loc(lever_axes, u: 0mm, v: levw / 2)
    lv1: Loc(lever_axes, u: lev, v: levw / 2)
    lv2: Loc(lever_axes, u: lev, v: -levw / 2)
    lv3: Loc(lever_axes, u: 0mm, v: -levw / 2)
    line lv_a(lv0.p, lv1.p)
    line lv_c(lv2.p, lv3.p)
    // the hole: half of its section, on one side of the axis it is turned about
    x0: Loc(hole_axes, u: -rbar, v: 0mm)
    x1: Loc(hole_axes, u: -rbar, v: dhole / 2)
    x2: Loc(hole_axes, u: rbar, v: dhole / 2)
    x3: Loc(hole_axes, u: rbar, v: 0mm)
  }

  // Both placements put c on the front view's vertical datum. The longitudinal
  // plane contains that datum and the barrel axis; projection carries c's height.
  in front {
    point datum_up hint(x: front.origin.x, y: front.origin.y + 1mm)
    line datum(front.origin, datum_up)
    line front_u(front.origin, front.toward)
    datum perpendicular front_u
    front.origin distance(1mm) datum_up
    claim c on datum
  }
  // Share the front view's upright page frame: radius runs right, axial z down.
  plane longitudinal(origin: front.origin, toward: front.toward, from: front, fold: -90deg)
  line fold(front.origin, front.toward)
  param zback = -(bossz / 2 + tback)
  param zkeep = -(bossz / 2 + tretain)
  in longitudinal {
    point section_center hint(x: front.origin.x - (c.y - front.origin.y), y: front.origin.y)
    section_center on fold
    section_center project c
    back_axis: At(section_center, dx: 0mm, dy: -zback)
    back: At(section_center, dx: rbar, dy: -zback)
    retain0: At(section_center, dx: rbar, dy: -zkeep + torw / 2)
    retain1: At(section_center, dx: torgb / 2, dy: -zkeep + torw / 2)
    retain2: At(section_center, dx: torgb / 2, dy: -zkeep - torw / 2)
    retain3: At(section_center, dx: rbar, dy: -zkeep - torw / 2)
    seal0: At(section_center, dx: rbar, dy: torz + torw / 2)
    seal1: At(section_center, dx: torgb / 2, dy: torz + torw / 2)
    seal2: At(section_center, dx: torgb / 2, dy: torz - torw / 2)
    seal3: At(section_center, dx: rbar, dy: torz - torw / 2)
    seal4: At(section_center, dx: rbar, dy: -torz + torw / 2)
    seal5: At(section_center, dx: torgb / 2, dy: -torz + torw / 2)
    seal6: At(section_center, dx: torgb / 2, dy: -torz - torw / 2)
    seal7: At(section_center, dx: rbar, dy: -torz - torw / 2)
    shoulder: At(section_center, dx: rbar, dy: -bossz / 2)
    hub_back: At(section_center, dx: hubr, dy: -bossz / 2)
    hub_front: At(section_center, dx: hubr, dy: -(bossz / 2 + levw))
    front_axis: At(section_center, dx: 0mm, dy: -(bossz / 2 + levw))
    line axis(back_axis.p, front_axis.p)
    face profile(back_axis.p, back.p, retain0.p, retain1.p, retain2.p, retain3.p,
                 seal0.p, seal1.p, seal2.p, seal3.p, seal4.p, seal5.p, seal6.p, seal7.p,
                 shoulder.p, hub_back.p, hub_front.p, front_axis.p, axis)
  }

  solid turned(profile, about: axis)
  solid arm(face(lv_a, lv2.p, lv_c, lv0.p), from: bossz / 2, to: bossz / 2 + levw)
  solid knob_s(face(knob), from: bossz / 2, to: bossz / 2 + levw)
  solid cross(face(x0.p, x1.p, x2.p, x3.p, -> close), about: hax)
  solid body(turned)
  arm on body
  knob_s on body
  cross cut body
}
