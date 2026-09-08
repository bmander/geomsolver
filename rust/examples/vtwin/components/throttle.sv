// The throttle's turned stock: one longitudinal half-profile, revolved about its axis.
// The hub and three O-ring grooves are steps in that profile. The lever and knob are
// added at the front, and the transverse passage is cut through the barrel afterward.
// The end-on sketch remains the assembly drawing and carries the part's dimensions.

use std
use components.dims
use components.parts

component Throttle(front: plane, c: point, ref: line, phi: Angle, dims: group) {
  plane lever_axes(origin: c, toward: tip)
  plane hole_axes(origin: c, toward: cx)
  param dhole = dims.wch   // the cross-hole matches the manifold passage
  param torgb = 2 * dims.rbar - 2 * (1 - dims.seal.squeeze) * dims.tor
  param torw = dims.seal.width_factor * dims.tor
  param hd = sqrt(dims.rbar^2 - (dhole / 2)^2)

  in front {
    circle barrel(center: c) hint(r: dims.rbar)
    radius(dims.rbar) barrel
    point tip hint(x: c.x + dims.lev * sin(phi), y: c.y + dims.lev * cos(phi))
    line lever(c, tip)
    c distance(dims.lev) tip
    ref angle(phi, sense: cw) lever
    circle knob(center: tip) hint(r: 2.5mm)
    radius(2.5) knob
    circle hub(center: c) hint(r: dims.hubr)
    radius(dims.hubr) hub
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
    claim radius(dims.rbar) barrel
    claim e0 distance(dhole) e2
    claim c distance(dims.lev) tip
    claim radius(dims.hubr) hub

    // -- what the solid is made of --------------------------------------------------------
    // **The lever's own frame**: its line, and one square to it through the barrel's centre.
    // `ref` is the sheet's own axis and is only square to the lever at full open. The second
    // line is also the *cross-hole's axis*, since the hole runs across the lever: `h0` and `h1`
    // above are two chords parallel to the lever, half a hole either side of it.
    point cx hint(x: c.x + dims.rbar * cos(phi), y: c.y - dims.rbar * sin(phi))
    line hax(c, cx)
    hax perpendicular lever
    c distance(dims.rbar) cx
    // the lever, `levw` wide — the same number as its thickness — so `lever` stays the
    // centreline the angle is measured on and these two flanks are what the material is
    point lv0 hint(x: lever_axes.origin.x + (0mm) * lever_axes.c - (dims.levw / 2) * lever_axes.s,
                      y: lever_axes.origin.y + (0mm) * lever_axes.s + (dims.levw / 2) * lever_axes.c)
    point lv1 hint(x: lever_axes.origin.x + (dims.lev) * lever_axes.c - (dims.levw / 2) * lever_axes.s,
                      y: lever_axes.origin.y + (dims.lev) * lever_axes.s + (dims.levw / 2) * lever_axes.c)
    point lv2 hint(x: lever_axes.origin.x + (dims.lev) * lever_axes.c - (-dims.levw / 2) * lever_axes.s,
                      y: lever_axes.origin.y + (dims.lev) * lever_axes.s + (-dims.levw / 2) * lever_axes.c)
    point lv3 hint(x: lever_axes.origin.x + (0mm) * lever_axes.c - (-dims.levw / 2) * lever_axes.s,
                      y: lever_axes.origin.y + (0mm) * lever_axes.s + (-dims.levw / 2) * lever_axes.c)
    line lv_a(lv0, lv1)
    line lv_c(lv2, lv3)
    line lever_start(lv0, lv3)
    line lever_end(lv1, lv2)
    c midpoint lever_start
    tip midpoint lever_end
    lever_end perpendicular lever
    lever_start equal lever_end
    lever_start perpendicular lever
    distance(dims.levw) lever_start
    // the hole: half of its section, on one side of the axis it is turned about
    point x0 hint(x: hole_axes.origin.x + (-dims.rbar) * hole_axes.c - (0mm) * hole_axes.s,
                      y: hole_axes.origin.y + (-dims.rbar) * hole_axes.s + (0mm) * hole_axes.c)
    point x1 hint(x: hole_axes.origin.x + (-dims.rbar) * hole_axes.c - (dhole / 2) * hole_axes.s,
                      y: hole_axes.origin.y + (-dims.rbar) * hole_axes.s + (dhole / 2) * hole_axes.c)
    point x2 hint(x: hole_axes.origin.x + (dims.rbar) * hole_axes.c - (dhole / 2) * hole_axes.s,
                      y: hole_axes.origin.y + (dims.rbar) * hole_axes.s + (dhole / 2) * hole_axes.c)
    point x3 hint(x: hole_axes.origin.x + (dims.rbar) * hole_axes.c - (0mm) * hole_axes.s,
                      y: hole_axes.origin.y + (dims.rbar) * hole_axes.s + (0mm) * hole_axes.c)
  }

  // The half-section's ends are square to its axis, spanning the barrel diameter.
  in front {
    x0 symmetry(lever) cx
    x3 coincident cx
    line entry(x0, x1)
    line wall(x1, x2)
    line exit(x2, x3)
    entry perpendicular hax
    wall parallel hax
    exit perpendicular hax
    distance(dhole / 2) entry
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
  param zback = -(dims.bossz / 2 + dims.tback)
  param zkeep = -(dims.bossz / 2 + dims.tretain)
  in longitudinal {
    point section_center hint(x: front.origin.x - (c.y - front.origin.y), y: front.origin.y)
    section_center on fold
    section_center project c
    back_axis: At(section_center, dx: 0mm, dy: -zback)
    back: At(section_center, dx: dims.rbar, dy: -zback)
    retain0: At(section_center, dx: dims.rbar, dy: -zkeep + torw / 2)
    retain1: At(section_center, dx: torgb / 2, dy: -zkeep + torw / 2)
    retain2: At(section_center, dx: torgb / 2, dy: -zkeep - torw / 2)
    retain3: At(section_center, dx: dims.rbar, dy: -zkeep - torw / 2)
    seal0: At(section_center, dx: dims.rbar, dy: dims.torz + torw / 2)
    seal1: At(section_center, dx: torgb / 2, dy: dims.torz + torw / 2)
    seal2: At(section_center, dx: torgb / 2, dy: dims.torz - torw / 2)
    seal3: At(section_center, dx: dims.rbar, dy: dims.torz - torw / 2)
    seal4: At(section_center, dx: dims.rbar, dy: -dims.torz + torw / 2)
    seal5: At(section_center, dx: torgb / 2, dy: -dims.torz + torw / 2)
    seal6: At(section_center, dx: torgb / 2, dy: -dims.torz - torw / 2)
    seal7: At(section_center, dx: dims.rbar, dy: -dims.torz - torw / 2)
    shoulder: At(section_center, dx: dims.rbar, dy: -dims.bossz / 2)
    hub_back: At(section_center, dx: dims.hubr, dy: -dims.bossz / 2)
    hub_front: At(section_center, dx: dims.hubr, dy: -(dims.bossz / 2 + dims.levw))
    front_axis: At(section_center, dx: 0mm, dy: -(dims.bossz / 2 + dims.levw))
    line axis(back_axis.p, front_axis.p)
    face profile(back_axis.p, back.p, retain0.p, retain1.p, retain2.p, retain3.p,
                 seal0.p, seal1.p, seal2.p, seal3.p, seal4.p, seal5.p, seal6.p, seal7.p,
                 shoulder.p, hub_back.p, hub_front.p, front_axis.p, axis)
  }

  solid turned(profile, about: axis)
  solid arm(face(lv_a, lv2, lv_c, lv0), from: dims.bossz / 2, to: dims.bossz / 2 + dims.levw)
  solid knob_s(face(knob), from: dims.bossz / 2, to: dims.bossz / 2 + dims.levw)
  solid cross(face(x0, x1, x2, x3, -> close), about: hax)
  solid body(turned)
  arm on body
  knob_s on body
  cross cut body
}

// Open this file to preview the throttle fully open.
// ../throttle.svd arranges three projections of this preview.
preview {
  unit mm
  line ref(std.origin, std.up.toward) in std.front
  thr: Throttle(std.front, std.origin, ref, phi: 0deg, dims: vtwin_dims)
}
