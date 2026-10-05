// The throttle's turned stock: one longitudinal half-profile, revolved about its axis.
// The hub and three O-ring grooves are steps in that profile. The lever and knob are
// added at the front, and the transverse passage is cut through the barrel afterward.
// The end-on sketch remains the assembly drawing and carries the part's dimensions.

use std
use components.dims
use components.parts

component Throttle(front: plane, c: point, ref: line, phi: Angle, dims: group) {
  lever_axes := std.Turned(c, tip) in front
  hole_axes := std.Turned(c, cx) in front
  dhole := dims.wch   // the cross-hole matches the manifold passage
  torgb := 2 * dims.rbar - 2 * (1 - dims.seal.squeeze) * dims.tor
  torw := dims.seal.width_factor * dims.tor
  hd := sqrt(dims.rbar^2 - (dhole / 2)^2)

  in front {
    barrel := circle(center: c) hint(r: dims.rbar)
    radius(dims.rbar) barrel
    tip := point hint(x: c.x + dims.lev * sin(phi), y: c.y + dims.lev * cos(phi))
    lever := line(c, tip)
    c distance(dims.lev) tip
    ref angle(phi, sense: cw) lever
    knob := circle(center: tip) hint(r: 2.5mm)
    radius(2.5) knob
    hub := circle(center: c) hint(r: dims.hubr)
    radius(dims.hubr) hub
    claim lever angle(phi) ref
    // the cross-hole: two chords of the barrel, half a hole either side of the lever's line
    e0 := point hint(x: c.x - dhole / 2 * cos(phi) + hd * sin(phi), y: c.y + dhole / 2 * sin(phi) + hd * cos(phi))
    e1 := point hint(x: c.x - dhole / 2 * cos(phi) - hd * sin(phi), y: c.y + dhole / 2 * sin(phi) - hd * cos(phi))
    e2 := point hint(x: c.x + dhole / 2 * cos(phi) + hd * sin(phi), y: c.y - dhole / 2 * sin(phi) + hd * cos(phi))
    e3 := point hint(x: c.x + dhole / 2 * cos(phi) - hd * sin(phi), y: c.y - dhole / 2 * sin(phi) - hd * cos(phi))
    e0 coincident barrel
    e1 coincident barrel
    e2 coincident barrel
    e3 coincident barrel
    e0 distance(dhole / 2, side: left) lever
    e1 distance(dhole / 2, side: left) lever
    e2 distance(dhole / 2, side: right) lever
    e3 distance(dhole / 2, side: right) lever
    h0 := line(e0, e1)
    h1 := line(e2, e3)
    claim radius(dims.rbar) barrel
    claim e0 distance(dhole) e2
    claim c distance(dims.lev) tip
    claim radius(dims.hubr) hub

    // -- what the solid is made of --------------------------------------------------------
    // **The lever's own frame**: its line, and one square to it through the barrel's centre.
    // `ref` is the sheet's own axis and is only square to the lever at full open. The second
    // line is also the *cross-hole's axis*, since the hole runs across the lever: `h0` and `h1`
    // above are two chords parallel to the lever, half a hole either side of it.
    cx := point hint(x: c.x + dims.rbar * cos(phi), y: c.y - dims.rbar * sin(phi))
    hax := line(c, cx)
    hax perpendicular lever
    c distance(dims.rbar) cx
    // the lever, `levw` wide — the same number as its thickness — so `lever` stays the
    // centreline the angle is measured on and these two flanks are what the material is
    lv0 := point hint(at: lever_axes.axes, x: 0mm, y: dims.levw / 2)
    lv1 := point hint(at: lever_axes.axes, x: dims.lev, y: dims.levw / 2)
    lv2 := point hint(at: lever_axes.axes, x: dims.lev, y: -dims.levw / 2)
    lv3 := point hint(at: lever_axes.axes, x: 0mm, y: -dims.levw / 2)
    lv_a := line(lv0, lv1)
    lv_c := line(lv2, lv3)
    lever_start := line(lv0, lv3)
    lever_end := line(lv1, lv2)
    c midpoint lever_start
    tip midpoint lever_end
    lever_end perpendicular lever
    lever_start equal lever_end
    lever_start perpendicular lever
    distance(dims.levw) lever_start
    // the hole: half of its section, on one side of the axis it is turned about
    x0 := point hint(at: hole_axes.axes, x: -dims.rbar, y: 0mm)
    x1 := point hint(at: hole_axes.axes, x: -dims.rbar, y: dhole / 2)
    x2 := point hint(at: hole_axes.axes, x: dims.rbar, y: dhole / 2)
    x3 := point hint(at: hole_axes.axes, x: dims.rbar, y: 0mm)
  }

  // The half-section's ends are square to its axis, spanning the barrel diameter.
  in front {
    x0 symmetry(lever) cx
    x3 coincident cx
    entry := line(x0, x1)
    wall := line(x1, x2)
    exit := line(x2, x3)
    entry perpendicular hax
    wall parallel hax
    exit perpendicular hax
    distance(dhole / 2) entry
  }
  // Both placements put c on the front view's vertical datum. The longitudinal
  // plane contains that datum and the barrel axis; projection carries c's height.
  in front {
    datum_up := point hint(x: 0mm, y: 1mm)
    datum := vertical line(front.origin, datum_up)
    front.origin distance(1mm) datum_up
    claim c coincident datum
  }
  // Square to the front along its vertical datum, through its origin: radius runs right, down
  // the front's up, and axial out of the front, into its depth.
  private down := axis hint(x: 0, y: 0, z: -1)
  down parallel front.v
  private out := axis hint(x: 0, y: 1, z: 0)
  out perpendicular front
  longitudinal := plane(u: down, v: out)
  front.origin coincident longitudinal.origin
  zback := -(dims.bossz / 2 + dims.tback)
  zkeep := -(dims.bossz / 2 + dims.tretain)
  in longitudinal {
    section_center := point hint(x: -c.y, y: 0mm)
    section_center distance(0mm, along: v) longitudinal
    section_center project c
    back_axis := components.parts.At(section_center, dx: 0mm, dy: -zback)
    back := components.parts.At(section_center, dx: dims.rbar, dy: -zback)
    retain0 := components.parts.At(section_center, dx: dims.rbar, dy: -zkeep + torw / 2)
    retain1 := components.parts.At(section_center, dx: torgb / 2, dy: -zkeep + torw / 2)
    retain2 := components.parts.At(section_center, dx: torgb / 2, dy: -zkeep - torw / 2)
    retain3 := components.parts.At(section_center, dx: dims.rbar, dy: -zkeep - torw / 2)
    seal0 := components.parts.At(section_center, dx: dims.rbar, dy: dims.torz + torw / 2)
    seal1 := components.parts.At(section_center, dx: torgb / 2, dy: dims.torz + torw / 2)
    seal2 := components.parts.At(section_center, dx: torgb / 2, dy: dims.torz - torw / 2)
    seal3 := components.parts.At(section_center, dx: dims.rbar, dy: dims.torz - torw / 2)
    seal4 := components.parts.At(section_center, dx: dims.rbar, dy: -dims.torz + torw / 2)
    seal5 := components.parts.At(section_center, dx: torgb / 2, dy: -dims.torz + torw / 2)
    seal6 := components.parts.At(section_center, dx: torgb / 2, dy: -dims.torz - torw / 2)
    seal7 := components.parts.At(section_center, dx: dims.rbar, dy: -dims.torz - torw / 2)
    shoulder := components.parts.At(section_center, dx: dims.rbar, dy: -dims.bossz / 2)
    hub_back := components.parts.At(section_center, dx: dims.hubr, dy: -dims.bossz / 2)
    hub_front := components.parts.At(section_center, dx: dims.hubr, dy: -(dims.bossz / 2 + dims.levw))
    front_axis := components.parts.At(section_center, dx: 0mm, dy: -(dims.bossz / 2 + dims.levw))
    ax := line(back_axis.p, front_axis.p)
    profile := face(back_axis.p, back.p, retain0.p, retain1.p, retain2.p, retain3.p,
                 seal0.p, seal1.p, seal2.p, seal3.p, seal4.p, seal5.p, seal6.p, seal7.p,
                 shoulder.p, hub_back.p, hub_front.p, front_axis.p, ax)
  }

  turned := solid(profile, about: ax)
  arm := solid(face(lv_a, lv2, lv_c, lv0), from: dims.bossz / 2, to: dims.bossz / 2 + dims.levw)
  knob_s := solid(face(knob), from: dims.bossz / 2, to: dims.bossz / 2 + dims.levw)
  cross := solid(face(x0, x1, x2, x3, -> close), about: hax)
  body := solid(turned)
  arm union body
  knob_s union body
  cross cut body
}

// Open this file to preview the throttle fully open.
// ../throttle.svd arranges three projections of this preview.
preview {
  unit mm
  in std.front {
    ref := line(std.origin, hint(x: 0, y: 1))
    fix(x == 0, y == 1) ref.p2
  }
  thr := Throttle(std.front, std.origin, ref, phi: 0deg, dims: components.dims.vtwin_dims)
}
