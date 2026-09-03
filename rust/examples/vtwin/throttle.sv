// The throttle barrel, designed in one place: three views (§6.7).
//
// Along its axis: the barrel in its bore, the cross-hole `phi` out of line with the passage,
// the hub and the lever on the front.  From the side: its length — the hub proud of the boss's
// face, the body through the boss and `tback` past it, the cross-hole at the boss's mid-plane,
// an O-ring groove either side of it sealing the barrel in its bore, and a third groove behind
// the boss for the O-ring that retains it (a soft circlip: it stands proud of the barrel and
// bears on the boss's back).  From above: the same, the cross-hole seen into.  The assembly
// draws it in the boss, turned to `phi`; the part sheet draws it upright, full open.  Its
// outline is `class barrel`, so a sheet showing it inside the boss dashes it and its own sheet
// draws it solid.  Print it lever down, or on its side.

use vtwin.dims
use vtwin.parts

component Throttle(front: plane, side: plane, top: plane, c: point, ref: line, phi: Angle,
                   o_s: point, o_t: point, draw_side: Int, draw_top: Int) {
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
  }

  // -- from the side: `o_s` is on the barrel's axis at the cross-hole; the front is to the left --
  repeat draw_side {
    in side {
      point c_s hint(x: o_s.x - bossz / 2 - levw / 2, y: o_s.y)
      point tip_s hint(x: o_s.x - bossz / 2 - levw / 2, y: o_s.y + lev * cos(phi))
      o_s distance(-bossz / 2 - levw / 2, along: x) c_s
      o_s distance(-bossz / 2 - levw / 2, along: x) tip_s
      body: Box(o_s, x0: -bossz / 2, y0: -rbar, x1: bossz / 2 + tback, y1: rbar) class barrel
      hub_s: Box(o_s, x0: -bossz / 2 - levw, y0: -hubr, x1: -bossz / 2, y1: hubr) class lever
      arm: Slab(o_s, x0: -bossz / 2 - levw, x1: -bossz / 2, top: tip_s, bottom: c_s) class lever
      hole_s: Box(o_s, x0: -dhole / 2, y0: -rbar * cos(phi), x1: dhole / 2, y1: rbar * cos(phi)) class hidden
      seal0: Box(o_s, x0: -torz - torw / 2, y0: -torgb / 2, x1: -torz + torw / 2, y1: torgb / 2) class barrel
      seal1: Box(o_s, x0: torz - torw / 2, y0: -torgb / 2, x1: torz + torw / 2, y1: torgb / 2) class barrel
      keep: Box(o_s, x0: bossz / 2 + tretain - torw / 2, y0: -torgb / 2, x1: bossz / 2 + tretain + torw / 2, y1: torgb / 2) class barrel
      claim hub_s.a distance(bossz + levw + tback, along: x) body.b class detail at (0, -9)
      claim seal0.b distance(2 * torz - torw) seal1.a class detail at (0, 8)
      claim seal0.a distance(torw) seal0.b class detail at (0, -7)
      claim seal0.a distance(torgb, along: y) seal0.d class detail at (0, 5)
      claim keep.b distance(tback - tretain - torw / 2, along: x) body.b class detail at (0, -7)
    }
    c project c_s
    tip project tip_s
  }

  // -- from above: the cross-hole seen into, the grooves across ----------------------------------
  repeat draw_top {
    in top {
      body_t: Box(o_t, x0: -rbar, y0: -bossz / 2, x1: rbar, y1: bossz / 2 + tback) class barrel
      hub_t: Box(o_t, x0: -hubr, y0: -bossz / 2 - levw, x1: hubr, y1: -bossz / 2) class lever
      circle hole_t(center: o_t) hint(r: dhole / 2)
      radius(dhole / 2) hole_t
      seal0_t: Box(o_t, x0: -torgb / 2, y0: -torz - torw / 2, x1: torgb / 2, y1: -torz + torw / 2) class barrel
      seal1_t: Box(o_t, x0: -torgb / 2, y0: torz - torw / 2, x1: torgb / 2, y1: torz + torw / 2) class barrel
      keep_t: Box(o_t, x0: -torgb / 2, y0: bossz / 2 + tretain - torw / 2, x1: torgb / 2, y1: bossz / 2 + tretain + torw / 2) class barrel
    }
  }
}
