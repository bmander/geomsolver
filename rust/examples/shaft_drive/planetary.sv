// The bottom bracket's step-up: a helical planetary, carrier in, sun out, ring held.
//
// The crank spindle drives the carrier; the ring is pressed into the shell and held; the sun
// turns `1 + Zr / Zs` = 3 times as fast as the cranks, the same way, and carries the hypoid gear
// on its sleeve.  Three planets share the load, a third of a turn apart, which they can be because
// `(Zs + Zr) / 3` is whole: at that spacing every planet meets the sun and the ring in the same
// phase, so each is the first one moved, not turned (a third of a turn is 10 of the sun's pitches
// and 5 of a planet's).
//
// The sun is right-handed, the planets and the ring left-handed: external gears in mesh are of
// opposite hands, an internal gear and its pinion of the same.  The planets' thrust from the sun
// and from the ring are equal and opposite, so the planets run free of axial load; the sun's
// thrust is taken by its sleeve's bearing, the ring's by the shell.
//
// Drawn about its own origin, its axis its frame's `std.z` (toward the drive side in the
// assembly), its mid-plane its frame's `std.top`, and given no argument: closed, placed by a mate.
unit mm
use std
use std.solids
use std.solids (union)
use std.place (seats)
use gears
use dims (Zs, Zp, Zr, planets, mn, alpha, beta, pw, ha, hf, backlash, edge, a, ring_od,
          sun_bore, planet_bore, sleeve_r, sleeve_to, spindle_d, pin_d, plate_t, plate_r, gap,
          bridge_w, bridge_r)

component Planetary() {
  // the sun, its sleeve running on toward the drive side to carry the hypoid gear
  sun := gears.Helical(N: Zs, mn: mn, alpha: alpha, beta: beta, hand: 1, width: pw, ha: ha,
    hf: hf, backlash: backlash, bore: sun_bore, edge: edge, hub_r: sleeve_r, hub_from: 0mm,
    hub_to: sleeve_to)
  // the ring, shaped by a left-handed shaper of its own count
  ring := gears.Ring(N: Zr, mn: mn, alpha: alpha, beta: beta, hand: -1, width: pw, od: ring_od,
    ha: ha, hf: hf, backlash: backlash, edge: edge)

  // where the planets' pins stand, a third of a turn apart, the first on `std.x`; and between
  // each and the next, the carrier's bridge, a bar `bridge_w` wide on the bisector from just
  // outside the sun's tips to the plates' rim
  in std.top {
    ring planets about std.top.origin {
      pc := point
      std.top.origin distance(a) pc
      m := point
      std.top.origin distance(a) m
      line(pc, m) equal line(m, next.pc)
      ccw(std.top.origin, pc, m)
      ccw(std.top.origin, m, next.pc)
      construction cl := line(std.top.origin, m)
      b1 := point
      b2 := point
      b3 := point
      b4 := point
      b1 distance(bridge_w / 2, side: right) cl
      b2 distance(bridge_w / 2, side: right) cl
      b3 distance(bridge_w / 2, side: left) cl
      b4 distance(bridge_w / 2, side: left) cl
      std.top.origin distance(>= 0mm, along: cl) b1
      std.top.origin distance(>= 0mm, along: cl) b2
      std.top.origin distance(>= 0mm, along: cl) b3
      std.top.origin distance(>= 0mm, along: cl) b4
      side_r := line(b1, b2)
      outer := arc(center: std.top.origin, start: b2, end: b3)
      side_l := line(b3, b4)
      inner := arc(center: std.top.origin, start: b1, end: b4)
      radius(plate_r) outer
      radius(bridge_r) inner
      bridge := solid(std.solids.Prism(face(side_r, outer, side_l, inner),
        from: -pw / 2 - gap, to: pw / 2 + gap))
    }
    std.top.origin distance(0mm, along: v) pc[0]
    std.top.origin distance(>= 0mm, along: u) pc[0]

    rim := circle(center: std.top.origin)
    radius(plate_r) rim
    sleeve_bore := circle(center: std.top.origin)
    radius(sleeve_r + 1mm) sleeve_bore
    spindle_bore := circle(center: std.top.origin)
    radius(spindle_d / 2) spindle_bore
    repeat planets as k {
      pin_c := circle(center: pc[k])
      radius(pin_d / 2) pin_c
    }
  }

  // the planets, each on its pin, unturned
  repeat planets as k {
    planet := gears.Helical(N: Zp, mn: mn, alpha: alpha, beta: beta, hand: -1, width: pw, ha: ha,
      hf: hf, backlash: backlash, bore: planet_bore, edge: edge, hub_r: 0mm, hub_from: 0mm,
      hub_to: 0mm)
    planet seats(turn: 0deg, up: std.z, east: std.x) pc[k]
    pin := solid(std.solids.Prism(face(pin_c[k]), from: -pw / 2 - gap - plate_t,
      to: pw / 2 + gap + plate_t))
  }

  // the carrier: a plate either side of the planets, the drive side's open round the sun's
  // sleeve, the other's on the spindle, joined by the three bridges
  plate_drive := solid(std.solids.Prism(face(rim, holes: sleeve_bore, pin_c[0], pin_c[1], pin_c[2]),
    from: pw / 2 + gap, to: pw / 2 + gap + plate_t))
  plate_free := solid(std.solids.Prism(face(rim, holes: spindle_bore, pin_c[0], pin_c[1], pin_c[2]),
    from: -pw / 2 - gap - plate_t, to: -pw / 2 - gap))
  carrier := solid(plate_drive)
  plate_free union carrier
  repeat planets as k {
    bridge[k] union carrier
  }
  // what turns, for an assembly to read
  turns := {sun: sun.body, ring: ring.body}
}

preview {
  box := Planetary()
}
