// Shaped spur gears, as a module: `ShapedSpur`, the radiator bot's gear (bmander/radiatorbot,
// hardware/gears.sv), kept here so the gear train (train.sv) is a case of the library.
//
// Every gear is **shaped**: its blank is cut by the basic rack's tooth swept through the roll of
// the rack on the gear's pitch circle, as a gear shaper or a hob cuts one, in one statement
// (`sweep(tooth, through: shaped_blank, under: rack, …)`): the rack rolls for as long as its tooth
// meets the blank, a roll the kernel finds, so none is stated.  Nothing states an involute; the
// flanks the sweep leaves are one, exactly, and a small pinion is undercut where a real one would
// be.  The rack's tooth is drawn `thin` wider than the basic rack's, so the tooth comes out that
// much thinner on its pitch circle: a pair's backlash is the two gears' `thin`s added.
//
// A gear's tooth spaces are one space turned a pitch at a time, so the kernel builds one sector
// and turns it; a spoked gear is shaped whole first and its windows cut after, since the windows
// do not turn by a pitch.
//
// A gear is drawn about its own std.origin, its axis std.z and its faces at z0 and z1, so it is
// placed by a mate and every copy of one size is one figure.  Its solid is construction: an
// assembly makes its parts of gears (a compound is two, joined), and shows those.
//
// Each face's edge is chamfered at 45deg, the teeth's tips a `chamfer` short of the tip circle at
// the face.  A large gear may be spoked: its web, between a hub and a rim under the roots, cut
// away but for `spokes` straight spokes.
unit mm
use std
use std.place (vertical, coords, on_v, raised, seats)
use std.motion (turns, slides)
use std.sets

// A shaped spur gear: N teeth of module m and pressure angle phi, its rack drawn `shift` modules
// out from the pitch circle (a profile shift; a pinion under 17 teeth wants about (17 - N) / 17 to
// lose its undercut, and the wheel it runs with the same taken off, so the centre distance stays
// m (N1 + N2) / 2).
//
// The sweep cuts one tooth space, the one centred on +v (std.y); the others are that space turned
// a pitch at a time about the axis, so a tooth is centred 180deg / N on from +v.  With `shaped`
// 0 the gear is its blank, its tip cylinder: quick to draw while the layout is worked on.
//
// `chamfer` is the 45deg chamfer's leg at the tips (0mm for none).  With `spokes` over 0 the web
// is opened between a hub of radius `hub` and a rim `web` thick under the tooth roots, leaving
// that many spokes `web` wide, a window centred on -v.
component ShapedSpur(N: Int, m: Length, phi: Angle, shift: Scalar, thin: Length, z0: Length,
                     z1: Length, bore: Length, shaped: Int, chamfer: Length, spokes: Int,
                     hub: Length, web: Length) {
  R := m * N / 2                         // the pitch circle
  Rp := R + shift * m                    // where the rack's pitch line stands off the axis
  Rt := Rp + m                           // the tip: an addendum off the rack's pitch line
  hw := pi * m / 4 + thin / 2            // the rack tooth's half thickness on its pitch line

  in std.front {
    construction spindle := vertical line(std.origin, hint((0, 10)))
    length(10mm) spindle
  }
  in std.top {
    // the rack's pitch line, tangent to the pitch circle at +v and running -u: the way the
    // blank's pitch point goes as it turns counter-clockwise, so the rack rolls with it
    construction rack_line := line(hint((10, Rp)), hint((0, Rp)))
    rack_line.p1 coords(du: 10mm, dv: Rp) std.top
    rack_line.p2 on_v(d: Rp) std.top

    // the basic rack's tooth, pointing in, centred on +v: its flanks lean `phi` off the radial and
    // cross the pitch line `2 hw` apart; it runs a full dedendum (1.25 m) in past the pitch line
    // to cut the root, and out past the blank's tip
    construction centre := line(std.top.origin, rack_line.p2)
    a := point hint((-hw - m, Rp + 1.5 * m))
    b := point hint((-hw + m / 2, Rp - 1.25 * m))
    c := point hint((hw - m / 2, Rp - 1.25 * m))
    d := point hint((hw + m, Rp + 1.5 * m))
    flank := line(b, a)
    centre angle(phi) flank
    on_pitch := point hint((-hw, Rp))
    on_pitch coords(du: -hw, dv: Rp) std.top
    on_pitch coincident flank
    std.top.origin distance(Rp - 1.25 * m, along: v) b
    std.top.origin distance(Rp + 1.5 * m, along: v) a
    c symmetry(centre) b
    d symmetry(centre) a
    rack_tooth := face(a, b, c, d, -> close)

    tip := circle(center: std.top.origin) hint(r: Rt)
    radius(Rt) tip
    hole := circle(center: std.top.origin) hint(r: bore / 2)
    radius(bore / 2) hole
    construction pitch := circle(center: std.top.origin) hint(r: R)
    radius(R) pitch

    // the chamfers' cones through the tips' edge at each face, `chamfer` in from the tip circle:
    // both found at the top face, the bottom's cone grown 45deg from the bottom face to it
    top_at := point hint((0, Rt - chamfer))
    top_at on_v(d: Rt - chamfer) std.top
    bottom_at := point hint((0, Rt - chamfer + z1 - z0))
    bottom_at on_v(d: Rt - chamfer + z1 - z0) std.top
  }
  private construction tooth := solid(rack_tooth, from: z0 - 1mm, to: z1 + 1mm)

  // the generating roll: the blank turns, the rack slides its pitch circle's length a turn
  blank_turn := frame
  blank_turn turns(1) spindle
  rack := frame
  rack slides(2 * pi * R) rack_line
  index := frame
  index turns(1) spindle

  // the blank, chamfered and shaped: a solid of revolution but for its tooth spaces, then the
  // gear, that less its windows
  private construction shaped_blank := solid(face(tip, holes: hole), from: z0, to: z1)
  construction gear := solid(shaped_blank)

  // the chamfers: the gear kept inside two 45deg cones, each through the tips' edge at a face
  top_q := point
  top_q raised(d: z1, along: std.z) top_at
  bottom_q := point
  bottom_q raised(d: z1, along: std.z) bottom_at
  when chamfer > 0mm {
    top_cone := std.sets.ConeThrough(spindle, top_q, lean: -45deg)
    bottom_cone := std.sets.ConeThrough(spindle, bottom_q, lean: 45deg)
    bicone := { p | p inside top_cone; p inside bottom_cone }
    private construction chamfers := solid(bicone)
    chamfers common shaped_blank
  }

  // The spokes: the web cut away in `spokes` windows, the hub to the rim between neighbouring
  // spokes, their corners rounded `web / 2`.  One window is drawn, centred on -v between the
  // spokes either side of it, and turned round the axis for the rest.  Where each corner meets
  // the rim or the hub and where it meets a spoke's side are worked out here; every arc's centre
  // is found from its two ends and its radius.
  when spokes > 0 {
    ro := Rp - 1.25 * m - web            // the rim's inside
    rc := web / 2                        // the windows' corners
    so := web / 2 + rc                   // a corner's centre off its spoke's centre line
    beta := 270deg - 180deg / spokes     // the left-hand spoke's centre line, seen from +z
    th_o := beta + asin(so / (ro - rc))  // the bearings of the corners' centres, at the rim
    th_i := beta + asin(so / (hub + rc)) // and at the hub
    run_o := sqrt((ro - rc) ^ 2 - so ^ 2)          // the corners' feet along the spoke
    run_i := sqrt((hub + rc) ^ 2 - so ^ 2)
    in std.top {
      // where the corners meet the rim and the hub, left of -v and mirrored right of it
      lo_t := point hint((ro * cos(th_o), ro * sin(th_o)))
      lo_t coords(du: ro * cos(th_o), dv: ro * sin(th_o)) std.top
      ro_t := point hint((-ro * cos(th_o), ro * sin(th_o)))
      ro_t coords(du: -ro * cos(th_o), dv: ro * sin(th_o)) std.top
      li_t := point hint((hub * cos(th_i), hub * sin(th_i)))
      li_t coords(du: hub * cos(th_i), dv: hub * sin(th_i)) std.top
      ri_t := point hint((-hub * cos(th_i), hub * sin(th_i)))
      ri_t coords(du: -hub * cos(th_i), dv: hub * sin(th_i)) std.top
      // where they meet the spokes' sides, `web / 2` off each centre line
      lo_s := point hint((run_o * cos(beta) - web / 2 * sin(beta),
                          run_o * sin(beta) + web / 2 * cos(beta)))
      lo_s coords(du: run_o * cos(beta) - web / 2 * sin(beta),
                  dv: run_o * sin(beta) + web / 2 * cos(beta)) std.top
      ro_s := point hint((-run_o * cos(beta) + web / 2 * sin(beta),
                          run_o * sin(beta) + web / 2 * cos(beta)))
      ro_s coords(du: -run_o * cos(beta) + web / 2 * sin(beta),
                  dv: run_o * sin(beta) + web / 2 * cos(beta)) std.top
      li_s := point hint((run_i * cos(beta) - web / 2 * sin(beta),
                          run_i * sin(beta) + web / 2 * cos(beta)))
      li_s coords(du: run_i * cos(beta) - web / 2 * sin(beta),
                  dv: run_i * sin(beta) + web / 2 * cos(beta)) std.top
      ri_s := point hint((-run_i * cos(beta) + web / 2 * sin(beta),
                          run_i * sin(beta) + web / 2 * cos(beta)))
      ri_s coords(du: -run_i * cos(beta) + web / 2 * sin(beta),
                  dv: run_i * sin(beta) + web / 2 * cos(beta)) std.top

      // walked counter-clockwise round the window: the rim left to right (the window is on -v,
      // so counter-clockwise about the axis), up the right spoke, the hub right to left
      // (clockwise about the axis), down the left spoke
      rim := arc(center: hint((0, 0)), start: lo_t, end: ro_t)
      radius(ro) rim
      lo := arc(center: hint(((ro - rc) * cos(th_o), (ro - rc) * sin(th_o))), start: lo_s,
                   end: lo_t)
      radius(rc) lo
      left := line(lo_s, li_s)
      li := arc(center: hint(((hub + rc) * cos(th_i), (hub + rc) * sin(th_i))), start: li_t,
                   end: li_s)
      radius(rc) li
      hub_arc := arc(center: hint((0, 0)), start: li_t, end: ri_t)
      radius(hub) hub_arc
      ri := arc(center: hint((-(hub + rc) * cos(th_i), (hub + rc) * sin(th_i))), start: ri_s,
                   end: ri_t)
      radius(rc) ri
      right := line(ri_s, ro_s)
      ro_k := arc(center: hint((-(ro - rc) * cos(th_o), (ro - rc) * sin(th_o))), start: ro_t,
                     end: ro_s)
      radius(rc) ro_k
    }
    private construction window := solid(face(rim, ro_k, right, ri, hub_arc, li, left, lo),
                                         from: z0 - 1mm, to: z1 + 1mm)
    window cut gear
    repeat spokes - 1 as i {
      turned_window := window at(under: index) ((i + 1) * 360deg / spokes)
      turned_window cut gear
    }
  }
  when shaped == 1 {
    private construction space := sweep(tooth, through: shaped_blank, under: rack,
                                        seen_from: blank_turn)
    space cut shaped_blank
    repeat N - 1 as j {
      turned := space at(under: index) ((j + 1) * 360deg / N)
      turned cut shaped_blank
    }
  }
}

preview {
  pinion := ShapedSpur(N: 12, m: 1.5mm, phi: 20deg, shift: 0.3, thin: 0.15mm, z0: 0mm, z1: 10mm,
                       bore: 6.35mm, shaped: 1, chamfer: 0.6mm, spokes: 0, hub: 0mm, web: 0mm)
  wheel := ShapedSpur(N: 48, m: 0.8mm, phi: 20deg, shift: -0.3, thin: 0.15mm, z0: 0mm, z1: 5mm,
                      bore: 5.35mm, shaped: 1, chamfer: 0.3mm, spokes: 5, hub: 7.5mm, web: 2.4mm)
  wheel_at := point
  wheel_at raised(d: 40mm, along: std.x) std.origin
  wheel seats(turn: 0deg, up: std.z, east: std.x) wheel_at
  shown := solid(pinion.gear)
  shown_wheel := solid(wheel.gear)
}
