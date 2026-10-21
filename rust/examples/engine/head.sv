// The cylinder head, designed in one place: a casting of its own standing on its gasket, with
// the pent-roof chambers, the valves square to the roof, and two overhead camshafts in their
// bearings.
//
// One component, three views (§6.7).  Along the axis it is the section through cylinder 1: the
// head's outline from the gasket face up, the roof over the bore, the plug, the two valves and the
// two lobes over them — where each lobe points and how far each valve is lifted is the cycle's,
// worked out from the timing in `dims.sv`.  Across the axis it is the head edge on: the casting,
// the camshaft (the two shafts one behind the other) in its five bearings, and every lobe at its
// own cylinder's angle.  From above it is the two shafts with their bearing caps, and the valves
// and plug of each cylinder.  Heights in the side view and widths in the plan are the end view's
// by projection, inside the part.

use engine.dims (D, P, back, camcap, camh, camx, deck, dev, div, dn_e, dn_i, ecenter, front,
                 gasket, head, hw, icenter, rb, rcamj, rn, roof, stem, va, vs, wcamb)
use engine.parts
use engine.valvetrain
use std (horizontal, offset, right_of)

// A cam bearing cap, edge on or from above: a block `wcamb` long round the journal.
part CamBearing(c: point) {
  cap := engine.parts.Box(c, x0: -wcamb / 2, y0: -(rcamj + camcap), x1: wcamb / 2, y1: rcamj + camcap)
}

part CylinderHead(end: plane, side: plane, top: plane, o: point, o_s: point, o_t: point) {
  // -- along the axis: the section through cylinder 1 -------------------------------------
  in end {
    // the casting: its face on the gasket, its sides, its top
    f_l := point hint((o.x - hw, o.y + deck + gasket))
    f_r := point hint((o.x + hw, o.y + deck + gasket))
    t_l := point hint((o.x - 110mm, o.y + deck + head))
    tr := engine.parts.At(o, dx: 110mm, dy: deck + head)
    (gasket := line(f_l, f_r)) -> (side_r := line(f_r, tr.p)) -> (topline := line(tr.p, t_l)) -> (side_l := line(t_l, f_l)) -> close
    f_l offset(dx: -hw, dy: deck + gasket) o
    o distance(deck + gasket, along: v) f_r
    f_l distance(2 * hw) f_r
    t_l offset(dx: -110, dy: deck + head) o
    // the pent roof over the bore, from the face at the bore's edges up to the ridge
    r_l := engine.parts.At(o, dx: -D / 2, dy: deck + gasket)
    r_r := engine.parts.At(o, dx: D / 2, dy: deck + gasket)
    ridge := engine.parts.At(o, dx: 0mm, dy: roof + gasket)
    roof_l := line(r_l.p, ridge.p)
    roof_r := line(r_r.p, ridge.p)
    plug := engine.parts.Box(ridge.p, x0: -7mm, y0: 0mm, x1: 7mm, y1: 40mm)
    // the valve axes: through the seat centres, square to the roof, up to the cam centres
    seat_i := point hint((o.x + vs, o.y + deck + gasket + (D / 2 - vs) * tan(va)))
    seat_e := point hint((o.x - vs, o.y + deck + gasket + (D / 2 - vs) * tan(va)))
    seat_i coincident roof_r
    seat_e coincident roof_l
    o distance(vs, along: u) seat_i
    o distance(-vs, along: u) seat_e
    cam_i := point hint((o.x + camx, o.y + camh + gasket))
    cam_e := point hint((o.x - camx, o.y + camh + gasket))
    vaxis_i := line(seat_i, cam_i)
    vaxis_e := line(seat_e, cam_e)
    vaxis_i perpendicular roof_r
    vaxis_e perpendicular roof_l
    seat_i distance(stem + rb) cam_i
    seat_e distance(stem + rb) cam_e
    // the camshaft journals, hidden behind the lobes
    j_i := circle(center: cam_i) hint(r: rcamj)
    j_e := circle(center: cam_e) hint(r: rcamj)
    radius(rcamj) j_i
    radius(rcamj) j_e
    // where cylinder 1 is in its cycle says where each lobe points and how far each valve is
    // off its seat: the lobe's reach along the axis, less the base circle (see `dims.sv`)
    ai := (engine.dims.cycle - icenter) / 2
    ae := (engine.dims.cycle - ecenter) / 2
    lift_now_i := max(rb, dn_i * cos(ai) + rn) - rb
    lift_now_e := max(rb, dn_e * cos(ae) + rn) - rb
    lobe_i := engine.valvetrain.Lobe(cam_i, vaxis_i, phi: 180deg + ai, dn: dn_i)
    lobe_e := engine.valvetrain.Lobe(cam_e, vaxis_e, phi: 180deg + ae, dn: dn_e)
    v_i := engine.valvetrain.Valve(seat_i, vaxis_i, lift: lift_now_i, head: div)
    v_e := engine.valvetrain.Valve(seat_e, vaxis_e, lift: lift_now_e, head: dev)
  }

  // -- across the axis: the head edge on ----------------------------------------------
  in side {
    hfl := point hint((o_s.x + front, o_s.y + deck + gasket))
    hfr := point hint((o_s.x + back, o_s.y + deck + gasket))
    htl := point hint((o_s.x + front, o_s.y + deck + head))
    htr := point hint((o_s.x + back, o_s.y + deck + head))
    (hface := line(hfl, hfr)) -> (hback := line(hfr, htr)) -> (htop := line(htr, htl)) -> (hfront := line(htl, hfl)) -> close
    o_s distance(front, along: u) hfl
    o_s distance(front, along: u) htl
    o_s distance(back, along: u) hfr
    o_s distance(back, along: u) htr
    horizontal hface
    horizontal htop
    // the camshaft: the two shafts lie one behind the other here, one journal's outline
    cam := point hint((o_s.x + front, o_s.y + camh + gasket))
    camb := point hint((o_s.x + back, o_s.y + camh + gasket))
    camline := line(cam, camb)
    o_s distance(front, along: u) cam
    o_s distance(back, along: u) camb
    horizontal camline
    ju0 := point hint((o_s.x + front + 10mm, o_s.y + camh + gasket + rcamj))
    ju1 := point hint((o_s.x + back - 10mm, o_s.y + camh + gasket + rcamj))
    jd0 := point hint((o_s.x + front + 10mm, o_s.y + camh + gasket - rcamj))
    jd1 := point hint((o_s.x + back - 10mm, o_s.y + camh + gasket - rcamj))
    shaft_u := line(ju0, ju1)
    shaft_d := line(jd0, jd1)
    o_s distance(front + 10mm, along: u) ju0
    o_s distance(back - 10mm, along: u) ju1
    o_s distance(front + 10mm, along: u) jd0
    o_s distance(back - 10mm, along: u) jd1
    cam distance(rcamj, along: v) ju0
    cam distance(rcamj, along: v) ju1
    cam distance(-rcamj, along: v) jd0
    cam distance(-rcamj, along: v) jd1
    // five bearings, between and beyond the cylinders
    repeat 5 as j {
      bc := point hint((o_s.x + front + 25mm + j * P, o_s.y + camh + gasket))
      o_s distance(front + 25mm + j * P, along: u) bc
      cam horizontal bc
      bearing := CamBearing(bc)
    }
    // every lobe at its own cylinder's angle: the firing order 1-3-4-2 puts cylinder 3 a half
    // turn behind 1 in the cycle, 4 a full turn, 2 a turn and a half.  Edge on, a lobe reaches
    // above and below the shaft by the nose's height on the page with the nose circle round it,
    // or the base circle if that is taller; the intake axis leans out one way, the exhaust the other.
    repeat 4 as i {
      off := 180deg * (i * (3 - i) / 2) + 360deg * (i - 2 * floor(i / 2))
      c := engine.dims.cycle - off - 720deg * floor((engine.dims.cycle - off) / 720deg)
      ai := (c - icenter) / 2
      ae := (c - ecenter) / 2
      ny_i := dn_i * sin(90deg - va + 180deg + ai)
      ny_e := dn_e * sin(90deg + va + 180deg + ae)
      top_i := max(rb, ny_i + rn)
      bot_i := max(rb, rn - ny_i)
      top_e := max(rb, ny_e + rn)
      bot_e := max(rb, rn - ny_e)
      lc := point hint((o_s.x + front + 25mm + P / 2 + i * P, o_s.y + camh + gasket))
      o_s distance(front + 25mm + P / 2 + i * P, along: u) lc
      cam horizontal lc
      lobe_i := engine.parts.Box(lc, x0: 14mm, y0: -bot_i, x1: 26mm, y1: top_i)
      lobe_e := engine.parts.Box(lc, x0: -26mm, y0: -bot_e, x1: -14mm, y1: top_e)
    }
  }

  // -- from above: the head's outline, the two shafts in their bearings, and each cylinder's
  // valves and plug -----------------------------------------------------------------------
  in top {
    hfl_t := point hint((o_t.x + front, o_t.y - 110mm))
    hfr_t := point hint((o_t.x + back, o_t.y - 110mm))
    hbr_t := point hint((o_t.x + back, o_t.y + 110mm))
    hbl_t := point hint((o_t.x + front, o_t.y + 110mm))
    (h1 := line(hfl_t, hfr_t)) -> (h2 := line(hfr_t, hbr_t)) -> (h3 := line(hbr_t, hbl_t)) -> (h4 := line(hbl_t, hfl_t)) -> close
    horizontal h1
    vertical h2
    horizontal h3
    vertical h4
    o_t distance(front, along: u) hfl_t
    o_t distance(back, along: u) hbr_t
    ci := point hint((o_t.x + front + 10mm, o_t.y + camx))
    ce := point hint((o_t.x + front + 10mm, o_t.y - camx))
    ci1 := point hint((o_t.x + back - 10mm, o_t.y + camx))
    ce1 := point hint((o_t.x + back - 10mm, o_t.y - camx))
    cl_i := line(ci, ci1)
    cl_e := line(ce, ce1)
    o_t distance(front + 10mm, along: u) ci
    o_t distance(back - 10mm, along: u) ci1
    o_t distance(front + 10mm, along: u) ce
    o_t distance(back - 10mm, along: u) ce1
    horizontal cl_i
    horizontal cl_e
    // the shafts' outlines, `rcamj` either side of each centreline
    repeat 2 as s {
      sgn := 1 - 2 * s
      a0 := point hint((o_t.x + front + 10mm, o_t.y + sgn * (camx + rcamj)))
      a1 := point hint((o_t.x + back - 10mm, o_t.y + sgn * (camx + rcamj)))
      b0 := point hint((o_t.x + front + 10mm, o_t.y + sgn * (camx - rcamj)))
      b1 := point hint((o_t.x + back - 10mm, o_t.y + sgn * (camx - rcamj)))
      outer := line(a0, a1)
      inner := line(b0, b1)
      o_t distance(front + 10mm, along: u) a0
      o_t distance(back - 10mm, along: u) a1
      o_t distance(front + 10mm, along: u) b0
      o_t distance(back - 10mm, along: u) b1
      ci distance(sgn * (camx + rcamj) - camx, along: v) a0
      ci distance(sgn * (camx + rcamj) - camx, along: v) a1
      ci distance(sgn * (camx - rcamj) - camx, along: v) b0
      ci distance(sgn * (camx - rcamj) - camx, along: v) b1
    }
    repeat 5 as j {
      bi := point hint((o_t.x + front + 25mm + j * P, o_t.y + camx))
      be := point hint((o_t.x + front + 25mm + j * P, o_t.y - camx))
      o_t distance(front + 25mm + j * P, along: u) bi
      o_t distance(front + 25mm + j * P, along: u) be
      ci horizontal bi
      ce horizontal be
      cap_i := CamBearing(bi)
      cap_e := CamBearing(be)
    }
    repeat 4 as i {
      pc := point hint((o_t.x + front + 25mm + P / 2 + i * P, o_t.y))
      pc right_of(d: front + 25mm + P / 2 + i * P) o_t
      plug := circle(center: pc) hint(r: 7mm)
      radius(7) plug
      repeat 2 as k {
        vi := engine.parts.At(pc, dx: -16mm + k * 32mm, dy: vs)
        ve := engine.parts.At(pc, dx: -16mm + k * 32mm, dy: -vs)
        intake := circle(center: vi.p) hint(r: div / 2)
        exhaust := circle(center: ve.p) hint(r: dev / 2)
        radius(div / 2) intake
        radius(dev / 2) exhaust
      }
    }
  }

  // -- the views agree ------------------------------------------------------------------
  f_l project hfl              // the gasket face's height
  t_l project htl              // the head's top
  cam_i project cam            // the camshafts' height
  cam_i project ci             // and where each stands across the engine
  cam_e project ce
  t_l project hbl_t            // the head's width, from its top corner
  tr.p project hfl_t

  // -- the casting as a solid -------------------------------------------------------------
  // The end section's outline run along the crank axis from the front face to the rear, bored
  // for the two camshafts; each shaft runs in its tunnel from its pulley, forward of the block,
  // to its last bearing.
  casting := solid(face(gasket, side_r, topline, side_l), from: front, to: back)
  tunnel_i := solid(face(j_i), from: front - 1mm, to: back + 1mm)
  tunnel_e := solid(face(j_e), from: front - 1mm, to: back + 1mm)
  tunnel_i cut casting
  tunnel_e cut casting
  camshaft_i := solid(face(j_i), from: front - 55mm, to: back - 10mm)
  camshaft_e := solid(face(j_e), from: front - 55mm, to: back - 10mm)
}

// Open this file to preview the head alone: the casting in the three views the assembly draws it
// in, about the world's origin, with its valves and lobes where the table's cycle puts them, and
// the casting and the two camshafts as solids.
preview {
  unit mm
  cylinder_head := CylinderHead(std.side, std.front, std.top, std.side.origin, std.origin,
                               std.top.origin)
}
