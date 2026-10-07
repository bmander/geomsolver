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

use engine.dims
use engine.parts
use engine.valvetrain
use std (horizontal, offset, right_of)

// A cam bearing cap, edge on or from above: a block `wcamb` long round the journal.
component CamBearing(c: point, dims: group) {
  cap := engine.parts.Box(c, x0: -dims.wcamb / 2, y0: -(dims.rcamj + dims.camcap), x1: dims.wcamb / 2, y1: dims.rcamj + dims.camcap)
}

component CylinderHead(end: plane, side: plane, top: plane, o: point, o_s: point, o_t: point, dims: group) {
  // -- along the axis: the section through cylinder 1 -------------------------------------
  in end {
    // the casting: its face on the gasket, its sides, its top
    f_l := point hint((o.x - dims.hw, o.y + dims.deck + dims.gasket))
    f_r := point hint((o.x + dims.hw, o.y + dims.deck + dims.gasket))
    t_l := point hint((o.x - 110mm, o.y + dims.deck + dims.head))
    tr := engine.parts.At(o, dx: 110mm, dy: dims.deck + dims.head)
    (gasket := line(f_l, f_r)) -> (side_r := line(f_r, tr.p)) -> (topline := line(tr.p, t_l)) -> (side_l := line(t_l, f_l)) -> close
    f_l offset(dx: -dims.hw, dy: dims.deck + dims.gasket) o
    o distance(dims.deck + dims.gasket, along: y) f_r
    f_l distance(2 * dims.hw) f_r
    t_l offset(dx: -110, dy: dims.deck + dims.head) o
    // the pent roof over the bore, from the face at the bore's edges up to the ridge
    r_l := engine.parts.At(o, dx: -dims.D / 2, dy: dims.deck + dims.gasket)
    r_r := engine.parts.At(o, dx: dims.D / 2, dy: dims.deck + dims.gasket)
    ridge := engine.parts.At(o, dx: 0mm, dy: dims.roof + dims.gasket)
    roof_l := line(r_l.p, ridge.p)
    roof_r := line(r_r.p, ridge.p)
    plug := engine.parts.Box(ridge.p, x0: -7mm, y0: 0mm, x1: 7mm, y1: 40mm)
    // the valve axes: through the seat centres, square to the roof, up to the cam centres
    seat_i := point hint((o.x + dims.vs, o.y + dims.deck + dims.gasket + (dims.D / 2 - dims.vs) * tan(dims.va)))
    seat_e := point hint((o.x - dims.vs, o.y + dims.deck + dims.gasket + (dims.D / 2 - dims.vs) * tan(dims.va)))
    seat_i coincident roof_r
    seat_e coincident roof_l
    o distance(dims.vs, along: x) seat_i
    o distance(-dims.vs, along: x) seat_e
    cam_i := point hint((o.x + dims.camx, o.y + dims.camh + dims.gasket))
    cam_e := point hint((o.x - dims.camx, o.y + dims.camh + dims.gasket))
    vaxis_i := line(seat_i, cam_i)
    vaxis_e := line(seat_e, cam_e)
    vaxis_i perpendicular roof_r
    vaxis_e perpendicular roof_l
    seat_i distance(dims.stem + dims.rb) cam_i
    seat_e distance(dims.stem + dims.rb) cam_e
    // the camshaft journals, hidden behind the lobes
    j_i := circle(center: cam_i) hint(r: dims.rcamj)
    j_e := circle(center: cam_e) hint(r: dims.rcamj)
    radius(dims.rcamj) j_i
    radius(dims.rcamj) j_e
    // where cylinder 1 is in its cycle says where each lobe points and how far each valve is
    // off its seat: the lobe's reach along the axis, less the base circle (see `dims.sv`)
    ai := (dims.cycle - dims.icenter) / 2
    ae := (dims.cycle - dims.ecenter) / 2
    lift_now_i := max(dims.rb, dims.dn_i * cos(ai) + dims.rn) - dims.rb
    lift_now_e := max(dims.rb, dims.dn_e * cos(ae) + dims.rn) - dims.rb
    lobe_i := engine.valvetrain.Lobe(cam_i, vaxis_i, phi: 180deg + ai, dn: dims.dn_i, dims: dims)
    lobe_e := engine.valvetrain.Lobe(cam_e, vaxis_e, phi: 180deg + ae, dn: dims.dn_e, dims: dims)
    v_i := engine.valvetrain.Valve(seat_i, vaxis_i, lift: lift_now_i, head: dims.div, dims: dims)
    v_e := engine.valvetrain.Valve(seat_e, vaxis_e, lift: lift_now_e, head: dims.dev, dims: dims)
  }

  // -- across the axis: the head edge on ----------------------------------------------
  in side {
    hfl := point hint((o_s.x + dims.front, o_s.y + dims.deck + dims.gasket))
    hfr := point hint((o_s.x + dims.back, o_s.y + dims.deck + dims.gasket))
    htl := point hint((o_s.x + dims.front, o_s.y + dims.deck + dims.head))
    htr := point hint((o_s.x + dims.back, o_s.y + dims.deck + dims.head))
    (hface := line(hfl, hfr)) -> (hback := line(hfr, htr)) -> (htop := line(htr, htl)) -> (hfront := line(htl, hfl)) -> close
    o_s distance(dims.front, along: x) hfl
    o_s distance(dims.front, along: x) htl
    o_s distance(dims.back, along: x) hfr
    o_s distance(dims.back, along: x) htr
    horizontal hface
    horizontal htop
    // the camshaft: the two shafts lie one behind the other here, one journal's outline
    cam := point hint((o_s.x + dims.front, o_s.y + dims.camh + dims.gasket))
    camb := point hint((o_s.x + dims.back, o_s.y + dims.camh + dims.gasket))
    camline := line(cam, camb)
    o_s distance(dims.front, along: x) cam
    o_s distance(dims.back, along: x) camb
    horizontal camline
    ju0 := point hint((o_s.x + dims.front + 10mm, o_s.y + dims.camh + dims.gasket + dims.rcamj))
    ju1 := point hint((o_s.x + dims.back - 10mm, o_s.y + dims.camh + dims.gasket + dims.rcamj))
    jd0 := point hint((o_s.x + dims.front + 10mm, o_s.y + dims.camh + dims.gasket - dims.rcamj))
    jd1 := point hint((o_s.x + dims.back - 10mm, o_s.y + dims.camh + dims.gasket - dims.rcamj))
    shaft_u := line(ju0, ju1)
    shaft_d := line(jd0, jd1)
    o_s distance(dims.front + 10mm, along: x) ju0
    o_s distance(dims.back - 10mm, along: x) ju1
    o_s distance(dims.front + 10mm, along: x) jd0
    o_s distance(dims.back - 10mm, along: x) jd1
    cam distance(dims.rcamj, along: y) ju0
    cam distance(dims.rcamj, along: y) ju1
    cam distance(-dims.rcamj, along: y) jd0
    cam distance(-dims.rcamj, along: y) jd1
    // five bearings, between and beyond the cylinders
    repeat 5 as j {
      bc := point hint((o_s.x + dims.front + 25mm + j * dims.P, o_s.y + dims.camh + dims.gasket))
      o_s distance(dims.front + 25mm + j * dims.P, along: x) bc
      cam horizontal bc
      bearing := CamBearing(bc, dims: dims)
    }
    // every lobe at its own cylinder's angle: the firing order 1-3-4-2 puts cylinder 3 a half
    // turn behind 1 in the cycle, 4 a full turn, 2 a turn and a half.  Edge on, a lobe reaches
    // above and below the shaft by the nose's height on the page with the nose circle round it,
    // or the base circle if that is taller; the intake axis leans out one way, the exhaust the other.
    repeat 4 as i {
      off := 180deg * (i * (3 - i) / 2) + 360deg * (i - 2 * floor(i / 2))
      c := dims.cycle - off - 720deg * floor((dims.cycle - off) / 720deg)
      ai := (c - dims.icenter) / 2
      ae := (c - dims.ecenter) / 2
      ny_i := dims.dn_i * sin(90deg - dims.va + 180deg + ai)
      ny_e := dims.dn_e * sin(90deg + dims.va + 180deg + ae)
      top_i := max(dims.rb, ny_i + dims.rn)
      bot_i := max(dims.rb, dims.rn - ny_i)
      top_e := max(dims.rb, ny_e + dims.rn)
      bot_e := max(dims.rb, dims.rn - ny_e)
      lc := point hint((o_s.x + dims.front + 25mm + dims.P / 2 + i * dims.P, o_s.y + dims.camh + dims.gasket))
      o_s distance(dims.front + 25mm + dims.P / 2 + i * dims.P, along: x) lc
      cam horizontal lc
      lobe_i := engine.parts.Box(lc, x0: 14mm, y0: -bot_i, x1: 26mm, y1: top_i)
      lobe_e := engine.parts.Box(lc, x0: -26mm, y0: -bot_e, x1: -14mm, y1: top_e)
    }
  }

  // -- from above: the head's outline, the two shafts in their bearings, and each cylinder's
  // valves and plug -----------------------------------------------------------------------
  in top {
    hfl_t := point hint((o_t.x + dims.front, o_t.y - 110mm))
    hfr_t := point hint((o_t.x + dims.back, o_t.y - 110mm))
    hbr_t := point hint((o_t.x + dims.back, o_t.y + 110mm))
    hbl_t := point hint((o_t.x + dims.front, o_t.y + 110mm))
    (h1 := line(hfl_t, hfr_t)) -> (h2 := line(hfr_t, hbr_t)) -> (h3 := line(hbr_t, hbl_t)) -> (h4 := line(hbl_t, hfl_t)) -> close
    horizontal h1
    vertical h2
    horizontal h3
    vertical h4
    o_t distance(dims.front, along: x) hfl_t
    o_t distance(dims.back, along: x) hbr_t
    ci := point hint((o_t.x + dims.front + 10mm, o_t.y + dims.camx))
    ce := point hint((o_t.x + dims.front + 10mm, o_t.y - dims.camx))
    ci1 := point hint((o_t.x + dims.back - 10mm, o_t.y + dims.camx))
    ce1 := point hint((o_t.x + dims.back - 10mm, o_t.y - dims.camx))
    cl_i := line(ci, ci1)
    cl_e := line(ce, ce1)
    o_t distance(dims.front + 10mm, along: x) ci
    o_t distance(dims.back - 10mm, along: x) ci1
    o_t distance(dims.front + 10mm, along: x) ce
    o_t distance(dims.back - 10mm, along: x) ce1
    horizontal cl_i
    horizontal cl_e
    // the shafts' outlines, `rcamj` either side of each centreline
    repeat 2 as s {
      sgn := 1 - 2 * s
      a0 := point hint((o_t.x + dims.front + 10mm, o_t.y + sgn * (dims.camx + dims.rcamj)))
      a1 := point hint((o_t.x + dims.back - 10mm, o_t.y + sgn * (dims.camx + dims.rcamj)))
      b0 := point hint((o_t.x + dims.front + 10mm, o_t.y + sgn * (dims.camx - dims.rcamj)))
      b1 := point hint((o_t.x + dims.back - 10mm, o_t.y + sgn * (dims.camx - dims.rcamj)))
      outer := line(a0, a1)
      inner := line(b0, b1)
      o_t distance(dims.front + 10mm, along: x) a0
      o_t distance(dims.back - 10mm, along: x) a1
      o_t distance(dims.front + 10mm, along: x) b0
      o_t distance(dims.back - 10mm, along: x) b1
      ci distance(sgn * (dims.camx + dims.rcamj) - dims.camx, along: y) a0
      ci distance(sgn * (dims.camx + dims.rcamj) - dims.camx, along: y) a1
      ci distance(sgn * (dims.camx - dims.rcamj) - dims.camx, along: y) b0
      ci distance(sgn * (dims.camx - dims.rcamj) - dims.camx, along: y) b1
    }
    repeat 5 as j {
      bi := point hint((o_t.x + dims.front + 25mm + j * dims.P, o_t.y + dims.camx))
      be := point hint((o_t.x + dims.front + 25mm + j * dims.P, o_t.y - dims.camx))
      o_t distance(dims.front + 25mm + j * dims.P, along: x) bi
      o_t distance(dims.front + 25mm + j * dims.P, along: x) be
      ci horizontal bi
      ce horizontal be
      cap_i := CamBearing(bi, dims: dims)
      cap_e := CamBearing(be, dims: dims)
    }
    repeat 4 as i {
      pc := point hint((o_t.x + dims.front + 25mm + dims.P / 2 + i * dims.P, o_t.y))
      pc right_of(d: dims.front + 25mm + dims.P / 2 + i * dims.P) o_t
      plug := circle(center: pc) hint(r: 7mm)
      radius(7) plug
      repeat 2 as k {
        vi := engine.parts.At(pc, dx: -16mm + k * 32mm, dy: dims.vs)
        ve := engine.parts.At(pc, dx: -16mm + k * 32mm, dy: -dims.vs)
        intake := circle(center: vi.p) hint(r: dims.div / 2)
        exhaust := circle(center: ve.p) hint(r: dims.dev / 2)
        radius(dims.div / 2) intake
        radius(dims.dev / 2) exhaust
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
}
