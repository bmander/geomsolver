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

// A cam bearing cap, edge on or from above: a block `wcamb` long round the journal.
component CamBearing(c: point, dims: group) {
  cap: Box(c, x0: -dims.wcamb / 2, y0: -(dims.rcamj + dims.camcap), x1: dims.wcamb / 2, y1: dims.rcamj + dims.camcap)
}

component CylinderHead(end: plane, side: plane, top: plane, o: point, o_s: point, o_t: point, dims: group) {
  // -- along the axis: the section through cylinder 1 -------------------------------------
  in end {
    // the casting: its face on the gasket, its sides, its top
    point f_l hint(x: o.x - dims.hw, y: o.y + dims.deck + dims.gasket)
    point f_r hint(x: o.x + dims.hw, y: o.y + dims.deck + dims.gasket)
    point t_l hint(x: o.x - 110mm, y: o.y + dims.deck + dims.head)
    tr: At(o, dx: 110mm, dy: dims.deck + dims.head)
    line gasket(f_l, f_r) -> line side_r(f_r, tr.p) -> line topline(tr.p, t_l) -> line side_l(t_l, f_l) -> close
    o distance(-dims.hw, along: x) f_l
    o distance(dims.deck + dims.gasket, along: y) f_l
    o distance(dims.deck + dims.gasket, along: y) f_r
    f_l distance(2 * dims.hw) f_r
    o distance(110, along: left) t_l
    o distance(dims.deck + dims.head, along: y) t_l
    // the pent roof over the bore, from the face at the bore's edges up to the ridge
    r_l: At(o, dx: -dims.D / 2, dy: dims.deck + dims.gasket)
    r_r: At(o, dx: dims.D / 2, dy: dims.deck + dims.gasket)
    ridge: At(o, dx: 0mm, dy: dims.roof + dims.gasket)
    line roof_l(r_l.p, ridge.p)
    line roof_r(r_r.p, ridge.p)
    plug: Box(ridge.p, x0: -7mm, y0: 0mm, x1: 7mm, y1: 40mm)
    // the valve axes: through the seat centres, square to the roof, up to the cam centres
    point seat_i hint(x: o.x + dims.vs, y: o.y + dims.deck + dims.gasket + (dims.D / 2 - dims.vs) * tan(dims.va))
    point seat_e hint(x: o.x - dims.vs, y: o.y + dims.deck + dims.gasket + (dims.D / 2 - dims.vs) * tan(dims.va))
    seat_i on roof_r
    seat_e on roof_l
    o distance(dims.vs, along: x) seat_i
    o distance(-dims.vs, along: x) seat_e
    point cam_i hint(x: o.x + dims.camx, y: o.y + dims.camh + dims.gasket)
    point cam_e hint(x: o.x - dims.camx, y: o.y + dims.camh + dims.gasket)
    line vaxis_i(seat_i, cam_i)
    line vaxis_e(seat_e, cam_e)
    vaxis_i perpendicular roof_r
    vaxis_e perpendicular roof_l
    seat_i distance(dims.stem + dims.rb) cam_i
    seat_e distance(dims.stem + dims.rb) cam_e
    // the camshaft journals, hidden behind the lobes
    circle j_i(center: cam_i) hint(r: dims.rcamj)
    circle j_e(center: cam_e) hint(r: dims.rcamj)
    radius(dims.rcamj) j_i
    radius(dims.rcamj) j_e
    // where cylinder 1 is in its cycle says where each lobe points and how far each valve is
    // off its seat: the lobe's reach along the axis, less the base circle (see `dims.sv`)
    param ai = (dims.cycle - dims.icenter) / 2
    param ae = (dims.cycle - dims.ecenter) / 2
    param lift_now_i = max(dims.rb, dims.dn_i * cos(ai) + dims.rn) - dims.rb
    param lift_now_e = max(dims.rb, dims.dn_e * cos(ae) + dims.rn) - dims.rb
    lobe_i: Lobe(cam_i, vaxis_i, phi: 180deg + ai, dn: dims.dn_i, dims: dims)
    lobe_e: Lobe(cam_e, vaxis_e, phi: 180deg + ae, dn: dims.dn_e, dims: dims)
    v_i: Valve(seat_i, vaxis_i, lift: lift_now_i, head: dims.div, dims: dims)
    v_e: Valve(seat_e, vaxis_e, lift: lift_now_e, head: dims.dev, dims: dims)
  }

  // -- across the axis: the head edge on ----------------------------------------------
  in side {
    point hfl hint(x: o_s.x + dims.front, y: o_s.y + dims.deck + dims.gasket)
    point hfr hint(x: o_s.x + dims.back, y: o_s.y + dims.deck + dims.gasket)
    point htl hint(x: o_s.x + dims.front, y: o_s.y + dims.deck + dims.head)
    point htr hint(x: o_s.x + dims.back, y: o_s.y + dims.deck + dims.head)
    line hface(hfl, hfr) -> line hback(hfr, htr) -> line htop(htr, htl) -> line hfront(htl, hfl) -> close
    o_s distance(dims.front, along: x) hfl
    o_s distance(dims.front, along: x) htl
    o_s distance(dims.back, along: x) hfr
    o_s distance(dims.back, along: x) htr
    horizontal hface
    horizontal htop
    // the camshaft: the two shafts lie one behind the other here, one journal's outline
    point cam hint(x: o_s.x + dims.front, y: o_s.y + dims.camh + dims.gasket)
    point camb hint(x: o_s.x + dims.back, y: o_s.y + dims.camh + dims.gasket)
    line camline(cam, camb)
    o_s distance(dims.front, along: x) cam
    o_s distance(dims.back, along: x) camb
    horizontal camline
    point ju0 hint(x: o_s.x + dims.front + 10mm, y: o_s.y + dims.camh + dims.gasket + dims.rcamj)
    point ju1 hint(x: o_s.x + dims.back - 10mm, y: o_s.y + dims.camh + dims.gasket + dims.rcamj)
    point jd0 hint(x: o_s.x + dims.front + 10mm, y: o_s.y + dims.camh + dims.gasket - dims.rcamj)
    point jd1 hint(x: o_s.x + dims.back - 10mm, y: o_s.y + dims.camh + dims.gasket - dims.rcamj)
    line shaft_u(ju0, ju1)
    line shaft_d(jd0, jd1)
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
      point bc hint(x: o_s.x + dims.front + 25mm + j * dims.P, y: o_s.y + dims.camh + dims.gasket)
      o_s distance(dims.front + 25mm + j * dims.P, along: x) bc
      cam distance(0, along: y) bc
      bearing: CamBearing(bc, dims: dims)
    }
    // every lobe at its own cylinder's angle: the firing order 1-3-4-2 puts cylinder 3 a half
    // turn behind 1 in the cycle, 4 a full turn, 2 a turn and a half.  Edge on, a lobe reaches
    // above and below the shaft by the nose's height on the page with the nose circle round it,
    // or the base circle if that is taller; the intake axis leans out one way, the exhaust the other.
    repeat 4 as i {
      param off = 180deg * (i * (3 - i) / 2) + 360deg * (i - 2 * floor(i / 2))
      param c = dims.cycle - off - 720deg * floor((dims.cycle - off) / 720deg)
      param ai = (c - dims.icenter) / 2
      param ae = (c - dims.ecenter) / 2
      param ny_i = dims.dn_i * sin(90deg - dims.va + 180deg + ai)
      param ny_e = dims.dn_e * sin(90deg + dims.va + 180deg + ae)
      param top_i = max(dims.rb, ny_i + dims.rn)
      param bot_i = max(dims.rb, dims.rn - ny_i)
      param top_e = max(dims.rb, ny_e + dims.rn)
      param bot_e = max(dims.rb, dims.rn - ny_e)
      point lc hint(x: o_s.x + dims.front + 25mm + dims.P / 2 + i * dims.P, y: o_s.y + dims.camh + dims.gasket)
      o_s distance(dims.front + 25mm + dims.P / 2 + i * dims.P, along: x) lc
      cam distance(0, along: y) lc
      lobe_i: Box(lc, x0: 14mm, y0: -bot_i, x1: 26mm, y1: top_i)
      lobe_e: Box(lc, x0: -26mm, y0: -bot_e, x1: -14mm, y1: top_e)
    }
  }

  // -- from above: the head's outline, the two shafts in their bearings, and each cylinder's
  // valves and plug -----------------------------------------------------------------------
  in top {
    point hfl_t hint(x: o_t.x + dims.front, y: o_t.y - 110mm)
    point hfr_t hint(x: o_t.x + dims.back, y: o_t.y - 110mm)
    point hbr_t hint(x: o_t.x + dims.back, y: o_t.y + 110mm)
    point hbl_t hint(x: o_t.x + dims.front, y: o_t.y + 110mm)
    line h1(hfl_t, hfr_t) -> line h2(hfr_t, hbr_t) -> line h3(hbr_t, hbl_t) -> line h4(hbl_t, hfl_t) -> close
    horizontal h1
    vertical h2
    horizontal h3
    vertical h4
    o_t distance(dims.front, along: x) hfl_t
    o_t distance(dims.back, along: x) hbr_t
    point ci hint(x: o_t.x + dims.front + 10mm, y: o_t.y + dims.camx)
    point ce hint(x: o_t.x + dims.front + 10mm, y: o_t.y - dims.camx)
    point ci1 hint(x: o_t.x + dims.back - 10mm, y: o_t.y + dims.camx)
    point ce1 hint(x: o_t.x + dims.back - 10mm, y: o_t.y - dims.camx)
    line cl_i(ci, ci1)
    line cl_e(ce, ce1)
    o_t distance(dims.front + 10mm, along: x) ci
    o_t distance(dims.back - 10mm, along: x) ci1
    o_t distance(dims.front + 10mm, along: x) ce
    o_t distance(dims.back - 10mm, along: x) ce1
    horizontal cl_i
    horizontal cl_e
    // the shafts' outlines, `rcamj` either side of each centreline
    repeat 2 as s {
      param sgn = 1 - 2 * s
      point a0 hint(x: o_t.x + dims.front + 10mm, y: o_t.y + sgn * (dims.camx + dims.rcamj))
      point a1 hint(x: o_t.x + dims.back - 10mm, y: o_t.y + sgn * (dims.camx + dims.rcamj))
      point b0 hint(x: o_t.x + dims.front + 10mm, y: o_t.y + sgn * (dims.camx - dims.rcamj))
      point b1 hint(x: o_t.x + dims.back - 10mm, y: o_t.y + sgn * (dims.camx - dims.rcamj))
      line outer(a0, a1)
      line inner(b0, b1)
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
      point bi hint(x: o_t.x + dims.front + 25mm + j * dims.P, y: o_t.y + dims.camx)
      point be hint(x: o_t.x + dims.front + 25mm + j * dims.P, y: o_t.y - dims.camx)
      o_t distance(dims.front + 25mm + j * dims.P, along: x) bi
      o_t distance(dims.front + 25mm + j * dims.P, along: x) be
      ci distance(0, along: y) bi
      ce distance(0, along: y) be
      cap_i: CamBearing(bi, dims: dims)
      cap_e: CamBearing(be, dims: dims)
    }
    repeat 4 as i {
      point pc hint(x: o_t.x + dims.front + 25mm + dims.P / 2 + i * dims.P, y: o_t.y)
      o_t distance(dims.front + 25mm + dims.P / 2 + i * dims.P, along: x) pc
      o_t distance(0, along: y) pc
      circle plug(center: pc) hint(r: 7mm)
      radius(7) plug
      repeat 2 as k {
        vi: At(pc, dx: -16mm + k * 32mm, dy: dims.vs)
        ve: At(pc, dx: -16mm + k * 32mm, dy: -dims.vs)
        circle intake(center: vi.p) hint(r: dims.div / 2)
        circle exhaust(center: ve.p) hint(r: dims.dev / 2)
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
