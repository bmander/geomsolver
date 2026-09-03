// The frame plate, designed in one place: one component, three views (§6.7).
//
// Along the crank axis: the plate the cylinders rock against, with the shaft's hole, the two
// pivot bolts' holes, the four ports and the exhaust passages drilled in from its edges; the
// plenum inside it joining the two intake ports; and the inlet boss on its top edge, with the
// brass coupling for the air line's plug, the passage down to the plenum, and the throttle
// (`vtwin.throttle`) across that passage.  From the side: the plate edge on with the foot and
// the bearing boss behind it, the bearing pocket, the holes and the ports going in from the
// face, the plenum and the passage in section.  From above: the plate, the foot, the bearing
// boss, the inlet boss with the coupling's hole.  The assembly draws all three, since the plate
// stands still; the part sheet draws them with the printer's dimensions on.  One printed part,
// printed foot down.

use vtwin.dims
use vtwin.parts
use vtwin.throttle

// One bank's share of the plate: the pivot `H` up the bank axis with the bolt's hole, and the
// two ports on the arc the cylinder's port sweeps — the intake counter-clockwise of the bank,
// the exhaust clockwise.  Which is which follows from the crank turning clockwise: driven
// toward the crank, a piston's cylinder has its top rocked counter-clockwise of its bank, and
// that is where its port must find air.  Both banks are this one component, so they cannot
// come out as mirror images: the engine is one bank turned a quarter turn, and its ports too.
component Side(o: point, ref: line, alpha: Angle, dim: Int) {
  point piv hint(x: o.x + H * sin(alpha), y: o.y + H * cos(alpha))
  line axis(o, piv) class axis
  o distance(H) piv
  ref angle(-alpha) axis
  circle bolt(center: piv) hint(r: studclr / 2) class hidden
  radius(studclr / 2) bolt
  ip: At(piv, dx: a * sin(alpha - beta), dy: a * cos(alpha - beta))
  ep: At(piv, dx: a * sin(alpha + beta), dy: a * cos(alpha + beta))
  circle intake(center: ip.p) hint(r: dport / 2) class hidden
  circle exhaust(center: ep.p) hint(r: dport / 2) class hidden
  radius(dport / 2) intake
  radius(dport / 2) exhaust
  // the sweep: the arc the cylinder's port travels, drawn a little past the rock either way
  point s0 hint(x: piv.x + a * sin(alpha + swing + 6deg), y: piv.y + a * cos(alpha + swing + 6deg))
  point s1 hint(x: piv.x + a * sin(alpha - swing - 6deg), y: piv.y + a * cos(alpha - swing - 6deg))
  arc sweep(center: piv, start: s0, end: s1) hint(r: a) class phantom
  radius(a) sweep
  s0 distance(-a * sin(swing + 6deg)) axis
  s1 distance(a * sin(swing + 6deg)) axis
  // the ports' radials from the crank axis: all four ports are `rpl` out, and these carry the
  // bearings the part sheet states them by
  line rad_i(o, ip.p) class gone
  line rad_e(o, ep.p) class gone
  repeat dim {
    claim o distance(H) piv class shown at (0, 40)
    claim piv distance(a) ip.p class shown at (0, 22)
  }
  claim o distance(rpl) ip.p class detail at (0, 8)
  claim o distance(rpl) ep.p class detail at (0, -8)
  claim radius(studclr / 2) bolt class detail at (1.2, 12)
  claim radius(dport / 2) intake class detail at (-1.2, 10)
}

component Frame(front: plane, side: plane, top: plane, o: point, ref: line, o_s: point, o_t: point,
                draw_side: Int, draw_top: Int) {
  param mid = tp / 2
  in front {
    r: Side(o, ref, alpha: alphaR, dim: 0)
    l: Side(o, ref, alpha: alphaL, dim: 1)
    claim r.axis angle(V) l.axis class shown at (0.785, 34)
    claim rad_bR angle(alphaR) ref class detail at (0.4, 20)
    claim ref angle(-alphaL) rad_bL class detail at (-0.4, 20)
    claim r.rad_i angle(bR_i) ref class detail at (0.35, 26)
    claim r.rad_e angle(bR_e) ref class detail at (0.5, 32)
    claim ref angle(-bL_i) l.rad_i class detail at (0.35, 26)
    claim ref angle(-bL_e) l.rad_e class detail at (0.5, 32)
    line rad_bR(o, r.piv) class gone
    line rad_bL(o, l.piv) class gone

    // the plate: a chamfered rectangle about the crank axis
    p0: At(o, dx: -fx, dy: fy0)
    p1: At(o, dx: fx, dy: fy0)
    p2: At(o, dx: fx, dy: fy1 - fch)
    p3: At(o, dx: fx - fch, dy: fy1)
    p4: At(o, dx: -(fx - fch), dy: fy1)
    p5: At(o, dx: -fx, dy: fy1 - fch)
    line bottom(p0.p, p1.p) -> line edge_r(p1.p, p2.p) -> line cham_r(p2.p, p3.p) ->
      line topline(p3.p, p4.p) -> line cham_l(p4.p, p5.p) -> line edge_l(p5.p, p0.p) -> close
    claim p0.p distance(2 * fx) p1.p class shown at (0, -10)
    claim p1.p distance(fy1 - fy0, along: y) p3.p class shown at (0, -27)
    claim p3.p distance(fch, along: x) p2.p class detail at (0, 8)
    claim p2.p distance(fch, along: y) p3.p class detail at (0, 8)
    // the shaft's hole, and the bearings' pocket behind
    circle sh(center: o) hint(r: shafthole / 2) class hidden
    radius(shafthole / 2) sh
    circle bp(center: o) hint(r: rbrg) class hidden
    radius(rbrg) bp
    claim radius(shafthole / 2) sh class detail at (-2.6, 22)
    claim radius(rbrg) bp class detail at (-2.9, 30)

    // the exhausts vent sideways: a passage from each exhaust port out through the nearest edge
    point xr hint(x: o.x + fx, y: ep_y_r)
    xr on edge_r
    r.ep.p distance(0, along: y) xr
    line exh_r(r.ep.p, xr) class hidden
    point xl hint(x: o.x - fx + 5mm, y: ep_y_l)
    xl on cham_l
    l.ep.p distance(0, along: y) xl
    line exh_l(l.ep.p, xl) class hidden

    // the plenum: a channel inside the plate, an arc about the crank axis from one intake port
    // to the other, since both are the same radius from it
    point ci0 hint(x: o.x + (l.ip.p.x - o.x) * kin, y: o.y + (l.ip.p.y - o.y) * kin)
    point ci1 hint(x: o.x + (r.ip.p.x - o.x) * kin, y: o.y + (r.ip.p.y - o.y) * kin)
    point co0 hint(x: o.x + (l.ip.p.x - o.x) * kout, y: o.y + (l.ip.p.y - o.y) * kout)
    point co1 hint(x: o.x + (r.ip.p.x - o.x) * kout, y: o.y + (r.ip.p.y - o.y) * kout)
    arc ch_in(center: o, start: ci1, end: ci0) hint(r: rpl - wch / 2) class hidden
    arc ch_out(center: o, start: co1, end: co0) hint(r: rpl + wch / 2) class hidden
    radius(rpl - wch / 2) ch_in
    radius(rpl + wch / 2) ch_out
    ci0 on l.rad_i
    ci1 on r.rad_i
    co0 on l.rad_i
    co1 on r.rad_i
    claim radius(rpl + wch / 2) ch_out class detail at (1.2, 14)

    // the inlet: the boss on the plate's top edge, the coupling set into it, the passage down
    // through the throttle to the plenum, and the plug that will click onto it, in phantom
    boss: Box(o, x0: -bossw / 2, y0: fy1, x1: bossw / 2, y1: bossh)
    cpl_in: Box(o, x0: -cpl / 2, y0: bossh - cplin, x1: cpl / 2, y1: bossh) class hidden
    cpl_out: Box(o, x0: -cpl / 2, y0: bossh, x1: cpl / 2, y1: bossh - cplin + cpll)
    cplh: Box(o, x0: -cplhole / 2, y0: bossh - cplin, x1: cplhole / 2, y1: bossh) class hidden
    passage: Box(o, x0: -wch / 2, y0: rpl + wch / 2, x1: wch / 2, y1: bossh - cplin) class hidden
    plug_body: Box(o, x0: -mplug_body_d / 2, y0: bossh - cplin + cpll, x1: mplug_body_d / 2, y1: bossh - cplin + cpll + mplug_body_l) class phantom
    plug_nose: Box(o, x0: -mplug_nose_d / 2, y0: bossh - cplin + cpll + mplug_body_l, x1: mplug_nose_d / 2, y1: bossh - cplin + cpll + mplug_body_l + mplug_nose_l) class phantom
    claim boss.a distance(bossw) boss.b class detail at (0, -6)
    claim boss.a distance(bossh - fy1, along: y) boss.d class detail at (0, 8)
    claim cplh.a distance(cplhole) cplh.b class detail at (0, 6)
    claim cplh.a distance(cplin, along: y) cplh.d class detail at (0, -6)
    claim passage.a distance(wch) passage.b class detail at (0, -6)
    tb: At(o, dx: 0mm, dy: Ty)
    circle tbore(center: tb.p) hint(r: barbore / 2) class hidden
    radius(barbore / 2) tbore
    claim o distance(Ty, along: y) tb.p class detail at (0, 20)
    claim radius(barbore / 2) tbore class detail at (2.2, 10)
  }
  param kin = (rpl - wch / 2) / rpl
  param kout = (rpl + wch / 2) / rpl
  param ep_y_r = H * cos(alphaR) + a * cos(alphaR + beta)
  param ep_y_l = H * cos(alphaL) + a * cos(alphaL + beta)
  // the ports' bearings from the crank axis, clockwise from up
  param bR_i = atan2(H * sin(alphaR) + a * sin(alphaR - beta), H * cos(alphaR) + a * cos(alphaR - beta))
  param bR_e = atan2(H * sin(alphaR) + a * sin(alphaR + beta), H * cos(alphaR) + a * cos(alphaR + beta))
  param bL_i = atan2(H * sin(alphaL) + a * sin(alphaL - beta), H * cos(alphaL) + a * cos(alphaL - beta))
  param bL_e = atan2(H * sin(alphaL) + a * sin(alphaL + beta), H * cos(alphaL) + a * cos(alphaL + beta))

  // the throttle in its bore, turned to the table's `throttle`; drawn along the axis only, its
  // other views being its own sheet's
  thr: Throttle(front, side, top, tb.p, ref,
                phi: throttle, o_s: o_s, o_t: o_t, draw_side: 0, draw_top: 0)

  // -- from the side: the plate edge on, the foot and the boss behind it, everything drilled
  // from the face going in ------------------------------------------------------------------
  repeat draw_side {
    in side {
      point ptop hint(x: o_s.x + mid, y: o_s.y + fy1)
      point pbot hint(x: o_s.x + mid, y: o_s.y + fy0)
      point pv_s hint(x: o_s.x + mid, y: o_s.y + H * cos(alphaR))
      point ir_s hint(x: o_s.x + mid, y: o_s.y + H * cos(alphaR) + a * cos(alphaR - beta))
      point il_s hint(x: o_s.x + mid, y: o_s.y + H * cos(alphaL) + a * cos(alphaL - beta))
      point er_s hint(x: o_s.x + mid, y: o_s.y + ep_y_r)
      point el_s hint(x: o_s.x + mid, y: o_s.y + ep_y_l)
      o_s distance(mid, along: x) ptop
      o_s distance(mid, along: x) pbot
      o_s distance(mid, along: x) pv_s
      o_s distance(mid, along: x) ir_s
      o_s distance(mid, along: x) il_s
      o_s distance(mid, along: x) er_s
      o_s distance(mid, along: x) el_s
      plate: Slab(o_s, x0: 0mm, x1: tp, top: ptop, bottom: pbot)
      foot: Box(o_s, x0: tp, y0: fy0, x1: tp + footd, y1: fy0 + footh)
      bboss: Box(o_s, x0: tp, y0: -(rbrg + 3mm), x1: tp + boss, y1: rbrg + 3mm)
      bpkt: Box(o_s, x0: tp + boss - brgpocket, y0: -rbrg, x1: tp + boss, y1: rbrg) class hidden
      shaft_s: Box(o_s, x0: 0mm, y0: -shafthole / 2, x1: tp + boss - brgpocket, y1: shafthole / 2) class hidden
      bolt_s: Box(pv_s, x0: -mid, y0: -studclr / 2, x1: mid, y1: studclr / 2) class hidden
      // the intake ports go in to the plenum, the exhausts to the mid-plane where their passages
      // out through the edges meet them
      irp: Box(ir_s, x0: -mid, y0: -dport / 2, x1: -mid + mid + wch / 2, y1: dport / 2) class hidden
      ilp: Box(il_s, x0: -mid, y0: -dport / 2, x1: -mid + mid + wch / 2, y1: dport / 2) class hidden
      erp: Box(er_s, x0: -mid, y0: -dport / 2, x1: 0mm, y1: dport / 2) class hidden
      elp: Box(el_s, x0: -mid, y0: -dport / 2, x1: 0mm, y1: dport / 2) class hidden
      plenum: Box(o_s, x0: mid - wch / 2, y0: rpl - wch / 2, x1: mid + wch / 2, y1: rpl + wch / 2) class hidden
      passage_s: Box(o_s, x0: mid - wch / 2, y0: rpl + wch / 2, x1: mid + wch / 2, y1: bossh - cplin) class hidden
      boss_s: Box(o_s, x0: mid - bossz / 2, y0: fy1, x1: mid + bossz / 2, y1: bossh)
      cplh_s: Box(o_s, x0: mid - cplhole / 2, y0: bossh - cplin, x1: mid + cplhole / 2, y1: bossh) class hidden
      cpl_s: Box(o_s, x0: mid - cpl / 2, y0: bossh, x1: mid + cpl / 2, y1: bossh - cplin + cpll)
      tbore_s: Box(o_s, x0: mid - bossz / 2, y0: Ty - barbore / 2, x1: mid + bossz / 2, y1: Ty + barbore / 2) class hidden
      claim plate.a distance(tp) plate.b class detail at (0, -8)
      claim foot.a distance(footd) foot.b class detail at (0, -8)
      claim foot.a distance(footh, along: y) foot.d class detail at (0, 6)
      claim bboss.a distance(boss) bboss.b class detail at (0, -6)
      claim bboss.a distance(2 * (rbrg + 3mm), along: y) bboss.d class detail at (0, -8)
      claim bpkt.a distance(brgpocket) bpkt.b class detail at (0, 6)
      claim boss_s.a distance(bossz) boss_s.b class detail at (0, 8)
    }
    p3.p project ptop
    p0.p project pbot
    r.piv project pv_s
    r.ip.p project ir_s
    l.ip.p project il_s
    r.ep.p project er_s
    l.ep.p project el_s
  }

  // -- from above: the plate's top edge with the boss on it, the foot and the bearing boss
  // behind --------------------------------------------------------------------------------------
  repeat draw_top {
    in top {
      point fl_t hint(x: o_t.x - fx, y: o_t.y + mid)
      point fr_t hint(x: o_t.x + fx, y: o_t.y + mid)
      o_t distance(mid, along: y) fl_t
      o_t distance(mid, along: y) fr_t
      plate_t: Wide(o_t, y0: 0mm, y1: tp, left: fl_t, right: fr_t)
      foot_t: Wide(o_t, y0: tp, y1: tp + footd, left: fl_t, right: fr_t)
      bboss_t: Box(o_t, x0: -(rbrg + 3mm), y0: tp, x1: rbrg + 3mm, y1: tp + boss)
      boss_t: Box(o_t, x0: -bossw / 2, y0: mid - bossz / 2, x1: bossw / 2, y1: mid + bossz / 2)
      cc_t: At(o_t, dx: 0mm, dy: mid)
      circle cplh_t(center: cc_t.p) hint(r: cplhole / 2)
      radius(cplhole / 2) cplh_t
      tbore_t: Box(o_t, x0: -barbore / 2, y0: mid - bossz / 2, x1: barbore / 2, y1: mid + bossz / 2) class hidden
      boltR_t: Box(o_t, x0: H * sin(alphaR) - studclr / 2, y0: 0mm, x1: H * sin(alphaR) + studclr / 2, y1: tp) class hidden
      boltL_t: Box(o_t, x0: H * sin(alphaL) - studclr / 2, y0: 0mm, x1: H * sin(alphaL) + studclr / 2, y1: tp) class hidden
    }
    p0.p project fl_t
    p1.p project fr_t
  }
}
