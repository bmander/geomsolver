// The frame plate: **one section, and the solid it is a section of** (§6.9).
//
// Seen along the crank axis — which is how it is designed — the plate is the face the cylinders
// rock against, with the shaft's hole, the two pivot bolts' holes, the four ports and the
// exhaust vents out through its edges; the plenum inside it joining the two intake ports; the
// foot it stands on and the bearing boss behind it; and the inlet boss on its top edge, with the
// brass coupling for the air line's plug, the passage down to the plenum, and the throttle
// (`vtwin.throttle`) across that passage.  Every one of those is this section swept along the
// crank axis, and the two that are *not* — the coupling's hole and nothing else — is a turn
// about a line lying in the page.
//
// It used to be written three times: this section, then the whole plate redrawn edge on as
// eighteen `Box`es and again from above as eight more, tied back by seven projections, with
// every ordinate along the crank axis — the plate's thickness, the foot's depth, the boss, the
// bearing pocket, how far each port is drilled — kept in step by hand from the side view's own
// origin.  They are now the `param z…` lines below, said once, and the views that show them are
// asked for.  One printed part, printed foot down.

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
  ref angle(alpha, sense: cw) axis
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
  s0 distance(a * sin(swing + 6deg), side: right) axis
  s1 distance(a * sin(swing + 6deg), side: left) axis
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

component Frame(front: plane, o: point, ref: line) {
  param mid = tp / 2
  in front {
    r: Side(o, ref, alpha: alphaR, dim: 0)
    l: Side(o, ref, alpha: alphaL, dim: 1)
    claim r.axis angle(V) l.axis class shown at (0.785, 34)
    claim rad_bR angle(alphaR) ref class detail at (0.4, 20)
    claim ref angle(alphaL, sense: cw) rad_bL class detail at (-0.4, 20)
    claim r.rad_i angle(bR_i) ref class detail at (0.35, 26)
    claim r.rad_e angle(bR_e) ref class detail at (0.5, 32)
    claim ref angle(bL_i, sense: cw) l.rad_i class detail at (0.35, 26)
    claim ref angle(bL_e, sense: cw) l.rad_e class detail at (0.5, 32)
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

    // -- what the solid is made of -----------------------------------------------------------
    // Two things the plate has always had but only the *other* views drew, and which a body
    // written from this one section must therefore say here: the foot it stands on, across the
    // bottom edge and `footd` deep, and the bearing boss round the crank axis behind it.
    ft: Box(o, x0: -fx, y0: fy0, x1: fx, y1: fy0 + footh) class hidden
    circle bcirc(center: o) hint(r: rbrg + 3mm) class hidden
    radius(rbrg + 3mm) bcirc
    // the coupling's hole is round and its axis runs *up the page* — in this plane — so it is a
    // turn, where every other hole here is drilled along the crank axis and is a sweep
    cph0: At(o, dx: 0mm, dy: bossh - cplin)
    cph1: At(o, dx: 0mm, dy: bossh)
    cph2: At(o, dx: cplhole / 2, dy: bossh)
    cph3: At(o, dx: cplhole / 2, dy: bossh - cplin)
    line cpax(cph0.p, cph1.p) class gone
    // each exhaust vent, `wch` square like the plenum and the passage it is a sibling of — the
    // drawing has always carried it as a centreline, and a solid has to be told how wide a
    // channel is.  It runs at the plate's mid-plane, which is where the port it drains ends.
    vr0: At(r.ep.p, dx: 0mm, dy: wch / 2)
    vr1: At(xr, dx: 0mm, dy: wch / 2)
    vr2: At(xr, dx: 0mm, dy: -wch / 2)
    vr3: At(r.ep.p, dx: 0mm, dy: -wch / 2)
    vl0: At(l.ep.p, dx: 0mm, dy: wch / 2)
    vl1: At(xl, dx: 0mm, dy: wch / 2)
    vl2: At(xl, dx: 0mm, dy: -wch / 2)
    vl3: At(l.ep.p, dx: 0mm, dy: -wch / 2)
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
  thr: Throttle(front, tb.p, ref, phi: throttle)

  // -- the solid: the section's faces swept, and the body their one rule (§6.9) ----------------
  // **The section is the plate's mid-plane**, and the plate says so itself: `tp` is "thick enough
  // to carry the plenum on its mid-plane", the plenum, the passage, the exhaust vents, the inlet
  // boss and the throttle's bore are all centred there, and the coupling's hole is a *turn* about
  // a line lying in this plane — which puts it on the plane whatever else is written, exactly as
  // the set screw does on the crank disc.  Drawn from the face instead, every one of those would
  // have needed half a thickness added to it and the coupling's hole would have come out
  // straddling the face.  So the zero is the middle, and the ordinates below are symmetric.
  //
  // These are the numbers the side view used to carry as `Box` ordinates measured from its own
  // origin — the same numbers, said once, where the thing they measure is defined.
  param zf = tp / 2                    // the face the cylinders rock against
  param zb = -tp / 2                   // and the back
  param zfoot = zb - footd
  param zbb = zb - boss
  param zpkt = zbb + brgpocket
  param zshaft = zbb + brgpocket
  param zch0 = -wch / 2
  param zch1 = wch / 2

  solid stock(face(bottom, edge_r, cham_r, topline, cham_l, edge_l), from: zb, to: zf)
  solid foot(ft.profile, from: zfoot, to: zb)
  solid bboss(face(bcirc), from: zbb, to: zb)
  solid iboss(boss.profile, from: -bossz / 2, to: bossz / 2)
  solid bpkt(face(bp), from: zbb, to: zpkt)
  solid shaft(face(sh), from: zshaft, to: zf)
  solid boltR(face(r.bolt), from: zb, to: zf)
  solid boltL(face(l.bolt), from: zb, to: zf)
  // the intake ports run in as far as the plenum; the exhausts only to the mid-plane, where
  // their vents out through the edges meet them
  solid portRi(face(r.intake), from: zch0, to: zf)
  solid portLi(face(l.intake), from: zch0, to: zf)
  solid portRe(face(r.exhaust), from: 0mm, to: zf)
  solid portLe(face(l.exhaust), from: 0mm, to: zf)
  // the plenum: the channel between its two arcs, closed by a radial cap at each intake port
  solid plenum(face(ch_in, co1, ch_out, ci0), from: zch0, to: zch1)
  solid passage_s(passage.profile, from: zch0, to: zch1)
  solid ventR(face(vr0.p, vr1.p, vr2.p, vr3.p, -> close), from: zch0, to: zch1)
  solid ventL(face(vl0.p, vl1.p, vl2.p, vl3.p, -> close), from: zch0, to: zch1)
  solid cplh(face(cph0.p, cph1.p, cph2.p, cph3.p, -> close), about: cpax)
  solid tbore_s(face(tbore), from: -bossz / 2, to: bossz / 2)

  solid body(stock)
  foot on body
  bboss on body
  iboss on body
  bpkt through body
  shaft through body
  boltR through body
  boltL through body
  portRi through body
  portLi through body
  portRe through body
  portLe through body
  plenum through body
  passage_s through body
  ventR through body
  ventL through body
  cplh through body
  tbore_s through body
}
