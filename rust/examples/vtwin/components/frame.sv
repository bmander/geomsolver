// The V-twin frame: plate and bearing support, two banks of ports, intake manifold,
// exhaust vents, and inlet hardware. One printed part, printed foot down.

use std
use components.dims
use components.parts
use components.throttle

// Both banks use the same port timing, rotated: intake counter-clockwise of the bank,
// exhaust clockwise, for a clockwise crank. The pivot remains available as r.piv or l.piv.
component FrameBank(o: point, ref: line, alpha: Angle, dim: Int, dims: group) {
  param port_bearing = atan2(dims.a * sin(dims.beta), dims.H + dims.a * cos(dims.beta))
  point piv hint(x: o.x + dims.H * sin(alpha), y: o.y + dims.H * cos(alpha))
  line axis(o, piv)
  o distance(dims.H) piv
  ref angle(alpha, sense: cw) axis
  circle bolt(center: piv) hint(r: dims.studclr / 2)
  radius(dims.studclr / 2) bolt
  ip: At(piv, dx: dims.a * sin(alpha - dims.beta), dy: dims.a * cos(alpha - dims.beta))
  ep: At(piv, dx: dims.a * sin(alpha + dims.beta), dy: dims.a * cos(alpha + dims.beta))
  circle intake(center: ip.p) hint(r: dims.dport / 2)
  circle exhaust(center: ep.p) hint(r: dims.dport / 2)
  radius(dims.dport / 2) intake
  radius(dims.dport / 2) exhaust
  point s0 hint(x: piv.x + dims.a * sin(alpha + dims.swing + 6deg), y: piv.y + dims.a * cos(alpha + dims.swing + 6deg))
  point s1 hint(x: piv.x + dims.a * sin(alpha - dims.swing - 6deg), y: piv.y + dims.a * cos(alpha - dims.swing - 6deg))
  arc sweep(center: piv, start: s0, end: s1) hint(r: dims.a)
  radius(dims.a) sweep
  s0 distance(dims.a * sin(dims.swing + 6deg), side: right) axis
  s1 distance(dims.a * sin(dims.swing + 6deg), side: left) axis
  line rad_i(o, ip.p)
  line rad_e(o, ep.p)
  claim ref angle(alpha, sense: cw) axis
  claim ref angle(alpha - port_bearing, sense: cw) rad_i
  claim ref angle(alpha + port_bearing, sense: cw) rad_e
  repeat dim {
    claim o distance(dims.H) piv
    claim piv distance(dims.a) ip.p
  }
  claim o distance(dims.rpl) ip.p
  claim o distance(dims.rpl) ep.p
  claim radius(dims.studclr / 2) bolt
  claim radius(dims.dport / 2) intake
}

// Plate outline, foot, and crankshaft bearing support. Depth zero is the plate's mid-plane.
component FrameBlank(o: point, dims: group) {
  param zf = dims.tp / 2
  param zb = -dims.tp / 2
  param zbb = zb - dims.boss
  param zpkt = zbb + dims.brgpocket
  p0: At(o, dx: -dims.fx, dy: dims.fy0)
  p1: At(o, dx: dims.fx, dy: dims.fy0)
  p2: At(o, dx: dims.fx, dy: dims.fy1 - dims.fch)
  p3: At(o, dx: dims.fx - dims.fch, dy: dims.fy1)
  p4: At(o, dx: -(dims.fx - dims.fch), dy: dims.fy1)
  p5: At(o, dx: -dims.fx, dy: dims.fy1 - dims.fch)
  line bottom(p0.p, p1.p) -> line edge_r(p1.p, p2.p) -> line cham_r(p2.p, p3.p) ->
    line topline(p3.p, p4.p) -> line cham_l(p4.p, p5.p) -> line edge_l(p5.p, p0.p) -> close
  claim p0.p distance(2 * dims.fx) p1.p
  claim p1.p distance(dims.fy1 - dims.fy0, along: y) p3.p
  claim p3.p distance(dims.fch, along: x) p2.p
  claim p2.p distance(dims.fch, along: y) p3.p
  // the shaft's hole, and the bearings' pocket behind
  circle sh(center: o) hint(r: dims.shafthole / 2)
  radius(dims.shafthole / 2) sh
  circle bp(center: o) hint(r: dims.rbrg)
  radius(dims.rbrg) bp
  claim radius(dims.shafthole / 2) sh
  claim radius(dims.rbrg) bp

  ft: Box(o, x0: -dims.fx, y0: dims.fy0, x1: dims.fx, y1: dims.fy0 + dims.footh)
  circle bcirc(center: o) hint(r: dims.rbrg + 3mm)
  radius(dims.rbrg + 3mm) bcirc

  solid stock(face(bottom, edge_r, cham_r, topline, cham_l, edge_l), from: zb, to: zf)
  solid foot(ft.profile, from: zb - dims.footd, to: zb)
  solid bboss(face(bcirc), from: zbb, to: zb)
  solid bpkt(face(bp), from: zbb, to: zpkt)
  solid shaft(face(sh), from: zpkt, to: zf)
}

// A radial feed from the inset plenum to an intake port. The datum points from
// the crank axis to the port; the feed overlaps both cavities at its ends.
component IntakePassage(f: plane, dims: group) {
  a: Loc(f, u: dims.rman, v: -dims.wch / 2)
  b: Loc(f, u: dims.rpl, v: -dims.wch / 2)
  c: Loc(f, u: dims.rpl, v: dims.wch / 2)
  d: Loc(f, u: dims.rman, v: dims.wch / 2)
  solid body(face(a.p, b.p, c.p, d.p, -> close), from: -dims.wch / 2, to: dims.wch / 2)
}

// Inset plenum and radial intake feeds; rman leaves a wall beside the exhaust ports.
component IntakeManifold(o: point, right: line, left: line, dims: group) {
  param kin = (dims.rman - dims.wch / 2) / dims.rpl
  param kout = (dims.rman + dims.wch / 2) / dims.rpl
  plane intake_r(origin: o, toward: right.p2)
  plane intake_l(origin: o, toward: left.p2)
  feedR: IntakePassage(intake_r, dims: dims)
  feedL: IntakePassage(intake_l, dims: dims)
  point ci0 hint(x: o.x + (left.p2.x - o.x) * kin, y: o.y + (left.p2.y - o.y) * kin)
  point ci1 hint(x: o.x + (right.p2.x - o.x) * kin, y: o.y + (right.p2.y - o.y) * kin)
  point co0 hint(x: o.x + (left.p2.x - o.x) * kout, y: o.y + (left.p2.y - o.y) * kout)
  point co1 hint(x: o.x + (right.p2.x - o.x) * kout, y: o.y + (right.p2.y - o.y) * kout)
  arc ch_in(center: o, start: ci1, end: ci0) hint(r: dims.rman - dims.wch / 2)
  arc ch_out(center: o, start: co1, end: co0) hint(r: dims.rman + dims.wch / 2)
  radius(dims.rman - dims.wch / 2) ch_in
  radius(dims.rman + dims.wch / 2) ch_out
  ci0 on left
  ci1 on right
  co0 on left
  co1 on right
  claim radius(dims.rman + dims.wch / 2) ch_out
  solid plenum(face(ch_in, co1, ch_out, ci0), from: -dims.wch / 2, to: dims.wch / 2)
}

// A square exhaust passage from a port to its nearest plate edge.
component ExhaustPassage(opening: point, edge: line, dims: group) {
  point outlet hint(x: (edge.p1.x + edge.p2.x) / 2, y: opening.y)
  outlet on edge
  opening distance(0mm, along: y) outlet
  line center(opening, outlet)
  a: At(opening, dx: 0mm, dy: dims.wch / 2)
  b: At(outlet, dx: 0mm, dy: dims.wch / 2)
  c: At(outlet, dx: 0mm, dy: -dims.wch / 2)
  d: At(opening, dx: 0mm, dy: -dims.wch / 2)
  solid body(face(a.p, b.p, c.p, d.p, -> close), from: -dims.wch / 2, to: dims.wch / 2)
}

// Inlet boss, coupling, plug outline, and the passage through the throttle bore.
component FrameInlet(o: point, dims: group) {
  param coupling_base = dims.bossh - dims.cplin
  param coupling_top = coupling_base + dims.cpll
  param plug_top = coupling_top + dims.mplug_body_l
  boss: Box(o, x0: -dims.bossw / 2, y0: dims.fy1, x1: dims.bossw / 2, y1: dims.bossh)
  cpl_in: Box(o, x0: -dims.cpl / 2, y0: coupling_base, x1: dims.cpl / 2, y1: dims.bossh)
  cpl_out: Box(o, x0: -dims.cpl / 2, y0: dims.bossh, x1: dims.cpl / 2, y1: coupling_top)
  cplh: Box(o, x0: -dims.cplhole / 2, y0: coupling_base, x1: dims.cplhole / 2, y1: dims.bossh)
  passage: Box(o, x0: -dims.wch / 2, y0: dims.rman, x1: dims.wch / 2, y1: coupling_base)
  plug_body: Box(o, x0: -dims.mplug_body_d / 2, y0: coupling_top, x1: dims.mplug_body_d / 2, y1: plug_top)
  plug_nose: Box(o, x0: -dims.mplug_nose_d / 2, y0: plug_top, x1: dims.mplug_nose_d / 2, y1: plug_top + dims.mplug_nose_l)
  claim boss.a distance(dims.bossw) boss.b
  claim boss.a distance(dims.bossh - dims.fy1, along: y) boss.d
  claim cplh.a distance(dims.cplhole) cplh.b
  claim cplh.a distance(dims.cplin, along: y) cplh.d
  claim passage.a distance(dims.wch) passage.b
  tb: At(o, dx: 0mm, dy: dims.Ty)
  circle tbore(center: tb.p) hint(r: dims.barbore / 2)
  radius(dims.barbore / 2) tbore
  claim o distance(dims.Ty, along: y) tb.p
  claim radius(dims.barbore / 2) tbore

  // The coupling bore turns about an axis in the page; the other features are sweeps.
  cph0: At(o, dx: 0mm, dy: coupling_base)
  cph1: At(o, dx: 0mm, dy: dims.bossh)
  line cpax(cph0.p, cph1.p)

  solid iboss(boss.profile, from: -dims.bossz / 2, to: dims.bossz / 2)
  solid passage_s(passage.profile, from: -dims.wch / 2, to: dims.wch / 2)
  solid cplh(face(cph0.p, cph1.p, cplh.c, cplh.b, -> close), about: cpax)
  solid tbore_s(face(tbore), from: -dims.bossz / 2, to: dims.bossz / 2)
}

// One mid-plane section supplies all solids. The part sheet chooses the projections.
component Frame(layout: group, dims: group) {
  param zf = dims.tp / 2
  param zb = -dims.tp / 2
  param zch0 = -dims.wch / 2

  in layout.front {
    blank: FrameBlank(layout.origin, dims: dims)
    r: FrameBank(layout.origin, layout.axis, alpha: dims.alphaR, dim: 0, dims: dims)
    l: FrameBank(layout.origin, layout.axis, alpha: dims.alphaL, dim: 1, dims: dims)
    claim r.axis angle(dims.V) l.axis
    air: IntakeManifold(layout.origin, r.rad_i, l.rad_i, dims: dims)
    ventR: ExhaustPassage(r.ep.p, blank.edge_r, dims: dims)
    ventL: ExhaustPassage(l.ep.p, blank.cham_l, dims: dims)
    inlet: FrameInlet(layout.origin, dims: dims)
  }
  thr: Throttle(layout.front, inlet.tb.p, layout.axis, phi: dims.throttle, dims: dims)

  solid boltR(face(r.bolt), from: zb, to: zf)
  solid boltL(face(l.bolt), from: zb, to: zf)
  // Intakes reach the plenum; exhausts reach the mid-plane vents.
  solid portRi(face(r.intake), from: zch0, to: zf)
  solid portLi(face(l.intake), from: zch0, to: zf)
  solid portRe(face(r.exhaust), from: 0mm, to: zf)
  solid portLe(face(l.exhaust), from: 0mm, to: zf)

  solid body(blank.stock)
  blank.foot on body
  blank.bboss on body
  inlet.iboss on body
  blank.bpkt cut body
  blank.shaft cut body
  boltR cut body
  boltL cut body
  portRi cut body
  portLi cut body
  portRe cut body
  portLe cut body
  air.plenum cut body
  air.feedR.body cut body
  air.feedL.body cut body
  inlet.passage_s cut body
  ventR.body cut body
  ventL.body cut body
  inlet.cplh cut body
  inlet.tbore_s cut body
}

// Open this file to preview the frame plate.
// ../plate.svd arranges three projections of this preview.
preview {
  unit mm
  line ref(std.origin, std.up.toward) in std.front
  group layout(front: std.front, origin: std.origin, axis: ref)
  plate: Frame(layout, dims: vtwin_dims)
}
