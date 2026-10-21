// The V-twin frame: plate and bearing support, two banks of ports, intake manifold,
// exhaust vents, and inlet hardware. One printed part, printed foot down.

use std (horizontal)
use hardware (mplug_body_d, mplug_body_l, mplug_nose_d, mplug_nose_l)
use components.dims (H, Ty, V, a, alphaL, alphaR, beta, boss, bossh, bossw, bossz, brgpocket, cpl,
                     cplhole, cplin, cpll, dport, fch, footd, footh, fx, fy0, fy1, rbar, rbrg, rman,
                     rpl, shafthole, studclr, swing, throttle, tp, wch)
use components.parts
use components.throttle

// Both banks use the same port timing, rotated: intake counter-clockwise of the bank,
// exhaust clockwise, for a clockwise crank. The frame names both pivots, `pivots.r` and `pivots.l`.
part FrameBank(o: point, ref: line, alpha: Angle) {
  port_bearing := atan2(a * sin(beta), H + a * cos(beta))
  piv := point hint((o.x + H * sin(alpha), o.y + H * cos(alpha)))
  ax := line(o, piv)
  o distance(H) piv
  ref angle(alpha, sense: cw) ax
  bolt := circle(center: piv) hint(r: studclr / 2)
  radius(studclr / 2) bolt
  ip := components.parts.At(piv, dx: a * sin(alpha - beta), dy: a * cos(alpha - beta))
  ep := components.parts.At(piv, dx: a * sin(alpha + beta), dy: a * cos(alpha + beta))
  intake := circle(center: ip.p) hint(r: dport / 2)
  exhaust := circle(center: ep.p) hint(r: dport / 2)
  radius(dport / 2) intake
  radius(dport / 2) exhaust
  s0 := point hint((piv.x + a * sin(alpha + swing + 6deg), piv.y + a * cos(alpha + swing + 6deg)))
  s1 := point hint((piv.x + a * sin(alpha - swing - 6deg), piv.y + a * cos(alpha - swing - 6deg)))
  sweep := arc(center: piv, start: s0, end: s1) hint(r: a)
  radius(a) sweep
  s0 distance(a * sin(swing + 6deg), side: right) ax
  s1 distance(a * sin(swing + 6deg), side: left) ax
  // the sweep is the arc beyond the pivot, away from the crank
  piv distance(>= 0, along: ax) s0
  piv distance(>= 0, along: ax) s1
  rad_i := line(o, ip.p)
  rad_e := line(o, ep.p)
  claim ref angle(alpha, sense: cw) ax
  claim ref angle(alpha - port_bearing, sense: cw) rad_i
  claim ref angle(alpha + port_bearing, sense: cw) rad_e
  claim o distance(H) piv
  claim piv distance(a) ip.p
  claim o distance(rpl) ip.p
  claim o distance(rpl) ep.p
  claim radius(studclr / 2) bolt
  claim radius(dport / 2) intake
}

// Plate outline, foot, and crankshaft bearing support. Depth zero is the plate's mid-plane.
part FrameBlank(o: point) {
  zf := tp / 2
  zb := -tp / 2
  zbb := zb - boss
  zpkt := zbb + brgpocket
  p0 := components.parts.At(o, dx: -fx, dy: fy0)
  p1 := components.parts.At(o, dx: fx, dy: fy0)
  p2 := components.parts.At(o, dx: fx, dy: fy1 - fch)
  p3 := components.parts.At(o, dx: fx - fch, dy: fy1)
  p4 := components.parts.At(o, dx: -(fx - fch), dy: fy1)
  p5 := components.parts.At(o, dx: -fx, dy: fy1 - fch)
  (bottom := line(p0.p, p1.p)) -> (edge_r := line(p1.p, p2.p)) -> (cham_r := line(p2.p, p3.p)) ->
    (topline := line(p3.p, p4.p)) -> (cham_l := line(p4.p, p5.p)) -> (edge_l := line(p5.p, p0.p)) -> close
  claim p0.p distance(2 * fx) p1.p
  claim p1.p distance(fy1 - fy0, along: v) p3.p
  claim p3.p distance(fch, along: u) p2.p
  claim p2.p distance(fch, along: v) p3.p
  // the shaft's hole, and the bearings' pocket behind
  sh := circle(center: o) hint(r: shafthole / 2)
  radius(shafthole / 2) sh
  bp := circle(center: o) hint(r: rbrg)
  radius(rbrg) bp
  claim radius(shafthole / 2) sh
  claim radius(rbrg) bp

  ft := components.parts.Box(o, x0: -fx, y0: fy0, x1: fx, y1: fy0 + footh)
  bcirc := circle(center: o) hint(r: rbrg + 3mm)
  radius(rbrg + 3mm) bcirc

  stock := solid(face(bottom, edge_r, cham_r, topline, cham_l, edge_l), from: zb, to: zf)
  foot := solid(ft.profile, from: zb - footd, to: zb)
  bboss := solid(face(bcirc), from: zbb, to: zb)
  bpkt := solid(face(bp), from: zbb, to: zpkt)
  shaft := solid(face(sh), from: zpkt, to: zf)
}

// A radial feed from the inset plenum to an intake port. The datum points from
// the crank axis to the port; the feed overlaps both cavities at its ends.
part IntakePassage(f: group) {
  ax := line(f.u.p1, f.u.p2)
  a := point hint(at: f.axes, (rman, -wch / 2))
  b := point hint(at: f.axes, (rpl, -wch / 2))
  c := point hint(at: f.axes, (rpl, wch / 2))
  d := point hint(at: f.axes, (rman, wch / 2))
  body := solid(face(a, b, c, d, -> close), from: -wch / 2, to: wch / 2)
  ab := line(a, b)
  bc := line(b, c)
  cd := line(c, d)
  da := line(d, a)
  ab parallel ax
  bc perpendicular ax
  cd parallel ax
  da perpendicular ax
  length(rpl - rman) ab
  length(wch) bc
  a distance(wch / 2, side: right) ax
  a distance(rman, along: u) f.axes
  // out along the feed's axis, from its right side to its left
  a distance(>= 0, along: ax) b
  ccw(f.u.p1, f.u.p2, c)
}

// Inset plenum and radial intake feeds; rman leaves a wall beside the exhaust ports.
part IntakeManifold(o: point, right: line, left: line) {
  kin := (rman - wch / 2) / rpl
  kout := (rman + wch / 2) / rpl
  intake_r := std.Turned(o, right.p2)
  intake_l := std.Turned(o, left.p2)
  feedR := IntakePassage(intake_r)
  feedL := IntakePassage(intake_l)
  ci0 := point hint((o.x + (left.p2.x - o.x) * kin, o.y + (left.p2.y - o.y) * kin))
  ci1 := point hint((o.x + (right.p2.x - o.x) * kin, o.y + (right.p2.y - o.y) * kin))
  co0 := point hint((o.x + (left.p2.x - o.x) * kout, o.y + (left.p2.y - o.y) * kout))
  co1 := point hint((o.x + (right.p2.x - o.x) * kout, o.y + (right.p2.y - o.y) * kout))
  ch_in := arc(center: o, start: ci1, end: ci0) hint(r: rman - wch / 2)
  ch_out := arc(center: o, start: co1, end: co0) hint(r: rman + wch / 2)
  radius(rman - wch / 2) ch_in
  radius(rman + wch / 2) ch_out
  ci0 coincident left
  ci1 coincident right
  co0 coincident left
  co1 coincident right
  // each end on its port's side of the crank axis
  o distance(>= 0, along: left) ci0
  o distance(>= 0, along: right) ci1
  o distance(>= 0, along: left) co0
  o distance(>= 0, along: right) co1
  claim radius(rman + wch / 2) ch_out
  plenum := solid(face(ch_in, co1, ch_out, ci0), from: -wch / 2, to: wch / 2)
  // what the frame cuts besides the plenum: each bank's feed
  feeds := {r: feedR.body, l: feedL.body}
}

// A square exhaust passage from a port to its nearest plate edge.
part ExhaustPassage(opening: point, boundary: line) {
  outlet := point hint(((boundary.p1.x + boundary.p2.x) / 2, opening.y))
  outlet coincident boundary
  opening horizontal outlet
  center := line(opening, outlet)
  a := components.parts.At(opening, dx: 0mm, dy: wch / 2)
  b := components.parts.At(outlet, dx: 0mm, dy: wch / 2)
  c := components.parts.At(outlet, dx: 0mm, dy: -wch / 2)
  d := components.parts.At(opening, dx: 0mm, dy: -wch / 2)
  body := solid(face(a.p, b.p, c.p, d.p, -> close), from: -wch / 2, to: wch / 2)
}

// Inlet boss, coupling, plug outline, and the passage through the throttle bore.
part FrameInlet(o: point) {
  barbore := 2 * rbar + 0.2mm   // diametral running clearance
  coupling_base := bossh - cplin
  coupling_top := coupling_base + cpll
  plug_top := coupling_top + mplug_body_l
  boss := components.parts.Box(o, x0: -bossw / 2, y0: fy1, x1: bossw / 2, y1: bossh)
  cpl_in := components.parts.Box(o, x0: -cpl / 2, y0: coupling_base, x1: cpl / 2, y1: bossh)
  cpl_out := components.parts.Box(o, x0: -cpl / 2, y0: bossh, x1: cpl / 2, y1: coupling_top)
  cplh := components.parts.Box(o, x0: -cplhole / 2, y0: coupling_base, x1: cplhole / 2, y1: bossh)
  passage := components.parts.Box(o, x0: -wch / 2, y0: rman, x1: wch / 2, y1: coupling_base)
  plug_body := components.parts.Box(o, x0: -mplug_body_d / 2, y0: coupling_top, x1: mplug_body_d / 2, y1: plug_top)
  plug_nose := components.parts.Box(o, x0: -mplug_nose_d / 2, y0: plug_top, x1: mplug_nose_d / 2, y1: plug_top + mplug_nose_l)
  claim boss.a distance(bossw) boss.b
  claim boss.a distance(bossh - fy1, along: v) boss.d
  claim cplh.a distance(cplhole) cplh.b
  claim cplh.a distance(cplin, along: v) cplh.d
  claim passage.a distance(wch) passage.b
  tb := components.parts.At(o, dx: 0mm, dy: Ty)
  tbore := circle(center: tb.p) hint(r: barbore / 2)
  radius(barbore / 2) tbore
  claim o distance(Ty, along: v) tb.p
  claim radius(barbore / 2) tbore

  // The coupling bore turns about an axis in the page; the other features are sweeps.
  cph0 := components.parts.At(o, dx: 0mm, dy: coupling_base)
  cph1 := components.parts.At(o, dx: 0mm, dy: bossh)
  cpax := line(cph0.p, cph1.p)

  iboss := solid(boss.profile, from: -bossz / 2, to: bossz / 2)
  passage_s := solid(passage.profile, from: -wch / 2, to: wch / 2)
  cplh := solid(face(cph0.p, cph1.p, cplh.c, cplh.b, -> close), about: cpax)
  tbore_s := solid(face(tbore), from: -bossz / 2, to: bossz / 2)
}

// One mid-plane section supplies all solids. The part sheet chooses the projections.
part Frame(o: point, ref: line) {
  zf := tp / 2
  zb := -tp / 2
  zch0 := -wch / 2

  in o.plane {
    blank := FrameBlank(o)
    r := FrameBank(o, ref, alpha: alphaR)
    l := FrameBank(o, ref, alpha: alphaL)
    claim r.ax angle(V) l.ax
    air := IntakeManifold(o, r.rad_i, l.rad_i)
    ventR := ExhaustPassage(r.exhaust.center, blank.edge_r)
    ventL := ExhaustPassage(l.exhaust.center, blank.cham_l)
    inlet := FrameInlet(o)
  }
  // each bank's pivot, for what swings on it
  pivots := {r: r.piv, l: l.piv}
  thr := components.throttle.Throttle(inlet.tbore.center, ref, phi: throttle)

  boltR := solid(face(r.bolt), from: zb, to: zf)
  boltL := solid(face(l.bolt), from: zb, to: zf)
  // Intakes reach the plenum; exhausts reach the mid-plane vents.
  portRi := solid(face(r.intake), from: zch0, to: zf)
  portLi := solid(face(l.intake), from: zch0, to: zf)
  portRe := solid(face(r.exhaust), from: 0mm, to: zf)
  portLe := solid(face(l.exhaust), from: 0mm, to: zf)

  body := solid(blank.stock)
  blank.foot union body
  blank.bboss union body
  inlet.iboss union body
  blank.bpkt cut body
  blank.shaft cut body
  boltR cut body
  boltL cut body
  portRi cut body
  portLi cut body
  portRe cut body
  portLe cut body
  air.plenum cut body
  air.feeds.r cut body
  air.feeds.l cut body
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
  in std.front {
    ref := line(std.origin, hint((0, 1)))
    fix((0, 1)) ref.p2
  }
  plate := Frame(std.origin, ref)
}
