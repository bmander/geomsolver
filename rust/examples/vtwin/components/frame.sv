// The V-twin frame: plate and bearing support, two banks of ports, intake manifold,
// exhaust vents, and inlet hardware. One printed part, printed foot down.

use std
use components.dims
use components.parts
use components.throttle

// Both banks use the same port timing, rotated: intake counter-clockwise of the bank,
// exhaust clockwise, for a clockwise crank. The pivot remains available as r.piv or l.piv.
component FrameBank(o: point, ref: line, alpha: Angle, dim: Int, dims: group) {
  port_bearing := atan2(dims.a * sin(dims.beta), dims.H + dims.a * cos(dims.beta))
  piv := point hint(x: o.x + dims.H * sin(alpha), y: o.y + dims.H * cos(alpha))
  axis := line(o, piv)
  o distance(dims.H) piv
  ref angle(alpha, sense: cw) axis
  bolt := circle(center: piv) hint(r: dims.studclr / 2)
  radius(dims.studclr / 2) bolt
  ip := components.parts.At(piv, dx: dims.a * sin(alpha - dims.beta), dy: dims.a * cos(alpha - dims.beta))
  ep := components.parts.At(piv, dx: dims.a * sin(alpha + dims.beta), dy: dims.a * cos(alpha + dims.beta))
  intake := circle(center: ip.p) hint(r: dims.dport / 2)
  exhaust := circle(center: ep.p) hint(r: dims.dport / 2)
  radius(dims.dport / 2) intake
  radius(dims.dport / 2) exhaust
  s0 := point hint(x: piv.x + dims.a * sin(alpha + dims.swing + 6deg), y: piv.y + dims.a * cos(alpha + dims.swing + 6deg))
  s1 := point hint(x: piv.x + dims.a * sin(alpha - dims.swing - 6deg), y: piv.y + dims.a * cos(alpha - dims.swing - 6deg))
  sweep := arc(center: piv, start: s0, end: s1) hint(r: dims.a)
  radius(dims.a) sweep
  s0 distance(dims.a * sin(dims.swing + 6deg), side: right) axis
  s1 distance(dims.a * sin(dims.swing + 6deg), side: left) axis
  rad_i := line(o, ip.p)
  rad_e := line(o, ep.p)
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
  zf := dims.tp / 2
  zb := -dims.tp / 2
  zbb := zb - dims.boss
  zpkt := zbb + dims.brgpocket
  p0 := components.parts.At(o, dx: -dims.fx, dy: dims.fy0)
  p1 := components.parts.At(o, dx: dims.fx, dy: dims.fy0)
  p2 := components.parts.At(o, dx: dims.fx, dy: dims.fy1 - dims.fch)
  p3 := components.parts.At(o, dx: dims.fx - dims.fch, dy: dims.fy1)
  p4 := components.parts.At(o, dx: -(dims.fx - dims.fch), dy: dims.fy1)
  p5 := components.parts.At(o, dx: -dims.fx, dy: dims.fy1 - dims.fch)
  (bottom := line(p0.p, p1.p)) -> (edge_r := line(p1.p, p2.p)) -> (cham_r := line(p2.p, p3.p)) ->
    (topline := line(p3.p, p4.p)) -> (cham_l := line(p4.p, p5.p)) -> (edge_l := line(p5.p, p0.p)) -> close
  claim p0.p distance(2 * dims.fx) p1.p
  claim p1.p distance(dims.fy1 - dims.fy0, along: y) p3.p
  claim p3.p distance(dims.fch, along: x) p2.p
  claim p2.p distance(dims.fch, along: y) p3.p
  // the shaft's hole, and the bearings' pocket behind
  sh := circle(center: o) hint(r: dims.shafthole / 2)
  radius(dims.shafthole / 2) sh
  bp := circle(center: o) hint(r: dims.rbrg)
  radius(dims.rbrg) bp
  claim radius(dims.shafthole / 2) sh
  claim radius(dims.rbrg) bp

  ft := components.parts.Box(o, x0: -dims.fx, y0: dims.fy0, x1: dims.fx, y1: dims.fy0 + dims.footh)
  bcirc := circle(center: o) hint(r: dims.rbrg + 3mm)
  radius(dims.rbrg + 3mm) bcirc

  stock := solid(face(bottom, edge_r, cham_r, topline, cham_l, edge_l), from: zb, to: zf)
  foot := solid(ft.profile, from: zb - dims.footd, to: zb)
  bboss := solid(face(bcirc), from: zbb, to: zb)
  bpkt := solid(face(bp), from: zbb, to: zpkt)
  shaft := solid(face(sh), from: zpkt, to: zf)
}

// A radial feed from the inset plenum to an intake port. The datum points from
// the crank axis to the port; the feed overlaps both cavities at its ends.
component IntakePassage(f: plane, dims: group) {
  axis := line(f.origin, f.toward)
  a := point hint(x: f.origin.x + (dims.rman) * f.c - (-dims.wch / 2) * f.s,
                    y: f.origin.y + (dims.rman) * f.s + (-dims.wch / 2) * f.c)
  b := point hint(x: f.origin.x + (dims.rpl) * f.c - (-dims.wch / 2) * f.s,
                    y: f.origin.y + (dims.rpl) * f.s + (-dims.wch / 2) * f.c)
  c := point hint(x: f.origin.x + (dims.rpl) * f.c - (dims.wch / 2) * f.s,
                    y: f.origin.y + (dims.rpl) * f.s + (dims.wch / 2) * f.c)
  d := point hint(x: f.origin.x + (dims.rman) * f.c - (dims.wch / 2) * f.s,
                    y: f.origin.y + (dims.rman) * f.s + (dims.wch / 2) * f.c)
  body := solid(face(a, b, c, d, -> close), from: -dims.wch / 2, to: dims.wch / 2)
  ab := line(a, b)
  bc := line(b, c)
  cd := line(c, d)
  da := line(d, a)
  ab parallel axis
  bc perpendicular axis
  cd parallel axis
  da perpendicular axis
  distance(dims.rpl - dims.rman) ab
  distance(dims.wch) bc
  a distance(dims.wch / 2, side: right) axis
  a distance(dims.rman, along: u) f
}

// Inset plenum and radial intake feeds; rman leaves a wall beside the exhaust ports.
component IntakeManifold(o: point, right: line, left: line, dims: group) {
  kin := (dims.rman - dims.wch / 2) / dims.rpl
  kout := (dims.rman + dims.wch / 2) / dims.rpl
  intake_r := plane(origin: o, toward: right.p2)
  intake_l := plane(origin: o, toward: left.p2)
  feedR := IntakePassage(intake_r, dims: dims)
  feedL := IntakePassage(intake_l, dims: dims)
  ci0 := point hint(x: o.x + (left.p2.x - o.x) * kin, y: o.y + (left.p2.y - o.y) * kin)
  ci1 := point hint(x: o.x + (right.p2.x - o.x) * kin, y: o.y + (right.p2.y - o.y) * kin)
  co0 := point hint(x: o.x + (left.p2.x - o.x) * kout, y: o.y + (left.p2.y - o.y) * kout)
  co1 := point hint(x: o.x + (right.p2.x - o.x) * kout, y: o.y + (right.p2.y - o.y) * kout)
  ch_in := arc(center: o, start: ci1, end: ci0) hint(r: dims.rman - dims.wch / 2)
  ch_out := arc(center: o, start: co1, end: co0) hint(r: dims.rman + dims.wch / 2)
  radius(dims.rman - dims.wch / 2) ch_in
  radius(dims.rman + dims.wch / 2) ch_out
  ci0 on left
  ci1 on right
  co0 on left
  co1 on right
  claim radius(dims.rman + dims.wch / 2) ch_out
  plenum := solid(face(ch_in, co1, ch_out, ci0), from: -dims.wch / 2, to: dims.wch / 2)
}

// A square exhaust passage from a port to its nearest plate edge.
component ExhaustPassage(opening: point, boundary: line, dims: group) {
  outlet := point hint(x: (boundary.p1.x + boundary.p2.x) / 2, y: opening.y)
  outlet on boundary
  opening distance(0mm, along: y) outlet
  center := line(opening, outlet)
  a := components.parts.At(opening, dx: 0mm, dy: dims.wch / 2)
  b := components.parts.At(outlet, dx: 0mm, dy: dims.wch / 2)
  c := components.parts.At(outlet, dx: 0mm, dy: -dims.wch / 2)
  d := components.parts.At(opening, dx: 0mm, dy: -dims.wch / 2)
  body := solid(face(a.p, b.p, c.p, d.p, -> close), from: -dims.wch / 2, to: dims.wch / 2)
}

// Inlet boss, coupling, plug outline, and the passage through the throttle bore.
component FrameInlet(o: point, dims: group) {
  barbore := 2 * dims.rbar + 0.2mm   // diametral running clearance
  coupling_base := dims.bossh - dims.cplin
  coupling_top := coupling_base + dims.cpll
  plug_top := coupling_top + dims.mplug_body_l
  boss := components.parts.Box(o, x0: -dims.bossw / 2, y0: dims.fy1, x1: dims.bossw / 2, y1: dims.bossh)
  cpl_in := components.parts.Box(o, x0: -dims.cpl / 2, y0: coupling_base, x1: dims.cpl / 2, y1: dims.bossh)
  cpl_out := components.parts.Box(o, x0: -dims.cpl / 2, y0: dims.bossh, x1: dims.cpl / 2, y1: coupling_top)
  cplh := components.parts.Box(o, x0: -dims.cplhole / 2, y0: coupling_base, x1: dims.cplhole / 2, y1: dims.bossh)
  passage := components.parts.Box(o, x0: -dims.wch / 2, y0: dims.rman, x1: dims.wch / 2, y1: coupling_base)
  plug_body := components.parts.Box(o, x0: -dims.mplug_body_d / 2, y0: coupling_top, x1: dims.mplug_body_d / 2, y1: plug_top)
  plug_nose := components.parts.Box(o, x0: -dims.mplug_nose_d / 2, y0: plug_top, x1: dims.mplug_nose_d / 2, y1: plug_top + dims.mplug_nose_l)
  claim boss.a distance(dims.bossw) boss.b
  claim boss.a distance(dims.bossh - dims.fy1, along: y) boss.d
  claim cplh.a distance(dims.cplhole) cplh.b
  claim cplh.a distance(dims.cplin, along: y) cplh.d
  claim passage.a distance(dims.wch) passage.b
  tb := components.parts.At(o, dx: 0mm, dy: dims.Ty)
  tbore := circle(center: tb.p) hint(r: barbore / 2)
  radius(barbore / 2) tbore
  claim o distance(dims.Ty, along: y) tb.p
  claim radius(barbore / 2) tbore

  // The coupling bore turns about an axis in the page; the other features are sweeps.
  cph0 := components.parts.At(o, dx: 0mm, dy: coupling_base)
  cph1 := components.parts.At(o, dx: 0mm, dy: dims.bossh)
  cpax := line(cph0.p, cph1.p)

  iboss := solid(boss.profile, from: -dims.bossz / 2, to: dims.bossz / 2)
  passage_s := solid(passage.profile, from: -dims.wch / 2, to: dims.wch / 2)
  cplh := solid(face(cph0.p, cph1.p, cplh.c, cplh.b, -> close), about: cpax)
  tbore_s := solid(face(tbore), from: -dims.bossz / 2, to: dims.bossz / 2)
}

// One mid-plane section supplies all solids. The part sheet chooses the projections.
component Frame(layout: group, dims: group) {
  zf := dims.tp / 2
  zb := -dims.tp / 2
  zch0 := -dims.wch / 2

  in layout.front {
    blank := FrameBlank(layout.origin, dims: dims)
    r := FrameBank(layout.origin, layout.axis, alpha: dims.alphaR, dim: 0, dims: dims)
    l := FrameBank(layout.origin, layout.axis, alpha: dims.alphaL, dim: 1, dims: dims)
    claim r.axis angle(dims.V) l.axis
    air := IntakeManifold(layout.origin, r.rad_i, l.rad_i, dims: dims)
    ventR := ExhaustPassage(r.ep.p, blank.edge_r, dims: dims)
    ventL := ExhaustPassage(l.ep.p, blank.cham_l, dims: dims)
    inlet := FrameInlet(layout.origin, dims: dims)
  }
  thr := components.throttle.Throttle(layout.front, inlet.tb.p, layout.axis, phi: dims.throttle, dims: dims)

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
  ref := line(std.origin, std.up.toward) in std.front
  layout := group(front: std.front, origin: std.origin, axis: ref)
  plate := Frame(layout, dims: components.dims.vtwin_dims)
}
