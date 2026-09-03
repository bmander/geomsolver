// The frame, designed in one place, seen along the crank axis: the plate the cylinders rock
// against, with the crank's bearing bore, the two pivot studs, the four ports and the exhaust
// passages drilled in from its edges; the plenum inside it joining the two intake ports; and
// the inlet boss on its top edge, with the brass coupling for the air line's plug, the passage
// down to the plenum, and the throttle barrel across that passage with its lever on the front.

use vtwin.dims
use vtwin.parts

// One bank's share of the plate: the pivot `H` up the bank axis with the stud through it, and
// the two ports on the arc the cylinder's port sweeps — the intake counter-clockwise of the
// bank, the exhaust clockwise.  Which is which follows from the crank turning clockwise: driven
// toward the crank, a piston's cylinder has its top rocked counter-clockwise of its bank, and
// that is where its port must find air.  Both banks are this one component, so they cannot
// come out as mirror images: the engine is one bank turned a quarter turn, and its ports too.
component Side(o: point, ref: line, alpha: Angle, dim: Int) {
  point piv hint(x: o.x + H * sin(alpha), y: o.y + H * cos(alpha))
  line axis(o, piv) class axis
  o distance(H) piv
  ref angle(-alpha) axis
  circle stud(center: piv) hint(r: rstud) class hidden
  radius(rstud) stud
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
  repeat dim {
    claim o distance(H) piv class shown at (0, 40)
    claim piv distance(a) ip.p class shown at (0, 22)
  }
}

component Frame(o: point, ref: line) {
  r: Side(o, ref, alpha: alphaR, dim: 0)
  l: Side(o, ref, alpha: alphaL, dim: 1)
  claim r.axis angle(V) l.axis class shown at (0.785, 34)

  // -- the plate: a chamfered rectangle about the crank axis ---------------------------------
  p0: At(o, dx: -fx, dy: fy0)
  p1: At(o, dx: fx, dy: fy0)
  p2: At(o, dx: fx, dy: fy1 - fch)
  p3: At(o, dx: fx - fch, dy: fy1)
  p4: At(o, dx: -(fx - fch), dy: fy1)
  p5: At(o, dx: -fx, dy: fy1 - fch)
  line bottom(p0.p, p1.p) -> line edge_r(p1.p, p2.p) -> line cham_r(p2.p, p3.p) ->
    line top(p3.p, p4.p) -> line cham_l(p4.p, p5.p) -> line edge_l(p5.p, p0.p) -> close
  claim p0.p distance(2 * fx) p1.p class shown
  claim p1.p distance(fy1 - fy0, along: y) p3.p class shown at (0, -27)

  // the exhausts vent sideways: a passage from each exhaust port out through the nearest edge
  point xr hint(x: o.x + fx, y: ep_y_r)
  xr on edge_r
  r.ep.p distance(0, along: y) xr
  line exh_r(r.ep.p, xr) class hidden
  point xl hint(x: o.x - fx + 5mm, y: ep_y_l)
  xl on cham_l
  l.ep.p distance(0, along: y) xl
  line exh_l(l.ep.p, xl) class hidden
  param ep_y_r = H * cos(alphaR) + a * cos(alphaR + beta)
  param ep_y_l = H * cos(alphaL) + a * cos(alphaL + beta)

  // -- the plenum: a channel inside the plate, an arc about the crank axis from one intake
  // port to the other, since both are the same radius from it ----------------------------------
  line rad_l(o, l.ip.p) class gone
  line rad_r(o, r.ip.p) class gone
  param kin = (rpl - wch / 2) / rpl
  param kout = (rpl + wch / 2) / rpl
  point ci0 hint(x: o.x + (l.ip.p.x - o.x) * kin, y: o.y + (l.ip.p.y - o.y) * kin)
  point ci1 hint(x: o.x + (r.ip.p.x - o.x) * kin, y: o.y + (r.ip.p.y - o.y) * kin)
  point co0 hint(x: o.x + (l.ip.p.x - o.x) * kout, y: o.y + (l.ip.p.y - o.y) * kout)
  point co1 hint(x: o.x + (r.ip.p.x - o.x) * kout, y: o.y + (r.ip.p.y - o.y) * kout)
  arc ch_in(center: o, start: ci1, end: ci0) hint(r: rpl - wch / 2) class hidden
  arc ch_out(center: o, start: co1, end: co0) hint(r: rpl + wch / 2) class hidden
  radius(rpl - wch / 2) ch_in
  radius(rpl + wch / 2) ch_out
  ci0 on rad_l
  ci1 on rad_r
  co0 on rad_l
  co1 on rad_r

  // -- the inlet: the boss on the plate's top edge, the coupling set into it, the passage down
  // through the throttle to the plenum, and the plug that will click onto it, in phantom -------
  boss: Box(o, x0: -bossw / 2, y0: fy1, x1: bossw / 2, y1: bossh)
  cpl_in: Box(o, x0: -cpl / 2, y0: bossh - cplin, x1: cpl / 2, y1: bossh) class hidden
  cpl_out: Box(o, x0: -cpl / 2, y0: bossh, x1: cpl / 2, y1: bossh - cplin + cpll)
  cpl_bore: Box(o, x0: -cplbore / 2, y0: bossh - cplin, x1: cplbore / 2, y1: bossh - cplin + cpll) class hidden
  passage: Box(o, x0: -wch / 2, y0: rpl + wch / 2, x1: wch / 2, y1: bossh - cplin) class hidden
  plug_body: Box(o, x0: -6mm, y0: bossh - cplin + cpll, x1: 6mm, y1: bossh - cplin + cpll + 14mm) class phantom
  plug_nose: Box(o, x0: -3.5mm, y0: bossh - cplin + cpll + 14mm, x1: 3.5mm, y1: bossh - cplin + cpll + 30mm) class phantom

  // the throttle: the barrel across the passage, its cross-hole `tau` out of line with it, and
  // the lever on the boss's face saying so
  tb: At(o, dx: 0mm, dy: Ty)
  circle barrel(center: tb.p) hint(r: rbar) class hidden
  radius(rbar) barrel
  point tip hint(x: o.x + lev * sin(tau), y: o.y + Ty + lev * cos(tau))
  line lever(tb.p, tip) class lever
  tb.p distance(lev) tip
  ref angle(-tau) lever
  circle knob(center: tip) hint(r: 2.5mm) class lever
  radius(2.5) knob
  circle hub(center: tb.p) hint(r: 4mm) class lever
  radius(4) hub
  claim lever angle(tau) ref class shown at (0.3, 14)
  // the cross-hole: two chords of the barrel, half a hole either side of the lever's line
  param hd = sqrt(rbar^2 - (dhole / 2)^2)
  point e0 hint(x: o.x - dhole / 2 * cos(tau) + hd * sin(tau), y: o.y + Ty + dhole / 2 * sin(tau) + hd * cos(tau))
  point e1 hint(x: o.x - dhole / 2 * cos(tau) - hd * sin(tau), y: o.y + Ty + dhole / 2 * sin(tau) - hd * cos(tau))
  point e2 hint(x: o.x + dhole / 2 * cos(tau) + hd * sin(tau), y: o.y + Ty - dhole / 2 * sin(tau) + hd * cos(tau))
  point e3 hint(x: o.x + dhole / 2 * cos(tau) - hd * sin(tau), y: o.y + Ty - dhole / 2 * sin(tau) - hd * cos(tau))
  e0 on barrel
  e1 on barrel
  e2 on barrel
  e3 on barrel
  e0 distance(dhole / 2) lever
  e1 distance(dhole / 2) lever
  e2 distance(-dhole / 2) lever
  e3 distance(-dhole / 2) lever
  line h0(e0, e1) class hidden
  line h1(e2, e3) class hidden
}
