// The end view: a transverse section through cylinder 1, looking along the crankshaft.
//
// Everything here is placed from the crank centre `o` by ordinates — this is the view that owns
// the engine's *heights* and *widths*, and the other two views take theirs from it by projection.
// The castings are four assemblies, each a component below: the bore, the block with its sump,
// the head with its valvetrain, and the timing drive.  The crankshaft, the rods and the piston
// are parts of their own (`engine.crankshaft`, `engine.conrod`), drawn in this view by the
// document.

use engine.dims
use engine.parts
use engine.valvetrain

// The cylinder wall, `wall` deep below the deck, and the two dimensions the sheet shows for it.
component Bore(o: point) {
  port l0: point hint(x: o.x - D / 2, y: o.y + deck)
  port r0: point hint(x: o.x + D / 2, y: o.y + deck)
  l1: At(o, dx: -D / 2, dy: deck - wall)
  r1: At(o, dx: D / 2, dy: deck - wall)
  line wall_l(l0, l1.p)
  line wall_r(r0, r1.p)
  o distance(-D / 2, along: x) l0
  o distance(deck, along: y) l0 class shown
  o distance(deck, along: y) r0
  l0 distance(D) r0 class shown
}

// The block's outline — deck, walls, skirt, pan rail — and the sump hung under the rail.
component Block(o: point) {
  port d_l: point hint(x: o.x - hw, y: o.y + deck)
  port d_r: point hint(x: o.x + hw, y: o.y + deck)
  s_l: At(o, dx: -hw, dy: 110mm)
  s_r: At(o, dx: hw, dy: 110mm)
  k_l: At(o, dx: -kw, dy: 30mm)
  k_r: At(o, dx: kw, dy: 30mm)
  port pr_l: point hint(x: o.x - kw, y: o.y + rail)
  point pr_r hint(x: o.x + kw, y: o.y + rail)
  line deckline(d_l, d_r) -> line b_r(d_r, s_r.p) -> line sk_r(s_r.p, k_r.p) ->
    line kr(k_r.p, pr_r) -> line railline(pr_r, pr_l) -> line kl(pr_l, k_l.p) ->
    line sk_l(k_l.p, s_l.p) -> line b_l(s_l.p, d_l) -> close
  o distance(-hw, along: x) d_l
  o distance(deck, along: y) d_l
  o distance(deck, along: y) d_r
  d_l distance(2 * hw) d_r class shown
  o distance(-kw, along: x) pr_l
  o distance(rail, along: y) pr_l
  o distance(rail, along: y) pr_r
  pr_l distance(2 * kw) pr_r class shown
  // the sump
  sh_l: At(o, dx: -kw, dy: rail - 30mm)
  sh_r: At(o, dx: kw, dy: rail - 30mm)
  sp_l: At(o, dx: -60mm, dy: sump)
  point sp_r hint(x: o.x + 60mm, y: o.y + sump)
  line su_r(pr_r, sh_r.p) -> line ss_r(sh_r.p, sp_r) -> line sb(sp_r, sp_l.p) ->
    line ss_l(sp_l.p, sh_l.p) -> line su_l(sh_l.p, pr_l)
  o distance(60, along: x) sp_r
  sp_r distance(-sump, along: y) o class shown
}

// The head over the bore: a pent roof from the bore's top corners, the head's outline, a plug,
// and the two valves square to the roof under their cams.  Cylinder 1 is on its intake stroke:
// the intake lobe's nose is down the valve axis and the valve is open on it; the exhaust lobe is
// turned away and its valve is shut on the base circle.
component Head(o: point, wl0: point, wr0: point, d_l: point, d_r: point) {
  ridge: At(o, dx: 0mm, dy: roof)
  line roof_l(wl0, ridge.p)
  line roof_r(wr0, ridge.p)
  port tl: point hint(x: o.x - 110mm, y: o.y + deck + head)
  tr: At(o, dx: 110mm, dy: deck + head)
  line headtop(tl, tr.p)
  line head_sl(d_l, tl)
  line head_sr(d_r, tr.p)
  o distance(-110, along: x) tl
  o distance(deck + head, along: y) tl class shown
  plug: Box(ridge.p, x0: -7mm, y0: 0mm, x1: 7mm, y1: 40mm)
  // the valve axes: through the seat centres, square to the roof, up to the cam centres
  point seat_i hint(x: o.x + vs, y: o.y + deck + (D / 2 - vs) * tan(va))
  point seat_e hint(x: o.x - vs, y: o.y + deck + (D / 2 - vs) * tan(va))
  seat_i on roof_r
  seat_e on roof_l
  o distance(vs, along: x) seat_i
  o distance(-vs, along: x) seat_e
  port cam_i: point hint(x: o.x + camx, y: o.y + camh)
  port cam_e: point hint(x: o.x - camx, y: o.y + camh)
  line vaxis_i(seat_i, cam_i) class axis
  line vaxis_e(seat_e, cam_e) class axis
  vaxis_i perpendicular roof_r
  vaxis_e perpendicular roof_l
  seat_i distance(stem + rb) cam_i
  seat_e distance(stem + rb) cam_e
  lobe_i: Lobe(cam_i, vaxis_i, phi: 180deg)
  lobe_e: Lobe(cam_e, vaxis_e, phi: 300deg)
  v_i: Valve(seat_i, vaxis_i, lobe_i.nose, head: div)
  v_e: Valve(seat_e, vaxis_e, lobe_e.base, head: dev)
}

// The timing drive on the front of the engine: crank pulley, two cam pulleys, the belt over them.
component Drive(o: point, cam_i: point, cam_e: point) {
  circle pcrank(center: o) hint(r: rcp) class belt
  circle pcam_i(center: cam_i) hint(r: rcam) class belt
  circle pcam_e(center: cam_e) hint(r: rcam) class belt
  radius(rcp) pcrank
  radius(rcam) pcam_i
  radius(rcam) pcam_e
  b1: Span(pcrank, pcam_i, side: -1) class belt
  b2: Span(pcam_i, pcam_e, side: -1) class belt
  b3: Span(pcam_e, pcrank, side: -1) class belt
}

component EndSection(o: point) {
  top: At(o, dx: 0mm, dy: deck + 30mm)
  port bore = axisline
  line axisline(o, top.p) class axis
  cyl: Bore(o)
  block: Block(o)
  head: Head(o, cyl.l0, cyl.r0, block.d_l, block.d_r)
  drive: Drive(o, head.cam_i, head.cam_e)
}
