// The engine block, designed in one place: the casting from the pan rail to the deck, the four
// bores, the crankcase bulkheads that carry the main bearings, and the sump hung under the rail.
//
// One component, three views (§6.7).  Along the axis it is the section through cylinder 1: the
// block's outline, the bore's walls, and — behind the section — the main bearing at the crank,
// its shell hidden and its cap below the parting line.  Across the axis it is the whole casting
// edge on: rail to deck, four bores, five bulkheads with their bearing shells and caps, the sump.
// From above it is the deck's outline with the four bores.  Every height in the side view and
// every width in the plan is the end view's, by projection, inside the part; every length is the
// side view's.

use engine.dims
use engine.parts

// A main bearing edge on, at the axis point `jc`: the shell above and below the journal, the
// bulkhead rising from the shell to the cylinder walls, the cap hung below it.
component MainBearingSide(jc: point) {
  upper: Box(jc, x0: -wmb / 2, y0: rj, x1: wmb / 2, y1: rmb)
  lower: Box(jc, x0: -wmb / 2, y0: -rmb, x1: wmb / 2, y1: -rj)
  web: Box(jc, x0: -bulk / 2, y0: rmb, x1: bulk / 2, y1: deck - wall)
  cap: Box(jc, x0: -wmb / 2, y0: -(rmb + capd), x1: wmb / 2, y1: -rmb)
}

component EngineBlock(end: plane, side: plane, top: plane, o: point, o_s: point, o_t: point) {
  // -- along the axis: the section through cylinder 1 -------------------------------------
  in end {
    // the bore's walls, `wall` deep below the deck
    point bl0 hint(x: o.x - D / 2, y: o.y + deck)
    point br0 hint(x: o.x + D / 2, y: o.y + deck)
    bl1: At(o, dx: -D / 2, dy: deck - wall)
    br1: At(o, dx: D / 2, dy: deck - wall)
    line wall_l(bl0, bl1.p)
    line wall_r(br0, br1.p)
    o distance(-D / 2, along: x) bl0
    o distance(deck, along: y) bl0 class shown
    o distance(deck, along: y) br0
    bl0 distance(D) br0 class shown
    // the outline: deck, walls, skirt, pan rail
    point d_l hint(x: o.x - hw, y: o.y + deck)
    point d_r hint(x: o.x + hw, y: o.y + deck)
    s_l: At(o, dx: -hw, dy: 110mm)
    s_r: At(o, dx: hw, dy: 110mm)
    k_l: At(o, dx: -kw, dy: 30mm)
    k_r: At(o, dx: kw, dy: 30mm)
    point pr_l hint(x: o.x - kw, y: o.y + rail)
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
    // the main bearing behind the section: the shell round the journal, hidden, and the cap
    // below the parting line at the crank's axis
    circle shell(center: o) hint(r: rmb) class hidden
    radius(rmb) shell
    point c_l hint(x: o.x - (rmb + capd), y: o.y)
    point c_r hint(x: o.x + (rmb + capd), y: o.y)
    arc cap(center: o, start: c_l, end: c_r) hint(r: rmb + capd)
    radius(rmb + capd) cap
    line parting(c_l, c_r)
    o distance(0, along: y) c_l
    o distance(0, along: y) c_r
    // the sump
    sh_l: At(o, dx: -kw, dy: rail - 30mm)
    sh_r: At(o, dx: kw, dy: rail - 30mm)
    point sp_l hint(x: o.x - 60mm, y: o.y + sump)
    point sp_r hint(x: o.x + 60mm, y: o.y + sump)
    line su_r(pr_r, sh_r.p) -> line ss_r(sh_r.p, sp_r) -> line sb(sp_r, sp_l) ->
      line ss_l(sp_l, sh_l.p) -> line su_l(sh_l.p, pr_l)
    o distance(-60, along: x) sp_l
    o distance(sump, along: y) sp_l
    o distance(60, along: x) sp_r
    sp_r distance(-sump, along: y) o class shown
  }

  // -- across the axis: the casting edge on -----------------------------------------------
  in side {
    point bfl hint(x: o_s.x + front, y: o_s.y + deck)
    point bfr hint(x: o_s.x + back, y: o_s.y + deck)
    point rfl hint(x: o_s.x + front, y: o_s.y + rail)
    point rfr hint(x: o_s.x + back, y: o_s.y + rail)
    line dl(bfl, bfr)
    line blockfront(bfl, rfl)
    line blockback(bfr, rfr)
    line rl(rfl, rfr)
    o_s distance(front, along: x) bfl
    o_s distance(front, along: x) rfl
    o_s distance(back, along: x) rfr
    horizontal dl
    horizontal rl
    bfl distance(back - front) bfr class shown
    // the sump: shallow at the front, deep at the back
    point q_a hint(x: o_s.x + front + 15mm, y: o_s.y + rail)
    point q_b hint(x: o_s.x + front + 45mm, y: o_s.y + sump + 45mm)
    point q_c hint(x: o_s.x + front + 150mm, y: o_s.y + sump)
    point q_d hint(x: o_s.x + back - 40mm, y: o_s.y + sump)
    point q_e hint(x: o_s.x + back - 10mm, y: o_s.y + rail)
    line s1(q_a, q_b) -> line s2(q_b, q_c) -> line s3(q_c, q_d) -> line s4(q_d, q_e)
    q_a on rl
    q_e on rl
    o_s distance(front + 15mm, along: x) q_a
    o_s distance(back - 10mm, along: x) q_e
    o_s distance(front + 45mm, along: x) q_b
    q_d distance(45, along: y) q_b
    o_s distance(front + 150mm, along: x) q_c
    o_s distance(back - 40mm, along: x) q_d
    horizontal s3
    // the four bores, their walls down from the deck
    repeat 4 as i {
      ax: At(o_s, dx: front + 25mm + P / 2 + i * P, dy: 0mm)
      point wl0 hint(x: ax.p.x - D / 2, y: ax.p.y + deck)
      point wr0 hint(x: ax.p.x + D / 2, y: ax.p.y + deck)
      point wl1 hint(x: ax.p.x - D / 2, y: ax.p.y + deck - wall)
      point wr1 hint(x: ax.p.x + D / 2, y: ax.p.y + deck - wall)
      line wall_l(wl0, wl1)
      line wall_r(wr0, wr1)
      wl0 on dl
      wr0 on dl
      ax.p distance(-D / 2, along: x) wl0
      ax.p distance(D / 2, along: x) wr0
      ax.p distance(-D / 2, along: x) wl1
      ax.p distance(D / 2, along: x) wr1
      wl0 distance(-wall, along: y) wl1
      wr0 distance(-wall, along: y) wr1
    }
    // the five bulkheads and their main bearings
    repeat 5 as j {
      jc: At(o_s, dx: front + 25mm + j * P, dy: 0mm)
      bearing: MainBearingSide(jc.p)
    }
    claim ax[0].p distance(P) ax[1].p class shown
  }

  // -- from above: the deck and the bores -----------------------------------------------
  in top {
    point fl hint(x: o_t.x + front, y: o_t.y - hw)
    point fr hint(x: o_t.x + back, y: o_t.y - hw)
    point br hint(x: o_t.x + back, y: o_t.y + hw)
    point bl hint(x: o_t.x + front, y: o_t.y + hw)
    line e1(fl, fr) -> line e2(fr, br) -> line e3(br, bl) -> line e4(bl, fl) -> close
    horizontal e1
    vertical e2
    horizontal e3
    vertical e4
    repeat 4 as i {
      point c hint(x: o_t.x + front + 25mm + P / 2 + i * P, y: o_t.y)
      o_t distance(0, along: y) c
      circle bore(center: c) hint(r: D / 2)
      radius(D / 2) bore
    }
  }

  // -- the views agree ------------------------------------------------------------------
  d_l project bfl              // the deck's height
  pr_l project rfl             // the rail's
  sp_l project q_d             // the sump's bottom
  bfl project fl               // the block's ends, along the axis
  bfr project br
  d_r project fl               // and its width, across
  d_l project br
  ax[0].p project c[0]         // the bores
  ax[1].p project c[1]
  ax[2].p project c[2]
  ax[3].p project c[3]
}
