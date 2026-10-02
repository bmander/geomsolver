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
component MainBearingSide(jc: point, dims: group) {
  upper := engine.parts.Box(jc, x0: -dims.wmb / 2, y0: dims.rj, x1: dims.wmb / 2, y1: dims.rmb)
  lower := engine.parts.Box(jc, x0: -dims.wmb / 2, y0: -dims.rmb, x1: dims.wmb / 2, y1: -dims.rj)
  web := engine.parts.Box(jc, x0: -dims.bulk / 2, y0: dims.rmb, x1: dims.bulk / 2, y1: dims.deck - dims.wall)
  cap := engine.parts.Box(jc, x0: -dims.wmb / 2, y0: -(dims.rmb + dims.capd), x1: dims.wmb / 2, y1: -dims.rmb)
}

component EngineBlock(end: plane, side: plane, top: plane, o: point, o_s: point, o_t: point, dims: group) {
  // -- along the axis: the section through cylinder 1 -------------------------------------
  in end {
    // the bore's walls, `wall` deep below the deck
    bl0 := point hint(x: o.x - dims.D / 2, y: o.y + dims.deck)
    br0 := point hint(x: o.x + dims.D / 2, y: o.y + dims.deck)
    bl1 := engine.parts.At(o, dx: -dims.D / 2, dy: dims.deck - dims.wall)
    br1 := engine.parts.At(o, dx: dims.D / 2, dy: dims.deck - dims.wall)
    wall_l := line(bl0, bl1.p)
    wall_r := line(br0, br1.p)
    o distance(-dims.D / 2, along: x) bl0
    o distance(dims.deck, along: y) bl0
    o distance(dims.deck, along: y) br0
    bl0 distance(dims.D) br0
    // the outline: deck, walls, skirt, pan rail
    d_l := point hint(x: o.x - dims.hw, y: o.y + dims.deck)
    d_r := point hint(x: o.x + dims.hw, y: o.y + dims.deck)
    s_l := engine.parts.At(o, dx: -dims.hw, dy: 110mm)
    s_r := engine.parts.At(o, dx: dims.hw, dy: 110mm)
    k_l := engine.parts.At(o, dx: -dims.kw, dy: 30mm)
    k_r := engine.parts.At(o, dx: dims.kw, dy: 30mm)
    pr_l := point hint(x: o.x - dims.kw, y: o.y + dims.rail)
    pr_r := point hint(x: o.x + dims.kw, y: o.y + dims.rail)
    (deckline := line(d_l, d_r)) -> (b_r := line(d_r, s_r.p)) -> (sk_r := line(s_r.p, k_r.p)) ->
      (kr := line(k_r.p, pr_r)) -> (railline := line(pr_r, pr_l)) -> (kl := line(pr_l, k_l.p)) ->
      (sk_l := line(k_l.p, s_l.p)) -> (b_l := line(s_l.p, d_l)) -> close
    o distance(-dims.hw, along: x) d_l
    o distance(dims.deck, along: y) d_l
    o distance(dims.deck, along: y) d_r
    d_l distance(2 * dims.hw) d_r
    o distance(-dims.kw, along: x) pr_l
    o distance(dims.rail, along: y) pr_l
    o distance(dims.rail, along: y) pr_r
    pr_l distance(2 * dims.kw) pr_r
    // the main bearing behind the section: the shell round the journal, hidden, and the cap
    // below the parting line at the crank's axis
    shell := circle(center: o) hint(r: dims.rmb)
    radius(dims.rmb) shell
    c_l := point hint(x: o.x - (dims.rmb + dims.capd), y: o.y)
    c_r := point hint(x: o.x + (dims.rmb + dims.capd), y: o.y)
    cap := arc(center: o, start: c_l, end: c_r) hint(r: dims.rmb + dims.capd)
    radius(dims.rmb + dims.capd) cap
    parting := line(c_l, c_r)
    o distance(0, along: y) c_l
    o distance(0, along: y) c_r
    // the sump
    sh_l := engine.parts.At(o, dx: -dims.kw, dy: dims.rail - 30mm)
    sh_r := engine.parts.At(o, dx: dims.kw, dy: dims.rail - 30mm)
    sp_l := point hint(x: o.x - 60mm, y: o.y + dims.sump)
    sp_r := point hint(x: o.x + 60mm, y: o.y + dims.sump)
    (su_r := line(pr_r, sh_r.p)) -> (ss_r := line(sh_r.p, sp_r)) -> (sb := line(sp_r, sp_l)) ->
      (ss_l := line(sp_l, sh_l.p)) -> (su_l := line(sh_l.p, pr_l))
    o distance(60, along: left) sp_l
    o distance(dims.sump, along: y) sp_l
    o distance(60, along: x) sp_r
    sp_r distance(-dims.sump, along: y) o
  }

  // -- across the axis: the casting edge on -----------------------------------------------
  in side {
    bfl := point hint(x: o_s.x + dims.front, y: o_s.y + dims.deck)
    bfr := point hint(x: o_s.x + dims.back, y: o_s.y + dims.deck)
    rfl := point hint(x: o_s.x + dims.front, y: o_s.y + dims.rail)
    rfr := point hint(x: o_s.x + dims.back, y: o_s.y + dims.rail)
    dl := line(bfl, bfr)
    blockfront := line(bfl, rfl)
    blockback := line(bfr, rfr)
    rl := line(rfl, rfr)
    o_s distance(dims.front, along: x) bfl
    o_s distance(dims.front, along: x) rfl
    o_s distance(dims.back, along: x) rfr
    horizontal dl
    horizontal rl
    bfl distance(dims.back - dims.front) bfr
    // the sump: shallow at the front, deep at the back
    q_a := point hint(x: o_s.x + dims.front + 15mm, y: o_s.y + dims.rail)
    q_b := point hint(x: o_s.x + dims.front + 45mm, y: o_s.y + dims.sump + 45mm)
    q_c := point hint(x: o_s.x + dims.front + 150mm, y: o_s.y + dims.sump)
    q_d := point hint(x: o_s.x + dims.back - 40mm, y: o_s.y + dims.sump)
    q_e := point hint(x: o_s.x + dims.back - 10mm, y: o_s.y + dims.rail)
    (s1 := line(q_a, q_b)) -> (s2 := line(q_b, q_c)) -> (s3 := line(q_c, q_d)) -> (s4 := line(q_d, q_e))
    q_a on rl
    q_e on rl
    o_s distance(dims.front + 15mm, along: x) q_a
    o_s distance(dims.back - 10mm, along: x) q_e
    o_s distance(dims.front + 45mm, along: x) q_b
    q_d distance(45, along: y) q_b
    o_s distance(dims.front + 150mm, along: x) q_c
    o_s distance(dims.back - 40mm, along: x) q_d
    horizontal s3
    // the four bores, their walls down from the deck
    repeat 4 as i {
      ax := engine.parts.At(o_s, dx: dims.front + 25mm + dims.P / 2 + i * dims.P, dy: 0mm)
      wl0 := point hint(x: ax.p.x - dims.D / 2, y: ax.p.y + dims.deck)
      wr0 := point hint(x: ax.p.x + dims.D / 2, y: ax.p.y + dims.deck)
      wl1 := point hint(x: ax.p.x - dims.D / 2, y: ax.p.y + dims.deck - dims.wall)
      wr1 := point hint(x: ax.p.x + dims.D / 2, y: ax.p.y + dims.deck - dims.wall)
      wall_l := line(wl0, wl1)
      wall_r := line(wr0, wr1)
      wl0 on dl
      wr0 on dl
      ax.p distance(-dims.D / 2, along: x) wl0
      ax.p distance(dims.D / 2, along: x) wr0
      ax.p distance(-dims.D / 2, along: x) wl1
      ax.p distance(dims.D / 2, along: x) wr1
      wl0 distance(-dims.wall, along: y) wl1
      wr0 distance(-dims.wall, along: y) wr1
    }
    // the five bulkheads and their main bearings
    repeat 5 as j {
      jc := engine.parts.At(o_s, dx: dims.front + 25mm + j * dims.P, dy: 0mm)
      bearing := MainBearingSide(jc.p, dims: dims)
    }
    claim ax[0].p distance(dims.P) ax[1].p
  }

  // -- from above: the deck and the bores -----------------------------------------------
  in top {
    fl := point hint(x: o_t.x + dims.front, y: o_t.y - dims.hw)
    fr := point hint(x: o_t.x + dims.back, y: o_t.y - dims.hw)
    br := point hint(x: o_t.x + dims.back, y: o_t.y + dims.hw)
    bl := point hint(x: o_t.x + dims.front, y: o_t.y + dims.hw)
    (e1 := line(fl, fr)) -> (e2 := line(fr, br)) -> (e3 := line(br, bl)) -> (e4 := line(bl, fl)) -> close
    horizontal e1
    vertical e2
    horizontal e3
    vertical e4
    repeat 4 as i {
      c := point hint(x: o_t.x + dims.front + 25mm + dims.P / 2 + i * dims.P, y: o_t.y)
      o_t distance(0, along: y) c
      bore := circle(center: c) hint(r: dims.D / 2)
      radius(dims.D / 2) bore
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
