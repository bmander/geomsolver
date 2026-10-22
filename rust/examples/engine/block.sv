// The engine block, designed in one place: the casting from the pan rail to the deck, the four
// bores in their water jacket, the crankcase bulkheads that carry the main bearings and their
// caps, and the oil pan bolted under the rail.
//
// One component, three views (§6.7).  Along the axis it is the section through cylinder 1: the
// block's outline, the bore's walls and the jacket either side of them, and — behind the section
// — the main bearing at the crank, its shell hidden and its cap below the parting line; under the
// rail the pan in section, its wall and its flange, and the pair of its bolts the section cuts.
// Across the axis it is the whole casting edge on: rail to deck, four bores, a core plug over
// each, five bulkheads with their bearing shells and caps, and the pan under them with its drain.
// From above it is the deck's outline: the four bores, the barrels round them standing in the
// jacket's opening, and the head bolts.  Every height in the side view and every width in the
// plan is the end view's, by projection, inside the part; every length is the side view's.

use engine.dims (D, P, back, bulk, capd, deck, front, hw, kw, rail, rbay, rj, rmb, sump, wall, wmb)
use engine.parts
use std (horizontal, offset, right_of)

// the oil pan: a casting `pt` thick, hung from a flange `pf` thick under the block's rail, its
// walls `pw` either side of the axis and its ends `pfe` in from the block's faces
pt := 4mm
pf := 8mm
pw := kw - 15mm
pfe := 10mm
// its drain plug, in a boss under the floor near the back, where the floor is deepest
xdrain := back - 60mm
rboss := 12mm
rdrain := 7mm
bh := 8mm
// the bolts that hang it, through the flange either side and on up into the rail, one over each
// bore: M6, midway across the flange's land, its hole `tap` deep in the rail
rbolt := 3.3mm
ybolt := (pw - pt + kw) / 2
tap := 16mm

// the block: the water jacket, a slot `jdepth` deep round the four cylinder barrels, open at the
// deck (an open-deck block), each barrel a wall `rbar - D / 2` thick round its bore, siamesed
// where they meet between the bores; the head bolts, two at each bulkhead, tapped down from the
// deck outside the jacket, `hb` either side of the axis; and the core plugs, in the holes the
// jacket's sand core was held by, in the block's sides over each bore
rbar := D / 2 + 7mm
jw := 55mm
jx := 55mm
jdepth := 110mm
hb := 64mm
rhb := 5mm
hdepth := 80mm
rplug := 16mm
zplug := deck - 60mm

// A main bearing edge on, at the axis point `jc`: the shell above and below the journal, the
// bulkhead rising from the shell to the cylinder walls, the cap hung below it.
part MainBearingSide(jc: point) {
  upper := engine.parts.Box(jc, x0: -wmb / 2, y0: rj, x1: wmb / 2, y1: rmb)
  lower := engine.parts.Box(jc, x0: -wmb / 2, y0: -rmb, x1: wmb / 2, y1: -rj)
  web := engine.parts.Box(jc, x0: -bulk / 2, y0: rmb, x1: bulk / 2, y1: deck - wall)
  cap := engine.parts.Box(jc, x0: -wmb / 2, y0: -(rmb + capd), x1: wmb / 2, y1: -rmb)
}

part EngineBlock() {
  // -- along the axis: the section through cylinder 1 -------------------------------------
  in std.side {
    // the bore's walls, `wall` deep below the deck
    bl0 := point hint((std.side.origin.x - D / 2, std.side.origin.y + deck))
    br0 := point hint((std.side.origin.x + D / 2, std.side.origin.y + deck))
    bl1 := engine.parts.At(std.side.origin, dx: -D / 2, dy: deck - wall)
    br1 := engine.parts.At(std.side.origin, dx: D / 2, dy: deck - wall)
    wall_l := line(bl0, bl1.p)
    wall_r := line(br0, br1.p)
    bl0 offset(dx: -D / 2, dy: deck) std.side.origin
    std.side.origin distance(deck, along: v) br0
    bl0 distance(D) br0
    // the water jacket either side of the bore, between the barrel and the slot's outside
    jl0 := engine.parts.At(std.side.origin, dx: -rbar, dy: deck)
    jl1 := engine.parts.At(std.side.origin, dx: -rbar, dy: deck - jdepth)
    jl2 := engine.parts.At(std.side.origin, dx: -jw, dy: deck - jdepth)
    jl3 := engine.parts.At(std.side.origin, dx: -jw, dy: deck)
    jr0 := engine.parts.At(std.side.origin, dx: rbar, dy: deck)
    jr1 := engine.parts.At(std.side.origin, dx: rbar, dy: deck - jdepth)
    jr2 := engine.parts.At(std.side.origin, dx: jw, dy: deck - jdepth)
    jr3 := engine.parts.At(std.side.origin, dx: jw, dy: deck)
    (barrel_l := line(jl0.p, jl1.p)) -> (jacket_l := line(jl1.p, jl2.p)) ->
      (jwall_l := line(jl2.p, jl3.p))
    (barrel_r := line(jr0.p, jr1.p)) -> (jacket_r := line(jr1.p, jr2.p)) ->
      (jwall_r := line(jr2.p, jr3.p))
    // the outline: deck, walls, skirt, pan rail
    d_l := point hint((std.side.origin.x - hw, std.side.origin.y + deck))
    d_r := point hint((std.side.origin.x + hw, std.side.origin.y + deck))
    s_l := engine.parts.At(std.side.origin, dx: -hw, dy: 110mm)
    s_r := engine.parts.At(std.side.origin, dx: hw, dy: 110mm)
    k_l := engine.parts.At(std.side.origin, dx: -kw, dy: 30mm)
    k_r := engine.parts.At(std.side.origin, dx: kw, dy: 30mm)
    pr_l := point hint((std.side.origin.x - kw, std.side.origin.y + rail))
    pr_r := point hint((std.side.origin.x + kw, std.side.origin.y + rail))
    (deckline := line(d_l, d_r)) -> (b_r := line(d_r, s_r.p)) -> (sk_r := line(s_r.p, k_r.p)) ->
      (kr := line(k_r.p, pr_r)) -> (railline := line(pr_r, pr_l)) -> (kl := line(pr_l, k_l.p)) ->
      (sk_l := line(k_l.p, s_l.p)) -> (b_l := line(s_l.p, d_l)) -> close
    d_l offset(dx: -hw, dy: deck) std.side.origin
    std.side.origin distance(deck, along: v) d_r
    d_l distance(2 * hw) d_r
    pr_l offset(dx: -kw, dy: rail) std.side.origin
    std.side.origin distance(rail, along: v) pr_r
    pr_l distance(2 * kw) pr_r
    // the main bearing behind the section: the shell round the journal, hidden, and the cap
    // below the parting line at the crank's axis
    shell := circle(center: std.side.origin) hint(r: rmb)
    radius(rmb) shell
    c_l := point hint((std.side.origin.x - (rmb + capd), std.side.origin.y))
    c_r := point hint((std.side.origin.x + (rmb + capd), std.side.origin.y))
    cap := arc(center: std.side.origin, start: c_l, end: c_r) hint(r: rmb + capd)
    radius(rmb + capd) cap
    parting := line(c_l, c_r)
    std.side.origin horizontal c_l
    std.side.origin horizontal c_r
    // the oil pan, in section: its flange under the rail, out to the block's sides, and its wall
    // hung from the flange's inner edge, straight then sloping in to the floor
    lo_l := engine.parts.At(std.side.origin, dx: -kw, dy: rail - pf)
    lo_r := engine.parts.At(std.side.origin, dx: kw, dy: rail - pf)
    li_l := engine.parts.At(std.side.origin, dx: -pw, dy: rail - pf)
    li_r := engine.parts.At(std.side.origin, dx: pw, dy: rail - pf)
    sh_l := engine.parts.At(std.side.origin, dx: -pw, dy: rail - 30mm)
    sh_r := engine.parts.At(std.side.origin, dx: pw, dy: rail - 30mm)
    sp_l := point hint((std.side.origin.x - 60mm, std.side.origin.y + sump))
    sp_r := point hint((std.side.origin.x + 60mm, std.side.origin.y + sump))
    (lip_r := line(pr_r, lo_r.p)) -> (flange_r := line(lo_r.p, li_r.p)) ->
      (su_r := line(li_r.p, sh_r.p)) -> (ss_r := line(sh_r.p, sp_r)) -> (sb := line(sp_r, sp_l)) ->
      (ss_l := line(sp_l, sh_l.p)) -> (su_l := line(sh_l.p, li_l.p)) ->
      (flange_l := line(li_l.p, lo_l.p)) -> (lip_l := line(lo_l.p, pr_l))
    sp_l offset(dx: -60, dy: sump) std.side.origin
    std.side.origin distance(60, along: u) sp_r
    sp_r distance(-sump, along: v) std.side.origin
    // the wall's inside, `pt` in from its outside all round, open at the rail
    it_l := point hint((std.side.origin.x - pw + pt, std.side.origin.y + rail))
    it_r := point hint((std.side.origin.x + pw - pt, std.side.origin.y + rail))
    ih_l := point hint((std.side.origin.x - pw + pt, std.side.origin.y + rail - 30mm))
    ih_r := point hint((std.side.origin.x + pw - pt, std.side.origin.y + rail - 30mm))
    ip_l := point hint((std.side.origin.x - 60mm + pt, std.side.origin.y + sump + pt))
    ip_r := point hint((std.side.origin.x + 60mm - pt, std.side.origin.y + sump + pt))
    (iu_r := line(it_r, ih_r)) -> (is_r := line(ih_r, ip_r)) -> (ib := line(ip_r, ip_l)) ->
      (is_l := line(ip_l, ih_l)) -> (iu_l := line(ih_l, it_l))
    su_r parallel iu_r
    ss_r parallel is_r
    sb parallel ib
    ss_l parallel is_l
    su_l parallel iu_l
    su_r distance(pt, side: right) iu_r
    ss_r distance(pt, side: right) is_r
    sb distance(pt, side: right) ib
    ss_l distance(pt, side: right) is_l
    su_l distance(pt, side: right) iu_l
    pr_r horizontal it_r
    pr_r horizontal it_l
    // the section passes through a pair of the pan's bolts: each hole through the flange and on
    // up into the rail
    bolt_l := engine.parts.Box(std.side.origin, x0: -ybolt - rbolt, y0: rail - pf,
                               x1: -ybolt + rbolt,
                               y1: rail + tap)
    bolt_r := engine.parts.Box(std.side.origin, x0: ybolt - rbolt, y0: rail - pf, x1: ybolt + rbolt,
                               y1: rail + tap)
  }

  // -- across the axis: the casting edge on -----------------------------------------------
  in std.front {
    bfl := point hint((std.origin.x + front, std.origin.y + deck))
    bfr := point hint((std.origin.x + back, std.origin.y + deck))
    rfl := point hint((std.origin.x + front, std.origin.y + rail))
    rfr := point hint((std.origin.x + back, std.origin.y + rail))
    dl := line(bfl, bfr)
    blockfront := line(bfl, rfl)
    blockback := line(bfr, rfr)
    rl := line(rfl, rfr)
    std.origin distance(front, along: u) bfl
    std.origin distance(front, along: u) rfl
    std.origin distance(back, along: u) rfr
    horizontal dl
    horizontal rl
    bfl distance(back - front) bfr
    // the oil pan: its flange under the rail the block's whole length, and below it the pan,
    // shallow at the front, deep at the back
    pf_f := engine.parts.At(rfl, dx: 0mm, dy: -pf)
    pf_b := engine.parts.At(rfr, dx: 0mm, dy: -pf)
    q_a := point hint((std.origin.x + front + pfe, std.origin.y + rail - pf))
    q_b := point hint((std.origin.x + front + 45mm, std.origin.y + sump + 45mm))
    q_c := point hint((std.origin.x + front + 150mm, std.origin.y + sump))
    q_d := point hint((std.origin.x + back - 40mm, std.origin.y + sump))
    q_e := point hint((std.origin.x + back - pfe, std.origin.y + rail - pf))
    (pe_f := line(rfl, pf_f.p)) -> (pu_f := line(pf_f.p, q_a)) -> (s1 := line(q_a, q_b)) ->
      (s2 := line(q_b, q_c)) -> (s3 := line(q_c, q_d)) -> (s4 := line(q_d, q_e)) ->
      (pu_b := line(q_e, pf_b.p)) -> (pe_b := line(pf_b.p, rfr))
    pf_f.p horizontal q_a
    pf_b.p horizontal q_e
    std.origin distance(front + pfe, along: u) q_a
    std.origin distance(back - pfe, along: u) q_e
    std.origin distance(front + 45mm, along: u) q_b
    q_d distance(45, along: v) q_b
    std.origin distance(front + 150mm, along: u) q_c
    std.origin distance(back - 40mm, along: u) q_d
    horizontal s3
    // and its wall's inside, `pt` in from the outside, up to the rail
    it_a := point hint((std.origin.x + front + pfe, std.origin.y + rail))
    j_b := point hint((std.origin.x + front + 45mm, std.origin.y + sump + 45mm + pt))
    j_c := point hint((std.origin.x + front + 150mm, std.origin.y + sump + pt))
    j_d := point hint((std.origin.x + back - 40mm - pt, std.origin.y + sump + pt))
    it_e := point hint((std.origin.x + back - pfe - pt, std.origin.y + rail))
    (t1 := line(it_a, j_b)) -> (t2 := line(j_b, j_c)) -> (t3 := line(j_c, j_d)) ->
      (t4 := line(j_d, it_e))
    s1 parallel t1
    s2 parallel t2
    s3 parallel t3
    s4 parallel t4
    s1 distance(pt, side: left) t1
    s2 distance(pt, side: left) t2
    s3 distance(pt, side: left) t3
    s4 distance(pt, side: left) t4
    rfl horizontal it_a
    rfl horizontal it_e
    // the drain boss under the floor, and the plug's hole up through it
    db0 := engine.parts.At(q_d, dx: xdrain - back + 40mm - rboss, dy: 0mm)
    db1 := engine.parts.At(q_d, dx: xdrain - back + 40mm - rboss, dy: -bh)
    db2 := engine.parts.At(q_d, dx: xdrain - back + 40mm + rboss, dy: -bh)
    db3 := engine.parts.At(q_d, dx: xdrain - back + 40mm + rboss, dy: 0mm)
    (boss_f := line(db0.p, db1.p)) -> (boss_u := line(db1.p, db2.p)) ->
      (boss_b := line(db2.p, db3.p))
    dh0 := engine.parts.At(q_d, dx: xdrain - back + 40mm - rdrain, dy: -bh)
    dh1 := engine.parts.At(q_d, dx: xdrain - back + 40mm - rdrain, dy: pt)
    dh2 := engine.parts.At(q_d, dx: xdrain - back + 40mm + rdrain, dy: -bh)
    dh3 := engine.parts.At(q_d, dx: xdrain - back + 40mm + rdrain, dy: pt)
    drain_f := line(dh0.p, dh1.p)
    drain_b := line(dh2.p, dh3.p)
    // the four bores, their walls down from the deck
    repeat 4 as i {
      ax := engine.parts.At(std.origin, dx: front + 25mm + P / 2 + i * P, dy: 0mm)
      wl0 := point hint((ax.p.x - D / 2, ax.p.y + deck))
      wr0 := point hint((ax.p.x + D / 2, ax.p.y + deck))
      wl1 := point hint((ax.p.x - D / 2, ax.p.y + deck - wall))
      wr1 := point hint((ax.p.x + D / 2, ax.p.y + deck - wall))
      wall_l := line(wl0, wl1)
      wall_r := line(wr0, wr1)
      wl0 coincident dl
      wr0 coincident dl
      ax.p distance(-D / 2, along: u) wl0
      ax.p distance(D / 2, along: u) wr0
      ax.p distance(-D / 2, along: u) wl1
      ax.p distance(D / 2, along: u) wr1
      wl0 distance(-wall, along: v) wl1
      wr0 distance(-wall, along: v) wr1
      // the core plug in the side over it
      pc := engine.parts.At(ax.p, dx: 0mm, dy: zplug)
      plug := circle(center: pc.p) hint(r: rplug)
      radius(rplug) plug
    }
    // the five bulkheads and their main bearings
    repeat 5 as j {
      jc := engine.parts.At(std.origin, dx: front + 25mm + j * P, dy: 0mm)
      bearing := MainBearingSide(jc.p)
    }
    claim ax[0].p distance(P) ax[1].p
  }

  // -- from above: the deck and the bores -----------------------------------------------
  in std.top {
    fl := point hint((std.top.origin.x + front, std.top.origin.y - hw))
    fr := point hint((std.top.origin.x + back, std.top.origin.y - hw))
    br := point hint((std.top.origin.x + back, std.top.origin.y + hw))
    bl := point hint((std.top.origin.x + front, std.top.origin.y + hw))
    (e1 := line(fl, fr)) -> (e2 := line(fr, br)) -> (e3 := line(br, bl)) -> (e4 := line(bl, fl)) -> close
    horizontal e1
    vertical e2
    horizontal e3
    vertical e4
    repeat 4 as i {
      c := point hint((std.top.origin.x + front + 25mm + P / 2 + i * P, std.top.origin.y))
      std.top.origin horizontal c
      bore := circle(center: c) hint(r: D / 2)
      radius(D / 2) bore
      barrel := circle(center: c) hint(r: rbar)
      radius(rbar) barrel
    }
    // the water jacket's opening in the deck, round the barrels
    jk0 := engine.parts.At(std.top.origin, dx: front + 25mm + P / 2 - jx, dy: -jw)
    jk1 := engine.parts.At(std.top.origin, dx: front + 25mm + P / 2 + 3 * P + jx, dy: -jw)
    jk2 := engine.parts.At(std.top.origin, dx: front + 25mm + P / 2 + 3 * P + jx, dy: jw)
    jk3 := engine.parts.At(std.top.origin, dx: front + 25mm + P / 2 - jx, dy: jw)
    jacket_rim := (jk_s := line(jk0.p, jk1.p)) -> (jk_b := line(jk1.p, jk2.p)) ->
      (jk_n := line(jk2.p, jk3.p)) -> (jk_f := line(jk3.p, jk0.p)) -> close
    // the head bolts, either side at each bulkhead
    repeat 5 as j {
      hb_n := engine.parts.At(std.top.origin, dx: front + 25mm + j * P, dy: hb)
      hb_s := engine.parts.At(std.top.origin, dx: front + 25mm + j * P, dy: -hb)
      head_bolt_n := circle(center: hb_n.p) hint(r: rhb)
      head_bolt_s := circle(center: hb_s.p) hint(r: rhb)
      radius(rhb) head_bolt_n
      radius(rhb) head_bolt_s
    }
    // under the block, where the plan does not see: the pan's bolts in the rail, and the drain
    repeat 4 as k {
      construction pb_n := point hint((std.top.origin.x + front + 25mm + P / 2 + k * P,
                                       std.top.origin.y + ybolt))
      construction pb_s := point hint((std.top.origin.x + front + 25mm + P / 2 + k * P,
                                       std.top.origin.y - ybolt))
      pb_n offset(dx: front + 25mm + P / 2 + k * P, dy: ybolt) std.top.origin
      pb_s offset(dx: front + 25mm + P / 2 + k * P, dy: -ybolt) std.top.origin
      construction pan_bolt_n := circle(center: pb_n) hint(r: rbolt)
      construction pan_bolt_s := circle(center: pb_s) hint(r: rbolt)
      radius(rbolt) pan_bolt_n
      radius(rbolt) pan_bolt_s
    }
    construction dc := point hint((std.top.origin.x + xdrain, std.top.origin.y))
    dc right_of(d: xdrain) std.top.origin
    construction drain_boss := circle(center: dc) hint(r: rboss)
    construction drain := circle(center: dc) hint(r: rdrain)
    radius(rboss) drain_boss
    radius(rdrain) drain
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

  // -- the casting as a solid -------------------------------------------------------------
  // The end section's outline run along the crank axis (the end view's normal) from the front
  // face to the rear, less the four bores drilled down from the deck as the plan draws them, and
  // less a bay for the crank between each pair of bulkheads, open below the rail to the sump;
  // the bulkheads are what is left, bored for the main bearings.  The sump is hung under the
  // rail: the end view's pan run along the axis, as deep as the side view's floor at each station.
  in std.side {
    // a bay's section: `rbay` either side of the bore axis, from below the rail up to where the
    // bores end, so the crank and the rods' big ends swing clear inside it
    construction bay0 := point hint((std.side.origin.x - rbay, std.side.origin.y + rail - 1mm))
    construction bay1 := point hint((std.side.origin.x + rbay, std.side.origin.y + rail - 1mm))
    construction bay2 := point hint((std.side.origin.x + rbay, std.side.origin.y + deck - wall))
    construction bay3 := point hint((std.side.origin.x - rbay, std.side.origin.y + deck - wall))
    bay0 offset(dx: -rbay, dy: rail - 1mm) std.side.origin
    bay1 offset(dx: rbay, dy: rail - 1mm) std.side.origin
    bay2 offset(dx: rbay, dy: deck - wall) std.side.origin
    bay3 offset(dx: -rbay, dy: deck - wall) std.side.origin
  }
  casting := solid(face(deckline, b_r, sk_r, kr, railline, kl, sk_l, b_l), from: front, to: back)
  repeat 4 as i {
    private cylinder := solid(face(bore[i]), from: deck - wall - 1mm, to: deck + 1mm)
    cylinder cut casting
  }
  repeat 4 as i {
    private crankcase := solid(face(bay0, bay1, bay2, bay3, -> close),
                               from: front + 25mm + bulk / 2 + i * P,
                               to: front + 25mm - bulk / 2 + (i + 1) * P)
    crankcase cut casting
  }
  mains := solid(face(shell), from: front - 1mm, to: back + 1mm)
  mains cut casting
  // The features the casting adds, gathered into one body and cut from it at once: the water
  // jacket, the slot round the barrels from the deck down, so the deck is open round them; the
  // head bolts' tapped holes down from the deck; the pan's tapped up into the rail; the core
  // holes through each side into the jacket (the side view looks along -y, so its ordinates run
  // that way); and the seat each bearing cap stands in under its bulkhead
  jacket := solid(jacket_rim, from: deck - jdepth, to: deck + 1mm)
  repeat 4 as i {
    private barrel_of := solid(face(barrel[i]), from: deck - jdepth - 1mm, to: deck + 2mm)
    barrel_of cut jacket
  }
  drillings := solid(jacket)
  repeat 5 as j {
    private bolt_n := solid(face(head_bolt_n[j]), from: deck - hdepth, to: deck + 1mm)
    private bolt_s := solid(face(head_bolt_s[j]), from: deck - hdepth, to: deck + 1mm)
    bolt_n union drillings
    bolt_s union drillings
  }
  repeat 4 as k {
    private tap_n := solid(face(pan_bolt_n[k]), from: rail - 1mm, to: rail + tap)
    private tap_s := solid(face(pan_bolt_s[k]), from: rail - 1mm, to: rail + tap)
    tap_n union drillings
    tap_s union drillings
  }
  repeat 4 as i {
    private core_n := solid(face(plug[i]), from: -hw - 1mm, to: -jw + 2mm)
    private core_s := solid(face(plug[i]), from: jw - 2mm, to: hw + 1mm)
    core_n union drillings
    core_s union drillings
  }
  repeat 5 as j {
    private seat := solid(face(cap, parting), from: front + 25mm + j * P - wmb / 2,
                          to: front + 25mm + j * P + wmb / 2)
    seat union drillings
  }
  drillings cut casting

  // The parts bolted to the casting, each one solid of its copies: the five main bearing caps,
  // the end view's cap below the parting line `wmb` long, bored with the bulkheads for the
  // shells; and the eight core plugs, pressed into their holes a millimetre below the sides.
  main_caps := solid()
  repeat 5 as j {
    private main_cap := solid(face(cap, parting), from: front + 25mm + j * P - wmb / 2,
                              to: front + 25mm + j * P + wmb / 2)
    private cap_bore := solid(face(shell), from: front + 24mm + j * P - wmb / 2,
                              to: front + 26mm + j * P + wmb / 2)
    cap_bore cut main_cap
    main_cap union main_caps
  }
  plugs := solid()
  repeat 4 as i {
    private plug_n := solid(face(plug[i]), from: -hw + 1mm, to: -hw + 5mm)
    private plug_s := solid(face(plug[i]), from: hw - 5mm, to: hw - 1mm)
    plug_n union plugs
    plug_s union plugs
  }

  // -- the oil pan as a solid -------------------------------------------------------------
  // Its outside is the end view's outline run along the axis, in common with the side view's
  // run across: the two meet in the flange, the block's own outline under the rail, and in the
  // walls, straight at the sides and sloping at the ends.  Its inside is the same of the two
  // insides, open at the rail, so the wall is `pt` thick everywhere.  The drain boss stands
  // into the floor from below, and the plug's hole and the flange's bolt holes go through.
  sump_body := solid(face(lip_r, flange_r, su_r, ss_r, sb, ss_l, su_l, flange_l, lip_l, -> close),
                     from: front - 1mm, to: back + 1mm)
  outline := solid(face(pe_f, pu_f, s1, s2, s3, s4, pu_b, pe_b, -> close),
                   from: -kw - 1mm, to: kw + 1mm)
  outline common sump_body
  cavity := solid(face(iu_r, is_r, ib, is_l, iu_l, -> close), from: front - 1mm, to: back + 1mm)
  inside := solid(face(t1, t2, t3, t4, -> close), from: -kw - 1mm, to: kw + 1mm)
  inside common cavity
  boss := solid(face(drain_boss), from: sump - bh, to: sump + pt / 2)
  pan := solid(sump_body)
  boss union pan
  cavity cut pan
  holes := solid(face(drain), from: sump - bh - 1mm, to: sump + pt + 1mm)
  repeat 4 as k {
    private hole_n := solid(face(pan_bolt_n[k]), from: rail - pf - 1mm, to: rail + 1mm)
    private hole_s := solid(face(pan_bolt_s[k]), from: rail - pf - 1mm, to: rail + 1mm)
    hole_n union holes
    hole_s union holes
  }
  holes cut pan
}

// Open this file to preview the block alone: the casting in the three views the assembly draws
// it in, about the world's origin, and the casting, its bearing caps and core plugs, and the oil
// pan as solids.
preview {
  unit mm
  block := EngineBlock()
}
