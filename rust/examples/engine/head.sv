// The cylinder head, designed in one place: a casting of its own standing on its gasket, with
// the pent-roof chambers, the valves square to the roof, and two overhead camshafts in their
// bearings.
//
// One component, three views (§6.7).  Along the axis it is the section through cylinder 1: the
// head's outline from the gasket face up, the roof over the bore, the plug, the two valves and the
// journals the camshafts ride in — how far each valve is lifted is the cycle's, worked out at
// `cycle`, cylinder 1's angle in it, from the timing in `dims.sv`.  Across the axis it is the head
// edge on: the casting and the camshaft (the two shafts one behind the other) in its five
// bearings.  From above it is the two shafts with their bearing caps, and the valves and plug of
// each cylinder.  Heights in the side view and widths in the plan are the end view's by
// projection, inside the part.  As a solid it is the casting: open above the camshafts' split
// line, pocketed for the lobes, a pent-roof chamber under each bore with the plug's well over it,
// and the head bolts' holes.
//
// What rides in it is a part of its own, mated to what the head gives it: each camshaft
// (`Camshaft`) in its journal, with a lobe over every valve pointed where that cylinder's angle in
// the cycle puts it, drawn along the axis and edge on, and its bearing caps; and the valves
// (`Valves`), a pair at each station along the axis, turned as solids about their own axes and
// lifted as far as the cycle says, each boring its bucket, guide and throat in the casting.

use engine.dims (D, P, back, camcap, camh, camx, deck, dev, div, dn_e, dn_i, ecenter, front,
                 gasket, head, hw, icenter, rb, rcamj, rn, roof, stem, va, vs, wcamb)
use engine.parts
use engine.valvetrain
use std (horizontal, offset, right_of)

// the camshafts' own sizes: the shaft between its journals, and each lobe's width along the axis,
// one lobe over each of a cylinder's four valves, which stand `vx` either side of its bore's axis
rcams := 11mm
wlobe := 12mm
vx := 16mm

// the casting's own: the cam box's reach past each shaft and the walls it leaves at the front and
// rear, and how deep each shaft's pocket is sunk below the split line — the lobes reach `dn + rn`
// from the shaft, so this clears them
wbox := 28mm
wwall := 10mm
dpocket := 30mm
// the head bolts' holes, on the block's: `ybolt` either side of the crank axis at each bulkhead
ybolt := 64mm
rbolt := 5mm

// A cam bearing cap, edge on or from above: a block `wcamb` long round the journal.
part CamBearing(c: point) {
  cap := engine.parts.Box(c, x0: -wcamb / 2, y0: -(rcamj + camcap), x1: wcamb / 2, y1: rcamj + camcap)
}

part CylinderHead(cycle: Angle) {
  // -- along the axis: the section through cylinder 1 -------------------------------------
  in std.side {
    // the casting: its face on the gasket, its sides, its top
    f_l := point hint((std.side.origin.x - hw, std.side.origin.y + deck + gasket))
    f_r := point hint((std.side.origin.x + hw, std.side.origin.y + deck + gasket))
    t_l := point hint((std.side.origin.x - 110mm, std.side.origin.y + deck + head))
    tr := engine.parts.At(std.side.origin, dx: 110mm, dy: deck + head)
    (gasket := line(f_l, f_r)) -> (side_r := line(f_r, tr.p)) -> (topline := line(tr.p, t_l)) -> (side_l := line(t_l, f_l)) -> close
    f_l offset(dx: -hw, dy: deck + gasket) std.side.origin
    std.side.origin distance(deck + gasket, along: v) f_r
    f_l distance(2 * hw) f_r
    t_l offset(dx: -110, dy: deck + head) std.side.origin
    // the pent roof over the bore, from the face at the bore's edges up to the ridge
    r_l := engine.parts.At(std.side.origin, dx: -D / 2, dy: deck + gasket)
    r_r := engine.parts.At(std.side.origin, dx: D / 2, dy: deck + gasket)
    ridge := engine.parts.At(std.side.origin, dx: 0mm, dy: roof + gasket)
    roof_l := line(r_l.p, ridge.p)
    roof_r := line(r_r.p, ridge.p)
    plug := engine.parts.Box(ridge.p, x0: -7mm, y0: 0mm, x1: 7mm, y1: 40mm)
    // the valve axes: through the seat centres, square to the roof, up to the cam centres
    seat_i := point hint((std.side.origin.x + vs,
                          std.side.origin.y + deck + gasket + (D / 2 - vs) * tan(va)))
    seat_e := point hint((std.side.origin.x - vs,
                          std.side.origin.y + deck + gasket + (D / 2 - vs) * tan(va)))
    seat_i coincident roof_r
    seat_e coincident roof_l
    std.side.origin distance(vs, along: u) seat_i
    std.side.origin distance(-vs, along: u) seat_e
    cam_i := point hint((std.side.origin.x + camx, std.side.origin.y + camh + gasket))
    cam_e := point hint((std.side.origin.x - camx, std.side.origin.y + camh + gasket))
    vaxis_i := line(seat_i, cam_i)
    vaxis_e := line(seat_e, cam_e)
    vaxis_i perpendicular roof_r
    vaxis_e perpendicular roof_l
    seat_i distance(stem + rb) cam_i
    seat_e distance(stem + rb) cam_e
    // the camshaft journals, hidden behind the lobes, which the camshafts (`Camshaft`) ride in
    j_i := circle(center: cam_i) hint(r: rcamj)
    j_e := circle(center: cam_e) hint(r: rcamj)
    radius(rcamj) j_i
    radius(rcamj) j_e
    // what the casting is cut by, drawn only for its solid: the cam box open above the split line
    // across both shafts, the pocket each shaft's lobes and buckets turn in below it, and the
    // chamber's section — the roof run on down past the gasket face, so the chamber cuts the face
    // cleanly rather than lying in it
    construction well := engine.parts.Box(std.side.origin, x0: -(camx + wbox), y0: camh + gasket,
                                          x1: camx + wbox, y1: deck + head + 2mm)
    construction pocket_i := engine.parts.Box(std.side.origin, x0: camx - dpocket,
                                              y0: camh + gasket - dpocket,
                                              x1: camx + wbox, y1: camh + gasket)
    construction pocket_e := engine.parts.Box(std.side.origin, x0: -(camx + wbox),
                                              y0: camh + gasket - dpocket,
                                              x1: -(camx - dpocket), y1: camh + gasket)
    construction ch_l := engine.parts.At(std.side.origin, dx: -(D / 2 + 3mm / tan(va)),
                                         dy: deck + gasket - 3mm)
    construction ch_r := engine.parts.At(std.side.origin, dx: D / 2 + 3mm / tan(va),
                                         dy: deck + gasket - 3mm)
    // where cylinder 1 is in its cycle says how far each valve is off its seat: the reach of
    // its lobe (the camshaft's) along the axis, less the base circle (see `dims.sv`)
    ai := (cycle - icenter) / 2
    ae := (cycle - ecenter) / 2
    lift_now_i := max(rb, dn_i * cos(ai) + rn) - rb
    lift_now_e := max(rb, dn_e * cos(ae) + rn) - rb
    v_i := engine.valvetrain.Valve(seat_i, vaxis_i, lift: lift_now_i, head: div)
    v_e := engine.valvetrain.Valve(seat_e, vaxis_e, lift: lift_now_e, head: dev)
  }

  // -- across the axis: the head edge on ----------------------------------------------
  in std.front {
    hfl := point hint((std.origin.x + front, std.origin.y + deck + gasket))
    hfr := point hint((std.origin.x + back, std.origin.y + deck + gasket))
    htl := point hint((std.origin.x + front, std.origin.y + deck + head))
    htr := point hint((std.origin.x + back, std.origin.y + deck + head))
    (hface := line(hfl, hfr)) -> (hback := line(hfr, htr)) -> (htop := line(htr, htl)) -> (hfront := line(htl, hfl)) -> close
    std.origin distance(front, along: u) hfl
    std.origin distance(front, along: u) htl
    std.origin distance(back, along: u) hfr
    std.origin distance(back, along: u) htr
    horizontal hface
    horizontal htop
    // the camshaft: the two shafts lie one behind the other here, one journal's outline
    cam := point hint((std.origin.x + front, std.origin.y + camh + gasket))
    camb := point hint((std.origin.x + back, std.origin.y + camh + gasket))
    camline := line(cam, camb)
    std.origin distance(front, along: u) cam
    std.origin distance(back, along: u) camb
    horizontal camline
    ju0 := point hint((std.origin.x + front + 10mm, std.origin.y + camh + gasket + rcamj))
    ju1 := point hint((std.origin.x + back - 10mm, std.origin.y + camh + gasket + rcamj))
    jd0 := point hint((std.origin.x + front + 10mm, std.origin.y + camh + gasket - rcamj))
    jd1 := point hint((std.origin.x + back - 10mm, std.origin.y + camh + gasket - rcamj))
    shaft_u := line(ju0, ju1)
    shaft_d := line(jd0, jd1)
    std.origin distance(front + 10mm, along: u) ju0
    std.origin distance(back - 10mm, along: u) ju1
    std.origin distance(front + 10mm, along: u) jd0
    std.origin distance(back - 10mm, along: u) jd1
    cam distance(rcamj, along: v) ju0
    cam distance(rcamj, along: v) ju1
    cam distance(-rcamj, along: v) jd0
    cam distance(-rcamj, along: v) jd1
    // five bearings, between and beyond the cylinders
    repeat 5 as j {
      bc := point hint((std.origin.x + front + 25mm + j * P, std.origin.y + camh + gasket))
      std.origin distance(front + 25mm + j * P, along: u) bc
      cam horizontal bc
      bearing := CamBearing(bc)
    }
  }

  // -- from above: the head's outline, the two shafts in their bearings, and each cylinder's
  // valves and plug -----------------------------------------------------------------------
  in std.top {
    hfl_t := point hint((std.top.origin.x + front, std.top.origin.y - 110mm))
    hfr_t := point hint((std.top.origin.x + back, std.top.origin.y - 110mm))
    hbr_t := point hint((std.top.origin.x + back, std.top.origin.y + 110mm))
    hbl_t := point hint((std.top.origin.x + front, std.top.origin.y + 110mm))
    (h1 := line(hfl_t, hfr_t)) -> (h2 := line(hfr_t, hbr_t)) -> (h3 := line(hbr_t, hbl_t)) -> (h4 := line(hbl_t, hfl_t)) -> close
    horizontal h1
    vertical h2
    horizontal h3
    vertical h4
    std.top.origin distance(front, along: u) hfl_t
    std.top.origin distance(back, along: u) hbr_t
    ci := point hint((std.top.origin.x + front + 10mm, std.top.origin.y + camx))
    ce := point hint((std.top.origin.x + front + 10mm, std.top.origin.y - camx))
    ci1 := point hint((std.top.origin.x + back - 10mm, std.top.origin.y + camx))
    ce1 := point hint((std.top.origin.x + back - 10mm, std.top.origin.y - camx))
    cl_i := line(ci, ci1)
    cl_e := line(ce, ce1)
    std.top.origin distance(front + 10mm, along: u) ci
    std.top.origin distance(back - 10mm, along: u) ci1
    std.top.origin distance(front + 10mm, along: u) ce
    std.top.origin distance(back - 10mm, along: u) ce1
    horizontal cl_i
    horizontal cl_e
    // the shafts' outlines, `rcamj` either side of each centreline
    repeat 2 as s {
      sgn := 1 - 2 * s
      a0 := point hint((std.top.origin.x + front + 10mm, std.top.origin.y + sgn * (camx + rcamj)))
      a1 := point hint((std.top.origin.x + back - 10mm, std.top.origin.y + sgn * (camx + rcamj)))
      b0 := point hint((std.top.origin.x + front + 10mm, std.top.origin.y + sgn * (camx - rcamj)))
      b1 := point hint((std.top.origin.x + back - 10mm, std.top.origin.y + sgn * (camx - rcamj)))
      outer := line(a0, a1)
      inner := line(b0, b1)
      std.top.origin distance(front + 10mm, along: u) a0
      std.top.origin distance(back - 10mm, along: u) a1
      std.top.origin distance(front + 10mm, along: u) b0
      std.top.origin distance(back - 10mm, along: u) b1
      ci distance(sgn * (camx + rcamj) - camx, along: v) a0
      ci distance(sgn * (camx + rcamj) - camx, along: v) a1
      ci distance(sgn * (camx - rcamj) - camx, along: v) b0
      ci distance(sgn * (camx - rcamj) - camx, along: v) b1
    }
    repeat 5 as j {
      bi := point hint((std.top.origin.x + front + 25mm + j * P, std.top.origin.y + camx))
      be := point hint((std.top.origin.x + front + 25mm + j * P, std.top.origin.y - camx))
      std.top.origin distance(front + 25mm + j * P, along: u) bi
      std.top.origin distance(front + 25mm + j * P, along: u) be
      ci horizontal bi
      ce horizontal be
      cap_i := CamBearing(bi)
      cap_e := CamBearing(be)
      // the head bolts either side at each bulkhead, where the block's holes are: each bored up
      // from the gasket face into the bearing's saddle, short of the camshaft's tunnel
      construction bolt_ip := point hint((std.top.origin.x + front + 25mm + j * P,
                                          std.top.origin.y + ybolt))
      construction bolt_ep := point hint((std.top.origin.x + front + 25mm + j * P,
                                          std.top.origin.y - ybolt))
      bolt_ip offset(dx: front + 25mm + j * P, dy: ybolt) std.top.origin
      bolt_ep offset(dx: front + 25mm + j * P, dy: -ybolt) std.top.origin
      construction bolt_i := circle(center: bolt_ip) hint(r: rbolt)
      construction bolt_e := circle(center: bolt_ep) hint(r: rbolt)
      radius(rbolt) bolt_i
      radius(rbolt) bolt_e
      private bolt_ib := solid(face(bolt_i), from: deck + gasket - 1mm, to: camh + gasket - dpocket)
      private bolt_eb := solid(face(bolt_e), from: deck + gasket - 1mm, to: camh + gasket - dpocket)
      bolt_ib cut casting
      bolt_eb cut casting
    }
    repeat 4 as i {
      pc := point hint((std.top.origin.x + front + 25mm + P / 2 + i * P, std.top.origin.y))
      pc right_of(d: front + 25mm + P / 2 + i * P) std.top.origin
      plug := circle(center: pc) hint(r: 7mm)
      radius(7) plug
      construction barrel := circle(center: pc) hint(r: D / 2)
      radius(D / 2) barrel
      // the combustion chamber over this bore: the roof's section run across the bore, kept
      // within the bore's own outline, so the roof's planes meet the bore's wall; and the plug's
      // well, from inside the chamber up to the cam box
      private chamber := solid(face(ch_l.p, ridge.p, ch_r.p, -> close),
                               from: front + 25mm + i * P, to: front + 25mm + P + i * P)
      private bore_b := solid(face(barrel), from: deck + gasket - 3mm, to: roof + gasket + 1mm)
      bore_b common chamber
      chamber cut casting
      private well_b := solid(face(plug), from: roof + gasket - 4mm, to: camh + gasket + 1mm)
      well_b cut casting
      repeat 2 as k {
        vi := engine.parts.At(pc, dx: (2 * k - 1) * vx, dy: vs)
        ve := engine.parts.At(pc, dx: (2 * k - 1) * vx, dy: -vs)
        intake := circle(center: vi.p) hint(r: div / 2)
        exhaust := circle(center: ve.p) hint(r: dev / 2)
        radius(div / 2) intake
        radius(dev / 2) exhaust
      }
    }
  }

  // -- the views agree ------------------------------------------------------------------
  f_l project hfl              // the gasket face's height
  t_l project htl              // the head's top
  cam_i project cam            // the camshafts' height
  cam_i project ci             // and where each stands across the engine
  cam_e project ce
  t_l project hbl_t            // the head's width, from its top corner
  tr.p project hfl_t

  // -- the casting as a solid -------------------------------------------------------------
  // The end section's outline run along the crank axis from the front face to the rear, bored
  // for the two camshafts; each shaft runs in its tunnel from its pulley, forward of the block,
  // to its last bearing.  Above the split line the cam box is open from the front wall to the
  // rear (the cover is not drawn), the caps standing in it on the bearings' saddles; between the
  // saddles each shaft's pocket is sunk for its lobes; and under each bore is its chamber, with
  // the plug's well (above, with the plan).  The valves bore their own guides and buckets in it
  // (`Valves`).
  casting := solid(face(gasket, side_r, topline, side_l), from: front, to: back)
  tunnel_i := solid(face(j_i), from: front - 1mm, to: back + 1mm)
  tunnel_e := solid(face(j_e), from: front - 1mm, to: back + 1mm)
  tunnel_i cut casting
  tunnel_e cut casting
  box := solid(well.profile, from: front + wwall, to: back - wwall)
  box cut casting
  repeat 4 as i {
    private pocket_ib := solid(pocket_i.profile, from: front + 25mm + wcamb / 2 + i * P,
                               to: front + 25mm + P - wcamb / 2 + i * P)
    private pocket_eb := solid(pocket_e.profile, from: front + 25mm + wcamb / 2 + i * P,
                               to: front + 25mm + P - wcamb / 2 + i * P)
    pocket_ib cut casting
    pocket_eb cut casting
  }
}

// A camshaft, riding in the head's journal `journal` (in the end section) and drawn edge on from
// `c_s`, its centre at the block's front face in the side section: the intake shaft for `intake`
// 1, timed against the intake valve axis `ref`, and the exhaust shaft for 0.  Along the axis it
// draws cylinder 1's lobe, and the other three cylinders' lobes about the same centre as
// construction, each pointed where its own cylinder's angle in the cycle puts it.  Edge on it
// draws every lobe at its station, one over each valve: it reaches above and below the shaft by
// the nose's height on the page with the nose circle round it, or by the base circle if that is
// taller, the intake axis leaning out one way and the exhaust the other.  As solids it is the
// shaft — the journal from the pulley's stub through the first bearing, a journal in each other
// bearing, the shaft between them and every lobe at its station — and its five bearing caps, the
// end section's cap over the journal from the split line up, bored for the journal.  So `cycle`,
// cylinder 1's angle in it — the assembly's running `t` — turns every lobe of both views and the
// solid alike, at half its speed.
part Camshaft(journal: circle, ref: line, c_s: point, intake: Int, cycle: Angle) {
  center := intake * icenter + (1 - intake) * ecenter
  nose := intake * dn_i + (1 - intake) * dn_e    // how far its nose stands out
  lean := (1 - 2 * intake) * va      // the valve axis's lean off the bore's, counter-clockwise

  // -- along the axis --------------------------------------------------------------------
  in journal.center.plane {
    lobe := engine.valvetrain.Lobe(journal.center, ref,
                                   phi: 180deg + (cycle - center) / 2, dn: nose)
    repeat 3 as j {
      off := 180deg * ((j + 1) * (2 - j) / 2) + 360deg * (j + 1 - 2 * floor((j + 1) / 2))
      cj := cycle - off - 720deg * floor((cycle - off) / 720deg)
      construction ghost := engine.valvetrain.Lobe(journal.center, ref,
                                                   phi: 180deg + (cj - center) / 2, dn: nose)
    }
    construction core := circle(center: journal.center) hint(r: rcams)
    radius(rcams) core
    construction cap := engine.parts.Box(journal.center, x0: -(rcamj + camcap), y0: 0mm,
                                         x1: rcamj + camcap, y1: rcamj + camcap)
  }

  // -- across the axis: every lobe edge on ---------------------------------------------
  in c_s.plane {
    repeat 4 as i {
      off := 180deg * (i * (3 - i) / 2) + 360deg * (i - 2 * floor(i / 2))
      ci := cycle - off - 720deg * floor((cycle - off) / 720deg)
      ny := nose * sin(90deg + lean + 180deg + (ci - center) / 2)
      top := max(rb, ny + rn)
      bot := max(rb, rn - ny)
      repeat 2 as k {
        xk := 25mm + P / 2 + i * P + (2 * k - 1) * vx      // from the front face
        edge := engine.parts.Box(c_s, x0: xk - wlobe / 2, y0: -bot, x1: xk + wlobe / 2, y1: top)
      }
    }
  }

  // -- the shaft and its caps as solids ---------------------------------------------------
  shaft := solid()
  stub := solid(face(journal), from: front - 55mm, to: front + 25mm + wcamb / 2)
  between := solid(face(core), from: front + 25mm, to: back - 10mm)
  stub union shaft
  between union shaft
  repeat 4 as j {
    private journal_b := solid(face(journal), from: front + 25mm + (j + 1) * P - wcamb / 2,
                               to: front + 25mm + (j + 1) * P + wcamb / 2)
    journal_b union shaft
  }
  repeat 4 as i {
    repeat 2 as k {
      xv := front + 25mm + P / 2 + i * P + (2 * k - 1) * vx
      when i == 0 {
        private lobe_b := solid(lobe.profile, from: xv - wlobe / 2, to: xv + wlobe / 2)
      }
      when i > 0 {
        private lobe_b := solid(ghost[i - 1].profile, from: xv - wlobe / 2, to: xv + wlobe / 2)
      }
      lobe_b union shaft
    }
  }
  caps := solid()
  repeat 5 as j {
    private cap_b := solid(cap.profile, from: front + 25mm + j * P - wcamb / 2,
                           to: front + 25mm + j * P + wcamb / 2)
    cap_b union caps
  }
  bore := solid(face(journal), from: front, to: back)
  bore cut caps
}

// Two of cylinder `cylinder`'s valves (1 to 4, front to back) in the head `h` (a `CylinderHead`),
// as solids, each with its bucket: the intake valve and the exhaust valve at one station along the
// crank axis, `vx` before the bore's axis (`aft` 0) or behind it (`aft` 1), each as far off its
// seat as `cycle` (cylinder 1's angle in it) puts that cylinder's lobe, and each boring its
// bucket, guide and throat in the head's casting.  The station is a plane parallel to the end
// section, made of a line across the engine in the head's plan and one up it in its side section
// from where the station crosses the crank axis: so it is the end section carried along the axis,
// its coordinates the end section's own, and the valves stand in it as the end section draws
// cylinder 1's, by the same numbers.
part Valves(h: group, cylinder: Int, aft: Int, cycle: Angle) {
  i := cylinder - 1
  x := front + 25mm + P / 2 + i * P + (2 * aft - 1) * vx
  off := 180deg * (i * (3 - i) / 2) + 360deg * (i - 2 * floor(i / 2))
  ci := cycle - off - 720deg * floor((cycle - off) / 720deg)
  lift_i := max(rb, dn_i * cos((ci - icenter) / 2) + rn) - rb
  lift_e := max(rb, dn_e * cos((ci - ecenter) / 2) + rn) - rb
  construction sa := point in std.front hint((std.origin.x + x, std.origin.y))
  construction sb := point in std.top hint((std.top.origin.x + x, std.top.origin.y + 10mm))
  construction sc := point in std.front hint((std.origin.x + x, std.origin.y + 10mm))
  sa right_of(d: x) std.origin
  sb offset(dx: x, dy: 10mm) std.top.origin
  sc offset(dx: x, dy: 10mm) std.origin
  construction across := line(sa, sb)
  construction up := line(sa, sc)
  station := plane(u: across, v: up)           // its axes run as their lines do
  in station {
    construction seat_i := point hint((vs, deck + gasket + (D / 2 - vs) * tan(va)))
    construction seat_e := point hint((-vs, deck + gasket + (D / 2 - vs) * tan(va)))
    construction cam_i := point hint((camx, camh + gasket))
    construction cam_e := point hint((-camx, camh + gasket))
    seat_i offset(dx: vs, dy: deck + gasket + (D / 2 - vs) * tan(va)) station.origin
    seat_e offset(dx: -vs, dy: deck + gasket + (D / 2 - vs) * tan(va)) station.origin
    cam_i offset(dx: camx, dy: camh + gasket) station.origin
    cam_e offset(dx: -camx, dy: camh + gasket) station.origin
    construction ax_i := line(seat_i, cam_i)
    construction ax_e := line(seat_e, cam_e)
    construction drawn_i := engine.valvetrain.Valve(seat_i, ax_i, lift: lift_i, head: div)
    construction drawn_e := engine.valvetrain.Valve(seat_e, ax_e, lift: lift_e, head: dev)
    intake := engine.valvetrain.Poppet(drawn_i)
    exhaust := engine.valvetrain.Poppet(drawn_e)
    construction bore_i := engine.valvetrain.Bore(ax_i, head: div)
    construction bore_e := engine.valvetrain.Bore(ax_e, head: dev)
  }
  bore_i.cutter cut h.casting
  bore_e.cutter cut h.casting
}

// Open this file to preview the head alone: the casting in the three views the assembly draws it
// in, about the world's origin, with cylinder 1's valves where the table's cycle puts them, the
// two camshafts in its journals with every lobe where the cycle points it, and every cylinder's
// valves at their stations — the casting, the camshafts, their caps and the valves as solids.
preview {
  unit mm
  cylinder_head := CylinderHead(cycle: engine.dims.cycle)
  camshaft_i := Camshaft(cylinder_head.j_i, cylinder_head.vaxis_i, cylinder_head.cam, intake: 1,
                         cycle: engine.dims.cycle)
  camshaft_e := Camshaft(cylinder_head.j_e, cylinder_head.vaxis_e, cylinder_head.cam, intake: 0,
                         cycle: engine.dims.cycle)
  valves1f := Valves(cylinder_head, cylinder: 1, aft: 0, cycle: engine.dims.cycle)
  valves1r := Valves(cylinder_head, cylinder: 1, aft: 1, cycle: engine.dims.cycle)
  valves2f := Valves(cylinder_head, cylinder: 2, aft: 0, cycle: engine.dims.cycle)
  valves2r := Valves(cylinder_head, cylinder: 2, aft: 1, cycle: engine.dims.cycle)
  valves3f := Valves(cylinder_head, cylinder: 3, aft: 0, cycle: engine.dims.cycle)
  valves3r := Valves(cylinder_head, cylinder: 3, aft: 1, cycle: engine.dims.cycle)
  valves4f := Valves(cylinder_head, cylinder: 4, aft: 0, cycle: engine.dims.cycle)
  valves4r := Valves(cylinder_head, cylinder: 4, aft: 1, cycle: engine.dims.cycle)
}
