// The connecting rod, designed in one place: a part, the rod on its crank pin with its small end
// in the bore.
//
// It is given what it is mated to and nothing else: the crank pin, in the two views it is drawn
// in, and the bore axis.  It draws its geometry in each view the pin is drawn in (`in
// pin.end.plane`, `in pin.side.plane`), ties the two views by projection inside itself, and makes
// the plane section A-A is cut in, square to the rod at mid-shank: so the rod is a function of
// its pin and its bore, and the assembly places it by saying which pin and which bore.  Which of
// its pictures a sheet shows, and how, is the drawing's (`engine.svd`).
//
// Looking along the crankshaft (the end view) the rod is seen in its plane of swing: the big-end
// eye about the crank pin with a bearing shell, split at a parting line square to the rod with two
// cap bolts through it; the small-end eye about the piston pin with a bush; a shank tapering from
// big end to small end and filleted into both eyes; and the drilled oil passage up the middle.
// Looking across (the side view) it is seen edge on — the big end, the shank's flanges and the
// small end each a width along the crankshaft.  Section A-A is the shank's I-section.

use engine.dims (L, back, front, pinlen, rp, rpin)
use engine.parts
use std (offset, vertical)

// What a rod needs of the pin it rides: the pin in the end view, and in the side view.
interface CrankPin { end: point, side: point }

part ConRod(pin: CrankPin, bore: line) {
  // the rod's own dimensions
  wB := pinlen - 4mm     // big end, along the crank axis: the pin less 2 of clearance a side
  wS := 22mm             // small end, along the crank axis
  rB := rp + 1.5mm       // big-end bore: the crank pin and a bearing shell
  rS := rpin + 1mm       // small-end bore: the piston pin and a bush
  eB := 30mm             // big-end eye, outside
  eS := 16mm             // small-end eye, outside
  hB := 12mm             // shank half-width where it leaves the big end…
  hS := 9mm              // …and where it meets the small end
  rf := 6mm              // fillet between shank and eye
  bolt := 22mm           // cap bolt centres, off the rod's axis
  capd := 26mm           // the bolt reaches this far into the cap…
  rodd := 20mm           // …and this far into the rod
  fl := 18mm             // I-section: flange width, the shank's thickness across the engine
  ft := 4mm              // flange thickness
  wt := 5mm              // web thickness
  hM := (hB + hS) / 2    // the shank's half-width at mid-length, where the section is cut
  oil := 1.5mm           // the oil passage, half its bore

  in pin.end.plane {
    // the small end rides the bore axis one rod length from the pin
    sm := point hint((pin.end.x, pin.end.y + L))
    sm coincident bore
    pin.end distance(L) sm
    cl := line(pin.end, sm)
    bigbore := circle(center: pin.end) hint(r: rB)
    radius(rB) bigbore
    smallbore := circle(center: sm) hint(r: rS)
    radius(rS) smallbore

    // the shank's two flanks, each filleted into both eyes.  A fillet is an arc whose centre
    // is a half-width plus a radius off the rod's axis; it meets the eye on the axis from the
    // eye's centre (which is what makes the two arcs tangent there, without the double root a
    // bare circle–circle tangency has) and the flank square to it, the tangency stated at
    // that point (§1.5).  The eyes themselves are drawn as the arcs left between the
    // fillets, the long way round.
    cbl := point hint((pin.end.x - (hB + rf), pin.end.y + 31.2mm))
    cbr := point hint((pin.end.x + (hB + rf), pin.end.y + 31.2mm))
    csl := point hint((sm.x - (hS + rf), sm.y - 16.1mm))
    csr := point hint((sm.x + (hS + rf), sm.y - 16.1mm))
    // (that the centre is `eB + rf` from the pin follows: the contact is on the axis, on the
    // eye and on the fillet, so it is not stated a second time)
    cbl distance(hB + rf, side: left) cl
    cbr distance(hB + rf, side: right) cl
    csl distance(hS + rf, side: left) cl
    csr distance(hS + rf, side: right) cl
    rayBL := line(pin.end, cbl)
    rayBR := line(pin.end, cbr)
    raySL := line(sm, csl)
    raySR := line(sm, csr)
    sbl := point hint((pin.end.x - 15mm, pin.end.y + 26mm))
    sbr := point hint((pin.end.x + 15mm, pin.end.y + 26mm))
    ssl := point hint((sm.x - 10.9mm, sm.y - 11.7mm))
    ssr := point hint((sm.x + 10.9mm, sm.y - 11.7mm))
    sbl coincident rayBL
    sbr coincident rayBR
    ssl coincident raySL
    ssr coincident raySR
    ebl := point hint((pin.end.x - hB, pin.end.y + 31mm))
    ebr := point hint((pin.end.x + hB, pin.end.y + 31mm))
    esl := point hint((sm.x - hS, sm.y - 16mm))
    esr := point hint((sm.x + hS, sm.y - 16mm))
    flank_l := line(ebl, esl)
    flank_r := line(ebr, esr)
    fbl := arc(center: cbl, start: sbl, end: ebl) hint(r: rf)
    fbr := arc(center: cbr, start: ebr, end: sbr) hint(r: rf)
    fsl := arc(center: csl, start: esl, end: ssl) hint(r: rf)
    fsr := arc(center: csr, start: ssr, end: esr) hint(r: rf)
    radius(rf) fbl
    radius(rf) fbr
    radius(rf) fsl
    radius(rf) fsr
    flank_l tangent(at: p1) fbl
    flank_l tangent(at: p2) fsl
    flank_r tangent(at: p1) fbr
    flank_r tangent(at: p2) fsr
    eyeB := arc(center: pin.end, start: sbl, end: sbr) hint(r: eB)
    eyeS := arc(center: sm, start: ssr, end: ssl) hint(r: eS)
    radius(eB) eyeB
    radius(eS) eyeS

    // the cap: a parting line through the pin square to the rod, and the two bolts through it
    pl0 := point hint((pin.end.x - eB, pin.end.y))
    pl1 := point hint((pin.end.x + eB, pin.end.y))
    parting := line(pl0, pl1)
    pin.end midpoint parting
    parting perpendicular cl
    pl0 coincident eyeB
    bl0 := point hint((pin.end.x - bolt, pin.end.y - capd))
    bl1 := point hint((pin.end.x - bolt, pin.end.y + rodd))
    br0 := point hint((pin.end.x + bolt, pin.end.y - capd))
    br1 := point hint((pin.end.x + bolt, pin.end.y + rodd))
    bolt_l := line(bl0, bl1)
    bolt_r := line(br0, br1)
    bl0 distance(bolt, side: left) cl
    bl1 distance(bolt, side: left) cl
    br0 distance(bolt, side: right) cl
    br1 distance(bolt, side: right) cl
    bl0 distance(capd, side: right) parting
    bl1 distance(rodd, side: left) parting
    br0 distance(capd, side: right) parting
    br1 distance(rodd, side: left) parting
    claim bl0 distance(2 * bolt) br0

    // the oil passage, drilled from the big-end bore to the small-end bore
    ol0 := point hint((pin.end.x - oil, pin.end.y + rB))
    ol1 := point hint((sm.x - oil, sm.y - rS))
    or0 := point hint((pin.end.x + oil, pin.end.y + rB))
    or1 := point hint((sm.x + oil, sm.y - rS))
    oil_l := line(ol0, ol1)
    oil_r := line(or0, or1)
    ol0 coincident bigbore
    or0 coincident bigbore
    ol1 coincident smallbore
    or1 coincident smallbore
    ol0 distance(oil, side: left) cl
    ol1 distance(oil, side: left) cl
    or0 distance(oil, side: right) cl
    or1 distance(oil, side: right) cl

    // a pocket's outline, each side of the web: a flange's thickness inside the flanks, as far
    // clear of each eye
    kq1 := point hint((pin.end.x + 7.3mm, pin.end.y + 33.2mm))
    kq2 := point hint((sm.x + 4.6mm, sm.y - 19.5mm))
    kq3 := point hint((sm.x - 4.6mm, sm.y - 19.5mm))
    kq4 := point hint((pin.end.x - 7.3mm, pin.end.y + 33.2mm))
    pocket_r := line(kq1, kq2)
    pocket_l := line(kq3, kq4)
    kq1 distance(ft, side: left) flank_r
    kq2 distance(ft, side: left) flank_r
    kq3 distance(ft, side: right) flank_l
    kq4 distance(ft, side: right) flank_l
    pocket_b := arc(center: pin.end, start: kq1, end: kq4) hint(r: eB + ft)
    pocket_t := arc(center: sm, start: kq3, end: kq2) hint(r: eS + ft)
    radius(eB + ft) pocket_b
    radius(eS + ft) pocket_t

    // section A-A's plane: square to the rod's axis through the middle of the shank, its `u`
    // along the crank axis
    mid := point hint((pin.end.x, pin.end.y + L / 2))
    mid midpoint cl
  }
  cut := plane(u: hint(dir: pin.end.plane.normal),
                v: hint(dir: cl, turn: 90deg, about: pin.end.plane.normal))
  cut.origin coincident mid
  cut.u parallel pin.end.plane.normal
  cut.v perpendicular cl
  cut.v perpendicular cut.u

  in pin.side.plane {
    // the small end, edge on: square above the pin on the bore's line, as high as the end view
    // puts it
    sm_s := point hint((pin.side.x, pin.side.y + L))
    pin.side vertical sm_s
    sm project sm_s
    // the big end: a block `wB` along the axis, the parting line across it, a bolt down it
    ba := point hint((pin.side.x - wB / 2, pin.side.y - eB))
    bb := point hint((pin.side.x + wB / 2, pin.side.y - eB))
    bc := engine.parts.At(pin.side, dx: wB / 2, dy: eB)
    bd := engine.parts.At(pin.side, dx: -wB / 2, dy: eB)
    (b1 := line(ba, bb)) -> (b2 := line(bb, bc.p)) -> (b3 := line(bc.p, bd.p)) -> (b4 := line(bd.p, ba)) -> close
    ba offset(dx: -wB / 2, dy: -eB) pin.side
    pin.side distance(-eB, along: v) bb
    ba distance(wB) bb
    pa := engine.parts.At(pin.side, dx: -wB / 2, dy: 0mm)
    pb := engine.parts.At(pin.side, dx: wB / 2, dy: 0mm)
    parting_s := line(pa.p, pb.p)
    b0 := engine.parts.At(pin.side, dx: 0mm, dy: -capd)
    b1s := engine.parts.At(pin.side, dx: 0mm, dy: rodd)
    bolt_s := line(b0.p, b1s.p)
    // the small end
    sa := point hint((sm_s.x - wS / 2, sm_s.y - eS))
    sb := point hint((sm_s.x + wS / 2, sm_s.y - eS))
    sc := engine.parts.At(sm_s, dx: wS / 2, dy: eS)
    sd := engine.parts.At(sm_s, dx: -wS / 2, dy: eS)
    (s1 := line(sa, sb)) -> (s2 := line(sb, sc.p)) -> (s3 := line(sc.p, sd.p)) -> (s4 := line(sd.p, sa)) -> close
    sa offset(dx: -wS / 2, dy: -eS) sm_s
    sm_s distance(-eS, along: v) sb
    sa distance(wS) sb
    // the shank's flanges between them
    ka := point hint((pin.side.x - fl / 2, pin.side.y + eB))
    kb := point hint((pin.side.x + fl / 2, pin.side.y + eB))
    kc := point hint((sm_s.x + fl / 2, sm_s.y - eS))
    kd := point hint((sm_s.x - fl / 2, sm_s.y - eS))
    k1 := line(ka, kd)
    k2 := line(kb, kc)
    ka offset(dx: -fl / 2, dy: eB) pin.side
    pin.side distance(eB, along: v) kb
    ka distance(fl) kb
    kc offset(dx: fl / 2, dy: -eS) sm_s
    kd offset(dx: -fl / 2, dy: -eS) sm_s
    // the web between the pockets, hidden behind the flanges: only its corners are drawn
    wa := engine.parts.At(pin.side, dx: -wt / 2, dy: 0mm)
    wb := engine.parts.At(pin.side, dx: wt / 2, dy: 0mm)
    wc := engine.parts.At(sm_s, dx: wt / 2, dy: 0mm)
    wd := engine.parts.At(sm_s, dx: -wt / 2, dy: 0mm)
  }

  // section A-A: the shank's I-section at mid-length, about the section plane's origin
  in cut {
    q0 := engine.parts.At(cut.origin, dx: -fl / 2, dy: -hM)
    q1 := point hint((fl / 2, -hM))
    q2 := point hint((fl / 2, -hM + ft))
    q3 := point hint((wt / 2, -hM + ft))
    q4 := engine.parts.At(cut.origin, dx: wt / 2, dy: hM - ft)
    q5 := engine.parts.At(cut.origin, dx: fl / 2, dy: hM - ft)
    q6 := engine.parts.At(cut.origin, dx: fl / 2, dy: hM)
    q7 := point hint((-fl / 2, hM))
    q8 := engine.parts.At(cut.origin, dx: -fl / 2, dy: hM - ft)
    q9 := engine.parts.At(cut.origin, dx: -wt / 2, dy: hM - ft)
    q10 := engine.parts.At(cut.origin, dx: -wt / 2, dy: -hM + ft)
    q11 := engine.parts.At(cut.origin, dx: -fl / 2, dy: -hM + ft)
    (a1 := line(q0.p, q1)) -> (a2 := line(q1, q2)) -> (a3 := line(q2, q3)) -> (a4 := line(q3, q4.p)) ->
      (a5 := line(q4.p, q5.p)) -> (a6 := line(q5.p, q6.p)) -> (a7 := line(q6.p, q7)) -> (a8 := line(q7, q8.p)) ->
      (a9 := line(q8.p, q9.p)) -> (a10 := line(q9.p, q10.p)) -> (a11 := line(q10.p, q11.p)) -> (a12 := line(q11.p, q0.p)) -> close
    cut.origin distance(-hM, along: v) q1
    q0.p distance(fl) q1
    cut.origin distance(fl / 2, along: u) q2
    q1 distance(ft, along: v) q2
    cut.origin distance(-hM + ft, along: v) q3
    q10.p distance(wt) q3
    cut.origin distance(-fl / 2, along: u) q7
    q0.p distance(2 * hM, along: v) q7
  }

  // -- the rod as a solid: what both views agree it is.  The end view's outline, bored at both
  // eyes, runs along the crank axis; the side view's edge-on outline — big end, shank, small end
  // — runs across the engine either side of the bore axis; the rod is what the two have in common,
  // less a pocket each side of the web
  body := solid(face(eyeB, fbr, flank_r, fsr, eyeS, fsl, flank_l, fbl, holes: bigbore, smallbore),
                from: front, to: back)
  edge_on := solid()
  big_s := solid(face(b1, b2, b3, b4), from: -100mm, to: 100mm)
  shank_s := solid(face(ka, kb, kc, kd, -> close), from: -100mm, to: 100mm)
  small_s := solid(face(s1, s2, s3, s4), from: -100mm, to: 100mm)
  big_s union edge_on
  shank_s union edge_on
  small_s union edge_on
  edge_on common body
  // the I: each side of the web, the pocket's outline as deep as the flanges, cut from the rod
  pockets := solid(face(pocket_r, pocket_t, pocket_l, pocket_b), from: front, to: back)
  web_s := solid(face(wa.p, wb.p, wc.p, wd.p, -> close), from: -100mm, to: 100mm)
  web_s cut pockets
  pockets cut body
}

// Open this file to preview cylinder 1's rod: on its crank pin where the crankshaft puts it at the
// table's crank angle, its small end on the bore axis, in both sections and section A-A, and as a
// solid.
preview {
  unit mm
  in std.side {
    top := engine.parts.At(std.side.origin, dx: 0mm, dy: 300mm)
    bore := line(std.side.origin, top.p)
    pe := engine.parts.At(std.side.origin, dx: engine.dims.R * sin(engine.dims.theta),
                          dy: engine.dims.R * cos(engine.dims.theta))
  }
  in std.front {
    ps := engine.parts.At(std.origin, dx: front + 25mm + engine.dims.P / 2,
                          dy: engine.dims.R * cos(engine.dims.theta))
  }
  pin := {end: pe.p, side: ps.p}
  rod := ConRod(pin, bore)
}
