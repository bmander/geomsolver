// The connecting rod, designed in one place.
//
// A part is one component whose body carries its geometry **in each view** — an `in view { … }`
// block per view, over planes the assembly hands it — and the projections that tie those views
// together, so the whole design of the rod is this file: nothing about it is written in the view
// modules, which draw only the castings a view is of.  `draw_end`, `draw_side` and `draw_sec` say
// which of its pictures an instance draws (0 or 1, as `repeat` counts): an inline four's end
// section shows one rod, its side section four, and only one rod need carry the shank section.
//
// Looking along the crankshaft (the end view) the rod is seen in its plane of swing: the big-end
// eye about the crank pin with a bearing shell, split at a parting line square to the rod with two
// cap bolts through it; the small-end eye about the piston pin with a bush; a shank tapering from
// big end to small end and filleted into both eyes; and the drilled oil passage up the middle.
// Looking across (the side view) it is seen edge on — the big end, the shank's flanges and the
// small end each a width along the crankshaft.  Section A-A is the shank's I-section.

use engine.dims
use engine.parts
use std (offset)

component ConRod(end: plane, side: plane, secv: plane,
                 pin: point, ax: line, pin_s: point, sm_s: point, at: point,
                 draw_end: Int, draw_side: Int, draw_sec: Int, dims: group) {
  // the rod's own dimensions
  wB := dims.pinlen - 4mm     // big end, along the crank axis: the pin less 2 of clearance a side
  wS := 22mm             // small end, along the crank axis
  rB := dims.rp + 1.5mm       // big-end bore: the crank pin and a bearing shell
  rS := dims.rpin + 1mm       // small-end bore: the piston pin and a bush
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

  repeat draw_end {
    in end {
      // the small end rides the bore axis one rod length from the pin
      sm := point hint((pin.x, pin.y + dims.L))
      sm coincident ax
      pin distance(dims.L) sm
      cl := line(pin, sm)
      bigbore := circle(center: pin) hint(r: rB)
      radius(rB) bigbore
      smallbore := circle(center: sm) hint(r: rS)
      radius(rS) smallbore

      // the shank's two flanks, each filleted into both eyes.  A fillet is an arc whose centre
      // is a half-width plus a radius off the rod's axis; it meets the eye on the axis from the
      // eye's centre (which is what makes the two arcs tangent there, without the double root a
      // bare circle–circle tangency has) and the flank square to it, the tangency stated at
      // that point (§1.5).  The eyes themselves are drawn as the arcs left between the
      // fillets, the long way round.
      cbl := point hint((pin.x - (hB + rf), pin.y + 31.2mm))
      cbr := point hint((pin.x + (hB + rf), pin.y + 31.2mm))
      csl := point hint((sm.x - (hS + rf), sm.y - 16.1mm))
      csr := point hint((sm.x + (hS + rf), sm.y - 16.1mm))
      // (that the centre is `eB + rf` from the pin follows: the contact is on the axis, on the
      // eye and on the fillet, so it is not stated a second time)
      cbl distance(hB + rf, side: left) cl
      cbr distance(hB + rf, side: right) cl
      csl distance(hS + rf, side: left) cl
      csr distance(hS + rf, side: right) cl
      rayBL := line(pin, cbl)
      rayBR := line(pin, cbr)
      raySL := line(sm, csl)
      raySR := line(sm, csr)
      sbl := point hint((pin.x - 15mm, pin.y + 26mm))
      sbr := point hint((pin.x + 15mm, pin.y + 26mm))
      ssl := point hint((sm.x - 10.9mm, sm.y - 11.7mm))
      ssr := point hint((sm.x + 10.9mm, sm.y - 11.7mm))
      sbl coincident rayBL
      sbr coincident rayBR
      ssl coincident raySL
      ssr coincident raySR
      ebl := point hint((pin.x - hB, pin.y + 31mm))
      ebr := point hint((pin.x + hB, pin.y + 31mm))
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
      eyeB := arc(center: pin, start: sbl, end: sbr) hint(r: eB)
      eyeS := arc(center: sm, start: ssr, end: ssl) hint(r: eS)
      radius(eB) eyeB
      radius(eS) eyeS

      // the cap: a parting line through the pin square to the rod, and the two bolts through it
      pl0 := point hint((pin.x - eB, pin.y))
      pl1 := point hint((pin.x + eB, pin.y))
      parting := line(pl0, pl1)
      pin midpoint parting
      parting perpendicular cl
      pl0 coincident eyeB
      bl0 := point hint((pin.x - bolt, pin.y - capd))
      bl1 := point hint((pin.x - bolt, pin.y + rodd))
      br0 := point hint((pin.x + bolt, pin.y - capd))
      br1 := point hint((pin.x + bolt, pin.y + rodd))
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
      ol0 := point hint((pin.x - oil, pin.y + rB))
      ol1 := point hint((sm.x - oil, sm.y - rS))
      or0 := point hint((pin.x + oil, pin.y + rB))
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
    }
  }

  repeat draw_side {
    in side {
      // the big end: a block `wB` along the axis, the parting line across it, a bolt down it
      ba := point hint((pin_s.x - wB / 2, pin_s.y - eB))
      bb := point hint((pin_s.x + wB / 2, pin_s.y - eB))
      bc := engine.parts.At(pin_s, dx: wB / 2, dy: eB)
      bd := engine.parts.At(pin_s, dx: -wB / 2, dy: eB)
      (b1 := line(ba, bb)) -> (b2 := line(bb, bc.p)) -> (b3 := line(bc.p, bd.p)) -> (b4 := line(bd.p, ba)) -> close
      ba offset(dx: -wB / 2, dy: -eB) pin_s
      pin_s distance(-eB, along: y) bb
      ba distance(wB) bb
      pa := engine.parts.At(pin_s, dx: -wB / 2, dy: 0mm)
      pb := engine.parts.At(pin_s, dx: wB / 2, dy: 0mm)
      parting_s := line(pa.p, pb.p)
      b0 := engine.parts.At(pin_s, dx: 0mm, dy: -capd)
      b1s := engine.parts.At(pin_s, dx: 0mm, dy: rodd)
      bolt_s := line(b0.p, b1s.p)
      // the small end
      sa := point hint((sm_s.x - wS / 2, sm_s.y - eS))
      sb := point hint((sm_s.x + wS / 2, sm_s.y - eS))
      sc := engine.parts.At(sm_s, dx: wS / 2, dy: eS)
      sd := engine.parts.At(sm_s, dx: -wS / 2, dy: eS)
      (s1 := line(sa, sb)) -> (s2 := line(sb, sc.p)) -> (s3 := line(sc.p, sd.p)) -> (s4 := line(sd.p, sa)) -> close
      sa offset(dx: -wS / 2, dy: -eS) sm_s
      sm_s distance(-eS, along: y) sb
      sa distance(wS) sb
      // the shank's flanges between them
      ka := point hint((pin_s.x - fl / 2, pin_s.y + eB))
      kb := point hint((pin_s.x + fl / 2, pin_s.y + eB))
      kc := point hint((sm_s.x + fl / 2, sm_s.y - eS))
      kd := point hint((sm_s.x - fl / 2, sm_s.y - eS))
      k1 := line(ka, kd)
      k2 := line(kb, kc)
      ka offset(dx: -fl / 2, dy: eB) pin_s
      pin_s distance(eB, along: y) kb
      ka distance(fl) kb
      kc offset(dx: fl / 2, dy: -eS) sm_s
      kd offset(dx: -fl / 2, dy: -eS) sm_s
    }
  }

  // the rod's two views agree on where the small end is
  repeat draw_end * draw_side {
    sm[0] project sm_s
  }

  // section A-A: the shank's I-section at mid-length, about `at`
  repeat draw_sec {
    in secv {
      q0 := engine.parts.At(at, dx: -fl / 2, dy: -hM)
      q1 := point hint((at.x + fl / 2, at.y - hM))
      q2 := point hint((at.x + fl / 2, at.y - hM + ft))
      q3 := point hint((at.x + wt / 2, at.y - hM + ft))
      q4 := engine.parts.At(at, dx: wt / 2, dy: hM - ft)
      q5 := engine.parts.At(at, dx: fl / 2, dy: hM - ft)
      q6 := engine.parts.At(at, dx: fl / 2, dy: hM)
      q7 := point hint((at.x - fl / 2, at.y + hM))
      q8 := engine.parts.At(at, dx: -fl / 2, dy: hM - ft)
      q9 := engine.parts.At(at, dx: -wt / 2, dy: hM - ft)
      q10 := engine.parts.At(at, dx: -wt / 2, dy: -hM + ft)
      q11 := engine.parts.At(at, dx: -fl / 2, dy: -hM + ft)
      (a1 := line(q0.p, q1)) -> (a2 := line(q1, q2)) -> (a3 := line(q2, q3)) -> (a4 := line(q3, q4.p)) ->
        (a5 := line(q4.p, q5.p)) -> (a6 := line(q5.p, q6.p)) -> (a7 := line(q6.p, q7)) -> (a8 := line(q7, q8.p)) ->
        (a9 := line(q8.p, q9.p)) -> (a10 := line(q9.p, q10.p)) -> (a11 := line(q10.p, q11.p)) -> (a12 := line(q11.p, q0.p)) -> close
      at distance(-hM, along: y) q1
      q0.p distance(fl) q1
      at distance(fl / 2, along: x) q2
      q1 distance(ft, along: y) q2
      at distance(-hM + ft, along: y) q3
      q10.p distance(wt) q3
      at distance(-fl / 2, along: x) q7
      q0.p distance(2 * hM, along: y) q7
    }
  }
}
