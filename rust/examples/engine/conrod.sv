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

// the rod's own dimensions
param wB = 24mm             // big end, along the crank axis
param wS = 22mm             // small end, along the crank axis
param rB = rp + 1.5mm       // big-end bore: the crank pin and a bearing shell
param rS = rpin + 1mm       // small-end bore: the piston pin and a bush
param eB = 30mm             // big-end eye, outside
param eS = 16mm             // small-end eye, outside
param hB = 12mm             // shank half-width where it leaves the big end…
param hS = 9mm              // …and where it meets the small end
param rf = 6mm              // fillet between shank and eye
param bolt = 22mm           // cap bolt centres, off the rod's axis
param capd = 26mm           // the bolt reaches this far into the cap…
param rodd = 20mm           // …and this far into the rod
param fl = 18mm             // I-section: flange width, the shank's thickness across the engine
param ft = 4mm              // flange thickness
param wt = 5mm              // web thickness
param hM = (hB + hS) / 2    // the shank's half-width at mid-length, where the section is cut
param oil = 1.5mm           // the oil passage, half its bore

component ConRod(end: plane, side: plane, secv: plane,
                 pin: point, axis: line, pin_s: point, sm_s: point, at: point,
                 draw_end: Int, draw_side: Int, draw_sec: Int) {
  repeat draw_end {
    in end {
      // the small end rides the bore axis one rod length from the pin
      point sm hint(x: pin.x, y: pin.y + L)
      sm on axis
      pin distance(L) sm class shown
      line cl(pin, sm) class axis
      circle bigbore(center: pin) hint(r: rB)
      radius(rB) bigbore class shown
      circle smallbore(center: sm) hint(r: rS)
      radius(rS) smallbore class shown

      // the shank's two flanks, each filleted into both eyes.  A fillet is an arc whose centre
      // is a half-width plus a radius off the rod's axis; it meets the eye on the ray from the
      // eye's centre (which is what makes the two arcs tangent there, without the double root a
      // bare circle–circle tangency has) and the flank square to it, the tangency stated at
      // that point (§1.5).  The eyes themselves are drawn as the arcs left between the
      // fillets, the long way round.
      point cbl hint(x: pin.x - (hB + rf), y: pin.y + 31.2mm)
      point cbr hint(x: pin.x + (hB + rf), y: pin.y + 31.2mm)
      point csl hint(x: sm.x - (hS + rf), y: sm.y - 16.1mm)
      point csr hint(x: sm.x + (hS + rf), y: sm.y - 16.1mm)
      // (that the centre is `eB + rf` from the pin follows: the contact is on the ray, on the
      // eye and on the fillet, so it is not stated a second time)
      cbl distance(hB + rf) cl
      cbr distance(-(hB + rf)) cl
      csl distance(hS + rf) cl
      csr distance(-(hS + rf)) cl
      line rayBL(pin, cbl) class hidden
      line rayBR(pin, cbr) class hidden
      line raySL(sm, csl) class hidden
      line raySR(sm, csr) class hidden
      point sbl hint(x: pin.x - 15mm, y: pin.y + 26mm)
      point sbr hint(x: pin.x + 15mm, y: pin.y + 26mm)
      point ssl hint(x: sm.x - 10.9mm, y: sm.y - 11.7mm)
      point ssr hint(x: sm.x + 10.9mm, y: sm.y - 11.7mm)
      sbl on rayBL
      sbr on rayBR
      ssl on raySL
      ssr on raySR
      point ebl hint(x: pin.x - hB, y: pin.y + 31mm)
      point ebr hint(x: pin.x + hB, y: pin.y + 31mm)
      point esl hint(x: sm.x - hS, y: sm.y - 16mm)
      point esr hint(x: sm.x + hS, y: sm.y - 16mm)
      line flank_l(ebl, esl)
      line flank_r(ebr, esr)
      arc fbl(center: cbl, start: sbl, end: ebl) hint(r: rf)
      arc fbr(center: cbr, start: ebr, end: sbr) hint(r: rf)
      arc fsl(center: csl, start: esl, end: ssl) hint(r: rf)
      arc fsr(center: csr, start: ssr, end: esr) hint(r: rf)
      radius(rf) fbl
      radius(rf) fbr
      radius(rf) fsl
      radius(rf) fsr
      flank_l tangent(at: p1) fbl
      flank_l tangent(at: p2) fsl
      flank_r tangent(at: p1) fbr
      flank_r tangent(at: p2) fsr
      arc eyeB(center: pin, start: sbl, end: sbr) hint(r: eB)
      arc eyeS(center: sm, start: ssr, end: ssl) hint(r: eS)
      radius(eB) eyeB class shown
      radius(eS) eyeS

      // the cap: a parting line through the pin square to the rod, and the two bolts through it
      point pl0 hint(x: pin.x - eB, y: pin.y)
      point pl1 hint(x: pin.x + eB, y: pin.y)
      line parting(pl0, pl1)
      pin midpoint parting
      parting perpendicular cl
      pl0 on eyeB
      point bl0 hint(x: pin.x - bolt, y: pin.y - capd)
      point bl1 hint(x: pin.x - bolt, y: pin.y + rodd)
      point br0 hint(x: pin.x + bolt, y: pin.y - capd)
      point br1 hint(x: pin.x + bolt, y: pin.y + rodd)
      line bolt_l(bl0, bl1) class hidden
      line bolt_r(br0, br1) class hidden
      bl0 distance(bolt) cl
      bl1 distance(bolt) cl
      br0 distance(-bolt) cl
      br1 distance(-bolt) cl
      bl0 distance(-capd) parting
      bl1 distance(rodd) parting
      br0 distance(-capd) parting
      br1 distance(rodd) parting
      claim bl0 distance(2 * bolt) br0 class shown

      // the oil passage, drilled from the big-end bore to the small-end bore
      point ol0 hint(x: pin.x - oil, y: pin.y + rB)
      point ol1 hint(x: sm.x - oil, y: sm.y - rS)
      point or0 hint(x: pin.x + oil, y: pin.y + rB)
      point or1 hint(x: sm.x + oil, y: sm.y - rS)
      line oil_l(ol0, ol1) class hidden
      line oil_r(or0, or1) class hidden
      ol0 on bigbore
      or0 on bigbore
      ol1 on smallbore
      or1 on smallbore
      ol0 distance(oil) cl
      ol1 distance(oil) cl
      or0 distance(-oil) cl
      or1 distance(-oil) cl
    }
  }

  repeat draw_side {
    in side {
      // the big end: a block `wB` along the axis, the parting line across it, a bolt down it
      point ba hint(x: pin_s.x - wB / 2, y: pin_s.y - eB)
      point bb hint(x: pin_s.x + wB / 2, y: pin_s.y - eB)
      bc: At(pin_s, dx: wB / 2, dy: eB)
      bd: At(pin_s, dx: -wB / 2, dy: eB)
      line b1(ba, bb) -> line b2(bb, bc.p) -> line b3(bc.p, bd.p) -> line b4(bd.p, ba) -> close
      pin_s distance(-wB / 2, along: x) ba
      pin_s distance(-eB, along: y) ba
      pin_s distance(-eB, along: y) bb
      ba distance(wB) bb class shown
      pa: At(pin_s, dx: -wB / 2, dy: 0mm)
      pb: At(pin_s, dx: wB / 2, dy: 0mm)
      line parting_s(pa.p, pb.p)
      b0: At(pin_s, dx: 0mm, dy: -capd)
      b1s: At(pin_s, dx: 0mm, dy: rodd)
      line bolt_s(b0.p, b1s.p) class hidden
      // the small end
      point sa hint(x: sm_s.x - wS / 2, y: sm_s.y - eS)
      point sb hint(x: sm_s.x + wS / 2, y: sm_s.y - eS)
      sc: At(sm_s, dx: wS / 2, dy: eS)
      sd: At(sm_s, dx: -wS / 2, dy: eS)
      line s1(sa, sb) -> line s2(sb, sc.p) -> line s3(sc.p, sd.p) -> line s4(sd.p, sa) -> close
      sm_s distance(-wS / 2, along: x) sa
      sm_s distance(-eS, along: y) sa
      sm_s distance(-eS, along: y) sb
      sa distance(wS) sb class shown
      // the shank's flanges between them
      point ka hint(x: pin_s.x - fl / 2, y: pin_s.y + eB)
      point kb hint(x: pin_s.x + fl / 2, y: pin_s.y + eB)
      point kc hint(x: sm_s.x + fl / 2, y: sm_s.y - eS)
      point kd hint(x: sm_s.x - fl / 2, y: sm_s.y - eS)
      line k1(ka, kd)
      line k2(kb, kc)
      pin_s distance(-fl / 2, along: x) ka
      pin_s distance(eB, along: y) ka
      pin_s distance(eB, along: y) kb
      ka distance(fl) kb class shown
      sm_s distance(fl / 2, along: x) kc
      sm_s distance(-eS, along: y) kc
      sm_s distance(-fl / 2, along: x) kd
      sm_s distance(-eS, along: y) kd
    }
  }

  // the rod's two views agree on where the small end is
  repeat draw_end * draw_side {
    sm[0] project sm_s
  }

  // section A-A: the shank's I-section at mid-length, about `at`
  repeat draw_sec {
    in secv {
      q0: At(at, dx: -fl / 2, dy: -hM)
      point q1 hint(x: at.x + fl / 2, y: at.y - hM)
      point q2 hint(x: at.x + fl / 2, y: at.y - hM + ft)
      point q3 hint(x: at.x + wt / 2, y: at.y - hM + ft)
      q4: At(at, dx: wt / 2, dy: hM - ft)
      q5: At(at, dx: fl / 2, dy: hM - ft)
      q6: At(at, dx: fl / 2, dy: hM)
      point q7 hint(x: at.x - fl / 2, y: at.y + hM)
      q8: At(at, dx: -fl / 2, dy: hM - ft)
      q9: At(at, dx: -wt / 2, dy: hM - ft)
      q10: At(at, dx: -wt / 2, dy: -hM + ft)
      q11: At(at, dx: -fl / 2, dy: -hM + ft)
      line a1(q0.p, q1) -> line a2(q1, q2) -> line a3(q2, q3) -> line a4(q3, q4.p) ->
        line a5(q4.p, q5.p) -> line a6(q5.p, q6.p) -> line a7(q6.p, q7) -> line a8(q7, q8.p) ->
        line a9(q8.p, q9.p) -> line a10(q9.p, q10.p) -> line a11(q10.p, q11.p) -> line a12(q11.p, q0.p) -> close
      at distance(-hM, along: y) q1
      q0.p distance(fl) q1 class shown
      at distance(fl / 2, along: x) q2
      q1 distance(ft, along: y) q2 class shown
      at distance(-hM + ft, along: y) q3
      q10.p distance(wt) q3 class shown
      at distance(-fl / 2, along: x) q7
      q0.p distance(2 * hM, along: y) q7 class shown
    }
  }
}
