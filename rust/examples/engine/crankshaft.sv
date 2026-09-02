// The crankshaft, designed in one place.
//
// One component, two views (§6.7): looking along the axis it is a section through cylinder 1's
// throw — the main journal, the crank pin in its eye, the web flaring out from the eye to a
// counterweight whose rim is an arc about the axis, and the oil passage drilled from journal to
// pin — with the throw of cylinders 2 and 3 ghosted in a half turn on.  Looking across it is the
// whole shaft edge on: the nose the pulley sits on, five main journals, four pins on the cylinder
// pitch with their webs between, and the flange the flywheel bolts to.  The webs' heights in the
// side view are the crowns and heels of the end view's throws, by projection, inside the part.
//
// An inline four's pins lie in one plane, cylinders 1 and 4 up together and 2 and 3 a half turn
// on, which is why one section and its ghost place all four pins.

use engine.dims
use engine.parts

// the shaft's own dimensions
param eP = rp + 12mm        // the pin's eye, outside
param rcw = 55mm            // the counterweight rim
param hcw = 42mm            // half the web's width at the rim
param wj = 24mm             // a main journal's length along the axis
param wpin = pinlen         // a crank pin's length: the table's, since the rod's big end rides it
param web = P / 2 - (wj + wpin) / 2   // a web's thickness along the axis: what the pitch leaves
param rnose = 16mm          // the nose the pulley sits on
param rflange = 50mm        // the flywheel flange
param wflange = 18mm
param oilr = 2.5mm          // the oil passage, half its bore

// One throw seen along the axis: the pin at `theta` from the bore axis, clockwise from top dead
// centre, in its eye; the web's two flanks leaving the eye tangent and running out to the
// counterweight rim; the rim an arc about the axis, `hcw` either side of the crank arm.  `crown`
// and `heel` are where the arm's line crosses the eye and the rim — the throw's extreme points,
// which the side view reads.
component Throw(o: point, axis: line, theta: Angle) {
  port pin: point hint(x: o.x + R * sin(theta), y: o.y + R * cos(theta))
  line arm(o, pin) class axis
  o distance(R) pin class shown
  axis angle(-theta) arm
  circle kp(center: pin) hint(r: rp)
  radius(rp) kp class shown
  // the eye: the arc of the far side, between the two flank tangents
  point el hint(x: pin.x - eP * cos(theta), y: pin.y + eP * sin(theta))
  point er hint(x: pin.x + eP * cos(theta), y: pin.y - eP * sin(theta))
  arc eye(center: pin, start: er, end: el) hint(r: eP)
  radius(eP) eye class shown
  // the rim: an arc about the axis on the far side from the pin, `hcw` either side of the arm
  point cl hint(x: o.x - rcw * sin(theta - asin(hcw / rcw)), y: o.y - rcw * cos(theta - asin(hcw / rcw)))
  point cr hint(x: o.x - rcw * sin(theta + asin(hcw / rcw)), y: o.y - rcw * cos(theta + asin(hcw / rcw)))
  arc rim(center: o, start: cl, end: cr) hint(r: rcw)
  radius(rcw) rim class shown
  cl distance(hcw) arm
  cr distance(-hcw) arm
  // the flanks, tangent to the eye where they leave it
  line fl(el, cl)
  line fr(er, cr)
  fl tangent(at: p1) eye
  fr tangent(at: p1) eye
  // the crown of the eye and the heel of the rim, on the arm's own line
  port crown: point hint(x: pin.x + eP * sin(theta), y: pin.y + eP * cos(theta))
  port heel: point hint(x: o.x - rcw * sin(theta), y: o.y - rcw * cos(theta))
  crown on eye
  crown on arm
  heel on rim
  heel on arm
  // the oil passage, drilled up the arm from the journal's surface to the pin's
  point oa hint(x: o.x + rj * sin(theta) - oilr * cos(theta), y: o.y + rj * cos(theta) + oilr * sin(theta))
  point ob hint(x: pin.x - rp * sin(theta) - oilr * cos(theta), y: pin.y - rp * cos(theta) + oilr * sin(theta))
  point oc hint(x: o.x + rj * sin(theta) + oilr * cos(theta), y: o.y + rj * cos(theta) - oilr * sin(theta))
  point od hint(x: pin.x - rp * sin(theta) + oilr * cos(theta), y: pin.y - rp * cos(theta) - oilr * sin(theta))
  line oil_l(oa, ob) class hidden
  line oil_r(oc, od) class hidden
  oa distance(oilr) arm
  ob distance(oilr) arm
  oc distance(-oilr) arm
  od distance(-oilr) arm
  ob on kp
  od on kp
  oa distance(rj) o
  oc distance(rj) o
}

// A web seen edge on: a rectangle between `x0` and `x1` along the axis whose top and bottom are
// the heights of two points the end view placed.
component WebSide(o: point, x0: Length, x1: Length, top: point, bottom: point) {
  point a hint(x: o.x + x0, y: top.y)
  point b hint(x: o.x + x1, y: top.y)
  point c hint(x: o.x + x1, y: bottom.y)
  point d hint(x: o.x + x0, y: bottom.y)
  line ab(a, b) -> line bc(b, c) -> line cd(c, d) -> line da(d, a) -> close
  o distance(x0, along: x) a
  top distance(0, along: y) a
  o distance(x1, along: x) b
  top distance(0, along: y) b
  o distance(x1, along: x) c
  bottom distance(0, along: y) c
  o distance(x0, along: x) d
  bottom distance(0, along: y) d
}

component Crankshaft(end: plane, side: plane, o: point, axis: line, o_s: point,
                     draw_end: Int, draw_side: Int) {
  // -- along the axis: the section through cylinder 1 -------------------------------------
  repeat draw_end {
    in end {
      circle main(center: o) hint(r: rj)
      radius(rj) main class shown
      circle path(center: o) hint(r: R) class phantom
      radius(R) path
      t1: Throw(o, axis, theta: theta)
      t2: Throw(o, axis, theta: theta + 180deg) class phantom
    }
  }

  // -- across the axis: the whole shaft edge on ------------------------------------------
  repeat 5 * draw_side as j {
    in side {
      jc: At(o_s, dx: front + 25mm + j * P, dy: 0mm)
      journal: Box(jc.p, x0: -wj / 2, y0: -rj, x1: wj / 2, y1: rj)
    }
  }
  repeat 4 * draw_side as i {
    in side {
      param xc = front + 25mm + P / 2 + i * P
      // cylinders 1 and 4 are up together, 2 and 3 a half turn on
      param k = i * (3 - i) / 2
      param ph = theta + 180deg * k
      port pin_s: point hint(x: o_s.x + xc, y: o_s.y + R * cos(ph))
      o_s distance(xc, along: x) pin_s
      pin: Box(pin_s, x0: -wpin / 2, y0: -rp, x1: wpin / 2, y1: rp)
      // the throw's crown and heel at this cylinder, their heights the end view's
      point ct hint(x: o_s.x + xc, y: o_s.y + (R + eP) * cos(ph))
      point hb hint(x: o_s.x + xc, y: o_s.y - rcw * cos(ph))
      o_s distance(xc, along: x) ct
      o_s distance(xc, along: x) hb
      wl: WebSide(o_s, x0: xc - P / 2 + wj / 2, x1: xc - wpin / 2, top: ct, bottom: hb)
      wr: WebSide(o_s, x0: xc + wpin / 2, x1: xc + P / 2 - wj / 2, top: ct, bottom: hb)
    }
  }
  repeat draw_side {
    in side {
      nose: Box(o_s, x0: front - 70mm, y0: -rnose, x1: front + 25mm - wj / 2, y1: rnose)
      flange: Box(o_s, x0: back - 25mm + wj / 2, y0: -rflange, x1: back - 25mm + wj / 2 + wflange, y1: rflange)
      claim journal[0].a distance(wj, along: x) journal[0].b class shown
      claim pin[0].a distance(wpin, along: x) pin[0].b class shown
    }
  }

  // -- the two views agree: each pin, crown and heel is where the section puts it --------
  repeat draw_end * draw_side {
    t1[0].pin project pin_s[0]
    t2[0].pin project pin_s[1]
    t2[0].pin project pin_s[2]
    t1[0].pin project pin_s[3]
    t1[0].crown project ct[0]
    t2[0].crown project ct[1]
    t2[0].crown project ct[2]
    t1[0].crown project ct[3]
    t1[0].heel project hb[0]
    t2[0].heel project hb[1]
    t2[0].heel project hb[2]
    t1[0].heel project hb[3]
  }
}
