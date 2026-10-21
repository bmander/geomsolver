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

use engine.dims (P, R, back, front, pinlen, rflange, rfw, rj, rp, rseal, wflange, wfw)
use engine.parts
use std (horizontal)

// One throw seen along the axis: the pin at `theta` from the bore axis, clockwise from top dead
// centre, in its eye; the web's two flanks leaving the eye tangent and running out to the
// counterweight rim; the rim an arc about the axis, `hcw` either side of the crank arm.  `crown`
// and `heel` are where the arm's line crosses the eye and the rim — the throw's extreme points,
// which the side view reads.
part Throw(o: point, ax: line, theta: Angle, shape: group) {
  pin := point hint((o.x + R * sin(theta), o.y + R * cos(theta)))
  arm := line(o, pin)
  o distance(R) pin
  ax angle(theta, sense: cw) arm
  kp := circle(center: pin) hint(r: rp)
  radius(rp) kp
  // the eye: the arc of the far side, between the two flank tangents
  el := point hint((pin.x - shape.eye * cos(theta), pin.y + shape.eye * sin(theta)))
  er := point hint((pin.x + shape.eye * cos(theta), pin.y - shape.eye * sin(theta)))
  eye := arc(center: pin, start: er, end: el) hint(r: shape.eye)
  radius(shape.eye) eye
  // the rim: an arc about the axis on the far side from the pin, `shape.half_width` either side of the arm
  cl := point hint((o.x - shape.rim * sin(theta + asin(shape.half_width / shape.rim)), o.y - shape.rim * cos(theta + asin(shape.half_width / shape.rim))))
  cr := point hint((o.x - shape.rim * sin(theta - asin(shape.half_width / shape.rim)), o.y - shape.rim * cos(theta - asin(shape.half_width / shape.rim))))
  rim := arc(center: o, start: cl, end: cr) hint(r: shape.rim)
  radius(shape.rim) rim
  cl distance(shape.half_width, side: left) arm
  cr distance(shape.half_width, side: right) arm
  // the flanks, tangent to the eye where they leave it
  fl := line(el, cl)
  fr := line(er, cr)
  fl tangent(at: p1) eye
  fr tangent(at: p1) eye
  // the crown of the eye and the heel of the rim, on the arm's own line
  crown := point hint((pin.x + shape.eye * sin(theta), pin.y + shape.eye * cos(theta)))
  heel := point hint((o.x - shape.rim * sin(theta), o.y - shape.rim * cos(theta)))
  crown coincident eye
  crown coincident arm
  heel coincident rim
  heel coincident arm
  // the oil passage, drilled up the arm from the journal's surface to the pin's
  oa := point hint((o.x + rj * sin(theta) - shape.oil_radius * cos(theta), o.y + rj * cos(theta) + shape.oil_radius * sin(theta)))
  ob := point hint((pin.x - rp * sin(theta) - shape.oil_radius * cos(theta), pin.y - rp * cos(theta) + shape.oil_radius * sin(theta)))
  oc := point hint((o.x + rj * sin(theta) + shape.oil_radius * cos(theta), o.y + rj * cos(theta) - shape.oil_radius * sin(theta)))
  od := point hint((pin.x - rp * sin(theta) + shape.oil_radius * cos(theta), pin.y - rp * cos(theta) - shape.oil_radius * sin(theta)))
  oil_l := line(oa, ob)
  oil_r := line(oc, od)
  oa distance(shape.oil_radius, side: left) arm
  ob distance(shape.oil_radius, side: left) arm
  oc distance(shape.oil_radius, side: right) arm
  od distance(shape.oil_radius, side: right) arm
  ob coincident kp
  od coincident kp
  oa distance(rj) o
  oc distance(rj) o
  // which root each point takes, the arm running from `o` up to the pin: each read against
  // points already placed, or against a rim corner, whose side of the arm is stated and whose
  // height up it turns none of these the other way.  The throw only turns about `o`, so no
  // triangle here comes near a line.  The passage's journal end on the pin's side of the axis,
  // its pin end on the axis's side of the pin
  cw(o, cl, oa)
  ccw(o, cr, oc)
  ccw(pin, cl, ob)
  cw(pin, cr, od)
  // the heel behind the axis, the crown beyond the pin
  ccw(o, cl, heel)
  cw(pin, cl, crown)
  // the rim's corners behind the passage's journal end, so on the far side of the axis
  cw(oa, oc, cl)
  cw(oa, oc, cr)
  // the eye inside each flank: the tangent from the rim's corner that wraps it
  ccw(el, cl, pin)
  cw(er, cr, pin)
}

// A web seen edge on: a rectangle between `x0` and `x1` along the axis whose top and bottom are
// the heights of two points the end view placed.
part WebSide(o: point, x0: Length, x1: Length, top: point, bottom: point) {
  a := point hint((o.x + x0, top.y))
  b := point hint((o.x + x1, top.y))
  c := point hint((o.x + x1, bottom.y))
  d := point hint((o.x + x0, bottom.y))
  (ab := line(a, b)) -> (bc := line(b, c)) -> (cd := line(c, d)) -> (da := line(d, a)) -> close
  o distance(x0, along: u) a
  top horizontal a
  o distance(x1, along: u) b
  top horizontal b
  o distance(x1, along: u) c
  bottom horizontal c
  o distance(x0, along: u) d
  bottom horizontal d
}

// The crankshaft on its axis: `o` and `ax` where the end section shows it, `o_s` where the side
// section does, and `theta` the crank's turn — a formal, so the assembly says where it stands, or
// leaves it to the solve.  Each pin is read by the rod that rides it, in both views at once
// (`pin1` to `pin4`, cylinder by cylinder).
part Crankshaft(o: point, ax: line, o_s: point, theta: Angle) {
  // the shaft's own dimensions
  eP := rp + 12mm        // the pin's eye, outside
  rcw := 55mm            // the counterweight rim
  hcw := 42mm            // half the web's width at the rim
  wj := 24mm             // a main journal's length along the axis
  wpin := pinlen         // a crank pin's length: the table's, since the rod's big end rides it
  web := P / 2 - (wj + wpin) / 2   // a web's thickness along the axis: what the pitch leaves
  rnose := 16mm          // the nose the pulley sits on
  oilr := 2.5mm          // the oil passage, half its bore
  throw_dims := {eye: eP, rim: rcw, half_width: hcw, oil_radius: oilr}

  // -- along the axis: the section through cylinder 1 -------------------------------------
  in o.plane {
    main := circle(center: o) hint(r: rj)
    radius(rj) main
    path := circle(center: o) hint(r: R)
    radius(R) path
    t1 := Throw(o, ax, theta: theta, shape: throw_dims)
    t2 := Throw(o, ax, theta: theta + 180deg, shape: throw_dims)
  }

  // -- across the axis: the whole shaft edge on ------------------------------------------
  repeat 5 as j {
    in o_s.plane {
      jc := engine.parts.At(o_s, dx: front + 25mm + j * P, dy: 0mm)
      journal := engine.parts.Box(jc.p, x0: -wj / 2, y0: -rj, x1: wj / 2, y1: rj)
    }
  }
  repeat 4 as i {
    in o_s.plane {
      xc := front + 25mm + P / 2 + i * P
      // cylinders 1 and 4 are up together, 2 and 3 a half turn on
      k := i * (3 - i) / 2
      ph := theta + 180deg * k
      pin_s := point hint((o_s.x + xc, o_s.y + R * cos(ph)))
      o_s distance(xc, along: u) pin_s
      pin := engine.parts.Box(pin_s, x0: -wpin / 2, y0: -rp, x1: wpin / 2, y1: rp)
      // the throw's crown and heel at this cylinder, their heights the end view's
      ct := point hint((o_s.x + xc, o_s.y + (R + eP) * cos(ph)))
      hb := point hint((o_s.x + xc, o_s.y - rcw * cos(ph)))
      o_s distance(xc, along: u) ct
      o_s distance(xc, along: u) hb
      wl := WebSide(o_s, x0: xc - P / 2 + wj / 2, x1: xc - wpin / 2, top: ct, bottom: hb)
      wr := WebSide(o_s, x0: xc + wpin / 2, x1: xc + P / 2 - wj / 2, top: ct, bottom: hb)
    }
  }
  in o_s.plane {
    // the nose the pulley sits on, forward of the first journal; behind the last, the seal
    // journal through the block's rear wall, then the flange and the flywheel on it
    nose := engine.parts.Box(o_s, x0: front - 70mm, y0: -rnose, x1: front + 25mm - wj / 2, y1: rnose)
    seal := engine.parts.Box(o_s, x0: back - 25mm + wj / 2, y0: -rseal, x1: back + 8mm, y1: rseal)
    flange := engine.parts.Box(o_s, x0: back + 8mm, y0: -rflange, x1: back + 8mm + wflange, y1: rflange)
    flywheel := engine.parts.Box(o_s, x0: back + 8mm + wflange, y0: -rfw, x1: back + 8mm + wflange + wfw, y1: rfw)
    claim journal[0].a distance(wj, along: u) journal[0].b
    claim pin[0].a distance(wpin, along: u) pin[0].b
  }

  // -- the two views agree: each pin, crown and heel is where the section puts it --------
  t1.pin project pin_s[0]
  t2.pin project pin_s[1]
  t2.pin project pin_s[2]
  t1.pin project pin_s[3]
  t1.crown project ct[0]
  t2.crown project ct[1]
  t2.crown project ct[2]
  t1.crown project ct[3]
  t1.heel project hb[0]
  t2.heel project hb[1]
  t2.heel project hb[2]
  t1.heel project hb[3]

  // -- what rides on it: each cylinder's pin in both views, cylinders 1 and 4 on `t1`, 2 and 3
  // on `t2` (an inline four's pins lie in one plane)
  pin1 := {end: t1.pin, side: pin_s[0]}
  pin2 := {end: t2.pin, side: pin_s[1]}
  pin3 := {end: t2.pin, side: pin_s[2]}
  pin4 := {end: t1.pin, side: pin_s[3]}
}
