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

// One throw seen along the axis: the pin at `theta` from the bore axis, clockwise from top dead
// centre, in its eye; the web's two flanks leaving the eye tangent and running out to the
// counterweight rim; the rim an arc about the axis, `hcw` either side of the crank arm.  `crown`
// and `heel` are where the arm's line crosses the eye and the rim — the throw's extreme points,
// which the side view reads.
component Throw(o: point, axis: line, theta: Angle, dims: group, shape: group) {
  pin := point hint(x: o.x + dims.R * sin(theta), y: o.y + dims.R * cos(theta))
  arm := line(o, pin)
  o distance(dims.R) pin
  axis angle(theta, sense: cw) arm
  kp := circle(center: pin) hint(r: dims.rp)
  radius(dims.rp) kp
  // the eye: the arc of the far side, between the two flank tangents
  el := point hint(x: pin.x - shape.eye * cos(theta), y: pin.y + shape.eye * sin(theta))
  er := point hint(x: pin.x + shape.eye * cos(theta), y: pin.y - shape.eye * sin(theta))
  eye := arc(center: pin, start: er, end: el) hint(r: shape.eye)
  radius(shape.eye) eye
  // the rim: an arc about the axis on the far side from the pin, `shape.half_width` either side of the arm
  cl := point hint(x: o.x - shape.rim * sin(theta - asin(shape.half_width / shape.rim)), y: o.y - shape.rim * cos(theta - asin(shape.half_width / shape.rim)))
  cr := point hint(x: o.x - shape.rim * sin(theta + asin(shape.half_width / shape.rim)), y: o.y - shape.rim * cos(theta + asin(shape.half_width / shape.rim)))
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
  crown := point hint(x: pin.x + shape.eye * sin(theta), y: pin.y + shape.eye * cos(theta))
  heel := point hint(x: o.x - shape.rim * sin(theta), y: o.y - shape.rim * cos(theta))
  crown coincident eye
  crown coincident arm
  heel coincident rim
  heel coincident arm
  // the oil passage, drilled up the arm from the journal's surface to the pin's
  oa := point hint(x: o.x + dims.rj * sin(theta) - shape.oil_radius * cos(theta), y: o.y + dims.rj * cos(theta) + shape.oil_radius * sin(theta))
  ob := point hint(x: pin.x - dims.rp * sin(theta) - shape.oil_radius * cos(theta), y: pin.y - dims.rp * cos(theta) + shape.oil_radius * sin(theta))
  oc := point hint(x: o.x + dims.rj * sin(theta) + shape.oil_radius * cos(theta), y: o.y + dims.rj * cos(theta) - shape.oil_radius * sin(theta))
  od := point hint(x: pin.x - dims.rp * sin(theta) + shape.oil_radius * cos(theta), y: pin.y - dims.rp * cos(theta) - shape.oil_radius * sin(theta))
  oil_l := line(oa, ob)
  oil_r := line(oc, od)
  oa distance(shape.oil_radius, side: left) arm
  ob distance(shape.oil_radius, side: left) arm
  oc distance(shape.oil_radius, side: right) arm
  od distance(shape.oil_radius, side: right) arm
  ob coincident kp
  od coincident kp
  oa distance(dims.rj) o
  oc distance(dims.rj) o
}

// A web seen edge on: a rectangle between `x0` and `x1` along the axis whose top and bottom are
// the heights of two points the end view placed.
component WebSide(o: point, x0: Length, x1: Length, top: point, bottom: point) {
  a := point hint(x: o.x + x0, y: top.y)
  b := point hint(x: o.x + x1, y: top.y)
  c := point hint(x: o.x + x1, y: bottom.y)
  d := point hint(x: o.x + x0, y: bottom.y)
  (ab := line(a, b)) -> (bc := line(b, c)) -> (cd := line(c, d)) -> (da := line(d, a)) -> close
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
                     draw_end: Int, draw_side: Int, dims: group) {
  // the shaft's own dimensions
  eP := dims.rp + 12mm        // the pin's eye, outside
  rcw := 55mm            // the counterweight rim
  hcw := 42mm            // half the web's width at the rim
  wj := 24mm             // a main journal's length along the axis
  wpin := dims.pinlen         // a crank pin's length: the table's, since the rod's big end rides it
  web := dims.P / 2 - (wj + wpin) / 2   // a web's thickness along the axis: what the pitch leaves
  rnose := 16mm          // the nose the pulley sits on
  oilr := 2.5mm          // the oil passage, half its bore
  throw_dims := {eye: eP, rim: rcw, half_width: hcw, oil_radius: oilr}

  // -- along the axis: the section through cylinder 1 -------------------------------------
  repeat draw_end {
    in end {
      main := circle(center: o) hint(r: dims.rj)
      radius(dims.rj) main
      path := circle(center: o) hint(r: dims.R)
      radius(dims.R) path
      t1 := Throw(o, axis, theta: dims.theta, dims: dims, shape: throw_dims)
      t2 := Throw(o, axis, theta: dims.theta + 180deg, dims: dims, shape: throw_dims)
    }
  }

  // -- across the axis: the whole shaft edge on ------------------------------------------
  repeat 5 * draw_side as j {
    in side {
      jc := engine.parts.At(o_s, dx: dims.front + 25mm + j * dims.P, dy: 0mm)
      journal := engine.parts.Box(jc.p, x0: -wj / 2, y0: -dims.rj, x1: wj / 2, y1: dims.rj)
    }
  }
  repeat 4 * draw_side as i {
    in side {
      xc := dims.front + 25mm + dims.P / 2 + i * dims.P
      // cylinders 1 and 4 are up together, 2 and 3 a half turn on
      k := i * (3 - i) / 2
      ph := dims.theta + 180deg * k
      pin_s := point hint(x: o_s.x + xc, y: o_s.y + dims.R * cos(ph))
      o_s distance(xc, along: x) pin_s
      pin := engine.parts.Box(pin_s, x0: -wpin / 2, y0: -dims.rp, x1: wpin / 2, y1: dims.rp)
      // the throw's crown and heel at this cylinder, their heights the end view's
      ct := point hint(x: o_s.x + xc, y: o_s.y + (dims.R + eP) * cos(ph))
      hb := point hint(x: o_s.x + xc, y: o_s.y - rcw * cos(ph))
      o_s distance(xc, along: x) ct
      o_s distance(xc, along: x) hb
      wl := WebSide(o_s, x0: xc - dims.P / 2 + wj / 2, x1: xc - wpin / 2, top: ct, bottom: hb)
      wr := WebSide(o_s, x0: xc + wpin / 2, x1: xc + dims.P / 2 - wj / 2, top: ct, bottom: hb)
    }
  }
  repeat draw_side {
    in side {
      // the nose the pulley sits on, forward of the first journal; behind the last, the seal
      // journal through the block's rear wall, then the flange and the flywheel on it
      nose := engine.parts.Box(o_s, x0: dims.front - 70mm, y0: -rnose, x1: dims.front + 25mm - wj / 2, y1: rnose)
      seal := engine.parts.Box(o_s, x0: dims.back - 25mm + wj / 2, y0: -dims.rseal, x1: dims.back + 8mm, y1: dims.rseal)
      flange := engine.parts.Box(o_s, x0: dims.back + 8mm, y0: -dims.rflange, x1: dims.back + 8mm + dims.wflange, y1: dims.rflange)
      flywheel := engine.parts.Box(o_s, x0: dims.back + 8mm + dims.wflange, y0: -dims.rfw, x1: dims.back + 8mm + dims.wflange + dims.wfw, y1: dims.rfw)
      claim journal[0].a distance(wj, along: x) journal[0].b
      claim pin[0].a distance(wpin, along: x) pin[0].b
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
