// A Wankel housing's bore as the trochoid's parallel curve. An apex seal's tip is no point: it
// is crowned, a circle of radius `s` whose centre runs on the epitrochoid the motion makes
// (`wankel/wankel.sv` derives that curve from the motion), and the bore it wipes is the
// epitrochoid's parallel curve, `s` outside it — a front moving outward at unit speed, square to
// itself, marched along how far it has moved by the method of lines (#339 D4), as `front.sv`'s
// ellipse moves inward.
//
// The body runs over `d`, how far the front has moved, marched, and `u`, the eccentric's angle,
// sampled over half a turn: the bore is symmetric about the line of its lobes, and the other half
// is this one's mirror. `initially` is the epitrochoid, x = e cos 3u + R cos u,
// y = e sin 3u + R sin u, in the rotor's plane; `nl`, placed at every node, is its outward normal,
// square to `rate(p over u)`; the law and the gauge are `front.sv`'s.
//
// Outward, a convex arc never folds: its parallel curves only grow. An epitrochoid with R/e
// between 3 and 9 is not convex: its waist, at u = 90°, curves inward, its radius of curvature
// there (R − 3e)²/(9e − R), the least of its concave arcs. The design rule is that the seal's
// radius stays below it. This rotor is a short one, R/e = 5 (`wankel.sv`'s is 7, its waist's
// radius 120), so the waist's is 15. `bore` is the front run to the seal's 3: regular. `past`
// takes it on to 20, past the waist's radius, and is refused short of it, at the waist, where a
// node's area |x_d × x_u| = (R − 3e)(1 − d/15) per radian falls to a hundredth of its home's —
// 14.85 for the closed form, and earlier for the march, whose own error grows as the waist
// focuses. `solventc` says how far each is marched.
//
// Closed form: x(d, u) = E(u) + d·n(u), E the epitrochoid and n its outward unit normal (E′
// turned clockwise, normalised): a point of the bore stands `s` from the curve the seal's centre
// runs on, square to it. The fold at the waist, at d = (R − 3e)²/(9e − R) = 15.
unit mm
use std

R := 75mm                 // the generating radius, the rotor's centre to an apex
e := 15mm                 // the eccentricity, the shaft's crank
s := 3mm                  // the apex seal's tip radius

component Bore(c: point, x1: point, y1: point, z1: point, R: Length, e: Length, d: Length,
               u: Angle) {
  xa := line(c, x1)
  ya := line(c, y1)
  za := line(c, z1)
  p := point hint((e * cos(3 * u) + R * cos(u), e * sin(3 * u) + R * sin(u), 0mm))
  k := point hint((1.1 * (e * cos(3 * u) + R * cos(u)), 1.1 * (e * sin(3 * u) + R * sin(u)),
                   0mm))
  nl := line(p, k)
  length(10mm) nl
  c distance(0mm, along: za) k               // the normal in the rotor's plane...
  nl perpendicular rate(p over u)            // ...square to the front
  rate(p over d) parallel nl                 // the gauge: square to itself
  rate(p over d) distance(1, along: nl)      // the law: at unit speed
  initially {
    c distance(e * cos(3 * u) + R * cos(u), along: xa) p   // the Cauchy data: the epitrochoid
    c distance(e * sin(3 * u) + R * sin(u), along: ya) p
    c distance(0mm, along: za) p
  }
}

c := point hint((0, 0, 0))
fix((0, 0, 0)) c
x1 := point hint((10, 0, 0))
fix((10, 0, 0)) x1
y1 := point hint((0, 10, 0))
fix((0, 10, 0)) y1
z1 := point hint((0, 0, 10))
fix((0, 0, 10)) z1

bore := Bore(c, x1, y1, z1, R: R, e: e).p over (d in (0mm, s), u in (0deg, 180deg))
past := Bore(c, x1, y1, z1, R: R, e: e).p over (d in (0mm, 20mm), u in (0deg, 180deg))
