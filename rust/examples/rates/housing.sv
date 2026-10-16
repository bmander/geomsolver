// A Wankel housing's bore as the trochoid's parallel curve. An apex seal's tip is no point: it
// is crowned, a circle of radius `s` whose centre runs on the epitrochoid the motion makes
// (`wankel/wankel.sv` derives that curve from the motion), and the bore it wipes is the
// epitrochoid's parallel curve, `s` outside it — a front moving outward at unit speed, square to
// itself, marched along how far it has moved by the method of lines (#339 D4), as `front.sv`'s
// ellipse moves inward.
//
// The body runs over `d`, how far the front has moved, marched, and `u`, the eccentric's angle,
// sampled over half a turn: the bore is symmetric about the line of its lobes, and the other half
// is this one's mirror. `initially` is the Cauchy data, the apex's path: the apex as the rotor's
// motion carries it (#377), drawn and moved as `wankel.sv` draws and moves it — the rotor's
// centre `e` from the shaft's, the apex `R` beyond, the rotor turning a third as fast as the
// shaft against it. Over a turn of the shaft `t` the apex runs the epitrochoid
// x = e cos t + R cos(t/3), y = e sin t + R sin(t/3), so the eccentric's angle is `u = t/3`.
// `nl`, placed at every node square to `rate(p over u)`, is the curve's normal toward the shaft,
// which its seed at the centre picks; the gauge is `front.sv`'s, and the law moves the front away
// from `nl` at unit speed: outward.
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
use std (right_of)

R := 75mm                 // the generating radius, the rotor's centre to an apex
e := 15mm                 // the eccentricity, the shaft's crank
s := 3mm                  // the apex seal's tip radius

// -- the motion, as `wankel.sv` draws it --------------------------------------------------------
in std.top {
  private centre := point
  centre coincident std.top.origin
  private hub := point hint((15, 0))
  hub right_of(d: e) centre
  apex := point hint((90, 0))
  apex right_of(d: R) hub
}
counter := frame
std.Spin(counter, about: centre, on: std.top, ratio: -1)
spin := frame
std.Spin(spin, about: hub, on: std.top, ratio: -2 / 3)   // the rotor, seen from the housing

// -- the bore -----------------------------------------------------------------------------------
component Bore(c: point, z1: point, apex: point, m: frame, seen_from: frame, d: Length, u: Angle) {
  za := line(c, z1)
  p := point hint((0, 0, 0))
  k := point hint((0, 0, 0))
  nl := line(p, k)
  length(10mm) nl
  c distance(0mm, along: za) k               // the normal in the rotor's plane...
  nl perpendicular rate(p over u)            // ...square to the front
  rate(p over d) parallel nl                 // the gauge: square to itself
  rate(p over d) distance(-1, along: nl)     // the law: at unit speed, away from `nl`
  initially {
    // the Cauchy data: the apex the rotor carries
    p coincident(under: m, seen_from: seen_from, t == 3 * u) apex
  }
}

c := point hint((0, 0, 0))
fix((0, 0, 0)) c
z1 := point hint((0, 0, 10))
fix((0, 0, 10)) z1

bore := Bore(c, z1, apex, spin, counter).p over (d in (0mm, s), u in (0deg, 180deg))
past := Bore(c, z1, apex, spin, counter).p over (d in (0mm, 20mm), u in (0deg, 180deg))
