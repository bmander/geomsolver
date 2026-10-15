// D0 (#339): the hanging chain as a stated flow, probed in today's spelling. The target is
// `../catenary_flow.sv`.
//
// The body without its rate rows is `Pull`: the tension at `p` drawn as the line `tl` to `q`, its
// run `c` and its rise `v + s` — the weight of the chain between the home and `p` adds to what
// pulls up. It leaves `p` free (two freedoms), which the rate rows supply away from the home (`p`
// runs along `tl` at unit speed), and `c` and `v`, which are the drawing's.
//   - At the home (s = 0) the drawing holds `p` at `a`. The report is DOF 2: `c` and `v`, which
//     only the flow's far end can settle (`b` on the chain at s = 150). That is a two-point
//     problem the drawing solves by shooting, as §9.10's free curve solves its own.
//   - The closed form is drawn as today's computed curve, with c the root of c·sinh(50/c) = 75
//     (30.8236…, by bisection) and v = −75. The rate rows are claimed at s1 through today's
//     tangency, and today's free curve (`catenary.sv`'s: a to b, 150 long, its height least) is
//     claimed through the closed form at s1 and at the lowest point. All theorems.
// Closed form: x = 50 + c·asinh((s − 75)/c), y = √(c² + (s − 75)²) − √(c² + 75²).
unit mm
use std

c := 30.823646973604035mm
s1 := 40mm

component Pull(c: Length, v: Length, s: Length) {
  p := point
  q := point
  tl := line(p, q)
  p distance(c, along: right) q
  p distance(v + s, along: up) q
}

component Closed(a: point, c: Length, s: Length) {
  p := point(x: a.x + 50mm + c * ln((s - 75mm) / c + sqrt(((s - 75mm) / c)^2 + 1)), y: a.y + sqrt(c^2 + (s - 75mm)^2) - sqrt(c^2 + 75mm^2))
}

in std.front {
  a := point
  b := point
  fix((0mm, 0mm)) a
  fix((100mm, 0mm)) b

  // the home: the drawn instance, and what holds there alone
  home := Pull(c: hint(30mm), v: hint(-75mm), s: 0mm)
  home.p coincident a

  // the place s1, on the closed form
  place := Pull(c: c, v: -75mm, s: s1)
  place.p coincident(t == s1) cf
  k := point
  tc := line(place.p, k)
  length(10mm) tc
  cf tangent(at: place.p) tc
  claim tc parallel place.tl                    // the rate rows: p runs along the tension
  end := point
  end coincident(t == 150mm) cf
  claim end coincident b                         // the flow from a reaches b

  // today's free curve, through the closed form
  rope := curve(a, b)
  length(150mm) rope
  n := point
  n coincident rope
  n level(right) place.p
  claim n coincident place.p
  low := point
  low coincident(t == 75mm) cf
  m := point
  m coincident rope
  m level(right) low
  claim m coincident low
}
rope minimizes integral(p.y over p)
cf := Closed(a, c: c).p over s in (0mm, 150mm)
