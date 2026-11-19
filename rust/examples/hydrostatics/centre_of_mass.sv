// A centre of mass does not depend on how bodies are grouped (#953): a steel block and an
// aluminium one beside it, 40 and 60 long, 20 wide and 10 high, drawn twice — as two components of
// one instance, `pair`, and as one component of both bodies, `both`.  `g centerofmass pair` and
// `h centerofmass both` each read every body under the instance, each weighed by its own density,
// and put their points at x = Σ ρᵢ Vᵢ xᵢ / Σ ρᵢ Vᵢ = 37.02mm alike; `c centroid pair` weighs by
// volume alone, at the middle, 50mm.  Edit a material and both centres move together.
unit mm
use std
use std.solids

steel := material(E: 200GPa, nu: 0.3, rho: 7850kg / 1m^3)
alu := material(E: 70GPa, nu: 0.33, rho: 2700kg / 1m^3)

component Block(m: group, x0: Length, x1: Length) {
  in std.top {
    a := point
    b := point
    c := point
    d := point
    fix((x0, 0mm)) a
    fix((x1, 0mm)) b
    fix((x1, 20mm)) c
    fix((x0, 20mm)) d
    s1 := line(a, b)
    s2 := line(b, c)
    s3 := line(c, d)
    s4 := line(d, a)
    sec := face(s1, s2, s3, s4)
  }
  body := solid(std.solids.Prism(sec, from: 0mm, to: 10mm)) of m
}

// two components of one instance
component Pair(ms: group, ma: group) {
  s := Block(m: ms, x0: 0mm, x1: 40mm)
  a := Block(m: ma, x0: 40mm, x1: 100mm)
}

// one component of both bodies
component Both(ms: group, ma: group) {
  in std.top {
    a := point
    b := point
    c := point
    d := point
    e := point
    f := point
    fix((0mm, 0mm)) a
    fix((40mm, 0mm)) b
    fix((40mm, 20mm)) c
    fix((0mm, 20mm)) d
    fix((100mm, 0mm)) e
    fix((100mm, 20mm)) f
    s1 := line(a, b)
    s2 := line(b, c)
    s3 := line(c, d)
    s4 := line(d, a)
    t1 := line(b, e)
    t2 := line(e, f)
    t3 := line(f, c)
    t4 := line(c, b)
    left := face(s1, s2, s3, s4)
    right := face(t1, t2, t3, t4)
  }
  steel_body := solid(std.solids.Prism(left, from: 0mm, to: 10mm)) of ms
  alu_body := solid(std.solids.Prism(right, from: 0mm, to: 10mm)) of ma
}

pair := Pair(ms: steel, ma: alu)
both := Both(ms: steel, ma: alu)

g := point hint((40, 10, 5))
g centerofmass pair
h := point hint((40, 10, 5))
h centerofmass both
c := point hint((50, 10, 5))
c centroid pair
