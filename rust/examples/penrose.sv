// Penrose's P3 tiling by Robinson triangles, drawn by two components each calling the other.
//
// A rhomb of the tiling is two half rhombs.  An acute half (legs `l`, apex 36°) cuts into an
// acute and an obtuse one; an obtuse half (legs `s`, apex 108°) into two obtuse and an acute, each
// a golden ratio smaller.  Each component makes the new points by distances alone and calls the
// two or three triangles they cut it into, one level down; a `when` over the count ends it, and
// a leaf draws its two legs, apex first, so the drawn lines are the rhombs' sides.
//
// The sun is ten acute halves about `o`, its corners held on a circle of radius `l`.  Three
// levels make 130 half rhombs (10 → 20 → 50 → 130).  Edit `depth` (each level is about 2.6 times
// as many) or `l`; the seeds read the corners, so every point starts on its own branch.

use std
param depth := 3
param l := 100

component Acute(a: point, b: point, c: point, l: Length, n: Int) {
  when n == 0 {
    line(a, b)
    line(a, c)
  }
  when n > 0 {
    phi := (1 + sqrt(5)) / 2
    p := point hint(at: a, toward: b, by: 1 / phi)
    a distance(l / phi) p
    c distance(l / phi) p
    x := Acute(c, p, b, l: l / phi, n: n - 1)
    y := Obtuse(p, c, a, s: l / phi, n: n - 1)
  }
}

component Obtuse(a: point, b: point, c: point, s: Length, n: Int) {
  when n == 0 {
    line(a, b)
    line(a, c)
  }
  when n > 0 {
    phi := (1 + sqrt(5)) / 2
    r := point hint(at: b, toward: c, by: 1 / phi)
    q := point hint(at: b, toward: a, by: 1 / phi)
    b distance(s) r
    a distance(s / phi) r
    r distance(s / phi) q
    b distance(s / phi) q
    x := Obtuse(r, c, a, s: s / phi, n: n - 1)
    y := Obtuse(q, r, b, s: s / phi, n: n - 1)
    z := Acute(r, q, a, l: s / phi, n: n - 1)
  }
}

in std.front {
  o := point hint((0, 0))
  fix((0, 0)) o
  repeat 10 as k {
    v := point
    fix((l * cos(36deg * k), l * sin(36deg * k))) v
  }
  repeat 5 as k {
    t := Acute(o, v[2 * k + 1], v[2 * k], l: l, n: depth)
  }
  repeat 4 as k {
    m := Acute(o, v[2 * k + 1], v[2 * k + 2], l: l, n: depth)
  }
  last := Acute(o, v[9], v[0], l: l, n: depth)
}
