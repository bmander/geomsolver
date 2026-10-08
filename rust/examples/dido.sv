// Dido's problem: given a hide cut into a strip of fixed length, enclose the most land you can
// against a straight shore. The shore is the chord from `a` to `b`; the strip is a free curve
// `130mm` long between them; the land is the area the two enclose.
//
// The area is a line integral round the boundary, `(x dy - y dx) / 2`, and along the shore
// (`y = 0`) it adds nothing, so the whole of it is an integral along the strip: of the point `p`
// running along it and its tangent `t` there, weighted by length. `maximize` says the strip takes
// the shape that makes it greatest, and the report says the shape found is a maximum.
//
// What it comes to is the arc of a circle through the two ends: nothing here says so.
unit mm
use std

in std.front {
  a := point
  b := point
  fix((0mm, 0mm)) a
  fix((100mm, 0mm)) b
  shore := line(a, b)
  strip := spline(a, b)
  length(130mm) strip
}
maximize integral((p.x * t.y - p.y * t.x) / 2 over (p, t) in strip)
