// A ball thrown from `a` lands at `b`: its path stated by Jacobi's form of least action, not by a
// parabola. A particle of fixed energy moving under gravity takes the path that makes its speed
// integrated along it stationary. With `h` the height a ball thrown straight up from the ground
// would reach, its speed at height `y` goes as `sqrt(h - y)`, so the path is the curve that makes
// `sqrt(h - p.y)` integrated along it stationary — a parabola, though nothing here says so.
//
// Two throws reach `b` at that speed: the low line drive found here, a minimum, and a high lob,
// a saddle — past the point where the throws from `a` cross. Move `b` beyond `2 h` and no throw
// reaches it.
unit mm
use std

h := 50mm

in std.front {
  a := point
  b := point hint((80, 0))
  fix((0mm, 0mm)) a
  fix((80mm, 0mm)) b
  throw := curve(a, b)
}
throw minimizes integral(sqrt(h - p.y) over p)
