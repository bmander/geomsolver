// A groin vault: the solid common to two barrel-vaulted blocks on the square |x|, |y| ≤ 12, the
// second the first turned a quarter turn about the upright, so that the two vaults cross at right
// angles.  Each vault is a rational quadratic arc carried straight along its block: its control
// points (−12, 10), (0, 26) and (12, 10), the middle one weighing 3/5, which makes it exactly the
// arc of radius 15 about a point 1 above the floor, rising from the springing at 10 to its crown
// at 16.  The arcs are drawn as splines, not circles: the vaults are surfaces swept from a
// weighted spline, and the two meet in curves traced, not in closed form.  A spline here is a
// cubic, so the quadratic is raised to one exactly: its inner points two thirds of the way from
// each end towards the apex, weighted, (P₀ + 2w·P₁) / (1 + 2w), each weighing (1 + 2w) / 3.
//
// Over every point of the square the common keeps the lower vault.  The two meet along the
// square's diagonals in two arches, the groins, which cross at the crown, where the two vaults are
// tangent: four half-groins end there at one vertex.  The common's sides are four rectangles
// 24 × 10, each block's flat side lying in the plane of the other's arched one.  Its volume is
// 24²·10 + 8∫₀¹² x(√(225 − x²) − 9) dx = 7632, its area 576 + 960 + 8·90 = 2256.
unit mm
use std

half := 12mm     // the square is |x|, |y| ≤ half
spring := 10mm   // where each vault rises from its flat sides
apex := 26mm     // the quadratic's middle control point, where its end tangents meet
w := 3 / 5       // that point's weight: the arc a circle's
// the cubic's inner control points and their weight
reach := half / (1 + 2 * w)
rise := (spring + 2 * w * apex) / (1 + 2 * w)
heavy := (1 + 2 * w) / 3

// the first block: its section across y, swept along x
in std.side {
  a := point
  b := point
  c := point
  d := point
  d2 := point
  e := point
  fix((-half, 0mm)) a
  fix((half, 0mm)) b
  fix((half, spring)) c
  fix((reach, rise)) d
  fix((-reach, rise)) d2
  fix((-half, spring)) e
  floor := line(a, b)
  east := line(b, c)
  arch := spline(c, d, d2, e) weights [1, heavy, heavy, 1]
  west := line(e, a)
  along_x := solid(face(floor, east, arch, west), from: -half, to: half)
}

// the second: the same section across x, swept along y
in std.front {
  f := point
  g := point
  h := point
  i := point
  i2 := point
  j := point
  fix((-half, 0mm)) f
  fix((half, 0mm)) g
  fix((half, spring)) h
  fix((reach, rise)) i
  fix((-reach, rise)) i2
  fix((-half, spring)) j
  sole := line(f, g)
  north := line(g, h)
  crossing := spline(h, i, i2, j) weights [1, heavy, heavy, 1]
  south := line(j, f)
  along_y := solid(face(sole, north, crossing, south), from: -half, to: half)
}

vault := solid(along_x)
along_y common vault
