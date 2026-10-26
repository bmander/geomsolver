// A disc cam with a flat-faced (mushroom) follower: the tool is a line, the follower's face,
// sliding by a law, and the cam is what the face leaves as the cam turns under it.
//
// A flat face cannot reach into a hollow, so the cam it rides must be convex everywhere, and
// whether it is depends only on the law.  Written as the cam's support function, the face's
// distance from the cam's centre at the turn θ, `p(θ) = base + s(θ)`, the cam the face cuts is
// the envelope of the lines `p(θ)` out along the follower turned back by θ: it touches the face
// at `(p, -p')` in the follower's frame (`p'` a radian), off the follower's line by the speed of
// the lift, and it bends with radius `p + p'' = base + s + s''`.  Where that is positive the cam
// is convex; where it is not, no flat face can cut it.
//
// The follower rises `h` over the first half turn and returns over the second, each on the
// cycloidal law `h (x - sin(360deg x) / 2π)`, `x` the fraction of the half turn, written as one
// expression joined with `min` and `max`.  Its slide carries it `h` out in a whole turn of its
// value, so `360deg` is the whole lift.  Each half of the cam is its own envelope.
//
// A circle osculates the rise three quarters through it, where the follower slows hardest: the
// solve works its radius out (base + s + s'' = 29.09).  Edit `base`, `h` or the law, and the cam
// is cut again.

use std
use std.place (right_of)
use std.motion (spins, slides)

base := 25    // the face's nearest approach to the cam's centre
h := 15       // the follower's lift

param t: Angle over (0deg, 360deg)          // the cam's turn

in std.front {
  o := point
  fix((0, 0)) o
  s0 := point
  s1 := point
  fix((0, 0)) s0
  fix((10, 0)) s1
  path := line(s0, s1)                    // the follower's line, through the cam's centre
}

cam := frame
cam spins(1) o
follower := frame
follower slides(h) path                 // the value `360deg` is the whole lift
// rise over the first half turn, return over the second, each cycloidal
fix(value == 360deg * (min(t / 180deg, 1) - sin(360deg * min(t / 180deg, 1)) / (2 * pi)
                       - max((t - 180deg) / 180deg, 0)
                       + sin(360deg * max((t - 180deg) / 180deg, 0)) / (2 * pi))) follower

// the follower's face, square to its line, where the turn starts
in std.front {
  f0 := point
  f1 := point
  fix((base, -20)) f0
  fix((base, 20)) f1
  flat := line(f0, f1)
}

rise := envelope(flat over t in (0deg, 180deg), under: follower, seen_from: cam)
return := envelope(flat over t in (180deg, 360deg), under: follower, seen_from: cam)

// how tightly the cam bends three quarters through the rise
in std.front {
  k := point hint((0, 0))
  osc := circle(center: k) hint(r: 29)
  rise curvature(t == 135) osc
}
