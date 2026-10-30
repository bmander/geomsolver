// A turned can, sealed: a rectangle turned about its own edge, a cylinder's band and two caps.
// A turned body is no prism, so it is built by its walls: the band moved in by `wall`, the caps
// moved in, mitred where they meet.
//
// Built exactly: 12000π less the hollow's 18² π × 26, 3576π mm³.
unit mm
use std
use std.rounds (shell)

wall := 2mm

in std.front {
  a := point
  b := point
  c := point
  d := point
  fix((0, 0)) a
  fix((20, 0)) b
  fix((20, 30)) c
  fix((0, 30)) d
  (bottom := line(a, b)) -> (side := line(b, c)) -> (top := line(c, d)) -> (axle := line(d, a)) -> close
  half := face(bottom, side, top, axle)
}
can := solid(half, about: axle)

shell(t: wall) can
