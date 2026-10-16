// D0 (#339): the clothoid, in the proposed spelling. Does not parse: it stops at `rate`.
//
// A curve whose curvature is linear in its arc length, κ = s/A². The body carries the curve's
// point `p` with its tangent `tl` and left normal `nl`, each 10 long and square. Its algebraic
// rows leave three freedoms; three rate rows take them:
//   - `p` runs along `tl` at unit speed (so `s` is arc length): two rows;
//   - `tl` turns towards `nl` at κ: the rate of its far end along `nl` is 10·κ, one row.
// Every rate row is linear in the rates: an incidence or a component along a drawn direction.
// The speed is said as a component along `tl`, never as a magnitude (`length(1) rate(p over s)`
// is quadratic in the rate, so it would leave a root to choose at every step).
//
// The law integrates in closed form, so the same curve is also `datum angle(s^2 / (2 * A^2) *
// 1rad) tl` with only the first two rate rows: a quadrature, as L5's laws are expressions in `s`.
// The curvature form is the one that needs rates.
//
// Closed form: x(s) = ∫₀ˢ cos(σ²/2A²) dσ, y(s) = ∫₀ˢ sin(σ²/2A²) dσ (the Fresnel integrals).
unit mm
use std

component Clothoid(A: Length, s: Length) {
  p := point
  q := point
  k := point
  tl := line(p, q)
  nl := line(p, k)
  length(10mm) tl
  length(10mm) nl
  tl angle(90deg) nl
  rate(p over s) parallel tl
  rate(p over s) distance(1, along: tl)                 // unit speed: s is arc length
  rate(q over s) distance(10mm * s / A^2, along: nl)    // the tangent turns at κ = s/A²
}

in std.front {
  o := point
  x := point
  fix((0mm, 0mm)) o
  fix((10mm, 0mm)) x
  datum := line(o, x)
  cl := Clothoid(A: 50mm, s: 0mm)
  cl.p coincident o
  datum angle(0deg) cl.tl
}
spiral := trace(cl.p over s in (-100mm, 100mm))
