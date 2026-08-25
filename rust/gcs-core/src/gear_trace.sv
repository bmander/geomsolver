// The spur gear again — but this time the involute is *traced*, not computed.
//
// `gear.sv` writes the flank's curve as a formula: two expressions somebody derived on paper,
// the string unwound in closed form.  This document writes what the string *does* and leaves the
// deriving to the solver.  `trace p where { … }` (spec §6.5.1) says the curve is wherever its
// constraints put `p` as the roll runs — and the constraints below are Wikipedia's definition of
// an involute, verbatim: a point `t` on the base circle at bearing `u`; the string leaving
// perpendicular to the radius there; and taut, exactly as long as the arc it has unwound.  No
// closed form appears in any of them.
//
// Note what is *not* here: the string end has no seed at all.  Given `t`, the two statements
// about `p` — on the perpendicular through the tangent point, a signed arc's-worth from the
// radius — are linear in `p` and meet in exactly one place, so there is no branch to pick and
// nothing for a seed to say.  The sign is the winding: `point_line_distance` is positive to the
// left of the line, so one signed equation unwinds the string one way for positive roll and the
// other way for negative — which is how one family below serves both flanks of a tooth, where
// the closed form needs the sign threaded through every term.
//
// The tangent point keeps one seed, because `angle` is a statement mod 180°: `t` could sit at
// the bearing or opposite it, and a seed is how a branch is picked (spec §6.5.1).  But a seed is
// a *place*, and this language names places geometrically: `at c bearing (u + phase)` is the
// point at the edge of circle `c`, said the way a draughtsman says it.  It is a seed, not a
// claim: perturb the bearing and the constraints put `t` back exactly.
//
// The datum line is the bearing's zero.  It is called `datum` in every component that hands it
// down, exactly as `base`, `root` and `tip` keep their names — a formal renamed between levels
// does not resolve (issue #4).

curve involute(c: circle, datum: line, phase: Angle)(u) =
  trace p where {
    point t at c bearing (u + phase)
    point p
    line rad(c.center, t)
    line s(t, p)
    point_on_circle(t, c)                        // the string leaves the circle...
    angle(datum, rad) == u + phase               // ...at bearing u from the datum,
    perpendicular(rad, s)                        // perpendicular to the radius there,
    point_line_distance(p, rad) == -(c.r * u * pi / 180)   // and taut: let out == arc unwound
  }

// From here down the wheel is `gear.sv` unchanged — a flank between two circles, a tooth as two
// flanks, the tooth round a cycle — which is the point: what a curve *is* and what is drawn with
// it are separate statements, and swapping the family's body touched neither.

component Flank(base: circle, datum: line, root: circle, tip: circle,
                phase: Angle, u0: Angle, u1: Angle) {
  curve e = involute(base, datum, phase: phase) over (u0, u1)
  port lo: point
  port hi: point

  point_on_curve(lo, e, u = u0)
  point_on_curve(hi, e, u = u1)
  point_on_circle(lo, root)
  point_on_circle(hi, tip)
}

component Tooth(base: circle, datum: line, root: circle, tip: circle,
                a0: Angle, half: Angle, u0: Angle, u1: Angle) {
  r: Flank(base, datum, root, tip, phase: a0 - half, u0: u0, u1: u1)
  l: Flank(base, datum, root, tip, phase: a0 + half, u0: -u0, u1: -u1)

  line crown(r.hi, l.hi)
}

component Gear(N: Int, m: Length, phi: Angle, ded: Scalar) {
  param R = m * N / 2
  param Rt = R + m
  param Rb = R * cos(phi)
  // the root stays clear of the base circle — twelve teeth is stub-tooth territory, and below
  // `Rb` there is no involute to trace.  gear.sv says why at length; the reasons transfer.
  param clear = 0.02
  param Rr = max(R - ded * m, Rb * (1 + clear))

  param pitch = tau / N
  param ivp = tan(phi) * 180 / pi - phi
  param half = 90 / N + ivp
  param u0 = sqrt((Rr / Rb) ^ 2 - 1) * 180 / pi
  param u1 = sqrt((Rt / Rb) ^ 2 - 1) * 180 / pi

  point center at (0, 0)
  point anchor at (R, 0)
  line  datum(center, anchor) construction
  circle base(center: center, r: Rb) construction
  circle root(center: center, r: Rr) construction
  circle tip(center: center, r: Rt) construction

  radius(base) == Rb
  radius(root) == Rr
  radius(tip) == Rt
  ground(center)
  ground(anchor)

  cycle N as i {
    t: Tooth(base, datum, root, tip, a0: i * pitch, half: half, u0: u0, u1: u1)
    line gap(t.l.lo, next.t.r.lo)
  }
}

g: Gear(N: 12, m: 3, phi: 25, ded: 1)

// Diagnosed: fully constrained.  The two rolls per flank are still the solver's answers — and so
// now is every point of every flank in between.
