// The same gear, with the involute *described* instead of derived.
//
// `gear.sv` writes the flank as a formula that somebody worked out on paper first.  This file
// states what the string physically does and leaves the working to the solver.  `trace p where
// { … }` says: the curve is wherever these statements put `p`, as the string unwinds.
//
// And the statements are the textbook definition, said once and not rearranged — a point `t` on
// the base circle at angle `u`; the string leaving square to the radius there; the string taut,
// exactly as long as the arc it has unwound so far.  No formula for an involute appears
// anywhere.
//
// Notice what is *absent*: the far end of the string has no starting guess at all.  Given `t`,
// the two statements about `p` meet in exactly one place, so there is nothing left to guess at.
// The sign does the rest — a distance measured from a line is positive on one side of it and
// negative on the other, so one statement unwinds the string one way for a positive roll and the
// other way for a negative one.  That is how a single definition serves both flanks of a tooth,
// where a formula would need the sign threaded through every term by hand.
//
// No ambiguity remains.  An `angle` is directed — measured counter-clockwise from the datum's
// own direction, on the full turn — so which side of the datum `t` falls is in the residual
// itself, and it stays true however far the flank winds round.  Stated mod a half turn it
// would need a `ccw` predicate to name the side, read once at a roll where the answer is
// unmistakable and carried round by continuity.
//
// The datum line is where the angle is measured from, and it keeps the name `datum` in every
// component that hands it down.

curve involute(c: circle, datum: line, phase: Angle)(u) =
  trace p from (90 - phase) where {
    point t
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

  point center hint at (0, 0)
  point anchor hint at (R, 0)
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
