// The same gear, with the involute *described* instead of derived.
//
// `gear.sv` writes the flank as a formula that somebody worked out on paper first.  This file
// states what the string physically does and leaves the working to the solver: `Unwind` is a
// component whose statements place `p`, and `Unwind(…).p over u` says the curve is wherever
// they put it as the string unwinds.
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

component Unwind(c: circle, datum: line, phase: Angle, u: Angle) {
  t := point
  p := point
  rad := line(c.center, t)
  s := line(t, p)
  t coincident c                                       // the string leaves the circle...
  datum angle(u + phase) rad                   // ...at bearing u from the datum,
  rad perpendicular s                          // perpendicular to the radius there,
  p distance(-(c.r * u / 1rad)) rad            // and taut: let out == arc unwound
}

// From here down the wheel is `gear.sv` unchanged — a flank between two circles, a tooth as two
// flanks, the tooth round a cycle — which is the point: what a curve *is* and what is drawn with
// it are separate statements, and swapping the component the flank is a point of touched
// neither.

component Flank(base: circle, datum: line, root: circle, tip: circle,
                phase: Angle, u0: Angle, u1: Angle) {
  e := Unwind(base, datum, phase: phase).p over u in (u0, u1)
  // Seeded at the centre, as gear.sv's are and for the same reason: from there the first step
  // puts each end on the flank at the roll its contact's `hint(t: …)` names.  Started a unit
  // or so off-centre, the solve reached the *mirror* branch of the string — the same radii,
  // the wrong bearings, a tooth flaring the wrong way — and `over (u0, u1)` is what now refuses
  // that: a contact off the drawn interval is put back and held, and the drawing either solves
  // on the flank or says it did not.
  lo := point hint(x: 0, y: 0)
  hi := point hint(x: 0, y: 0)

  lo coincident e hint(t: u0)
  hi coincident e hint(t: u1)
  lo coincident root
  hi coincident tip
}

component Tooth(base: circle, datum: line, root: circle, tip: circle,
                a0: Angle, half: Angle, u0: Angle, u1: Angle) {
  r := Flank(base, datum, root, tip, phase: a0 - half, u0: u0, u1: u1)
  l := Flank(base, datum, root, tip, phase: a0 + half, u0: -u0, u1: -u1)

  crown := line(r.hi, l.hi)
}

component Gear(N: Int, m: Length, phi: Angle, ded: Scalar) {
  R := m * N / 2
  Rt := R + m
  Rb := R * cos(phi)
  // the root stays clear of the base circle — twelve teeth is stub-tooth territory, and below
  // `Rb` there is no involute to trace.  gear.sv says why at length; the reasons transfer.
  clear := 0.02
  Rr := max(R - ded * m, Rb * (1 + clear))

  pitch := tau / N
  ivp := tan(phi) * 1rad - phi
  half := 90deg / N + ivp
  u0 := sqrt((Rr / Rb) ^ 2 - 1) * 1rad
  u1 := sqrt((Rt / Rb) ^ 2 - 1) * 1rad

  center := point
  anchor := point
  datum := line(center, anchor)
  base := circle(center: center) hint(r: Rb)
  root := circle(center: center) hint(r: Rr)
  tip := circle(center: center) hint(r: Rt)

  radius(Rb) base
  radius(Rr) root
  radius(Rt) tip
  fix(x == 0, y == 0) center
  fix(x == R, y == 0) anchor

  cycle N as i {
    t := Tooth(base, datum, root, tip, a0: i * pitch, half: half, u0: u0, u1: u1)
    gap := line(t.l.lo, next.t.r.lo)
  }
}

g := Gear(N: 12, m: 3, phi: 25, ded: 1)

// Diagnosed: fully constrained.  The two rolls per flank are still the solver's answers — and so
// now is every point of every flank in between.
