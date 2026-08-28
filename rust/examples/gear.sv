// A spur gear whose tooth flanks are true involutes — the curve a taut string traces.
//
// A gear tooth is not a trapezium, and it is not a polyline pretending to curve.  Its flank is
// the **involute** of a circle: the path the end of a taut string sweeps as it unwinds from
// that circle.  That shape is the whole reason gears run smoothly — two involutes stay in
// contact along a fixed straight line as they roll past each other, so the speed ratio between
// the wheels never wavers, and the teeth do not knock.
//
// The involute is not built into this solver.  It is the three lines below, written in the same
// small language a dimension is written in: a curve *family*, defined over the circle it unwinds
// from.  `u` is how far the string has unwound, in degrees, so the length let out is `r · u/1rad`
// — and that string, square to the radius at the point it leaves the circle, is the definition.
//
// Everything after it is ordinary.  Thirty teeth are one tooth written once and repeated, and a
// point touching a curve is a single statement whichever family the curve belongs to.

curve involute(c: circle, phase: Angle)(u) =
  ( c.center.x + c.r * (cos(u + phase) + u / 1rad * sin(u + phase)),
    c.center.y + c.r * (sin(u + phase) - u / 1rad * cos(u + phase)) )

// One flank: the piece of an involute between the root circle and the tip.
//
// Nothing here says where the flank goes.  It says the curve is the involute of the base circle
// at this bearing, that it begins where it crosses the root circle and ends where it crosses the
// tip — and **the solver finds the two rolls that satisfy that**.  There is no closed form for
// either; that is a real solve, and it is the whole of the tooth's shape.
//
// `u = …` is a *seed*, not a statement: it is where the search starts, and deleting both still
// solves, from a worse start.
component Flank(base: circle, root: circle, tip: circle,
                phase: Angle, u0: Angle, u1: Angle) {
  curve e = involute(base, phase: phase) over (u0, u1)
  port lo: point
  port hi: point

  point_on_curve(lo, e) hint(u: u0)
  point_on_curve(hi, e) hint(u: u1)
  point_on_circle(lo, root)
  point_on_circle(hi, tip)
}

component Tooth(base: circle, root: circle, tip: circle,
                a0: Angle, half: Angle, u0: Angle, u1: Angle) {
  // the two flanks of one tooth: one involute unwinding one way from the tooth's leading edge,
  // the other the opposite way from its trailing one, which is a negative roll
  r: Flank(base, root, tip, phase: a0 - half, u0: u0, u1: u1)
  l: Flank(base, root, tip, phase: a0 + half, u0: -u0, u1: -u1)

  line crown(r.hi, l.hi)
}

component Gear(N: Int, m: Length, phi: Angle, ded: Scalar) {
  param R = m * N / 2            // the pitch circle: where a tooth is half the pitch thick
  param Rt = R + m               // the tip
  param Rb = R * cos(phi)        // the base circle the involute unwinds from

  // **Below the base circle there is no involute.**  A tooth's flank is only a flank down to
  // `Rb`; under that a real gear runs a fillet, which is a different curve for a different
  // reason.  This drawing has no fillet, so it does the other thing available to it: the root
  // never goes inside the base circle.  Asked for a deeper tooth than the involute reaches, the
  // gear gives a *shallower* one — the stub tooth a low count really does need — rather than a
  // curve that does not exist.
  //
  // It stops a little *clear* of the base circle rather than exactly on it, and a real gear does
  // the same — the fillet meets the flank at the form diameter, which is above `Rb`, never at it.
  // Here the reason is sharper than convention.  At the base circle the involute has a **cusp**:
  // `C'(u) = Rb u (cos u, sin u)` vanishes at `u = 0`, so a contact there has no direction to
  // slide along.  Worse than the point itself is its neighbourhood — `Param::scale` is the curve's
  // *mean* speed, and a flank that starts at the cusp runs an order of magnitude slower at one end
  // than the other, so the t column is wrong by that factor wherever the contact actually sits and
  // the solver crawls.  Two percent of `Rb` puts the flank's first roll near 11°, which keeps the
  // speed along it within a small factor and the solve at nine iterations.
  //
  // Where the dedendum fits — every count from about 22 up, at these proportions — `Rr` is the
  // textbook root and none of this is doing anything at all.
  param clear = 0.02
  param Rr = max(R - ded * m, Rb * (1 + clear))

  param pitch = tau / N
  // half a tooth's angular thickness, measured from the base circle.  `inv(u) = u - atan(u)`,
  // and the roll at the pitch circle is `tan(phi)` — the two facts an involute gear needs.
  param ivp = tan(phi) * 1rad - phi
  param half = 90deg / N + ivp
  // the rolls that reach the root and the tip: r(u) = Rb sqrt(1 + u²), so u = sqrt((r/Rb)² - 1),
  // in degrees because that is what the curve runs on
  param u0 = sqrt((Rr / Rb) ^ 2 - 1) * 1rad
  param u1 = sqrt((Rt / Rb) ^ 2 - 1) * 1rad

  point center hint(x: 0, y: 0)
  circle base(center: center) hint(r: Rb) class construction
  circle root(center: center) hint(r: Rr) class construction
  circle tip(center: center) hint(r: Rt) class construction

  // `r:` above only *seeds* a radius — a seed is where a solve starts, not something it must
  // honour — so without these the three circles would breathe.
  radius(base) == Rb
  radius(root) == Rr
  radius(tip) == Rt
  ground(center)

  // `cycle` and not `ring`: the teeth are congruent because each is given the same numbers, not
  // because the wheel is *claimed* to be symmetric.  Spec §12.3 makes the two equivalent when the
  // symmetry is stated as constraints; stating it is what `ring` would add.
  cycle N as i {
    t: Tooth(base, root, tip, a0: i * pitch, half: half, u0: u0, u1: u1)
    // the gap to the next tooth, drawn across the root circle
    line gap(t.l.lo, next.t.r.lo)
  }
}

g: Gear(N: 30, m: 3, phi: 25, ded: 1)

// Diagnosed: fully constrained.  Every number here is a length, a bearing or a tooth count; the
// two rolls per flank are the solver's answers, and there is no closed form for either.
