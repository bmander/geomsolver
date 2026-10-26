// `std.motion`: a frame's joint said as words — `turns`, `screws`, `spins`, `slides`, `rolls` —
// and its twist read by rates.

// -- a frame's joint: turns, screws and slides (#383, #464) --------------------------------------
//
// A frame's motion is its joint: an axis `f.axis`, a ratio `f.ratio` (radians a radian of the
// shared roll) and an advance `f.advance` (along the axis a turn of the roll), held by relations
// like anything else.  These are the words a designer states them in, the frame the subject and
// what it moves about or along the object: `counter spins(-1) centre`.  Roll 0 is as drawn: a
// member that must stand turned there is drawn turned.  The joint's axis runs the way what it is
// held to does (#535) — a line from its start toward its end, an axis its own way, a view's
// normal out of it — so which way a turn goes and a slide runs is the drawing's, never a seed's.

// `f turns(ratio) l`: `f` turns about the line `l`, `ratio` radians a radian of the roll, and
// moves not at all along it.
f turns(ratio) l := {
  l coincident(sense: same) f.axis
  fix(ratio == ratio, advance == 0mm) f
}

// `f screws(advance) l`: `f` screws about the line `l`, a radian a radian of the roll, and
// `advance` along it a turn — right-handed for a positive advance, left for a negative.
f screws(advance) l := {
  l coincident(sense: same) f.axis
  fix(ratio == 1, advance == advance) f
}

// `f spins(ratio) o`: `f` turns about the point `o` in the view it is drawn in, `ratio` radians a
// radian of the roll — about the axis through `o` square to that view.
f spins(ratio) o := {
  o coincident f.axis
  f.axis parallel(sense: same) o.plane.normal
  fix(ratio == ratio, advance == 0mm) f
}

// `f slides(advance) l`: `f` slides along the line `l`, `advance` a turn of the roll, turning not
// at all.
f slides(advance) l := {
  l coincident(sense: same) f.axis
  fix(ratio == 0, advance == advance) f
}

// `a rolls(at: p, along: d) b`: frames `a` and `b` roll on one another at `p` without slipping
// along the direction `d` (#383): the point of `a`'s frame there, seen from `b`'s, does not move
// along it — the rate of the point `a` carries, seen from `b`, at roll 0, nothing along it.  A
// joint's ratio left out is found so: two gears' pitch circles rolling at their pitch point turn
// at the ratio of their radii, and a ratio stated places the pitch point.  For two shafts
// parallel, or meeting, and `p` on the line of centres the rate has no other part, so one
// direction, the circles' common tangent there, says it all.  A fact about the two frames, so a
// word and not a definition: it makes nothing a drawing reads.
a rolls(at, along) b := {
  moved := point
  moved coincident(under: a, seen_from: b, t == 0deg) at
  rate(moved over t) distance(0mm, along: along)
}

// -- a motion's twist, read by rates (#326, #365) -------------------------------------------------
//
// A turn about a fixed line, a screw along one or a slide moves every point at a velocity, and so
// does one seen from another (`under: gear, seen_from: pinion`): its **twist**.  It has no words
// of its own: it is the rate of a point the frame carries.  `X coincident(under: m, t == 0deg) x`
// is `x` carried by the frame's pose at its roll 0, where it stands as drawn, and `rate(X over
// t)` is how fast `X` runs there, a length a radian of the roll — the twist at `x`.

// `l reciprocal(seen_from: g) m`: a force along the line `l` does no work on frame `m`'s motion
// seen from frame `g` — its velocity at a point of `l` is square to `l`, the same number at every
// point of it.  For the mesh of two members (`m` the gear, `g` the pinion) it is the meshing law
// of a line of action: the members move as fast along it (Phillips' spatial involute gearing).
// Seen from the world, `g` is a frame at rest.  Claimed (`claim l reciprocal(seen_from: g) m`),
// the point is carried all the same and only its rate is judged.
l reciprocal(seen_from) m := {
  moved := point
  moved coincident(under: m, seen_from: seen_from, t == 0deg) l.p1
  rate(moved over t) perpendicular l
}

// The instantaneous screw axis of a frame's motion, seen from `seen_from` where it is given (a
// frame left out is at rest: the world): `isa`, the axis every point of which `m` moves along it,
// and `pitch`, how fast — a length a radian of the roll, Ball's pitch times how fast the motion
// turns about the axis.  `s := std.motion.Screw(toward, gear, seen_from: pinion)` with no pitch
// given solves it, and `s.isa` and `s.pitch` read it.  Two points of the axis carried: their rates
// along it place it, and the first's speed along it is the pitch.  Which way the axis runs, and so
// the pitch's sign, is said by `toward`, a direction the caller holds: the axis runs its way (a
// bound, which adds no row).  A motion that turns nothing (a slide) has no such axis.
component Screw(toward: axis, m: frame, seen_from: frame, pitch: Length) {
  isa := axis
  private origin := point
  fix((0, 0, 0)) origin
  private a := point
  private b := point
  a coincident isa
  b coincident isa
  origin level(isa) a
  a distance(10mm, along: isa) b
  a distance(>= 0mm, along: toward) b
  private moved_a := point
  private moved_b := point
  moved_a coincident(under: m, seen_from: seen_from, t == 0deg) a
  moved_b coincident(under: m, seen_from: seen_from, t == 0deg) b
  rate(moved_a over t) parallel isa
  rate(moved_b over t) parallel isa
  rate(moved_a over t) distance(pitch, along: isa)
}
