// `std.stations`: where a section stands along a sweep or a loft — a plane square to its path,
// a frame along it, a station between rails or faces — and the laws a station runs.

use std.place (on_u, square)
use std.motion (screws, spins, turns)

// -- stations of a face family (#243 L3) --------------------------------------------------------
//
// A loft or a sweep is a face of a component as one of its numbers runs over a declared parameter
// (§6.9, #398): `param s: Scalar over (0, 1)` and `sweep(I(…).f over s)`.  These words only say
// where a station stands on its path: a plane `n` square to the path where its parameter is `s`,
// its `v` along `up`.  What the section is at that station — its shape, its twist, its scale, the
// rails it follows — is the writer's component, drawn `in st.n` and reading `s` (the laws below
// run a twist or a scale):
//
//   component Reducer(g: line, inlet: Length, outlet: Length, s: Scalar) {
//     st := std.stations.OnLine(g, up: std.z, s: s)
//     sec := Section(st.n.origin, size: inlet + (outlet - inlet) * s) in st.n
//   }
//   param s: Scalar over (0, 1)
//   body := sweep(Reducer(guide, inlet: 24mm, outlet: 12mm).sec.profile over s)
//
// The edge `ab` at one station is `ab` at every other, so two sections correspond by being one.

// The plane square to the line `g` at the fraction `s` of the way from its start to its end: its
// origin there, its normal along `g`, and its `v` held along `up` (any direction square to `g`).
// Along a straight guide every station is the first one moved, so nothing turns.  It starts as it
// ends, read off the guide and `up`: `v` along `up`, and `u` that turned back a quarter about `g`.
component OnLine(g: line, up: axis, s: Scalar) {
  m := point
  m coincident(t == s) g
  n := plane(u: hint(dir: up, turn: -90deg, about: g), v: hint(dir: up))
  n.origin coincident m
  n square(up: up) g
}

// The plane square to the arc `g` at the fraction `s` of its turn: through the arc's axis, its `u`
// running out from the centre to the arc, its `v` along `up`, the normal of the plane the arc is
// drawn in.  Square to the arc's tangent, and turned about the axis as the arc turns: along a
// guide in one plane this is the rotation-minimising frame.
component OnArc(g: arc, up: axis, s: Scalar) {
  m := point
  m coincident(t == s) g
  out := line(g.center, m)
  n := plane
  n.origin coincident m
  n.u parallel out
  n.v parallel up
}

// The plane square to the spline `g` where its parameter is `s`: the tangent there drawn as a line
// from `m`, the plane square to it, its `v` along `up` (the normal of the plane `g` is drawn in,
// so again rotation-minimising for a guide in one plane).  The tangent is drawn 10 mm long, or its
// far end would slide along it, and starts ahead along `g`, so the plane starts facing the way
// the guide runs, `v` along `up` and not the mirror of it.
component OnSpline(g: spline, up: axis, s: Scalar) {
  m := point
  m coincident(t == s) g
  q := point hint(at: m, along: g, by: 0.1)
  tl := line(m, q)
  g tangent(at: m) tl
  length(10mm) tl
  n := plane(u: hint(dir: up, turn: -90deg, about: tl), v: hint(dir: up))
  n.origin coincident m
  n square(up: up) tl
}

// The plane square to any path `g` — a line, an arc, a spline, a curve, or a curve traced in
// space (#363) — where its parameter is `s`: `OnSpline`'s place and tangent over a `path`, and the
// rotation-minimising frame along it, `Minimal`'s.  A line's and an arc's `s` is the fraction of
// its run (an arc's by angle), a spline's its own parameter, a traced curve's its formal.  Given a
// named open chain of lines and arcs, a family runs it link by link, `s` from 0 to the count of
// its links (#243 L4), the frame carried across each joint.  The frame is integrated along the
// running number from where its family stands drawn, its home, where `n.v` leans toward `up` and
// `n.u` is square to it; from there its axes turn only as the guide makes them (`n.u`'s rate has
// no part along `n.v`).  Over a guide in one plane with `up` its normal that is the frame holding
// `v` along `up`; along a twisted guide `up` is read at the home alone, so the plane stays square
// to the guide where an `up` held everywhere would lean it.  Give it the family's running number
// as `s`.
component On(g: path, up: axis, s: Scalar) {
  m := point
  m coincident(t == s) g
  q := point hint(at: m, along: g, by: 0.1)
  tl := line(m, q)
  g tangent(at: m) tl
  length(10mm) tl
  n := plane(u: hint(dir: up, turn: -90deg, about: tl), v: hint(dir: up))
  n.origin coincident m
  n.u perpendicular tl
  n.v perpendicular tl
  n.u perpendicular n.v
  rate(n.u over s) perpendicular n.v
  initially {
    n.u perpendicular up
  }
}

// A screw's station (#243 L6, #366, #391): the screw about `base`'s normal through its origin, a
// frame whose value is `360deg · turns · s` — turned that much and risen `pitch · turns · s`
// along it — and `n`, `base` drawn in it, square to the axis: a section drawn in `n` (an auger's
// flight, a helical gear's tooth) sweeps a twisted prism.  `axial` is the plane through the axis,
// `n` drawn in a turn a quarter about its `u`: its `u` runs out along `n.u` and its `v` up the
// axis, and a section drawn in it (a thread's) sweeps a thread.  The screw turns from `base.u`
// toward `base.v` as it rises, right-handed for a positive pitch, left for a negative.
component Helix(base: plane, pitch: Length, turns: Scalar, s: Scalar) {
  // the screw: a frame turning about `base`'s normal, rising `pitch` a turn, at its value
  private screw := frame
  screw screws(pitch) base.normal
  fix(value == 360deg * turns * s) screw
  n := base in screw
  // and `n` turned a quarter about its `u`
  private stand := frame
  stand turns(1) n.u
  fix(value == 90deg) stand
  axial := n in stand
}

// A helix's normal station: the plane square to the helix of radius `r` about `base`'s normal
// (`Helix`'s screw), rising `pitch` a turn, at `turns · s` turns from `base.u`.  Its `u` runs out
// from the axis and its `v` up the axis leaned back by the helix's lead angle, so a section drawn
// in it (a spring's wire) is carried along the helix, square to it at every station.  Where the
// helix starts, the plane is `base` drawn in a screw about its `u` axis whose value tilts `v` back
// from up the axis by the lead angle — `90deg + atan2(pitch, 2 * pi * r)` from `base.v` — and
// slides it `r` out along it; drawn in the screw about the normal, the helix's, it is carried
// from there (#366, #391), so its `normal` keeps its sense by the motions, never by continuity.
component OnHelix(base: plane, r: Length, pitch: Length, turns: Scalar, s: Scalar) {
  // the helix's screw, as `Helix`'s
  private screw := frame
  screw screws(pitch) base.normal
  fix(value == 360deg * turns * s) screw
  // where the helix starts: `base` turned about its `u` axis and slid `r` out along it
  private tilt := frame
  tilt screws(r * 360deg / (90deg + atan2(pitch, 2 * pi * r))) base.u
  fix(value == 90deg + atan2(pitch, 2 * pi * r)) tilt
  private start := base in tilt
  n := start in screw
}

// The rotation-minimising frame of a station (#339 D3, #243 L9): the plane `f` through `n`'s
// origin, square to the path as `n` is, whose axes turn only as the path makes them — `f.u`'s
// rate has no part along `f.v` (with both unit and square to the path, U′ ∥ T).  It is integrated
// along the running number from where its family stands drawn, its home, where `f.u` is square to
// `up` and `f.v` leans toward it; give it the family's running number as `s`.  Over a guide in
// one plane, with `up` that plane's normal, it is `On`'s frame; along a twisted path it turns
// against the path's Frenet frame at the path's torsion, where `OnHelix`'s turns with it.
component Minimal(n: plane, up: axis, s: Scalar) {
  f := plane(u: hint(dir: up, turn: -90deg, about: n.normal), v: hint(dir: up))
  f.origin coincident n.origin
  f.u perpendicular n.normal
  f.v perpendicular n.normal
  f.u perpendicular f.v
  rate(f.u over s) perpendicular f.v
  initially {
    f.u perpendicular up
  }
}

// Frenet's frame of a station on a curve traced in space (#363 S5): the plane square to `g`
// where its formal is `s`, `u` toward the centre of curvature there — the principal normal, its
// sense the centre's, no seed's — and `v` the binormal, so the plane faces the way the curve runs.
// Along a helix it is `OnHelix`'s plane, `u` the other way; it turns with the curve's torsion,
// where `Minimal`'s does not.  A curve with no curvature somewhere (a straight run, an
// inflection) has no centre there, and the station is refused where that happens.
component Frenet(g: path, s: Scalar) {
  m := point
  m coincident(t == s) g
  q := point hint(at: m, along: g, by: 0.1)
  tl := line(m, q)
  g tangent(at: m) tl
  length(10mm) tl
  c := point
  g curvature(at: m) c
  nl := line(m, c)
  n := plane(u: hint(dir: nl), v: hint(dir: tl, turn: -90deg, about: nl))
  n.origin coincident m
  n.u parallel(sense: same) nl
  n.v perpendicular tl
  n.v perpendicular n.u
}

// -- frames along a path (#398 C) ----------------------------------------------------------------
//
// A frame that rides along a path: its joint held along the path (`g coincident f.axis`, as a
// turn's is held about a line), so its pose where the path's parameter is `u` is read off the
// path after the solve — square to the path's tangent there, turned as the path makes it from
// where the frame is drawn, its home.  What is drawn in it is drawn at the home, and a face drawn
// there and swept as a declared parameter runs, `sweep(sec over s)`, stands at every station
// where the frame does, `s` the path's own parameter (a line's and an arc's fraction of its run,
// a spline's parameter, a traced curve's formal).  Where today's station words solve a plane at
// every station (`On`, `Minimal` and `Frenet` above), this frame is a motion, read once.
//
//   param s: Scalar over (0, 1)
//   along := std.stations.Along(g, up: std.z)
//   in along.f {
//     k := circle(center: along.f.origin)
//     radius(2mm) k
//   }
//   sec := face(k)
//   tube := sweep(sec over s)

// `std.stations.Along(g, up: u, home: h)` makes `f`, a frame along `g`, its rotation-minimising
// frame (`On`'s and `Minimal`'s): drawn square to `g` where its parameter is `h`, `v` leaning
// toward `u` and `u` square to it; its axes turn only as the tangent does.  The joint turns with
// none of the path's torsion: its ratio 0.  The frame is its own, placed by its rows from what it
// is given (through 0.175 it was handed one: `std.stations.Along(f, g, up: u)`).
component Along(g: path, up: axis, home: Scalar = 0) {
  private m := point
  m coincident(t == home) g
  private q := point hint(at: m, along: g, by: 0.1)
  private tl := line(m, q)
  g tangent(at: m) tl
  length(10mm) tl
  f := plane(u: hint(dir: up, turn: -90deg, about: tl), v: hint(dir: up))
  f.origin coincident m
  f.u perpendicular tl
  f.v perpendicular tl
  f.u perpendicular f.v
  f.u perpendicular up
  g coincident f.axis
  fix(ratio == 0, advance == 0mm) f
}

// `std.stations.AlongFrenet(g, home: h)` makes `f`, a frame along `g`, the path's own frame
// (`Frenet`'s): drawn square to `g` where its parameter is `h`, `u` toward the centre of curvature
// there and `v` the binormal; it turns with the path's torsion besides.  A path with no curvature
// somewhere has no such frame there.  The joint turns with all of the path's torsion: its ratio 1.
component AlongFrenet(g: path, home: Scalar = 0) {
  private m := point
  m coincident(t == home) g
  private q := point hint(at: m, along: g, by: 0.1)
  private tl := line(m, q)
  g tangent(at: m) tl
  length(10mm) tl
  private c := point
  g curvature(at: m) c
  private nl := line(m, c)
  f := plane(u: hint(dir: nl), v: hint(dir: tl, turn: -90deg, about: nl))
  f.origin coincident m
  f.u parallel nl
  f.v perpendicular tl
  f.v perpendicular f.u
  g coincident f.axis
  fix(ratio == 1, advance == 0mm) f
}

// A station with no guide (#243 L7): the plane through the places at `s` on three rails — a loft
// that joins its sections in space directly.  Each rail runs through one corner of every
// section: a chain of lines, one a link a pair of neighbouring sections (the ruled loft, a family
// running the chains in step), or a spline through them (`spline(through: …)`, the smooth one).
// The section's corners stand where their rails meet the plane, each `point in st.n` and
// `coincident` its rail.  `u` runs from the first rail's place toward the second's and `v`
// toward the third's, so the plane faces the way its rails are handed in.
//
//   component Quad(ra: path, rb: path, rc: path, rd: path, s: Scalar) {
//     st := std.stations.Across(ra, rb, rc, s: s)
//     a := point in st.n
//     a coincident ra
//     …
//     f := face(a, b, c, d, -> close)
//   }
component Across(a: path, b: path, c: path, s: Scalar) {
  private pa := point
  private pb := point
  private pc := point
  pa coincident(t == s) a
  pb coincident(t == s) b
  pc coincident(t == s) c
  private construction ab := line(pa, pb)
  private construction ac := line(pa, pc)
  n := plane(u: ab, v: ac)
}

// -- two faces drawn apart (#243 L8) -------------------------------------------------------------
//
// CAD's "loft these two sketches": the section at `s` joins corresponding corners of `a` and `b`
// by straight rails, corner `k` of each walk to corner `k` of the other, and stands where the
// plane through three of them at `s` cuts every rail (`Across`), so the ends are the faces
// themselves whatever their planes.  Correspondence is the walks' written order: a connector is
// where a walk starts (`face(b1, b2, b3, b0, -> close)` pairs `a`'s first corner with `b1`), and
// two walks of different lengths are made alike by naming a point on an edge in the shorter one.
// Nothing is inferred.  The first three corners must not stand in a line.
//
//   param s: Scalar over (0, 1)
//   body := sweep(std.stations.Ruled(inlet, outlet).f over s)
component Ruled(a: face, b: face, s: Scalar) {
  cycle (ea, eb) in (a, b) {
    private construction rail := line(ea.p1, eb.p1)
    q := point in st.n
    q coincident rail
    in st.n {
      side := line(q, next.q)
    }
  }
  st := Across(rail[0], rail[1], rail[2], s: s)
  f := face(side)
}

// A loft to a point: `a`'s corners joined to `tip` by straight rails, the section at `s` where
// the plane through three of them at `s` cuts every rail.  At `s = 1` the section is the point:
// the solid ends in an apex there.
component Apex(a: face, tip: point, s: Scalar) {
  cycle e in a {
    private construction rail := line(e.p1, tip)
    q := point in st.n
    q coincident rail
    in st.n {
      side := line(q, next.q)
    }
  }
  st := Across(rail[0], rail[1], rail[2], s: s)
  f := face(side)
}

// A section held to its rails (#243 L8): the face `prof`, drawn once anywhere, carried whole into
// the plane `n` by the map that takes `from` to `a`, `to` to `b` and `third` to `c` — moved,
// turned, scaled and sheared, so that three rails give an affine map (`mapped`, #357; its edges
// keep their names under `f`).  In a family whose station holds `a`, `b` and `c` where three
// rails cross its plane, the section follows the rails.  `from`, `to` and `third` standing in a
// line is refused where the map is stated.  Written as the word, `f := prof mapped(from -> a,
// to -> b, third -> c) in n`, this is the same statement.
component Affine(prof: face, from: point, to: point, third: point, n: plane, a: point, b: point,
                 c: point) {
  f := prof mapped(from -> a, to -> b, third -> c) in n
}

// A section held to two rails: the similarity taking `from` to `a` and `to` to `b`, keeping the
// turn — moved, turned and scaled alike both ways, never stretched: two rails give a similarity,
// and an arc or a circle of `prof` stays one, its radius scaled.
component Similar(prof: face, from: point, to: point, n: plane, a: point, b: point) {
  f := prof mapped(from -> a, to -> b) in n
}

// -- laws of a station (#243 L5) ----------------------------------------------------------------
//
// A twist or a scale is a dimension of the section reading `s`, and any expression in it is a
// law: `twist * (3 * s^2 - 2 * s^3)`, `r * (1 + 0.2 * sin(180 * s))`.  A word takes numbers, not
// laws (Solvent has no function values), so these take a total — the turn or the scale at
// `s = 1` — and run it linearly from the section as drawn at `s = 0`; any other law is the same
// word given another fraction, `s: 3 * s^2 - 2 * s^3`.  A `when` cannot say a law: it is made
// once, for the whole run, and the station is one shape all along it; `min`, `max` and `abs` say
// a law in pieces.  A section drawn once is carried whole, turned and scaled by the similarity
// its frame is mapped by (#357), the law in where its frame's second point stands: `Twisted`'s
// turned point, or any point the station places (`lofts/tapered.sv`).  `scaled` and
// `scaled_radius` are for a section written as coordinates in the station's own plane.
//
//   component Duct(g: path, up: axis, prof: face, o: point, x: point, s: Scalar) {
//     st := std.stations.On(g, up: up, s: s)
//     tw := std.stations.Twisted(st.n, twist: 90deg, reach: 10mm, s: s)
//     sec := prof mapped(o -> st.n.origin, x -> tw.t) in st.n
//   }

// The point `reach` along the plane `n`'s `u` from its origin, turned `twist · s` about the origin
// within it — drawn in a turn of `n` about its origin whose value is that angle (#366, #391):
// where a section drawn once carries the second point of its frame, so the section turns as it
// runs — `sec := prof mapped(o -> n.origin, x -> tw.t) in n`, `o` and `x` the section's frame
// `reach` apart (#357).  At `s = 0` it stands on `n`'s own `u`, and the turn is the angle itself,
// so a twist past a half turn is a twist past a half turn.
component Twisted(n: plane, twist: Angle, reach: Length, s: Scalar) {
  private turn := frame
  turn spins(1) n.origin
  fix(value == twist * s) turn
  in n {
    private x := point
    x on_u(d: reach) n
    t := x in turn
  }
}

// Where a point stands in a plane's frame as its section is scaled about the origin: `(du, dv)`
// at `s = 0`, `scale` times that at `s = 1`, and run linearly between — `p coords` of a section
// drawn at its start's size.
p scaled(du, dv, scale, s) P := {
  p distance(du * (1 + (scale - 1) * s), along: u) P
  p distance(dv * (1 + (scale - 1) * s), along: v) P
}

// A round's radius scaled as `scaled` scales a point: `r` at `s = 0`, `scale` times it at `s = 1`.
scaled_radius(r, scale, s) k := radius(r * (1 + (scale - 1) * s)) k
