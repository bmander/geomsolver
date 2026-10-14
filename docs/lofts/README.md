# Lofts and sweeps written in Solvent

The record of #243, the epic to write lofts and sweeps in Solvent rather than in Rust. Each item
adds its section here as it reports.

**The rule every step is held to** (#183's, restated for lofts): what a loft *means* is written in
Solvent. Rust grows only three kinds of things:
- general language features, each with a use outside lofts;
- one primitive, the face family, and its general evaluators;
- fast paths that recognise a shape the family already describes (a rigid station, an affine
  corner) and build it exactly. They never change what the family says, and the general
  evaluation gates each one.

## L1: the face family (2026-10-14)

**Verdict: the primitive stands.** `solid(I(…).f over s in (a, b))` parses, elaborates and
evaluates (Solvent 0.93, §6.9); every L0 program the language can write today sweeps its closed
form (`rust/gcs-core/tests/face_family.rs`):

| program | solid | closed form (mm³) | faceted (mm³) |
|---|---|---|---|
| `reducer.sv` | `body` | 5120 | 5120.000000 (flat walls: exact) |
| `elbow.sv` | `body` | 1920π = 6031.858 | 6031.734 |
| `twisted_duct.sv` | `duct`, 90° | 16000 | 15999.974 |
| | `auger`, 1260° | 16000 | 15996.32 |
| | `duct`, eased `twist·(3s² − 2s³)` | 16000 | 15999.97 |
| `pyramid.sv` | `pyramid`, to its apex | 4000 | 4000.000000 (the apex a limit) |
| | stopped at `s = ½` | 3500 (frustum) | 3500.000000 |
| `square_to_circle.sv` | `transition` | 8000 + 1000π = 11141.59 | 11141.34 |
| `spring.sv` | `spring` | π·1.5²·4√((20π)² + 25) = 1782.18 | 1780.52 (the wire a 111-gon) |
| | `thread` | 240π = 753.98 | 753.83 |
| `blade.sv` | `blade` | 2100 | 2100.000000 |
| `bottle.sv` | `bottle` | π∫x² dy, Gauss–Legendre exact | within 1e-4 |

How it is built (the rules are in [the solids contract](../contract/solids.md)):

- **The station is drawn, hidden, and solved with the drawing** at the middle of the interval:
  the flattener walks the instance under a key the source cannot write (`#f12.`), the running
  formal left unbound and **held** — a fixed parameter every expression reads as a number
  (`Sketch::held`), so a law in `s` is any expression and no unknown is ever minted for `s`.  The
  document's DOF and diagnosis are the drawn station's too: an under-held section is said where
  the drawing is.
- **Each other station is the station's part solved again** (`io::Part`, everything else held
  where the drawing stands), started where its two neighbours carry it, so the branch the drawing
  chose is the one carried; a station inside the interval must be regular (full column rank).
  Steps are as long as a cubic through four solved stations stays within the sheet's flatness,
  and the facets between are that cubic's: the auger is 117 solves, not 4000.
- **Ends as limits** (finding 9): an end that loses rank is read as the limit of the stations
  closing in on it, never solved — the pyramid's apex, the square-to-circle's vanished flats.
- **Topology is the station's**: a loop crossing itself, an edge lost or turned over, a hole
  touching the section, and a swept volume crossing itself (a section reaching past its bend's
  axis) are each refused at the station where they happen.
- **Names** (finding 12): caps `start` and `end`, walls the face's edge names under the section's
  path in the instance — `body.sec.ab`, `body.sec.inner.loop.ab`, `transition.k0`.
- **Reads as a traced body** (finding 1): a station's dimension may read a number of the geometry
  it was given (`rim.r`, `a.x`), held as that geometry stands at each station; one of its own is
  refused.
- **Twisted walls are fanned**: a quad that does not lie flat is four triangles about its middle,
  which hold a ruled patch's volume where a diagonal errs to one side — what takes the duct from
  −0.6 % to −1.6e-6.

What L1 found on the way:

- **Two L0 programs solved to the wrong shape and said DOF 0.**  `square_to_circle.sv`'s drawn
  station took every flat to a point and every corner arc the long way round; it now seeds the
  flats' ends where the rounded square has them (`rect_fillets.sv`'s way).  `bottle.sv`'s rail
  could reach a shoulder of radius 28 at 60 only by running past the neck and back (its control
  point 464 mm up); its shoulder is 20 now.  The family refused the first and integrated the
  second over the wrong rail — DOF 0 says a drawing is held, not that it is the drawing meant.
- **`io::Part` had two holes**, both now closed: a plane's origin, held at its own `(0, 0)`, was
  taken for a wall, so a part reached through a moving plane stopped at it; and `graft` rebuilt a
  plane held along drawn lines (`std.Turned`'s) as a free plane, its intrinsic rows lost.
- **Cost**: a station is a solve of its part (about 2 ms for the twisted duct's 77 unknowns), so
  the stations are as few as a cubic allows and the faceting is interpolated; every L0 family
  evaluates in under a second.

Handed on: L2's exact surfaces (`Surface::Family`, the analytic fast paths), and the drawn,
draggable station of finding 11 (L10).

## L0: the target programs (2026-10-12)

**Verdict: the bet holds.** Ten programs say every option of a CAD loft dialog as ordinary
constraints in a section component drawn at a running station:
- where the section stands on its path, and how its frame turns;
- a twist of any size and law, a scale, two rails;
- a smooth loft through three sections, with its end conditions;
- square to circle, and a loft to a point.

**No new relation kind was needed.** The section of each of the eight programs that today's
language can nearly say was solved at one station (`s: 0.5`) in today's spelling. Every one
solves at **DOF 0**, standing where its closed form says. What is missing is the primitive itself
(`solid(… over s in …)`, finding 1) and a short list of general features (findings 2–9).

The programs are in [`l0/`](l0/), outside `rust/examples/` (the only directory the suites sweep),
so nothing in the repository reads them but L1's gate (`tests/face_family.rs`). At L0 each
stopped at its first error:
- eight stop at the family (`expected , or )` at `over`);
- `pipe.sv` and `two_faces.sv` stop at the words that cannot be written yet
  ([`l0/pending.sv`](l0/pending.sv): `path` is not a type).

The station probes are in [`l0/stations/`](l0/stations/): each program with its family drawn at
`s: 0.5` in place of its solid, over [`l0/stations/lofts.sv`](l0/stations/lofts.sv), the library in
today's spelling. They run (`solventc docs/lofts/l0/stations/reducer.sv`) and are L1's first
fixtures.

| program | from | what it exercises | closed form (mm³) | station at s = ½ |
|---|---|---|---|---|
| [`reducer.sv`](l0/reducer.sv) | `solid_loft.sv` | one hollow section, its size running | 5120 | DOF 0, outer half 9 |
| [`elbow.sv`](l0/elbow.sv) | `solid_elbow.sv` | a section carried round an arc | 1920π (Pappus) | DOF 0, at 45° on R30 |
| [`twisted_duct.sv`](l0/twisted_duct.sv) | `solid_twist.sv` | a twist law; 1260° in one statement | 16000 at any twist (Cavalieri) | DOF 0, corner turned 630° |
| [`spring.sv`](l0/spring.sv) | — | a wire square to a helix; an axial thread under a screw | 9π·√(400π² + 25); 240π | DOF 0, lead angle, rise 10 |
| [`pipe.sv`](l0/pipe.sv) | — | a chain of lines and tangent arcs as one path | 16π(60 + 25π) | *pending: `path`* |
| [`bottle.sv`](l0/bottle.sv) | — | smooth through three sections; end tangents | quadrature | DOF 0 (shoulder R 20 since L1) |
| [`blade.sv`](l0/blade.sv) | — | two rails in plan holding the width | 2100 | DOF 0, rails at ±7 |
| [`square_to_circle.sv`](l0/square_to_circle.sv) | — | a square to a circle, one section | 8000 + 1000π | DOF 0, corners ±5, R5 |
| [`pyramid.sv`](l0/pyramid.sv) | — | a loft to a point | 4000 | DOF 0, half 5 |
| [`two_faces.sv`](l0/two_faces.sv) | `solid_twist.sv` | two faces drawn apart, paired by the word | 40/6 · 1856 (prismatoid; today's `solid_twist` measures it) | *pending: finding 8* |

### The vocabulary the programs assume

- **`solid(I(…).f over s in (a, b))`**: the solid swept by face `f` of the instance `I` as its
  numeric formal `s` runs over the interval. It is §6.5's curve (`Involute(…).p over u in (…)`)
  one dimension up. The instance is written in place and never drawn.
- **A station word** (`lofts.OnLine`, `OnArc`, `OnSpline`, `Screwed`, `OnHelix` in
  [`l0/lofts.sv`](l0/lofts.sv); `On` over any path in `pending.sv`) is a component holding a plane
  `n` square to the path where its parameter is `s`, with its `v` along `up`. Along a guide in one
  plane, that is the rotation-minimising frame. Twist is not the word's: it is a directed angle in
  the station's own plane (`twisted_duct.sv`, through `std.Turned`), so the angle is in a view and
  any number of turns is one statement.
- **The section is the writer's component** drawn `in st.n`:
  - its numbers read `s` (`size: inlet + (outlet - inlet) * s`);
  - its corners may reach rails drawn elsewhere (`rim := point in st.n, front`, §6.7's point in two
    planes);
  - correspondence is identity: the edge `ab` at one station is `ab` at every other.
- **Caps** are `start` and `end`, as today. Walls are named by the section's edge paths
  (finding 12).

### Findings

1. **The family is the one primitive, and the only thing the programs cannot say.** Everything
   else in the eight probed programs is today's language (with findings 2–4's workarounds). The
   work is in Rust: a trace body today is planar points only (`compile_trace` refuses planes, arcs
   and points in space), and a station is a whole instance in space.
   - L1 either generalises the trace to whole instances, or solves each station as a part
     (`io::Part`) warm-started from its neighbour.
   - Either way, the family's body must read as a traced body: dimensions may read geometry
     (finding 8) and any expression in `s` (`twist * (3 * s^2 - 2 * s^3)`), since `s` is a column
     the station holds fixed, not a free name of the drawing (which is affine only).

2. **A point at a fraction of a line's or an arc's run.** *Done (#248, Solvent 0.82).*
   - `m coincident(t == s) g` was refused on a line and on an arc ("`coincident` has no slot `t`").
     A spline's contact has `t`, and so does a curve's.
   - A line's and an arc's point contact now take `t` too, as a fraction from `p1` to `p2` (on past
     either end) and from `start` to `end` by angle (held to the arc), in a view, across views and
     in a traced body. The probe's `OnLine` and `OnArc` are the target's, and no station is handed
     a run or a sweep (`tests/fractions.rs` holds `reducer.sv` and `elbow.sv` to their stations).

3. **A point in space on a curve drawn in a plane.**
   - `m coincident(t == s) g` with `g` an arc or a spline in `std.top` and `m` in space is E062.
     *An arc's is read in space since #248* (`PointAlongArc3`: the elbow's `OnArc` passes no
     `view`).
     The station's point must be drawn in the guide's plane.
   - A closed component cannot name the plane its argument is drawn in, so the probe passes it
     (`view: plane`).
   - Options:
     - read such a contact in space, as 0.74 reads `gear.M coincident q` (the twin rule);
     - or give a drawn entity a member naming its plane (`g.plane`).
   - **Done for a spline and a traced curve** (#249): a point in space said on one is drawn in its
     plane (membership, §6.21's one rule), and a line said tangent to one has its free ends drawn
     there, so the probe's `OnSpline` takes no `view:`. With an arc's fraction read in space
     (#248), neither does `OnArc`: no station word is handed a plane.

4. **A plane's own axes are not held square, and every station word has to say so.** The first
   drafts of `OnLine` and `OnHelix` came out DOF 1 and DOF 4 (the reducer's and the spring's
   stations): `n.u` swung about `n.v`, which is what §6.7 says a plane is. Each word now states
   `n.u perpendicular up` or `n.v perpendicular n.u`. A std word for "the plane square to this
   direction, its `v` along that one" would say it once; it is useful beyond lofts (any datum
   square to a line). **Done (#250):** `std`'s `P square(up: d) l`, over a relation word's
   parameter that is an entity; `OnLine`, `OnSpline` and `pending.On` say it so. With `up` not
   square to the guide it is no conflict: the plane leans, its normal the guide's part square to
   `up`.

5. **One station word over any path** (`pending.On`) needs three things (the first and the
   third done, #251 — `path` and `g tangent(at: m) l` over every kind, `tests/tangent_at.rs`; the
   second is #248):
   - a `path` formal type: a line, an arc, a spline, a traced curve, an envelope, or a named chain;
   - finding 2's `t` on every kind;
   - **the tangent at a point** on every kind:
     - an arc has its At-form (`TangentArcLine`);
     - a spline has only `g tangent(t == s) tl`, which beside `m coincident(t == s) g` restates the
       point. That is a numeric-rank dependency (`stations/spline_station.sv`: DOF 1, "1
       geometric dependency"), the double root CLAUDE.md's At-form exists to avoid. A spline and
       a curve need `g tangent(at: m) l`.
   - `tl tangent g` is refused where `g tangent tl` is accepted. Operand order carries meaning by
     design, but for a symmetric tangency the refusal reads as a bug.

   Done in #251: `g tangent(at: m) l` reads the place of the contact holding `m` (one row), of an
   arc or a circle it is the radius square to `l`, of a line `parallel`; a line and a spline or a
   curve touch whichever is written first. With the dependency gone the spline station read DOF 1:
   the tangent line's own length, `q` sliding along it, which the dependency had hidden. The words
   now draw it 10 mm long — finding 13's paper cut again, a direction with no word of its own.

6. **A chain as one path** (`pipe.sv`). The station runs over a named chain link by link (`s` from
   0 to 5), so the joints need no gluing and the wall is one face per section edge across the
   bends.
   - This needs chains as values (a formal type; `repeat e in a.loop` works, passing a chain does
     not) and finding 5's contacts on a chain.
   - The alternative, a family per link unioned (`repeat e in route { … }`), needs kind dispatch
     (finding 8) and names a wall once per link.

7. **Seeding the first station.** *Done (#252, Solvent 0.81).*
   - The station plane's attitude must start near its answer: the bottle's twin rim point was judged
     against the seeded plane before the solve (E061, "`bottle.st.n` and `std.front` are parallel").
   - A literal seed in the word cannot serve every guide: seeding it for the bottle broke the
     reducer and the blade (a `stations/seeded.sv` was the bottle's alone).
   - Now:
     - a direction seed reads geometry (§6.4): `OnLine` seeds `n` from its guide and `up`,
       `plane(u: hint(dir: up, turn: -90deg, about: g), v: hint(dir: up))`, so every station
       starts square to its guide, and the bottle and every other station solve over the one
       library. L1 seeds every later station from its neighbour;
     - E061's twin judgement waits for planes the solve places: judged once `views::place` has
       placed them, else where the solve leaves them.

8. **The generic two-face loft is not writable** (`two_faces.sv`, `pending.Ruled`). CAD's
   "loft these two sketches" needs, in a word's body:
   - repeating over a face's walk (`repeat e in a`: "`a` is not a named chain"; a face is no
     chain, and there is no chain formal);
   - pairing two walks link by link (the link of `b` in the same place);
   - a face of a repeat's links when the count is not written (`face(e[0], …, e[3])` works only
     with the count spelled out);
   - a dimension reading a corner's coordinate (`a.x`: E103 outside a traced body; finding 1 lifts
     it inside a family);
   - making an arc where the walk has an arc (kind dispatch).

   **Decision needed:** L3 can retire `along:` with the running-section style (`reducer.sv`,
   `elbow.sv`, `twisted_duct.sv` say all of today's examples) and bring the two-face word back at
   L8. Between the two, pairing two independent sketches is not available. The alternative is to
   keep `SolidDef::Loft` until L8.

9. **A section that vanishes at an end.** At `s = 1` the pyramid's square and the transition's
   flats are nothing, and the station loses rank (DOF 2 and DOF 4, "equations carry no
   information"). At `s = 0` the transition's corners are arcs of radius 0, and that station
   solves. The family must read an end where an edge or the whole section vanishes as its limit
   (`s → 1⁻`), never solve it:
   - a vanished section is an apex;
   - a vanished edge ends its wall in a point.
   Only an interval's ends may degenerate; a station inside that loses rank is refused there.

10. **A law is the station's body, not an argument.** A twist or a scale law is an expression in
    `s` written where the section is. A library word can take only numbers (Solvent has no
    function values), so std words take a total twist or scale and run it linearly, and any other
    law is the writer's own station. This needs no language change; it is said here so no item
    adds a `law:` key.

11. **The spelling reads well in every program**: the instance written in place, `over s in
    (0, 1)`, as a curve's `over u in (…)`. The open alternative is a drawn station
    (`st := Reducer(guide, …)`, `solid(st.sec.profile over s in (0, 1))`): the workspace would
    show one station to drag along the path (L10). Unbound, though, `s` is a DOF of the drawing.
    **Decision:** in-place for now; the drawn form at L10.

12. **Wall names.** The walls are named by the section's edge paths under its instance
    (`body.sec.outer.ab`, `body.sec.inner.cd`), and the caps `start` and `end`, so a path never
    renames.
    - A wall whose edge vanishes at an end keeps its name.
    - Across a chain (finding 6) a wall is one name over several B-rep faces, as `Exact::named`
      keeps a face's two halves.

13. **Paper cuts** met on the way:
    - formals cannot be called `right`/`left` (direction words, E040) or `turn` (a built-in, W112);
    - a line cannot be called `tangent`;
    - `std.top` passed down from a component that cannot see it is reported at the callee's
      `in view` ("no such entity: `view`") rather than at the argument;
    - `Screwed` needs a reference line of an arbitrary length (`10mm`) to turn an angle from: a
      direction turned within a plane by a directed angle, with no length, would say it.

### What L0 hands on

- **Before L1 (language, each its own small issue):**
  - ~~finding 2 (`t` on a line's and an arc's contact)~~, done in #248;
  - ~~finding 3 (a point in space on a planar curve)~~, done in #249 (splines, curves) and #248 (arcs);
  - ~~finding 4 (a std word for a square frame)~~, done in #250;
  - finding 5 (`path` type, the At-form of a spline's and a curve's tangency);
  - ~~finding 7 (seed reading geometry for a plane; E061 after placement)~~, done in #252.

  Each has a use outside lofts.
- **L1 (the family):**
  - finding 1 (trace semantics over a whole instance in space);
  - finding 9 (ends as limits);
  - finding 12 (names).

  `l0/stations/` are its first fixtures, and the closed forms above its gates.
- **L3:** finding 8's decision, then `reducer.sv`, `elbow.sv` and `twisted_duct.sv` replace today's
  four examples.
- **L4:** finding 6 (chains as paths), with `pipe.sv` as the gate.
- **L5–L8:** `twisted_duct.sv`, `spring.sv`, `bottle.sv`, `blade.sv`, `square_to_circle.sv`,
  `pyramid.sv` and `two_faces.sv`, each its item's gate. Finding 10 keeps L5 free of a `law:` key.
