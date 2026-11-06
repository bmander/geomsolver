# Sets in the notation of sets (#783): P0 and P1

This is the record #783's P0 asks for. The corpus's hardest set documents were written by hand in
the proposed spelling (`in`, `not in`, `or` in a set body, `some`, `boundary`, the library's
regions), each new spelling was put to `solventc` as it stands (`0309375`), and wherever today's
words can already say what the proposal says, the two were run side by side and their outputs
compared. Below: what the compiler says to each new spelling, what already holds to the byte, the
documents rewritten, what reads better and what does not, and what P1 has to decide that the issue
did not ask. Nothing here changes the language; the rules stay in [sets.md](contract/sets.md) and
[solids-as-sets.md](solids-as-sets.md) records the move this one finishes.

## What today's compiler says to the new spellings

Each was tried in a document of two points `c` (the origin) and `e` (30 mm along `u`) in
`std.front`, and the line `ax` between them.

| tried | what came back |
|---|---|
| `solid({ p \| p in std.sets.Sphere(c, r: 10mm) })` | E001 *`in` is not a constraint* |
| `{ p \| p distance(<= 10mm) c; p not in { q \| … } }` | E001 *more on this line than the statement wanted* |
| `{ p \| p distance(<= 10mm) c or p distance(<= 10mm) e }` | the same, at each `or` |
| `k coincident boundary({ q \| q distance(<= 10mm) c })` | E001 *every group member needs a name*: `boundary(` is read as a call and `{` as a group |
| `k coincident { q \| … }.surface` | E001 *more on this line*: `.surface` reaches only a named set |
| `solid({ p \| some t: p in b at(under: f) t })` | E001 *expected a name* |
| `solid({ p \| some t in [0deg, 90deg]: p in b at(under: f) t })` | E001 *more on this line* |
| `solid({ p \| every t in [0deg, 90deg]: p in b })` | the same |
| `k in { q \| q distance(<= 10mm) c }`, `k in std.top` (a drawn point) | E001 *`in` is not a constraint* |
| `solid({ p \| p distance(<= 2mm) ball })`, `ball` a region | E040 *`ball` is a set …: at a distance from its surface by `distance(d, side: outside)` or `side: inside`*, then E080 *a solid is a region: a set whose body bounds a number, or a set its `inside` reads one of* |
| `k distance(3mm) ball`, `ball` a region or a `Sphere` (a drawn point) | the same E040 |
| `l tangent { q \| q distance(<= 10mm) c }` | E040 *a region has no surface to touch: touch the set it is bounded by* |

Every construct the proposal needs is refused today, by the parser or with a reason, so each is a
meaning given to a construct the compiler rejects (the bible's preferred change), and none
competes with a reading a document already relies on. E080's own wording is the one-number rule
the issue retires: *a set its `inside` reads one of*.

Two refusals met on the way are not the proposal's but stand in its path:

- `construction b := solid({ p | p distance(<= 10mm) c })` is E001 *geometry modifiers apply to
  declarations*: a solid whose set is written in place takes `construction` only where the parser
  rewrites the set as a sweep (`swept_set`). `private s := { p | … }` is the same E001. The
  proposal writes every set in place (`solid({ p | p in bead or p in bead2 })`), so P1 lifts both.
- `{ … }.surface` on a set written in place is E001 above; `boundary(…)`, a call, has no such
  gap.

## What already holds, measured

Where today's words already say the proposed set, the two were run through `solventc` (report,
`--step-assembly`, `--stl`; or `--step` for one solid) and compared byte for byte, the file
renamed apart so its name in the STEP header is the one difference allowed.

| the proposed set, in today's words | against | outcome |
|---|---|---|
| `solid({ p \| p distance(<= 10mm) c })`, the ball as its statement says | `solid({ p \| p inside std.sets.Sphere(c, r: 10mm) })` | STL and STEP identical |
| `solid({ p \| p distance(<= 2mm, side: outside) ball })`, `ball` the radius-10 region | — | the ball of radius 12 (`SPHERICAL_SURFACE(…, 12.0)`): `side: outside` with `<=` on a region is already S ⊕ B_d |
| `square_thread.sv`'s rod as three statements: `p distance(<= rod_r) ax`, `ax.p1 distance(>= 0mm, along: ax) p`, `ax.p2 distance(<= 0mm, along: ax) p` | the rod as written (`inside Cylinder`, `outside`/`inside HalfSpace`) | report, STEP assembly and STL identical but for the file's name |
| the same three statements as a local family `Rod(about, r)`, `rod := solid(Rod(ax, r: rod_r))` | the same | identical but for the file's name; 5.0 s against 5.1 s |
| `union.sv`'s wheel as `both := { p \| p outside { q \| q outside bead; q outside bead2 } }`, `wheel := solid(both)` | `wheel := solid(bead)`, `bead2 union wheel` | report, STEP assembly and STL identical |
| two balls' union of regions, `{ p \| p outside { q \| q outside A; q outside B } }` | — | solved and read by the field; its STEP refused with the reason (*a region in general position … is read by the field; no exact construction is made of it yet*) |
| the same of two whole solids | — | built exactly by the Rust kernel: 8377.58 mm³, two spherical faces |
| `hypoid.sv`'s plane sides `foot inside P` / `outside P` as `foot distance(<= 0mm, along: n) P` / `(>= 0mm, …)` | the document as written | `--json` report identical: the same residual (1.1377613370634375e-14) after the same 2182 evaluations |
| the same, two of the three sides flipped (a control) | — | still searching after ten minutes, against one second (stopped): the side is read, not ignored |

So **the proposal's regions are already what the evaluator builds.** A region written as its
statement says (`distance(<= r)`) and the same region read through a surface's one number are the
same term to the byte, so retiring the one-number rule moves nothing for the sets the corpus uses;
P1's byte-identical gate is the right one, and it is reachable. **`or` over solids named whole is
already the body term**: the De Morgan spelling the epic gave a meaning is recognised as `union`
exactly, so `or`, which replaces that spelling, needs no recognition of its own. A union of
regions is the evaluator's open item (the meridian fold takes no complement), as #737 left it.

## The documents, written in the proposed spelling

None of these parse. Each keeps its document's comments' claims; where a comment changed, it is
because the code now says what the comment said.

### `std.sets`

```sv
component Ball(center: point, r: Length) := { p | p distance(<= r) center }
component Sphere(center: point, r: Length) := boundary(Ball(center, r))
component Cylinder(about: line, r: Length) := { p | p distance(<= r) about }
component Cone(about: line, half: Angle) := { p |
  private construction g := line(about.p1, p)
  about angle(<= half) g
}
component HalfSpace(o: point, along: line, d: Length) := { p | o distance(<= d, along: along) p }
// a rod: within `r` of the line `about`, between its two ends
component Rod(about: line, r: Length) := { p |
  p distance(<= r) about
  about.p1 distance(>= 0mm, along: about) p
  about.p2 distance(<= 0mm, along: about) p
}
component Grown(S: set, d: Length) := { p | p distance(<= d) S }
// the points of `S` at least `d` from everything outside it
component Shrunk(S: set, d: Length) := { p | p in S; p distance(>= d) { q | q not in S } }
```

`ConeThrough` is `Cone`'s change at its last line (`spine angle(<= abs(lean)) g`). `Offset`
retires for `Grown`. `Slab` is not written: `o distance(in: (a, b), along: ax) p` already says a
slab in one statement (`hypoid_drive/drive.sv` writes twenty), so a word for it would be a second
spelling; `HalfSpace` stays because it is passed as a set (`Tooth(…, lo, hi)`).

### `std.solids`

```sv
// through everything: the points that are the section's slid some way along its normal
component Through(section: face) := { p | some h: p in section at(along: n) h }
component Prism(section: face, from: Length, to: Length) := { p |
  p in Through(section)
  p distance(>= from, along: n) section
  p distance(<= to, along: n) section
}
component Revolution(section: face, about: line, from: Angle = 0deg, to: Angle = 360deg) := { p |
  some t in (from, to): p in section at(under: about) t
}
component Drafted(section: face, from: Length, to: Length, draft: Angle) := { p |
  p distance(>= from, along: n) section
  p distance(<= to, along: n) section
  within := Through(section)
  beyond := { q | q not in within }
  distance(p, beyond) - distance(p, within) >= tan(draft) * distance(p, section)
}
```

### `square_thread.sv`

The section, the bar and the screw are as written; what changes:

```sv
// the rod
rod := solid(std.sets.Rod(ax, r: rod_r))
construction bar := solid(std.solids.Prism(face(lower, end, upper, nose), from: -reach, to: reach))

// the screw about the rod's axis, carrying the bar through it
cut := frame
cut screws(lead) ax
construction thread := solid({ p | some t: p in bar at(under: cut) t })
thread cut rod
```

### `union.sv`

```sv
construction bead := solid(std.solids.Revolution(face(rim), about: wheel_axis))
construction bead2 := solid(std.solids.Revolution(face(rim2), about: wheel_axis))
construction wheel := solid({ p | p in bead or p in bead2 })

feed := frame
feed slides(40mm) ax
construction groove := solid({ p | some t in (0deg, 360deg): p in wheel at(under: feed) t })
groove cut rod
```

### `hypoid.sv`: the cones, the blank and the generating crown

```sv
// GearCone, PinionCone: a foot below or above the pitch plane
  foot distance(<= 0mm, along: n) P
  virtual_foot distance(>= 0mm, along: n) P

// A member's blank, one region: inside its pitch cone grown by the addendum and its heel ball,
// outside its toe ball and its pitch cone shrunk by the back depth.
component Blank(rim: line, ax: line, design: BlankDesign) {
  toe := std.sets.Ball(rim.p1)
  heel := std.sets.Ball(rim.p1)
  rim.p2 distance(design.width / 2) toe
  rim.p2 distance(design.width / 2) { q | q not in heel }
  pitch := std.sets.Cone(ax)
  rim.p2 in boundary(pitch)
  tip := std.sets.Grown(pitch, d: design.addendum)
  back := { q | q not in std.sets.Shrunk(pitch, d: design.back) }
  beyond_toe := { q | q not in toe }
  region := { p | p in tip; p in heel; p not in toe; p in back
                  p in std.rounds.Chamfered(tip, beyond_toe, la: design.relief,
                                            lb: design.relief)
                  p in std.rounds.Chamfered(tip, heel, la: design.relief, lb: design.relief) }
  material := solid(region)
}

// FaceMill: the gear's space cutter reaches to a cylinder through `cap`
  reach := std.sets.Cylinder(ax)
  cap in boundary(reach)

// The crown tooth: beyond the flank cone `a` and within `b`, each grown by `grow`, above `lo` and
// below `hi`, its tip's edges rounded by `r`.
component Tooth(a: set, b: set, lo: set, hi: set, grow: Length, r: Length) {
  inner := std.sets.Grown({ q | q not in a }, d: grow)
  outer := std.sets.Grown(b, d: grow)
  base := { q | q not in lo }
  tip := { q | q in hi }
  inner_round := std.rounds.Rounded(inner, tip, r: r)
  outer_round := std.rounds.Rounded(outer, tip, r: r)
  region := { p | p in inner; p in outer; p in base; p in tip; p in inner_round; p in outer_round }
  construction crown := solid(region)
}
// Space: the same, with `cap`
  near := { p | p in inner; p in cap; p in tip; p in base; p in inner_round }
  far := { p | p in outer; p in tip; p in base; p in outer_round }

component Generated(body: solid, tool: solid, under: frame, seen_from: frame, index: frame,
    n: Int) {
  construction removal := solid({ p | some t: p in tool at(under: under, seen_from: seen_from) t })
  repeat n as i {
    private construction indexed := removal at(under: index) i * 360deg / n
    indexed cut body
  }
}

// Topping: a kink on the flank as grown
  kink in boundary(flank)

// Blade: the solid cone, its last line
  spine angle(<= half) g

// Balanced: each member's flank is the envelope of its crown flank's surface (`gear_flank` alike)
  pinion_flank := envelope(boundary(inner) over t in (-30deg, 30deg), under: crown_roll,
                           seen_from: pinion_roll)
```

### `std.rounds`

```sv
component Run(a: face, b: face, r: Length) symmetric(a, b) := { p |
  p in a.smooth
  p in b.smooth
  angle(a, b, at: p) <= 180deg
  centres := { c | c in std.sets.Shrunk(a.smooth, d: r); c in std.sets.Shrunk(b.smooth, d: r) }
  crease := { q | q in a; q in b }
  meeting := { q | q in boundary(a.side); q in boundary(b.side); q in crease.ends }
  distance(p, centres) >= r
  distance(p, crease) <= distance(p, meeting)
  p in crease.ends
}
component Fill(a: face, b: face, r: Length) symmetric(a, b) := { p |
  p not in a.smooth
  p not in b.smooth
  angle(a, b, at: p) >= 180deg
  centres := { c | c distance(>= r) a.smooth; c distance(>= r) b.smooth }
  …                                            // crease, meeting and the rest as `Run`'s
}
// within reach of a ball of radius `r` lying within both: the opening of `a` and `b` together
component Rounded(a: set, b: set, r: Length) :=
  std.sets.Grown(std.sets.Shrunk({ q | q in a; q in b }, d: r), d: r)
component Chamfered(a: set, b: set, la: Length, lb: Length) := { p |
  la * distance(p, { q | q not in a }) + lb * distance(p, { q | q not in b }) >= la * lb
}
// a solid grown: `std.sets.Grown(B, d)` and `std.sets.Shrunk(B, d)` made solids
component Grown(B: solid, d: Length) {
  when d > 0mm { body := solid(std.sets.Grown(B, d: d)) }
  when d < 0mm { body := solid(std.sets.Shrunk(B, d: -d)) }
  when d == 0mm { body := solid(B) }
}
```

`a.side` is a name this record has to invent: see below.

### A parameter declared, a curve's place, a drive's hub

```sv
// twist_drill/drill.sv: `param t: Angle over (start, finish)` as written
construction flute := solid({ p | some t: p in flute_wheel.body at(under: grind) t })
// lofts/thread.sv: `param s: Scalar over (0, 1)`, `sec` drawn reading `s`
thread := solid({ p | some s: p in sec at s })
// paths/bent_pipe.sv
pipe := solid({ p | some s: p in Tube(route, std.z, r: r).f at s })
// std.panels.Developable: p on the ruling between m and n
  p in std.sets.Segment(m, n)
// hypoid_drive/drive.sv's Mount: the hub's rim, the core's points within `rim` of the back
  rim := { q | q not in seat; q distance(<= m.rim) seat
               foot distance(in: (m.front, m.web), along: ax) q }
```

## What reads better

- **The blank reads as its comment.** "Its pitch cone grown by the addendum … outside its pitch
  cone shrunk by the back depth" is now `Grown(pitch, d: design.addendum)` and `not in
  Shrunk(pitch, d: design.back)`; today the second is `q distance(<= design.back, side: inside)
  pitch`, which a reader decodes into the comment. The same holds of the crown tooth ("beyond the
  flank cone, grown": `Grown({ q | q not in a }, d: grow)`) and the hub's rim.
- **`not in` and `distance` say the side, so `side:` has nothing left to say.** `rim.p2
  distance(width / 2) toe` is outside the toe ball by half a width because the distance from a ball
  is nothing inside it; `Fill`'s centres lose their `outside` conjunct, which the distance already
  implied (*each fact is stated once*).
- **`Rounded` is the opening of mathematical morphology**, `Grown(Shrunk(a ∩ b, r), r)`, and says
  so in one line; today its body is that formula with `side: inside` standing in for the erosion.
- **The quantifier makes a sweep's form one form.** A revolution, a motion's sweep over an
  interval, the whole roll, a declared parameter's family and a prism through everything are all
  `some x …: p in S at(…) x`; today they are `coincident(under: …, over …)`,
  `inside(under: …, over …)`, `inside(under: …)` and `coincident(over s)`, four parenthesised
  forms of one ∃. A generating cut (`hypoid.sv`'s `Generated`, `wankel.sv`, `swept_torus.sv`)
  now reads as what it is, the points the tool occupies at some roll.
- **`or` is a union** (`wheel := solid({ p | p in bead or p in bead2 })`), where today it is the
  De Morgan puzzle or two statements (`solid(bead)`, `bead2 union wheel`).
- **A rod is one call**, and the three statements it stands for are already the evaluator's
  (above): the issue's `Rod` costs nothing.
- **A complement is never implicit.** Every `{ q | q not in X }` stands where the reader would
  otherwise have had to know which side of `X` an `outside` reads.

## What reads worse, or does not work as proposed

- **`boundary(…)` is ceremony where a point stands on a library set.** `rim.p2 in
  boundary(pitch)`, `cap in boundary(reach)`, `kink in boundary(flank)` and the two envelopes:
  five of the hypoid's lines grow by a word, and every point or line `std.sets`'s doc examples put
  on a set (`p coincident shaft`, `l tangent shaft`) would; for the tangency today's E040 already
  says so (*touch the set it is bounded by*). It is honest — the cone and its surface are different
  sets — but it is the price of the regions.
- **A point inside a set at a distance needs the complement written in place**: `rim.p2
  distance(width / 2) { q | q not in heel }` against today's `side: inside`. It is the one place
  the proposal is longer and no clearer; `Shrunk` does not help, since the point is on its boundary,
  not in it.
- **`{ q | q not in X }` is written ten times** in the rewrites above. It reads, but a library
  word for it (`Outside(X)`) would be a second spelling of `not in`; the record leaves it written.
- **A face read as a set is the prism over it today, and the face under `∈`.** `p inside sec` is
  `RegionShape::Section`: the point's foot on the face's plane lies within it, so off the plane the
  face's infinite prism. With `in` as membership `p in sec` is the planar face itself, a set of no
  volume, and `Through`, `Prism` and `Drafted` lose their spelling. The rewrite above says the prism
  as the face slid along its normal, `some h: p in section at(along: n) h`, which needs `at` to take
  a slide as it takes a turn (`under: line`). Written that way with an interval, though, `Prism`
  would be capped `start`/`end` by the naming rule for an interval, not `near`/`far`; so `Prism`
  stays two ordinates over `Through`, and only `Through` uses the quantifier.
- **`Drafted`'s `distance(p, section)` is the ordinate's size**, the distance from the face's
  plane. As the ordinary distance from the face it is that only over the face's footprint; off it
  the lean's other terms already refuse the point for a draft of zero or more, but not for a
  negative one. P1 writes it from the plane.
- **A face is two sets and a surface, and `boundary` is none of the extras.** `std.rounds` reads a
  face as its patch (`q coincident a`), the material side of its surface (`p inside a`), that side
  carried over tangent seams (`a.smooth`) and the surface itself (`a.surface`). `boundary(a)` of a
  patch is its closure (its interior in space is empty), not the surface it lies on. The rewrite
  makes the patch `a`, the two sides regions (`a.side`, `a.smooth`) and every surface the boundary
  of one of them; `a.side` is a name this record invents, and P1 has to choose one.
- **`.surface` on a conjunction was never `boundary`.** It makes each bound an equality, so of a
  set of two bounds it is where both surfaces meet, a curve; `boundary` is the union of each bound's
  surface where the others hold. Every `.surface` in the corpus (15 lines) is of a set of one bound
  or of a face, where the two agree, so the rename moves nothing; it is a change of meaning all the
  same, and said so.
- **`some s: p in sec` hides that `sec` reads `s`.** A face drawn reading a declared parameter is a
  family of faces, but the term names no `s`; the rewrite writes `sec at s` to show it, the reading
  at an instant the language already has (`REF at E`).
- **`some t:` means two ranges by what is in scope.** In `drill.sv` `t` is a declared
  `param t: Angle over (start, finish)`, so `some t:` ranges its interval; in `square_thread.sv` it
  is fresh, so it ranges the whole roll, bounded by what the solid is cut from. That is one rule (a
  variable ranges its declared domain, else all of it), but a `t` meant fresh in a component of a
  file that declares `param t` would silently range the parameter's interval. P1 lets a quantifier
  name a declared parameter (and range its interval) and refuses one that names anything else in
  scope, so a fresh name is fresh where it is written.
- **`Sphere := boundary(Ball(…))` and `Rounded := Grown(Shrunk(…))` are families whose body is a
  term, not a `{ p | … }`.** Today a family's body is a set written out. P1 either allows a set
  term there or writes `{ p | p in boundary(Ball(center, r)) }`.
- **Two `Grown`s.** `std.rounds.Grown(B, d)` (a solid's `.body`) and the issue's `std.sets.Grown`
  are one concept under one name in two modules, the first a `when` over the second and `Shrunk`.
  P1 keeps one: `solid(std.sets.Grown(B, d: gap))` says what `std.rounds.Grown(B, d: gap).body`
  says.
- **A line is two sets.** `p distance(r) about` reads the line's extension (so `Cylinder`, and
  `Rod` needs its two ordinates: the distance from the segment would be a capsule), while the
  developables' `p coincident(t in (0, 1)) line(m, n)` reads the segment. The rewrite makes the
  extension the line and the segment a word, `std.sets.Segment(m, n)` (the line and two
  ordinates). The developables read their clause with machinery of their own (#645), so P1 shows
  the word builds the same panel before it replaces the clause.

## The issue's open questions, as the experiment answers them

- **Interval brackets: keep `(a, b)`.** `[a, b]` is a list literal (`NAME := [ITEM, ...]`), and `x
  in LIST` already ranges a list's members (`repeat h in LIST`), so `some t in [0deg, 90deg]` would
  read as *t is 0° or 90°*. The language already writes an interval `(a, b)` in three places —
  `param t over (a, b)`, `over t in (a, b)`, `distance(in: (a, b))` — and reads all three closed.
  P1 says once in the specification that an interval after `in`, `over` or `in:` is closed, and
  writes `some t in (a, b)`. (The issue's `Slab(ax, in: [a, b])` falls with `Slab`.)
- **`or` and the exact export: `or` is enough.** Over solids named whole it is the body term, built
  exactly to the byte (measured above through the De Morgan spelling, which `or` replaces); over
  regions it is read point by point, as today's union is, and the exact union of regions is the
  meridian fold's complement — an item of the evaluator, not of the language.
- **`in`'s remaining collisions.** In the corpus `in` is: the block `in P { … }` and the
  declaration `p := point in P` (a point drawn in a plane: membership), `repeat e in S`, `cycle e
  in`, `cycle (a, b) in (W1, W2)` (17 lines: a copy for each member, ∀ by enumeration), `over t in
  (a, b)` (membership of the roll), `when c in B` (`std.rounds`: membership of a face among a
  solid's), `distance(in: (a, b))` (20 lines: the ordinate's membership of an interval), and
  `n := x in f` (7 lines: placement). Only the last is not membership, so only it moves to `at`,
  where it joins `x at(under: m) τ`: a reading at a placement, either way. One collision is new:
  `when not w in A` already writes the negation in front, where the proposal writes `p not in A`;
  P1 takes `not in` in both, and `not w in A` becomes the error naming it.
- **Which relations `in` takes over.** The issue retires `coincident` against a *set* and keeps it
  between entities. A plane is both: `p := point in P` declares a point in it, but `k in P` as a
  relation is E001 today and `k coincident P` the spelling. P1 decides whether a point in a plane is
  `in` (a handful of corpus lines move, `paths/routed_line.sv`'s `d coincident std.top` among them)
  or stays `coincident`, and says the line between an entity and a set once; this record's
  rewrites keep `coincident` for every entity, planes included.
- **Ligature reach** is P2's. The highlighter already singles `in` out (`syntax/highlight.rs`), so
  the tokens P2 needs exist for `in`; `some`, `every` and `not in` are new tokens P1 makes.

## What P1 takes from this

The words the issue lists, with these settled or added:

- intervals stay `(a, b)`, closed;
- `at(along: n) h`, a slide along a direction, beside `at(under: l) t`, a turn about a line, so
  `Through` is a quantifier; a slide's interval caps a prism `near`/`far` only if the naming rule
  says so, else `Prism` stays its two ordinates;
- a face's side and smooth side as regions, under names P1 chooses, and `boundary` for every
  surface;
- `construction` and `private` on a solid or a set written in place;
- a family's body may be a set term;
- a quantifier names a declared parameter or a fresh name, never anything else in scope;
- `n := x in f` becomes `n := x at f`, and `when not w in A` becomes `when w not in A`;
- `std.sets.Grown`/`Shrunk`/`Rod`/`Segment` in; `Offset`, `std.rounds.Grown` and `Slab` out;
- the corpus rewritten by script and held to the byte, as #737's was: the rows above say the
  regions already are.

## What P0 did not try

- **The body words over `open`** (P3): nothing was written for `x cut body` as a conjunct.
- **`every`**: reserved, not written; no document needs it yet.
- **A full corpus run.** The comparisons above are the documents the issue names, and the probe
  sets; the corpus-wide byte gate is P1's, once the spellings parse.

## P1 — the words, as built ([0.205])

**How it is read.** The parser lowers the notation to the forms the evaluator has always read
(`syntax/notation.rs`, the contract in [sets.md](contract/sets.md)): `p in S` to `p inside S`,
`some t in (a, b): p in X at(under: m) t` to the sweep, `boundary(S)` to `S.surface`, and so on,
each lowered relation marked (`Written::lowered`). What needs the operand's kind is read once it is
known: in the flattener a membership of a set with no inside is `coincident` (the one-number rule
gone), a plain distance from a region is the ordinary one, and `coincident` written against a set
and `side:` on one are refused; in the elaborator `p in a`, `a` one face of a solid, is the face,
and `p in P`, `P` a plane, is refused. Because the evaluator meets the same internal forms, the
corpus's outputs did not move: each of the 282 documents of `rust/examples/` gives the
same report, STEP assembly and STL, byte for byte, before the rewrite and after.

**What was decided as it was built**, beyond the P0 record's list:

- **`std.sets.Grown` and `Shrunk` are not words.** A set grown is one statement, `{ q | q
  distance(<= d) S }`, as a slab is (`Slab` was dropped for the same reason), and
  `std.rounds.Grown`, the solid grown and built exactly, keeps its name: every word of `std` has
  one home, and the solid's exact route reads the distance from a solid's set (`{ c | c in B }`), which `distance(<=
  d) B` written against the solid itself does not say yet. Retiring `std.rounds.Grown` for one word
  over sets and solids needs the distance from a solid to be the ordinary one in the elaborator;
  that is left open.
- **`claim a inside b` stays** (until P3, below). Between two solids it says one lies within the other — ⊆, not ∈ —
  so it is no membership; the notation leaves it, and a spelling for it waits on the body words
  (P3).
- **`side:` stays where there is no inside**: a developable sheet's band says which side of the
  sheet it lies on, and a plane's side grown is still read so (an ordinate along its normal, `p
  distance(<= d, along: n) P`, is the spelling the documents use).
- **A family's body stays a set written out.** `std.sets.Sphere` is `{ p | p distance(r) center }`,
  the sphere, so no family needs a set term for its body.
- **`boundary(…)` takes a call** written in place (`boundary(std.sets.Cone(l, half: h))`), hoisted
  with its member; a set literal inside it is not read yet.
- **A quantifier over a declared parameter** ranges its interval (`some t:`, `t` a `param` of the
  file); a fresh name ranges the whole roll. The rule that a bound name never shadows another
  declaration is not enforced yet: the lowering reads the file's `param`s, not its scopes.
- **`some … at t` with no motion** is a face family where its operand is a face (drawn, a formal
  `face`, or a family's member) and a ridden body's sweep otherwise.

## P2 — the signs, as built

The editor draws `in` as ∈, `not in` as ∉ and `some` as ∃; the document stays ASCII
([app.md](contract/app.md)).
What was decided as it was built:

- **Reach: the three words that exist.** `every` is reserved and refused, so it has no sign until
  it is read. `or` stays a word.
- **Which `in`:** wherever it stands between two operands, which in the corpus is always
  membership — of a set, of an interval (`t in (a, b)`, `over t in`), of a list (`repeat e in S`)
  or of a plane (`p := point in P`, a component drawn `in std.front`). The block `in P { … }`
  opens a line with nothing before it, and keeps its word. Across `rust/examples` and `std`: 272
  ∈, 21 ∉, 45 ∃.
- **The caret shows the words; no key handler.** The issue asked for a handler so that a backspace
  at a sign reverts it. Showing the ASCII wherever the caret touches a sign does that with the
  textarea's own keys: at `p in |` a backspace takes the space, the caret touches `in`, and the
  next takes its `n`. While the box does not have the focus every sign is drawn. A sign needs an
  operand after it, so `p in` being typed at a line's end is the word until something follows.
- **Not a font feature.** The words stay in the copy, ink invisible, and the sign is an absolutely
  placed box over their cells; `npm run overlay` holds that no glyph moves.

## P3 — the body words over a primitive, as built ([0.206])

**The experiment.** The question P0 left was what a body's set "extended by statements written
elsewhere" is. Read as inclusions, `cut` and `common` are `B ⊆ ∁x` and `B ⊆ x`, but `union` is not
`x ⊆ B`: the union is taken first, so a pocket cut from the body eats a boss in union with it —
`(stock ∪ boss) ∖ pocket` — and `boss ⊆ B` is then false (the primer's "name the intermediate").
So the primitive cannot be a set of facts the body satisfies; it is the body's formula assembled
from pieces said where the body is not declared: one disjunction, its stock or each union, and one
conjunct for each cut or common. Written that way it is a statement the parser already had the
pieces of — a set written in place, after `and` or `or`:

```
x union B := B or { p | p in x }
x cut B := B and { p | p not in x }
x common B := B and { p | p in x }
```

Each of `std.rounds`' round, chamfer and shell, `hardware`'s groove and bolt pattern, an empty
body joined by a `repeat`, and a swept solid taking a feature directly was read through the words
so defined, against the built-in statement: the same `SolidDef::Body`, so the same solid to the bit
(`tests/body_words.rs`) and the same reports, STEP and STL across the corpus (below).

**What was decided as it was built:**

- **No `open`.** Every body takes contributions, as every one always could; marking the ones that
  do would have rewritten every body and every swept solid that takes a feature, and said nothing
  the contributions themselves do not.
- **The words live in `std.solids` and are imported by name**, as every word of `std` is: there is
  no prelude, so a file writes `use std.solids (cut, union)`, the names it uses. The corpus gained
  them by script (`scripts/body_words_imported.py`); `hardware` imports its own, and `std.rounds`
  writes the primitive, its contributions repeated over pairs and triples of faces.
  Unimported, the use is E102 naming the import. The words are no longer reserved: a file may name
  a thing `cut`.
- **One membership of a solid per contribution, for now.** `B and { p | p not in x; p in y }` or a
  set with a bound in it would be a body's conjunct as well; the parser refuses them naming the
  three forms until a body reads them.
- **The stratum is the primitive's rule.** A contribution whose solid reads the body's own faces (a
  round's taken and filled solids) is read against the body the others make — the evaluator's
  rule as it was (`solid::faces::stratum`), now said of the primitive.
- **`every` lands, in claims.** One solid within another is `claim every q in a: q in b`, lowered to
  the claim the solid claims judge; `claim a inside b` is refused naming it, and `every` outside a
  claim is refused (a statement that every value of a running number, or every acceptable part,
  satisfies something waits on the tolerances). The editor draws it ∀.

- **A thing named for a word is renamed where the file imports the word.** Two documents called a
  screw's frame `cut` (`solid_sweeps/square_thread.sv`, `stopped_screw.sv`) and cut with the word
  as well; a file's own name and a word it imports may not share a spelling (E071), so the frame is
  `feed`. A solid named `inside` (`engine/block.sv`) stays: a constraint word with no prefix
  reading, read as a reference with a word and an operand after it, is the name it was given
  (`inside common cavity`).
- **The report says the containment claim as it is written**, `claim every q in bore: q in stock —
  …`.

The corpus's outputs did not move: each of the 282 documents of `rust/examples/` gives the same
report, STEP assembly and STL, byte for byte, before P3 and after.
