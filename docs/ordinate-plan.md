# An ordinate along any direction, and `level`: plan

**Status (2026-10-06): implemented** on branch `ordinate-level`, all five phases in one change
(Solvent Draft 0.46). The proposed defaults below were taken as written. Relation aliases in the
library are #94.

A drawing that wants "this point is `d` along that direction from that one" has seven ways to
say it today, chosen by what the direction happens to be: the page's x or y
(`HorizontalDistance`, `VerticalDistance`), a plane's u or v with the point drawn in it
(`CoordinateU`, `CoordinateV`) or not drawn in it (`Ordinate3U`, `Ordinate3V`), and a plane's normal
(`PointPlaneDistance`). Two more say the same thing at zero (`HorizontalPoints`,
`VerticalPoints`). No form takes an arbitrary axis or line, so the corpus reaches for workarounds:

```
centre distance(0mm, along: u) std.top          // the wankel: "centre is at the origin"
centre distance(0mm, along: v) std.top
h distance(-10mm * sin(setting), along: u) side // twist_drill/wheel.sv: an offset along a
h distance(10mm * cos(setting), along: v) side  //   turned axis, split into components
```

Every one of these is one equation,

  **(Y − X) · t̂ = d**

the signed distance from X to Y measured along a directed line t. This plan makes that the
statement: one kind for it, **`Ordinate`**, and one for its zero, **`Level`**, with the seven
plus two folded in. The words written today (`along: x`, `along: u` against a plane, …) stay as
spellings of particular cases. `along:` also takes a reference: any axis or drawn line.

## Decisions

Made in conversation on 2026-10-06:

* **One kind for the ordinate.** The statement is a single `CKind`, written `a distance(d,
  along: t) b`. Which kernel evaluates it, a page axis's, a line's in a view, or a direction in
  space, is the core's choice from the operands, not a second kind.
* **`level` is the zero.** `a level(t) b` says that a and b have the same ordinate
  along t: one row, (b − a)·t̂ = 0, with no number and no callout. It is a relation, so it is a
  kind of its own (a kind with a `Length` slot is a dimension and is drawn); it shares
  `Ordinate`'s slots, apart from the number, and its kernels at d = 0. Its direction is
  positional, as `symmetry(l)`'s line is: with no number in the parentheses there is nothing for
  a key to tell it apart from.
* **`horizontal` and `vertical` between points are aliases.** `a horizontal b` is `a level(up)
  b` and `a vertical b` is `a level(right) b`, and both spellings are accepted. `horizontal l` of
  a *line* stays `Horizontal`: a line's direction is a different statement and decomposes as one.
  Ideally the alias would be a standard-library definition; the language has no way to define a
  relation word yet (see [Open](#open)), so it is one entry in the core's operator table, written
  so that it can move to `std` without changing what a document says.

Revised later the same day: the first draft keyed `level`'s direction (`level(along: t)`) and
made `horizontal` the only spelling of the page case.

Proposed here, and taken as written:

* **A page word needs a view.** `along: x` is the x of the view both points are drawn in, held
  as that view's `u` axis. A 2D sketch with no plane (hand-built tests, a bare JSON document) has
  no axis to hold, so the page words are refused there, and the tests that use them get a plane.
  The alternative is a slot that holds a word *or* an entity, which every walk over a kind's
  entities (`entities()`, `graft`, the drag part, `remove`) would then have to special-case. A
  planeless sketch is no longer anything the language writes (`in std.front` is the 2D drawing),
  and nothing has been deployed.
* **A literal zero ordinate is E040, naming `level`.** `distance(0mm, along: …)` is the smell
  this plan removes: as a dimension it gets a callout and asks to be edited. A zero that comes
  out of a parameter (`e := 0`) is still a dimension and is accepted, as with a negative
  magnitude, which is refused only where it is written.
* **No plane–plane form.** `P distance(d) Q` stays `PlaneDistance`: it relates two planes, not
  two points.

## The spelling

| written | means | X, Y | t |
|---|---|---|---|
| `a distance(d, along: t) b` | (b − a)·t̂ = d | a, b | axis or drawn line `t` |
| `a distance(d, along: x) b` (`y`) | the run (rise) in the points' view | a, b | the view's `u` (`v`) |
| `a distance(d, along: right) b` (`left`, `up`, `down`) | the same, the sign as a word | a, b | the view's `u`/`v`, ±1 |
| `p distance(d, along: u) P` (`v`) | p's ordinate in P | P.origin, p | P.u (P.v) |
| `p distance(d, along: n) P` | p's height off P | P.origin, p | P's normal |
| `a level(t) b` | (b − a)·t̂ = 0 | a, b | axis or drawn line `t` |
| `a level(up) b` (`down`, `y`) | the same height in the points' view | a, b | the view's `v` |
| `a level(right) b` (`left`, `x`) | the same across | a, b | the view's `u` |
| `a horizontal b` / `a vertical b` | `a level(up) b` / `a level(right) b` | a, b | the view's `v` / `u` |
| `p level(u) P` (`v`) | p is on the line through P.origin along P.v (P.u) | P.origin, p | P.u (P.v) |

At zero a direction and its reverse say the same thing, so `level` takes every page word; the
sign words are there because `up` is how a reader says "the same height".

Refused, each at the key or the word:

* `p level(n) P`: E040 naming `p coincident P`.
* `along: P` or `level(P)` with P a plane: E040. "Along a plane" reads as "within it"; the normal is said
  `along: n` against the plane.
* A page word (`along: x`, `level(up)`, `horizontal`) with the points in different views, or in space: E062, as today
  ("x of which view?"). In space, name the axis: `along: std.x`.
* Both points drawn in one view and t that view's normal: E040, since the row is identically
  zero. This generalises today's `PointPlaneDistance` refusal (`validate`, "a view's own points
  are on it by construction"). For an axis it is refused by value when the axis's direction is
  held and lies square to the view, which the structural count cannot see (cf. #88).
* A literal `0` as an ordinate's number: E040, naming `level`.

**A word or a name.** In `along:` and in `level`'s parentheses the vocabulary (`x y u v n right
left up down`) is read as words. A bare name there that also names an axis or line in scope is refused as ambiguous at the key, so
an axis called `u` inside a component is written by another name; a dotted path (`f.u`,
`std.x`) is always a reference.

**Sign.** The ordinate is signed, as the run and the rise are now: from the first operand to
the second, along t's direction (an axis is directed; a line runs `p1 → p2`). In the plane
forms the plane is the datum, so `p distance(d, along: u) P` is measured from `P.origin` to `p`.
That keeps the meaning every one of today's ~80 uses has.

## The model

### Slots

```
Ordinate: (p: Point, q: Point, along: Along, d: Length, word: Str, form: Int)
Level:    (p: Point, q: Point, along: Along, word: Str, form: Int)   // written a level(t) b
```

* `along` is a new `SpecKind::Along`: an axis, a line or a plane (the plane only through the word
  `n`). It is a real entity slot, so `graft`, the drag part, `remove` and `topology_key` follow
  it. `SpecKind::Direction` is not widened, since `Parallel3`'s "parallel to a plane" would then
  read the normal. `along` is **inferred** (`infers_arg`) when a word is written, filled at
  `io::seed_omitted` (`constraints::infer_entity`) from the word and the points' view, the way
  `Project`'s planes are, so the elaborator, `from_json` and `gcs_constraint_add` share one rule.
* `word` keeps what was written (`x`, `left`, `u`, …, empty for a reference): `side_words` reads
  its sign (right/up +1, left/down −1), `words(slot)` publishes the vocabulary, and the printer
  spells the case back from it. It is never sniffed out of the entities.
* `form` is which kernel reads the row, an inferred `Int` the core fills at `seed_omitted`
  (`infer_value`) and no one writes, as `LineLine3`'s `sign` is. Memberships are fixed at
  elaboration, so it is a structural fact, carried by `topology_key`.
* Three entity slots, as `Symmetric` has; the "one or two entity slots" invariant gains a second
  exception.

The plane forms are rewritten when settled: `p distance(d, along: u) P` settles to
`Ordinate(P.origin, p, P.u, d, "u")`, as `distance l` already settles to its ends in
`Written::assemble`. Every plane has an `origin` point and `u`/`v` axes (`PlaneE`), so the
rewrite always has entities to name.

### Forms and kernels

| `form` | when | columns | row | replaces |
|---|---|---|---|---|
| `PageU` | p, q in view V, t = V.u | p.x p.y q.x q.y | q.x − p.x − d (constant J) | HorizontalDistance |
| `PageV` | p, q in V, t = V.v | the same | q.y − p.y − d | VerticalDistance |
| `InView` | p, q and t's ends in V | p, q, a, b (2D) | (q − p)·(b − a)/‖b − a‖ − d | — |
| `Space` | otherwise, t an axis or line | X, Y, A, B | (Y − X)·(B − A)/‖B − A‖ − d | — |
| `CoordU`/`CoordV` | from a view's own origin, q drawn in it | q.x (q.y) | q.x − d (q.y − d) | CoordinateU/V |
| `FrameU`/`FrameV` | a plane's word `u`/`v`, from its origin, q drawn elsewhere | Y, o, du, dv | (Y − o)·û (v̂) − d | Ordinate3U/V |
| `FrameN` | the word `n`, t the plane | Y, o, du, dv | (Y − o)·n̂ − d | PointPlaneDistance |

As built: the plane's words read its **frame**, over its own columns, as the kinds they replace
did — `v` is the plane's `v̂`, square to `û` within it, which its `v` axis need not be when the
two axes are not square — so an axis written outright (`along: P.v`) is read along itself, in
`Space`, and the word `v` is not, unless the plane holds its axes square, where the two agree.
Reading the frame lifts no origin, and a coordinate from a view's own origin reads the one column
it moves (`CoordU`/`CoordV`, not the run over four columns, two of them the held origin's), so
the structural count — and the hypoid's 118 blocks — is what it was.

* `PageU`/`PageV` are today's run and rise kernels, renamed. They need no axis columns: a point's
  x in V *is* its coordinate along V.u, whatever V's attitude. The best-conditioned row in the
  system stays the one every 2D drawing uses.
* `Space` reads an axis as the segment (0, d) through `Sketch::origin_param` and a line as its
  lifted ends, as `Constraint::axis_columns` already does. X and Y are lifts (`lifted_columns`), so
  a point in space is its own three numbers, and `P.origin` is lifted on request like any view
  point.
* An ordinate reads t's **direction only**, so `place_slots` stays empty: an axis named as a
  direction is never freed or placed (`Sketch::place_axis`).
* Every form gets a free twin (`free_kernel`; `every_dimension_can_be_written_free`) and a
  `jacobians.rs` row. `Level` uses the same kernels with d = 0 and needs no twin.
* `Constraint::kernel()` picks by `form`, as it already picks the magnitude kernel and the free
  twin, and `spatial()` becomes a question about the constraint for these two kinds (`form` ∈
  {`Space`, `FrameU`, `FrameV`, `FrameN`}). The call sites of `CKind::spatial()` that decide lifting and
  validation (`lifted_points`, `validate`, `io.rs`'s inferred tail, `reading::view_points`) ask
  `Constraint::in_space(sk)`; the kind-level ones (declarations, `edit.rs`, solids) are
  unaffected.

## Seam by seam

* **`constraints.rs`**: the two kinds; `ALONG` becomes the word table for `word`, with the plane
  words; `infix_op`'s `distance` arms for (Point, Point) and (Point, Plane) give `Ordinate`, and
  the new `level` word gives `Level`; `horizontal`/`vertical` (Point, Point) are one alias entry,
  `Level` with the word `up`/`right`, read before the operands' kinds as `gauge_op` reads `fix`; `operator()` maps both; `commutative` holds
  for `Level` only. Retire the nine kinds from every exhaustive arm.
* **`syntax`**: `along:` accepts a `Ref` (`OpArg::Named` already carries an `Arg::Ref`);
  `assemble` routes a vocabulary word to `word` and anything else to `along`. The flattener
  rescopes the ref like any operand. `level`'s direction is the unlabelled third operand
  (`OpArg::Ent`, as `symmetry(l)`), and a vocabulary word written there goes to `word`. `level`
  joins `OPERATORS` and the highlighter's word list.
* **`program/reading.rs`**: the `CoordinateU/V → Ordinate3U/V` branch goes away. An `Ordinate` is
  the same kind in or out of a view; only `form` differs. A page word across views stays E062.
* **`program/views.rs`** (`place`): today's rule, "a point measured in a plane is placed by the
  plane, not the plane by it", applies to an `Ordinate`/`Level` whose `p` is a plane's origin
  (`plane_of_origin`); the measured point is `q`.
* **`validate`**: the identically-zero refusal, generalised; the forms in space need every point
  to have a place in space, as the existing "a point of a 2D sketch" check says.
* **`callout.rs`**: `PageU`/`PageV` are `axis_distance` as now; `InView` is `Pen::linear` along
  the line's direction in the view; the forms in space have no callout (no relation in space has
  one), so `frame` returns `None` for them. `pair_dimension` returns `Distance` or `Ordinate`
  with the word `x`/`y`, and `gcs_dimension_pair_kind` returns the word beside the registry index.
  `every_dimension_is_drawn` needs an in-view instance.
* **`cgraph.rs`**: `Level` with `form` `PageU`/`PageV` takes `HorizontalPoints`'s arm (a virtual
  line in the x-axis class). `Level` along a drawn line in the view can be a virtual line at π/2
  to it; that is optional and goes last. `Ordinate` stays `unsupported`, as the run and the rise
  are now, on purpose.
* **Printer** (`syntax::operator_text`, `print.rs`'s `named_along`): spelled from `word`. `u`/`v`/`n`
  with `p` a plane's origin print the plane form; `x`/`y`/`right`/… print the word; a reference
  prints `along: <name>`; `Level` prints `level(t)` or `level(<word>)`. Which of an alias and
  its expansion was written is not recoverable from the slots, so the printer writes
  `horizontal`/`vertical` for `Level` with `up`/`down`/`y` and `right`/`left`/`x`. The source is
  never reprinted, so this shows only in `describe` lines and appended statements, where the
  alias is what the app's buttons mean.
  `describe`, `reconcile`'s appends and `lift::to_program` all go through it.
* **JSON**: wire names `ordinate` and `level`, `along` an entity index, `form` never written
  (inferred on read). The old names are not read (no compatibility).
* **FFI and binding**: the registry publishes the two kinds and their words; the generated
  classes follow. `web/src/core/constraints.ts` loses the four exported constructors it names,
  `commands.ts`'s Horizontal/Vertical buttons build `Level` with the word, and `pairDim` builds
  `Ordinate(p, q, null, v, 'x')`, the core inferring `along` from the word and the view.

## Phases

Each phase lands green (`make test` once, captured) with its own gate.

1. **The general statement.** `Ordinate` and `Level` with a reference direction only, forms
   `InView` and `Space`, beside the existing kinds. Parser, settle, kernels and twins, `validate`,
   printer, JSON. Gate: a new `tests/ordinate.rs` (listed in `tests/main.rs`): ordinates along
   `std.x/y/z`, a turned axis, a drawn line in the view and one in another view, against closed
   forms; `level` across views; each refusal at its span; DOF counts.
2. **The plane forms fold in.** `along: u|v|n` against a plane settle to `Ordinate`/`Level` from
   `P.origin`; forms `CoordU`/`CoordV`/`FrameU`/`FrameV`/`FrameN`; `views::place` generalised. Retire `CoordinateU/V`,
   `Ordinate3U/V`, `PointPlaneDistance` and their kernels. Gates: `tests/coordinates.rs`,
   `spatial_lang.rs`, `planes`/`axis` suites, the V-twin and the spiral bevel unchanged
   (`hypoid_layout.rs`).
3. **The page forms fold in.** `along: x|y|right|…` and `horizontal`/`vertical` between points.
   Retire `HorizontalDistance`, `VerticalDistance`, `HorizontalPoints`, `VerticalPoints`;
   `cgraph`, `callout`, `pair_dimension`, the FFI, binding and app. Hand-built tests move onto a
   plane. Gates: `callout.rs`, `decompose.rs`, `chain.rs`, `io.rs`, `jacobians.rs`, `npm test`,
   and `make bench` against the baseline (the page kernels are unchanged; the drag path must not
   move).
4. **The corpus and the literal zero.** E040 on `distance(0…, along: …)`; rewrite the 64 zero
   ordinates in `rust/examples` as `level`, `coincident P.origin` or `horizontal`/`vertical`, as
   reads best (`wankel.sv`: `centre coincident std.top.origin`); `twist_drill/wheel.sv`'s sin/cos
   pair as one ordinate along a turned axis and a `level` across it. Gate: `examples_sv.rs`, every
   example's report unchanged.
5. **Docs.** Spec to Draft 0.46: §9.2's `distance` row and the [0.21]/[0.24] ordinate paragraphs
   restated as one statement, `level` added; the primer's table (§1.5 and the `distance` row);
   CLAUDE.md's `along: n` and `ALONG` lines and the points-in-space paragraph's `Ordinate3U`.
   Optionally, last: `cgraph` decomposing `Level` along a drawn line.

## Open

* **Relation aliases in the library** (#94). `horizontal` belongs in `std.sv` as `a horizontal b
  := a level(up) b`, but the language can define components, not relation words. Two things are
  needed: a definition form for an infix or prefix word over its operands, expanded by the
  flattener like a component body; and a `use` that imports individual names
  (`use std (horizontal)`), not an exemption from §14.4's full-path rule. Out of scope here. The
  alias is one entry, `constraints::level_alias`, so that it is the only thing that moves.
* `InView` along a drawn line whose ends are p or q (`a level(l) b` with `l :=
  line(a, b)`) forces a collapse. It is legal and silly, and is not refused.
