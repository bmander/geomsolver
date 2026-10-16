# Frames: planes and motions as one concept: plan

**Status (2026-10-16): proposed.** Nothing here is built. Decisions 1 and 2 were made in
conversation. The rest are proposals, and [Open questions](#open-questions) lists the ones that
need an answer before the stage that depends on them.

Solvent has two concepts that are one concept seen twice.

* **A plane is a frame.** `plane(u: r1, v: r2)` has an origin and three axes (right `u`, out
  `u × v`, up `out × u`; `docs/contract/space.md`). Where it stands is three unknowns of the
  solve. `in P` writes a point in its coordinates.
* **A motion is a frame that moves.** `motion(about: ax, ratio:, phase:, advance:)` is a rigid
  pose over the one shared roll (§6.14). `motion(m, relative_to: n)` changes the observer,
  `n⁻¹·m`. `X coincident(under: m, t == τ) x` takes `x` in the moving frame to the world frame
  at the roll τ. An envelope is a tool fixed in one frame, traced in another.

The language keeps these apart, so three things are harder to say than they should be:

* **Which frame a thing rides on** is said at each reading (`under: m`), never where the thing is
  declared. In a carried point, argument order decides which point moves.
* **A frame cannot ride on a frame.** The axis of a motion is drawn geometry at rest, so a planet
  on a carrier arm, or a tool on a slide on a turntable, has no direct spelling. It can be
  written today, but only through a trick (see [Chains already work](#chains-already-work)).
* **Drawing in a moving part** means drawing in a static view and remembering to say `under:`
  everywhere the part is read.

This plan makes these changes:

* **one entity kind, the frame.** A plane is a frame whose `in` draws flat. A motion is a frame
  with a *joint*. A relative motion is a frame derived from two others.
* **`in F` attaches.** A declaration in a frame rides on it. Whether it is also flat is a
  property of the frame: the std views and `plane(…)` are planar, `frame(…)` is spatial.
* **frames nest.** `in` blocks nest, a frame declared inside a frame rides on it, and the pose of
  a nested frame is the product of its joints.
* **readings name an observer, not a motion.** `seen_from: F` replaces `relative_to:`, and a tool
  that rides on a frame needs no `under:`.

The motion arithmetic stays as it is: one shared roll, screw joints driven at a constant ratio,
exact poses and derivatives. Every consumer that restricts which motions it accepts (planar
envelopes, held envelopes in space, the generating classes, the canal) keeps its restriction and
refuses by name, as it does today.

## Decisions

1. **Views and motions are consolidated under frames** (2026-10-16).
2. **`in F` means attached to F; being flat is the frame's property** (2026-10-16, option (b) of
   the conversation). The std views `front`, `top`, `side` and `up`, and every `plane(u:, v:)`,
   are planar: a point `in` one has two coordinates, as today, and every existing `in` keeps its
   meaning. A `frame(u:, v:)` is spatial: a point in it has three coordinates, along `u`, up and
   out. A jointed frame with no axes of its own gives coordinates in the frame it was declared
   in.

### Proposed

3. **There is no compatibility.** This follows the precedent of `docs/planes-plan.md`: nothing is
   deployed, so an old spelling stops parsing at the stage that replaces it. The corpus is
   rewritten in the same change, and no diagnostic names an old spelling beyond the one refusal
   that points to the new one.
4. **Everything is drawn at rest.** A jointed frame at joint value 0 (the identity pose, *not*
   roll 0, which reads `phase:`) stands exactly on its parent. So geometry declared in it is
   drawn where its numbers put it, and **a relation that names no roll reads the drawing at
   rest**. This is what every relation does today, so "a motion never moves the sketch" becomes a
   consequence rather than a rule.
5. **Joint axes are stated at rest, and a pose is a product of exponentials.** A frame nested in
   jointed frames `J₁ … Jₖ` (outermost first) has the pose `J₁(τ)·J₂(τ)·…·Jₖ(τ)`, each `Jᵢ` a
   screw about its axis *where that axis is drawn*. This is Brockett's product-of-exponentials
   form. No joint axis has to be moved by hand, which is what makes nesting safe to write.
6. **`under: F` stays, as the explicit form.** "Read this as if it rode on F" is still needed
   when one drawn thing is carried by several motions: the indexed pattern's one tool at six
   rolls, `geodesic_circles.sv`'s static point turned by a sampled formal, the Wankel housing
   that is both the housing and a sweep seen from the rotor. A reading whose operand rides on a
   frame *and* names `under:` is refused, to begin with (E040, naming both).
7. **The word is `frame`.** It has been reserved since 0.42, which retired 0.15's `frame` into
   `plane` (`syntax/parser/statements.rs` refuses it, twice), so nothing in the corpus uses it.
   That refusal is removed. `plane` stays as the word for a planar frame, because a plane is also
   a set the language relates (`P coincident p`, `P distance(d) Q`, `P.normal`), and a spatial
   frame is not.
8. **Drawing views in `.svd` are untouched.** A `.svd` `view` is a camera onto model frames, not
   a frame. CLAUDE.md's first rule stands: presentation stays out of model source.

## The model

### One kind

`EntKind::Plane` becomes `EntKind::Frame`. Ids are permanent (`gcs-ffi/src/entities.rs`,
`kind_id`), so it keeps id 6. `EntKind::Motion` is retired and id 10 is taken up, as the
ellipse's and the old frame's were.

```rust
pub struct FrameE {
    /// The frame it was declared in: the world's where none (`None`), else the innermost frame of
    /// the `in` blocks around it.  Its own carrier chain is its parent's plus its own joint.
    pub parent: Option<u32>,
    /// Axes of its own: `plane(u:, v:)` and `frame(u:, v:)` have them, a jointed frame does not
    /// and gives coordinates in its parent's.  Today's `PlaneE`, with `planar` saying whether a
    /// point `in` it has two coordinates or three.
    pub chart: Option<Chart>,
    /// A screw about a line or a point of a view (today's `MotionDef::Rotation`, `Turn`,
    /// `Translation`, with their `measured` numbers), or a frame derived from two others
    /// (`Relative`, `seen_from:`).
    pub joint: Option<Joint>,
    pub name: String,
    pub class: Classes,
}
pub struct Chart { pub u: u32, pub v: u32, pub o: [u32; 3], pub origin: u32,
                   pub normal: Option<u32>, pub planar: bool }
```

`Sketch::planes` and `Sketch::motions` become `Sketch::frames`. A frame with a chart and a joint
together (a moving plane with axes of its own) is representable, but not written in the first
stages: nest a `plane(…)` in a jointed frame instead.

### A point's place

`PointE` has `x`, `y`, an optional `z`, and an optional `plane` (the chart its numbers are in).
It gains nothing new: its `plane` becomes its *frame*. The chart is that frame's own axes, or
those of the nearest ancestor that has axes, or the world's. The carrier chain is the jointed
frames on its frame's path.

* `Sketch::world_point` stays the point **at rest**: the chart's lift, which is what it is now.
  Joints never enter it, so the solve, the diagnosis, the decomposition and the DOF ledger see
  nothing new.
* A point in a spatial chart has `z: Some`, read along the chart's `out`, not the world's `z`.
  `world_in` and `on_view_sheet` learn the third coordinate. `Basis::lift` takes three numbers.
* A line, circle, arc, spline or face rides on a frame when its points do. Mixed carriers are
  refused **where a carrier is read** (a sweep's tool, an envelope's tool, a carried point's
  operand), never at declaration, so a line from a riding point to a static one stays a line.

### Joints are read after the solve, as now

Motions are built after the primitives (`program/motions.rs`), because their axes are solved
geometry. That order stays: a frame is *declared* with the primitives (so declarations can name
it as their frame), and its joint is *resolved* in the motion pass, reading the solved axis as
`Family::read` does now. A forward reference from a joint to geometry declared in the frame
itself (`planet := frame(about: c)` with `c` in `planet`) is a cycle and is refused.

### Chains already work

Today `motion(planet, relative_to: motion(about: o, ratio: -1))` is `arm · planet`. Inverting
the outer turn by negating its ratio makes `relative_to:`'s `n⁻¹·m` into a product. A planet's
point carried that way, at roll 60°, solves to the closed form `R(τ)·(c + R(−3τ)(x − c))` =
(10, 17.3205) for `o` = (0, 0), `c` = (30, 0), `x` = (40, 0).

So **a chain lowers to existing steps**, with no new evaluation code:

* `A·B` is `Relative { source: B, observer: inv(A) }`;
* the inverse of a screw negates its ratio, phase and advance (a measured number is negated where
  it is read);
* the inverse of a relative swaps its source and observer.

Every consumer that reads a `Relative` reads a chain on the first day:

* `Family::at`, with the derivative of the moving inverse;
* `Family::in_frame` and its planar steps;
* `envelope::held::Moving::of_any`;
* `MotionBounds`, the roll search and the carried-point kernel's columns.

A dedicated `Step::Chain` is an optimisation (one product rather than an inverse of an inverse)
and comes later, if a profile asks for it.

### What consumers see

Consumers that match on step shapes go on matching the same shapes:

* a single rotation is a screw (`Family::screw`) or a turning (`Family::turning`);
* a plane is kept when every step keeps it (`Step::keeps`);
* the planar emission (`Family::in_frame`), and the generating classes of `solid::admission` and
  `solid::constant_twist`.

A chain is more steps, so those matchers refuse it by name until a stage teaches them otherwise.
One cheap canonicalisation is worth doing in the motion pass: **coaxial turns compose to one
turn** (ratios add, phases add). That is the common gear-train case, and it then reads as the
single rotation every consumer already admits.

## The language

### Frames

```sv
P := plane(u: r1, v: r2)                     // planar: `in P` draws flat, as today
F := frame(u: r1, v: r2)                     // spatial: a point in F has (x, y, z) along u, up, out
spin := frame(about: shaft, ratio: 2, phase: 10deg)   // a turn about a line; with advance:, a screw
turn := frame(about: o)                      // a turn in o's view
slide := frame(along: rail, advance: 20mm)   // a slide, by advance a turn
seen := frame(crown, seen_from: blank)       // derived: crown as blank sees it, blank⁻¹·crown
```

The numbers of a joint are today's: `ratio:` a Scalar (default 1), `phase:` an Angle (default
0deg), `advance:` a Length a turn (default 0). A measurement of the drawing may stand in each, as
it may now (§6.14; E107 elsewhere).

### `in` attaches, and nests

```sv
in arm {                       // everything here rides on arm
  in std.top {                 // ...and is drawn flat in top
    c := point
    fix((30, 0)) c             // relations read at rest
  }
  planet := frame(about: c, ratio: -3)   // declared in arm: arm is its parent
}
```

* `in F { … }` and `p := point in F` set the declaration's frame. Nested blocks compose: the
  chart comes from the innermost frame that has axes, the carrier chain from every jointed frame
  on the path.
* `in A { in B { … } }` and `in B { in A { … } }` mean the same thing when only one has a joint.
  The printer writes the jointed frame outermost.
* Two static charts nested is meaningless (the inner one wins, and the outer says nothing), so it
  is warned about: W, naming the outer one.
* Nesting inside a root `repeat` or `when` stays as it is today (the clause written per
  declaration, `statements.rs`'s `in_block`). What is relaxed is nesting `in` inside `in`.

### Readings across frames

| today | with frames |
|---|---|
| `X coincident(under: m, t == τ) x` | `X coincident(t == τ) x` when `x` rides on `m` and `X` does not. The pose is `path(X)⁻¹·path(x)` at τ. `under: m` remains for an operand that rides on nothing (decision 6). |
| `envelope(tool, under: m, from:, to:)` | `envelope(tool, from:, to:)` when the tool rides, read in the world, or `envelope(tool, seen_from: G, from:, to:)`. **The envelope rides on its observer**: the profile a rack cuts rides on the blank it is cut in, at rest. |
| `solid(tool, under: m, at: θ)` | `solid(tool, at: θ)` when the tool rides. `under:` stays for one tool placed by a frame it is not declared in (the indexed pattern). |
| `solid(tool, under: m, from:, to:)` and `through: body` | the same, with `seen_from: G` for a generating cut. The swept solid rides on its observer. |
| `claim … over m` | `claim … over F`: the placements under it advance (`Sweep::motion`) |
| `rate(X over t)`, `std.Screw`, `reciprocal` | unchanged: they read the roll slot of the contact holding `X` |
| `meshing(f1, f2, under: mesh, …)` | `meshing(f1, f2, …)` with each flank riding its member. `under:` stays as a derived frame. |
| component formal `m: motion` | `m: frame`. A component asking for a jointed frame is refused one without a joint where the joint is read. |

**The roll is still one shared number.** A relation naming `t ==` crosses frames at that roll. A
relation that names none reads the drawing at rest (decision 4), whatever frames its operands
ride. So `p coincident q`, with `p` on the arm and `q` static, holds where both are drawn, as
today.

### Before and after

The rack that cuts an involute (primer §2.11), today:

```sv
blank := motion(about: o, ratio: 1)
rack := motion(along: slide, advance: 2 * pi * 30)
cutting := motion(rack, relative_to: blank)
f0 := point hint((30, 5))
f1 := point hint((39.4, 8.4))
flank := line(f0, f1)                           // drawn static, carried at every reading
cut := envelope(flank, under: cutting, from: -25deg, to: 25deg)
```

With frames:

```sv
blank := frame(about: o, ratio: 1)
rack := frame(along: slide, advance: 2 * pi * 30)
in rack {
  in std.front {
    f0 := point hint((30, 5))
    f1 := point hint((39.4, 8.4))
    flank := line(f0, f1)                       // the rack's flank, riding the rack
  }
}
cut := envelope(flank, seen_from: blank, from: -25deg, to: 25deg)   // rides on blank
```

A planet on an arm, which today needs the inverted-turn trick:

```sv
arm := frame(about: o)
in arm {
  in std.top {
    c := point
    fix((30, 0)) c
  }
  planet := frame(about: c, ratio: -3)
}
in planet {
  in std.top {
    x := point
    fix((40, 0)) x
  }
}
X coincident(t == 60deg) x            // (10, 17.3205): R(τ)·(c + R(−3τ)(x − c))
```

## Stages

Each stage lands green: `make test` passes, and `tests/examples_sv.rs` holds every document to
the numbers it advertised before the stage. A corpus rewrite must not move a solved value
(compare `--where` output across the rewrite, to 1e-12).

### Stage 1: chains in the core, with no syntax

* `MotionDef::Chain { outer, inner }`, lowered in the motion pass to a `Relative` over a minted
  inverse ([Chains already work](#chains-already-work)). Coaxial turns are folded to one turn.
* Height and cycle checks count the minted nodes (`Family::read`'s 64 levels and
  `program/motions.rs`'s `heights`).
* A test file `tests/frames.rs`, listed in `tests/main.rs`:
  * a planet's carried point against the closed form at several rolls;
  * a chain of three turns against the hand product;
  * a turn on a slide against a cycloid;
  * coaxial folding, which leaves one rotation that `Family::screw` reads.
* `tests/jacobians.rs`: a `CarriedPoint` row under a chain, FD-checked over both axes' columns.
* Built from Rust, not text: there is no spelling yet, and the words come in stages 3 and 4.

### Stage 2: one kind

* `EntKind::Plane` → `EntKind::Frame` (id 6), `PlaneE` → `FrameE` with `chart` and
  `planar: true`. Every arm of the exhaustive matches CLAUDE.md lists is renamed, with no
  behaviour change.
* `frame(u:, v:)` parses: the `frame` refusals go, and a spatial chart gives a point three
  coordinates.
  * `Basis::lift` takes three numbers, and `world_in` / `on_view_sheet` take a third coordinate.
  * `hint(at: F, (x, y, z))` seeds in a spatial frame.
  * The ordinates `along: u | v | n` read a spatial frame's axes.
  * A plane relation over a spatial frame (`F coincident p`, `F distance(d) Q`) is refused,
    because a spatial frame is not a set. `F.uv`, its `u`–`v` plane, is left as an open question.
* The binding's `model.ts` `KIND_ID` table and the app's plane pickers read `Frame`, with the
  same ids.
* Gate: `tests/frames.rs` adds a point in a spatial frame lifting to its chart, and a spatial
  frame placed by relations like a plane (7 DOF free).
* Spec §6.7 gets a paragraph and the changelog an entry.

### Stage 3: joints fold in

* `MotionE` → `FrameE.joint`. `EntKind::Motion` is retired and id 10 taken up.
* `frame(about:…)`, `frame(along:…)` and `frame(m, seen_from: n)` parse. `motion(` and
  `relative_to:` stop parsing, each with one refusal naming the new spelling.
* `under:` takes a frame, and the formal type `motion` becomes `frame`.
* The FFI's `gcs_motion_sample` becomes `gcs_frame_sample`, taking a frame index and refusing one
  with no joint. `web/src/core/motion.ts` becomes `frame.ts`.
* The corpus is rewritten mechanically: 26 `.sv` files carry 88 `motion(` lines, 19 files carry
  28 `relative_to:` lines, and 32 core test files build motions from text.
  `docs/contract/generating.md` and the primer's §1.15 are reworded, the spec's §6.14 is
  rewritten, and the changelog gets an entry.
* Gate: no solved value moves (`--where` across the rewrite). Each `tests/carried.rs`,
  `generation.rs`, `roll_through.rs` and `wankel.rs` case passes unchanged except for its
  spelling.

### Stage 4: riding

* The parser lets `in` nest inside `in`:
  * `stamp_plane` becomes `stamp_frame`, stamping the path;
  * `InBlock` records nested headers, so `edit::remove` can splice an inner or outer block;
  * deleting a frame splices its blocks as deleting a plane does (`docs/contract/space.md`).
* A declaration's frame is the innermost block's, and a frame declared in a block takes it as its
  parent.
* Carriers are inferred where a carrier is read: carried points, envelopes, swept and placed
  solids, meshing flanks, `claim over`.
  * `seen_from:` on a reading.
  * Envelopes and swept solids ride on their observer.
  * Mixed carriers are refused at the reading (E040, naming the two frames).
* A relation with no roll reads at rest, which is today's code path. The new rows are only the
  readings that name a roll and cross frames, and those lower to `CarriedPoint` (or an
  envelope's motion) over the relative frame `path(X)⁻¹·path(x)`.
* The app:
  * drawing in a jointed frame draws at rest;
  * the workspace offers a frame's chart as a place to draw, as it offers planes now;
  * a membership edit on a block-stamped declaration is refused with the cause, as now (`app.md`);
  * moving a declaration between frames is a later gesture.
* Gate (`tests/frames.rs`):
  * the planet and rack examples above, against their closed forms;
  * a swept solid riding its observer, against the same body built with `under:`;
  * the refusals.
* The language contract gets a section on frames and riding (`docs/contract/language.md`, or a
  new `docs/contract/frames.md` with a row in CLAUDE.md).

### Stage 5: documents ride where it reads better

The generation examples, the Wankel and the spiral bevel are rewritten to declare their tools in
the frames that carry them, one document at a time, each holding its numbers. Where `under:`
reads better (one tool, several frames) it stays. This stage is editorial and may stop
whenever the remaining documents read well as they are.

### Later, and not in this plan

* **General drives:** a joint's value as an expression of the roll (a cam law, a varying ratio),
  differentiated by `taylor::Jet`, with bounds on its speed by interval arithmetic so that
  `MotionBounds` and the roll search stay conservative. Spec §6.14 already says the families
  "do not yet specify arbitrary motion laws".
* **Several rolls:** independent rolls for carried points and placements only. Envelopes and
  sweeps keep one drive, since an envelope of a two-parameter family is a different object.
* **Station frames:** a loft station's frame (`std.On`, `std.Minimal`, `std.Frenet`) is a frame
  driven by a path parameter, not by the roll. It could be one later, with several rolls.
* **`Step::Chain`**, if lowering chains through inverses shows in a profile.
* **An assembly-joint solver** (§6.14's other "not yet"): poses as unknowns of the solve. Frames
  here have no state, and every reading names its roll.

## Open questions

1. **`seen_from:`** or another word for the observer (`in:` collides with membership,
   `relative_to:` is retired with `motion`). Needed by stage 3.
2. **`F.uv`**: does a spatial frame offer its three coordinate planes as members, so that
   `in F.uv` draws flat in a spatial frame? Needed by stage 2, or deferred.
3. **`under:` on an operand that already rides**: refuse (proposed), or compose `under:` outermost?
   Needed by stage 4.
4. **Nesting order**: accept both orders of a jointed and a static block (proposed), or require
   the jointed one outermost? Needed by stage 4.
5. **A riding envelope's or swept solid's frame**: the observer, at rest (proposed), so that a cut
   gear profile rides on the gear. The alternative is the world. Needed by stage 4.
