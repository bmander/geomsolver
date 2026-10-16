# Frames: one declaration for planes and motions, constrained after the fact: plan

**Status (2026-10-16): in progress**, issue #383. Stage 1 is built (`Sketch::chain`,
`tests/frames.rs`), stage 2 (`EntKind::Frame`, `f := frame`, spec 0.134) and stage 3 (joints as
members, `std.Turn`/`Spin`/`Slide`, `seen_from:`, spec 0.135). Two departures in stage 3: the
lowered motion graph (`Sketch::motions`, `EntKind::Motion` inside the core) stays as what a
joint and a reading lower to, no longer a declaration or an ABI kind; and a measured ratio is
refused already (E107) rather than carried into `fix`, the documents that measured one
stating it from their own numbers. The corpus holds its solved positions to 1e-12 (`hypoid_drive`,
which imports the hypoid pair, to 7e-11); a swept solid's meshed measures move by up to 1e-3,
since its material field's mesh follows the solve's last bits, which the joints' axis unknowns
change — the pre-rewrite binary moves them as much for one unrelated axis added. This version replaces the
first one, which kept a menagerie of constructor labels on `frame(…)`. Decisions 1–3 were made in
conversation. The rest are proposals, and [Open questions](#open-questions) lists what needs an
answer before the stage that depends on it.

## Why

Solvent has two concepts that are one concept seen twice.

* **A plane is a frame.** `plane(u: r1, v: r2)` has an origin and three axes (right `u`, out
  `u × v`, up `out × u`; `docs/contract/space.md`). Where it stands is three unknowns of the
  solve, and `in P` writes a point in its coordinates.
* **A motion is a frame that moves.** `motion(about: ax, ratio:, phase:, advance:)` is a rigid
  pose over the one shared roll (§6.14). `motion(m, relative_to: n)` changes the observer,
  `n⁻¹·m`. `X coincident(under: m, t == τ) x` takes `x` in the moving frame to the world frame
  at the roll τ, and an envelope is a tool fixed in one frame, traced in another.

The language keeps these apart, so three things are harder to say than they should be. Which
frame a thing rides on is said at each reading (`under: m`), never where the thing is declared.
A frame cannot ride on a frame (a planet on an arm has no direct spelling). And between them
the two take nine constructor labels (`u:`, `v:`, `about:`, `along:`, `ratio:`, `phase:`,
`advance:`, `relative_to:`, `of:`), each a new word for something the language can already say
with relations.

The language's own principle is the way out: **an entity is declared bare and constrained after
the fact by relation words**, not built by an ever-growing set of constructor words. A plane
already works this way (`p := plane` is 7 DOF, placed by relations). So does a circle (`radius(r)
circle(center: c)`), and so does `std.Screw` (`rust/lib/std.sv`). That component reads a
motion's instantaneous screw axis from nothing but the rates of two points the motion carries.
Today it solves for the axis given the motion. With the motion's numbers as unknowns, the same
component given the axis pins the motion.

This plan makes these changes:

* **One declaration, `frame`.** A plane is a frame whose `in` draws flat. A frame's motion is
  three *members*: an axis and two numbers. They are relation-held like anything else, and in
  the end solved like anything else.
* **`in F` attaches.** A declaration in a frame rides on it. Frames nest, and a nested frame's
  pose is the product of its joints.
* **The constructor labels move into the standard library.** `std.Turn`, `std.Spin` and
  `std.Slide` are components over the members, written in Solvent, so the parser keeps no
  motion words.
* **Readings name an observer.** `seen_from: F` stands on a reading, and a tool that rides on a
  frame needs no `under:`.

## Decisions

1. **Views and motions are consolidated under frames** (2026-10-16).
2. **`in F` means attached to F, and being flat is the frame's property** (2026-10-16). The std
   views `front`, `top`, `side` and `up`, and every `plane`, are planar: a point `in` one has
   two coordinates, so every existing `in` keeps its meaning. A `frame` is spatial: a point in
   one has three coordinates, along its `u`, up and out.
3. **A frame is declared bare and constrained after the fact** (2026-10-16): `in (f := frame) {
   … }`, then relations on `f`. A frame's motion is unknowns held by relations.

### Proposed

4. **The joint is an axis and two numbers.** Every jointed frame turns about `f.axis` at
   `f.ratio` radians a radian of the roll, and advances `f.advance` along it a turn. This is
   the *twist* in the coordinates a designer states (see [The joint](#the-joint)).
5. **Roll 0 is as drawn, and `phase:` goes.** With `ratio ≠ 0` a phase φ is a shift of the
   roll: the pose at φ + ρτ is the pose at ρ(τ + φ/ρ). So every phase in the corpus is rewritten
   as an offset at the reading that names the roll. A member that must stand turned at roll 0
   is drawn turned. **A relation that names no roll reads the drawing as drawn**, which is what
   every relation does today.
6. **A frame owns only what the document can observe.** This follows the precedent of
   `P.normal`, which exists "where the document reads one" (`PlaneE::normal`).
   * Its axes (`f.u`, `f.v`, `f.origin`) are minted when named. Until then it draws in its
     parent's coordinates.
   * Its joint is minted when a reading crosses it at a roll, or when `f.axis`, `f.ratio` or
     `f.advance` is named.

   A frame nothing reads at a roll has a motion no relation could detect, so omitting the joint
   is not a hidden constraint. A joint that is minted but not held is freedom, reported by the
   diagnosis like any other.
7. **Joint axes are stated at rest, and a pose is a product of exponentials.** A frame nested in
   jointed frames `J₁ … Jₖ` (outermost first) has the pose `J₁(τ)·…·Jₖ(τ)`, each `Jᵢ` about its
   axis *where that axis is drawn* (Brockett's form). No axis is moved by hand.
8. **The observer stays structural.** Two constant twists seen one from the other are not a
   constant twist: the pitch point of two meshing gears runs round a circle in either gear's
   frame. So "seen from" cannot be a relation on one frame's members. It is nesting (`in`) and
   `seen_from:` on a reading.
9. **`under: F` stays, as the explicit form.** It is needed for one drawn thing carried by
   several frames: the indexed pattern's one tool at six rolls, `geodesic_circles.sv`'s static
   point turned by a sampled formal, the Wankel housing that is both the housing and a sweep seen
   from the rotor.
10. **There is no compatibility.** Following `docs/planes-plan.md`'s precedent, an old spelling
    stops parsing at the stage that replaces it. The corpus is rewritten in the same change, with
    one refusal per retired spelling pointing to the new one.
11. **The words are `frame` and `plane`.** `frame` has been reserved since 0.42, which folded
    0.15's `frame` into `plane` (`syntax/parser/statements.rs` refuses it, twice), so nothing in
    the corpus uses it. `plane` stays as the flat frame, because a plane is also a set the
    language relates (`P coincident p`, `P distance(d) Q`), and a spatial frame is not.
12. **Drawing views in `.svd` are untouched.** A `.svd` `view` is a camera onto model frames.
    Presentation stays out of model source (CLAUDE.md's first rule).

## The model

### One kind

`EntKind::Plane` becomes `EntKind::Frame`. Ids are permanent (`gcs-ffi/src/entities.rs`,
`kind_id`), so it keeps id 6. `EntKind::Motion` is retired and id 10 is taken up, as the
ellipse's and the old frame's were.

```rust
pub struct FrameE {
    /// The frame it was declared in: the world's where none, else the innermost frame of the `in`
    /// blocks around it.
    pub parent: Option<u32>,
    /// Axes of its own, minted when `f.u`, `f.v` or `f.origin` is named (a plane's always are):
    /// today's `PlaneE`, with `planar` saying whether a point `in` it has two coordinates or three.
    pub chart: Option<Chart>,
    /// Its motion relative to its parent at rest, minted when read (decision 6).
    pub joint: Option<Joint>,
    pub name: String,
    pub class: Classes,
}
pub struct Chart { pub u: u32, pub v: u32, pub o: [u32; 3], pub origin: u32,
                   pub normal: Option<u32>, pub planar: bool }
/// `f.axis`, an `AxisE` (a place and a direction, 4 DOF); `f.ratio`, a Scalar; `f.advance`, a
/// Length a turn: six numbers, the twist.
pub struct Joint { pub axis: u32, pub ratio: u32, pub advance: u32 }
```

`Sketch::planes` and `Sketch::motions` become `Sketch::frames`. `ratio` and `advance` are
`Param`s, which stage 5 makes into unknowns that kernels differentiate. Until then they must be
stated.

### The joint

A joint is a constant twist relative to the frame's parent at rest. By Chasles' theorem every
constant twist is a screw motion, so one family covers every motion Solvent has. With `d̂` the
axis's direction and `q` its foot:

* `ω = ratio · d̂`
* `v = −ω × q + (advance / 2π) · d̂`

* **A turn** is `advance == 0`. **A screw** is both numbers nonzero.
* **A slide** is `ratio == 0`. Then `v = (advance / 2π) · d̂`, a translation along the axis's
  direction, and the axis's foot is read by nothing. A slide must hold its foot (a slide along a
  rail holds the axis on the rail), or the diagnosis reports two free numbers.

This parametrisation is today's `MotionDef::Rotation` (`axis`, `ratio`, `advance`) with the
numbers made members, which buys three things over a bare six-vector. A slide is not a
singularity of the coordinates. The designer states what the shop states (an axis, a ratio, a
lead). And `f.axis` is an existing kind, with existing relations (`parallel`, `perpendicular`,
`p coincident r`, `fix(dir ==, origin ==)`) and existing settlement before the solve
(`views::place`).

### A point's place

`PointE` keeps `x`, `y`, an optional `z` and an optional `plane`, which becomes its *frame*. The
chart is that frame's own, or its nearest ancestor's with axes, or the world's. The carrier chain
is the jointed frames on its path.

* `Sketch::world_point` stays the point **as drawn**, the chart's lift. Joints never enter it,
  so the solve, the diagnosis and the DOF ledger see no new rows for geometry at rest.
* A point in a spatial chart has `z: Some`, read along the chart's out, not the world's `z`.
  `Basis::lift`, `world_in` and `on_view_sheet` take the third coordinate.
* A line, circle, arc, spline or face rides when its points ride. Mixed carriers are refused
  **where a carrier is read** (a sweep's tool, an envelope's tool, a carried point's operand),
  never at declaration.

### Chains already work

Today `motion(planet, relative_to: motion(about: o, ratio: -1))` is `arm · planet`, because
negating the outer turn's ratio makes `relative_to:`'s `n⁻¹·m` a product. A planet's point
carried that way at roll 60° solves to the closed form `R(τ)·(c + R(−3τ)(x − c))` =
(10, 17.3205), for `o` = (0, 0), `c` = (30, 0), `x` = (40, 0) (checked with `solventc`).

So **a chain lowers to existing steps**, with no new evaluation code:

* `A·B` is `Relative { source: B, observer: inv(A) }`;
* the inverse of a joint negates its ratio and advance (and, until stage 3 retires it, its phase);
* the inverse of a relative swaps its two sides.

Every reader of a `Relative` reads a chain on the first day: `Family::at` (with the derivative of
the moving inverse), `Family::in_frame`, `envelope::held::Moving::of_any`, `MotionBounds` and the
roll search. A dedicated product step is an optimisation for later. **Coaxial joints fold to one
joint** (ratios add, advances add), which is the common gear-train case, so it then reads as the
single rotation that `Family::screw`, `Family::turning` and the generating classes admit.

### What consumers need to know before the solve

Some consumers decide a motion's *kind* when the drawing is built:

* a planar envelope needs a turn square to its view (`Step::keeps`, `MotionDef::Turn`);
* the canal needs one turn about one axis;
* the generating classes of `solid::admission` and `solid::constant_twist` need their shapes.

Today they read it off `MotionDef`. With members held by relations, they read it off **the
relations that hold the joint**, as `views::origins_on_axes` reads which axes are wholly held
before the solve. A joint whose axis is held parallel to a plane's normal, with its advance held
at 0, keeps that plane. A joint whose relations leave its kind open is refused by every consumer
that needs one, naming what is missing. Consumers that read values (solids, built after the
solve) and consumers that are rows (carried points, rates, held envelopes) need no kind.

## The language

### Declaring and constraining

```sv
f := frame                          // a spatial frame; `in f` gives three coordinates
in (g := frame) { … }               // declared in the block's header: `g := frame` then `in g { … }`
P := plane(u: r1, v: r2)            // a flat frame; the slots alias its axes, as `circle(center: c)` does

f.axis parallel shaft               // the joint's members take ordinary relations
shaft.p1 coincident f.axis
fix(ratio == 2, advance == 0mm) f   // its numbers held as an axis's `dir` and `origin` are
```

### The standard library carries the menagerie

```sv
std.Turn(f, about: shaft, ratio: 2)              // the axis on the line; advance 0 unless given
std.Turn(f, about: hole_axis, ratio: 1, advance: 1.5mm)   // a screw, 1.5 mm a turn
std.Spin(f, about: o, ratio: 1)                  // a turn in o's view: the axis through o along
                                                 // its plane's normal
std.Slide(f, along: rail, advance: 20mm)         // ratio 0, the axis on the rail
s := std.Screw(isa, f)                           // unchanged: reads f's instantaneous screw axis
```

Each is a component of a few relations on `f.axis`, `f.ratio` and `f.advance`, written in
`std.sv`. A numeric formal left unbound is an unknown, as for any component (`std.Screw` with no
`pitch:` solves its pitch). So `std.Turn(f, about: shaft)` with no `ratio:` leaves the ratio to
the solve, which stage 5 makes meaningful.

### `in` attaches, and nests

```sv
in (arm := frame) {
  in std.top {
    c := point
    fix((30, 0)) c                   // relations read as drawn
  }
  in (planet := frame) {             // declared in arm: arm is its parent
    in std.top {
      x := point
      fix((40, 0)) x
    }
  }
}
std.Spin(arm, about: o, ratio: 1)
std.Spin(planet, about: c, ratio: -3)
X coincident(t == 60deg) x           // (10, 17.3205): R(τ)·(c + R(−3τ)(x − c))
```

* A declaration's frame is its innermost block's, its chart the innermost with axes, and its
  carrier chain every jointed frame on the path.
* `in A { in B { … } }` and `in B { in A { … } }` mean the same when only one has a joint. The
  printer writes the jointed frame outermost. Two charts nested is warned about (the inner one
  wins, and the outer says nothing).
* Nesting inside a root `repeat` or `when` stays as it is today (`statements.rs`'s `in_block`).
  What is relaxed is `in` inside `in`.

### Readings across frames

| today | with frames |
|---|---|
| `X coincident(under: m, t == τ) x` | `X coincident(t == τ) x` when `x` rides and `X` does not: the pose `path(X)⁻¹·path(x)` at τ. `under: f` remains for an operand that rides on nothing. |
| `envelope(tool, under: m, from:, to:)` | `envelope(tool, seen_from: G, from:, to:)` when the tool rides. **The envelope rides on its observer**: the profile a rack cuts rides on the blank. |
| `solid(tool, under: m, at: θ)` | `solid(tool, at: θ)` when the tool rides. `under:` for one tool placed by a frame it is not in. |
| `solid(tool, under: m, from:, to:)`, `through: body` | the same, with `seen_from: G` for a generating cut. The swept solid rides on its observer. |
| `meshing(f1, f2, under: mesh, …)` | `meshing(f1, f2, …)`, each flank riding its member |
| `claim … over m` | `claim … over f` |
| `rate(X over t)`, `reciprocal`, `std.Screw` | unchanged |
| a component formal `m: motion` | `f: frame` (with the observer as a second formal where it was a relative motion) |

**The roll is still one shared number.** A relation naming `t ==` crosses frames at that roll,
and one that names none reads the drawing as drawn, whatever its operands ride. So `p coincident
q`, with `p` on the arm and `q` static, holds where both are drawn.

### Before and after

The rack that cuts an involute (primer §2.11), today:

```sv
blank := motion(about: o, ratio: 1)
rack := motion(along: slide, advance: 2 * pi * 30)
cutting := motion(rack, relative_to: blank)
flank := line(f0, f1)                                   // drawn static, carried at every reading
cut := envelope(flank, under: cutting, from: -25deg, to: 25deg)
```

With frames:

```sv
in (rack := frame) {
  in std.front {
    f0 := point hint((30, 5))
    f1 := point hint((39.4, 8.4))
    flank := line(f0, f1)                               // the rack's flank, riding the rack
  }
}
blank := frame
std.Spin(blank, about: o, ratio: 1)
std.Slide(rack, along: slide, advance: 2 * pi * 30)
cut := envelope(flank, seen_from: blank, from: -25deg, to: 25deg)   // rides on blank
```

A gear ratio, today a measurement read after the solve (`lantern_generation.sv`, the corpus's
one, allowed only in a motion's numbers, E107 elsewhere):

```sv
pinion_turn := motion(about: pinion_axis, ratio: -length(wheel_radius) / length(pinion_radius))
```

With the joint in the solve (stage 5), the ratio is an unknown held by the condition that makes
it a gear ratio. The pitch circles roll without slipping:

```sv
std.Turn(wheel, about: wheel_axis, ratio: 1)
std.Turn(pinion, about: pinion_axis)            // no ratio: an unknown
std.Rolls(pitch_point, pinion, wheel)           // no slip at the pitch point: ratio = −R/r
```

The ratio follows the radii, and can be stated instead and the radii solved.

## Stages

Each stage lands green: `make test` passes, and `tests/examples_sv.rs` holds every document to
the numbers it advertised before the stage. A corpus rewrite must not move a solved value
(compare `--where` across the rewrite, to 1e-12).

### Stage 1: chains in the core, with no syntax

* `MotionDef::Chain { outer, inner }`, lowered in the motion pass to a `Relative` over a minted
  inverse. Coaxial joints are folded into one.
* Height and cycle checks count the minted nodes (`Family::read`'s 64 levels and
  `program/motions.rs`'s `heights`).
* A test file `tests/frames.rs`, listed in `tests/main.rs`:
  * a planet's carried point against the closed form at several rolls;
  * three chained turns against the hand product;
  * a turn on a slide against a cycloid;
  * coaxial folding read by `Family::screw`.
* `tests/jacobians.rs`: a `CarriedPoint` row under a chain, FD-checked over both axes' columns.

### Stage 2: one kind, and the bare frame

* `EntKind::Plane` → `EntKind::Frame` (id 6). Every arm of the exhaustive matches CLAUDE.md lists
  is renamed, with no behaviour change.
* `f := frame` parses (the `frame` refusals go), and so does `in (f := frame) { … }`:
  * the app's editor splices a header that declares (`InBlock`, `edit::remove`);
  * `f.u`, `f.v` and `f.origin` are minted when named;
  * a point in a spatial chart has three coordinates (`Basis::lift`, `world_in`, `on_view_sheet`,
    `hint(at: f, (x, y, z))`, ordinates `along: u | v | n`);
  * a plane relation over a spatial frame is refused, since it is not a set.
* The binding's `KIND_ID` table and the app's plane pickers read `Frame`, with the same ids.
* Gate (`tests/frames.rs`): a point in a spatial frame lifting through its chart; a bare frame's
  axes placed by relations; an unnamed frame owning no unknowns.
* Spec §6.7 gets a paragraph, and the changelog an entry.

### Stage 3: joints as members, stated

* `MotionE` → `FrameE::joint`, with `f.axis`, `f.ratio` and `f.advance` minted when read.
  `EntKind::Motion` is retired and id 10 taken up.
* In this stage the numbers must be **stated** (held by `fix`, or by a bound std formal) and the
  axis held by relations. That is the same power as today, spelt the new way. A measurement
  stands in `fix(ratio == …) f` as it stands in `ratio:` now: the E107 exception moves with it.
* Kinds are read from the holding relations ([before the solve](#what-consumers-need-to-know-before-the-solve)).
* `std.Turn`, `std.Spin` and `std.Slide` go in `std.sv`. `motion(`, `relative_to:` and `phase:`
  stop parsing, each with one refusal naming the new spelling. `under:` takes a frame, and the
  formal type `motion` becomes `frame`.
* The FFI's `gcs_motion_sample` becomes `gcs_frame_sample`, and `web/src/core/motion.ts` becomes
  `frame.ts`.
* **The corpus rewrite:**
  * 26 `.sv` files carry 88 `motion(` lines, 19 files carry 28 `relative_to:` lines, and 32 core
    test files build motions from text.
  * The three phases become roll offsets. `twist_drill/drill.sv`'s `clearance_grind` sweeps from
    `start + φ`; `formate.sv` and `meshing_problem.sv`'s `gear_next` becomes `gear_index` read a
    pitch on (`Space` takes the pitch as its angle, `solid(flank, under: step, at: pitch)`).
* `docs/contract/generating.md`, the primer's §1.15 and the spec's §6.14 are rewritten, and the
  changelog gets an entry.
* Gate: no solved value moves. `tests/carried.rs`, `generation.rs`, `roll_through.rs` and
  `wankel.rs` pass with only their spelling changed.

### Stage 4: riding

* `in` nests inside `in`:
  * `stamp_plane` becomes `stamp_frame`, stamping the path;
  * `InBlock` records nested headers;
  * deleting a frame splices its blocks as deleting a plane does.
* A frame declared in a block takes it as its parent.
* Carriers are inferred where they are read: carried points, envelopes, swept and placed solids,
  meshing flanks, `claim over`.
  * `seen_from:` on a reading.
  * Envelopes and swept solids ride on their observer.
  * Mixed carriers are refused at the reading.
* Readings that name a roll and cross frames lower to `CarriedPoint` (or an envelope's motion)
  over the relative frame `path(X)⁻¹·path(x)`.
* The app: drawing in a jointed frame draws as drawn; the workspace offers a frame's chart as a
  place to draw; a membership edit on a block-stamped declaration is refused with the cause, as
  now.
* Gate (`tests/frames.rs`): the planet and the rack above against their closed forms; a swept
  solid riding its observer against the same body built with `under:`; the refusals.
* The language contract gets a section on frames, or a new `docs/contract/frames.md` with a row
  in CLAUDE.md.

### Stage 5: joints in the solve

* `f.ratio` and `f.advance` become **unknowns**, and `f.axis` is already one. They become columns
  of every kernel that reads a pose during the solve. Each gets a `tests/jacobians.rs` row,
  FD-checked over the joint's six numbers:
  * `CarriedPoint` (`Reading::Carried`, today constants `consts` over axis columns);
  * the rate rows and the rate of a contact (`contact_rate.rs`);
  * held envelopes (`envelope::held`, whose steps are constants today);
  * meshing;
  * planar generated envelopes, whose ratio and phase are tape values today. As columns they
    count against `generate::OUTER_MAX`.
* Unbound std formals become unknowns that mean something. `std.Rolls` (no slip at a point, the
  rate of the point carried by one member seen from the other held at zero) goes in `std.sv`.
  `lantern_generation.sv`'s measured ratio is rewritten with it, and **the E107 exception for
  motions is retired**: a measurement is again needed only where a number is needed before the
  solve.
* Kinds still come from the holding relations. A ratio the solve finds is fine for rows and for
  solids, which are built after it. A consumer needing a kind needs the relations that settle it,
  not the value.
* Gate: a gear pair's ratio solved from rolling, against `−R/r`; the centre distance solved from
  a stated ratio; a joint left free reported by the diagnosis with its six numbers.

### Stage 6: documents ride where it reads better

The generation examples, the Wankel and the spiral bevel are rewritten to declare their tools in
the frames that carry them, one document at a time, each holding its numbers. Where `under:`
reads better (one tool, several frames) it stays. This stage is editorial and may stop whenever
the remaining documents read well.

### Later, and not in this plan

* **General drives:** a joint's value as an expression of the roll (a cam law), differentiated by
  `taylor::Jet`, with its speed bounded by interval arithmetic so `MotionBounds` and the roll
  search stay conservative. Spec §6.14 says the families "do not yet specify arbitrary motion
  laws". A drive is not a constant twist, so it is outside the joint of this plan by
  construction.
* **Several rolls:** independent rolls for carried points and placements only. Envelopes and
  sweeps keep one, since an envelope of a two-parameter family is a different object.
* **Station frames:** a loft station's frame (`std.On`, `std.Minimal`, `std.Frenet`) is a frame
  driven by a path parameter. It could join, with several rolls.
* **A product step** in `Family`, if lowering chains through inverses shows in a profile.
* **An assembly solver** (§6.14's other "not yet"). Stage 5 solves a joint's *constants*. A
  frame still holds no state, and every reading names its roll.

## Open questions

Each is answered as built; the proposal stood unless it says otherwise.

1. **`seen_from:`** is the word. It is written beside `under:` from stage 3, where
   `relative_to:` goes, on every reading that names a motion: `coincident(under: m, seen_from:
   g, t == τ)`, `envelope(…)`, `meshing(…)`, `solid(…)` and `claim … over m seen_from g`.
2. **`f.uv`** is deferred: a spatial frame's chart has three numbers and no coordinate plane of
   its own; a flat drawing in it is a `plane` declared in it.
3. **A plane's slots** stay, aliases of its axes as proposed.
4. **The std words** are three: `std.Turn(f, about: line, ratio:, advance: = 0mm)`, `std.Spin(f,
   about: point, on: plane, ratio:)` — the plane named, since a component reads no point's plane
   — and `std.Slide(f, along: line, advance:)`.
5. **`under:` on an operand that already rides** is refused, as proposed.
6. **Nesting order**: both orders are accepted, as proposed.
7. **Equal rates**: a zero rate along each of two directions is enough; `std.Rolls` is written
   with two (stage 5).

**Two findings while building.** `frame` was not unused: documents name things `frame`
(`hub_motor`'s groups, `hypoid_drive`'s `frame := Frame(…)`), so the word declares a frame only
as a definition's whole value, `f := frame` (and in a block's header), and stays a name
everywhere else. And a left-out `frame` formal is the instance's own frame, declared bare, at
rest — what an observer formal needs where a component is called with a motion seen from the
world.
