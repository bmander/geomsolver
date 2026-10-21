# A reusable solve boundary: the contract

**Status (2026-10-21): a design for review (#451); the advisory classifier (#452) built, §16; the
prototype (#454) built, §17, its rules kept in [`contract/boundaries.md`](contract/boundaries.md);
the benchmark (#459) run, §18: no syntax now, and a sparse parent before a cache.** It states what a component
must satisfy for a parent to treat it as a **reusable solve boundary**: a function of its inputs,
solved once per distinct input and consumed through its interface. It is the first step of track A
of epic #465, written before the classifier (#452), the uniqueness spike (#453) and the prototype
(#454), each of which builds against it. Each rule is stated so that a test can hold it, and §12
lists those tests and the issue that writes each one. Once the prototype lands, the rules it keeps
become a contract topic file, `docs/contract/boundaries.md`, with a row in `CLAUDE.md`.

Background: [the direction paper](solvent-direction.md) §3.2 (C3, C4) and §5 (N2, N3, N8, N9), and
[the review](https://github.com/bmander/geomsolver/issues/448#issuecomment-6303715169) of its
first version. The paper proposed boundaries, and the review found that its guarantees outran its
machinery. This document answers by saying exactly what is promised and what is not. No syntax is
proposed here. Where a rule needs something the language cannot say yet (a domain, a context
value), the prototype supplies it from Rust, and #461 decides on #459's evidence whether it
becomes syntax.

Everything shown as current behaviour was run through `solventc` or read in the code at `c6f4ca7`.

---

## 1. What exists today

The contract is built from parts the core already has. They are listed here so that the rules below
can say which parts they keep and which they change.

* **Closed component scope** (spec §5). A body reads its formals and its own declarations, and
  calls definitions and built-ins. Reading the root's geometry, a value of the file, an enclosing
  component's names or `std.front` is refused: E101 for an entity, E103 for a number
  (`flatten/resolve.rs`, `flatten/values.rs`; `tests/closed_scopes.rs`). So closure *by name*
  already holds. §3 lists the channels it does not cover.
* **Unbound numeric formals are the instance's unknowns.** A number the call leaves out is
  `inst.formal`, solved with the drawing and seeded by `f: hint(v)` (`flatten/bindings.rs`). This is
  the mechanism §5 uses for exported freedoms.
* **Interfaces type formals** (spec §8.1, 0.84; `flatten/interfaces.rs`). A formal typed by an
  interface sees only the members the interface declares. In the spec's words, "an interface names
  no solve boundary and makes nothing". Interfaces constrain what a body *reads*, never what a
  parent reads of an instance.
* **A parent reads, and relates, anything not `private`.** `inst.p` and `inst.sub.p` are paths, and
  a parent may state relations on them (`rates/geodesic.sv` places `g.p` and squares `g.tl` to
  `g.ax` from outside). Today that silently merges blocks of the solve.
* **The locus is a boundary in miniature** (`locus.rs`; `docs/contract/curves.md`). A traced
  component is lowered once into rows over `[u, θ, values, q, w]`. The block must be square ("a
  trace block must determine its points"). Each evaluation is a Newton solve with LU. The derivative
  with respect to `(u, θ)` is `Jq·S = −B`, one factorisation for every right-hand side: the implicit
  function theorem. Evaluation resumes from the pose this contact last accepted, checked by
  `continues` against the tangent's prediction. If that fails, it cold-starts at the anchor and
  marches. `ccw` predicates are enforced at the anchor by reflecting and solving again. Restarts
  draw from a fixed-seed `Rng`, so "the answer cannot depend on history"
  (`trace.rs::an_evaluation_does_not_depend_on_what_was_evaluated_before`). Four things it lacks
  matter below:
  - it has no condition estimate: `lu_factor` fails only on an exactly zero pivot;
  - a failed solve returns its best point with `ok: false`, and `point_on_body_res` writes
    `p − C(u)` without reading `ok`, so a nearby value can reach the parent;
  - its memo is keyed by the address of the contact's constants;
  - its module comment says the fallback marches "from the low end of the domain", while the code
    marches from the anchor.
* **Claims are judged and never solved for** (spec §9.7; `diagnose.rs`). A claim's rows are stacked
  onto the conditioned system at the solution. It is a *theorem* if they add no rank, *consuming* if
  it holds only by the pose, and *violated* if it does not hold.
* **Root choices.** A bound (spec §9.6) chooses the root and is verified: the seed is carried across
  it, `steer` acts, the block pass searches, and a bound still broken ends with `BOUND_BROKEN`. On
  the sheet, `ccw`/`cw` stated in the document is held the same way since 0.161 (#157: carried on
  the seeds, steered, `ORIENTATION_BROKEN`), apart from the choices the constructive plan records,
  which are not held; the block pass's search reads bounds only.
* **"Solved"** is `max_relative_residual ≤ 1e-6` (each row over `extent^degree`), and **settled**
  is `success && status != 4`. #449 shows that even settled is not enough: a document can be
  infeasible and still report success at the iteration limit.
* **Block order.** `System::block_order` gives the Dulmage–Mendelsohn square part as strongly
  connected blocks, in solve order. `BlockTr` solves one block with every other column held.
* **Rank.** Rank and null space are judged on `System::conditioned` against the absolute
  `RANK_TOL = 1e-9`. The ledger `Driven` reads the rank a set of rows adds to the rest.

## 2. Terms

A **boundary** is a component definition, together with the structural numbers its call decides
(§3, CL3), that a parent treats as one function. An **instance** is one call of it.

The instance's **inputs** `x` are:

* its **operands**: the entities its formals alias (points, lines, circles, planes, axes, frames,
  paths, sets) and their coordinates;
* its **carriers**: the planes its entities are drawn in. A carrier is an operand whether a formal
  names it (`in view`) or the instance took it from where it was written (`in std.front { t :=
  Rung(a) }`);
* its **numbers**: the numeric formals. Each is *bound* (a value, substituted at elaboration) or
  *unbound* (an unknown of the parent, §5);
* its **context**: values fixed at elaboration that the boundary reads without a formal. Closed
  scope admits none today. §3 keeps a place for them (N7 of the paper) without proposing syntax.

Its **own unknowns** `y` are the coordinates of everything its body declares. Its **rows**
`F(y, x) = 0` are the body's acting statements: claims and bounds compile no row, as today. Its
**root choices** `R(y, x)` are its bounds and orientations: sign conditions, not rows. Its
**outputs** `o = g(y, x)` are what a parent may read (§7).

Its **shape** `s(x)` is the part of its inputs invariant under proper rigid motion: radii, distances
between operands, angles between them, and the numbers. Its **pose** `P(x)` is the motion that
takes a canonical placement of that shape to where the operands stand (§6).

An **evaluation** is one solve of `F(·, x) = 0` subject to `R`:

* A **fresh** evaluation starts from the body's seeds at `x`.
* A **continued** evaluation starts from the last value this instance accepted, as the locus's
  resume does.

Its **domain** `Ω` is the set of inputs at which the boundary promises a value (§8).

---

## 3. Closure

**CL1 — A value depends on the definition, the inputs and the context, and on nothing else.** A
fresh evaluation is a function of `(definition, x, κ)`. It does not depend on:

* the rest of the document: other instances, the root's geometry, the document's extent;
* the order in which instances are evaluated;
* what the solver evaluated before.

*Held by:* evaluate one instance inside two documents that differ only elsewhere: another instance
added, or geometry far from it that makes the document's extent 1000× larger. The values agree to
the inner solve's tolerance. Evaluating a key after an unrelated history gives the same bits (the
locus's history test, generalised).

Closed scope gives the name half of CL1. It leaves four channels open, and the contract closes each:

* **The carrier taken from where the instance is written.** Take `component Rung(a: point, w:
  Length)`, whose body declares `b` and states `a distance(w) b` and `a horizontal b`, with no plane
  formal. Written once in `in std.front` and once in `in std.top`, it gives one figure in plane
  coordinates and two in space. Run: `b` lands at `(30, 0)` in each plane, and `horizontal` read
  each plane's `up`. That is correct,
  and it means the carrier is an *input*: it is part of `x`, its key and its dependencies, though no
  formal names it.
* **The document's scale.** Today a block solve keeps the whole system's `row_scale` on purpose ("a
  block is never compiled as a sub-sketch, whose different extent would scale it differently"). A
  boundary needs the opposite. Its acceptance, its rank and its conditioning (§8) are judged at its
  **own** scale, read off its inputs' shape. Otherwise one instance could be accepted in a small
  document and refused in a large one.
* **Solver history.** The locus's memo is keyed by an address and its resume by the last pose. Both
  are allowed only as the *continued* evaluation of DT5. A fresh value never reads them.
* **Seeds.** Seeds are inert for the solution set (P3), but they choose which root a tier-1
  boundary returns. CL1 therefore holds of the value only if every seed is a function of the inputs.
  A seed in a carrier's coordinates is one, because the carrier is an input. Whether it survives a
  motion is EQ's question (§6), not closure's.

**CL2 — The inputs are known before anything is solved.** For each instance, elaboration lists its
operands, carriers, numbers (each bound or unbound) and context reads, and nothing outside the
list is read during an evaluation. *Held by:* the listed inputs equal those found by walking the
lowered rows' columns, and no column of the lowered rows lies outside `x ∪ y`. This is the locus's
range check (`view` returns `None` for a column past the table), applied to a component.

**CL3 — A structural number is part of the definition, never an input.** An `Int` formal, a `when`
condition, `repeat N` or `ring N` decides *what the boundary is*. Two calls that differ in one are
two definitions, keyed separately and never interpolated. An `Int` formal left unbound is therefore
not a freedom. Today it is one: `Rung(a)` with an unbound `n: Int` elaborates with `r.n` as a free
unknown, although the spec calls a count "never an unknown" (`flatten/bindings.rs` skips only
non-numbers, and `Ty::number` includes `Int`). A boundary refuses it. *Held by:* an instance leaving
an `Int` formal unbound is classified "not a boundary: a count is unbound".

---

## 4. Determination

**DT1 — Locally determined.** At input `x`, with value `y*`, the boundary's Jacobian in its own
unknowns, `F_y(y*, x)`, is square and nonsingular. It is judged on the conditioned matrix, at the
boundary's own scale (CL1), against `RANK_TOL`. That is all full rank shows: `y*` is isolated, and
by the implicit function theorem a smooth branch `y(x)` passes through it. It says nothing about:

* other roots at the same `x`;
* whether a root exists at other inputs;
* which root a seed reaches.

A boundary is **generically locally determined** when DT1 holds at the instance's inputs and at `K`
inputs perturbed from them along random shape directions (seeded `rng::Rng`; `K` is the
classifier's to choose).

Run: `x distance(5) a`, `x distance(5) b` with `a`, `b` six apart is DT1 at both `(3, 4)` and
`(3, −4)`, numeric rank 2, DOF 0. *Held by:* that case is DT1; the same body with `b` coincident
with `a` is not.

**DT2 — Branch explicitly chosen.** DT1 holds, and every discrete choice the boundary makes is made
by a stated root choice: a bound, or `ccw`/`cw`. The solver verifies the choice on the value and
fails the evaluation where none holds (§9). This tier is not available for orientations until #157:
today `ccw(a, b, x)` added to the example above still returns `(3, −4)` from a seed below the line.

That the stated choices resolve *every* ambiguity cannot be shown at one input. So DT2 is
**claimed by the author and tested by refutation**. From `S` scattered starts at the same `x` (the
block pass's restart scheme: points scattered about the operands at their size, a fixed stream
per start), every start that converges and satisfies `R` must reach the same `y*` to tolerance. Two
distinct roots that satisfy `R` refute DT2 at that `x`. The classifier labels the result a
heuristic: it finds no second root, and proves none absent. *Held by:* the two-root point without
`ccw` is refuted (two roots found); with `ccw`, once #157 lands, it is not.

**DT3 — Verified unique over a stated domain.** A domain `Ω` is declared (§8, DD4). For every
`x ∈ Ω`, exactly one `y` satisfies `F = 0` and `R`, and DT1 holds there. The verification is
either an argument recorded beside the definition's test (the common tangent's, §13) or a
computation (#453: interval Newton or Krawczyk over a box of `Ω`, the root choices cutting the
box). "Not verified" is an honest answer, and the boundary falls back to DT2 or DT1. *Held by:* the
spike's cases, and for each boundary that claims DT3, a test that samples `Ω` (seeded) and finds DT2
unrefuted at every sample. The sampling only checks the argument; it does not replace it.

The spike (#453, `tests/uniqueness.rs`) built the computation and found what it needs and costs:

* **Uniqueness is relative to a region `S` every admissible root lies in.** A row such as `s on a`
  gives `S` for free; a circle through three points does not, and its `S` has to be stated. The
  verdict is "exactly one root in `S`", never more.
* **The verification is over the shape, not the placement** (§6): the first operand at the origin,
  the second on an axis. Its cost grows as a power of the input dimension, and the three
  placement coordinates multiply it by one to two orders of magnitude.
* **It judges the rows as compiled, not the figure.** `CommonTangent` stated with the regular
  tangency at each end (`run tangent(at: s) a`, the `tangent_arc_line` kernel) has a second
  admissible root wherever the circles cross: `s = e` at the crossing, a zero-length run. The
  verifier proves it a root (a refutation) and the solver reports it solved, DOF 0. Stated with the
  radii to `s` and `e` parallel instead, the same tangent verifies on every box tried, from
  circles well clear to circles nearly nested.

**DT4 — Deterministic.** A fresh evaluation is a function of the key (§6, EQ4): the definition, the
shape, the context and the carriers' parts that the boundary reads. It is computed in the canonical
placement from seeds stated relative to the operands, then posed. Two instances with one key get one
value, moved by their poses. This is what a cache needs, and it holds at every tier, because it is a
property of the procedure, not of the equations. *Held by:* N instances of one key evaluated in
shuffled orders give bit-identical canonical values.

**DT5 — Continuous along a parent's path, and checked there.** Within a parent's iteration, an
instance is evaluated by **continuation**: warm-started from the value it last accepted. The new
value is accepted only if:

* the Newton correction stays within a fixed fraction of the move the derivative predicted (the
  locus's `continues`, `PREDICTED = 0.5`);
* the sign of `det F_y` is unchanged (DD3).

Otherwise the evaluation fails with *branch changed* (§9). This is what a parent's Newton needs, and
determinism does not give it. A fresh evaluation at nearby inputs may land on another root
wherever the basins of the seeds' starts cross. *Held by:* a two-root boundary driven by a parent
along a path that crosses the seeds' basin boundary keeps its branch under continuation and fails
when the path reaches the fold (#455's test).

**DT6 — Path independence is checked at the end.** At a parent's accepted solution, each instance's
continued value must equal its fresh value at the final inputs, to tolerance. A mismatch means the
document's answer depends on the path its solve took. It is reported, naming the instance and both
values, and the solve is not *settled*. At DT3 inside `Ω` a mismatch cannot occur, so the check
costs one fresh evaluation per distinct key and finds only bugs. At DT1 and DT2 it is the only
thing that does. *Held by:* a document whose solve walks a DT1 boundary from one basin into the
other reports the mismatch.

What each tier permits:

| tier | the parent may | what it relies on |
|---|---|---|
| not DT1 | nothing: the component is inlined, as today | — |
| DT1 | solve through it (DD1), checked (DT5, DT6); share one value per key (DT4) | seeds pick the root |
| DT2 | the same, and its value is the author's stated choice | refutation, not proof |
| DT3 | the same, and DT6 cannot fail inside `Ω` | an argument or a verified box |

---

## 5. Exported freedoms

**FR1 — A boundary is square in its own unknowns.** It states exactly as many independent rows as
it has own unknowns: `dim F = dim y`, the locus's squareness rule applied to a component.

A body that states a fact about its inputs alone is a *verb* fused to a noun, and is not a
boundary. Such a fact is a row whose columns are all inputs, or a row beyond what its own unknowns
absorb, such as `a distance(10) b` over two operands. The fix is to split it: the noun builds, and
the parent states the fact (paper R3). *Held by:* a component with one such row is classified "not
a boundary: states a fact about its operands", naming the row.

**FR2 — A freedom is exported only as an unbound numeric formal.** A crank's angle is a formal
`theta: Angle` the call leaves unbound. Inside the boundary it is an input like any number. In the
parent it is an unknown, `crank.theta`, seeded by `theta: hint(30deg)` and placed by the parent's
own statements (a coupler's length). That is exactly what an unbound formal is today, so no new
mechanism is needed. A freedom the body leaves that is not such a formal (a point free to slide)
fails DT1. The classifier lists it as a freedom the parent would have to own and cannot name.
*Held by:* `Crank(o, theta: hint(30deg))` is DT1 with `theta` an input. The same body with the
angle an unknown of its own (no formal) is not DT1, and the classifier names that unknown.

**FR3 — A boundary has no gauge.** FR1 and FR2 make `F_y` square, so the derivative of §8 is
always `−F_y⁻¹F_x` and never a minimum-norm choice. #455 asks how a boundary with exported freedoms
is differentiated, "minimum-norm, or the freedom held as an input". The contract answers: **held
as an input**. A minimum-norm derivative would make the value depend on how the freedom is
parametrised and on the previous pose, which breaks DT4.

---

## 6. Rigid-motion equivariance

**EQ1 — The group.** Equivariance is with respect to **proper** rigid motions `g` of space acting
on every input:

* operands are moved;
* carriers are moved as *oriented* planes, normal included;
* numbers are left alone.

A boundary is **equivariant** when `B(g·x) = g·B(x)`, entity by entity, for every `g`. Reflections
are excluded: `ccw`, `turn:` and a bearing's sense are read about a carrier's normal, and a
reflection reverses them. A mirrored instance is another key, or a declared mirror of one (§15).

**EQ2 — A carrier is read as a plane, not as a frame.** A carrier has an origin and in-plane axes
`u`, `v`. A boundary may read its normal and its place, but not those axes or that origin. If it
read them, two instances drawn in `std.front` at different places in it would be related by a
motion that moves their operands and leaves the carrier where it is, and EQ1 would not apply to
them. So equivariance is checked twice:

* move everything by `g`;
* and, separately, **spin and slide each carrier's frame within its plane**, with the operands held
  where they stand in space (so their plane coordinates change). The value in space must not change.

The body reads a carrier's axes, and fails the second check, through:

* `horizontal`, `vertical`, `level(up | right | u | v)`;
* `along: u`;
* a seed in plane coordinates, `hint((3, 4))` or `hint(at: P, (3, 4))`;
* a bearing seed, `hint(at: k, bearing: β)`, measured "from the x axis of the plane";
* `fix((x, y))`;
* any read of a coordinate, `p.x`.

These are the classifier's "reads the world" (#452). Closed scope cannot catch them, because the
carrier is legitimately an input.

Run, with a seed in plane coordinates:

```solvent
unit mm
use std
component Apex(view: plane, a: point, b: point) {
  in view {
    x := point hint((3, 4))
    x distance(5mm) a
    x distance(5mm) b
  }
}
in std.front {
  a := point
  b := point
  fix((0, 0)) a
  fix((6, 0)) b
  c := point
  d := point
  fix((100, 0)) c
  fix((94, 0)) d
}
one := Apex(std.front, a, b)
two := Apex(std.front, c, d)
```

`two`'s operands are `one`'s turned 180° about `(50, 0)`, so equivariance asks for `two.x =
(97, −4)`. It solves to `(97, 4)`, with numeric rank 4 and DOF 0. The body is closed, DT1, and still
not equivariant. Write the seed from the operands instead, `hint(at: a, toward: b, by: 0.8333, turn:
53.13deg)`, and `two.x` is `(97, −4)`. *Held by:* exactly this pair: the first refuted by the
spin check and the motion check, the second passing both.

**EQ3 — The canonical frame is built from the operands, in their declared order.**

* **The origin** is the first operand's anchor: a point itself, a circle's or an arc's centre, a
  line's `p1`, a plane's or a frame's origin.
* **The first axis** points from that origin to the next operand's anchor that is distinct from it.
  If there is none, it is the first line's or axis's direction.
* **The second axis** is, for a boundary drawn in a carrier, `n × u` with `n` the carrier's normal,
  which also fixes the handedness. In space, it comes from the next anchor or direction independent
  of the first.

Where the operands fix no axis, the canonical frame takes one by a fixed rule (the carrier's `u`).
Examples: a lone circle; two concentric circles; a lone point with a carrier.

**EQ4 — The key is the shape in that frame.** The key is the definition, the operands' coordinates
in the canonical frame (which are their invariants), the numbers, and the context. The pose is the
canonical frame itself. Ordered containers only, and a stable encoding of floats. Inputs equal to
within tolerance but not in bits are *not* the same key: a key never rounds, so a hit can never
return a value its own inputs would not.

**EQ5 — Where the operands leave the frame undefined.** Let `G_x` be the proper motions that fix
every input. Two cases:

* **`G_x` connected** (rotations about a lone circle's centre, about a line in space). Then **an
  equivariant DT1 boundary is invariant under it automatically.** For `g ∈ G_x`, `g·y*` solves the
  same equations at the same inputs. The orbit `G_x·y*` is connected and contains `y*`, which is
  isolated, so the orbit is `{y*}`. A body whose result would turn with the free rotation (a keyway
  on a pulley given only a centre) is not DT1: the rotation is a free direction of `F_y`. It fails
  DT1, not EQ, and the classifier names the freedom. The fix is a datum operand that fixes the
  frame, or an exported angle (FR2).
* **`G_x` finite and non-trivial**: a symmetry that fixes every operand but maps one isolated root
  to another. Every operand Solvent has carries an orientation: a line runs `p1 → p2`, an axis has
  its `d`, a circle is drawn in a carrier with a normal, and a plane is read with its normal. A
  motion fixing points and oriented directions is a rotation about a point, a rotation about a line,
  a translation within a plane, a combination of these, or the identity, and all of them are
  connected. So we know of no operand pattern in the language with a finite `G_x`. The two-root
  point's roots are related by the half-turn about `ab`, and that turns the carrier over, so it is
  not in `G_x`. If one arises (an unoriented set operand, say), the root choice must be stated
  (DT2), or a datum given, or those inputs left out of `Ω`.

*Held by:* `Pulley(centre)` (a rim and a bore) evaluated at a centre and at that centre with the
canonical axis rotated gives one value. A keyway at a bare centre fails DT1, naming the rotation.

**EQ6 — Identity survives the pose.** Every entity of an instance is the corresponding entity of
the canonical value, moved by the instance's pose, and keeps its own name and key (`flatten::keys`).
A value is shared as numbers, never by reference. The document's writeback, its diagnosis and its
callouts see per-instance entities, exactly as now. *Held by:* N instances of one key, each with
its own `upper.run.p1` read by the parent, each matching the flat solve's.

---

## 7. Interface and outputs

**IO1 — What a parent may read.** A boundary's **outputs** are its non-`private` named members and
their own fields, with nothing anonymous and no rows among them:

* the entities its body names (`upper.run`, `upper.run.p1`, `small.rim.r`);
* its numeric formals (`crank.theta`);
* the members of a named nested instance, read through that instance's name (`drive.upper.run`).
  Each step of such a path reads an output of the step before it.

Interfaces are unchanged. They type a formal, and say what a body reads of its operands. *Held by:*
a parent reading a `private` member of a boundary is refused (E101, as today). An anonymous child
has no path to read it by.

**IO2 — A boundary never yields.** A parent's statement over an output is a row over the
**parent's** unknowns. It reaches them only through `o(x)` and its derivative, and never moves the
output off `B(x)`. Two consequences:

* A statement such as `fix(r == 20mm) small.rim` is legitimate exactly when the radius is an
  exported freedom (an unbound `r` of `Pulley`). The row is then the parent's equation for
  `small.r`.
* A statement over outputs that reaches no parent unknown is an over-constraint of the **parent**,
  diagnosed there, naming the output and the instance. It is never absorbed by the boundary's
  interior, because that interior has no freedom to give (FR1, FR2).

Flat and boundary solves agree on this whenever the boundary is DT1 (§10). *Held by:* the same
over-constraint in both forms, and the boundary form's diagnosis names the instance.

**IO3 — A parent cannot choose a boundary's root.** A bound or an orientation stated by the parent
over a boundary's outputs constrains the parent's unknowns. It does not choose the boundary's
branch. In the flat solve it would: a parent's `ccw(a, b, one.x)` picks `one.x`'s root. Through a
boundary the root is the boundary's own (DT2), and if the parent's choice is broken, the parent's
solve fails with that choice named. A choice the parent wants to make is an *input*: a `Side`
formal, decided at elaboration (CL3), that the body's own root choice reads. *Held by:* the
two-root point as a boundary with its `ccw` inside, and a parent stating the opposite `ccw` over
`one.x`, reports the parent's choice broken. With a `Side` input, the parent gets the root it asked
for.

**IO4 — Outputs carry identities, and values flow one way.** An output of one boundary may be an
operand of another (`CommonTangent(small.rim, large.rim)`). The second boundary's inputs are then
the first's outputs, and the instances form a DAG within the parent. A cycle among a parent's
children through outputs and inputs is a coupled system of the parent's, solved at the parent
(paper C5: "closed explicitly at the level that owns it"), never by iterating the children against
each other. *Held by:* two boundaries each reading the other's output are solved as one parent
system, and the instance graph reports the cycle.

---

## 8. Derivatives and domains

**DD1 — Derivatives come from the boundary's own factorisation.** At an accepted value,

```
dy/dx = −F_y⁻¹ F_x,      do/dx = g_y · dy/dx + g_x,
```

from the one factorisation of `F_y` the evaluation finished with: the locus's `finish`, `Jq·S =
−B`, for every input column at once. Outputs measured from the sketch (a length, an angle, an arc's
length) are differentiated exactly. Finite differences are not used for them. *Held by:* a
finite-difference check of every output against every input, at the scale and tolerance
`tests/jacobians.rs` uses for kernels (#454).

**DD2 — Conditioning is reported, and a near-singular value is a failure.** Each evaluation reports
an estimate of the smallest singular value of the conditioned `F_y` at its own scale, from the
factorisation already computed (a condition estimate on the LU, or the QR's `R`). Below a threshold
`τ` the evaluation **fails** with *singular* (§9). It does not return a value with a huge
derivative. `τ` is #455's to choose, and it sits above `RANK_TOL`. The positive root of `x² = u`
shows why it cannot be left to the parent: `dx/du = 1/(2x)` is finite at every `u > 0`, and a
parent's Newton walking `u` down meets ever larger steps until it lands at or past zero. With `τ`,
the boundary refuses before that, and the parent's trust region shrinks (DD5). *Held by:* that
boundary, approached from `u > 0`, fails *singular* at a `u` above 0, and the parent stops short and
reports (#455's test).

**DD3 — A branch change is detected, not inferred.** Each accepted evaluation records the sign of
`det F_y`, from the LU's pivots and its permutation's parity (rows and columns in the instance's
compiled order, so successive signs compare). Along a parent's path, a change of sign between two
accepted evaluations means the path crossed a fold or a bifurcation. That evaluation fails with
*branch changed*, even where its correction passed `continues`. *Held by:* a two-root boundary
whose inputs are driven toward the fold where its roots meet, by steps that land a warm start on
the other root, fails there and does not return that root.

**DD4 — A domain is a set of conditions on the inputs alone.** `Ω` is declared as conditions over
measurements of the inputs, such as `distance(a.center, b.center) > |a.r − b.r|`. They are judged
before the inner solve: outside `Ω` the evaluation fails with *outside*, naming the condition,
and nothing is solved. Because they read only inputs, they are bounds over the parent's unknowns,
and the parent can steer by them as it steers by bounds today. Until #461 decides on syntax, the
prototype states a boundary's `Ω` in Rust. A boundary with no stated `Ω` has `Ω` = everywhere DT1
and DD2 hold, and fails where they do not. *Held by:* the common tangent called with one circle
inside the other fails *outside*, naming the condition, and never runs Newton.

**DD5 — The parent stays inside.** A parent's step that would take any instance's inputs out of its
`Ω` is shortened to the condition's edge, as a step is carried across a bound. A failed evaluation
(§9) rejects the step and shrinks the trust region, as a NaN residual does today. So a parent near
a domain's edge slows and stops short. It never accepts a value from outside. *Held by:* the
belt-length loop of the paper's §6.4, started near `c = r₁ + r₂`, converges without an evaluation
outside `Ω`. The evaluations made are counted, and each is checked.

**DD6 — Sketch outputs only.** This contract covers outputs measured from the solved sketch. An
output through an evaluated solid (a volume, a mass) needs an accuracy contract for its finite
differences and detection of a change of topology within a perturbation. It is #458's phase 2, and
stays outside this contract until then. Spec §1.2's "no edge back" holds within a boundary.

---

## 9. Failure

**FA1 — An evaluation returns a value or a failure, never a nearby value.** A **value** carries:

* the pose and the canonical figure;
* the derivative (DD1), the condition estimate (DD2) and the sign of `det F_y` (DD3);
* the claims' verdicts (§11).

A **failure** carries a reason and the inputs at which it happened:

| reason | when |
|---|---|
| *outside* | a condition of `Ω` is false (DD4); nothing was solved |
| *unsettled* | the inner solve did not reach `max_relative_residual ≤ 1e-6` at the boundary's own scale, judged on the residual itself, not on a status (#449) |
| *singular* | the condition estimate fell below `τ` (DD2) |
| *branch changed* | a continued evaluation failed `continues`, or `det F_y` changed sign (DT5, DD3) |
| *root choice broken* | a stated bound or orientation does not hold at the value (DT2) |

The locus does not meet FA1 today: a failed solve keeps its best point with `ok: false`, and
`point_on_body_res` writes the residual from it. A boundary's failure reaches the parent as NaN
rows (which `System` already reads as "not converged") or as a typed result, never as coordinates.
*Held by:* each reason produced by a small boundary built to produce it, and in each case the
parent's residual is not finite.

**FA2 — In a parent's iteration a failure rejects the step.** The trust region shrinks and the step
is tried again. A failure is never accepted, cached as a value, or replaced by the last good value.

**FA3 — A failure the parent ends on is the parent's failure.** If the parent's solve ends on a
failed evaluation, it reports `success: false`, and its status names the instance, the reason and
the input values. The diagnosis attributes the failure to the **parent**: it names the parent's
rows that hold the instance's inputs where the failure is, as a broken bound is reported today. It
never lists the boundary's interior as conflicting. The boundary's own diagnosis at those inputs is
available beside it, for example "*outside*: one circle holds the other: centres 10 apart, radii 20
and 45". *Held by:* the belt drive with its centres fixed closer than the radii's difference:
`success` false, the drive's tangent named, the parent's `distance` named as what holds it there.

**FA4 — Failures are values of the key too.** A fresh evaluation that fails, fails for every
instance with that key (DT4). It may be cached as a failure, and is re-reported for each instance
by name.

---

## 10. Agreement with the flat solve

**FL1 — Soundness.** The pose a document solves to through boundaries (the parent's unknowns, each
instance's inputs, outputs and interior) satisfies every row of the same document's flat
elaboration, to the flat solve's tolerance. *Held by:* #454's acceptance test, for every boundary
document in the test set.

**FL2 — A fixed point of the flat solve.** The flat solve started from that pose stays there.
*Held by:* the same documents; zero iterations, or a residual already under tolerance.

**FL3 — The rank adds up.** Where every instance is DT1 at the solution, the flat document's
numeric rank is the sum of the instances' own unknowns and the rank of the parent's reduced system
(its rows over its unknowns, through the instances' derivatives). So the DOF agrees, and so does
every over-constraint the parent reports. *Held by:* the rank identity checked on each boundary
document, and on the belt drive with one of its rows duplicated at the parent.

**FL4 — Not promised: the same root.** Below DT3, the flat solve from the document's seeds may reach
a root the boundaries do not, and the boundaries may reach one the flat solve does not. Both are
solutions (FL1). The benchmark counts such disagreements and reports them (#459). They are not
failures of the contract.

---

## 11. Dependencies, invalidation and claims

### Dependencies

**DP1 — Each output has a dependency set.** `D(o)` is the set of inputs and context values that
output `o` reads. It is computed structurally from the boundary's lowered rows, in their block
order (`graph::blocks` over `F`'s incidence):

* the inputs read by the rows of every block upstream of the block that places `o`;
* the inputs `g` reads directly;
* for a nested instance, its own `D` of each output it consumes, transitively.

A structural set over-approximates: an input may be in `D(o)` and still have no numerical effect.
That is sound, and costs only a needless re-evaluation. *Held by:* for each output and each input
outside `D(o)`, the derivative of DD1 is exactly zero. The sets are exposed for diagnosis
("`belt_length` depends on `r_small`, `ratio`, the shafts' distance").

**DP2 — Shape and pose are separate dependencies.** For each output, `D(o)` splits into what it
reads of the shape and whether it reads the pose:

* an **invariant** output (a length, an angle, a radius, any number measured from the figure) reads
  the shape only;
* an entity output reads both.

A change that moves every input of an instance by one rigid motion changes its pose only. Nothing
is solved again: entity outputs are re-posed, and invariant outputs are unchanged. *Held by:* moving
a drive's two shafts together re-solves nothing (evaluations counted), and its `belt_length` is
bit-identical.

**DP3 — Invalidation follows `D`, and cutoff is exact.** After a change `δ` to inputs or context,
an output is re-evaluated only if `δ` meets `D(o)`, or if `δ` moves its pose and it reads the pose.
An instance is re-solved only if an output some consumer reads is invalid. A re-evaluated output
**bit-identical** to its previous value invalidates nothing downstream: early cutoff, as in
incremental build systems. Cutoff is exact, never within a tolerance. A tolerance would make what a
document solves to depend on what was cached before. *Held by:* the paper's §6.4 cases on a
two-level document shaped like the belt drive, with evaluations counted, not timed (#457):

* a value read only by a bore re-evaluates the pulleys and not the parent's loop;
* a new ratio re-evaluates what reads it and nothing else.

**DP4 — Reuse is indistinguishable from re-evaluation.** A fresh output is computed from the blocks
upstream of it only, each from seeds stated relative to its inputs. So its value is a function of
`D(o)`'s values alone, and a reused output is bit-identical to one evaluated again. A continued
evaluation (DT5) depends on its path and is never reused across a change. It is checked against a
fresh one instead (DT6). *Held by:* for each output, perturbing then restoring an input outside
`D(o)` and evaluating fresh gives the same bits as not perturbing it.

### Claims

**CM1 — A claim in a boundary is judged over the boundary's family, not at its inputs.** The claim's
rows are stacked on `F` over the columns of **both** `y` and `x` (the inputs free, as the parent's
unknowns are in the flat diagnosis), at the boundary's value, and judged as `diagnose_with` judges
today: *theorem*, *consuming* or *violated*. With the inputs held fixed instead, the judgement would
be empty. `F_y` is square and nonsingular (FR1, DT1), so every row over `y` is in its span, and
every claim that holds would be a "theorem". Judged over `(y, x)`, a theorem means what it should:
the body implies the claim at every input near this one, whatever the parent does. *Held by:* at
unequal radii, the belt drive's tangents meeting on the line of centres is a theorem of the drive. A
claim true only at the instance's inputs (that the centres are 120 apart) is *consuming*. With the
inputs held, both would read *theorem*.

**CM2 — One verdict per distinct solve.** A claim's verdict is a property of the fresh value of a
key. It is judged once per key, invariant under the pose for an equivariant claim, and reported
under every instance's name. *Held by:* N instances of one key give N reports of one judgement,
and the judgement runs once.

**CM3 — A claim never restricts a boundary's domain.** `Ω` and every failure of §9 are decided by
the acting rows and root choices alone. Witness geometry built only for a claim is solved after the
boundary, against its value held, and only for the claim (#460's `claim { … }`). A witness that
cannot be built fails the claim, which is reported as not applicable, never as the boundary. Until
#460 lands, geometry written outside a claim is acting geometry, and restricts `Ω` as any row does.
That is #460's case: the belt at equal radii, whose witness `x` has nowhere to stand. *Held by:*
after #460, the drive at equal radii evaluates, and its claim is reported not applicable.

---

## 12. The rules and the tests that hold them

Every test is a Rust test in `rust/gcs-core/tests/`, in a file listed in `tests/main.rs`, unless it
says otherwise.

| rule | held by | built in |
|---|---|---|
| CL1 | one instance in two documents (one 1000× larger) agrees (held, to the inner tolerance); evaluation after unrelated history is bit-identical | #454; the history half #456 |
| CL2 | listed inputs equal the lowered rows' columns; no column outside `x ∪ y` (held) | #452, #454 |
| CL3 | an unbound `Int` formal is "not a boundary" | #452 |
| DT1 | the two-root point is DT1; with `b` on `a` it is not | #452 |
| DT2 | the two-root point is refuted without `ccw`, unrefuted with it (after #157) | #452, #157 |
| DT3 | the spike's verified boxes; sampled `Ω` for each DT3 boundary | #453 |
| DT4 | N instances of one key in shuffled orders give bit-identical canonical values | #456 |
| DT5 | continuation keeps a branch across a basin boundary and fails at the fold | #455 |
| DT6 | a walk from one basin to the other reports the mismatch | #455 |
| FR1 | a body with a row over its operands alone is "not a boundary", naming it | #452 |
| FR2 | `Crank(o, theta: hint(…))` is DT1; with the angle the body's own unknown it is not | #452 |
| FR3 | no boundary's derivative is minimum-norm: `F_y` square in every DT1 test (held: a boundary not square is refused) | #454 |
| EQ1, EQ2 | `Apex` with a page seed fails the motion and spin checks; with a relative seed it passes | #452, #456 |
| EQ3, EQ4 | key equality under random proper motions of the operands; no rounding in keys | #456 |
| EQ5 | `Pulley(centre)` invariant under the free rotation; a keyway there fails DT1 | #456 |
| EQ6 | N instances' `upper.run.p1` each match the flat solve | #456 |
| IO1 | a `private` member read from outside is E101 | #454 |
| IO2 | one over-constraint, flat and through a boundary, named at the parent | #455 (the prototype runs no diagnosis) |
| IO3 | a parent's opposite `ccw` over `one.x` is reported broken; a `Side` input chooses | after #157 |
| IO4 | two children reading each other's outputs are one parent system (held) | #454 |
| DD1 | finite-difference check of every output against every input (held, outputs the boundary's own coordinates) | #454 |
| DD2 | `x² = u` fails *singular* above 0 and the parent stops short | #455 |
| DD3 | a fold crossed by the inputs fails *branch changed* | #455 |
| DD4 | nested circles fail *outside* without a Newton step | #455 |
| DD5 | the belt loop near `c = r₁ + r₂` converges with no evaluation outside `Ω` | #455, #458 |
| FA1–FA4 | one boundary per reason; the parent's residual is not finite; the report names the instance and the parent's rows | #455 |
| FL1–FL3 | flat and boundary agree; flat from the boundary pose does not move; the rank identity (held) | #454 |
| DP1 | the derivative is exactly zero outside `D(o)` | #457 |
| DP2 | moving a drive's shafts together re-solves nothing; `belt_length` bit-identical | #457 |
| DP3, DP4 | the belt's counted evaluations; perturb-and-restore is bit-identical | #457 |
| CM1 | the tangents' claim is a theorem of the drive; a pose claim is consuming | #455 (not built by #454) |
| CM2 | N instances of one key, one judgement | #456 |
| CM3 | the drive at equal radii evaluates and its claim is not applicable | #460 |

## 13. Worked cases

| case | inputs | tier | EQ | notes |
|---|---|---|---|---|
| a point 5 from `a` and `b` | `a`, `b`, carrier | DT1; DT2 with `ccw` after #157 | with a relative seed | two roots, related by the half-turn that turns the carrier over (EQ5) |
| `CommonTangent(a, b)`: the left external tangent | two circles, carrier | stated by `tangent(at:)` at both ends: DT3 only where the circles clear each other; where they cross, `s = e` at the crossing is a second root (#453). Stated with the radii parallel: DT3 by computation on the boxes #453 tried, across `Ω`; both once its `ccw` is checked (#157) | yes | at `Ω`'s edge the outer tangents merge and the tangent points' derivatives grow without bound (DD2) |
| `Pulley(centre)`: rim and bore | a point, carrier; `r`, `bore` | DT3 trivially (linear) | yes, invariant about the centre | key `(r, bore)`; the canonical axis is the carrier's `u` (EQ3) |
| `Crank(o, theta)` | a point, carrier; `theta` unbound | DT1 with `theta` an input | yes | `theta` is the parent's unknown (FR2) |
| `x² = u`, `x ≥ 0` | `u` | DT3 on `u > 0` | n/a | singular at the edge (DD2) |
| `CircleThrough(p1, p2, p3)`, `r > 0` | three points, carrier | DT3 by computation within a stated `S` (#453); without the bound, two roots `±r` | yes | the cost grows steeply as the points near a line, and the verifier gives up there |
| `Apex` with `hint((3, 4))` | `a`, `b`, carrier | DT1 | **no**: reads the carrier's frame | §6's run |

## 14. What the contract does not promise

* **Total work.** A parent summing 1,000 distinct children's outputs makes 1,000 evaluations per
  iteration (the review's §3). The contract bounds what one parent step factorises, through
  squareness and the derivative of DD1, and lets reuse follow dependencies (DP3). #459 measured
  the rest (§18): with a dense parent, one step's factorisation is the whole cost.
* **Uniqueness below DT3.** DT1 is isolation. DT2 is the author's claim, refuted where it can be.
* **That a boundary beats the flat solve.** The flat solve already decomposes the graph it is given
  (`decompose`, `block_order`). Whether boundaries pay, and where, was #459's question; on the
  prototype they did not (§18).
* **Solids.** DD6.
* **Anything about syntax.** `given`, `context`, binding by value and a domain clause are #461's,
  after #459.

## 15. Open questions

* **Mirrors.** A declared mirror would share one key between an instance and its reflection,
  reading every orientation reversed. Is it worth a rule, or is a mirrored part simply another key?
* **Context, today.** Closed scope admits no context, so every value a boundary reads is a formal
  bound by value, and the key holds it. Whether N7's `context` is needed at all depends on how much
  of the corpus threads a standard (#452's corpus report, the paper's §2.3).
* **`τ` and own scale.** DD2's threshold and CL1's own scale need numbers. #455 should choose them
  against the corpus's well-posed components, not against synthetic cases alone.
* **Inferred boundaries.** If every rule above can be checked from the elaborated component (CL2,
  FR1, DT1, EQ2 are all mechanical), a parent might treat a qualifying instance as a boundary with
  no mark in the source. #461 decides that, on the benchmark's evidence. Proposed for review in
  [parts-plan.md](parts-plan.md): declared (`part`), checked while elaborating, solved flat.
* **The locus's own contract.** The locus would meet most of this with a condition estimate (DD2),
  a determinant sign (DD3), no nearby value on failure (FA1), and its module comment corrected to
  march from the anchor. #454's call: the comment is corrected and the square Newton is shared
  (`newton::square`), but the determinant sign and the refusal of a nearby value land only in the
  boundary built on it (§17). A trace keeps its march and its fallback; changing what a contact
  reads on a failed solve is a change to every traced document, and is left to be decided
  against them.

## 16. The classifier (#452)

`boundary::classify`, printed by `solventc --components`, reads each component through its first
instance at the document's solved pose and says which of the tiers it reaches. It is advisory, as
#452 asks, and checks what it can without the prototype:

* **its own unknowns** are the numbers of what the instance made (every entity sited under its
  path, `SourceMap::instances`), with the numbers no entity owns that its rows read (a contact's
  place, a lift). Its **inputs** are every other column its rows read. An unbound numeric formal
  is an input, reported as exported (FR2), and an unbound `Int` formal as "not a boundary" (CL3);
* **DT1** is the rank of its rows over its own unknowns, the inputs held, at the instance's inputs
  and at `PERTURBED` inputs moved from them by a seeded draw. The rank at the document's scale, not
  the boundary's own, until #455 sets one (CL1);
* **FR1** is the rank its rows add with the inputs freed as well. A body that declares nothing a
  parent reads and states such facts, or holds an operand's numbers by `fix`, **asserts only**;
* **DT2** is refuted from `STARTS` starts scattered at twice the instance's size, each a short probe
  solve. Roots are compared by the figure (points in space, radii, axes), and a root the body's own
  `ccw`/`cw` refuses is counted apart: a probe is steered by none, though a document's solve is
  held to them (#157);
* **EQ2** is read from the text and the rows, not by moving anything: a row over a plane's own axes,
  a `fix` at coordinates, a seed in coordinates or by a bearing, and a coordinate read.

`tests/boundary.rs` holds one component of each class, the two-root point among them.

## 17. The prototype (#454)

`solve::solve_through` solves a document through instances named from Rust (`solve::Boundary`,
built by `boundary::instance` from the classifier's own reading of an instance), with no syntax.
The rules it keeps are [`contract/boundaries.md`](contract/boundaries.md); the gate is
`tests/boundary_solve.rs`. In short:

* **On the locus's machinery.** Each boundary's block is a `Subset` of the flat document's one
  `System`: no second compile. It is solved by the locus's square Newton, moved to
  `newton::square` and shared with `locus.rs`, and differentiated by the implicit function theorem
  from one LU of `F_y` (DD1). Each evaluation after the first resumes from the last value accepted,
  started where its tangent predicts, and is checked by `newton::continues` and by the sign of
  `det F_y` (DT5, DD3).
* **The parent** is a `TrustRegion` over its own unknowns, minimised by `newton::dogleg`: its rows
  read at `(z, y(z))`, its Jacobian `P_z + P_y·dy/dz`, chained through boundaries that read one
  another in order (IO4). A failed evaluation is NaN rows, which the DogLeg rejects (FA1, FA2).
* **What it shows.** The direction paper's belt drive (§6.1, turning frames left out) with both runs
  as boundaries, the two-root apex placed through its output, two apexes chained, and the common
  perpendicular of two skew lines in space each solve to the flat answer to 1e-9 (1e-8 in space),
  with the rank identity of FL3; each boundary's derivative passes a central difference. Asked to
  nest the pulleys, the parent stops at the edge of the runs' domain and names the run that held
  it.
* **What it does not do.** Judge at the boundary's own scale (CL1: the document's, until #455);
  estimate a condition or refuse below `τ` (DD2), state a domain (DD4, DD5) or check path
  independence (DT6), all #455's; start a fresh evaluation from seeds stated relative to its
  inputs or share a value between instances (DT4, #456); run the diagnosis through boundaries
  (IO2, CM1); enforce a root choice (#157). It is a solve, not a fast one: #459 measures it (§18).

## 18. The benchmark (#459)

**Verdict: on every case measured, the prototype's boundaries cost more than the flat solve, and
what they cost is the parent, not the children.** The flat solve already decomposes a document of
N drives into blocks of four columns and solves it in time linear in N; the parent through the
boundaries is one dense matrix of eight columns a drive, factored at every step. A cache (#456) or
per-output invalidation (#457) saves inner solves, and the inner solves are not where the time
goes. Boundaries do find two things the flat solve does not say: a run at the edge of its domain is
named, and the solve stops in a few steps rather than a hundred. That is diagnosis, and it does
not need a boundary solve.

**How it was measured.** `cargo run --release -p gcs-core --bin bench boundaries [most]` (through
solves to `most` drives, 100 by default; the flat solve to 1000), on a 4-core 2.1 GHz Xeon
container, release build. Times are medians of 7 runs (3 at N = 30, 1 at N ≥ 100), each a compile
and a solve from the document's seeds, flat (`solve`, a document's solve) and through
(`solve_through`). The documents and counts are `boundary::measure`; the counts that do not depend
on the machine are held at small sizes by `tests/boundary_measure.rs`. *Inner solves*: Newton
solves of one boundary (`Through::evaluations`); *inner it*: their iterations, each one assembly and
LU of a block's `F_y`; *largest*: the most columns the flat solve's block-triangular order puts in
one block; *parent*: the dense Jacobian through the boundaries, rows × columns. Every through solve
below ends at the flat solve's pose, to 1e-12 for small documents and to 1e-7 at the largest (the
parent's DogLeg stops at its tolerance on rows at the document's scale).

### Repeated and distinct parts

N belt drives (§6.1, `boundary_solve.rs`'s), each with both runs as boundaries. *Identical*: N
copies standing apart. *Distinct*: the large pulley's radius differs in each, so no two runs are
asked the same question. *One unknown*: distinct, and every drive's centre distance is one declared
unknown that the first drive's crossing settles: one parent unknown read by every child.

| case | N | flat ms | flat it | largest | through ms | it | inner solves | inner it | parent |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---|
| identical | 1 | 0.2 | 4 | 4 | 0.1 | 4 | 10 | 30 | 32 × 8 |
| identical | 10 | 0.6 | 4 | 4 | 4.3 | 4 | 100 | 290 | 104 × 80 |
| identical | 30 | 1.8 | 4 | 4 | 82.7 | 4 | 300 | 720 | 264 × 240 |
| identical | 100 | 6.9 | 4 | 4 | 3,675 | 4 | 1,000 | 2,300 | 824 × 800 |
| identical | 300 | 17.9 | 4 | 4 | 647,157 † | 4 | 3,000 | 6,600 | 2,424 × 2,400 |
| identical | 1,000 | 85.4 | 3 | 4 | not run | | | | 8,024 × 8,000 |
| distinct | 100 | 4.3 | 4 | 4 | 3,678 | 4 | 1,000 | 2,299 | 824 × 800 |
| distinct | 1,000 | 88.1 | 3 | 4 | not run | | | | |
| one unknown | 10 | 0.5 | 4 | 13 | 3.1 | 4 | 100 | 248 | 106 × 82 |
| one unknown | 100 | 5.0 | 4 | 13 | 2,494 | 3 | 800 | 2,099 | 826 × 802 |
| one unknown | 1,000 | 196 | 10 | 13 | not run | | | | |

† one run, beside the 19.5 ms flat solve of the same run.

* **The parent is the cost.** Through is 7× the flat time at N = 10, 46× at 30, 530× at 100 and
  about 33,000× at 300: the time grows as N³ to N⁴, the dense parent's factorisation (and the rank
  read at the end), while the inner work grows as N. One whole drive solved through its runs takes
  0.1 ms, parent included, so the hundred drives' inner solves are at most 10 ms of the 3.7 s.
* **Nothing is shared.** N identical drives make exactly N times one drive's inner solves and
  iterations (held). Every copy is asked the same question at every step, so a placement-independent
  cache would answer all but one: 10 inner solves instead of 1,000, against 3.7 s of parent.
* **Distinct parts cost what identical ones do**, and no cache could answer one. With one shared
  unknown every run is solved again at every residual the parent asks for (held: each run's count
  is the parent's `nfev`), the case the review's §3 named.
* The flat solve's largest block stays four columns at every N (thirteen with the shared unknown,
  which welds the first drive's crossing to its shaft), and its time is linear: 85 ms for 16,000
  unknowns. Below `DENSE_MAX` (120 columns) it is dense, which is why three drives take longer
  than ten.
* Naming the boundaries (`boundary::instance`, reading the source map once an instance) is
  quadratic: 0.2 s at 100 drives, 2 s at 300.

### A closed loop

A crank and a chain of `k` two-root points, each over the last joint and its own pivot, closed by
the last joint's distance from the crank (`measure::dyads`). Flat, the closure couples the whole
chain into one block of 2k + 2 columns. Through, the parent is the crank's two unknowns over a
chain of 2 × 2 solves, evaluated in order (IO4).

| k | flat ms | flat it | largest | through ms | it | inner solves | inner it | parent |
|---:|---:|---:|---:|---:|---:|---:|---:|---|
| 4 | 0.0 | 4 | 10 | 0.1 | 4 | 20 | 41 | 26 × 2 |
| 16 | 0.3 | 4 | 34 | 0.1 | 4 | 80 | 128 | 26 × 2 |
| 32 | 1.2 | 4 | 66 | 0.3 | 4 | 160 | 248 | 26 × 2 |
| 64 | 0.3 | 4 | 130 | 0.8 | 4 | 320 | 463 | 26 × 2 |
| 256 | 1.4 | 4 | 514 | 4.9 | 3 | 1,024 | 1,143 | 26 × 2 |

This is the one case where a boundary's structure is a real one: it tears a coupled block at the
crank. Both grow linearly. Through is faster only where the flat block is dense and large (from
16 links to 59, the last under `DENSE_MAX`); above it the flat sparse solve is three times faster. The flat block pass
already tears a block at its declared unknowns (#283, `System::tearing`), so tearing a loop is
within reach of the flat solve, with no syntax. Seeded relative to the joint before, a chain of 32
fails both ways (the seeds drift onto the other root), so every joint is seeded in coordinates.

### Branch and singular inputs

Each input swept, every solve started where the last ended, flat and through alike.
*|dy/dx|* is the largest entry of the boundaries' derivative at the pose reached, and *FD* its worst
disagreement with a central difference (DD1's measure, a step of 1e-6 of each input).

| sweep | value | flat | through | \|dy/dx\| | FD |
|---|---:|---|---|---:|---:|
| centres (edge 25) | 120 | 4 it | 4 it | 1.08 | 8e-7 |
| | 25.1 | 7 it | 3 it | 20.0 | 8e-7 |
| | 25.01 | 10 it | 5 it, 1 rejected | 63.6 | 3e-6 |
| | 25.001 | 10 it | 5 it, 1 rejected | 201 | 3e-4 |
| | 25.0001 | 10 it | 5 it, 1 rejected | 636 | 3e-2 |
| | 25 | solved, 19 it | solved, 42 it, 21 rejected | (the difference crosses the edge) | |
| | 24.9 | **fails**, 100 it, 6e-4 off | **fails**, 16 it: `upper_0` branch changed | | |
| x² = u (fold at 40) | 39.9 | 6 it | 5 it, 1 rejected | 6.15 | 5e-6 |
| | 39.99 | 6 it | 5 it, 1 rejected | 19.4 | 7e-8 |
| | 39.9999 | 5 it | 5 it, 1 rejected | 194 | 7e-4 |
| | 39.99999 | 5 it | 4 it, 1 rejected | 612 | 1e-1 |
| | 40 | solved, 12 it | solved, 45 it, 22 rejected | (`one` unsettled) | |
| | 40.1 | **fails**, 100 it, 1e-3 off | **fails**, 24 it: `one` branch changed | | |
| radii equal (r₁ 20) | 20.1 | 98 it | 16 it | 1.00 | 3e-7 |
| | 20.001 | solved, crossing at −4.6e5 | solved, crossing at −2.4e6 (true: −2.4e6) | 1.00 | 3e-7 |
| | 20.0001 | solved, crossing at −4.6e5 | solved, crossing at −1.5e7 (true: −2.4e7) | 1.00 | 3e-7 |
| | 20 | **solved**, no crossing exists | **solved**, no crossing exists | 1.00 | 3e-7 |

* **Past a domain's edge, through says which boundary and stops** in 16 to 24 iterations, where
  the flat solve takes its hundred and ends off. At the edge itself both report solved: through
  only after twenty rejected steps, since nothing yet refuses a nearly singular `F_y` (DD2, #455).
* **The derivative is the implicit function theorem's to the edge**, growing as the inverse square
  root of the distance to it (√10 a decade, held at the fold). A central difference at a fixed step
  loses it first, once the step is within two orders of the distance: DD1's check, not the
  derivative, is what fails there.
* **A singularity in the parent is not the boundaries' to see.** As the radii go equal, the runs
  stay well conditioned (|dy/dx| = 1) and their crossing, the parent's, runs off. Each solve in
  turn stops short of it and reports solved: the flat solve from r = 20.001 (its crossing at −4.6e5
  where the true one is at −2.4e6, which through reaches), through a decade later. Both report
  solved at equal radii, where no crossing exists: #449, independent of boundaries.

### Invalidation: an edit to one drive

N identical drives solved, then one drive's centres moved from 120 to 125 and the document solved
again from the solved pose.

| N | flat ms | flat it | through ms | it | inner solves | runs re-solved | drives moved |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 | 0.10 | 3 | 0.07 | 3 | 8 | 2 of 2 | 1 of 1 |
| 10 | 0.39 | 3 | 2.2 | 3 | 80 | 20 of 20 | 10 of 10 |
| 100 | 6.3 | 3 | 2,847 | 3 | 800 | 200 of 200 | 100 of 100 |

The prototype keeps nothing between solves, so every run is solved afresh; and the parent's dense
step corrects every drive's residual noise, so every drive moves in its last bits (within 1e-9,
held). Per-output invalidation would save 2N − 2 inner solves, a few milliseconds at N = 100,
against the parent's 2.8 s.

### Recommendation

1. **#461: no syntax for boundaries now.** Nothing measured here pays for it. The flat solve,
   with its block-triangular decomposition, is linear and at least as fine-grained on repeated
   and distinct parts; on a loop the two are within a small factor either way, and the flat
   solve's tearing reaches the same structure without a mark in the source.
2. **#456 and #457 wait.** They optimise inner solves, which are under 1% of a through solve's
   time here. Neither can help while the parent is dense. If Track A goes on for scale, the next
   experiment is a **sparse parent**: `P_z + P_y·dy/dz` kept block-sparse (it is block-diagonal
   for independent drives) and solved by the flat solve's sparse path, measured again on these
   documents. Only then can a cache's or invalidation's savings show.
3. **Where boundaries could pay** is a component whose interior is expensive and whose interface
   is narrow: many rows or a costly kernel inside (a trace, an envelope, a solid's evaluation)
   against a few outputs, read many times with equal inputs. The corpus has few such candidates
   (#452: 8 of 158), and the locus already solves a trace's block this way. A sketch-level part,
   whose rows are as cheap as the parent's, is not one.
4. **Keep #455's diagnostic half.** Naming the component at a domain's edge, and stopping at it
   rather than spending the iteration limit, is the one thing boundaries gave here that the flat
   solve does not. It can be had from the flat solve, by reading a failing block's rows back to
   the instance that states them (the classifier's `split`), with no boundary solve. #449 stands
   on its own: both solves report an infeasible document solved.

## 19. The sparse parent (#459, after §18)

**Verdict: §18's cost was the prototype's engineering, not the architecture.** With the parent's
Jacobian kept sparse and stepped as the flat solve steps, solving through boundaries takes about
what the flat solve takes on every belt document, linear in N, and less at the largest sizes. What
§18 recommended as the next experiment is done, and it changes §18's recommendations 1 and 2.

**What changed** (`solve/through.rs`, `System::subset`, `boundary::instances`; the contract's rules
in `docs/contract/boundaries.md`):

1. **A sparse parent.** Each boundary's `dy/dz` is held over the parent unknowns it moves with
   (those it reads, and those of each boundary it reads), not over all of them, and the parent's
   Jacobian `P_z + P_y·dy/dz` is compressed rows with a structure fixed when the boundaries are
   taken out. To `DENSE_MAX` unknowns its step is the dense minimum-norm step as before, and above
   it the normal equations of its entries (`sparse::Ata`, refined), as a flat system's is. N drives
   are `20N` entries.
2. **The rank, part by part.** FL3's rank was one dense pivoted QR of the whole parent, which at
   300 drives with a shared unknown cost 37 s alone. Parts that share no column are ranked apart and
   summed. A column every part reads (the shared centre distance) is held out, and the rank of what
   no part spans is added by a block QR.
3. **A block's evaluation writes only what it reads.** Each inner Newton iteration copied the
   whole document's unknowns into the system before evaluating its four rows (`apply_z`), which is
   quadratic in N over a step. `System::subset` now writes only its rows' columns, to the same
   bits. The flat block pass shares it.
4. **Construction and naming are linear.** `Boundaries::new` found each boundary's inputs and its
   read order by scanning every row and every other boundary per boundary. `boundary::instance`
   read the whole source map per instance, which took 21 s to name 2,000 runs. Both now index once.
   Naming takes 34 ms.

**Measured** as §18, release build, one 4-core 2.1 GHz Xeon container, after the test suite (load
falling from 5 to 2 over the run), medians of 7 runs (3 at N ≥ 300). *Entries*: the parent's
Jacobian's. *Off*: how far the runs' tangent points stand from the closed form of an outer tangent
(`measure::run_error`), mm, flat / through.

| case | N | flat ms | through ms (§18) | through ms | parent | entries | off: flat / through |
|---|---:|---:|---:|---:|---|---:|---|
| identical | 10 | 0.5 | 4.3 | 2.4 | 104 × 80 | 200 | 1e-11 / 3e-14 |
| identical | 30 | 1.2 | 82.7 | 1.6 | 264 × 240 | 600 | 1e-11 / 4e-7 |
| identical | 100 | 6.1 | 3,675 | 8.4 | 824 × 800 | 2,000 | 4e-11 / 4e-7 |
| identical | 300 | 23.0 | 647,157 | 20.3 | 2,424 × 2,400 | 6,000 | 1e-11 / 4e-7 |
| identical | 1,000 | 90.7 | not run | 77.9 | 8,024 × 8,000 | 20,000 | 3e-5 / 3e-6 |
| distinct | 100 | 4.0 | 3,678 | 4.9 | 824 × 800 | 2,000 | 7e-11 / 6e-7 |
| distinct | 1,000 | 84.0 | not run | 87.2 | 8,024 × 8,000 | 20,000 | 8e-5 / 4e-6 |
| one unknown | 100 | 4.4 | 2,494 | 5.4 | 826 × 802 | 2,106 | 1e-6 / 8e-9 |
| one unknown | 300 | 16.6 | not run | 18.1 | 2,426 × 2,402 | 6,306 | 3e-5 / 5e-9 |
| one unknown | 1,000 | 184 | not run | 90.1 | 8,026 × 8,002 | 21,006 | 3e-1 / 9e-5 |

An edit to one drive of N, solved again from the solved pose: 4.2 ms through against 3.7 flat at
N = 100 (2,847 in §18), and 86 against 105 at N = 1,000. The loop and the sweeps are §18's, to
the iteration: the change is in the parent's linear algebra, not its steps.

* **Through is within a small factor of flat everywhere, and linear.** Below `DENSE_MAX` parent
  unknowns (ten drives) it still takes the dense step and is slower (2.4 ms against 0.5), as a
  flat system under `DENSE_MAX` is slower than a larger sparse one (§18). The inner solves are now
  a real share of the time: at N = 1,000, 10,000 of them in 78 ms.
* **"Apart" is not "off".** §18 read the two solves' disagreement as the through solve's error.
  Against the closed form, both solves hold every row to about 1e-12, and a tangency holds its
  point only to the square root of its residual, which here is judged at the document's scale (400 m
  wide at N = 1,000). Through is off by about 5e-7 mm from N = 30, the floor of its inner tolerance
  (`SquareTol::BLOCK`, unchanged). Flat is off by 1e-11 to 1e-9 to N = 300, and by more after.
  At N = 1,000 with the shared unknown, flat stops with every tangent point 0.27 mm out, where
  through is within 1e-4.

**What this changes in §18's recommendations.** (1) The case against #461 was the parent's cost,
and that is gone; #461 is open again, to be decided on what boundaries give (reuse, invalidation,
diagnosis) rather than on a cost they no longer carry. (2) The inner solves are now a large share
of the through solve's time, so #456 (a cache over identical drives answers 9,990 of 10,000) and
#457 (an edit re-solves 2,000 runs to move one drive) have something to save, and the next slice
of #459 measures them. (3) and (4) stand.

## 20. Where the time goes (#459, after §19)

**Verdict: a perfect cache of evaluations (#456) would save at most a fifth of a solve through
boundaries at 1,000 drives, and less after an edit.** With the parent sparse (§19), the time is
spread across compiling, taking the boundaries out, the parent's step and the inner solves, and the
inner solves are the smallest of the large ones.

**How it was measured.** `Through::times` times each phase of `solve_through` with
`clock::Instant`; the phases do not overlap, and `other` is what the whole holds beyond them (the
DogLeg's own vector work, the flat rows judged at the end, the write back). `bench boundaries`
prints the medians of at least 5 solves of each document, on the machine and documents of §19,
release build, load under 3.

| case | N | compile | setup | inner | jacobian | rows | step | rank | other | total | inner % | µs / inner solve |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| identical | 10 | 0.07 | 0.15 | 0.17 | 0.03 | 0.00 | 1.70 | 0.03 | 0.01 | 2.18 | 8% | 1.75 |
| identical | 100 | 0.55 | 1.27 | 1.35 | 0.11 | 0.03 | 0.63 | 0.22 | 0.12 | 4.29 | 31% | 1.35 |
| identical | 1,000 | 8.74 | 17.03 | 13.80 | 1.89 | 0.28 | 21.07 | 3.12 | 1.23 | 68.92 | 20% | 1.73 |
| distinct | 1,000 | 8.75 | 15.82 | 13.88 | 1.77 | 0.28 | 21.98 | 3.00 | 1.17 | 66.78 | 21% | 1.73 |
| one unknown | 1,000 | 7.95 | 16.79 | 15.99 | 2.18 | 0.35 | 24.66 | 3.95 | 1.49 | 73.02 | 22% | 1.60 |
| edit to one drive | 100 | 0.53 | 1.35 | 0.75 | 0.09 | 0.02 | 0.62 | 0.22 | 0.11 | 3.74 | 20% | 0.94 |
| edit to one drive | 1,000 | 8.60 | 17.47 | 7.50 | 1.41 | 0.23 | 20.92 | 2.97 | 1.15 | 62.49 | 12% | 1.25 |

Milliseconds. *Compile* is `System::new`, which a flat solve does first too; *setup* is the rest
of `Boundaries::new` (each boundary's block, the order they are evaluated in, the parent's rows and
its Jacobian's structure); *inner* is every evaluation of a boundary: its Newton solve, its
derivative by the implicit function theorem, and the evaluations answered again.

* **The inner solves are 20% of the time at 1,000 drives and 31% at 100**, about 1.5 µs each (a
  4 × 4 Newton of two or three iterations and its derivative). A cache that answered every one of
  them but the first, at no cost of its own, would take 69 ms to about 55 at N = 1,000. A real
  cache keys each evaluation by its inputs up to a rigid motion (#456) and must look up in well
  under 1.5 µs to save anything. For parts this cheap the cache is not where the time is, which is
  §18's third recommendation measured: boundaries pay for an interior that is expensive behind a
  narrow interface, and a belt run is neither.
* **The largest phases are boundaries' own machinery, not their evaluation.** Taking the
  boundaries out costs twice the document's compile (17 ms against 9 at N = 1,000), and the
  parent's step on its normal equations (21 ms) is as much as the flat solve spends whole beyond
  its compile. Both are engineering of the kind §19 did, open to the same treatment, and neither
  is touched by #456 or #457.
* **Below `DENSE_MAX` the step is the cost.** At ten drives the dense minimum-norm step is 1.7 of
  2.2 ms; that is the whole of the gap to the flat solve's 0.5 ms (§19).
* **After an edit the inner share falls to 12%**: only the edited drive's runs move far, and the
  rest converge in a step. What invalidation (#457) could save is not the inner solves but the
  untouched drives' share of every phase, including the compile, which needs the parent solved part
  by part and the document compiled incrementally; the flat solve's block order could be given the
  same, so #457 does not distinguish boundaries.

**For the next slice.** #456 should not be built for sketch-level parts: its ceiling is a fifth of
the time here. If boundaries are to pay, they do it where an evaluation is expensive (a trace, an
envelope, a solid's evaluation, §18's third point), and the next measurement is one of those, not
more belt drives. On belt drives, the cheaper gains are the setup and the step.
