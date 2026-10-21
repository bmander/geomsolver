# Solvent's direction: statements, nouns and verbs

**Status (2026-10-20, revised): a discussion paper, nothing adopted.** It sets out where the
language stands, what has gone wrong with it, and two independent proposals: a style for
statements, judged on readability, and reusable solve boundaries, to be established by
experiment before any grammar is built around them. One example is carried through every stage.
The syntax in §5 and §6.3 is proposed and does not parse; everything shown as current was run
through `solventc`. This revision answers an adversarial review of the first version (see *What
changed after review*, below); the plan it leads to is epic #465.

## Summary

Solvent began as a list of facts, and the best of it still reads like a proof: `Af distance(width)
Bf`, `Ef project Et`, a `claim` the diagnosis judges a theorem. Under steady growth, much of it
written by coding agents, it has drifted toward the idiom of an object-oriented program —
components called with long labelled argument lists, configuration records threaded through call
trees, and procedures called for their effect. Separately, the engine takes no advantage of the
language's own structure: every component is inlined into one flat system and the decomposition
is rediscovered from the graph, so nothing in the source says which parts of a large assembly
can be solved apart, and identical parts are solved again each time.

These are two problems, and this paper treats them as two:

* **Style** (§4). Statements that read aloud as sentences of a geometric argument, relations
  written as relations rather than procedure calls, numbers that travel with the thing they
  measure. This is a goal in its own right, judged on readability. It does not follow from the
  scalability argument and is not needed by it.
* **Reusable solve boundaries** (§5). A component that builds something its inputs determine —
  a *noun*, in the sense of a function symbol of first-order logic, as against a *verb*, a
  predicate — could be solved once per distinct input, in its own frame, and consumed by its
  parent through its interface. Whether that pays, and under what conditions, is a question for
  an experiment (§8): a contract, an advisory classifier, one boundary built on the machinery
  `locus.rs` already has, and a benchmark against the flat solve. Only then syntax.

What a boundary can promise is narrower than the first version said. It bounds the size of the
coupled system each Newton step factorises, under stated structural conditions; it lets reuse
follow dependencies; it does not bound total work, and caching needs more than closure. §5 says
what it needs.

## What changed after review

The first version of this paper overstated several guarantees. An [adversarial
review](https://github.com/bmander/geomsolver/issues/448#issuecomment-6303715169) checked them
against `main` at `0bb8c14`; every executable claim in it reproduces, and was re-run for this
revision. What it changed:

* **Total work (C4).** Bounded per-level system size does not bound total work: a parent summing
  1,000 distinct child outputs makes 1,000 child evaluations per iteration. C4 now promises a
  bounded coupled system under stated conditions and dependency-driven reuse, and total work is
  measured separately.
* **Uniqueness (N2).** A full-rank Jacobian at a generic input shows local determination, not a
  single-valued function: a point 5 from each of `(0, 0)` and `(6, 0)` solves to `(3, 4)` or
  `(3, −4)` by its seed, both with numeric rank 2 and DOF 0. N2 now has three tiers, and the
  checker is advisory. Checking the second tier for this revision found that `ccw` on the sheet,
  which §6.3 relied on to choose a root, is today neither enforced nor checked (#157).
* **Equivariance and invalidation (N3, N9).** Caching up to placement needs rigid-motion
  equivariance, not only closure, and invalidation must be tracked per output: §6.4's claim that
  a new fit class leaves the solve for `c` a hit contradicted `BeltDrive`'s own cache key.
* **A numerical contract (N8).** The implicit function theorem needs domains, conditioning
  checks and failure propagation stated, smooth sketch outputs before solid measurements. It
  cites `locus.rs`, which already solves an inner block and differentiates through it.
* **The belt (§6).** At equal radii the runs are parallel and the witness `x` cannot exist, so a
  construction made only for the theorem restricted the model. The domain is now stated, the
  theorem conditional, and its witness scoped to the claim (#460). The equal-radius document
  also exposed a current solver bug, #449.
* **Style versus architecture.** Read-aloud statements, binary relations and "define by
  property" are kept as their own goal; `ccw` is ternary, and a coordinate formula can be
  cheaper and better behaved than the constraints that replace it.
* **The ratchet (R10)** is replaced by a lint of semantic patterns (#462).
* **Corrections.** `interface Shaft` beside `component Shaft` is E071 (the component is now
  `Rod`); `gear_trace.sv` already defines the involute by its property (§2.4); typed interfaces
  already exist (spec §8.1, 0.84).
* **Next steps (§8)** are reordered experiment first: contract, advisory classifier, one boundary
  on the locus machinery, a benchmark against the flat solve, and only then syntax.

---

## 1. The language today

### 1.1 What it is

Solvent is declarative: a document declares entities and states relations among them, and the
solver finds coordinates satisfying every statement at once. The spec's design principles
(`solvent-spec.md` §1.1) are already those of a theorem rather than a program:

* **P1** — connection is aliasing; constraint is explicit.
* **P2** — component bodies are unordered: a body is a set of statements, and reordering it never
  changes its meaning.
* **P3** — hints are semantically inert: deleting every seed leaves the solution set unchanged.
  (The solution *set*: which member of it a solve returns is the seed's business.)
* **P4** — component boundaries are decomposition structure: "solve interiors against the
  formals; solve the inter-component system over them" (a SHOULD).
* **P5** — symmetry is a claim, not a macro: a `ring` is solved in its fundamental domain.

### 1.2 What already reads like a proof

Much of the language is already sentence-shaped, and the recommendations below mostly generalise
rules it has:

* **Relations are operators.** Every constraint is a prefix or infix word: `horizontal l`,
  `a distance(80) b`, `l1 angle(30deg) l2`, `p coincident c` — and the orientation predicates
  keep a call, `ccw(a, b, c)`, because they are about a triangle, not a pair (spec §9.2).
* **Brackets say what a thing is made of.** The primer's rule: write `c := circle(center: o)
  hint(r: 25)`, never `circle(center: o, r: 25)` — the centre is structure, the radius a guess —
  and state a held number with `radius(25) c` or `fix(r == 25) c`.
* **Relation words and sets are predicates** (§9.9, §6.21): `a horizontal b := a level(up) b`;
  `{ p | p distance(r) center }`.
* **Claims are conclusions.** `claim` states a consequence without enforcing it, and the diagnosis
  says whether it is implied — a theorem — or merely true here.

`rust/examples/bracket.sv` reads almost as a draughtsman's proof:

```solvent
in std.front {
  horizontal (ab := line(Af, Bf)) -> vertical (bc := line(Bf, Cf)) -> …
  Af distance(width) Bf
  Bf distance(base) Cf
  Af distance(leg, along: u) Df
}
in std.top {
  At distance(depth, along: v) A2t
  Bf project Bt
  Ef project Et
}
```

and `rust/examples/generation/spur_mesh.sv` ends with a claim — that the centre of curvature of a
generated tooth lies on the base circle — which the diagnosis judges a theorem.

### 1.3 How the engine treats structure

Elaboration (§14.1) inlines every component body into the root's instance tree, resolves aliases,
substitutes values, and collects one flat constraint store. The solver then decomposes that flat
graph (a Fudos–Hoffmann-style decomposition and a Dulmage–Mendelsohn block order) and caches the
plan per topology (`docs/implementation-status.md`). That is fast at the sizes the corpus has — a
600-entity truss replays in 0.39 ms — but P4 is not implemented as such: the instance tree plays no
part in the solve, identical instances are solved separately, and decomposability is something
discovered in the graph rather than declared in the source.

There is one exception, and it matters for §5. A **locus** (`rust/gcs-core/src/locus.rs`) is a
curve defined by the constraints that force it: `Unwind(…).p over u` in `gear_trace.sv`. Each
evaluation is a small Newton solve of the component's block against its inputs, and its
derivatives with respect to those inputs come from one factorisation of that block, by the
implicit function theorem. Its branch is picked by seeds and carried by continuity, with a
fallback march from the low end of the domain when a resumed step cannot be checked. That is a
reusable solve boundary in miniature, already built and tested.

Separately, §1.2 of the spec stratifies the model: the sketch solves, then solids are evaluated,
"and there is no edge back". No solve may read a volume, a mass or any evaluated measurement.

---

## 2. Flaws

### 2.1 Drift into call syntax

Counted over the 183 `.sv` files in `rust/examples` and `rust/lib` (the patterns are in the
appendix):

| idiom | count |
|---|---|
| `name: name` puns (`pinion_teeth: pinion_teeth`) | **641** |
| instance calls, `X := Comp(…)` | 487 |
| statements with three or more labelled arguments | 382 |
| component definitions | 204 |
| relation-word definitions | **19** |
| `group` formals — configuration records passed down | 46 |
| bare component calls standing as statements | 73 |

The counts describe the corpus; they are not a measure of drift to hold CI to (R10 says why).
The language has grown by components (204) far faster than by words (19), and the components
take their configuration as argument lists. The worst cases:

```solvent
pair := HypoidPair(std.top, std.y, std.z, pinion_teeth: pinion_teeth, gear_teeth: gear_teeth,
  mean_module: mean_module, shaft_angle: shaft_angle, offset: offset,
  spiral_angle: spiral_angle, backlash: backlash, tip_relief: tip_relief, end_relief: end_relief,
  pressure_shift: hint(10deg), cutter_radius: hint(80mm))

mount := {journal: 5mm, clearance: 0.2mm, play: 0.15mm, root: 4mm, keys: 3, key_w: 2.5mm, …}
                                        // thirty fields, handed whole to `Mount` and `Frame`
```

### 2.2 Procedures disguised as components

Of the 73 bare calls, 67 are `std.Turn`, `std.Spin` and `std.Slide`. They declare nothing; they
are called for their effect:

```solvent
counter := frame
std.Spin(counter, about: centre, on: std.top, ratio: -1)
```

and the definition of `Spin` is three constraints, one of them a pun (`fix(ratio == ratio, …)`).
`std.CircleOnSphere(k, ball, std.side)` is the same: a component whose body only asserts facts
about what it is handed. These are relations written as procedure calls.

### 2.3 Configuration threaded through call trees

The 641 puns and 46 group formals are the object-oriented configuration object: a number is
stated once at the top and passed explicitly through every level that does not read it, on the
way to one that does. `engine/conrod.sv` says `use engine.dims` and also takes `dims: group`. The
V-twin's `ConRod` takes `draw_end: Int`, `draw_side: Int` and `draw_sec: Int` — presentation
flags, which by the model/drawing split belong in `.svd`.

### 2.4 Formula over property

`gear.sv`'s comment defines the involute properly — the end of a taut string unwinding from a
circle, square to the radius where it leaves it — and then the code writes coordinates:

```solvent
component Involute(c: circle, phase: Angle, u: Angle) {
  p := point(x: c.center.x + c.r * (cos(u + phase) + u / 1rad * sin(u + phase)),
             y: c.center.y + c.r * (sin(u + phase) - u / 1rad * cos(u + phase)))
}
```

The language can already say it the other way. `rust/examples/gear_trace.sv` is the same gear
with the involute *described*: `Unwind` places a point on the base circle at a bearing from a
datum, a string square to the radius there, and the string's far end as far along it as the arc
unwound, and the flank is `trace(Unwind(base, datum, phase: phase).p over u in (u0, u1))`. No
formula for an involute appears in it.

The two are not equal in cost, and the formula is not simply the worse one. It is closed,
deterministic, differentiable and cheap; the property is a Newton solve per evaluation, and it
has branches — `gear_trace.sv`'s own comments record a start that reached the mirror branch of the
string, a tooth flaring the wrong way, until the curve's interval refused it. Which form a part
should use is a judgement about readability against cost and behaviour, and §4's R6 is written as
a preference with that in mind.

### 2.5 Seeds as arithmetic

Seeds placed by coordinate arithmetic clutter the argument and tie a part to the page:
`engine/conrod.sv` has dozens like `sbl := point hint((pin.x - 15mm, pin.y + 26mm))`. The language
already has seeds that name a place — `hint(at: k, bearing: 90deg)`, `hint(at: a, toward: b, by:
0.5, turn: 90deg)` — but they are the minority.

### 2.6 Inconsistencies found in the rewrite

Rewriting one example (§6) in the style this paper recommends ran into three refusals, each a
small inconsistency in the language as it stands:

* **E004** — a relation word's measure must be labelled: `driver turns(1) shaft` is refused,
  "given by label: `turns(k: 1)`", where the built-in `distance(80)` takes its number bare.
* **E101** — a relation word reads only its operands and parameters, so a word cannot name the
  plane its operand is drawn in (`f.axis perpendicular std.front` in a word body is refused).
* **E040 / E103** — `fix` reaches an entity's own numbers but not an instance's (`fix(width ==
  10mm) small` is refused), and a solid's extent cannot read a number given from outside.

The first two are #463.

### 2.7 Structure is not a property of the source

Because components are inlined (§1.3):

* one relation that reaches into a subassembly's internals silently merges two blocks of the
  solve, and nothing reports it;
* a thousand identical fasteners are a thousand solves;
* no part can be a function of its inputs, because a part's points alias global points and its
  geometry is solved in world coordinates;
* a requirement that depends on an evaluated measurement — a payload's delta-v depends on the
  stage's mass, which depends on its tank's volume — cannot be stated, because there is no edge
  back from solids to the solve.

None of this matters for a bracket. Whether it matters for a large assembly, and by how much, is
what the benchmark of §8 is for: the flat solve already decomposes the graph it is given, and a
boundary has to beat that, not an unstructured solve.

---

## 3. Constraints on a revision

A revision must keep P1–P5. The constraints below are in two groups that stand or fall
separately: C1 and C2 are about how a document reads; C3–C5 are about solving, and are
hypotheses to test (§8), not properties the grammar delivers.

### 3.1 Style

**C1 — Theorem-native, not procedural-native.** A document is a list of statements, each readable
aloud as one sentence of a geometric argument: "AB is perpendicular to CD", "let M be the midpoint
of AB", "hence X lies on the line of centres". A construct that reads only as "call F with these
arguments" fails. Order carries no meaning (P2); a document may still *read* in the order of a
proposition — given, let, such that, hence.

**C2 — A noun/verb split.** Words are of two kinds:

* a **noun** builds a thing — "the common tangent of A and B" — and is *of* its operands;
* a **verb** states a fact — "A is tangent to B" — and is *between* them.

A set is a predicate, so it sits with the verbs; that unification (§9.9) stays. A verb's arity is
not fixed: first-order predicates may have any, and the language's own `ccw(a, b, c)` has three.
A verb may also introduce auxiliary geometry that exists but is not unique ("there is a point on
c such that…") without becoming a construction. What the split matters for architecturally is
only C3: whether a component is a candidate solve boundary. Binary verbs are a readability
preference (R2), not part of that.

### 3.2 Solving

**C3 — A boundary is a function of its inputs.** A component a parent treats as a reusable solve
boundary must be:

* *closed* — its figure depends only on what it declares it reads, including any context;
* *determined*, to a stated tier (N2): locally determined, its branch explicitly chosen, or
  verified unique over a stated input domain, apart from freedoms it declares and exports;
* *deterministic* — the same inputs, with seeds stated relative to them, always give the same
  pose; this is what a cache needs;
* *continuous on its domain* — as its inputs vary inside a declared domain, the chosen branch
  varies continuously; this, and not determinism, is what a parent's Newton needs (N8);
* *equivariant* — moving every input by a proper rigid motion moves the result by the same
  motion (N3); this, and not closure, is what makes a cache key independent of placement.

**C4 — Bounded coupled systems, reuse by dependency.** Under stated structural conditions —
children enter their parent only through their interfaces and outputs, no relation reaches a
child's internals, and each child meets C3 — the coupled system that any one Newton step
factorises is no larger than the largest level's own unknowns plus its children's interface
variables, not the size of the assembly. Identical instances (the same shape inputs up to
placement) share a solve, and a change re-solves only the outputs whose dependencies it touched
(N9). **Total work is not bounded by this**: a parent whose one equation sums the outputs of
1,000 distinct children makes 1,000 child evaluations per iteration, however small each is. Total
work depends on how many distinct evaluations a change touches, their cost, the interface sizes
and the iteration counts, and is measured separately (#459).

**C5 — Composable, laterally and hierarchically.**

* *Laterally*: a standard — a fit class, a thread series, a fastener table — reaches every part
  that reads it, across the whole assembly, without being passed through the parts that do not.
* *Hierarchically*: a requirement at the top — an output speed, a delta-v for a payload — flows
  down as inputs; performance — a belt length, a mass — flows up as outputs; and a loop between
  them is closed explicitly at the level that owns it, under the numerical contract of N8.

---

## 4. Statement-level recommendations

These serve C1 and C2, and are judged on readability. They came first, and one of them (R4) is
revised by §5. None is required by §5, and none should be enforced by a refusal before its
benefit to readers is shown.

**R1 — The read-aloud test.** Every statement reads as one sentence. Every rule below is a way of
passing it.

**R2 — Prefer binary relations.** A statement is, where it can be, subject, verb, object and at
most one measure: `a WORD(measure) b`, or `WORD(measure) a`. More operands often mean two facts
fused, or a construction not yet named — but not always: `ccw(a, b, c)` is ternary because the
predicate is about a triangle, and a verb of three operands is better than two that each hide a
third. A user word's measure is unlabelled, as a built-in's is (§2.6, E004).

**R3 — A component that only asserts is a verb.** `std.Spin(counter, about: centre, on: std.top,
ratio: -1)` becomes `counter spins(-1) centre`; `std.CircleOnSphere(k, ball, std.side)` becomes
`k coincident ball`, by widening `coincident` to a curve on a set. A component call standing bare
as a statement is a lint (R10); the 67 motion calls are #464's migration.

**R4 — Brackets for structure, numbers stated after.** *Revised by N5:* the first form of this —
`fix(r == 20mm) small.rim`, reaching inside an instance — makes a component neither closed nor
determined by its inputs. What survives is that a noun's numbers are bound by statement rather
than by argument list, and only its declared inputs are.

**R5 — A closed table of prepositions.** A small fixed set — `in`, `on`, `about`, `along`, `over`,
`under`, `from`, `to`, `through`, `seen from`, `by` — each with one meaning everywhere, replaces
labelled qualifiers: `envelope(apex over t in (0deg, 1080deg), seen_from: counter)` becomes `bore
:= envelope of apex over t in (0deg, 1080deg) seen from counter`. The language already does this
with `in REF`, `ring N about C` and `over t in (a, b)`. New syntax may add a preposition only by
adding a row.

**R6 — Prefer a property to a formula, with measured exceptions.** Where a property reads better
than the coordinates that satisfy it, state the property, as `gear_trace.sv` does. But a formula is
closed, deterministic, cheap and differentiable, and the constraints that replace it add unknowns,
branches and a chance of not converging (§2.4). A formula stays where it is measurably cheaper or
better behaved, and says in a comment what property it computes.

**R7 — A document reads as a proposition.** Given (inputs), let (declarations), such that
(relations), hence (claims). P2 keeps this a convention and a printer order, not grammar. The
payoff is in diagnosis: a report that numbers statements and cites them — "(7) and (12) both hold
`sm.y`" — reads as a proof check.

**R8 — Seeds name a place.** Prefer `hint(at: …, toward: …, turn: …)` to coordinate arithmetic. A
noun's seeds are stated relative to its operands, which is what makes it deterministic up to
placement (N3). Seeds do not choose a branch in any sense a cache can rely on (P3); a root choice
is a predicate (N2).

**R9 — Grow by vocabulary, not by options.** A capability arrives as a word, written in Solvent in
`std.sv` where possible, rather than a new label on a built-in. A proposed construct is shown read
aloud, shown not to be an existing word with a wider meaning, and names the preposition it uses.

**R10 — Lint semantic drift, not totals.** The first version proposed a ratchet on the counts of
§2.1, failing CI when one rose. That measures the wrong thing: a legitimate new example raises the
counts, and moving labelled arguments onto separate lines lowers them without reducing any
coupling. Instead, an advisory `solventc` lint names patterns that are drift by definition (#462):
puns in a call; a bare component statement; a component that declares nothing and only relates its
formals; a path reaching more than one level into another instance; an `Int` formal used only to
choose what is drawn; a `group` handed on whole by a component that reads none of it. Whether CI
fails on *new* occurrences is decided per pattern, against an allowed list that can only shrink,
as `tests/layering.rs` holds the layering. Readability rules (R1, R2) stay guidance.

---

## 5. Nouns and verbs as solve boundaries

These serve C3–C5. Each is a hypothesis for the experiment of §8; #451 is to state them as a
contract a test can hold.

**N1 — Two kinds of word.** A noun is a function: it builds, is *of* its operands, and answers to
a definite description ("the tangent from A to B"). A verb is a predicate: it states, and is
*between* its operands. A component whose body declares nothing and only relates its formals is a
verb, and is written as one (R3). A component that builds something its operands determine is a
candidate boundary, however small: `Crossing(a, b)` in the hypoid drive — the common perpendicular
of two lines — is a good one.

**N2 — How determined a noun is, in three tiers.** The definite article is a reading test, not a
proof: "*the* tangent from A to B" promises a function, "*a* point on c" a freedom the parent must
own. What a checker can establish is one of three things:

1. **Locally determined.** The diagnosis, run against the component's formals at a generic input,
   finds it well-constrained relative to its inputs, apart from freedoms it declares and exports
   (a crank's angle). This is what full rank shows, and it is all it shows: a full-rank Jacobian
   at one input says the solution is isolated there, not that it is the only one, nor that one
   exists for every input. Two circles meet in two points, and a point 5 from each of `(0, 0)` and
   `(6, 0)` is found at `(3, 4)` or at `(3, −4)` depending only on its seed, both solves reporting
   numeric rank 2 and DOF 0:

   ```solvent
   unit mm
   use std
   in std.front {
     a := point
     b := point
     fix((0, 0)) a
     fix((6, 0)) b
     x := point hint((3, 4))        // hint((3, -4)) gives (3, -4), with the same diagnosis
     x distance(5mm) a
     x distance(5mm) b
   }
   ```

2. **Branch explicitly chosen.** Every discrete choice is made by a stated root choice — `ccw`/`cw`
   or a bound (spec §9.6) — rather than by seeds alone, and the solver verifies it on the solution
   and fails the solve where none holds. The spec requires that verification of both kinds, but
   today only bounds and a traced body's predicates (enforced at the anchor, spec §6.5) meet it.
   On the sheet, `ccw` records a root choice for the constructive plan and is neither enforced nor
   checked (#157): adding `ccw(a, b, x)` above still returns `(3, −4)` from the seed `(3, −4)`, or
   from `(1, −1)`, with the solve reporting success. Tier 2 needs #157 first. Even then, that the
   stated choices resolve *every* ambiguity is not established by checking one input.
3. **Verified unique over a stated domain.** For every input in a declared domain there is exactly
   one solution meeting the root choices. How far this can be checked, by intervals or by
   argument, is #453's spike; for small nouns (a common tangent) it can be argued by hand.

The checker (#452) is **advisory**: it reports the tier it can establish and why, for each
component in the corpus, and refuses nothing. Two further properties are separate from the tiers
and needed by different consumers. A cache needs **determinism**: the same inputs, and seeds
stated relative to them, give the same answer every time. A parent's Newton needs **branch
continuity**: as the inputs vary, the answer moves continuously on one branch. `locus.rs` already
meets both for a traced curve — a seed at the target, a resumed step checked against the tangent
that predicted it, a march from the domain's low end when that fails — and is the model to follow.

**N3 — A noun is solved in its own frame, which needs equivariance.** Its figure is solved relative
to its operands, and where the instance stands is a pose. Its cache key is its inputs' *shape* —
the part of them invariant under rigid motion: a common tangent of two circles is keyed by their
radii and the distance between their centres — and the pose is applied after. Identical instances
are one solve and many poses: P5's fundamental domain, generalised. Closure is not enough for this.
The noun must be **rigid-motion equivariant**: moving all its inputs together must move its result
the same way. That requires:

* **No fixed world direction.** A body that says `horizontal`, `in std.front`, `std.z`, or a seed
  in page coordinates is closed and may be determinate, and is still not equivariant: it answers
  differently when its operands are turned. Its local frame must be built from its operands — a
  line of centres, a shaft's axis.
* **Handedness.** `ccw` is preserved by proper motions and reversed by reflections, so the key is
  up to *proper* rigid motion. A mirrored instance is another key, or a declared mirror of one.
* **Symmetric operands.** Operands may leave the local frame undefined: a lone circle fixes no
  direction about its centre, and two concentric circles fix no line of centres. Either the result
  is invariant under the operands' symmetry (a pulley's rim and bore are), or the noun takes a
  datum that fixes the frame, or those inputs are outside its domain.
* **Identity.** Every point, line and face of the posed instance is the corresponding one of the
  cached solve moved by the pose, so a parent's reference to `upper.run.p1` is well defined.

#456 is the cache, built only after the boundary itself (#454).

**N4 — Verbs relate interfaces, at the lowest common parent.** A verb between two nouns relates
what they export (their `interface`), and stands in the lowest instance that owns both. A deep path
— `pair.gear_cone.ax`, `gear_side.crank.handle` — couples a level to the inside of another, and
breaks C4's structural condition; it is one of R10's lints.

**N5 — Inputs are declared and bound by value.** A noun declares its inputs (`given r: Length`); a
parent binds one by definition (`small.r := 20mm`), which is substituted while elaborating and
costs the solve nothing. An input left unbound is an unknown of the parent's solve, as an unbound
formal is today. A relation never completes a noun from outside.

**N6 — A number travels with the thing it measures.** A shaft carries its diameter; a pulley *on* a
shaft reads `shaft.d`. This is the existing rule that an instance's numbers are read by its name,
used deliberately — and with it most puns have nothing left to pass.

**N7 — Lateral information is context.** A standard is a value — a table, or a function from a
nominal size to dimensions — never an unknown. A noun declares that it reads it (`context fit:
FitClass`); an assembly provides it once, lexically (`with fit := fits.H7 { … }`), at whatever
level it applies. Nouns in between do not mention it: the implementation tracks which nouns, and
which of their outputs, read which context (N9). The alternatives fail C3 or C4: threading is §2.3;
dynamic scoping breaks closure; and equating every hole to every other puts a global coupling into
the solve.

**N8 — Inputs down, outputs up, under a numerical contract.** A requirement is bound as an input;
performance is an output computed from the solved figure (`belt_length := length(upper.run) + …`).
A loop between them is an equation at the level that owns it, over that level's own unknowns, with
each child entering as a function of its inputs and its derivative taken from the child's
factorisation by the implicit function theorem — as `locus.rs` does for a traced point today. That
lifts §1.2's "no edge back" *between* nouns while keeping it *within* one. The theorem only applies
where the child's Jacobian is invertible once its gauge freedoms are accounted for, so the
boundary needs a numerical contract (#455):

* **Domains.** Each noun declares the inputs it is valid for. Outside them it is not evaluated.
* **Where derivatives exist.** At a singular input the derivative may be unbounded even on the
  chosen branch: the positive root of `x² = u` has `dx/du = 1/(2x)`, unbounded as `u → 0`. A
  domain's edge is often such a place, and a generic-input check does not keep a parent away from
  it.
* **Conditioning.** Each evaluation reports the conditioning of its factorisation, and a parent
  treats a derivative from an ill-conditioned one as unreliable rather than as exact.
* **Failure.** A child that fails to solve, leaves its domain or changes branch returns a failure,
  never a stale or nearby value; the parent's step is rejected and shortened, and a failure that
  persists is reported by the parent's diagnosis naming the child and the input.
* **Staying inside.** A parent's trust region is bounded by its children's domains, so a step is
  shortened before it leaves one rather than after.
* **Smooth sketch outputs first.** An output measured from the solved sketch (a length, an angle)
  is the first kind to support (#458). An output through an evaluated solid (a volume, a mass) comes
  later: finite differences need an accuracy contract, since evaluation noise can swamp the
  derivative and a perturbation can cross a change of topology.

**N9 — Caching follows dependencies, per output.** A noun's result is keyed by its definition, the
shape of its inputs (N3) and the context it read; elaboration is cached as well as the solve. But
invalidation is tracked **per output** (#457): each output records which inputs and context values
it read, and a change re-solves the outputs that read it. A key per noun is too coarse — §6.4 shows
a change that misses a noun's key and still leaves the output its parent needs unchanged.

---

## 6. Worked example: a belt drive

One drawing, written three times: in the current style, in the statement style of §4, and with
nouns and verbs. The claim throughout is a real theorem, and a conditional one: **where the
pulleys' radii differ**, the belt's two straight runs, produced, meet on the line of centres, at
the drive's external centre of similitude, at a distance r₁c/(r₂ − r₁) behind the small pulley.
Where the radii are equal the runs are parallel and meet nowhere.

### 6.1 The current style

At the radii given — 20 mm and 45 mm — this parses and solves: DOF 0, the claim judged a theorem,
the runs meeting at x = −96, which is −r₁c/(r₂ − r₁) = −20 · 120 / 25.

```solvent
// An open belt drive: a small driving pulley and a large driven one, the belt's two straight
// runs tangent to both, each pulley a disc bored for its shaft and made solid, and the two
// shafts turning at the ratio of the radii.
unit mm
use std (horizontal)

drive := {r_small: 20mm, r_large: 45mm, centres: 120mm, bore: 4mm, width: 10mm}

// A pulley: a rim about the centre, a bore through it, the two made a disc `width` thick.
component Pulley(view: plane, centre: point, r: Length, bore: Length, width: Length) {
  in view {
    rim := circle(center: centre) hint(r: r)
    radius(r) rim
    hole := circle(center: centre) hint(r: bore)
    radius(bore) hole
  }
  body := solid(face(rim, holes: hole), from: 0mm, to: width)
}

// One straight run of the belt, from `a` to `b`, touching each: `up` 1 for the run over the
// top, -1 for the one under, which picks the branch.
component Run(view: plane, a: circle, b: circle, up: Scalar) {
  in view {
    s := point hint((a.center.x, a.center.y + up * a.r))
    e := point hint((b.center.x, b.center.y + up * b.r))
    span := line(s, e)
    s coincident a
    e coincident b
    span tangent(at: p1) a
    span tangent(at: p2) b
  }
}

// Where two runs cross, and the line it must lie on.
component Crossing(view: plane, a: line, b: line, l: line) {
  in view {
    x := point hint((-60, 0))
    x coincident a
    x coincident b
    claim x coincident l
  }
}

in std.front {
  o1 := point
  fix((0, 0)) o1
  o2 := point hint((120, 0))
  o1 horizontal o2
  o1 distance(drive.centres) o2
  construction centres := line(o1, o2)
}

small := Pulley(std.front, o1, r: drive.r_small, bore: drive.bore, width: drive.width)
large := Pulley(std.front, o2, r: drive.r_large, bore: drive.bore, width: drive.width)
upper := Run(std.front, small.rim, large.rim, up: 1)
lower := Run(std.front, small.rim, large.rim, up: -1)
meet := Crossing(std.front, upper.span, lower.span, centres)

driver := frame
std.Spin(driver, about: o1, on: std.front, ratio: 1)
driven := frame
std.Spin(driven, about: o2, on: std.front, ratio: drive.r_small / drive.r_large)
```

What is wrong with it, by §2: the bottom of the file is a program that builds the drawing through
five calls, and the theorem is hidden inside one of them; a configuration record is threaded
through every call; `view: plane` is passed to every component although the language can draw an
instance `in std.front`; `Crossing` asserts and builds nothing that the parent could not say; `up:
±1` exists only to pick a branch by seed arithmetic; and `std.Spin` is a procedure.

And one thing worse than style. `Crossing` builds its witness `x` unconditionally, and the two
coincidences that place it are constraints of the model, not of the claim. Set `r_large` to 20 mm
and the drive is still a valid drive, but `x` cannot exist. Today that document reports
`success: true` (status "max iterations reached", residual norm 7.2 × 10⁻⁸), DOF 0 and the claim a
theorem, with `x` at about −10,362 mm and the upper run's contact missing the small rim by
0.1326 mm; remove the witness and the same drive solves to 10⁻¹⁶. A construction made only to state
a theorem has restricted, and here silently broken, the model. The false "solved" is a bug of its
own (#449); the restriction is the document's, and §6.3 answers it by scoping the witness to the
claim (#460).

### 6.2 The statement style (§4 only)

Every line but four parses today; the four marked `[new]` are the refusals of §2.6. With those
four written in today's syntax (`Pulley(o1, width: 10mm)`, `turns(ratio: 1)`) the document solves
to the same drawing, and the claim is again judged a theorem — at these radii. Its witness is
unconditional as §6.1's is, so at equal radii it has nowhere to stand.

```solvent
// An open belt drive.
//
// Given two pulleys of different radii on parallel shafts, and a belt drawn tight round both.
// Then the belt's two straight runs, produced, meet on the line of centres — the drive's centre
// of similitude — and the shafts turn at the inverse ratio of the radii.
unit mm
use std (horizontal)

// -- words ---------------------------------------------------------------------------------------
l leaves k := {
  l.p1 coincident k
  l tangent(at: p1) k
}
l reaches k := {
  l.p2 coincident k
  l tangent(at: p2) k
}
f turns(ratio) shaft := {
  shaft coincident f.axis
  fix(ratio == ratio, advance == 0mm) f
}

component Pulley(centre: point, width: Length) {
  rim := circle(center: centre)
  hole := circle(center: centre)
  body := solid(face(rim, holes: hole), from: 0mm, to: width)
}

// -- given ---------------------------------------------------------------------------------------
r_small := 20mm
r_large := 45mm
bore := 4mm

in std.front {
  o1 := point
  o2 := point hint((120, 0))
  fix((0, 0)) o1
  o1 horizontal o2
  o1 distance(120mm) o2
  centres := line(o1, o2)

  small := Pulley(o1)
  large := Pulley(o2)
  fix(r == r_small) small.rim
  fix(r == r_large) large.rim
  fix(r == bore) small.hole
  fix(r == bore) large.hole
}
fix(width == 10mm) small                                    // [new]
fix(width == 10mm) large                                    // [new]

shaft1 := axis
o1 coincident shaft1
shaft1 perpendicular std.front
shaft2 := axis
o2 coincident shaft2
shaft2 perpendicular std.front

// -- the belt ------------------------------------------------------------------------------------
in std.front {
  a := point hint(at: small.rim, bearing: 90deg)
  b := point hint(at: large.rim, bearing: 90deg)
  upper := line(a, b)
  upper leaves small.rim
  upper reaches large.rim

  c := point hint(at: small.rim, bearing: -90deg)
  d := point hint(at: large.rim, bearing: -90deg)
  lower := line(c, d)
  lower leaves small.rim
  lower reaches large.rim
}

driver := frame
driver turns(1) shaft1                                      // [new]
driven := frame
driven turns(r_small / r_large) shaft2                      // [new]

// -- hence ---------------------------------------------------------------------------------------
in std.front {
  x := point hint(at: o1, toward: o2, by: -1)
  x coincident upper
  x coincident lower
  claim x coincident centres
}
```

Against §6.1: 15 labelled arguments become 4, all of them seeds; 5 component calls become 2; the
procedure and the asserting component are gone; and the file reads as a proposition. That is
§4's whole case, and it stands on readability alone. It does nothing for solving, and arguably
goes the other way: the runs, which were constructions, became verbs stated in the global system;
and `fix(r == r_small) small.rim` completes the pulley from outside, so the pulley is neither
closed nor determined by its inputs. §5 is a separate proposal, about that.

### 6.3 Nouns and verbs (§4 and §5)

The drive is now a noun inside a small machine, so both directions of C5 appear: a speed
requirement flows down into the drive, a stock belt closes a loop above it, and a fit standard
reaches every bore without passing through the drive. This is proposed syntax, apart from
`interface`, `ccw` and `when`, which exist.

```solvent
// A belt drive between a motor and a spindle.
//
// Given a motor shaft and a spindle shaft of stock rod, a speed the spindle must turn at, and a
// standard belt.  Then the shafts stand where that belt is tight, and, where the pulleys differ,
// the belt's two runs, produced, meet on the line of centres.
unit mm
use std (horizontal, leaves, reaches, spins)
use hardware
use fits

// -- nouns ---------------------------------------------------------------------------------------

// What a pulley needs of a shaft: where it stands and how thick it is.
interface Shaft { centre: point, d: Length }

// A rod of stock size through a centre.  Its diameter travels with it.
component Rod(centre: point) {
  given d: Length
}

// The pulley on a shaft: a rim `r` about it, bored to the shaft at the fit in force, a disc
// `width` thick.
component Pulley(shaft: Shaft) {
  given r: Length
  given width: Length = 10mm
  context fit: FitClass

  rim := circle(center: shaft.centre)
  hole := circle(center: shaft.centre)
  radius(r) rim
  radius(fit.hole(shaft.d) / 2) hole
  body := solid(face(rim, holes: hole), from: 0mm, to: width)
}

// The run of belt from circle `a` to circle `b`, both on its right: the left external tangent.
// A function of the two circles on its domain, where neither circle holds the other (centres
// farther apart than the difference of the radii): its seeds are stated relative to them, never
// to the page, and the two predicates choose the one tangent of the four.
component CommonTangent(a: circle, b: circle) {
  s := point hint(at: a.center, toward: b.center, by: 0.1, turn: 90deg)
  e := point hint(at: b.center, toward: a.center, by: 0.1, turn: -90deg)
  run := line(s, e)
  run leaves a
  run reaches b
  ccw(a.center, b.center, s)
  ccw(a.center, b.center, e)
}

// The drive between two shafts: the pulleys, the belt's runs and wraps, and the turning.
// Its domain: r_small > 0, ratio > 0, and the shafts farther apart than the two rims' radii
// together, so the pulleys clear each other and both external tangents exist.
component BeltDrive(a: Shaft, b: Shaft) {
  given r_small: Length
  given ratio: Scalar                         // the driven shaft's speed over the driver's

  small := Pulley(a)
  small.r := r_small
  large := Pulley(b)
  large.r := r_small / ratio

  upper := CommonTangent(small.rim, large.rim)
  lower := CommonTangent(large.rim, small.rim)
  around_small := arc(center: a.centre, start: upper.run.p1, end: lower.run.p2)
  around_large := arc(center: b.centre, start: lower.run.p1, end: upper.run.p2)
  belt_length := length(upper.run) + length(around_large) + length(lower.run) + length(around_small)

  driver := frame
  driver spins(1) a.centre
  driven := frame
  driven spins(ratio) b.centre

  // hence, where the radii differ: the runs, produced, meet on the line of centres.  The
  // witness is the claim's own: solved after the drive, against it held, and for nothing else.
  when ratio != 1 {
    claim {
      x := point
      x coincident upper.run
      x coincident lower.run
      x coincident line(a.centre, b.centre)
    }
  }
}

// -- the machine ---------------------------------------------------------------------------------

param c: Length hint(120mm)                   // how far apart the shafts stand: this level's unknown

in std.front {
  m := point
  s := point hint((120, 0))
  fix((0, 0)) m
  m horizontal s
  m distance(c) s
}

motor := Rod(m)
motor.d := hardware.rod516_d
spindle := Rod(s)
spindle.d := hardware.rod516_d

with fit := fits.H7 {
  drive := BeltDrive(motor, spindle)
}
drive.r_small := 20mm
drive.ratio := 4 / 9                          // the spindle at 4/9 of the motor's speed
drive.belt_length == hardware.gt2_loop_450    // a stock belt: this closes the loop for `c`
```

What changed from §6.2:

* **The runs are nouns again.** `upper := CommonTangent(small.rim, large.rim)` is "the tangent from
  the small rim to the large one", and inside it the sentences `run leaves a` and `run reaches b`
  survive one level down. The lower run is the same noun with its operands swapped — the belt runs
  one way along the top and back along the bottom — so one definition serves both and no `up: ±1`
  is needed.
* **The branch is chosen, not suggested** (N2, tier 2). Two circles clear of each other have four
  common tangents. On an inner one the tangent points fall on opposite sides of the line of
  centres; on the two outer ones on the same side, and the two `ccw` predicates admit only the
  left one — once a predicate on the sheet is enforced and checked, which today it is not
  (#157). That this holds over the whole domain is a short argument — the radii to the tangent
  points are parallel and on one side — which is tier 3 for this noun, made by hand. It fails at
  the domain's edge: as the centres close to the difference of the radii, the outer tangents
  merge and the tangent points' derivatives with respect to the centre distance grow without
  bound (N8).
* **No number is threaded.** There are no puns. The shaft's diameter travels with the shaft (N6);
  the fit class is context (N7), and `BeltDrive` never mentions it although both pulleys inside it
  read it.
* **Inputs are bound by value** (N5): `small.r := r_small`, never `fix(r == …) small.rim`.
* **The theorem is conditional, and its witness is the claim's.** `when` (spec §12.9, today's)
  decides the condition while elaborating, from the bound `ratio`; the claim block (#460,
  proposed) solves `x` after the drive and against it, so at `ratio := 1` the drive is untouched and
  the claim is simply not made. A witness that cannot be built fails its claim, never the model.
  The claim lives in `BeltDrive`, so every drive in an assembly carries it, judged once per distinct
  solve.
* **The machine closes the loop** (N8): the ratio comes down as an input, the belt length goes up
  as an output, and the one equation tying them — the centre distance must make a stock belt fit —
  is the machine's.
* **No world direction inside a noun** (N3). Only the machine says `std.front` and `horizontal`.
  `CommonTangent` builds its frame from the line of centres, and `BeltDrive` from its shafts.

### 6.4 What is solved where

| level | its own unknowns | shape inputs (the cache key, up to proper rigid motion) |
|---|---|---|
| `Pulley` | its two circles | `r`, `width`, `shaft.d`, `fit` |
| `CommonTangent` | two points: four numbers | the two radii and the distance between centres |
| `BeltDrive` | the wraps, and the claim's `x` after | `r_small`, `ratio`, the shafts' `d`, their distance, `fit` |
| machine | **`c`**: one equation, one unknown | — |

The machine's solve is Newton on one number: the drive enters as a function of `c`, and its
derivative comes from its children's factorisations. Here the loop can be checked by hand. With
`Δ` the difference of the radii, `belt_length(c) = 2√(c² − Δ²) + π(r₁ + r₂) + 2Δ·asin(Δ/c)`, whose
derivative is `2√(c² − Δ²)/c`: positive on the whole domain, so a target belt in range has exactly
one `c` (for the 450 mm loop at 20 and 45 mm, about 120.3 mm), and approaching zero as `c → Δ`, so
the step would blow up near the domain's edge — which the drive's own domain, `c > r₁ + r₂`, keeps
the parent away from. That is N8's contract at work: a domain, a derivative that exists inside it,
and a parent kept inside.

What each change costs:

* **A new `c`** (each Newton step): the two tangents re-solve; both pulleys are cache hits — their
  inputs do not involve `c`, and moving a shaft moves only a pose (N3).
* **A different fit class** (a press fit): both pulleys' holes change, so their keys miss and they
  re-solve; the tangents hit, since the rims did not change. `BeltDrive`'s key includes `fit`,
  which it reads through its pulleys, so a key per noun misses there too and would re-solve the
  machine's loop for nothing. With dependencies tracked **per output** (N9), `belt_length` read
  only the rims and the tangents, none of which read `fit`, so it is unchanged and the solve for
  `c` is a hit. That is the case for #457.
* **A new ratio from above**: the large pulley and the tangents miss, the small pulley hits, `c`
  re-solves.

With a thousand drives in an assembly the coupled system any one step factorises stays this size
(C4), and drives of the same shape are one solve. But a change that touches a thousand distinct
drives makes a thousand evaluations, and whether the whole beats the flat solve, which already
decomposes this graph, is for #459 to measure.

### 6.5 What §6.3 needs that the language lacks

* `given` inputs, and binding one by `inst.x := v` (N5);
* `context` and `with … { }` (N7);
* a claim that owns its witness, `claim { … }` (#460); `when` exists, and whether a claim's
  precondition is better written as a `when` over values or judged on the solved pose is #460's
  question;
* a way to state a noun's domain, which today lives in a comment;
* an output computed from the solved figure (`belt_length := length(upper.run) + …`) — today a
  value may not read geometry — and an equation at the parent over a child's output (N8, #458);
* `leaves`, `reaches` and `spins` in `std`, a word's measure unlabelled (R2), and `spins` reading
  the plane its operand is drawn in (§2.6, #463);
* the plane a pulley's circles are drawn in taken from its operands (the shaft's axis), so that no
  noun names `std.front` (N3); §6.3 leaves it implicit;
* own-frame solving (N3), which needs no syntax: it is a rule about what an instance means.

Typed interfaces are not on the list: `interface NAME { … }` and a formal typed by one exist
(spec §8.1, 0.84). What would be new is an interface that names a solve boundary; today "an
interface names no solve boundary and makes nothing", and an interface may not share a name with a
component of its file (E071), which is why the component here is `Rod`.

---

## 7. Open questions

* **Inferred or declared.** If boundaries pay (#459), a parent might find them from the closed
  component scope alone, with no new syntax; `given` and `context` would then be for readers. #461
  decides on the benchmark's evidence.
* **Inputs as statements or in brackets.** `small.r := r_small` on its own line is the proof's
  idiom, but four inputs cost four lines. A `where` clause — `Pulley(a) where r := r_small` — is
  the compromise, and risks drifting back to an argument list.
* **A noun that is one thing.** "The tangent from A to B" *is* a line, so `upper.run` might be
  `upper`. When a noun stands for its single result needs a rule.
* **Structural numbers.** A number that changes structure (`N` in `ring N`) is decided while
  elaborating, never solved; it needs a mark distinct from an input the parent may leave unknown.
* **How much of the corpus is which tier.** How many components are locally determined, how many
  choose their branches, and how many leave freedoms undeclared, is unmeasured; it is #452's corpus
  report.
* **Problems that are genuinely global.** A tolerance stack-up, a flexing structure, a closed
  kinematic loop through many subassemblies: these are coupled systems, and no language makes them
  small. The aim is that they are explicit, confined to the level that owns them, and their cost
  visible.
* **Sensitivities through evaluated solids.** An output like a mass, read by a parent's solve
  (N8), needs a derivative through the solid's evaluation — analytic, or by differencing a cached
  evaluation under an accuracy contract that survives a change of topology.
* **Seeds outside the argument.** Moving solver-written seeds to a sidecar, as presentation moved
  to `.svd`, would leave the proof clean, but P2 requires state to travel with its statement.
* **Context versus scoping.** N7 is close to dynamic scoping; what keeps it honest is that a noun
  *declares* what it reads. Whether intermediate nouns should also declare what passes through
  them — explicit, but threading again — is open.

## 8. Next steps

Experiment first, syntax last. Epic #465 tracks both tracks.

**Track A — reusable solve boundaries**, in order:

1. **The contract (#451).** C3, N2's tiers, N3's equivariance, N8's numerical contract and N9's
   per-output dependencies, written as rules a test can hold.
2. **An advisory classifier (#452)** in `solventc`, reporting each component's tier and what it
   leaves undeclared, with a corpus report; refuses nothing. In parallel, **a spike on tier 3
   (#453)**: verifying a component unique over an input domain.
3. **One reusable boundary on the locus machinery (#454)**, which already solves an inner block
   and differentiates through it; then **the numerical contract (#455)**, **a cache keyed up to
   placement (#456)**, **per-output invalidation (#457)** and **outputs read from a solved figure
   (#458)**, sketch measurements before solid ones.
4. **A benchmark against the flat solve (#459)**: repeated and distinct parts, closed loops, branch
   boundaries and singular inputs, measuring solve work, cache reuse, derivative accuracy and
   invalidation.
5. **Only then syntax (#461)**: `given`, binding by value, `context`, built, narrowed or dropped on
   #459's evidence.

Track A also leans on one existing gap: **`ccw` on the sheet enforced and checked (#157)**, without
which no drawn component reaches N2's second tier.

**Track B — statements and words**, independent of Track A and judged on readability:

* **A false "solved" (#449)**, found by the review: an infeasible document reports success and
  judges a claim a theorem.
* **Claim witnesses scoped to the claim (#460)**, so geometry made only to state a theorem cannot
  restrict the model.
* **The drift lint (#462)** of R10.
* **The small consistencies of §2.6 (#463)**: an unlabelled measure for a user word; a word reading
  its operand's plane.
* **Verbs for the motion components (#464)**: `Spin`, `Turn`, `Slide` as words, and their 67 bare
  calls migrated, after #463.

---

## Appendix: how the counts were made

Over `find rust/examples rust/lib -name "*.sv"` (183 files), with `grep -hE` per line unless
noted; counts are of lines or matches, approximate by construction, and descriptive only (R10).

| idiom | pattern |
|---|---|
| puns | `grep -ohE "\b(\w+): \1\b"` (matches) |
| instance calls | `":= *([a-z_]+\.)*[A-Z]\w*\("` |
| three labelled arguments | `"\([^()]*\b\w+: [^()]*\b\w+: [^()]*\b\w+: "` |
| component definitions | `"^component"` |
| relation-word definitions | `"^[a-zA-Z_]+ [a-z_]+(\([^)]*\))? [a-zA-Z_]+ :="` |
| group formals | `": group\b"` |
| bare component statements | `"^\s*([a-z_]+\.)*[A-Z]\w*\("` |
