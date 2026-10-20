# Solvent's direction: statements, nouns and verbs

**Status (2026-10-20): a discussion paper, nothing adopted.** It sets out where the language
stands, what has gone wrong with it, the constraints a revision has to meet, and a set of
recommendations with one example carried through every stage. The syntax in §5 and §6.3 is
proposed and does not parse; everything shown as current was run through `solventc`.

## Summary

Solvent began as a list of facts, and the best of it still reads like a proof: `Af distance(width)
Bf`, `Ef project Et`, a `claim` the diagnosis judges a theorem. Under steady growth, much of it
written by coding agents, it has drifted toward the idiom of an object-oriented program —
components called with long labelled argument lists, configuration records threaded through call
trees, and procedures called for their effect. At the same time the engine takes no advantage of
the language's own structure: every component is inlined into one flat system and the
decomposition is rediscovered from the graph, so nothing in the source guarantees that a large
assembly solves in small pieces.

The paper argues that both problems have one answer, and that it is the grammar of a proof.
First-order logic has two kinds of symbol: **functions**, which build terms that name things ("the
midpoint of AB"), and **predicates**, which build formulas that state facts ("AB ⟂ CD"). Solvent
should have the same two, and keep them apart:

* a **noun** is a function — closed over what it declares it reads, determined by its inputs, and
  solved in its own frame — so it can be solved once per distinct input and cached;
* a **verb** is a predicate — an equation between at most two things, stated at the lowest level
  that owns both.

With that split, a document stays a list of sentences, a standard (a fit class, a fastener table)
reaches every part that reads it without being threaded, a requirement at the top (a speed, a
delta-v) flows down as inputs while performance flows up as outputs, and the cost of a solve is the
size of the largest system at any one level rather than the size of the assembly.

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
* **P4** — component boundaries are decomposition structure: "solve interiors against the
  formals; solve the inter-component system over them" (a SHOULD).
* **P5** — symmetry is a claim, not a macro: a `ring` is solved in its fundamental domain.

### 1.2 What already reads like a proof

Much of the language is already sentence-shaped, and the recommendations below mostly generalise
rules it has:

* **Relations are operators.** Every constraint is a prefix or infix word: `horizontal l`,
  `a distance(80) b`, `l1 angle(30deg) l2`, `p coincident c`.
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

None of this matters for a bracket. All of it matters for an assembly the size of a space shuttle.

---

## 3. Constraints on the new language

A revision must meet five constraints, and keep P1–P5.

**C1 — Theorem-native, not procedural-native.** A document is a list of statements, each readable
aloud as one sentence of a geometric argument: "AB is perpendicular to CD", "let M be the midpoint
of AB", "hence X lies on the line of centres". A construct that reads only as "call F with these
arguments" fails. Order carries no meaning (P2); a document may still *read* in the order of a
proposition — given, let, such that, hence.

**C2 — A principled noun/verb split.** Every word is one of two kinds, told apart by the language
and checked:

* a **noun** builds a thing — "the common tangent of A and B" — and is *of* a few operands;
* a **verb** states a fact — "A is tangent to B" — and is *between* at most two.

A set is a predicate, so it sits with the verbs; that unification (§9.9) stays.

**C3 — Functional.** A noun is a function: its figure depends only on what it declares it reads
(closed), it is determined by those inputs up to freedoms it declares (determinate), and it is
solved in its own frame, so where it stands is a pose (placed). A noun with those three
properties can be solved once per distinct input and cached.

**C4 — Efficient for arbitrarily large assemblies.** The cost of a solve is the size of the
largest system at any one level of the instance tree, not the size of the assembly. Identical
instances share a solve. A change re-solves only what read the changed value.

**C5 — Composable, laterally and hierarchically.**

* *Laterally*: a standard — a fit class, a thread series, a fastener table — reaches every part
  that reads it, across the whole assembly, without being passed through the parts that do not.
* *Hierarchically*: a requirement at the top — an output speed, a delta-v for a payload — flows
  down as inputs; performance — a belt length, a mass — flows up as outputs; and a loop between
  them is closed explicitly at the level that owns it.

---

## 4. Statement-level recommendations

These serve C1. They came first, and one of them (R4 below) is revised by §5.

**R1 — The read-aloud test.** Every statement reads as one sentence. Every rule below is a way of
passing it.

**R2 — Relations are binary.** A statement is subject, verb, object and at most one measure: `a
WORD(measure) b`, or `WORD(measure) a`. More operands means two facts fused, or a construction
not yet named. A user word's measure is unlabelled, as a built-in's is (§2.6, E004).

**R3 — A component that only asserts is a verb.** `std.Spin(counter, about: centre, on: std.top,
ratio: -1)` becomes `counter spins(-1) centre`; `std.CircleOnSphere(k, ball, std.side)` becomes
`k coincident ball`, by widening `coincident` to a curve on a set. A component call standing bare
as a statement is refused; the 73 in the corpus are the migration list.

**R4 — Brackets for structure, numbers stated after.** *Revised by N5:* the first form of this —
`fix(r == 20mm) small.rim`, reaching inside an instance — makes a noun neither closed nor
determinate. What survives is that a noun's numbers are bound by statement rather than by
argument list, and only its declared inputs are.

**R5 — A closed table of prepositions.** A small fixed set — `in`, `on`, `about`, `along`, `over`,
`under`, `from`, `to`, `through`, `seen from`, `by` — each with one meaning everywhere, replaces
labelled qualifiers: `envelope(apex over t in (0deg, 1080deg), seen_from: counter)` becomes `bore
:= envelope of apex over t in (0deg, 1080deg) seen from counter`. The language already does this
with `in REF`, `ring N about C` and `over t in (a, b)`. New syntax may add a preposition only by
adding a row.

**R6 — Define by property, not by formula.** A coordinate formula is the last resort. Where a
property cannot yet be stated, that is a missing word, not a reason to write coordinates.

**R7 — A document reads as a proposition.** Given (inputs), let (declarations), such that
(relations), hence (claims). P2 keeps this a convention and a printer order, not grammar. The
payoff is in diagnosis: a report that numbers statements and cites them — "(7) and (12) both hold
`sm.y`" — reads as a proof check.

**R8 — Seeds name a place.** Prefer `hint(at: …, toward: …, turn: …)` to coordinate arithmetic. A
noun's seeds are stated relative to its operands (N3 needs it).

**R9 — Grow by vocabulary, not by options.** A capability arrives as a word, written in Solvent in
`std.sv` where possible, rather than a new label on a built-in. A proposed construct is shown read
aloud, shown not to be an existing word with a wider meaning, and names the preposition it uses.

**R10 — Make drift visible.** A `solventc --style` lint and a ratchet test counting the idioms of
§2.1 across the corpus; CI fails if a count rises — the way `every_file_is_a_module` holds test
files. The primer, which agents read first, points at this paper's principles.

---

## 5. Nouns and verbs

These serve C2–C5.

**N1 — Two kinds of word, checked.** A noun is a function: it builds, is *of* its operands, and
answers to a definite description ("the tangent from A to B"). A verb is a predicate: it states,
and is *between* its operands. A component whose body declares nothing and only relates its
formals is a verb, and is written as one (R3). A component that builds something its operands
determine is a noun, however small: `Crossing(a, b)` in the hypoid drive — the common
perpendicular of two lines — is a good noun.

**N2 — A noun is closed and determinate, and the checker proves it.** When a component is
defined, the diagnosis runs against its formals at a generic input: a noun must be well-constrained
relative to its inputs, apart from freedoms it declares and exports (a crank's angle). Its branch
must be chosen by its own `ccw`/`cw` and by seeds relative to its operands, never by where the
global solve happens to start. The definite article is the test: "*the* tangent from A to B" is a
function; "*a* point on c" leaves a freedom the parent must own.

**N3 — A noun is solved in its own frame.** Its figure is solved relative to its operands, and
where the instance stands is a pose. Its cache key is its inputs up to rigid placement: a common
tangent of two circles is keyed by their radii and the distance between their centres, not by
where they are. Identical instances are one solve and many poses — P5's fundamental domain,
generalised.

**N4 — Verbs relate interfaces, at the lowest common parent.** A verb between two nouns relates
what they export (their `interface`), and stands in the lowest instance that owns both. A deep path
— `pair.gear_cone.ax`, `gear_side.crank.handle` — is a warning: it couples a level to the inside
of another.

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
level it applies. Nouns in between do not mention it: the implementation tracks which nouns read
which context, and the cache key of each includes the context values it (transitively) read. The
alternatives fail C3 or C4: threading is §2.3; dynamic scoping breaks closure; and equating every
hole to every other puts a global coupling into the solve.

**N8 — Hierarchical information flows as inputs down and outputs up.** A requirement is bound as
an input; performance is an output computed from the solved figure (`belt_length := length(upper)
+ …`). A loop between them is one equation at the level that owns it, over that level's own
unknowns, with each child entering as a cached function of its inputs and its derivative taken
from the child's factorisation (the implicit function theorem). This lifts §1.2's "no edge back"
*between* nouns while keeping it *within* one: a child's evaluated measurements are a function of
its inputs that a parent may read.

**N9 — Caching is part of the meaning.** A noun's result is keyed by its definition, its input
values up to placement, and the context it read; elaboration is cached as well as the solve, and
invalidation follows dependency, as in an incremental build system.

---

## 6. Worked example: a belt drive

One drawing, written three times: in the current style, in the statement style of §4, and with
nouns and verbs. The claim throughout is a real theorem: the belt's two straight runs, produced,
meet on the line of centres, at the drive's external centre of similitude.

### 6.1 The current style

This parses and solves: DOF 0, the claim judged a theorem, the runs meeting at x = −96, which is
−r₁c/(r₂ − r₁) = −20 · 120 / 25.

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

### 6.2 The statement style (§4 only)

Every line but four parses today; the four marked `[new]` are the refusals of §2.6. With those
four written in today's syntax (`Pulley(o1, width: 10mm)`, `turns(ratio: 1)`) the document solves
to the same drawing, and the claim is again judged a theorem.

```solvent
// An open belt drive.
//
// Given two pulleys on parallel shafts, and a belt drawn tight round both.  Then the belt's two
// straight runs, produced, meet on the line of centres — the drive's centre of similitude — and
// the shafts turn at the inverse ratio of the radii.
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
procedure and the asserting component are gone; and the file reads as a proposition. **But it goes
the wrong way for scale.** The runs, which were constructions, became verbs stated in the global
system; and `fix(r == r_small) small.rim` completes the pulley from outside, so the pulley is
neither closed nor determinate. That is what §5 corrects.

### 6.3 Nouns and verbs (§4 and §5)

The drive is now a noun inside a small machine, so both directions of C5 appear: a speed
requirement flows down into the drive, a stock belt closes a loop above it, and a fit standard
reaches every bore without passing through the drive. This is proposed syntax.

```solvent
// A belt drive between a motor and a spindle.
//
// Given a motor shaft and a spindle shaft of stock rod, a speed the spindle must turn at, and a
// standard belt.  Then the shafts stand where that belt is tight, and the belt's two runs,
// produced, meet on the line of centres.
unit mm
use std (horizontal, leaves, reaches, spins)
use hardware
use fits

// -- nouns ---------------------------------------------------------------------------------------

// A shaft: a rod of stock size through a centre.  Its diameter travels with it.
interface Shaft { centre: point, d: Length }
component Shaft(centre: point) {
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
// A function of the two circles, so its seeds are stated relative to them, never to the page.
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

  // hence: the runs, produced, meet on the line of centres
  x := point
  x coincident upper.run
  x coincident lower.run
  claim x coincident line(a.centre, b.centre)
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

motor := Shaft(m)
motor.d := hardware.rod516_d
spindle := Shaft(s)
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
  is needed. The two `ccw` statements choose the root, which a seed can only suggest, so the noun
  is a function (N2).
* **No number is threaded.** There are no puns. The shaft's diameter travels with the shaft (N6);
  the fit class is context (N7), and `BeltDrive` never mentions it although both pulleys inside it
  read it.
* **Inputs are bound by value** (N5): `small.r := r_small`, never `fix(r == …) small.rim`.
* **The theorem travels with the noun.** The claim lives in `BeltDrive`, so every drive in an
  assembly carries it, judged once per cached solve.
* **The machine closes the loop** (N8): the ratio comes down as an input, the belt length goes up
  as an output, and the one equation tying them — the centre distance must make a stock belt fit —
  is the machine's.

### 6.4 What is solved where

| level | its own unknowns | cached on (up to placement) |
|---|---|---|
| `Pulley` | its two circles | `r`, `width`, `shaft.d`, `fit` |
| `CommonTangent` | two points: four numbers | the two radii and the distance between centres |
| `BeltDrive` | the wraps and `x` | `r_small`, `ratio`, the shafts' `d`, their distance, `fit` |
| machine | **`c`**: one equation, one unknown | — |

The machine's solve is Newton on one number: the drive enters as a function of `c`, its derivative
from its children's factorisations. What each change costs:

* **A new `c`** (each Newton step): the two tangents re-solve; both pulleys are cache hits — their
  inputs do not involve `c`, and moving a shaft moves only a pose (N3).
* **A different fit class** (a press fit): both pulleys miss; the tangents hit, since the rims did
  not change; and the solve for `c` is a hit too, because `belt_length` never read `fit`.
* **A new ratio from above**: the large pulley and the tangents miss, the small pulley hits, `c`
  re-solves.

With a thousand drives in an assembly the table holds per drive, and identical drives are one
solve.

### 6.5 What §6.3 needs that the language lacks

* `given` inputs, and binding one by `inst.x := v` (N5);
* `context` and `with … { }` (N7);
* an `interface` as a formal's type (`a: Shaft`);
* an output computed from the solved figure (`belt_length := length(upper.run) + …`) — today a
  value may not read geometry — and an equation at the parent over a child's output (N8);
* `leaves`, `reaches` and `spins` in `std`, a word's measure unlabelled (R2), and `spins` reading
  the plane its operand is drawn in (§2.6);
* own-frame solving (N3), which needs no syntax: it is a rule about what an instance means.

---

## 7. Open questions

* **Inputs as statements or in brackets.** `small.r := r_small` on its own line is the proof's
  idiom, but four inputs cost four lines. A `where` clause — `Pulley(a) where r := r_small` — is
  the compromise, and risks drifting back to an argument list.
* **A noun that is one thing.** "The tangent from A to B" *is* a line, so `upper.run` might be
  `upper`. When a noun stands for its single result needs a rule.
* **Structural numbers.** A number that changes structure (`N` in `ring N`) is decided while
  elaborating, never solved; it needs a mark distinct from an input the parent may leave unknown.
* **Branch choice under caching.** A noun's branch must be determined by its inputs and its own
  `ccw`/`cw` and relative seeds (N2). How much of the corpus already meets that is unmeasured.
* **Problems that are genuinely global.** A tolerance stack-up, a flexing structure, a closed
  kinematic loop through many subassemblies: these are coupled systems, and no language makes them
  small. The aim is that they are explicit, confined to the level that owns them, and their cost
  visible.
* **Sensitivities through evaluated solids.** An output like a mass, read by a parent's solve
  (N8), needs a derivative through the solid's evaluation — analytic, or by differencing a cached
  evaluation.
* **Seeds outside the argument.** Moving solver-written seeds to a sidecar, as presentation moved
  to `.svd`, would leave the proof clean, but P2 requires state to travel with its statement.
* **Context versus scoping.** N7 is close to dynamic scoping; what keeps it honest is that a noun
  *declares* what it reads. Whether intermediate nouns should also declare what passes through
  them — explicit, but threading again — is open.

## 8. Next steps

In order of cost:

1. **The noun check (N2).** `solventc` diagnoses each component against its formals at a generic
   input and reports which are already functions, which leave freedoms, and which only assert.
   Run over the corpus, it measures how far the library is from cacheable nouns, and it is built
   from the diagnosis that exists.
2. **The ratchet (R10)**: a test counting the idioms of §2.1, failing CI when one rises.
3. **The small consistencies of §2.6**: an unlabelled measure for a user word; a word reading its
   operand's plane.
4. **Verbs for the motion components (R3)**: `Spin`, `Turn`, `Slide` as words, and the 73 bare
   calls migrated.
5. **`given` and `inst.x := v` (N5)**, then **`context`/`with` (N7)**.
6. **Own-frame solving and the instance cache (N3, N9)** — the first change to the engine rather
   than the language, and the one that makes C4 true.
7. **Outputs and the edge back between nouns (N8).**

---

## Appendix: how the counts were made

Over `find rust/examples rust/lib -name "*.sv"` (183 files), with `grep -hE` per line unless
noted; counts are of lines or matches, approximate by construction.

| idiom | pattern |
|---|---|
| puns | `grep -ohE "\b(\w+): \1\b"` (matches) |
| instance calls | `":= *([a-z_]+\.)*[A-Z]\w*\("` |
| three labelled arguments | `"\([^()]*\b\w+: [^()]*\b\w+: [^()]*\b\w+: "` |
| component definitions | `"^component"` |
| relation-word definitions | `"^[a-zA-Z_]+ [a-z_]+(\([^)]*\))? [a-zA-Z_]+ :="` |
| group formals | `": group\b"` |
| bare component statements | `"^\s*([a-z_]+\.)*[A-Z]\w*\("` |
