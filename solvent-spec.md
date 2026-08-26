# Solvent: A Declarative Language for Constrained Geometry

**Specification, Draft 0.2 — August 2026**

*Draft 0.2 amends 0.1 in seven places, from implementing 0.1 end to end (bmander/geomsolver#2).
Marked **[0.2]** where they appear. In summary: the `=` / `==` seed mark (§4.3) that makes
Invariant H checkable by looking; seeds written inline (§6.4, §11); document state attached to its
statement, which is what makes P2 true rather than merely asserted (§13.1); the identity of a
statement under expansion (§12.7); curve families as a document declaration, promoted out of §17
(§6.5); a reporting duty on an implementation that unrolls a `ring` (§12.3); and `Line` cut back to
a segment whose infinite carrier is what constraints read (§3.1).*

*"Solvent" is a placeholder name; nothing in this document depends on it.*

---

## 1. Overview

Solvent is a declarative language for describing planar geometry under constraint. A Solvent program does not construct geometry step by step; it declares a set of entities, relations among them, and structural facts (symmetry, connectivity), and delegates the discovery of coordinates to a solver. The language borrows its module discipline from hardware description languages: designs are built from **components** with **ports**, component bodies are **unordered**, and connection between components is **aliasing** (net formation), not constraint.

### 1.1 Design principles (normative)

These principles are binding on the rest of the specification. Where a detailed rule appears to conflict with a principle, the principle governs and the conflict is a defect in this document.

**P1 — Connection is aliasing; constraint is explicit.**
Binding a port, passing an entity as an instance argument, or writing `port x = y` makes two names refer to *one* entity. Aliasing has zero solver cost and cannot be violated. A constraint always relates *distinct* entities and always contributes residual equations (or inequalities) to the system.

**P2 — Component bodies are unordered.**
Statements within a component body form a set. Reordering the statements of a body MUST NOT change the meaning of a program. Statements interact only through the entities they name. The only ordered contexts in the language are the interiors of single statements where order is semantic (path traversals, argument lists).

**[0.2]** P2 binds the *document* as well as the program text: every piece of state a document carries MUST be attached to the statement it qualifies, and MUST NOT be keyed by a statement's position in a body or by an entity's index (§13.1). Without that rule P2 is an assertion an implementation can satisfy in the parser and lose in the file format — and it fails silently, which is the worst way for it to fail.

**P3 — Hints are semantically inert.**
Deleting every seed from a program MUST NOT change its solution set. Seeds may change which solution a solver finds, or whether it converges, but never what counts as a solution. Any annotation that changes the solution set (orientation predicates, arc branch selection, tangency side) is a **constraint** and is classified as such by the implementation, regardless of surface syntax.

**[0.2]** The two are told apart by one mark, not by analysis: a number written with `=` is a seed and a solver may rewrite it; a number written with `==` is a constraint and a solver MUST NOT (§4.3). This is what makes Invariant H (§11) checkable *syntactically*, as that section already requires.

**P4 — Component boundaries are decomposition structure.**
A component is solvable against its ports. Implementations SHOULD exploit the component instance tree as a decomposition plan (solve interiors against port entities; solve the inter-component system over ports).

**P5 — Symmetry is a claim, not a macro.**
The `ring` construct asserts cyclic symmetry. Its solution set contains exactly the symmetric solutions of the corresponding unrolled system. Implementations SHOULD solve in the fundamental domain, and one that does not MUST say so (§12.3).

### 1.2 Scope of this draft

This draft specifies **planar (2D) geometry only**. The entity vocabulary, constraint library, and group actions are two-dimensional. Section 17 lists the known lifting questions for 3D. This draft also excludes curve entities beyond lines and circles (no involutes, splines, or conics); see §17.

### 1.3 Conformance keywords

MUST, MUST NOT, SHOULD, SHOULD NOT, and MAY are used as in RFC 2119. Text marked *non-normative* is explanatory.

---

## 2. Lexical structure

- **Identifiers:** `[A-Za-z_][A-Za-z0-9_]*`. Component names are conventionally capitalized; this is not enforced.
- **Keywords:** `component`, `port`, `param`, `point`, `circle`, `line`, `frame`, `path`, `repeat`, `cycle`, `ring`, `about`, `as`, `next`, `prev`, `hint`, `at`, `ground`, `fix`, `ccw`, `cw`, `rev`, `true`, `false`, **[0.2]** `curve`, `over`, `ellipse`, `spline`, `construction`. **[0.4]** In a chain (§6.6) the words `to` and `close` are meaningful *contextually*; they are not reserved, and an entity may bear either as a name. **[0.5]** A coordinate seed is written `hint at` (§6.4).
- **Literals:** decimal numbers with optional unit suffix (`10`, `2.5mm`, `30deg`). The constant `tau` (= 2π) and `pi` are predefined.
- **Comments:** `//` to end of line; `/* ... */` nesting not required.
- **Operators and punctuation:** `== + - * / ( ) { } [ ] , : . = -> ~`
- Whitespace and newlines are insignificant except as token separators. Statements are newline- or `;`-terminated; implementations MUST accept either.

---

## 3. Types and dimensions

### 3.1 Value types

| Type | Meaning | Free DOF when unconstrained |
|---|---|---|
| `Int` | compile-time integer (parameters, counts, indices) | — (elaboration-time) |
| `Scalar` | dimensionless real | 1 |
| `Length` | real with length dimension | 1 |
| `Angle` | real with angle dimension | 1 |
| `Point` | position in the plane | 2 |
| `Line` | segment between two `Point`s; its infinite carrier is what constraints read **[0.2]** | 0 of its own (4 through its ends) |
| `Circle` | center + radius | 3 |
| `Frame` | origin + orientation | 3 |
| `Path` | directed piecewise boundary curve | 0 (derived object) |

**[0.2] `Line` was a 2-DOF undirected infinite line with a `.dir` field in 0.1.** It is now a segment between two points, and every constraint that reads a line reads the infinite carrier through those points — which is what `parallel`, `perpendicular`, `on` and `angle` mean by a line anyway.

The change is a concession to cost. A 2-DOF line is a second representation of a line alongside the one every constraint already works in, so it needs its own column layout and its own kernel for roughly fourteen constraint types, and the whole return is that §16.3's ledger comes out differently. Where a drawing wants a line with no ends — a datum, an axis — it says so with a construction segment.

### 3.2 Sub-entities

Compound entities expose sub-entities by field access. Sub-entities are ordinary entities and participate in aliasing and constraints.

| Entity | Field | Type |
|---|---|---|
| `Circle` | `.center` | `Point` |
| `Circle` | `.r` | `Length` |
| `Frame` | `.origin` | `Point` |
| `Frame` | `.angle` | `Angle` |
| `Line` | `.p1`, `.p2` | `Point` **[0.2]** |

### 3.3 Dimensional analysis

Implementations SHOULD check dimensions in expressions and constraints (`Length == Length`, `Angle == Angle`, `Length * Scalar → Length`, `Length / Length → Scalar`, etc.) and MUST NOT silently coerce `Angle` to `Scalar` or vice versa. Angle arithmetic is mod 2π where the operand is a bearing and signed where it is a turn; see §9.4.

---

## 4. Program structure

A program is a set of component definitions. One component is designated the **root** (by tool invocation, not by language syntax); the elaborated model is the root's instance tree.

```
component Name(p1: Type1, p2: Type2, ...) {
    <statements>
}
```

### 4.1 Parameters

Parameters are passed by name or position at instantiation. A parameter of entity type (`Point`, `Circle`, `Frame`, `Line`) is **bound by aliasing** (P1): the formal name and the actual argument denote the same entity. A parameter of value type (`Int`, `Scalar`, `Length`, `Angle`) is a compile-time or definitional value; it contributes no unknowns.

### 4.2 Statement classes

Every statement belongs to exactly one class. The classification is normative because P3 depends on it.

| Class | Statements | Affects solution set? |
|---|---|---|
| **Declaration** | entity declarations, `param`, `port`, `curve` family definitions, instance declarations | introduces entities/aliases |
| **Constraint** | predicate calls, `==` equations, **pinned seeds (`==`, §4.3) [0.2]**, orientation predicates, arc-branch and tangency-side decorations, `ring` symmetry, gauge statements | **yes** |
| **Seed** *(was Hint)* | `hint ... at ...`, **and every `=` seed written inline in a declaration (§6.4) [0.2]** | **no (P3)** |
| **Structure** | `repeat`/`cycle` blocks, `path` declarations (net of their derived constraints, §10.4) | organizational |

### 4.3 Seeds and pins: `=` and `==` **[0.2]**

> **A number written with `=` is a seed. A number written with `==` is a constraint.**

This is the only distinction between the two classes, and it is lexical on purpose. §11 requires an implementation to verify Invariant H *syntactically*; a mark you can see does that, and no analysis of what a number "is really doing" does.

```
point p at (0, 0)                    // seed: a solve may move it
circle c(center: o, r: 25)           // seed: a solve may move the radius
distance(a, b) == 80                 // constraint: a solve must not move it
point_on_spline(p, s, t = 0.37)      // seed: the contact may slide along the curve
point_on_spline(p, s, t == 0.37)     // constraint: the contact is pinned there
```

The last pair is why the rule is needed at all. A curve contact carries its own parameter, and
whether that parameter is free is not a fact about how it looks — it is a fact about the solution
set. A curve fitted through *m* points whose contacts are unpinned keeps *m* degrees of freedom and
can slide along itself; pin them and it does not. In 0.1 both forms would have been written the
same way and classified by intent.

**Consequences, all normative.**

1. A solver MUST NOT rewrite a `==` number. It MAY rewrite an `=` number, and doing so is how a
   drawing records where it ended up (§13.1).
2. A statement's class is decidable by inspection. An implementation MUST NOT need to consult the
   solution set, the solver, or the geometry to classify one.
3. Deleting every `=` number from a program leaves its solution set unchanged (P3). Deleting a
   `==` number generally does not.

**Non-normative.** The mark also settles seed *writeback*, which is what an implementation needs
when the drawing is edited by drawing on it rather than by typing:

> A seed is writable iff it is written with `=` and not `==`, is a literal and not an expression,
> and is reached by exactly one instance path.

The first clause is this section. The second keeps a radius written as a component's parameter
(`r: Rr`) from being overwritten with the number it happened to come to — the author said what it
*is*, not where it starts. The third is §12.7: thirty instances share one statement, and there is
no one pose to record.

---

## 5. Names, scope, and resolution

- The scope of a name is the entire component body in which it is declared (P2). Forward reference is legal and idiomatic.
- Redeclaration of a name within one body is an error (**E001**).
- Instance members are accessed by dotted paths: `t.lead`, `g.hub.origin`.
- Inside `repeat`/`cycle`/`ring` blocks, the index binder (`as i`) and the pseudo-instances `next` / `prev` are in scope (§12).
- There is no shadowing: a block binder that collides with an outer name is an error (**E002**).

---

## 6. Entity declarations

### 6.1 Free declarations

```
point p, q
line  l
```

A bare declaration introduces an entity all of whose coordinates are unknowns.

### 6.2 Constructor declarations

```
circle pitch(center, R)
frame  f0 = frame(center, ray(center, t.lead))
```

Constructor arguments have two behaviors, by type:

- An argument of **entity type** *aliases* the corresponding sub-entity (P1). `circle pitch(center, R)` makes `pitch.center` and `center` one entity.
- **[0.2]** An argument of **value type** *seeds* the corresponding coordinate. It is an unknown of the sketch with a starting value, not a substitution.

The `name = expr` form declares an entity wholly defined by an expression; all its coordinates are definitional.

**[0.2] The value-type rule is reversed from 0.1**, which made such an argument definitional — `circle pitch(center, R)` introducing `pitch.r == R` as a substitution. That cannot stand beside §4.3, and it is wrong on its own terms: a radius is a coordinate a user drags. Under 0.1's reading, taking hold of a rim and pulling would be an *edit of what the program means* rather than a move within its solution set, and every direct-manipulation gesture on a scalar would be a different kind of event from the same gesture on a point.

A radius that is meant to be *held* says so:

```
circle c(center: o, r: 25)     // 25 is where it starts
radius(c) == 25                // 25 is what it is
fix(c.r)                       // 25 is what it is, without a dimension on the drawing
```

### 6.3 `param`

```
param R = m * N / 2
```

Introduces a named definitional value. `param` values are evaluated at elaboration time when all inputs are `Int`/literal, otherwise they are definitional scalars.

### 6.4 Seeds written inline **[0.2]**

A declaration MAY carry the starting values of its own coordinates:

```
point   p  hint at (0, 0)
circle  c(center: o, r: 25)
ellipse e(center: q, major: m, b: 12)
```

These are seed-class (§4.2, §4.3) and semantically inert (P3). They are the primitive form; §11's `hint` statement remains, for the case it is actually good at.

**[0.5] `hint at`, where 0.2–0.4 wrote a bare `at`.** A coordinate seed reads as an assertion it does not make: `point p at (0, 0)` says where the point *is*, and it is not there — that is only where the solve begins, and the solve will move it. `hint at` says as much, in the words §11 already uses for the statement form, so the two read alike: `hint p at (0, 0)` standing on its own and `point p hint at (0, 0)` inline. Implementations SHOULD keep reading a bare `at`, so that documents written against 0.2–0.4 load, and SHOULD write the current spelling.

**What `hint` marks is that a solve revises the number — not that the number is seed-class.** The two are not the same set, and the difference decides where the word belongs. Seed-class is §4.3's classification: inert under P3, so deleting it changes no solution set. That is true of a coordinate seed *and* of a callout placement (§13.1), which is why a placement keeps its bare `at` even though it is every bit as inert. What separates them is who writes them. A coordinate seed is an input a solve overwrites, every time, which is the whole of what §6.4's writeback does. A placement is never touched by a solve at all: it is derived by the layout until somebody drags the callout, and from then on it records where that person put it. So a placement is not a guess about anything, and `at` there says what it means.

*Non-normative:* an implementation can therefore read `hint` as the mark of "this is the solver's to answer", which is a narrower and more useful claim than "this is inert". A reader wanting to know what may be deleted without changing the drawing should ask §4.3, which answers for both.

**Why inline is the primitive.** A seed's job is to say where a coordinate starts, and the place a reader looks for that is the declaration of the thing that has the coordinate. It matters more than taste once a drawing is edited by drawing on it: a solve that wants to record where a point ended up rewrites six characters of a declaration that already exists, where under 0.1 it would have to locate that point's `hint` statement among the body's statements, or synthesise one and decide where to put it. The first is a splice; the second is a program transformation, and it is performed on every drag.

`hint` keeps the cases inline cannot express — seeding an entity declared elsewhere, and seeding from an expression over other geometry (`hint t.lead at center + polar(root.r, 0)`).

### 6.5 Curve families **[0.2]**

```
curve involute(c: circle, phase: Angle)(u) =
  ( c.center.x + c.r * (cos(u + phase) + u * pi / 180 * sin(u + phase)),
    c.center.y + c.r * (sin(u + phase) - u * pi / 180 * cos(u + phase)) )
```

`curve NAME(FORMALS)(PARAM) [over (A, B)] = ( XEXPR, YEXPR )` declares a **family**: a map from a parameter and some geometry to a point in the plane. It declares no entity and no unknown — a family is a *kind* of curve, and several drawings may be written from one.

An instance is a declaration like any other, and takes contacts like any other curve:

```
curve e = involute(base, phase: a0) over (u0, u1)
point_on_curve(p, e, u = u0)
```

`point_on_curve(p, C, u)` says `p − C(u) = 0`: two residuals, one new unknown (the parameter, seed-class per §4.3), and so one equation net — the same arithmetic as a contact with any other curve.

**Requirements.** An implementation MUST differentiate a family's expressions with respect to the parameter *and* with respect to every coordinate they read. `∂C/∂u` is which way a contact may slide; `∂C/∂θ` is how the curve moves when the geometry it is written over moves, and an implementation that computes only the first will solve a contact once and drop it the moment that geometry is dragged. Both are mechanical from the expression, so this is a requirement on effort, not on ingenuity.

A name a family's expressions cannot reach is an error (**E016**), not a free variable. A dimension may name an unknown of the drawing; a curve is written over geometry that exists, and a misspelling there would quietly add a degree of freedom to every point on the curve.

**Why a declaration and not an entity kind.** 0.1 deferred this to §17.2 as "curve entities ... as first-class entities," which reads as one entity kind per family: an involute kind, a cycloid kind, a trochoid kind, each with its own constraints and its own place in every exhaustive match. Written as a declaration they are library code instead, and a second family is a second pair of expressions rather than a second constraint family. The gear in §18 is the case that makes the difference plain: its flanks are involutes because the document says what an involute is, and nothing in the solver knows the word.

### 6.5.1 Trace families **[0.3]**

A family's body may be a **locus** instead of a formula: `curve NAME(FORMALS)(PARAM) [over (A, B)] = trace POINT where { BODY }`. The block's statements are ordinary declarations and constraints; `C(u)` is the position they force on the traced point, given the parameter and the geometry the family is written over. This is the form a definition takes when a person states it — an involute is "the curve traced by the end of a taut string as it unwinds", and that sentence is the block:

```
curve involute(c: circle, datum: line, phase: Angle)(u) =
  trace p from (90 - phase) where {
    point t
    point p
    line rad(c.center, t)
    line s(t, p)
    point_on_circle(t, c)                                // the string leaves the circle...
    angle(datum, rad) == u + phase                       // ...at bearing u,
    perpendicular(rad, s)                                // perpendicular to the radius there,
    point_line_distance(p, rad) == -(c.r * u * pi / 180) // and taut: let out == arc unwound
    ccw(datum.p1, datum.p2, t)                           // which bearing mod 180: this one
  }
```

A dimension in the block may be an expression over the parameter, the formals' coordinates and the value formals. The block MUST determine its points — as many equations as inner coordinates — or the definition is an error: an under- or over-constrained locus is a curve that does not exist, and it must not elaborate quietly. Instances, contacts and `over` behave exactly as for an expression family, and the derivative requirement above stands unchanged: `∂C/∂u` and `∂C/∂θ` now come from the implicit function theorem at the block's solution rather than from the expressions, but a contact must still follow the curve when the geometry it is written over is dragged.

**Branches.** A locus generically has several solutions, and a block states its way onto one — three instruments, in order of strength:

1. **A signed constraint,** wherever the vocabulary can say it. Above, the taut string's winding is not a branch at all, because `point_line_distance` is signed: one equation unwinds the string one way for positive roll and the other for negative, where an unsigned `distance` would have left a mirror pair for something else to break.
2. **An orientation predicate.** `ccw(a, b, x)` / `cw(a, b, x)` in a block is §9.6's statement doing §9.6's job: it contributes no residual and *selects among the discrete solution components*. Its third point MUST be one the block places. Above, it settles the one branch a residual cannot — `angle` is a statement mod 180°, so `t` could sit at the bearing or opposite it, and the ccw says which. A predicate is read **at the home** — the parameter value `from (expr)` names (the expression is over the formals and the family's values; absent, the instance's domain begins evaluation) — and an implementation MUST enforce it there (reflect the placed point across the oriented line and solve again) and MUST NOT re-enforce it elsewhere: away from the home, continuity governs, and the component the predicate picks at the home is the component the whole curve is on, even where the curve has since wound to where the predicate no longer reads true. Choose the home so the predicates read unambiguously — above, the roll at which the string points squarely to the datum's counter-clockwise side. A block with predicates needs no seeds at all: an implementation MUST fall back to deterministic restarts when the seeds (or their absence) leave the home solve nowhere to start.
3. **A seed.** What neither an equation nor a predicate says, a seed says: the block's `at` seeds are places over the parameter and the formals, evaluation starts from them, and away from them continuity governs — an implementation MUST evaluate the curve as one continuation along the parameter, so the branch picked at the home is the branch everywhere. Deleting a seed still traces *a* branch, from a worse start — the same bargain a contact's `u = …` seed strikes in §4.3.

**Places, not coordinates.** Inside a trace block a seed is a *place*, and this language names places geometrically: `point t hint at c bearing (u + phase)` is the point at the edge of circle `c` at that bearing from the page's x-axis, and `point p hint at t` is wherever `t` starts (a point already named must be declared first). Both lower to exactly what the coordinate spelling would — the bearing form is `centre + r·(cos β, sin β)` said the way a draughtsman says it — so the coordinate form `hint at (xexpr, yexpr)` remains available and means the same thing. Outside a trace block a seed is a number a solve writes back, which a place named by reference is not, so the geometric forms are trace-block-only (**E103** elsewhere).

### 6.6 Chains **[0.4]**

Sugar, and only sugar: a chain writes a run of declarations and the constraints *between* them in one ordered breath, and it elaborates to exactly the statements a person would otherwise write out. It is a parser construct — nothing downstream of the parser knows it exists.

```
horizontal line bottom(b1, b2) tangent
arc a_br(center: c_br, r: r) tangent
vertical line right(r1, r2) tangent close
```

```
CHAIN  ::= LINK (JOINT LINK)* [JOINT "close"]
LINK   ::= PREFIX* DECL | REF
PREFIX ::= a constraint name whose spec is one entity slot     // horizontal, vertical
JOINT  ::= "to" | "tangent" | "equal" | INFIX
INFIX  ::= a constraint name whose spec is two entity slots    // perpendicular, equal_length, equal_radius
```

**[0.5] Two kinds of chain, told apart by their operands.** A chain whose links are DECLarations draws a **contour**: its joints are corners, and threading (below) applies. A chain whose links are REFerences states a **relation** among elements declared elsewhere — `a_br equal a_tr equal a_tl` — and threads nothing, because there is no corner it was written at and welding one would be an invention. A chain MUST NOT mix the two: the choice of threading rule would then be arbitrary. `to`, `tangent` and `close` are contour vocabulary and are errors in a relation chain.

- A **prefix** desugars to that constraint applied to the declaration it stands before: `horizontal line bottom(b1, b2)` is `line bottom(b1, b2)` plus `horizontal(bottom)`. Eligibility is registry-derived — one entity slot and nothing else — so a new unary constraint joins the grammar without the grammar changing.
- A **joint** stands between two links and says how they meet. Constraints return nothing, so a chain reads like a chained comparison, not an expression: each joint binds its two neighbours, and there is no precedence anywhere. `a equal b equal c` is therefore two statements, not three, and n operands give n−1 — the same rank as any other spanning set over the same elements, stated as a path rather than a star. An INFIX word is the two-argument counterpart of PREFIX, derived from the same registry: it desugars to that constraint over the pair, positionally, and MUST fit both — a word whose slots the pair cannot fill is an error, not a guess.
- **[0.5]** `equal` is **polymorphic**: `equal_length` between lines, `equal_radius` between circles or arcs, and an error between one of each, since no constraint equates a length to a radius. Like `tangent` it is drafting vocabulary rather than a constraint name, so no registry lookup can resolve it — the pair it stands between does. Where a chain declares its elements the keywords settle it as the program is read; where a chain names them it cannot be settled until the names are resolved, since a name may be declared further down the body (P2) or come from a component, so the word travels to elaboration and is settled there. Both report the same error.
- **Threading.** Every link of a chain is a line or an arc — an element with an entry and an exit, read left to right (`p1 → p2`; CCW, `start → end`). At each joint the shared point MUST be named by exactly one side, or by both in agreement; the name fills whichever boundary field the other side left out. An open chain's first entry and last exit are not joints and MUST be named where they stand. A kind with no boundary points — a circle — cannot sit in a chain.
- `JOINT close` after the last link seals the loop: the last exit threads to the first entry, and the joint says how they meet there.
- A statement otherwise ends at its line's end (§2); a line ending in a joint word continues its chain onto the next.

**Every joint is the regular form.** The joint knows the shared point, so `tangent` between a line and an arc is `tangent_arc_line(arc, line, at: start|end)` — tangent *at* the point just threaded — and never the bare tangency over a coincidence, whose Jacobian is rank-deficient at every solution. `tangent` between two lines is collinearity (`parallel` over the shared point). A pair the vocabulary has no regular form for (two arcs, today) is an error, never a silently degenerate statement. `to` states nothing: the shared point is the whole of a plain corner.

Each desugared statement keeps an identity of its own and a span into the chain's text, so a caret, a diagnosis culprit and a splice land on the word that stated the thing. (§12.7 is many instances from one statement; a chain is several statements from one *line*, each still its own.) Deleting a chain-borne constraint is therefore a word splice — a joint steps down to `to`, a prefix word goes where it stands — and deleting a link is refused: no splice takes one link out and leaves a chain behind, so that edit belongs to the source.

*Non-normative:* chains and paths (§10) answer different questions. A path is a traversal of geometry that already exists — vertices, the circles its arc segments lie on, orientation and branch rules, for boundary composition and export. A chain *declares* the geometry: it is how a contour's elements, their meetings and the levels on its straight runs are written down in the first place. The case library's fillet rectangle is the canonical chain; its longhand form states the same sketch in thirty statements.

---

## 7. Ports

```
port lead: Point          // form A: declare a fresh entity and export it
port hub = f0             // form B: export an existing entity under a new name
```

A port is **a name on the component boundary for an interior entity** — nothing more (P1). Ports carry no joint semantics, no constraint machinery, and no direction. Form A is sugar for a fresh declaration plus an export. Form B exports an alias.

At an instantiation site, `inst.portname` denotes the interior entity. Passing one component's port as another component's argument merges the two entities into one alias class.

*Non-normative:* joints between components are ordinary constraints written at the assembly site, e.g. `revolute(gear.hub, shaft.j3)`. The language deliberately has no "connect with joint" primitive; see the weld lint **W101** in §16.

---

## 8. Instances

```
t: Tooth(root, tip, slot: tau/N)
```

Instantiation elaborates the named component's body into the current scope with formals bound per §4.1. Instance elaboration is recursive; cyclic instantiation is an error (**E003**).

---

## 9. Constraints

### 9.1 Equational form

```
<expr> == <expr>
```

Both sides are dimension-checked. The residual is `lhs − rhs` (componentwise for future vector expressions; in this draft `==` applies to scalar-dimensioned expressions and to `Point` via `coincident`, §9.3).

### 9.2 Predicate form

```
on(root, lead, trail)
angle(lead, root.center, trail) == slot / 2
ccw(lead, tl, tr)
```

Predicates with more than the minimum arity distribute: `on(c, p1, p2, ..., pk)` means the conjunction of `on(c, pi)`.

### 9.3 Standard constraint library

Residual conventions: points are ℝ²; `×` is the scalar 2D cross product; `∠(u, v)` is the signed angle from `u` to `v` in (−π, π].

| Predicate | Residual(s) | Eq. count | Notes |
|---|---|---|---|
| `on(C: Circle, p: Point)` | ‖p − C.center‖ − C.r | 1 | |
| `on(L: Line, p: Point)` | n(L)·p − d(L) | 1 | |
| `coincident(p, q)` | p − q | 2 | for *distinct* entities; see **W100** |
| `distance(p, q) == e` | ‖p − q‖ − e | 1 | |
| `angle(a, b, c) == e` | ∠(a−b, c−b) − e | 1 | signed; see §9.4 |
| `parallel(L1, L2)` | sin(L1.dir − L2.dir) | 1 | |
| `perpendicular(L1, L2)` | cos(L1.dir − L2.dir) | 1 | |
| `tangent(C1, C2)` | ‖c1−c2‖ − (r1 + r2) *or* ‖c1−c2‖ − \|r1 − r2\| | 1 | branch by decoration, §9.5 |
| `tangent(C, L)` | dist(C.center, L) − C.r | 1 | |
| `equal(e1, e2)` | e1 − e2 | 1 | any matching dimension |
| `midpoint(m, a, b)` | m − (a+b)/2 | 2 | |
| `ccw(a, b, c)` | (b−a) × (c−a) > 0 | 0 | inequality; selects a connected component |
| `cw(a, b, c)` | (b−a) × (c−a) < 0 | 0 | |
| `revolute(f1: Frame, f2: Frame)` | f1.origin − f2.origin | 2 | relative angle free |
| `weld(f1: Frame, f2: Frame)` | f1.origin − f2.origin, f1.angle − f2.angle | 3 | triggers **W101** |

Implementations MAY extend this library. Extensions MUST document residuals and equation counts, and MUST classify each decoration as hint or constraint per P3.

### 9.4 Signed angles

`angle(a, b, c)` is the signed turn at vertex `b` from ray `b→a` to ray `b→c`, positive counterclockwise, in (−π, π]. Equating it to an expression is a 1-equation constraint. Programs that need the unsigned angle write `abs(angle(...))`; implementations MUST warn (**W102**) that `abs` introduces a branch (two solution families) unless an orientation predicate elsewhere disambiguates.

### 9.5 Branch decorations are constraints

Several predicates have discrete solution branches. Branch selection changes the solution set and is therefore constraint-class (P3), written as a decoration:

```
tangent.ext(c1, c2)      // external tangency: ‖c1−c2‖ = r1 + r2
tangent.int(c1, c2)      // internal tangency: ‖c1−c2‖ = |r1 − r2|
```

Undecorated `tangent` is an error (**E010**) — there is no default branch, because no consistent global rule exists for tangency the way one does for arcs (§10.3).

### 9.6 Inequalities

Orientation predicates (`ccw`, `cw`) are the only inequalities in this draft. They contribute no equations; they select among the discrete solution components of the equality system. Solvers MUST verify them on candidate solutions and MUST NOT report a solution violating one.

---

## 10. Paths

Paths are the one ordered construct (P2, exception). A path is a directed traversal of vertices connected by segments.

### 10.1 Grammar

```
path outline: ccw = lead -> tl ~tip~ tr -> trail
```

- `path NAME : ORIENT = PATHEXPR` declares a named path with orientation `ccw` or `cw`.
- A bare `PATHEXPR` statement is an anonymous **path fragment** (used for splicing, §10.5).
- Segments: `a -> b` is a straight segment; `a ~C~ b` is an arc on circle `C`; `a ~C rev~ b` is the reversed-branch arc (§10.3).

### 10.2 Orientation

Every path or fragment containing an arc segment MUST have an orientation, either declared (`: ccw`) or inherited (§10.5). A closed path's declared orientation MUST match the winding of its solved vertex sequence; mismatch is a solve-time error (**E011**).

### 10.3 Arc branch rule (the consistent default)

> **An arc segment traverses its circle in the direction of the path's orientation.**

In a `ccw` path, `a ~C~ b` is the counterclockwise arc on `C` from `a` to `b`; in a `cw` path, the clockwise arc. The decoration `rev` selects the opposite branch. `rev` is constraint-class (P3): it changes the solution set of the *shape* (and, where arc-length or containment constraints reference the path, of the coordinate system too).

*Non-normative:* this rule makes convex-ish boundaries annotation-free. Tracing a gear outline counterclockwise, every tip arc and every root gap arc is counterclockwise on its own circle; no segment needs `rev`.

### 10.4 Derived constraints

An arc segment `a ~C~ b` implies `on(C, a)` and `on(C, b)`. These derived incidences enter the constraint store subject to deduplication (§14.3), so restating them explicitly is legal and free.

### 10.5 Fragment composition

Path fragments compose by **endpoint identity**: two fragments whose end and start vertices are the same alias class concatenate. A fragment without declared orientation inherits the orientation of the (unique) named path it composes into; if composition is ambiguous or orientations conflict, error **E012**. A set of fragments whose composition closes (every vertex has in-degree = out-degree = 1) forms a closed boundary; implementations MUST report boundaries that fail to close when a closed boundary is demanded by export (**E013**).

*Non-normative:* this is how the gear outline is assembled: each `Tooth` contributes `lead → tl ⌒ tr → trail`, and the `ring` body contributes the gap arc `t.trail ~root~ next.t.lead`. Under ring elaboration the fragments chain around and close at instance N−1 → 0 with no seam case.

---

## 11. Hints

**[0.2]** A hint is one of the two seed forms; the other, and the primitive, is the inline seed of §6.4. Everything in this section applies to both, and "hint" below should be read as "seed". Use `hint` when inline cannot say it: seeding an entity declared elsewhere, or seeding from an expression over other geometry.

```
hint t.lead at center + polar(root.r, 0)
```

`hint REF at EXPR` seeds the entity `REF` at the value of `EXPR` (evaluated with whatever definitional values and previously seeded values are available; unseeded quantities in a hint expression are an error **E014**).

Normative invariant (**Invariant H**): *for every program P, sol(P) = sol(P minus all seeds).* Implementations MUST maintain a statement classification sufficient to verify Invariant H syntactically — i.e., the seed class is closed under everything the grammar allows in a seed, and nothing in the seed class can generate residuals or alter aliasing.

**[0.2]** §4.3 is how that classification is meant to be maintained: `=` is seed-class and `==` is constraint-class, so the check is a look at the mark rather than an argument about the statement.

Hints on entities inside a `ring` seed the fundamental-domain representative (§12.4). Hints MAY use block indices in `repeat`/`cycle` (where each instance is a distinct variable) and MUST NOT use them in `ring` (**E015**: there is only one representative to seed).

---

## 12. Repetition

Three constructs, three meanings. All take a compile-time `Int` count and an optional index binder.

### 12.1 `repeat` — open array

```
repeat N as i { ... }
```

Pure elaboration: N copies of the body, index `i` ∈ 0..N−1 available in expressions and hints. `next`/`prev` are illegal in `repeat` (**E020**); cross-instance references use explicit indexing `name[k]` from outside or arithmetic indexing patterns from inside.

### 12.2 `cycle` — structural closure, no symmetry

```
cycle N as i { ... }
```

Elaborates N copies; `next` denotes instance (i+1) mod N and `prev` instance (i−1) mod N. Instances are independent variables; nothing forces them to resemble one another. Use for closed chains of unequal links.

### 12.3 `ring` — cyclic symmetry claim

```
ring N about center as i { ... }
```

**Semantics.** Let g = Rot(center, τ/N), the rotation by τ/N about the point entity named in the `about` clause. Define the unrolled program U = the same body under `cycle N`. Then:

> sol(`ring`) = { x ∈ sol(U) : instance i+1 of every ring-local entity equals g · (instance i) }.

That is, `ring` ≡ `cycle` + symmetry constraints. `ring` is constraint-class: it restricts the solution set to the C_N-symmetric solutions. This is a normative equivalence — an implementation MAY literally elaborate to `cycle` plus per-instance rotation equalities and MUST get the same solution set as one that solves in the quotient.

**[0.2] An implementation that unrolls MUST report that it did**, per `ring`, wherever it reports the DOF ledger (§16.3). The solution sets match; nothing else does. A `ring` states symmetry so that an implementation can *exploit* it, and an unrolled one gives every bit of that back: measured on a 30-tooth gear, unrolling put the wheel outside the cluster vocabulary entirely (so it fell to a numeric residual), past the size at which a drag can be answered by moving rigid bodies, and past the size at which the numeric rank cross-check runs at all — so the dependency reporting of §16 silently switched off. None of that is visible in the solution set, and all of it is visible to a user. 0.1 let an implementation take the licence in §12.3 and skip the SHOULD in §12.4 without ever saying so; that is the gap this closes.

**The `about` clause is mandatory.** The axis point MUST be an entity invariant under g — which for a rotation means the axis point itself (trivially) — and MUST be declared outside the ring.

### 12.4 Fundamental-domain solving (SHOULD)

Implementations SHOULD solve a `ring` in the quotient: one representative per ring-local entity name; a reference `next.e` inside the body denotes g·(representative of e); `prev.e` denotes g⁻¹·(representative of e); an external reference `name[k].e` denotes gᵏ·(representative of e). Every body constraint is instantiated once over representatives with group-element annotations. Solutions lift by orbit expansion xᵢ = gⁱ·x₀.

*Non-normative:* this is N× fewer unknowns and structurally excludes asymmetric spurious roots and permutation-collapsed roots. It is why one hint seeds one tooth and the gear cannot stack its teeth.

### 12.5 Invariance of external references

An entity declared **outside** a ring and referenced **inside** it MUST be invariant under g. Implementations MUST verify this by the following syntactic criterion, and MAY additionally prove invariance semantically:

- the axis point itself: invariant;
- a `Circle` whose `.center` is (an alias of) the axis point: invariant;
- any value-typed entity (`Scalar`, `Length`, `Angle` used as magnitude): invariant;
- everything else: **not** established — error **E021** ("entity referenced in ring is not C_N-invariant").

*Non-normative:* E021 is one of the language's best diagnostics; it converts "the solver produced something weird and asymmetric" into a precise compile-time message.

### 12.6 Nesting

Nested `repeat`/`cycle` inside `ring` (and vice versa) is legal. Nested `ring` inside `ring` requires the inner axis to be invariant under the outer generator; implementations MAY reject nested `ring` in this draft (**E022**, "nested ring not supported") and MUST NOT silently mis-solve it. Full nested-group semantics is deferred (§17).

### 12.7 The identity of a statement under expansion **[0.2]**

> **A statement inside a `repeat`, `cycle` or `ring` body is ONE statement, however many things it makes.** What tells its instances apart is the **instance path**, not the statement.

A `cycle` of thirty makes thirty entities from one line of source. That line is the statement: it is what a span points at, what a caret lands on, what a diagnostic names and what an edit rewrites. The thirty are distinguished by the sequence of block indices reached to get to each — outermost first — which an implementation MUST record alongside whatever it records about where an entity came from.

An implementation MUST NOT give each expanded copy a statement identity of its own.

**Why this is normative rather than an implementation detail.** It decides whether the language can be edited at all. Give each copy its own identity and every entity a `cycle` or a component produced names a statement that appears nowhere in the source — so a caret in the text cannot find what it draws, a diagnostic cannot point at the line that caused it, and an edit computed against a span has nothing to splice. It also hides exactly the fact a seed writeback needs (§4.3): a statement reached thirty times has thirty poses and no single one to record, and that is visible only if the thirty agree on which statement they are.

An implementation MAY still need a per-instance key for its own tables. That key is `(statement, path)`, and it is not a statement.

---

## 13. Gauge fixing

Well-posed models are typically invariant under rigid motion; the Jacobian is rank-deficient by design. The language names this freedom rather than letting the solver pick:

```
ground(center)                        // pins a Point: removes 2 DOF
fix(direction(center, t.lead))        // pins a bearing: removes 1 DOF
fix(<scalar expr>)                    // pins any 1-DOF quantity at its hinted/current value
```

`ground` and `fix` are constraint-class. `fix(e)` constrains `e` to the value obtained from hints/definitions at elaboration; if no such value is determined, error **E030**. Implementations MUST report residual gauge freedom (rank deficiency whose null space is spanned by rigid motions) with the suggestion to add `ground`/`fix` (**W103**), and MUST distinguish it from genuine under-constraint.

### 13.1 Document state travels on its statement **[0.2]**

A document generally carries more than the program: where an annotation was dragged to, which of several solutions the drawing is on, and whatever else a tool needs to reopen a drawing as its author left it.

> **Every such datum MUST be attached to the statement it qualifies, and MUST NOT be keyed by a statement's position in a body or by an entity's index.**

This is P2 applied to the document rather than to the program text, and it is stated separately because it is the half implementations get wrong. Reordering a body is required to preserve meaning (P2); a body can be reordered by an editor, by a code formatter, or by an implementation's own printer. Two keys make that reordering destructive:

- **position in a list.** An annotation stored as "the 7th constraint's placement" follows the 7th position when the 7th statement moves, so it silently reappears on some other statement's annotation.
- **an entity's index.** A recorded solution branch stored as a triple of point indices goes *inert* when the points are renumbered: the document still carries it, a reader still loads it, and fewer of them apply. Nothing reports anything.

Both failures are silent, and both are invisible to any test that checks only the solution set — which is why 0.1 could assert P2 in the parser and lose it in the file format. Where a datum has no statement to ride on, it MUST name what it qualifies (an entity by name, a solution branch by the names of the points that orient it) rather than by index.

---

## 14. Elaboration semantics

Elaboration lowers a program to the **kernel form** consumed by solvers. The pipeline is normative in effect, not in mechanism.

### 14.1 Phases

1. **Instance expansion.** Recursively inline component bodies for the root's instance tree, freshening names by instance path. `repeat`/`cycle` unroll. `ring` either unrolls to `cycle` + symmetry constraints (§12.3) or lowers to quotient form (§12.4); the two MUST be solution-equivalent.
2. **Alias resolution.** Union-find over all names, merging classes for: port exports, entity-typed arguments, `port x = y`, and constructor entity-arguments. Each class gets one representative entity. Type mismatch within a class is an error (**E040**).
3. **Definitional substitution.** Definitional equalities (constructor value-arguments, `param`, `= expr` declarations) are substituted, METAFONT-style: they are not residuals and consume no solver iterations. A cyclic definitional dependency is an error (**E041**).
4. **Constraint collection.** Predicate statements, `==` equations, derived path incidences (§10.4), symmetry constraints, and gauges are collected into the constraint store.
5. **Path assembly.** Fragments compose per §10.5 into boundary curves attached to the model as derived objects.

### 14.2 Kernel form

The kernel is a **bipartite entity/constraint graph, quotiented by group actions**:

```
Kernel := {
  groups:      [ { id, order N, axis: EntityRef } ],
  entities:    [ { id, type, dof, orbit: Fixed | Orbit(group, size N), seed? } ],
  constraints: [ { pred, args: [ (entity, power) ], params, class: Eq|Ineq, span } ],
  gauges:      [ ... ],
  paths:       [ ordered segment lists over (entity, power) refs ],
}
```

- `orbit: Fixed` marks entities invariant under the relevant group (axis, on-axis circles, scalars).
- Constraint arguments carry a **group power**: `(e, +1)` means "the image of e under the generator" — this is how `t.trail ~root~ next.t.lead` appears with one Tooth's worth of variables.
- `span` is a source location; every kernel object MUST be traceable to source for diagnostics.

*Non-normative:* with all groups trivial this degrades to exactly a SketchGraphs-style bipartite graph, which is the intended interchange representation for external tooling.

### 14.3 Deduplication

The constraint store is a **set**: two constraints identical after alias resolution and definitional substitution (same predicate, same argument classes and powers, same parameters) are one constraint. This makes derived path incidences free when also stated explicitly, and it is the reason redundancy diagnostics (§16) report *semantic* redundancy rather than syntactic repetition.

---

## 15. Solver contract

The numerical method is unspecified. Whatever the method, a conforming solver:

- MUST treat alias classes as single variables (no residuals for binding — P1).
- MUST apply definitional substitutions before iteration (§14.1 phase 3).
- MUST use hints as initial values only (Invariant H).
- MUST verify all inequalities and declared path orientations on any reported solution.
- MUST NOT report a solution to a `ring` program that is not symmetric (§12.3).
- SHOULD decompose along component boundaries (P4) and solve `ring` in the quotient (§12.4).
- MUST report the diagnostics of §16 with source spans; "solver did not converge" without a structural diagnosis is a nonconforming failure mode when a structural diagnosis is computable (rank analysis at the seed or at the failure point).

---

## 16. Static and solve-time diagnostics

### 16.1 Errors

| Code | Condition |
|---|---|
| E001 | redeclaration within a body |
| E002 | block binder shadows outer name |
| E003 | cyclic component instantiation |
| E010 | undecorated `tangent` between circles |
| E011 | closed path winding contradicts declared orientation |
| E012 | ambiguous or conflicting path-fragment composition |
| E013 | boundary fails to close where closure demanded |
| E014 | hint expression references unseeded/undetermined quantity |
| E015 | indexed hint inside `ring` |
| E020 | `next`/`prev` in `repeat` |
| E021 | external entity referenced in `ring` not provably invariant |
| E022 | nested `ring` (if unsupported) |
| E030 | `fix` target has no determined value |
| E040 | type mismatch within an alias class |
| E041 | cyclic definitional dependency |
| E050 | inconsistent system (no solution); report a minimal infeasible subset when computable |

### 16.2 Warnings and lints

| Code | Condition |
|---|---|
| W100 | `coincident(p, q)` where making `p`,`q` one entity would suffice — "consider binding instead of constraining" |
| W101 | frames fully welded by constraints — "consider a port alias" |
| W102 | `abs(angle(...))` without a disambiguating orientation predicate |
| W103 | rank deficiency spanned by rigid motions — "add ground/fix" |
| W104 | under-constrained: report the number of residual DOF and, when computable, a basis of unconstrained motions attributed to source entities |
| W105 | consistent redundancy: constraints dependent on others; report the dependent set with spans |

### 16.3 DOF ledger

Implementations SHOULD emit, on request, a degrees-of-freedom ledger: per alias class, its free DOF after definitional substitution; per constraint, its equation count; totals per component and for the model; gauge accounting. (§18.3 shows the gear's ledger.)

---

## 17. Deferred and open issues (non-normative)

1. **3D lift.** `Frame` generalizes; `ring` generalizes to rotation about a line; the arc-branch rule needs a replacement (no global winding in 3D). Joint library grows (revolute gains an axis argument, add prismatic/cylindrical/spherical).
2. ~~**Curve entities.**~~ **[0.2] Settled — see §6.5.** A curve is a *family declared in the document*, two expressions over the geometry it is drawn from, rather than an entity kind per curve. Involute, cycloid and trochoid are library code. What remains open is `tangent` against such a curve (one more order in the parameter, and what a *mating* gear needs) and the path grammar's slot for a curve segment.
3. **Nested symmetry groups.** Semantics of `ring` in `ring` beyond the reject-or-elaborate rule of §12.6 (planetary sets are the motivating case: carrier symmetry ≠ sun symmetry).
4. **Constraint strengths.** A Cassowary-style required/strong/weak hierarchy for graceful over-constraint. Interacts with P3 (a weak constraint changes the solution set; it is constraint-class) and with diagnostics (W105 becomes resolution, not warning).
5. **Reflection symmetry.** `mirror about <line>` as a second group kind; the kernel's group table already permits it.
6. **Inequality vocabulary.** Beyond orientation: `inside`, `min_distance`, clearance constraints.
7. **Assemblies and motion.** A `mechanism` layer where some constraints are joints with time-varying free coordinates; out of scope for the static solve contract.

---

## 18. Worked example: spur gear with revolute teeth

**[0.2]** The tooth below is drawn with straight `->` flank segments, because 0.1 had no way to say
what an involute is (§17.2). §6.5 now does, and a flank written that way is a curve rather than a
chord across one:

```
curve involute(c: circle, phase: Angle)(u) =
  ( c.center.x + c.r * (cos(u + phase) + u * pi / 180 * sin(u + phase)),
    c.center.y + c.r * (sin(u + phase) - u * pi / 180 * cos(u + phase)) )

component Flank(base: circle, root: circle, tip: circle,
                phase: Angle, u0: Angle, u1: Angle) {
  curve e = involute(base, phase: phase) over (u0, u1)
  port lo: point
  port hi: point
  point_on_curve(lo, e, u = u0)      point_on_circle(lo, root)
  point_on_curve(hi, e, u = u1)      point_on_circle(hi, tip)
}
```

Nothing there says *where* the flank goes. It says the curve is the involute of the base circle at
this bearing, that it begins where it crosses the root circle and ends where it crosses the tip —
and the solver finds the two rolls that satisfy it. There is no closed form for either, which is
the point: the shape of the tooth is a solve, not arithmetic performed in a `param` block.

One caution the geometry imposes and the language cannot: **below the base circle there is no
involute.** A textbook 1.25·m dedendum on a small tooth count puts the root circle inside the base
circle, where `u0` has no real value; a conforming implementation reports that rather than fudging
it.

The 0.1 source is kept below as written, since §18.2 and §18.3 walk through it.

### 18.1 Source

```
component Tooth(root: Circle, tip: Circle, slot: Angle) {
  port lead:  Point
  port trail: Point
  point tl, tr

  on(root, lead, trail)
  on(tip,  tl, tr)

  path outline: ccw = lead -> tl ~tip~ tr -> trail

  angle(lead, root.center, trail) == slot / 2
  angle(lead, root.center, tl) == angle(tr, root.center, trail)
  ccw(lead, tl, tr)
}

component Gear(N: Int, m: Length) {
  param R = m * N / 2
  point  center
  circle pitch(center, R)
  circle tip(center, R + m)
  circle root(center, R - 1.25*m)

  frame f0 = frame(center, ray(center, t[0].lead))
  port hub = f0

  ring N about center as i {
    t: Tooth(root, tip, slot: tau/N)
    t.trail ~root~ next.t.lead
    angle(t.trail, center, next.t.lead) == tau/(2*N)
  }

  hint t.lead at center + polar(root.r, 0)

  ground(center)
  fix(direction(center, t.lead))
}
```

### 18.2 Elaboration walk-through

- **Aliasing:** `pitch.center`, `tip.center`, `root.center`, `f0.origin`, and `center` form one class. `Tooth`'s formals `root`, `tip` alias the gear's circles (entity-typed arguments). The three circles' radii are definitional (`R`, `R+m`, `R−1.25m` substitute out).
- **Ring lowering:** group G = C_N about `center`. Ring-local free entities: one representative each of `t.lead`, `t.trail`, `t.tl`, `t.tr` (orbit size N). `center` and the circles pass the §12.5 invariance criterion (`Fixed`). The gap constraints reference `(t.lead, +1)`.
- **Paths:** each Tooth contributes `lead → tl ⌒tip tr → trail` (ccw); the ring body contributes `t.trail ⌒root (t.lead,+1)`, inheriting ccw. Composition closes after N teeth and N gaps: one closed ccw boundary. All arcs take the default branch (ccw on their circles) — zero `rev` decorations, per §10.3.
- **Deduplication:** the arc-derived incidences `on(tip, tl)`, `on(tip, tr)`, `on(root, trail)`, `on(root, (lead,+1))` duplicate the explicit `on` constraints (the last after symmetry transport) and merge in the store.

### 18.3 DOF ledger (quotient system)

| Item | DOF / equations |
|---|---|
| `center` | +2 |
| circle radii | 0 (definitional) |
| `t.lead`, `t.trail`, `t.tl`, `t.tr` (representatives) | +8 |
| **Unknowns** | **10** |
| `on(root, lead)`, `on(root, trail)` | −2 |
| `on(tip, tl)`, `on(tip, tr)` | −2 |
| tooth span angle `== slot/2` | −1 |
| flank symmetry angle equality | −1 |
| gap angle `== tau/(2N)` | −1 |
| `ground(center)` | −2 |
| `fix(direction(center, t.lead))` | −1 |
| **Equations** | **10** |
| `ccw(lead, tl, tr)` | 0 (inequality) |

Square system, full rank at the hinted seed; one Newton basin per §12.4, lifted to N teeth by orbit expansion. Expected solution: lead at bearing 0 on the root circle, trail at bearing τ/2N, tip corners inset symmetrically — a gear.

---

## 19. Grammar (EBNF)

```ebnf
program        = { component } ;
component      = "component" IDENT "(" [ params ] ")" "{" { statement } "}" ;
params         = param { "," param } ;
param          = IDENT ":" type ;
type           = "Int" | "Scalar" | "Length" | "Angle"
               | "Point" | "Line" | "Circle" | "Frame" | "Path" ;

statement      = decl | constraint | hint | gauge | block | path_decl | frag ;

decl           = entity_decl | param_decl | port_decl | curve_def | instance_decl ;
entity_decl    = ekw binder { "," binder }
               | ekw IDENT "=" expr ;
ekw            = "point" | "circle" | "line" | "frame" | "ellipse" | "spline" | "curve" ;
(* the geometric `at` forms — `at t`, `at c bearing (…)` — are trace-block seeds, §6.5.1 *)
binder         = IDENT [ "(" ctor_arg { "," ctor_arg } ")" ]
                 [ "at" ( "(" expr "," expr ")" | ref [ "bearing" "(" expr ")" ] ) ] ;
ctor_arg       = [ IDENT ":" ] expr ;                      (* value args are SEEDS, §6.2 *)

(* a curve FAMILY, §6.5; an instance is an ordinary entity_decl *)
curve_def      = "curve" IDENT "(" [ params ] ")" "(" IDENT ")"
                 [ "over" "(" expr "," expr ")" ]
                 "=" ( "(" expr "," expr ")"
                     | "trace" IDENT [ "from" "(" expr ")" ] "where" "{" { stmt } "}" ) ;
param_decl     = "param" IDENT "=" expr ;
port_decl      = "port" IDENT ":" type
               | "port" IDENT "=" ref ;
instance_decl  = IDENT ":" IDENT "(" [ args ] ")" ;
args           = arg { "," arg } ;
arg            = [ IDENT ":" ] expr ;

constraint     = expr "==" expr
               | pred [ "." IDENT ] "(" args ")" [ "==" expr ] ;  (* decoration e.g. tangent.ext *)
pred           = IDENT ;

(* §4.3: inside an argument list, `=` seeds and `==` pins.  The two are the whole of the
   seed/constraint classification, and they are told apart by the mark alone. *)
arg            = [ IDENT ":" ] expr
               | IDENT "=" expr                            (* seed: a solve may move it *)
               | IDENT "==" expr ;                         (* pin:  a solve may not *)

path_decl      = "path" IDENT ":" orient "=" path_expr ;
frag           = path_expr ;                               (* statement-level fragment *)
orient         = "ccw" | "cw" ;
path_expr      = ref seg ref { seg ref } ;
seg            = "->" | "~" ref [ "rev" ] "~" ;

hint           = "hint" ref "at" expr ;
gauge          = "ground" "(" ref ")" | "fix" "(" expr ")" ;

block          = "repeat" expr [ "as" IDENT ] "{" { statement } "}"
               | "cycle"  expr [ "as" IDENT ] "{" { statement } "}"
               | "ring"   expr "about" ref [ "as" IDENT ] "{" { statement } "}" ;

ref            = ( IDENT | "next" | "prev" ) { "." IDENT | "[" expr "]" } ;
expr           = expr addop term | term ;
term           = term mulop factor | factor ;
factor         = NUMBER [ UNIT ] | "tau" | "pi" | ref | call
               | "(" expr ")" | "-" factor ;
call           = IDENT "(" [ args ] ")" ;                  (* polar, direction, angle, abs, ... *)
addop          = "+" | "-" ;
mulop          = "*" | "/" ;
```

Parsing note: a statement beginning with an expression is disambiguated by the token following the first `ref`/`expr`: `==` → constraint; `->` or `~` → fragment; otherwise error. `ccw`/`cw` appear both as orientation keywords (after `:` in `path`) and as predicates; context disambiguates. **[0.2]** `curve NAME(` opens a family definition and `curve NAME =` an instance; the token after the name settles which.

**[0.2] Note on the trailing `==`.** After the `==` that follows a predicate's closing parenthesis, an implementation MAY take the rest of the logical line verbatim rather than tokenizing it, and hand that text to whatever evaluates dimension expressions. This is not laziness: `3 1/2` is three and a half and `31/2` is a division, and that rule belongs to one tokenizer. Two copies of it are two rules the moment one is edited. An `==` *inside* an argument list is the pin of §4.3 and is lexed normally; the two never meet.

---

## 20. Conformance checklist for a first implementation

A minimal conforming implementation provides:

1. Parser for §19; classifier assigning every statement to §4.2 classes **by the `=` / `==` mark (§4.3)**; Invariant H enforced by construction.
2. Elaborator: instance expansion **preserving statement identity (§12.7)**, union-find aliasing, definitional substitution, dedup store, `ring` lowering (quotient form or cycle-plus-symmetry — either, per §12.3, **and reported if unrolled**).
3. Invariance check §12.5 (syntactic criterion), gauge analysis (W103), DOF ledger (§16.3).
4. A numeric backend satisfying §15 — Newton on the quotient system seeded by hints is sufficient — with rank-deficiency reporting attributed to source spans.
5. Path assembly and closed-boundary export (the solved gear outline as a polyline+arc sequence).
6. **[0.2]** Document state attached to its statement, never to a list position or an entity index (§13.1).

Deliberately *not* required for v0: 3D, nested rings, constraint strengths, decomposition planning (P4 is a SHOULD). **[0.2]** Curve families (§6.5) are not required either, but they are no longer deferred: an implementation that wants involute or cycloid geometry has a way to say it, and needs no new entity kind to do so.
