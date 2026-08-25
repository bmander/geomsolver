# Solvent: A Declarative Language for Constrained Geometry

**Specification, Draft 0.1 — August 2026**

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

**P3 — Hints are semantically inert.**
Deleting every `hint` statement from a program MUST NOT change its solution set. Hints may change which solution a solver finds, or whether it converges, but never what counts as a solution. Any annotation that changes the solution set (orientation predicates, arc branch selection, tangency side) is a **constraint** and is classified as such by the implementation, regardless of surface syntax.

**P4 — Component boundaries are decomposition structure.**
A component is solvable against its ports. Implementations SHOULD exploit the component instance tree as a decomposition plan (solve interiors against port entities; solve the inter-component system over ports).

**P5 — Symmetry is a claim, not a macro.**
The `ring` construct asserts cyclic symmetry. Its solution set contains exactly the symmetric solutions of the corresponding unrolled system. Implementations SHOULD solve in the fundamental domain.

### 1.2 Scope of this draft

This draft specifies **planar (2D) geometry only**. The entity vocabulary, constraint library, and group actions are two-dimensional. Section 17 lists the known lifting questions for 3D. This draft also excludes curve entities beyond lines and circles (no involutes, splines, or conics); see §17.

### 1.3 Conformance keywords

MUST, MUST NOT, SHOULD, SHOULD NOT, and MAY are used as in RFC 2119. Text marked *non-normative* is explanatory.

---

## 2. Lexical structure

- **Identifiers:** `[A-Za-z_][A-Za-z0-9_]*`. Component names are conventionally capitalized; this is not enforced.
- **Keywords:** `component`, `port`, `param`, `point`, `circle`, `line`, `frame`, `path`, `repeat`, `cycle`, `ring`, `about`, `as`, `next`, `prev`, `hint`, `at`, `ground`, `fix`, `ccw`, `cw`, `rev`, `true`, `false`.
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
| `Line` | undirected infinite line | 2 |
| `Circle` | center + radius | 3 |
| `Frame` | origin + orientation | 3 |
| `Path` | directed piecewise boundary curve | 0 (derived object) |

### 3.2 Sub-entities

Compound entities expose sub-entities by field access. Sub-entities are ordinary entities and participate in aliasing and constraints.

| Entity | Field | Type |
|---|---|---|
| `Circle` | `.center` | `Point` |
| `Circle` | `.r` | `Length` |
| `Frame` | `.origin` | `Point` |
| `Frame` | `.angle` | `Angle` |
| `Line` | `.dir` | `Angle` (mod π) |

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
| **Declaration** | entity declarations, `param`, `port`, instance declarations | introduces entities/aliases |
| **Constraint** | predicate calls, `==` equations, orientation predicates, arc-branch and tangency-side decorations, `ring` symmetry, gauge statements | **yes** |
| **Hint** | `hint ... at ...` | **no (P3)** |
| **Structure** | `repeat`/`cycle` blocks, `path` declarations (net of their derived constraints, §10.4) | organizational |

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
- An argument of **value type** *defines* the corresponding coordinate: it introduces a definitional equality `pitch.r == R` that the implementation MUST treat as a substitution, not a residual (see §14.2).

The `name = expr` form declares an entity wholly defined by an expression; all its coordinates are definitional.

### 6.3 `param`

```
param R = m * N / 2
```

Introduces a named definitional value. `param` values are evaluated at elaboration time when all inputs are `Int`/literal, otherwise they are definitional scalars.

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

```
hint t.lead at center + polar(root.r, 0)
```

`hint REF at EXPR` seeds the entity `REF` at the value of `EXPR` (evaluated with whatever definitional values and previously seeded values are available; unseeded quantities in a hint expression are an error **E014**).

Normative invariant (**Invariant H**): *for every program P, sol(P) = sol(P minus all hint statements).* Implementations MUST maintain a statement classification sufficient to verify Invariant H syntactically — i.e., the hint class is closed under everything the grammar allows in a hint, and nothing in the hint class can generate residuals or alter aliasing.

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

---

## 13. Gauge fixing

Well-posed models are typically invariant under rigid motion; the Jacobian is rank-deficient by design. The language names this freedom rather than letting the solver pick:

```
ground(center)                        // pins a Point: removes 2 DOF
fix(direction(center, t.lead))        // pins a bearing: removes 1 DOF
fix(<scalar expr>)                    // pins any 1-DOF quantity at its hinted/current value
```

`ground` and `fix` are constraint-class. `fix(e)` constrains `e` to the value obtained from hints/definitions at elaboration; if no such value is determined, error **E030**. Implementations MUST report residual gauge freedom (rank deficiency whose null space is spanned by rigid motions) with the suggestion to add `ground`/`fix` (**W103**), and MUST distinguish it from genuine under-constraint.

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
2. **Curve entities.** Involutes, splines, conics as first-class entities with `on`/`tangent` support. The gear's flanks become `involute(base_circle)` segments; the path grammar already has the slot (`~inv~`).
3. **Nested symmetry groups.** Semantics of `ring` in `ring` beyond the reject-or-elaborate rule of §12.6 (planetary sets are the motivating case: carrier symmetry ≠ sun symmetry).
4. **Constraint strengths.** A Cassowary-style required/strong/weak hierarchy for graceful over-constraint. Interacts with P3 (a weak constraint changes the solution set; it is constraint-class) and with diagnostics (W105 becomes resolution, not warning).
5. **Reflection symmetry.** `mirror about <line>` as a second group kind; the kernel's group table already permits it.
6. **Inequality vocabulary.** Beyond orientation: `inside`, `min_distance`, clearance constraints.
7. **Assemblies and motion.** A `mechanism` layer where some constraints are joints with time-varying free coordinates; out of scope for the static solve contract.

---

## 18. Worked example: spur gear with revolute teeth

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

decl           = entity_decl | param_decl | port_decl | instance_decl ;
entity_decl    = ekw binder { "," binder }
               | ekw IDENT "=" expr ;
ekw            = "point" | "circle" | "line" | "frame" ;
binder         = IDENT [ "(" expr { "," expr } ")" ] ;
param_decl     = "param" IDENT "=" expr ;
port_decl      = "port" IDENT ":" type
               | "port" IDENT "=" ref ;
instance_decl  = IDENT ":" IDENT "(" [ args ] ")" ;
args           = arg { "," arg } ;
arg            = [ IDENT ":" ] expr ;

constraint     = expr "==" expr
               | pred [ "." IDENT ] "(" args ")" ;        (* decoration e.g. tangent.ext *)
pred           = IDENT ;

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

Parsing note: a statement beginning with an expression is disambiguated by the token following the first `ref`/`expr`: `==` → constraint; `->` or `~` → fragment; otherwise error. `ccw`/`cw` appear both as orientation keywords (after `:` in `path`) and as predicates; context disambiguates.

---

## 20. Conformance checklist for a first implementation

A minimal conforming implementation provides:

1. Parser for §19; classifier assigning every statement to §4.2 classes; Invariant H enforced by construction.
2. Elaborator: instance expansion, union-find aliasing, definitional substitution, dedup store, `ring` lowering (quotient form or cycle-plus-symmetry — either, per §12.3).
3. Invariance check §12.5 (syntactic criterion), gauge analysis (W103), DOF ledger (§16.3).
4. A numeric backend satisfying §15 — Newton on the quotient system seeded by hints is sufficient — with rank-deficiency reporting attributed to source spans.
5. Path assembly and closed-boundary export (the solved gear outline as a polyline+arc sequence).

Deliberately *not* required for v0: 3D, curves beyond line/circle, nested rings, constraint strengths, decomposition planning (P4 is a SHOULD).
