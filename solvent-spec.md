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

Solvent is a declarative language for describing planar geometry under constraint. A Solvent program does not construct geometry step by step; it declares a set of entities, relations among them, and structural facts (symmetry, connectivity), and delegates the discovery of coordinates to a solver. The language borrows its module discipline from hardware description languages: designs are built from **components**, component bodies are **unordered**, and connection between components is **aliasing** (net formation), not constraint.

### 1.1 Design principles (normative)

These principles are binding on the rest of the specification. Where a detailed rule appears to conflict with a principle, the principle governs and the conflict is a defect in this document.

**P1 — Connection is aliasing; constraint is explicit.**
Passing an entity as an instance argument makes two names refer to *one* entity **[0.13]** (a port once did too; §7). Aliasing has zero solver cost and cannot be violated. A constraint always relates *distinct* entities and always contributes residual equations (or inequalities) to the system.

**P2 — Component bodies are unordered.**
Statements within a component body form a set. Reordering the statements of a body MUST NOT change the meaning of a program. Statements interact only through the entities they name. The only ordered contexts in the language are the interiors of single statements where order is semantic (path traversals, argument lists).

**[0.2]** P2 binds the *document* as well as the program text: every piece of state a document carries MUST be attached to the statement it qualifies, and MUST NOT be keyed by a statement's position in a body or by an entity's index (§13.1). Without that rule P2 is an assertion an implementation can satisfy in the parser and lose in the file format — and it fails silently, which is the worst way for it to fail.

**P3 — Hints are semantically inert.**
Deleting every seed from a program MUST NOT change its solution set. Seeds may change which solution a solver finds, or whether it converges, but never what counts as a solution. Any annotation that changes the solution set (orientation predicates, arc branch selection, tangency side) is a **constraint** and is classified as such by the implementation, regardless of surface syntax.

**[0.2]** The two are told apart by one mark, not by analysis: a number written with `=` is a seed and a solver may rewrite it; a number written with `==` is a constraint and a solver MUST NOT (§4.3). This is what makes Invariant H (§11) checkable *syntactically*, as that section already requires.

**P4 — Component boundaries are decomposition structure.**
A component is solvable against the entities it is written over. Implementations SHOULD exploit the component instance tree as a decomposition plan (solve interiors against the formals; solve the inter-component system over them).

**P5 — Symmetry is a claim, not a macro.**
The `ring` construct asserts cyclic symmetry. Its solution set contains exactly the symmetric solutions of the corresponding unrolled system. Implementations SHOULD solve in the fundamental domain, and one that does not MUST say so (§12.3).

### 1.2 Scope of this draft

This draft specifies **planar (2D) geometry only**. The entity vocabulary, constraint library, and group actions are two-dimensional. Section 17 lists the known lifting questions for 3D. This draft also excludes curve entities beyond lines and circles (no involutes, splines, or conics); see §17.

**[0.18] More precisely: the draft specifies planar geometry *solved*, with solids evaluated over it.** A document may say what object its drawing is of — a face is a region of a plane (§6.8), a solid is a face swept or a term over other solids (§6.9), and a view or a section of one is a picture the sheet asks for (§6.11) — and **nothing three-dimensional is ever an unknown**. A plane's attitude is a constant (§6.7), an extent is an expression (§6.9), and neither a face nor a solid owns a parameter, appears in a residual, or is reached by a constraint. The strata run one way and there is no edge back: the sketch solves, the extents are worked out, the terms are ordered, the outputs are read. So everything §15 says about a solver, and every count in §16.3's ledger, is unchanged by the presence of an object — which is the whole of what makes the addition affordable.

### 1.3 Conformance keywords

MUST, MUST NOT, SHOULD, SHOULD NOT, and MAY are used as in RFC 2119. Text marked *non-normative* is explanatory.

---

## 2. Lexical structure

- **Identifiers:** `[A-Za-z_][A-Za-z0-9_]*`. Component names are conventionally capitalized; this is not enforced.
- **Keywords:** `component`, `param`, `point`, `circle`, `line`, `frame`, `path`, `repeat`, `cycle`, `ring`, `about`, `as`, `next`, `prev`, `hint`, `at`, `ground`, `fix`, `ccw`, `cw`, `rev`, `true`, `false`, **[0.2]** `curve`, `over`, `spline` (and `ellipse`, until **[0.15]** made the ellipse a library component — `Ellipse` in `std`, a computed point on a datum traced as a curve, whose contacts are the curve's; an implementation keeps the word only to refuse it). **[0.7]** `unit`, `class` and `style` in, `construction` out; every constraint is a prefix or an infix operator (§9.2), so `on`, `equal`, `tangent`, `curvature`, `symmetry` and `distance` are the words a statement is written with — it is a class now, and the base sheet is what draws it dashed (§13.2). **[0.4]** In a chain (§6.6) the word `close` is meaningful *contextually*; it is not reserved, and an entity may bear it as a name. **[0.8]** `to` is retired: the plain corner is the `->` marker, and threading is stated at the joint rather than inferred from the operands. **[0.5]** A coordinate seed is written `hint at` (§6.4). **[0.7]** Every seed is written in one `hint(…)` clause (§4.3, §6.4); `hint at REF` kept its own form inside a trace block (§6.5.1) until **[0.14]**, when a place became the `at:` and `bearing:` keys of the same clause — `hint(at: REF, bearing: β)` — so `at` after `hint` is refused, and `bearing` is a key and no keyword. **[0.10]** `plane`, `in`, `project` and `fold` in (§6.7); `from` is contextual there as it is in a trace family. **[0.13]** `port` is retired (§7); an implementation keeps the word only to refuse it. **[0.18]** `face` and `solid` are element keywords (§6.8, §6.9) and `view` and `section` open a statement (§6.11); `through` is the body rule's own word and `on` gains a reading over two solids (§9.2), so both join the operator words a name may not be. The six labels a solid's brackets take — `from`, `to`, `depth`, `about`, `sweep`, `sense` — and `offset` (§6.10) and `at` (§6.11) are **contextual**: they are read as labels inside the brackets that take them and are reserved nowhere, so a `param` or a point may still bear any of them as a name (`param face = -(fw + D / 2)` is idiomatic). A declaration's *name*, however, may not be an element keyword, and three shipped examples renamed a line that had been called `face`.
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
| `Plane` | the datum: origin + orientation **[0.6]**, and a **view** — a plane in space, given as a constant orthonormal basis `(u, v)` **[0.10]**; one with no attitude written is the page's **[0.15]** | 0 beyond its two points (§6.7) |
| `Path` | directed piecewise boundary curve | 0 (derived object) |
| `Face` | a closed loop of edges of one plane: a region swept **[0.18]** | 0 — it owns no parameter |
| `Solid` | a face swept, or a term over other solids **[0.18]** | 0 — it owns no parameter |

**[0.18] `Face` and `Solid` are evaluated after the solve, and are the only kinds that are.** Neither owns a coordinate, allocates an unknown, or contributes a residual; neither may be an argument of any constraint of §9.3, be dragged, be dimensioned, or be put on a plane with `in`. What each *is* is settled once the drawing has been solved and every extent worked out (§1.2, §6.9). A kind of this stratum therefore adds nothing to §16.3's ledger.

**[0.2] `Line` was a 2-DOF undirected infinite line with a `.dir` field in 0.1.** It is now a segment between two points, and every constraint that reads a line reads the infinite carrier through those points — which is what `parallel`, `perpendicular`, `on` and `angle` mean by a line anyway.

The change is a concession to cost. A 2-DOF line is a second representation of a line alongside the one every constraint already works in, so it needs its own column layout and its own kernel for roughly fourteen constraint types, and the whole return is that §16.3's ledger comes out differently. Where a drawing wants a line with no ends — a datum, an axis — it says so with a construction segment.

### 3.2 Sub-entities

Compound entities expose sub-entities by field access. Sub-entities are ordinary entities and participate in aliasing and constraints.

| Entity | Field | Type |
|---|---|---|
| `Circle` | `.center` | `Point` |
| `Circle` | `.r` | `Length` |
| `Plane` | `.origin`, `.toward` | `Point` **[0.6]** |
| `Plane` | `.c`, `.s` | `Scalar` **[0.6]** — the unit rotor |
| `Plane` | `.angle` | `Angle` — derived, `atan2(s, c)`; readable in trace-block expressions **[0.6]** |
| `Plane` | its basis `u`, `v` | constants of the declaration, not sub-entities **[0.10]** |
| `Plane` | its offset along `n` | a constant of the declaration, not a sub-entity **[0.18]** (§6.10) |
| `Line` | `.p1`, `.p2` | `Point` **[0.2]** |
| `Face` | its edges | the entities it names, aliased — a face exposes no field of its own **[0.18]** |
| `Solid` | `.near`, `.far` | a prism's two caps, at the higher and the lower ordinate **[0.18]** |
| `Solid` | `.<edge>` | the side swept from the edge the source called `<edge>` **[0.18]** |
| `Solid` | `.start`, `.end` | a partial revolution's two caps **[0.18]** |
| `Solid` | `.volume`, `.area`, `.bounds.x0`… | what a report measures off it (§16.3) **[0.18]** |

**[0.18] A solid's derived names are *paths*, and a path is not a sub-entity.** A face of a solid carries no coordinates, joins no alias class and takes no constraint: the path is what a report writes a number under (`block.side_l.area`) and what a derived picture labels a stroke with. Through a body the operand keeps its own name, so a face of `block` inside `body` is `body.block.near` — a path never renames, which is the naming problem every history-based kernel has and this one does not, because a boolean cannot renumber a name. The *sub-entities* a solid does have are the ones its declaration is written over: the face it is swept from, a revolution's axis line, and the solids of its term. Those are ordinary entities, and deleting one takes the solid with it.

**[0.6] A datum's orientation is a unit rotor, not a stored angle.** `plane f(origin: o,
toward: q)` declares an origin (aliased), a second point it is pointed at (aliased), and two
scalars `(c, s)` of its own, carrying two **intrinsic** constraints the declaration implies —
`c² + s² = 1`, and `(toward − origin) = r·(c, s)` with the chord's length `r` an unknown the
alignment owns — so the rotor is a first-class unknown of the sketch (it drags, grounds and
diagnoses like any other) that adds **no** freedom beyond the two points it is slaved to.  The
representation is the 2D form of the unit quaternion a 3D workplane will want: the eventual
lift changes a component count, not the construct.  `.angle` is *derived* — implementations
MUST NOT store it — and is what a trace-block expression reads to state a bearing relative to
the datum rather than the page: `hint(at: c, bearing: u + f.angle)` (§6.5.1).  The 0.1–0.5 constructor
spelling `frame(center, ray(center, p))` is superseded by the field form; `ray` is dropped.
**[0.15] `frame` is folded into `plane`**: the datum and the view were one construct with the
attitude optional, so there is one kind, `plane`, and a plane with no attitude written is a datum
with the page's — a view of the page, which is what a datum on the sheet is.  A formal `f: plane`
offers `f.angle` as `f: frame` did.  An implementation keeps the word `frame` only to refuse it,
naming the spelling.

### 3.3 Dimensional analysis and units **[0.7]**

Implementations **MUST** check dimensions in expressions and constraints, and MUST NOT silently coerce `Angle` to `Scalar` or vice versa. Angle arithmetic is mod 2π where the operand is a bearing and signed where it is a turn; see §9.4. *(SHOULD in 0.1–0.6, and never implemented; MUST from 0.7.)*

**Two base dimensions**, because two is what the language has: a **length** and an **angle**. A quantity is a rational power of each — rational, because `sqrt` halves one and `sqrt(area)` is a length.

- `*` and `/` **derive**: the exponents add and subtract.
- `+` and `-` **demand agreement**, and so do `min`, `max`, `hypot` and `atan2`'s two arguments.
- `^` takes a plain number, and a **dimensioned base takes a whole power**: `x ^ 2.5` where `x` is a length is not a dimension anybody meant, and `sqrt` is how a half is written.
- The dimension an expression comes to is checked against the **slot** it is written in.

**A bare number is dimensionless, and a *context* may take one.** That is what "drawing units" means: `distance(a, b) == 80` is a length because the slot says so, and `sin(30)` reads 30 as degrees because the function does. A context may **not** speak for a second operand: `90 / N + ivp` is a plain number added to an angle, and an implementation MUST report it rather than choose. The asymmetry is the design — a slot and a function say what they want; two operands are not a context, and neither of them is authoritative.

A **name** is worth a number, and where that number is *used* decides what it is: `w = 80` in a `Length` slot does not make `w` a length, since the same 80 may be a run, a rise or an angle. A unit on the literal (`w = 80mm`) says otherwise, and so does a component formal's declared type (§8) — which is what catches `param x = w + phi`.

**[0.16] One namespace for a number's names.** A number is named three ways — `param w = 60`, a named dimension `distance(w = 60)`, and a bare `w` nothing defines, which is a free variable — and they resolve by one rule (§5): a definition, either kind, declares its name in the body it is written in; a name nothing in scope declares is an unknown of the *instance* the body is elaborated as. A `param` and a named dimension differ in where the number is edited (the source, or the drawing) and in nothing else: a `param` MAY read a named dimension, a second definition of a name in one body is **E001** whichever kinds the two are, and a `param` reading a free variable is an error, since nothing in scope gives the name a number.

| function | |
|---|---|
| `sin`, `cos`, `tan` | `Angle → Scalar` |
| `asin`, `acos`, `atan` | `Scalar → Angle` |
| `atan2` | `(D, D) → Angle`, arguments agreeing |
| `sqrt` | `D → D^½` |
| `abs`, `min`, `max`, `hypot` | `D → D`, arguments agreeing |
| `exp`, `ln`, `log` | `Scalar → Scalar` |
| `floor`, `ceil`, `round` | `Scalar → Scalar` |

`floor`, `ceil` and `round` are `Scalar`-only **deliberately**: rounding a dimensioned quantity depends on which unit you round in, and a language that silently picked one would be wrong half the time.

#### 3.3.1 The literal

```
80mm     3.5in     45deg     0.5rad     12
1' 6 3/16"
```

A number MAY carry a unit. The length units are `mm`, `cm`, `m`, `km`, `in`, `ft`, `thou`; the angle units are `deg`, `rad`, `grad`. `'` and `"` are the foot and inch marks.

**Feet-and-inches is one literal**, and it is a rule the language already had: *a space is what tells the readings apart*, exactly as it does in a mixed fraction, where `3 1/2` is three and a half and `31/2` is a division. So `1' 6"` is one length for the same reason.

**The language therefore has no string literal.** A `"` is the inch mark and there is nothing else for one to be; every `Str` argument is written as the word it is (`at: start`), and a raw branch key is written bare (§13.1).

#### 3.3.2 `unit`

```
unit mm
```

`unit` names the document's **length** unit. A bare number in a `Length` slot is that unit, so every existing document keeps working with one added line, and a suffixed literal converts to it.

**Without a `unit` line the document is in drawing units** — a length dimension with no name. Everything still checks: `distance(a, b) == 45deg` is still an error and `Length + Angle` is still an error. You simply cannot write `mm` or `"`, because there is nothing to convert to, and an implementation MUST report that rather than guess.

**Storing the document's unit costs an implementation nothing for lengths**, because a well-formed kernel is homogeneous in length: scale every length in a sketch by a constant and no residual, no tolerance and no rank moves. **Angles are the exception, and it is not a choice**: `cos θ` is not homogeneous, and there is no consistent unit it works in other than radians. So an angle is stored in radians and converted at the text seam, which is exactly where it converts anyway; what units remove is not the conversion but the *guess*.

Where a document may be copied into another, a paste between documents in different units **SHOULD convert**: a figure is the same figure in either, and two inches is 50.8 millimetres.

#### 3.3.3 `pi` and `tau`

`pi` is the dimensionless mathematical constant. `tau` and `turn` are a full **turn**, which is an `Angle`. They were 3.14159 and 360 side by side with nothing saying why; units settle it, and `tau == 2 * pi * 1rad` now holds dimensionally where it used to be a coincidence of digits.

A conversion written out — `* 180 / pi` — is `* 1rad`, and the `1rad` that remains is not noise: it is the fact that `inv φ = tan φ − φ` **holds only in radians**, which the formula never said and which the check now makes it say.

---

## 4. Program structure

A program is a set of component definitions. One component is designated the **root** (by tool invocation, not by language syntax); the elaborated model is the root's instance tree.

```
component Name(p1: Type1, p2: Type2, ...) {
    <statements>
}
```

### 4.1 Parameters

Parameters are passed by name or position at instantiation. A parameter of entity type (`Point`, `Circle`, `Frame`, `Line`) is **bound by aliasing** (P1): the formal name and the actual argument denote the same entity. A parameter of value type (`Int`, `Scalar`, `Length`, `Angle`, **[0.17]** `Side`) is a compile-time or definitional value; it contributes no unknowns. A `Side` is one of the words `left` and `right` (§9.2) and is not a number: it may be passed on to another instance and written in a selector, and nothing else.

**[0.17] Which way is a word, not a sign.** A distance measured **from a line** — a point to a line, a line to a line — is a **magnitude**: its solution set is *both* sides, and which one a solver finds is the seed's business (P3), as it is in every other sketcher. A negative one is an error (**E040**) wherever the number comes from, including a component's argument, since the kernel cannot tell one side from the other and the minus therefore said nothing a drawing could show. A statement that must pin a side writes the word — `p distance(12, side: left) ax`, left being of the line's own `p1 → p2` — and so does a tangency (`side: left | right`, which was `side: -1`). Where a sign is *arithmetic* rather than a convention — the run and the rise, signed from the first point to the second, and the directed angle of §9.4 — it stays a sign, because a component computes it from coordinates it is given; each gains the word that says the same thing in the open (`along: right | left | up | down`, `sense: cw | ccw`), and a document SHOULD prefer it. A component takes a side as a value of the type **`Side`** (`s: Side`, `side: s`), which is a word and not a number: encoded as ±1 it would put the unreadable idiom back one level down, inside every helper.

**[0.17] Labels are mandatory past the entities.** An argument bound by position MUST bind a parameter of entity type, and MUST NOT be written after a labelled argument; both are **E004**, reported at the argument. So an instantiation is the entities it is written over, in order, and then every number by the name of the formal it fills — `Cylinder(swing, side, top, piv, rod, across, dir: dir, fw: fw, o_s: o_s, o_t: o_t)`. Position is a count, and a count is the one thing a reader of a long formal list cannot check: an argument written one place off binds to the formal beside the one it was meant for, which is a *different* mistake from the one it is then reported as — a `Length` complaining it is not an `Angle`, a hexagon's `phase` arriving as its side count, a plane arriving where a number was wanted. The rule costs the entities nothing, because their order is the one an assembly reads by, and it costs a number one word, which is the word that says which.

### 4.2 Statement classes

Every statement belongs to exactly one class. The classification is normative because P3 depends on it.

| Class | Statements | Affects solution set? |
|---|---|---|
| **Declaration** | entity declarations, `param`, `curve` family definitions, instance declarations, **[0.18]** the body rule (`on` and `through` over two solids, §6.9) | introduces entities/aliases |
| **Constraint** | predicate calls, `==` equations, **pinned seeds (`==`, §4.3) [0.2]**, orientation predicates, arc-branch and tangency-side decorations, `ring` symmetry, gauge statements | **yes** |
| **Seed** *(was Hint)* | the `hint` statement (§11), **and every seed written inline in a `hint(…)` clause (§4.3, §6.4) [0.2] [0.7]** | **no (P3)** |
| **Structure** | `repeat`/`cycle` blocks, `path` declarations (net of their derived constraints, §10.4), **[0.18]** `view` and `section` (§6.11), which ask for a picture and declare nothing | organizational |

### 4.3 Seeds and pins: `hint(…)` and `==` **[0.2]** **[0.7]**

> **A number inside a `hint(…)` clause is a seed. Every other number is not.**

This is the only distinction between the two classes, and it is lexical on purpose. §11 requires an implementation to verify Invariant H *syntactically*; a mark you can see does that, and no analysis of what a number "is really doing" does.

```
point p hint(x: 0, y: 0)             // seed: a solve may move it
circle c(center: o) hint(r: 25)      // seed: a solve may move the radius
distance(a, b) == 80                 // constraint: a solve must not move it
point_on_spline(p, s) hint(t: 0.37)  // seed: the contact may slide along the curve
point_on_spline(p, s, t == 0.37)     // constraint: the contact is pinned there
param w = 100                        // neither: a number worked out while elaborating
```

**[0.7] One clause, where 0.2–0.6 had three spellings.** A seed used to be written three ways depending only on what happened to carry it — `hint at (0, 0)` for a point's coordinates, a labelled `r: 25` inside the constructor for any other scalar, `t = 0.37` for a constraint's own unknown. Three spellings for one class of thing, and the middle one was the worst: it put a number the solver will move inside the same brackets as the structure it may not, so `circle c(center: o, r: 25)` read as though the radius were as much a part of what the circle *is* as its centre.

`hint(key: value, …)` says it once, keys in any order, on declarations and on constraints alike. **The brackets after the name are what the thing is made of; the `hint(…)` after them is where the solve begins.** It also mends the headline of this section, which was only ever approximately true: `r: 25` was a seed written with `:`, and `param w = 100` is not a seed at all.

The three retired spellings — `point p at (0, 0)`, `point p hint at (0, 0)`, `circle c(center: o, r: 25)` — MUST NOT parse. A document written against 0.2–0.6 does not load; the change is small, mechanical and worth doing once.

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
- Redeclaration of a name within one body is an error (**E001**). A `param` and a named dimension (§9.1) declare a name alike, so `param w` beside `distance(w = 60)` is E001.
- Instance members are accessed by dotted paths: `t.lead`, `g.hub.origin` — a dimension named inside an instance included: `t.w`.
- **[0.16]** A name in an expression that nothing in scope declares — no formal, no `param`, no named dimension of the body, the file or a `use`d module — is a **free variable of the instance** the body is elaborated as (`t1.w`, `t2.w`), the rule §6.5 applies to a formal left unbound. On the sheet the instance is the document, and the unknown is the document's. A component therefore cannot read the document it is drawn in by name, and a module's component cannot read the caller's at all. Inside a `cycle` or `repeat`, a name the block declares is each copy's own (a dimension named in a block is defined once per copy) and a name it does not declare is the enclosing body's, shared by every copy.
- Inside `repeat`/`cycle`/`ring` blocks, the index binder (`as i`) and the pseudo-instances `next` / `prev` are in scope (§12).
- There is no shadowing: a block binder that collides with an outer name is an error (**E002**).
- **[0.17] A name that shadows a built-in is said.** The constants and functions of §3.3 (`pi`, `tau`, `turn`, `sin`, `min`, …) are known to every expression before the document is, so a `param`, a formal or a block binder of one of those names does *not* shadow it: a text carrying the name is substituted and reads the declaration, a number worked out reads the built-in, and the two answers differ silently — `param tau = 35deg` passed to a `tau: Angle` formal arrived as a full turn. A named dimension of a built-in name is an error where it is parsed; the other three declarations are a warning at the declaration (**W112**), because the drawing is not wrong, the name is. An implementation MUST say it once per declaration, whether or not the component is ever instantiated.

---

## 6. Entity declarations

### 6.1 Free declarations

```
point p, q
line  l                        // makes two points: l.p1, l.p2
circle c                       // makes one:        c.center
arc   a                        // makes three:      a.center, a.start, a.end
```

A bare declaration introduces an entity all of whose coordinates are unknowns.

**[0.7] The children come with it.** A declaration that writes no argument list gets the ones its kind is built from, *unnamed*, and they are reached by dot access: `l.p1` is an ordinary point, and it constrains, drags and is picked like any other. Half the points in a drawing are named only because a line has to be written from something; a name earns its place when something says it twice.

**The dotted path is the name.** An anonymous child has no name in the source, so `l.p1` *is* its name, and everything that identifies an entity by name MUST agree — a constraint written against it, a selection that outlives a re-elaboration, a diagnostic that reports it.

**[0.9] The name itself is optional**, independently of everything after it: `line` alone is a syntactically valid statement — a line with no name, implicit children and no hint — and so are `line(p1, p2)`, `circle hint(r: 25)` and `arc(center: c)` — a line owns no scalar of its own, so its anonymous seeded form puts the seeds in the slots, `line(hint(x: 0, y: 0), hint(x: 60, y: 20))` (§6.2). The token after the kind keyword decides what the statement says next, so a trailing-clause word (`hint`, `knots`, `class`, `at`, `close`) can no longer be a declaration's name — the same reservation element keywords and operator words carry. `curve` keeps requiring a name: its form is `curve name = family(…)`, and the name is what the contact constraints address. **Identity is minted on demand.** Internally the statement suffices — an anonymous element parses, elaborates, draws, drags and deletes without ever being named. The moment the *source* must reference it (a constraint applied from a tool, a dimension stated on it), the implementation MUST splice a real name into the declaration — the same bargain §6.4's writeback strikes with an unwritten `hint(…)` clause. No hidden names: a name is what the source calls the thing, and an unnamed thing has none until the source needs one. Chains are in scope (§6.6): `line -> tangent arc -> tangent line` is a fully anonymous open contour, the corner-minting rule naming shared points by the parent's dotted path.

Where an unnamed and unseeded coordinate *starts* is not the language's business (§15). It is worth saying only that the obvious answer is wrong: two implicit endpoints both at the origin is a zero-length line, with no direction for `horizontal(l)` to bite on and a singular row for any tangency, so an implementation MUST NOT place them coincidently.

A **list** child slot — a spline's control polygon, a curve's arguments — has no arity to conjure children from, so a bare `spline s` remains **E103**.

### 6.2 Constructor declarations

```
circle pitch(center)
frame  f0(origin: center, toward: t.lead)   -- [0.6]
line   l(hint(x: 0, y: 0), hint(x: 60, y: 20))   -- [0.7]
```

Constructor arguments have two behaviors, by type:

- An argument of **entity type** *aliases* the corresponding sub-entity (P1). `circle pitch(center)` makes `pitch.center` and `center` one entity.
- **[0.7]** A child slot may hold a **`hint(…)`** instead of a reference: an anonymous point, and where its solve begins. It is the same clause as §4.3's, standing in for a child rather than qualifying the declaration it follows, so one construct means "where this begins" wherever it appears.

**[0.8] Every slot is a name, a seed, or implicit.** A written slot carries a reference or a `hint(…)`; a slot the list leaves out — by a label naming a later one, or by a chain's marker filling only the ends it speaks for (§6.6) — is an **implicit child**, minted exactly as a declaration that writes no list at all mints them, and reached by its dotted path. There is no bare `hint` meaning "anonymous and unseeded", because an empty slot already says it. (Through 0.7 a partial list was **E103** — "all the children, or none" — a rule the `->` marker retired: `line l1 -> line l2` fills one slot of `l2` and leaves the rest the drawing's own. E103 remains the refusal for *more* children than the kind has slots.)

**[0.2] The value-type rule is reversed from 0.1**, which made a value argument definitional — `circle pitch(center, R)` introducing `pitch.r == R` as a substitution. That cannot stand beside §4.3, and it is wrong on its own terms: a radius is a coordinate a user drags. Under 0.1's reading, taking hold of a rim and pulling would be an *edit of what the program means* rather than a move within its solution set, and every direct-manipulation gesture on a scalar would be a different kind of event from the same gesture on a point. **[0.7]** A value argument is no longer written there at all; it is §4.3's `hint(…)`.

The `name = expr` form declares an entity wholly defined by an expression; all its coordinates are definitional.

A radius that is meant to be *held* says so:

```
circle c(center: o) hint(r: 25)   // 25 is where it starts
radius(c) == 25                   // 25 is what it is
fix(c.r)                          // 25 is what it is, without a dimension on the drawing
```

### 6.3 `param`

```
param R = m * N / 2
```

Introduces a named definitional value. `param` values are evaluated at elaboration time when all inputs are `Int`/literal, otherwise they are definitional scalars.

**[0.12] A file's top-level `param`s are in scope in every component the file defines**, and in the file's root body — the numbers a drawing is drawn from, a bore and a stroke, stated once at the top rather than threaded through every formal list. A formal of the same name shadows one, since a component's interface must not break when a `param` of that name is added above it. **[0.16]** The file's top-level *named dimensions* are in scope the same way, and a body's own definition shadows the file's; a module's named dimensions are numbers to the components that read them, its drawing not being drawn. The `param`s of a module the file `use`s (§14.4) are in the file's scope too, under its own. A `param` MUST NOT read geometry (`a.x`): a `param` feeds constraints, and a number read off a seed would make the solution set depend on where a solve began (P3); a *seed* may (§6.4).

### 6.4 Seeds written inline **[0.2]**

A declaration MAY carry the starting values of its own scalars, in a trailing `hint(…)` clause:

```
point   p  hint(x: 0, y: 0)
point   p  hint(y: 12)                     // an omitted scalar is 0
point   t                                  // no clause at all
circle  c(center: o) hint(r: 25)
```

These are seed-class (§4.2, §4.3) and semantically inert (P3). They are the primitive form; §11's `hint` statement remains, for the case it is actually good at.

**[0.7] One clause for every seed.** `hint(…)` joins the trailing-clause loop, so it is order-free against `knots` and `class` (§13.2) exactly as those two already are against each other. A constraint's own unknown is written the same way — `point_on_spline(p, s) hint(t: 0.4)` — and the *pin* stays in the argument list, `point_on_spline(p, s, t == 0.4)`: `hint` marks what a solve revises, and a pin is precisely what it does not.

Two things that look like seeds and are not, and stay where they are. **`knots [...]`** is document data no solve moves. **A curve instance's values** — `curve e = involute(base, phase: 0)` — are numbers the family takes, not numbers the solve revises.

**What `hint` marks is that a solve revises the number — not that the number is seed-class.** The two are not the same set, and the difference decides where the word belongs. Seed-class is §4.3's classification: inert under P3, so deleting it changes no solution set. That is true of a coordinate seed *and* of a callout placement (§13.1), which is why a placement keeps its bare `at (t, r)` even though it is every bit as inert. What separates them is who writes them. A coordinate seed is an input a solve overwrites, every time, which is the whole of what §6.4's writeback does. A placement is never touched by a solve at all: it is derived by the layout until somebody drags the callout, and from then on it records where that person put it. So a placement is not a guess about anything, and `at` there says what it means.

*Non-normative:* an implementation can therefore read `hint` as the mark of "this is the solver's to answer", which is a narrower and more useful claim than "this is inert". A reader wanting to know what may be deleted without changing the drawing should ask §4.3, which answers for both.

**Why inline is the primitive.** A seed's job is to say where a coordinate starts, and the place a reader looks for that is the declaration of the thing that has the coordinate. It matters more than taste once a drawing is edited by drawing on it: a solve that wants to record where a point ended up rewrites six characters of a declaration that already exists, where under 0.1 it would have to locate that point's `hint` statement among the body's statements, or synthesise one and decide where to put it. The first is a splice; the second is a program transformation, and it is performed on every drag.

`hint` keeps the cases inline cannot express — seeding an entity declared elsewhere, and seeding from an expression over other geometry (`hint t.lead(x: center.x + root.r, y: center.y)`).

**[0.12] A seed may read geometry, and reads its seed.** The text in a `hint(…)` clause — a declaration's or a child slot's (`line l(hint(x: p.x + 10, y: p.y), …)`) — MAY name another entity's scalar by its dotted path: `p.x`, `p.y`, `k.center.x`, `k.r`, `e.b`. What it reads is that scalar's **own seed**, never a solved value, so the clause stays seed-class: delete it and the solution set is unchanged, as §4.3 requires. The same clause names a place outright **[0.14]**: `hint(at: REF)` — where another point starts — and `hint(at: K, bearing: β)`, the point on circle `K`'s edge at bearing `β` from the page's x-axis. `at` and `bearing` are keys beside `x`, `y`, `r` and `t`, and the rule of §4.3 is then lexical with no exception: a seed is what is inside `hint(…)`. A clause naming a place carries no coordinate (`hint(at: p, x: 3)` is an error at the key), `bearing` without `at` is an error, and both are refused where they are written, as an unknown key is. (In 0.7–0.13 the place had a grammar of its own, `hint at REF [bearing (β)]`, which MUST NOT parse now; an implementation SHOULD say what the spelling became.) Both forms were a trace block's words (§6.5) and mean the same thing on the sheet. Seeds are settled once every declaration has one, in statement order, so a seed reading a seed that was itself read from a third comes out right when the three are written in the order they depend on; written the other way round it reads the earlier one's provisional seed (an unseeded point's scatter, an unwritten radius's default), which an implementation MAY warn about and MUST NOT refuse. A read of a scalar the entity has not (`p.z`), or of nothing (`nobody.x`), is **E103**. A geometry read is a `Length` where the document names a `unit` (so it adds to `150mm` and not to `10`) and a bare number where it does not, since there no literal can be a length. Inside a component the names resolve as references do — a formal reads as the entity it aliases, a name inside a block's copy as that copy's. A seed written this way is an expression and is never written back by a solve.

### 6.5 Curves **[0.11]**

A curve is **a point of a component, as one of the component's numeric formals runs over an interval**:

```
curve NAME = REF over FORMAL in ( A, B )                    an instance's point
curve NAME = Component(ARGS).REF over FORMAL in ( A, B )    an instance written in place
```

`REF` names a point the component places — a declaration of its body or a nested instance's; a formal the component is written over does not move with the swept formal and is refused (**E103**). `FORMAL` is a numeric formal of that component, `Angle` or `Length` (**E040** otherwise); the interval's ends are expressions over the parameters in scope. A curve declares an entity and takes contacts like any other curve, each owning the curve's parameter — spelled `t`, as a spline's is, whatever the swept formal is called **[0.15]** (0.2–0.14 spelled it `u`, which an implementation SHOULD name when refusing the old key): `p on e hint(t: …)` says `p − C(t) = 0`, two residuals and one new unknown; `e tangent l` holds the line through `C(u)` along `C'(u)`, two residuals against the one unknown; `e curvature k` makes the circle the curve's osculating circle at `u`, three residuals against it. A tangency needs `C'` and its derivatives in the geometry — second order — and a curvature `C''` and `C'''`: a computed point supplies them exactly from its expressions, and a locus supplies `C'` exactly (the implicit function theorem) with its derivatives by difference, and no higher order at all, so a curvature stated against a traced curve is an error (**E103**). There is no separate curve family: 0.2's `curve NAME(FORMALS)(PARAM) = …` and 0.3's `trace POINT where { … }` are retired, and an implementation MUST refuse them with a message naming this form.

**Two ways a component places the point.** A **computed** point, `point p = ( XEXPR, YEXPR )` **[0.13]** (`port p = …` in 0.12), gives the coordinates as expressions over the formals and the params; a component with one is drawn only as a curve, and an instance of it on the sheet is an error (**E103**), since nothing on the sheet holds a point to a formula. Any **other** point is placed by the body's statements — the locus form: `C(u)` is where the constraints put the point, given the formal's value and the geometry the component is written over. Traced, the body MUST determine its own coordinates — as many equations as coordinates of its own — or the curve is an error (**E103**): an under- or over-constrained locus is a curve that does not exist, and it must not elaborate quietly. Drawn, the same component may be closed from outside like any other.

```
component Unwind(c: circle, datum: line, phase: Angle, u: Angle) {
  point t
  point p
  line rad(c.center, t)
  line s(t, p)
  t on c                                       // the string leaves the circle...
  datum angle(u + phase) rad                   // ...at bearing u — directed, so this side
  rad perpendicular s                          // perpendicular to the radius there,
  p distance(-(c.r * u / 1rad)) rad            // and taut: let out == arc unwound
}
curve e = Unwind(base, datum, phase: a0).p over u in (u0, u1)
```

**Over a drawn instance.** `curve path = leg.toe over theta in (0, 360)` names a point of an instance the drawing holds. The trace is **anchored at the drawing**: the pose the instance stands in on the sheet is where evaluation begins, and the value the instance gave the swept formal is the anchor's parameter. An instance that leaves a numeric formal **unbound** makes it an unknown of the drawing — the rule that a name nothing defines is a free variable (§3.4), applied to a formal, named under the instance (`leg.theta`) so two instances leaving the same formal unbound have two unknowns — and the anchor then follows that unknown wherever the solve puts it. This is the form a mechanism is written in: drawn once with its crank free, and traced from the same statements.

**Over an instance written in place.** `Involute(base, phase: a0).p` binds the arguments as an instance statement would and draws nothing: the curve is the only thing made of it. The anchor is the value it gives the swept formal, or the interval's start when it gives none.

**Requirements.** An implementation MUST differentiate `C` with respect to the swept formal *and* with respect to every coordinate the component reads. `∂C/∂u` is which way a contact may slide; `∂C/∂θ` is how the curve moves when the geometry it is written over moves, and an implementation that computes only the first will solve a contact once and drop it the moment that geometry is dragged. For a computed point both are mechanical from the expressions; for a locus both come from the implicit function theorem at the body's solution. A name a computed point's expressions cannot reach is an error (**E016**), not a free variable: a curve is written over geometry that exists, and a misspelling there would quietly add a degree of freedom to every point on the curve.

**Branches.** A locus generically has several solutions, and a component states its way onto one — three instruments, in order of strength:

1. **A signed constraint,** wherever the vocabulary can say it. Above, *neither* choice is a branch at all: `angle` is directed (§9.4), so `t` sits at the bearing and not opposite it, and `point_line_distance` is signed, so one equation unwinds the string one way for positive roll and the other for negative.
2. **An orientation predicate.** `ccw(a, b, x)` / `cw(a, b, x)` in the body is §9.6's statement doing §9.6's job: it contributes no residual and *selects among the discrete solution components*. Traced, its third point MUST be one the component places. A predicate is read **at the anchor** — the drawn pose, or the value the instance gave the swept formal, chosen where the predicates read unambiguously — and an implementation MUST enforce it there (reflect the placed point across the oriented line and solve again) and MUST NOT re-enforce it elsewhere: away from the anchor, continuity governs, and the component the predicate picks at the anchor is the component the whole curve is on, even where the curve has since wound to where the predicate no longer reads true. A body with predicates needs no seeds at all: an implementation MUST fall back to deterministic restarts, scaled by the geometry the component is written over and by nothing else, when the seeds (or their absence) leave the anchor solve nowhere to start. Drawn, the same predicate records the root choice the drawing is on (§9.6).
3. **A seed.** What neither an equation nor a predicate says, a seed says: the body's seeds are places over the formals, evaluation of an instance written in place starts from them, and away from the anchor continuity governs — an implementation MUST evaluate the curve as one continuation along the parameter, so the branch picked at the anchor is the branch everywhere. A curve over a drawn instance starts from the pose on the sheet and reads no seed.

**Places, not coordinates.** Inside a component that is only ever traced, a seed may be a *place*: `point t hint(at: c, bearing: u + phase)` is the point at the edge of circle `c` at that bearing from the page's x-axis, and `point p hint(at: t)` is wherever `t` starts (a point already named must be declared first). Both lower to exactly what the coordinate spelling would, so `hint(x: xexpr, y: yexpr)` remains available and means the same thing. On the sheet a seed is a number a solve writes back, which a place named by reference is not, so a drawn instance of a component with a geometric seed is an error (**E103**).

**Bearings may be measured from a frame.** A bare bearing is page-fixed, and a body posed against a datum (`datum angle(u) swing`) with page-fixed seeds goes quietly stale the moment the datum tilts (bmander/geomsolver#10). A component written over a `frame` reads its derived `.angle` in any expression, so `hint(at: c, bearing: u + f.angle)` and `hint(x: o.x + d * cos(u + f.angle), …)` state the same bearing *from the frame*. A name of the form `x.angle` where the variable table holds the rotor `x.c` / `x.s` compiles to its `atan2`; any other unknown name remains the misspelling error it was.

**Why a point of a component and not a family of its own.** 0.2 gave a curve a construct of its own — a family with formals, a parameter and a body — and 0.3 a second body form, and every mechanism was then written twice: once as the drawing and once more inside the family, with the formals passed to themselves (bmander/geomsolver#46). A component already has formals, a body, params and instances, and a curve is one question asked of it. The gear in §18 is the case that makes the difference plain: its flanks are involutes because the document says what an involute is, and nothing in the solver knows the word; the walking leg of `jansen.sv` is drawn once, and its stride is the same leg asked where its toe goes.

### 6.6 Chains **[0.4]**

Sugar, and only sugar: a chain writes a run of declarations and the constraints *between* them in one ordered breath, and it elaborates to exactly the statements a person would otherwise write out. It is a parser construct — nothing downstream of the parser knows it exists.

```
horizontal line bottom(b1, b2) -> tangent
arc a_br(center: c_br) hint(r: r) -> tangent
vertical line right(r1, r2) -> tangent close
```

```
CHAIN  ::= LINK (JOINT LINK)* ["->" INFIX* "close"]
LINK   ::= PREFIX* DECL | REF
PREFIX ::= a constraint name whose spec is one entity slot     // horizontal, vertical
JOINT  ::= "->" INFIX* ["->"] | INFIX+ ["->"]                  // at least one marker or word
INFIX  ::= "tangent" | "equal" | a constraint name whose spec is two entity slots
```

**[0.8] Threading is a statement, not an inference.** The `->` marker on a joint says the two links beside it share a boundary point, threaded left-to-right along the traversal below; its absence says they do not. `->` alone is the plain corner — this is what `to` said through 0.7, and `to` is retired into the marker. `-> INFIX` is a corner that also states the relation, at the point just threaded. `INFIX` alone is the relation and no corner: `a_br equal a_tr equal a_tl` says three arcs are the same size and nothing whatever about where they meet, and welding them would be an invention. Because each joint states its own threading, a chain may mix declarations and names freely — `line l1(a, b) perpendicular line l2(c, d)` declares two separate lines at a right angle, and `line l1(a, k.start) -> tangent k` extends a fresh contour onto geometry that is already there.

- A **prefix** desugars to that constraint applied to the declaration it stands before: `horizontal line bottom(b1, b2)` is `line bottom(b1, b2)` plus `horizontal(bottom)`. Eligibility is registry-derived — one entity slot and nothing else — so a new unary constraint joins the grammar without the grammar changing.
- **[0.8] A joint may state several relations.** `A -> equal angle(30deg) B` states both between the two links, at the corner the marker threads; each word desugars to a statement of its own, with its own identity and span. The marker may stand on either side of the words, or both — `A -> equal -> B` is the one joint `A -> equal B` is — and words with no marker state their relations and weld nothing. The words need no punctuation because fixity sorts them: a word is read at the joint until one opens the next link — an element keyword, or a prefix word standing before one — so in `-> tangent horizontal line b` the tangency is the corner's and the levelling is the link's. Deleting one relation splices its word out and leaves the rest standing; the whole joint deleted at once falls back to what a single word's deletion would be — the corner stays, or the statement breaks. A trailing placement (`at (t, r)`, §13.1) qualifies the line's **one** relation; a line stating several relations refuses it, there being no way to say which.
- A **joint** stands between two links and says how they meet. Constraints return nothing, so a chain reads like a chained comparison, not an expression: each joint binds its two neighbours, and there is no precedence anywhere. `a equal b equal c` is therefore two statements, not three, and n operands give n−1 — the same rank as any other spanning set over the same elements, stated as a path rather than a star. An INFIX word is the two-argument counterpart of PREFIX, derived from the same registry: it desugars to that constraint over the pair, positionally, and MUST fit both — a word whose slots the pair cannot fill is an error, not a guess.
- **[0.5]** `equal` is **polymorphic**: `equal_length` between lines, `equal_radius` between circles or arcs, and an error between one of each, since no constraint equates a length to a radius. Like `tangent` it is drafting vocabulary rather than a constraint name, so no registry lookup can resolve it — the pair it stands between does. Where a chain declares its elements the keywords settle it as the program is read; where a chain names them it cannot be settled until the names are resolved, since a name may be declared further down the body (P2) or come from a component, so the word travels to elaboration and is settled there. Both report the same error.
- **Threading.** **[0.8]** A link a marker reaches is a line or an arc — an element with an entry and an exit, read left to right (`p1 → p2`; CCW, `start → end`); a kind with no boundary points — a circle — cannot be reached by a marker, though it may stand in a chain no marker touches. At each `->` the shared point may be named by one side, by both in agreement (two different names are an error), or — between two declarations — by nobody: the chain then mints it, the earlier-built side's boundary being an anonymous child with a name (its dotted path, §6.1), which fills the later side's slot. `line l1 -> line l2` is therefore two lines and three points, one shared. A link that only *names* an element offers no list to read or fill and no kind to read a boundary field off — so at a corner with one, the declared side MUST name the shared point, usually by the existing element's own child (`k.start`). An end no marker reaches is an implicit child like any other unwritten slot (§6.1).
- `-> close` after the last link seals the loop: the last exit threads to the first entry, and a word beside the marker says how they meet there. A loop is a thread, so a `close` without the marker is an error.
- A statement otherwise ends at its line's end (§2); a line ending in a joint — the marker or a word — continues its chain onto the next.

**Every threaded joint is the regular form.** A threaded joint knows the shared point, so `-> tangent` between a line and an arc is `tangent_arc_line(arc, line, at: start|end)` — tangent *at* the point just threaded — and never the bare tangency over a coincidence, whose Jacobian is rank-deficient at every solution; `-> tangent` between two lines is collinearity (`parallel` over the shared point), and a pair the vocabulary has no regular at-form for (two arcs meeting at a corner) is an error, never a silently degenerate statement. An **unthreaded** `tangent` is the plain pair (`tangent_line_circle`, `tangent_circle_circle`), which is the correct and well-conditioned statement exactly when the two are separate — the `at:` argument is only ever supplied by a threaded joint. A bare `->` states nothing beyond the corner: the shared point is the whole of it.

**[0.8] A block body may end mid-joint.** Inside a `repeat`, `cycle` or `ring` body (§12), the body's final chain may end in a *threaded* joint — the marker, or the marker with words — standing at the body's `}`: the trailing joint threads the chain onto the **next copy's** first link, and is stated between copy i's last link and copy i+1's first exactly as an in-chain joint would be — the weld a shared point, every worded tangency the regular at-form. Which pairs are stated is the block's kind: `cycle` and `ring` wrap, so every copy states it and the trailing joint is the loop's closure — `cycle N { distance(d) line -> angle(a) }` is the dimensioned N-gon, with no `close`, no names and no written points — while `repeat` does not wrap, so the *last* copy's trailing joint is simply not stated and `repeat N { line -> angle(a) }` is an open polyline of N sides and N−1 corners, the natural reading rather than an error. Both boundary links MUST be declarations of kinds with ends (a name-link's boundary is §6.6's ordinary rule: the point must be named where it stands), and at most one of the two boundary slots may name its point — both named are two *different* points across the copy seam, and that coincidence is stated longhand. Where the construct can mean nothing — a `component` body, a trace block, the top level — it is an error, and an unthreaded trailing word still wants its right operand. The joint is one written joint however many copies state it: each stated copy keeps the one statement identity, told apart by the instance path (§12.7). A statement inside a braced body also ends at the body's `}` as at a line break, so the whole of a block may sit on one line.

Each desugared statement keeps an identity of its own and a span into the chain's text, so a caret, a diagnosis culprit and a splice land on the word that stated the thing. (§12.7 is many instances from one statement; a chain is several statements from one *line*, each still its own.) Deleting a chain-borne constraint is therefore a splice — a threaded joint steps down to the bare corner `->`, a prefix word goes where it stands, and **[0.8]** an unthreaded joint becomes a statement break, its span grown over a terminal name-link that a break would leave dangling (inside a chain that closes there is no safe break, and the deletion is refused). Deleting a link is refused: no splice takes one link out and leaves a chain behind, so that edit belongs to the source.

*Non-normative:* chains and paths (§10) answer different questions. A path is a traversal of geometry that already exists — vertices, the circles its arc segments lie on, orientation and branch rules, for boundary composition and export. A chain *declares* the geometry: it is how a contour's elements, their meetings and the levels on its straight runs are written down in the first place. The case library's fillet rectangle is the canonical chain; its longhand form states the same sketch in thirty statements.

### 6.7 Planes and projection **[0.10]**

A multiview drawing is several pictures of one object on one sheet, each on a stated plane in space, related by projection. The language states it the draughtsman's way — descriptive geometry — and **nothing three-dimensional is ever solved for**: a document stays planar, and what is added is a datum with an attitude, a membership, and one equation.

**A `plane` is the datum with an attitude.** `plane front(origin: o, toward: q)` declares an origin, a point it is turned toward and a unit rotor (§3.2 [0.6]) — that is where the view sits on the page and which way it is turned, and all of it is solved for as a frame's is. What it adds is a **basis** `(u, v)`, an orthonormal pair in space with `n = u × v` toward the viewer, which is a *constant* of the declaration: document data like a spline's knots, never a seed and never moved by a solve — which is why it stands in the brackets with the children (§6.2) and not in `hint(…)` (§4.3). It is written one of three ways:

```
plane front(origin: o, toward: q)                                   // the page itself
plane top(origin: o2, toward: q2, from: front, fold: 0deg)          // folded from another
plane p(origin: o4, toward: q4, u: (0.6, 0.8, 0), v: (0, 0, 1))     // given outright
```

- With neither `from` nor a basis, the plane is **the page**: `u = (1, 0, 0)`, `v = (0, 0, 1)`, so `n = (0, −1, 0)` — the front view, the viewer standing at −y.
- `from: P, fold: θ` **folds** the plane from `P` about the line at bearing `θ` in `P`: the new plane is perpendicular to `P` and contains that line, which is its `u` — `u = cos θ·u_P + sin θ·v_P` — and its `v = −n_P` points *away* from `P`'s viewer, so distance from the fold line in the new view is depth behind `P` (third-angle projection). `fold` is an `Angle`; **[0.18]** an omitted one was 0 through 0.17 and that default is **withdrawn** — a `from:` with no clause beside it is now the plane *moved* rather than the plane turned (§6.10), and a document that means the fold writes `fold: 0deg`. From the page, `fold: 0deg` is the top view (`u = x`, `v = y`) and `fold: -90deg` the right view (`u = −z`, `v = y`), drawn with its frame turned −90° so z is up on the page. Two folds reach any plane; `P` may be declared after the plane that folds from it (P2), and a plane folded from itself, however indirectly, is **E041**. `from` names a plane and nothing else (**E040**, **E101**).
- `u:`, `v:` give the basis outright as two triples of dimensionless expressions. They are normalised, `v` is orthogonalised against `u`, and a pair spanning no plane is **E103**. Neither half alone, and not both a `from` and a basis, is a syntax error.

An implementation MUST NOT make the attitude an unknown; a document that wants a view to follow the geometry writes the fold as an expression over its parameters.

**A point says which plane it is on with `in`.** `point a in top` is a trailer of the declaration, order-free against `hint`, `knots` and `class`, and it applies to **every point the declaration mints or names**: `line l(a, b) in top` puts `a` and `b` on `top`, `circle c in right` its centre, `arc` and `spline` likewise. A membership moves nothing — it is a label, read only by `project` — and a point with none is simply on the page. A point put on two different planes by two declarations is **E060**; agreement is not an error. `frame`, `plane` and `curve` have no points of their own to put anywhere, and `in` on them is a syntax error. Inside a `ring` (§12.5) a plane is invariant: a membership or a fold referencing one is true of every copy alike.

**`a project b` says two points are images of one point in space.** It is an infix operator over two points (§9.2), each `in` a plane; the two planes are **inferred** from the memberships and are never written — an implementation MUST refuse (**E061**) a point on no plane, two points on one plane (a view relates nothing to itself), and two planes that are parallel (they share no fold line), each at the statement. With `d = (n_A × n_B)/|n_A × n_B|` the fold line the planes share, `d_A = (u_A·d, v_A·d)` its direction in A's own 2D coordinates and `d_B` likewise, the residual is

`d_A · Rᵀ(c_A, s_A)(p − o_A) − d_B · Rᵀ(c_B, s_B)(q − o_B) = 0`, `Rᵀ(c, s)(x, y) = (c·x + s·y, −s·x + c·y)`

— one equation: two images of one point agree on their coordinate along the fold line their views share, and on nothing else. Each view's origin is taken as the image of one shared origin in space, so the origins of a drawing's views are all images of one point and need no projection between them; and a view slides freely along its projectors (perpendicular to the fold on the page) without moving the row, which is the free spacing between the views of a drawing. `project` is claimable (§9.7): `claim a project b` asks whether two views are consistent. It is not commutative.

**The block form writes the clause once.** `in top { … }` marks every declaration in its body `in top`; **[0.12]** it stands at the top level of a document and inside a *component* body — over a plane the component was handed, which is how a part carries its geometry for each view in one place, its own `project` statements tying them — and not inside a root block, where a header buried in another statement's span would be a splice no deletion could compose; a `repeat`, `cycle` or `ring` inside it marks the declarations of every copy, so a contour drawn as a chain round a cycle is drawn in the view. The statements are ordinary statements of the enclosing body, and an implementation MUST treat them exactly as if each had written the clause itself — they splice, diagnose and delete as themselves, only the header and the closing brace are the block's, and deleting the plane removes exactly those, leaving the statements as page geometry. The block stands at the top level of a document (inside a body, the clause says it one declaration at a time); a declaration inside that writes its own `in`, and a kind with no points of its own, are refused where they stand.

**An instance joins a view whole.** `t: Tooth(…) in top` puts every point-bearing declaration the instance's expansion makes — through nested components and blocks — on the plane: the block's rule, over the statements one statement stands for. A datum or a curve inside is left alone, having no points of its own to put there. A point aliased in through an argument joins through any body declaration that names it, and one already on another plane is E060; an expansion given two planes — a clause of its own under an enclosing `in` — is refused. An instance inside an `in { … }` block takes the block's plane the same way.

*Non-normative:* the front, top and right views of a part are then three planes — the page, `fold: 0deg`, `fold: -90deg` — with the part's corners drawn `in` each and tied across them by `project`; an auxiliary view folded at the bearing of an inclined face shows that face true-size, and its corners can be placed by projection alone. `rust/examples/bracket.sv` is the worked case.

### 6.8 Faces **[0.18]**

A **face** is a region of a plane: a closed loop of edges the document has already drawn.

```
face sec(mouth, side_r, lid, side_l)     // four lines, walked in order
face hole_f(hole)                        // one circle, which is a loop by itself
face bore_f(b_mouth, bore_r, b_head, b_axis)
```

A face is a **Declaration** (§4.2). It mints nothing and owns nothing — no coordinate, no unknown, no equation, no freedom — and its edges keep their own names: naming them is aliasing (P1), not constraint, so deleting an edge deletes the face. Its list is a `List` slot like a spline's control polygon, so there is no arity to conjure children from and a bare `face f` is refused (**E080**, the face's own code saying what a face is, where a bare `spline s` is §6.1's E103).

**A face is one closed loop.** Consecutive edges — and the last with the first — MUST share an endpoint, and sharing is asked of the **points**, which is aliasing and cannot be argued with: two neighbours that share none are **E080** naming both. The loop is walked in the order it is written. A `circle` is a whole loop by itself and MUST stand alone in one (E080); an edge that is not a line, an arc or a circle is E080 at the edge.

**A face has no holes, and the omission is the design.** There is no syntax for a second loop, because a hole in a part is a solid `through` the body (§6.9) and the body rule already says it. Written both ways, one drawing would state one hole twice, in two constructs an implementation would then have to keep in agreement — and P2 gives the body rule the better claim, since it holds however the statements are ordered.

**A face lies in one plane, and does not say so.** Its plane is read off the **memberships** (§6.7) of every point of every edge; those MUST agree, and a dissenting edge is **E080** naming it and both planes. Nothing is written on the face itself. A face bears no points of its own, so an `in` clause on one is refused where it stands — but a face **inside** an `in … { }` block is left *unstamped* rather than refused, exactly as a plane and a curve are: a block stamps the geometry the face is written over, the face is on the plane its edges are on, and refusing it would put a design and the region taken from it in two different blocks.

### 6.9 Solids **[0.18]**

A **solid** is a face swept, or a term over other solids.

```
solid block(sec, from: face, to: back)         // a prism between two ordinates
solid boss(boss_f, depth: 10mm)                // `depth: d` is `from: -d, to: 0`
solid bore(bore_f, about: ax)                  // a full turn about a line in the face's plane
solid lug(lug_f, about: ax, sweep: 90deg, sense: cw)
solid body(block)                              // a body, whose stock is `block`
bore through body                              //   ... less the bore
boss on body                                   //   ... plus the boss
```

**The brackets are what the thing is made of** (§4.3, §6.2), so the sweep stands in them beside the face: `from:`, `to:`, `depth:`, `about:`, `sweep:` and `sense:` are labels of the constructor and are neither seeds nor constraints. A mixture of the two sweeps, a half-written prism (`from:` with no `to:`), `from:`/`to:` beside `depth:`, and `sweep:` or `sense:` with no `about:` are each refused where they are written, with the shapes a solid has.

**Every extent is an expression, and MUST NOT be an unknown.** This is the `fold:` bargain of §6.7 exactly: a number in a solid's brackets is settled by the flattener over the parameters in scope — a `param`, a formal, a named dimension (§5) — and is then document data no solve moves, checked against its slot's dimension (`Length` for a prism's ordinates, `Angle` for a sweep; **E103** otherwise). A solid allocates no parameter, so P3's other half holds without a rule of its own: there is nothing here for a solve to rewrite.

- **A prism** runs `from:` one signed ordinate `to:` another **along the face's own plane normal**. Those signs are arithmetic and not a convention (§9.2 **[0.17]**) — they are ordinates on an axis, and a document writes both. `depth: d` is the draughtsman's spelling of `from: -d, to: 0`, the material *behind* the face the view shows, and is therefore a **magnitude**. A prism swept nowhere (`from` equal to `to`) is **E080**.
- **A revolution** turns a face about `about:` a line, which MUST be a line (**E081**) and MUST lie in the **face's own plane** (E081) — a line drawn in another view names a direction this face knows nothing about. `sweep:` is how far and is a **magnitude**, a full turn where the document writes none; **which way is a word, not a sign** (§9.2, §9.4): `sense: cw | ccw`, right-handed about the line's own `p1 → p2` unless `cw` is written, and a negative `sweep:` is **E040** at the value, in the words of the selector that replaces it.
- **A body** names its **stock** in the brackets and takes its features from the `on` and `through` statements that name it. A solid whose brackets hold solids and no face is a body; one written over a face is not, and a feature written into a swept solid is E080 with the cause and the spelling that fixes it.

**The body rule, and the whole of it.**

> **A solid is its stock, plus everything `on` it, minus everything `through` it.**

As a point set, with `on(s)` and `through(s)` the two **sets** of statements naming `s`, and `S(s)` its stock:

```
B(s) = ( S(s) ∪ ⋃ { B(x) : x on s } ) ∖ ⋃ { B(y) : y through s }
```

Union before difference, and neither group is ordered. `on` and `through` are Declaration-class (§4.2, §9.2): each says what its right operand *is*, contributes no residual, and enters no solve. A solid that reaches itself through its operands is **E041** — "made of itself", the words a plane folded from itself is refused in.

**Why a term and not a feature tree.** A feature tree is imperative because it is *stateful*: step *n* acts on the anonymous body as of step *n−1*, and names faces by the order they were made in. Solvent names everything, so the order lives **inside the term**, over names, exactly as it lives inside `h = w / 2`; between statements there is none, which is P2. `bore through body` says what `body` is and may stand anywhere in the file — above the declaration it qualifies, below it, or in a component beside it — and moving it changes nothing.

A design that wants the other order **names the intermediate**, and then there are two solids because there are two things:

```
solid recess(pocket)      // the pocket, less the boss standing in it
boss through recess
solid body(block)
recess through body       // block − (pocket − boss), which the flat rule cannot spell
```

**A solid's faces are reached by path, never by index** (§3.2). A prism's caps are `.far` and `.near`, the lower and the higher of its two ordinates along the normal, so `depth: d` leaves `.near` the face the view shows; each side is named by the edge it was swept from, in the name the *source* wrote (`block.side_l`). A revolution names its wall by the edge likewise and its caps `.start` and `.end` where the turn is partial. Through a body the operand keeps its own name (`body.block.near`). An implementation MUST NOT name a face by its position in anything: that is §13.1's rule, and a boolean is exactly the operation that would renumber one.

What a report says about a solid (§16.3) is therefore `NAME.volume`, `NAME.area`, `NAME.bounds.{x,y,z}{0,1}`, and `PATH.area` for each of its faces that survived — a bore that ate a cap leaves a name the document still writes and no area behind it, which is a fact and not an error. Those numbers MUST be taken at a faceting the **document** fixes and never at the screen's: a volume that changed with the zoom is a number nobody could quote.

**A solid stands on no plane.** It bears no points, so `in` on one is refused where it stands and an `in … { }` block leaves it alone (§6.8). It is not picked, dragged or dimensioned on the sheet, and it is evaluated after the drawing is solved (§1.2, §3.1).

*Non-normative:* `rust/examples/vtwin/cylinder.sv` is the worked case — one section in the plane of swing, the body swept from it, and the bore, the port, the bolt hole and the head's slot four more solids `through` it. The part had been written three times, the same body redrawn in two more views as page-aligned rectangles re-tied by `project`, with every depth ordinate related to the section by no statement at all.

### 6.10 Planes stood off **[0.18]**

**`from:` says which plane a plane is derived from, and the clause beside it says how.** §6.7 gave `from:` one reading; it has two.

```
plane top(origin: o2, toward: q2, from: front, fold: 0deg)     // turned about a line in `front`
plane back(origin: o,  toward: q,  from: front, offset: 12mm)  // moved along `front`'s normal
plane same(origin: o,  toward: q,  from: front)                // moved by nothing
```

- `offset: k` stands the plane off the one it is derived from, `k` **along that plane's own normal**, with the same attitude. It is `Length`-dimensioned, and it is document data exactly as the fold and the basis are: never a seed, never an unknown, never moved by a solve. An omitted one is 0.
- `fold:` and `offset:` in one declaration are refused where they are written, as `from:` beside a `u:`/`v:` basis already is. A plane folded from an offset plane stands where that plane stands: a fold turns about a line *in* the parent, so the two share an origin.

**The withdrawal, and why.** Through 0.17 a bare `from:` read as `fold: 0deg`. That default is withdrawn. No document in the corpus ever used it, and a plane that names another and folds nothing most plainly says *the same plane, moved* — which is what a stack of parts is written in, and the reading the offset now gives it. A document that meant the top view MUST write `fold: 0deg`, which every such document already did.

**An offset is along the normal, and only along it — which is why `project`'s residual is unchanged.** Let `d = (n_A × n_B)/|n_A × n_B|` be the fold line two views share (§6.7). It is perpendicular to both normals by construction, so a plane offset by `k` has its origin at `o = k·n` and

`d · o = k (d · n) = 0`

— the residual of §9.3 measures each image from its own plane's origin along `d`, and no term of it can see the move. An offset *within* the plane would move the origin both images are measured from and put a constant in a row that has none, which is why the language gives it no spelling.

*Non-normative:* the views of one drawing are images of one object and share one origin, which is `o = 0` for every plane in every document written before there were solids. A part standing a wall's thickness in front of another does not share it, and `offset:` is where that thickness is stated once.

**A mate places a plane, and then the offset is a consequence.**  A stack of parts is a chain of contacts, and stating each one's offset as a number is stating the same chain of subtractions in as many places as there are parts.

```
plane swingA(origin: O, toward: up, from: views.front)     // parallel, and placed by a mate
cylA.block.far against plate.body.near
washer.body.far against cylA.block.near
```

`F against G` says the two faces are **in contact**.  `F` and `G` each name a *face of a solid* by the path §6.9 gives it, and the statement is Declaration-class: it defines where the left face's plane stands and contributes no residual.  Operand order carries meaning — the left one is placed, the right is the datum.

- The plane `F`'s solid is drawn in MUST be a **placed** plane: one written `from: P` with neither `fold:` nor `offset:`.  A plane the document already positioned is **E083**, as is one placed twice, and a placed plane that no mate places is **E083** for the opposite reason.
- The two faces MUST be planar, parallel, and looking at each other — their outward normals opposite.  Each is **E083** at the statement.  A face at no single ordinate along its plane's normal (the side of a prism, the wall of a revolution) is nothing a stack can bear on: **E082**, naming it.
- Two faces in contact are at the same point along the normal they share, which fixes one offset per statement.  The offsets are worked out in **dependency order** and a cycle is **E041**, the same words a plane folded from itself gets.

*Non-normative:* what this replaces reads, in the drawing it was written for, `zA = fwA + D / 2` and `tdisc = zA - rw / 2 - wsh - zdisc`.  Those are true statements about the object; they are just not the ones the designer made, which were "cylinder B's face against the plate's front" and "a washer between the disc and rod A".

### 6.11 Views and sections **[0.18]**

A part carries no views. It is a solid, and a sheet that wants a picture of it **asks** for one:

```
view(cyl.body) in views.right                  // what that view sees of it
view(cyl.body) in views.top
section(body, at: front) in front              // cut at a plane, drawn in a view
view detail(block) in side class heavy         // named, and classed, both optional
```

`view(S) in P` and `section(S, at: Q) in P` are **outputs, not entities**. Neither declares anything: no entity is minted, no name is bound, no constraint may relate to one, and deleting the statement removes the picture and nothing else. `S` MUST name a solid and `P` and `Q` MUST name planes (**E040** naming the kind that arrived, **E101** for a name nothing declares). The name before the brackets is optional and is carried for the report; a `class` clause after the view is the ordinary one (§13.2). `at:` cuts a section and a plain view takes none; a `section` with no `at:` is refused where it is written.

**A section is drawn in a view parallel to the plane it is cut at**, or the true shape it shows is not the shape it is a section of; a cutting plane that shares a fold line with the view it is drawn in is **E084** at the statement.

Every stroke comes back in the **page** coordinates of the plane it is drawn in — through that plane's own origin and rotor — so a derived picture sits on the sheet where its view sits, over the geometry drawn there.

**Three drafting rules, and each is the whole of a convention.**

1. **A corner is drawn and a tessellation seam is not.** A round surface is reduced to flats by the same sagitta rule the drawing's own arcs are drawn by, and the seams between those flats are not edges of the design. A seam is drawn only where it is a **silhouette** — where the surface turns away from the eye across it — so a cylinder is the two lines a draughtsman draws and not the sixty-four its facets would give, at every zoom.
2. **What the material covers is dashed, not dropped.** A hidden line is a line. Visibility can change only at an apparent crossing or at an end, so each edge is cut at its crossings in the picture and each piece classified by the eye's ray from its middle; a piece the solid stands in front of is drawn under `.hidden`.
3. **Coincident page lines are drawn once, and visible wins.** Where several strokes fall on one line of the page, what is seen is the union of the visible stretches and what is dashed is the union of the hidden stretches *less* that — so a hidden edge behind a visible one is not drawn twice and does not break it. Where a stretch is claimed by both a corner and a silhouette the corner wins, a corner being a fact about the object and a silhouette only about this view.

**The classes are implicit** (§13.2): every stroke of a picture carries `.visible` or `.hidden`, a section's visible strokes carry `.section` beside `.visible`, and the statement's own classes are carried over those. So a sheet says what a hidden line looks like the way it says what a dimension does, and a document that already styles `.hidden` gets its dashes with nothing added.

*Non-normative:* a picture is laid out in the implementation and stroked by the front end, the seam a dimension callout and a curve's tessellation already sit on — so an export and a canvas draw one answer, and neither owns a line of three-dimensional arithmetic. The gate is not that a derived view matches some pixels; it is that a view taken in the plane a face was drawn in gives back the outline that was drawn.

---

### 6.12 The sheet as a report **[0.18]**

```
dimensions(cyl.body) in views.right
```

`dimensions(S) in P` asks for the callouts that **follow from the object**, laid out by the same engine that lays out every dimension a document states. Like `view` and `section` it is an output: nothing is minted, no name is bound, and it adds no equation, no unknown and no freedom. A generated callout is a *reading* of the drawing, so an implementation MUST NOT let one be addressed as a statement — dragged, edited, given a placement, or resolved back to a constraint — and MUST read it off the **solved** pose, since what it says is what the geometry came to and not what a statement asked for.

What it generates is deliberately bounded, and the boundary is the point:

- the part's **overall extents** in that view, one along each of the view's own axes, measured between the faces that bound them and stood clear of the outline; and
- the **diameter** of every round feature that view sees square on.

An implementation MUST NOT invent more than that. Which datum a stack is measured from, which fit is critical, what is a reference and what controls the drawing — those are the design, and a machine that chose them would be guessing. A sheet states the rest as it always did; this is what it no longer has to.

*Non-normative:* the complaint this answers is that half the edits to a part sheet were placements, moved off each other by trial and then rendered to see whether they had landed. A drawing's *author* needs the picture; nothing about producing it needs a person.

---

## 7. Ports **[0.13]**

**Retired.** Everything an instance makes is reachable by its dotted name — `inst.p` for a point of the body, `inst.sub.p` for a nested instance's, `inst.s[0].p1` for one inside a block's copy — so a port was a second name for a thing that already had one (bmander/geomsolver#47). Its three forms are written as what they were:

| was | is |
|---|---|
| `port lo: point hint(x: 0, y: 0)` | `point lo hint(x: 0, y: 0)` — a declaration of the body |
| `port hub = c` | nothing; the caller writes `inst.c` |
| `port p = (xexpr, yexpr)` | `point p = (xexpr, yexpr)` — a computed point (§6.5) |

An implementation MUST refuse `port` with a message naming these forms. Aliasing is untouched, being a property of argument passing and not of ports (P1): passing one instance's entity to another as an argument still merges the two into one alias class.

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

### 9.2 Operator form **[0.7]**

> **Every constraint is written as a prefix or an infix operator.  `name(args…)` is retired.**

Where a constraint needs more than its two operands — a number, a selector, a third entity — those go in parentheses **on the operator**:

```
horizontal line1                    point1 horizontal point2
radius(25) circle1                  point1 distance(1' 3") point2
distance(6) line1                   point1 symmetry(line1) point2
distance(x = 7) line1               point1 distance(60, along: x) point2
ground p1                           l1 angle(30) l2
fix c.r                             line1 tangent(side: -1) circle1
```

**The type system already has this shape.** Every constraint a person writes has 1 or 2 entity slots, always first in spec order: 1 for `horizontal`, `vertical` and `radius`, 2 for everything else, and 3 for symmetry alone — which the parentheses absorb exactly as proposed. So "two operands, the rest in parentheses" is a *description* of the library rather than a rule imposed on it.

What goes in the parentheses is a short list:

- **the number**, which may be named or an expression exactly as elsewhere: `distance(80)`, `distance(x = 7)`, `distance(h = w / 2)`, `distance(1' 3")`;
- **a selector** — `side: left`, `at: start`, `external: true`, `along: x`. **[0.17]** A selector's *key* must be one the word has (a slot of the settled kind, or `along`, which chooses the kind and fills no slot), and its *value* must be one of the words that slot takes — both **E040**, at the key. Neither was checked through 0.16, and both failures were silent: a mistyped key was dropped and the statement settled without it, and a word outside the set fell through to whichever reading the implementation tested for last, so `at: banana` meant `end`. An implementation MUST publish each slot's vocabulary in its registry, so that a front end offers what the core accepts rather than keeping a second list;
- **the third entity**, for `symmetry`;
- **a pin**, `t == 0.4`, for a slot the constraint owns. Its *seed* is the trailing `hint(t: 0.4)` where every seed in the language is (§4.3).

| word | fixity | operands → constraint |
|---|---|---|
| `on` | infix | (point, line \| circle \| arc \| spline \| curve) — **four** constraints; **[0.18]** (solid, solid) — the body rule (§6.9), and no constraint at all |
| `through` | infix | **[0.18]** (solid, solid) — the body rule's other half (§6.9), and no constraint at all |
| `distance` | infix | (p, p); +`along: x`/`y` for the run and the rise; (p, line); (line, line); (circle, circle) — **six** |
| `distance` | prefix | on a line: the distance between its own ends |
| `tangent` | infix | (line, circle); +`at:` for a tangency at a named end; (circle, circle); (arc, line); (spline, line) — **five** |
| `equal` | infix | (line, line) a length; (circle, circle) a radius |
| `curvature` | infix | (spline, circle), (curve, circle) |
| `horizontal`, `vertical` | prefix / infix | a line; or a pair of points |
| `angle` | infix | (line, line) |
| `radius` | prefix | a circle or an arc |
| `coincident`, `midpoint`, `parallel`, `perpendicular`, `symmetry` | infix | one each |
| `project` | infix | (point, point), each `in` a plane — the two planes are read off the memberships and never written (§6.7) **[0.10]** |
| `ground`, `fix` | prefix | the gauges (§13): a point, or one of an entity's own numbers by its field (`fix c.r`) |
| `ccw`, `cw` | call | three points, all in the parentheses (§9.6) |

The collapses are where the saving is: **`on` is five constraints, `distance` is six, `tangent` is six**, and `horizontal`/`vertical` are two each with the **fixity** doing the work — a line prefixed, a pair of points infixed, which is exactly the distinction the point-pair forms were added to draw. `angle` and `radius` keep their own words rather than folding into `distance`, because over two lines a length means a parallel distance and an angle means an angle, and nothing but the number's unit could separate them.

**Operand order carries meaning.** `arc tangent line` is a tangency at the arc's end; `line tangent circle` is the ordinary one. Each named itself before and the order was decoration; as an operator, which side the arc is written on picks the constraint.

**What a word means is the kinds of its operands, and a name does not carry its kind until elaboration** — a name may be declared further down the body (P2) or come from a component. So an implementation MUST settle the word after resolution, and MUST report a pair a word does not relate rather than guessing at one.

**The surface word and the wire name are separate.** An export format's constraint identifier is unaffected by this section; the operator is information beside it.

**`ccw` and `cw` keep a call.** Under the general rule they would be `a ccw(c) b`, which reorders three points that are symmetric: the predicate is about the *triangle*, not about a pair with a decoration. The call is a third fixity of the same table — every operand in the parentheses — and not a statement kind of its own. **[0.15]** The gauges and the orientation predicates are entries of the operator table like every other constraint: read by the one relation grammar, so a class, a placement and the chain's lookahead treat them as any other word, and settled by the word alone, since `fix c.r` names a number and `ccw(a, b, c)` has no operand outside its parentheses. They hold parameters or record a root choice rather than adding an equation, so a `claim` on one is refused (E040): a claim is judged by rank, and they add no row.

**[0.18] The body rule is written in this grammar and is not a constraint.** `boss on cyl` and `bore through cyl` are **Declaration**-class (§4.2): each says what its right operand *is* (§6.9), contributes no residual, and takes no part in a solve, a decomposition or any partition of work. `on` is settled the way every word here is — by the kinds of its operands, one step further out than `p on c` — so nothing new is spelled for it; `through` relates no geometry and has no residual to be settled into, and is read by the word alone. Neither is in the constraint library of §9.3, and neither may be `claim`ed: a claim is judged by rank and these add no row, which is the rule already stated for the gauges. `claim a through b` is refused where it is written, `through` being no constraint word; a `claim` written on a body `on` asserts nothing, and an implementation SHOULD refuse it rather than accept a statement that says nothing (an implementation that instead drops the word MUST still apply the body rule, since the operands' kinds are what the statement means).

A chain (§6.6) is the same grammar: a **lone infix statement is a one-joint chain**, and what a chain adds is the corner — which end two links meet at — that an operator between two names cannot know.

### 9.3 Standard constraint library

Residual conventions: points are ℝ²; `×` is the scalar 2D cross product; `∠(u, v)` is the signed angle from `u` to `v` in (−π, π].

| Predicate | Residual(s) | Eq. count | Notes |
|---|---|---|---|
| `on(C: Circle, p: Point)` | ‖p − C.center‖ − C.r | 1 | |
| `on(L: Line, p: Point)` | n(L)·p − d(L) | 1 | |
| `coincident(p, q)` | p − q | 2 | for *distinct* entities; see **W100** |
| `distance(p, q) == e` | ‖p − q‖ − e | 1 | |
| `angle(a, b, c) == e` | ∠(a−b, c−b) − e | 1 | signed; see §9.4 |
| `angle(L1, L2) == e` | wrap(∠(dir(L1), dir(L2)) − e) | 1 | directed, mod 2π; see §9.4 |
| `parallel(L1, L2)` | sin(L1.dir − L2.dir) | 1 | |
| `perpendicular(L1, L2)` | cos(L1.dir − L2.dir) | 1 | |
| `tangent(C1, C2)` | ‖c1−c2‖ − (r1 + r2) *or* ‖c1−c2‖ − \|r1 − r2\| | 1 | branch by decoration, §9.5 |
| `tangent(C, L)` | dist(C.center, L) − side·C.r | 1 | `side: left \| right` **[0.17]** |
| `equal(e1, e2)` | e1 − e2 | 1 | any matching dimension |
| `midpoint(m, a, b)` | m − (a+b)/2 | 2 | |
| `ccw(a, b, c)` | (b−a) × (c−a) > 0 | 0 | inequality; selects a connected component |
| `cw(a, b, c)` | (b−a) × (c−a) < 0 | 0 | |
| `revolute(f1: Frame, f2: Frame)` | f1.origin − f2.origin | 2 | relative angle free |
| `weld(f1: Frame, f2: Frame)` | f1.origin − f2.origin, f1.angle − f2.angle | 3 | triggers **W101** |
| `project(p, q)` **[0.10]** | d_A·Rᵀ(c_A, s_A)(p − o_A) − d_B·Rᵀ(c_B, s_B)(q − o_B) | 1 | the planes A, B inferred from `p`, `q`'s memberships; d the fold line they share, §6.7 |

Implementations MAY extend this library. Extensions MUST document residuals and equation counts, and MUST classify each decoration as hint or constraint per P3.

### 9.4 Signed angles

`angle(a, b, c)` is the signed turn at vertex `b` from ray `b→a` to ray `b→c`, positive counterclockwise, in (−π, π]. Equating it to an expression is a 1-equation constraint. Programs that need the unsigned angle write `abs(angle(...))`; implementations MUST warn (**W102**) that `abs` introduces a branch (two solution families) unless an orientation predicate elsewhere disambiguates.

**[0.17]** `sense: cw` turns the number a statement writes: `l1 angle(30, sense: cw) l2` states −30° and is the spelling a drawing SHOULD use, the minus being a coin a reader cannot check. An implementation MUST draw the figure from the number the statement *makes* — the arc sweeping the way the label reads.

`angle(L1, L2)` between two lines is likewise directed: `∠(dir(L1), dir(L2))`, the signed turn from `L1`'s direction (p1→p2) to `L2`'s, positive counterclockwise, in (−π, π] as everywhere else in this section. It is NOT a statement mod a half turn — the residual pins which side, so a bearing needs no orientation predicate beside it — and it is therefore sensitive to the order of the two lines and to the endpoint order each was declared with. Equating it to `e` compares the two mod 2π, so `e` may be written on any lap: 270° and −90° state the same thing, and an implementation MUST NOT treat a stated angle outside (−π, π] as an error.

### 9.5 Branch decorations are constraints

Several predicates have discrete solution branches. Branch selection changes the solution set and is therefore constraint-class (P3), written as a decoration:

```
tangent.ext(c1, c2)      // external tangency: ‖c1−c2‖ = r1 + r2
tangent.int(c1, c2)      // internal tangency: ‖c1−c2‖ = |r1 − r2|
```

Undecorated `tangent` is an error (**E010**) — there is no default branch, because no consistent global rule exists for tangency the way one does for arcs (§10.3).

### 9.6 Inequalities

Orientation predicates (`ccw`, `cw`) are the only inequalities in this draft. They contribute no equations; they select among the discrete solution components of the equality system. Solvers MUST verify them on candidate solutions and MUST NOT report a solution violating one.

### 9.7 Claims **[0.5]**

```
claim <relation>(<args…>) [== <expr>]
```

A **claim** is a relation stated as *expected to add no rank*: an assertion about the drawing
the rest of the document determines, not part of what determines it. The altitudes of a
triangle concur; the trace of a Peaucellier cell is straight — a claim is how a document says
so out loud and has the statement checked, rather than smuggling the theorem in as one more
constraint and hoping the diagnosis reads the intent.

A claim MUST NOT act. Solvers MUST exclude claims from the equation system, from
decomposition, and from any connectivity used to partition work (a claim spanning two figures
does not join them): the solution set, the degrees of freedom, and every diagnostic class of
the surrounding document are exactly what they would be with the claim deleted. In
particular, a claim never makes a document over-constrained or in conflict.

A claim MUST be judged. At a solution, diagnosis classifies each claim as exactly one of:

* **theorem** — the claim holds and its residual rows add no rank to the system: the document
  already implies it;
* **violated** — the claim does not hold at this solution;
* **consuming** — the claim holds, but its rows add rank: only the pose satisfies it, and
  enforcing it would have removed a freedom. The claim claims too much.

A claim MUST NOT introduce unknowns: a relation whose signature carries a solver-owned
parameter (a curve contact's `t`) is an error as a claim (**E017**), since its unknown would
appear in no equation. A claim's dimension may be an expression, but MUST NOT bind a free
variable, for the same reason.

`claim` qualifies a single longhand relation statement; it does not enter chains (§6.6).

### 9.8 Claims about solids **[0.18]**

A claim is a statement judged and never solved (§9.7).  Everything §9.7 says of a claim about the drawing holds of a claim about the **object**, and the reason is one stratum out rather than new: a solid is evaluated after the drawing is solved, so a statement about one compiles no row and can no more act than a `claim a project b` can.

Three words relate two solids.  Each is an operator like any other (§9.2), and each settles to no constraint kind, because a constraint kind is a thing with a kernel.

| written | asks |
|---|---|
| `a clear(d) b` | `a` and `b` are disjoint, and no point of one is nearer than `d` to the other |
| `a inside b` | every point of `a` is a point of `b` |
| `a fits(d) b` | `a` is inside `b`, with no point of it nearer than `d` to `b`'s boundary |

Both operands MUST name solids (**E040** naming the kind that arrived).  `clear` and `fits` take a `Length` in their parentheses and one written without it is **E040**; `inside` asks about containment and takes none.

**A verdict is a measurement, not a yes or no**, and it carries its own uncertainty.  An implementation MUST report, for each such claim: what was measured — a distance, negative where the two overlap — and how far the answer could be wrong.  A claim decided within that margin is reported **undecided**, which is a third answer and not a failure.  Where an implementation evaluates a solid by reducing its round surfaces to flats (§6.9), the margin is the sagitta of that reduction, and a faceted solid lies inside the true one; an implementation that computes exactly reports a margin of zero.  Two claims that fail, one by a hair and one by a hand's breadth, are different drawings, and a reader is owed the difference.

The trichotomy of §9.7 does **not** apply here.  *Consuming* asks whether enforcing a claim would take a freedom, and there is no rank to take: a solid claim holds, is refuted, or is undecided.

**A claim over a sweep.**  Every claim in the language is judged at one pose, and a fact about a *cycle* — a disc clearing a cylinder's mouth all the way round, a port open through mid-stroke — is not one of those.

```
claim over crank.theta in (0deg, 360deg) {
  crank.disc clear(1mm) bankA.cyl.body
  bankA.pis.body inside bankA.cyl.bore
}
```

`claim over NAME in (A, B) { … }` judges every claim in its body as the drawing runs along `NAME`, and reports the **worst** pose reached.  It is Structure-class: it says how the claims inside it are judged and asserts nothing itself.

- `NAME` MUST be a **free variable** of the drawing (§5) — an unknown the solver answers for.  A `param` is a number the document already fixed and sweeping a constant is not a question; naming one, or naming geometry, is **E040**.
- `A` and `B` are read in the units the free variable's readers are written in: an interval of an angle is an angle, and one of a length is a length.
- An implementation MUST state that its answer is by **sampling**, and how many poses it took.  A claim that holds at every sample is a claim that held at every sample; a swept claim is honest about that in the way a faceted one is honest about its margin.

*Non-normative:* the two together are what make a drawing's claims a test suite for the *object* rather than for one picture of it at one moment.  The loop an author works in — write, run, read the verdicts — needs the verdicts to be about the thing being made.

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
hint t.lead(x: center.x + root.r, y: center.y)
```

`hint REF(key: EXPR, …)` seeds the entity `REF`'s named scalars at the values of the expressions (evaluated with whatever definitional values and previously seeded values are available; unseeded quantities in a hint expression are an error **E014**).

Normative invariant (**Invariant H**): *for every program P, sol(P) = sol(P minus all seeds).* Implementations MUST maintain a statement classification sufficient to verify Invariant H syntactically — i.e., the seed class is closed under everything the grammar allows in a seed, and nothing in the seed class can generate residuals or alter aliasing.

**[0.2]** §4.3 is how that classification is meant to be maintained: a number inside `hint(…)` is seed-class and every other number is not, so the check is a look at the clause rather than an argument about the statement.

Hints on entities inside a `ring` seed the fundamental-domain representative (§12.4). Hints MAY use block indices in `repeat`/`cycle` (where each instance is a distinct variable) and MUST NOT use them in `ring` (**E015**: there is only one representative to seed).

---

## 12. Repetition

Three constructs, three meanings. All take a compile-time `Int` count and an optional index binder.

### 12.1 `repeat` — open array

```
repeat N as i { ... }
```

Pure elaboration: N copies of the body, index `i` ∈ 0..N−1 available in expressions and hints. `next`/`prev` are illegal in `repeat` (**E020**); cross-instance references use explicit indexing `name[k]` from outside or arithmetic indexing patterns from inside. **[0.8]** A body ending mid-joint (§6.6) states its trailing joint between consecutive copies and leaves the last copy's unstated — the joint is the block's own statement, so this does not put `next` in the body's scope.

### 12.2 `cycle` — structural closure, no symmetry

```
cycle N as i { ... }
```

Elaborates N copies; `next` denotes instance (i+1) mod N and `prev` instance (i−1) mod N. Instances are independent variables; nothing forces them to resemble one another. Use for closed chains of unequal links. **[0.8]** A body ending mid-joint (§6.6) states its trailing joint at every pair — the wrap included, so the trailing joint is the loop's closure.

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

**[0.17] A recorded solution branch is one record of one triangle.** Three points can be named six ways, and a chirality named in two of those orders is the same fact with the sign turned. An implementation MUST therefore record a branch **canonically** — one order of the three points, with the sign read against that order — and MUST normalise a record it reads into that form. Keyed by whichever order each writer happened to use, a document's stated orientation and the same choice as the implementation constructs it are two records that never meet: the stated one matches nothing and decides nothing, and the constructed one, written back out as a statement, names a different triple. Both are silent, which is this section's subject.

**[0.7] A callout's placement stays here, and the sheet takes everything it shares.** Where a dimension's number sits on the page is presentation: delete every `at (t, r)` and the figure, the solve, the DOF count and the diagnosis are identical. It is nonetheless written on the constraint statement, and that is deliberate rather than a leak.

The alternatives are worse in ways this section already names. Keying a placement by *position* or by *entity index* fails silently, which is what the section exists to forbid. A *selector* over the statement (`place distance(p0, p1) at (12, -4)`) is ambiguous by construction: a second length on the same pair is a state a document is allowed to be in, and reporting it as over-constrained naming both is the point, so a selector on type-and-arguments cannot always name one dimension. A *minted id* is a name with a machine's spelling, and it fails the first time somebody copies a block of text: two statements with one id and no reader able to tell which was meant. **Naming the dimension** (`distance(p0, p1) == 80 as d1` and a separate `place d1 at (12, -4)`) is the one alternative that satisfies both constraints — a name is identity, not appearance — and it costs every dimension anyone has ever dragged a name nobody asked for, minted by a second splice into the geometry statement.

So a placement stays on its statement, and orthogonality is achieved the other way round: **the sheet (§13.2) owns everything a callout *shares*** — the ink, the weight, the dash, and whatever else is added later — and the statement keeps the one pair of numbers that is about that statement alone and nothing else. A class is a rule many statements share; a placement is a fact about one, and that is exactly the difference between the two constructs.

Whatever an implementation does, a placement whose dimension is gone MUST be gone with it: removed, or reported. Never silently inert while the document still carries it, which is the failure this section names.

### 13.2 Presentation: classes and style sheets **[0.7]**

> **A document says what the drawing *is*. How it *looks* is a separate statement, in a separate part of the file, and changing it is never an edit of the geometry.**

A declaration carries a **class**; a `style` block says what a class looks like.

```
style .construction { dash: 7 4 }
style .centerline   { dash: 12 3 2 3; width: 0.5; color: #888888 }
style .heavy        { width: 2.5 }

line   datum(center, anchor) class construction
line   ab(a, b) class centerline heavy
circle base(center: c) hint(r: Rb) class construction
```

**[0.12] Three more places a class stands, and one more property.** A *relation* statement carries a class as a declaration does — `a distance(80) b class shown` — and a dimension's callout is drawn in `.dimension` (and `.reference` when it is a claim) under the classes its statement carries; on a relation that states no dimension a class is inert. An *instance* carries one — `t2: Throw(…) class phantom` — and every declaration **and relation** its expansion makes carries it *over* its own, the assembly's word being the later and stronger, the way `in` puts an instance in a view (§6.7). Every *point* is drawn under the implicit class `.point`, so a document may say that its handles are not part of the picture. The property `display: none` leaves what carries it out of the picture altogether — an entity is not drawn, a dimension is neither laid out nor picked — `display: inline` shows it again from a later class, since an absent property says nothing, and `display: geometry` draws what carries it and never dimensions it: an entity under it is shown and a dimension whose statement carries it is not, which is a phantom position. The idiom for a drawing dense with dimensions is `style .dimension { display: none }` and `class shown` on the few the sheet is to show. Nothing about the solve, the count or the diagnosis reads any of it.

**[0.18] Three more implicit classes, for the pictures a document asks for.** Every stroke of a `view` or a `section` (§6.11) carries `.visible` or `.hidden` according to whether the material stands between it and the eye, a section's visible strokes carry `.section` beside `.visible`, and the classes the statement itself was written with are carried **over** those — the assembly's word being the later and stronger, as everywhere in this section. The base sheet says:

```
style .hidden  { dash: 4 3; color: #7a7a7a }
style .section { width: 1.6 }
```

`.visible` has **no base rule**, and that is the cascade rule of this section rather than an omission: a visible stroke is the plain outline, so a rule stating a weight or an ink for it would beat a document's own `style .hidden { width: 2 }` on half the drawing. Each of the two rules that exist states only what its class *adds*, for the reason `.reference` does.

- **A class goes on an entity declaration**, and nowhere else. Not on a component definition, not on an instance, not on a `cycle`. Those are all reasonable later and none of them is needed.
- **A declaration MAY carry several classes**, space-separated. On a conflicting property the later one wins, so `class centerline heavy` is a centreline drawn thick — and *only* on the properties the later one states.
- **`style` blocks sit at the top level.** An external sheet, shared across drawings, is the natural extension once there is more than one drawing to style; it is not specified here.
- **An unmatched class is not an error.** It simply has no rule, exactly as in CSS — which is also what makes paste work: a figure copied out of a document with a sheet keeps its class names and picks up whatever the destination says about them, or nothing.

| property | value | |
|---|---|---|
| `dash` | a list of lengths | as `stroke-dasharray`; empty or absent is solid |
| `width` | a length | stroke weight |
| `color` | `#rrggbb` | stroke ink |

**Lengths in a sheet are screen pixels**, not world units — the rule that already governs everything drawn at a constant size. A dashed line does not change its dash pattern when you zoom.

**`construction` is retired as a keyword.** What it did is one rule in an **implicit base sheet** that an implementation ships and a document may override:

```
style .construction { dash: 7 4 }
```

A document that overrides `.construction` MUST change how it draws and nothing else: same solve, same DOF, same diagnosis.

**The base sheet is a layer *under* the document's, not a rule interleaved between a declaration's classes.** A style resolves by cascading the whole base sheet over the empty style in written class order, then the whole document sheet over that, again in written class order. So what a document states beats what the implementation ships whichever class it happens to be written on — the rule CSS states between an author sheet and the user agent's — and a base rule may state only what its class *adds*, since anything it restates would override a document rule written on an earlier class.

**No algorithm may consult a class.** This is the point of the section, and it is a normative constraint on implementations rather than on documents: presentation is read where a drawing is drawn and nowhere else. It is also why the *implementation* resolves the cascade rather than a front end — a callout's figure and a curve's tessellation are laid out in the same place for the same reason, so that two front ends draw one drawing alike.

An export format MAY record a class list. It SHOULD go on reading whatever the format wrote before there were classes; `construction` in particular SHOULD load as the class of that name.

---

## 14. Elaboration semantics

Elaboration lowers a program to the **kernel form** consumed by solvers. The pipeline is normative in effect, not in mechanism.

### 14.1 Phases

1. **Instance expansion.** Recursively inline component bodies for the root's instance tree, freshening names by instance path. `repeat`/`cycle` unroll. `ring` either unrolls to `cycle` + symmetry constraints (§12.3) or lowers to quotient form (§12.4); the two MUST be solution-equivalent.
2. **Alias resolution.** Union-find over all names, merging classes for: entity-typed arguments and constructor entity-arguments. Each class gets one representative entity. Type mismatch within a class is an error (**E040**).
3. **Definitional substitution.** Definitional equalities (constructor value-arguments, `param`, `= expr` declarations) are substituted, METAFONT-style: they are not residuals and consume no solver iterations. A cyclic definitional dependency is an error (**E041**).
4. **Constraint collection.** Predicate statements, `==` equations, derived path incidences (§10.4), symmetry constraints, and gauges are collected into the constraint store.
5. **Path assembly.** Fragments compose per §10.5 into boundary curves attached to the model as derived objects.

### 14.4 Modules **[0.12]**

```
use engine.dims
use engine.parts
```

A **module** is a Solvent document read for its components. `use NAME` at the top level of a document — never inside a body — asks for one; `NAME` is a dotted path, and **what it resolves to is the host's question**: an implementation with a working directory resolves `engine.parts` to `engine/parts.sv` beside the document, one without a filesystem resolves it against whatever library it carries, and both fall through to the other in that order. The core takes text and never opens a file. A module contributes exactly its **component definitions** and its top-level **`param`s** (§6.3); its own loose statements — its drawing — are not drawn, so a document that is also a library (`gear.sv`) is a module as it stands. A module's own `use`s are followed the same way, each module linked once however many times it is asked for, so a diamond is one copy and a cycle terminates. A module a host cannot resolve is **E070**, at the `use`. Two definitions of one component name, wherever they come from, are **E071**, at the document's own definition when the clash is with one and otherwise at the `use` that brought the later module in; there is no shadowing (§5). A module's own errors — a parse error, a faulty `param` — are reported to a reader of the document *at the `use` that brought the module in*, with the module's name, line and column in front of the message, since that line is the one the document can edit.

*Non-normative:* an implementation may parse a module with its spans offset past everything linked before it, so that every span in a linked program is one integer into one virtual text and no consumer learns a second coordinate; a splice on the document then walks the root body alone, which no module span is ever in.

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
| E004 | a positional argument that binds a value parameter, or that follows a labelled one (§4.1) **[0.17]** |
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
| E041 | cyclic definitional dependency (a plane folded from itself, §6.7; **[0.18]** a solid made of itself, §6.9) |
| E050 | inconsistent system (no solution); report a minimal infeasible subset when computable |
| E060 | a point put on two different planes (§6.7) **[0.10]** |
| E061 | `project` refused: a point on no plane, both on one plane, or parallel planes (§6.7) **[0.10]** |
| E070 | a `use` nothing resolves (§14.4) **[0.12]** |
| E071 | a component defined twice, across the document and its modules (§14.4) **[0.12]** |
| E080 | a face or a solid the model cannot build (§6.8, §6.9) **[0.18]**: a loop that does not close, an edge that is not a line, an arc or a circle, a circle standing *in* a loop rather than being one, edges in two planes, a swept solid written over anything but one face, a body made of what is not a solid, a prism swept nowhere, a feature written into a solid that is a face swept rather than a body |
| E081 | a revolution's axis: not a line, or not in the face's own plane (§6.9) **[0.18]** |
| E082 | a face of a body that the body no longer has (§6.9) **[0.18]** |
| E083 | a stack that contradicts itself, or a placed plane placed twice or never (§6.10) **[0.18]** |
| E084 | a section whose cutting plane is not parallel to the view it is drawn in (§6.11) **[0.18]** |

**[0.18]** Two of the five are reserved: an implementation is required to have the codes and to use them for these conditions when it detects them, and the reference implementation raises E080, E081 and E084 today. A negative sweep and a picture asked of the wrong kind are **E040** by the rules those codes already state (a magnitude written with a sign, §9.2 **[0.17]**; a kind mismatch, §14.1), and an unresolved name in any of these statements is **E101**, so no new code is spent on either.

### 16.2 Warnings and lints

| Code | Condition |
|---|---|
| W100 | `coincident(p, q)` where making `p`,`q` one entity would suffice — "consider binding instead of constraining" |
| W101 | frames fully welded by constraints — "consider passing one entity to both" |
| W102 | `abs(angle(...))` without a disambiguating orientation predicate |
| W103 | rank deficiency spanned by rigid motions — "add ground/fix" |
| W104 | under-constrained: report the number of residual DOF and, when computable, a basis of unconstrained motions attributed to source entities |
| W105 | consistent redundancy: constraints dependent on others; report the dependent set with spans |
| W112 | a `param`, formal or block binder declared over a built-in name (§3.3, §5) — the built-in is what an expression reads **[0.17]** |

### 16.3 DOF ledger

Implementations SHOULD emit, on request, a degrees-of-freedom ledger: per alias class, its free DOF after definitional substitution; per constraint, its equation count; totals per component and for the model; gauge accounting. (§18.3 shows the gear's ledger.)

**[0.17]** They SHOULD also emit, on request, **where each name landed**: the value of every scalar of every entity the source names, keyed by that name and the field (`hinge.x`, `base.r`, `view.angle`). A report that says how many freedoms a drawing has and which statements disagree, and never where anything *is*, leaves its reader to recover coordinates by stating a `claim` and reading whether it is refuted. The names and the numbers are both already in hand; this is a serialisation and imposes no analysis.

---

## 17. Deferred and open issues (non-normative)

1. **3D lift.** **[0.10]** Multiview drawing is settled *without* one — §6.7: a `plane` is a frame with a constant attitude, a point is `in` a view, and `project` is the one equation two images of a point share; nothing three-dimensional is solved for, no true length is measured, and a view's attitude is never an unknown.

   **[0.18] The object is settled too, and on the same terms.** A face is a region of a plane (§6.8), a solid is a face swept or a term over other solids (§6.9), a plane may be stood off another (§6.10), and a view or a section of a solid is a picture the sheet asks for (§6.11) — so a part is written once and every drawing of it is a question, with no depth kept in step by hand. The stratification is what makes it affordable and is the thing to preserve: **nothing three-dimensional is an unknown**, every extent is an expression, and a solid owns no parameter, so the solver contract of §15 and the ledger of §16.3 are untouched.

   What remains open is the lift itself, and it is now a shorter list. **Lofts** — a solid between two faces on two planes, which is the one sweep the grammar does not have. **Fillets and chamfers**, which need a name for an *edge* rather than for a face: the spelling is reserved, `body.block.side_l.near` — the edge where two named faces of one solid meet, in the path vocabulary §6.9 already uses, so that a boolean cannot renumber one. A **rigid-body mate solver**: joints between two solids in space, at which point something three-dimensional does become an unknown and P4's decomposition question is asked again one stratum out. **Export of the object itself** (a boundary format such as STEP), as against the picture of it a view already exports. Still open from before: `Frame` generalizes; the arc-branch rule needs a replacement (no global winding in 3D); the joint library grows (revolute gains an axis argument, add prismatic/cylindrical/spherical); and from §6.7, a solved-for fold (`fold: along l`).

   **[0.18]** One item of the old list is answered by §6.9 rather than deferred: "`ring` generalizes to rotation about a line" is what `about:` does for a **sweep**, and it needed no group action to do it, because a revolution is one solid and not *N* congruent copies. `ring` itself is still the cyclic-symmetry question of §12.3 and is untouched by this — the reference implementation goes on refusing the word until it can hold its copies congruent. The two were only ever adjacent.
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
  point lo
  point hi
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

  frame f0(origin: center, toward: t[0].lead)
  port hub = f0

  ring N about center as i {
    t: Tooth(root, tip, slot: tau/N)
    t.trail ~root~ next.t.lead
    angle(t.trail, center, next.t.lead) == tau/(2*N)
  }

  hint t.lead(x: center.x + root.r, y: center.y)

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
program        = { use_decl } { component } ;
use_decl       = "use" IDENT { "." IDENT } ;                         (* a module, §14.4 *)
component      = "component" IDENT "(" [ params ] ")" "{" { statement } "}" ;
params         = param { "," param } ;
param          = IDENT ":" type ;
type           = "Int" | "Scalar" | "Length" | "Angle" | "Side"      (* §4.1 [0.17] *)
               | "Point" | "Line" | "Circle" | "Plane" | "Path"
               | "Face" | "Solid" ;    (* [0.18]; `Frame` was folded into `Plane` in 0.15 *)

statement      = decl | constraint | hint | gauge | block | path_decl | style_rule
               | unit_decl | frag | in_block | body_rel | derived_decl ;   (* [0.18] *)
in_block       = "in" ref "{" { statement } "}" ;         (* membership, written once: §6.7 *)
style_rule     = "style" "." IDENT "{" { IDENT ":" value { ";" } } "}" ;   (* §13.2; `display: none | inline` *)
unit_decl      = "unit" IDENT ;                                           (* §3.3.2 *)

(* §3.3.1: a number may carry a unit, and feet-and-inches is ONE literal — a space is what
   tells the readings apart, exactly as it does in a mixed fraction. *)
number         = digits [ "." digits ] [ ("e"|"E") [ "+"|"-" ] digits ]
                 [ " " digits "/" digits ]                                (* mixed fraction *)
                 [ unit_suffix ] ;
unit_suffix    = "mm" | "cm" | "m" | "km" | "in" | "ft" | "thou"
               | "deg" | "rad" | "grad"
               | "'" [ " " number "\"" ]                                  (* feet, and inches *)
               | "\"" ;

decl           = entity_decl | param_decl | curve_def | instance_decl ;
(* face_decl and solid_decl below are entity_decls, spelled out for what their list holds *)
entity_decl    = ekw binder { "," binder }
               | ekw IDENT "=" expr
               | "point" IDENT "=" "(" expr "," expr ")" ;   (* a computed point, §6.5 [0.13] *)
ekw            = "point" | "circle" | "line" | "plane" | "spline"
               | "curve" | "face" | "solid" ;                   (* §6.8, §6.9 [0.18] *)
(* the trailing clauses are order-free: `hint(…)`, `knots […]`, `class …`, `in REF`.  A
   place — `hint(at: t)`, `hint(at: c, bearing: …)` — is the same clause with `at:` and
   `bearing:` for keys, §6.4 [0.14] *)
binder         = IDENT [ "(" ctor_arg { "," ctor_arg } ")" ] { trailer } ;
(* §6.1: no list at all is the anonymous form — the kind's children are minted and reached as
   `l.p1`.  A slot is a name, a seed, or implicit; only an overfull list is E103. *)
trailer        = hint_clause
               | "knots" "[" number { "," number } "]"
               | "class" IDENT { IDENT }                   (* presentation, §13.2 *)
               | "in" ref ;                                (* membership of a plane, §6.7 *)
hint_clause    = "hint" "(" hint_item { "," hint_item } ")" ;   (* SEEDS, §4.3 *)
hint_item      = IDENT ":" expr | "at" ":" ref | "bearing" ":" expr ;   (* a place, §6.4 [0.14] *)
ctor_arg       = [ IDENT ":" ] ( ref | hint_clause )       (* what the thing is made of, §6.2 *)
               | "from" ":" ref                            (* which plane it is derived from, §6.7, §6.10 *)
               | "fold" ":" expr                           (* the fold, an Angle, §6.7 *)
               | "offset" ":" expr                         (* stood off along the normal, §6.10 [0.18] *)
               | ( "u" | "v" ) ":" "(" expr "," expr "," expr ")"     (* a plane's basis, §6.7 *)
               | sweep_arg ;                               (* how a solid is swept, §6.9 [0.18] *)

(* §6.9 [0.18]: the sweep stands in the brackets with the face, being what the solid is made
   of; every extent is an expression and never an unknown.  `from:`/`to:` or `depth:` is a
   prism, `about:` a revolution, and neither is a body. *)
sweep_arg      = ( "from" | "to" | "depth" | "sweep" ) ":" expr
               | "about" ":" ref
               | "sense" ":" ( "cw" | "ccw" ) ;

(* §6.8, §6.9 [0.18].  Both are ordinary entity_decls — `face` and `solid` are element
   keywords — and are written out here for what their one list slot holds. *)
face_decl      = "face" [ IDENT ] "(" ref { "," ref } ")" [ "class" IDENT { IDENT } ] ;
solid_decl     = "solid" [ IDENT ] "(" term ")" [ "class" IDENT { IDENT } ] ;
term           = ref { "," sweep_arg }                     (* a face swept *)
               | ref { "," ref } ;                         (* a body: its stock, then solids *)
body_rel       = ref ( "on" | "through" ) ref ;   (* the body rule, §6.9 — `on` is the ordinary
                                                     infix word, read as this when both
                                                     operands are solids; `through` is its own *)

(* §6.10 [0.18]: a stack.  The operands name *faces* of solids by the path §6.9 gives them, so
   the word after the left one is past a dotted reference and not past one token. *)
mate           = ref "against" ref ;

(* §9.8 [0.18]: a claim about the object.  The three words are operators like any other, and a
   sweep says how the claims in its body are judged. *)
solid_claim    = [ "claim" ] ref ( "clear" "(" expr ")" | "fits" "(" expr ")" | "inside" ) ref ;
claim_over     = "claim" "over" ref "in" "(" expr "," expr ")" "{" { solid_claim } "}" ;

(* §6.11 [0.18]: a picture asked of a solid.  An output and not a declaration — nothing is
   minted and the optional name binds nothing. *)
derived_decl   = view_decl | section_decl | dims_decl ;
view_decl      = "view" [ IDENT ] "(" ref ")" "in" ref [ "class" IDENT { IDENT } ] ;
dims_decl      = "dimensions" [ IDENT ] "(" ref ")" "in" ref
                 [ "class" IDENT { IDENT } ] ;             (* §6.12 [0.18] *)
section_decl   = "section" [ IDENT ] "(" ref "," "at" ":" ref ")" "in" ref
                 [ "class" IDENT { IDENT } ] ;

(* a curve, §6.5 [0.11]: a point of a component as one of its numeric formals runs.  0.2's
   family form and 0.3's `trace … where { … }` are retired and MUST NOT parse. *)
curve_def      = "curve" IDENT "=" [ IDENT "(" [ args ] ")" "." ] ref
                 "over" IDENT "in" "(" expr "," expr ")" ;
param_decl     = "param" IDENT "=" expr ;
instance_decl  = IDENT ":" IDENT "(" [ args ] ")" [ "in" ref ] [ "class" IDENT { IDENT } ] ;   (* §6.7, §13.2 *)
args           = arg { "," arg } ;
arg            = [ IDENT ":" ] expr ;

(* §9.2: every constraint is a prefix or an infix operator; `name(args…)` is retired. *)
constraint     = [ "claim" ] ( prefix_form | infix_form )
                 [ hint_clause ] [ "at" "(" number "," number ")" ] ;  (* §4.3, §13.1 *)
prefix_form    = IDENT [ op_args ] ref ;
infix_form     = ref IDENT [ op_args ] ref ;
op_args        = "(" op_arg { "," op_arg } ")" ;
op_arg         = expr                                      (* the number it states *)
               | ref                                       (* a third entity: `symmetry` *)
               | IDENT ":" value                           (* a selector *)
               | IDENT "==" expr ;                         (* a pin, §4.3 *)

(* §4.3: a number inside `hint(…)` seeds; `==` inside an argument list pins.  The two are the
   whole of the seed/constraint classification, and they are told apart by the clause alone. *)
arg            = [ IDENT ":" ] expr
               | IDENT "==" expr ;                         (* pin:  a solve may not move it *)

path_decl      = "path" IDENT ":" orient "=" path_expr ;
frag           = path_expr ;                               (* statement-level fragment *)
orient         = "ccw" | "cw" ;
path_expr      = ref seg ref { seg ref } ;
seg            = "->" | "~" ref [ "rev" ] "~" ;

hint           = "hint" ref hint_clause ;                  (* §11; unimplemented *)
gauge          = "ground" ref | "fix" ref ;                 (* §9.2: prefix operators *)
orientation    = orient "(" ref "," ref "," ref ")" ;      (* §9.2: a call *)

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

Parsing note: a statement beginning with an expression is disambiguated by the token following the first `ref`/`expr`: `==` → constraint; `->` or `~` → fragment; otherwise error. `ccw`/`cw` appear both as orientation keywords (after `:` in `path`) and as predicates; context disambiguates. **[0.2]** `curve NAME(` opens a family definition and `curve NAME =` an instance; the token after the name settles which. **[0.18]** `view` and `section` open a statement and are therefore not names; `through` is settled by a one-token lookahead past the first `ref`, since it relates no geometry and has no residual for the operator machinery to settle it into, while `on` needs none — it is an ordinary infix word whose meaning is the kinds of its operands (§9.2). Inside a solid's brackets a sweep label is read before an attitude label, `from` being a word both constructs use and only one of them a plane.

**[0.2] Note on the trailing `==`.** After the `==` that follows a predicate's closing parenthesis, an implementation MAY take the rest of the logical line verbatim rather than tokenizing it, and hand that text to whatever evaluates dimension expressions. This is not laziness: `3 1/2` is three and a half and `31/2` is a division, and that rule belongs to one tokenizer. Two copies of it are two rules the moment one is edited. An `==` *inside* an argument list is the pin of §4.3 and is lexed normally; the two never meet. **[0.7]** A trailing `hint(…)` ends the verbatim region, since it is a clause of the statement and not part of the number.

---

## 20. Conformance checklist for a first implementation

A minimal conforming implementation provides:

1. Parser for §19; classifier assigning every statement to §4.2 classes **by the `=` / `==` mark (§4.3)**; Invariant H enforced by construction of the parser.
2. Elaborator: instance expansion **preserving statement identity (§12.7)**, union-find aliasing, definitional substitution, dedup store, `ring` lowering (quotient form or cycle-plus-symmetry — either, per §12.3, **and reported if unrolled**).
3. Invariance check §12.5 (syntactic criterion), gauge analysis (W103), DOF ledger (§16.3).
4. A numeric backend satisfying §15 — Newton on the quotient system seeded by hints is sufficient — with rank-deficiency reporting attributed to source spans.
5. Path assembly and closed-boundary export (the solved gear outline as a polyline+arc sequence).
6. **[0.2]** Document state attached to its statement, never to a list position or an entity index (§13.1).

Deliberately *not* required for v0: 3D, nested rings, constraint strengths, decomposition planning (P4 is a SHOULD). **[0.2]** Curve families (§6.5) are not required either, but they are no longer deferred: an implementation that wants involute or cycloid geometry has a way to say it, and needs no new entity kind to do so.
