# Solvent: A Declarative Language for Constrained Geometry

**Specification, Draft 0.139 — October 2026**

What changed in each draft, newest first, is the
[language changelog](docs/solvent-changelog.md); a **[0.n]** mark in the text below points at
the entry that introduced or amended a rule.

*"Solvent" is a placeholder name; nothing in this document depends on it.*

---

## 1. Overview

Solvent is a declarative language for describing geometry under constraint. A Solvent program does not construct geometry step by step; it declares a set of entities, relations among them, and structural facts (symmetry, connectivity), and delegates the discovery of coordinates to a solver. The language borrows its module discipline from hardware description languages: designs are built from **components**, component bodies are **unordered**, and connection between components is **aliasing** (net formation), not constraint.

### 1.1 Design principles (normative)

These principles are binding on the rest of the specification. Where a detailed rule appears to conflict with a principle, the principle governs and the conflict is a defect in this document.

**P1 — Connection is aliasing; constraint is explicit.**
Passing an entity as an instance argument makes two names refer to *one* entity **[0.13]** (a port once did too; §7). Aliasing has zero solver cost and cannot be violated. A constraint always relates *distinct* entities and always contributes residual equations (or inequalities) to the system.

**P2 — Component bodies are unordered.**
Statements within a component body form a set. Reordering the statements of a body MUST NOT change the meaning of a program. Statements interact only through the entities they name. The only ordered contexts in the language are the interiors of single statements where order is semantic (path traversals, argument lists).

**[0.2]** P2 binds the *document* as well as the program text: every piece of state a document carries MUST be attached to the statement it qualifies, and MUST NOT be keyed by a statement's position in a body or by an entity's index (§13.1). Without that rule P2 is an assertion an implementation can satisfy in the parser and lose in the file format — and it fails silently, which is the worst way for it to fail.

**P3 — Hints are semantically inert.**
Deleting every seed from a program MUST NOT change its solution set. Seeds may change which solution a solver finds, or whether it converges, but never what counts as a solution. Any annotation that changes the solution set (orientation predicates, arc branch selection, tangency side) is a **constraint** and is classified as such by the implementation, regardless of surface syntax.

**[0.2]** The two are told apart by one mark, not by analysis: a number written with `=` is a seed and a solver may rewrite it (**[0.7]** the mark is now the `hint(…)` clause: a number inside one is a seed, §4.3); a number written with `==` is a constraint and a solver MUST NOT (§4.3). This is what makes Invariant H (§11) checkable *syntactically*, as that section already requires.

**P4 — Component boundaries are decomposition structure.**
A component is solvable against the entities it is written over. Implementations SHOULD exploit the component instance tree as a decomposition plan (solve interiors against the formals; solve the inter-component system over them).

**P5 — Symmetry is a claim, not a macro.**
The `ring` construct asserts cyclic symmetry. Its solution set contains exactly the symmetric solutions of the corresponding unrolled system. Implementations SHOULD solve in the fundamental domain, and one that does not MUST say so (§12.3).

### 1.2 Scope of this draft

**[0.1]** This draft specified **planar (2D) geometry only**. The entity vocabulary, constraint library, and group actions are two-dimensional. Section 17 lists the known lifting questions for 3D. This draft also excludes curve entities beyond lines and circles (no involutes, splines, or conics); see §17. **[0.2]** Curves are settled in §6.5 and splines in §6.1; **[0.42]** points, axes and planes stand in space (§6.7).

**[0.18] More precisely: the draft specifies planar geometry *solved*, with solids evaluated over it.** A document may say what object its drawing is of — a face is a region of a plane (§6.8), a solid is a face swept or a term over other solids (§6.9), and a view or a section of one is a picture the sheet asks for (§6.11) — and **nothing past the sketch is ever an unknown**. **[0.42]** The sketch reaches into space: a point may stand there, and a plane and an axis (§3.1, §6.7) are unknowns of the solve, solved with the rest of the sketch before anything below reads them. An extent is an expression (§6.9), and neither a face nor a solid owns a parameter, appears in a residual, or is reached by a constraint. The strata run one way and there is no edge back: the sketch solves, the extents are worked out, the terms are ordered, the outputs are read. So everything §15 says about a solver, and every count in §16.3's ledger, is unchanged by the presence of an object — which is the whole of what makes the addition affordable.

### 1.3 Conformance keywords

MUST, MUST NOT, SHOULD, SHOULD NOT, and MAY are used as in RFC 2119. Text marked *non-normative* is explanatory.

---

## 2. Lexical structure

- **Identifiers:** `[A-Za-z_][A-Za-z0-9_]*`. Component names are conventionally capitalized; this is not enforced.
- **Keywords:** `component`, `param`, `point`, `circle`, `line`, `frame`, `path`, `repeat`, `cycle`, `ring`, **[0.72]** `when`, `about`, `as`, `next`, `prev`, `hint`, `at`, `fix`, `ccw`, `cw`, `rev`, `true`, `false`, **[0.2]** `curve`, `over`, `spline` (and `ellipse`, until **[0.15]** made the ellipse a library component — `Ellipse` in `std`, a computed point on a datum traced as a curve, whose contacts are the curve's; an implementation keeps the word only to refuse it). **[0.7]** `unit`, `class` and `style` in, `construction` out; every constraint is a prefix or an infix operator (§9.2), so `coincident`, `equal`, `tangent`, `curvature`, `symmetry` and `distance` are the words a statement is written with — it is a class now, and the base sheet is what draws it dashed (§13.2). **[0.4]** In a chain (§6.6) the word `close` is meaningful *contextually*; it is not reserved, and an entity may bear it as a name. **[0.19]** It is read the same way after `->` inside a `face`'s brackets (§6.8), which is the other place the language draws a loop. **[0.8]** `to` is retired: the plain corner is the `->` marker, and threading is stated at the joint rather than inferred from the operands. **[0.5]** A coordinate seed is written `hint at` (§6.4). **[0.7]** Every seed is written in one `hint(…)` clause (§4.3, §6.4); `hint at REF` kept its own form inside a trace block (§6.5.1) until **[0.14]**, when a place became the `at:` and `bearing:` keys of the same clause — `hint(at: REF, bearing: β)` — so `at` after `hint` is refused, and `bearing` is a key and no keyword. **[0.10]** `plane`, `in`, `project` and `fold` in (§6.7); `from` is contextual there as it is in a trace family. **[0.42]** `axis` in; `frame`, `fold`, and `from`, `offset` and `through` as a plane's labels, out — a plane's brackets take `u:` and `v:` and nothing else. **[0.13]** `port` is retired (§7); an implementation keeps the word only to refuse it. **[0.18]** `face` and `solid` are element keywords (§6.8, §6.9) and `view` and `section` open a statement (§6.11); `cut` is the body rule's own word and `on` gains a reading over two solids (§9.2), so both join the operator words a name may not be. **[0.32]** `union` is the body rule's too, and joins them; `on` loses its reading over two solids. **[0.42]** `on` is no constraint word: `coincident` says incidence (§9.2). **[0.20]** `view` and `section` leave model source for the drawing (§6.11). The eight labels a solid's brackets take — `from`, `to`, `depth`, `through`, `along`, `about`, `sweep`, `sense` — are **contextual**: they are read as labels inside the brackets that take them and are reserved nowhere, so a `param` or a point may still bear any of them as a name (`face := -(fw + D / 2)` is idiomatic). A declaration's *name*, however, may not be an element keyword, and three shipped examples renamed a line that had been called `face`. **[0.94]** `meshing` is an element keyword (§6.15.4); one shipped example renamed a motion that had been called it.
- **Literals:** decimal numbers with optional unit suffix (`10`, `2.5mm`, `30deg`). The constant `tau` (= 2π) and `pi` are predefined.
- **Comments:** `//` to end of line; `/* ... */` nesting not required.
- **Operators and punctuation:** `== + - * / ( ) { } [ ] , : . := -> ~ @`
- **Shorthands [0.44]:** `@` is the word `coincident` and `~` the word `hint`, read as those words wherever they may stand.
- Whitespace and newlines are insignificant except as token separators. Statements are newline- or `;`-terminated; implementations MUST accept either.

---

## 3. Types and dimensions

### 3.1 Value types

| Type | Meaning | Free DOF when unconstrained |
|---|---|---|
| `Int` | compile-time integer (parameters, counts, indices) | — (elaboration-time) |
| `Scalar` | dimensionless real | 1 |
| `group` | named bundle of numeric values, geometry aliases, or nested groups (§8.1) | 0 of its own |
| `Length` | real with length dimension | 1 |
| `Angle` | real with angle dimension | 1 |
| `Point` | a position in the plane it is drawn in, or **[0.42]** in space where it is drawn in none | 2 in a plane; 3 in space |
| `Line` | segment between two `Point`s; its infinite carrier is what constraints read **[0.2]** | 0 of its own (4 through its ends) |
| `Circle` | center + radius | 3 |
| `Arc` | center + radius, its start and end on that circle | 5 |
| `Axis` | a directed line in space with no start, drawn in no view **[0.42]** | 2 (its direction), and 2 more once a relation reads where it is |
| `Plane` | **[0.42]** a plane in space: its attitude two axes `u`, `v` through its origin **[0.43]**, and where it stands; a **view**, the points drawn in it having its own coordinates (§6.7) | 3 of its own (where it stands), beyond its axes'; a bare `plane` is 7 with the axes it mints **[0.43]** |
| `Path` | directed piecewise boundary curve | 0 (derived object) |
| `Face` | a planar profile (§6.8) | 0 — it owns no parameter |
| `Solid` | a face swept, or a term over other solids **[0.18]** | 0 — it owns no parameter |
| `Surface` | an analytic boundary patch read from a solid (§6.13) | 0 — it owns no parameter |
| `Motion` | a rigid-motion family about solved geometry (§6.14) | 0 — it owns no parameter |
| `Envelope` | the zero-normal-velocity locus of a moving surface (§6.15) | 0 — it owns no parameter |

**`Face`, `Solid`, `Surface`, `Motion` and `Envelope` are evaluated after the solve.** None owns a coordinate, allocates an unknown, or contributes a planar residual; none may be an argument of a planar constraint of §9.3, be dragged, be dimensioned, or be put on a plane with `in`. What each *is* is settled once the drawing has been solved and every extent worked out (§1.2, §6.9). A kind of this stratum therefore adds nothing to §16.3's ledger.

**[0.2] `Line` was a 2-DOF undirected infinite line with a `.dir` field in 0.1.** It is now a segment between two points, and every constraint that reads a line reads the infinite carrier through those points — which is what `parallel`, `perpendicular`, `coincident` and `angle` mean by a line anyway.

The change is a concession to cost. A 2-DOF line is a second representation of a line alongside the one every constraint already works in, so it needs its own column layout and its own kernel for roughly fourteen constraint types, and the whole return is that §16.3's ledger comes out differently. Where a drawing wants a line with no ends — a datum, an axis — it says so with a construction segment, or **[0.42]** in space with an `axis` (§6.7), which has none.

### 3.2 Sub-entities

Compound entities expose sub-entities by field access. Sub-entities are ordinary entities and participate in aliasing and constraints.

| Entity | Field | Type |
|---|---|---|
| `Circle` | `.center` | `Point` |
| `Circle` | `.r` | `Length` |
| `Arc` | `.center`, `.start`, `.end` | `Point` |
| `Arc` | `.r` | `Length` |
| `Axis` | `.x`, `.y`, `.z` | its unit direction — seeded by `hint(dir: (…, …, …))` (normalised) and held by `fix(dir == (…, …, …))` **[0.42]** **[0.45]** |
| `Axis` | `.px`, `.py`, `.pz` | `Length` — where it stands: its point nearest the world origin, held by `fix(origin == (…, …, …))` once a relation reads it **[0.43]** **[0.45]** |
| `Plane` | `.u`, `.v` | `Axis` — its axes, through its origin **[0.42]**; minted where the slot is not written (`P.u`, `P.v`) **[0.43]** |
| `Plane` | `.origin` | `Point` — drawn in the plane and held at its `(0, 0)` **[0.42]** |
| `Plane` | `.x`, `.y`, `.z` | `Length` — where it stands in space: read by a seed, seeded by `hint(origin: (…, …, …))` and held by `fix(origin == (…, …, …))` **[0.42]** **[0.45]** |
| `Line` | `.p1`, `.p2` | `Point` **[0.2]** |
| `Face` | its edges | aliased boundaries |
| `Solid` | `.near`, `.far` | a prism's two caps, at the higher and the lower ordinate **[0.18]** |
| `Solid` | `.<edge>` | the side swept from the edge the source called `<edge>` **[0.18]** |
| `Solid` | `.start`, `.end` | a partial revolution's two caps **[0.18]** |
| `Solid` | `.volume`, `.area`, `.bounds.x0`… | what a report measures off it (§16.3) **[0.18]** |

**[0.18] A solid's derived names are *paths*, and a path is not a sub-entity.** A face of a solid carries no coordinates, joins no alias class and is no operand of a relation over entities — but **[0.66]** a face path names a **set**: **[0.68]** the face as the solid has it, bounded by its edges, and `.surface` the surface it lies on (§6.21). A prism's flat face's surface is one the solve reads — a cap the profile's plane at its extent, a straight side the plane through its edge along the profile's normal — so `p coincident b0.near.surface` and `v1 tangent(at: v1.origin) b0.near.surface` read it as any set; a point or a plane drawn on the bounded face itself, a curved side's surface, a `through:` solid's caps and a revolution's or a loft's faces are not sets the solve reads yet (**E040**). A region's point reads any face, bounded or by its surface, off the solid after the solve. Otherwise the path is what a report writes a number under (`block.side_l.area`) and what a derived picture labels a stroke with. **[0.100]** A face family (a loft or a sweep along a path) has `.start` and `.end` caps and walls named by its section's edges under the instance, hole boundaries included (`duct.sec.ab`, `duct.sec.inner.loop.ab`). Through a body the operand keeps its own name, so a face of `block` inside `body` is `body.block.near` — a path never renames, which is the naming problem every history-based kernel has and this one does not, because a boolean cannot renumber a name. The *sub-entities* a solid does have are the ones its declaration is written over: the face it is swept from, a revolution's axis line (or axis), and the solids of its term. Those are ordinary entities, and deleting one takes the solid with it.

**[0.42] A plane is two axes and a place.** `P := plane(u: r1, v: r2)` aliases two axes (or
drawn lines, each read as the axis from its `p1` toward its `p2`) and owns three numbers of its
own, where it stands. Its attitude is its axes': right is `u`, out of the plane is `u × v`, toward
its viewer, and up is out × u, so `v` need not be square to `u` — it says which plane, and which
side is up. **[0.43]** The axes pass through its origin, so two planes share an axis only where
both stand on it. No plane has a rotor, an angle or a `toward`: 0.6's datum `plane(origin: o,
toward: q)`, its intrinsic unit rotor `(c, s)` and the derived `.angle` are retired, and so is
0.15's reading of a plane with no attitude written as the page's, and 0.15's `frame`, which names
nothing now.

### 3.3 Dimensional analysis and units **[0.7]**

Implementations **MUST** check dimensions in expressions and constraints, and MUST NOT silently coerce `Angle` to `Scalar` or vice versa. Angle arithmetic is mod 2π where the operand is a bearing and signed where it is a turn; see §9.4. *(SHOULD in 0.1–0.6, and never implemented; MUST from 0.7.)*

**Two base dimensions**, because two is what the language has: a **length** and an **angle**. A quantity is a rational power of each — rational, because `sqrt` halves one and `sqrt(area)` is a length.

**[0.98] Three more for values: a mass, a time and a current.** A machine's sizing lives in the same table as its geometry, and its figures check as its lengths do: `torque := power / omega` is a torque because the arithmetic says so, and `torque + power` is an error. The three are enough to derive every unit of §3.3.1. **No slot is of them** — every slot of §9.3 is a length, an angle or a plain number — so a figure carrying one reaches a constraint only divided by its unit (`torque / 1Nm`), as an angle reaches a length slot only through a function of it. A number of mass, time or current is in kilograms, seconds and amperes, and a length in it in the document's unit (§3.3.2): a watt in a millimetre document is 10⁶ kg·mm²/s³, and `1W / (1kg * (1m)^2 / (1s)^3)` is 1 in every document. A diagnostic names a dimension by its bases' powers and, where an SI unit is that dimension, the unit: `Length^2·Mass·Time^-3 (W)`.

- `*` and `/` **derive**: the exponents add and subtract.
- `+` and `-` **demand agreement**, and so do `min`, `max`, `hypot` and `atan2`'s two arguments.
- **[0.72]** So do a comparison's two sides — `<`, `<=`, `>`, `>=`, `==`, `!=` — which make no number but a condition, read only by `when` (§12.9): `relief > 0mm`, never `relief > 0`.
- `^` takes a plain number, and a **dimensioned base takes a whole power**: `x ^ 2.5` where `x` is a length is not a dimension anybody meant, and `sqrt` is how a half is written.
- The dimension an expression comes to is checked against the **slot** it is written in.

**A bare number is dimensionless, and a *context* may take one.** `a distance(80) b` is a length in the document's unit (§3.3.2) because the slot says so, and `sin(30)` reads 30 as degrees because the function does. A context may **not** speak for a second operand: `90 / N + ivp` is a plain number added to an angle, and an implementation MUST report it rather than choose. The asymmetry is the design — a slot and a function say what they want; two operands are not a context, and neither of them is authoritative.

A **name** is worth a number, and where that number is *used* decides what it is: `w := 80` in a `Length` slot does not make `w` a length, since the same 80 may be a run, a rise or an angle. A unit on the literal (`w := 80mm`) says otherwise, and so does a component formal's declared type (§8) — which is what catches `x := w + phi`.

**[0.16] One namespace for a number's names; [0.41] an unknown is declared.** A number is named one way, `w := 60` — `param w := 60` when it is one of the document's inputs — and a dimension's number reads names and defines none. A name resolves by one rule (§5): a definition declares its name in the body it is written in, and a second definition of a name in one body is **E001**. An **unknown** — a number the solve answers for — is declared too: an input with no value (`param beta: Angle`, §6.3) or a formal no call binds (§8). A name nothing in scope declares is **E101**, wherever it is read; it is never made an unknown of its own accord, which turned a misspelling into a degree of freedom (0.16–0.39 called it a free variable and warned, W111). A value reading an unknown is that unknown scaled (`q := 2 * beta`), carried as the affine form it is.

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

**[0.98]** The units of a design table's figures:

| dimension | units |
|---|---|
| mass | `kg`, `g` |
| time | `s`, `ms` |
| current | `A`, `mA` |
| frequency (Time⁻¹) | `Hz`, `kHz` |
| angular speed (Angle·Time⁻¹) | `rpm` |
| force | `N`, `kN` |
| torque, energy | `Nm`, `J` |
| power | `W`, `kW` |
| voltage | `V`, `mV`, `kV` |
| resistance | `ohm` |
| pressure, stress | `Pa`, `kPa`, `MPa`, `GPa` |
| magnetic flux | `Wb`, `mWb` |
| flux density | `T`, `mT` |

A unit is a suffix only, so `48V` is a voltage and `V` alone is still a name. There is no `h` and no `min`, which are a height and a function long before they are an hour and a minute. `rpm` is an angle over a time, so `255rpm / 1rad` is the angular speed in rad/s and `255rpm / turn` the revolutions a second, `1Hz` beside it.

**Feet-and-inches is one literal**, and it is a rule the language already had: *a space is what tells the readings apart*, exactly as it does in a mixed fraction, where `3 1/2` is three and a half and `31/2` is a division. So `1' 6"` is one length for the same reason.

**The language therefore has no string literal.** A `"` is the inch mark and there is nothing else for one to be; every `Str` argument is written as the word it is (`at: start`), and a raw branch key is written bare (§13.1).

#### 3.3.2 `unit`

```
unit mm
```

`unit` names the document's **length** unit. A bare number in a `Length` slot is that unit, so every existing document keeps working with one added line, and a suffixed literal converts to it.

**[0.93] Without a `unit` line the document is in millimetres**, exactly as if it began `unit mm`: a bare number in a `Length` slot is a millimetre, a suffixed literal converts to millimetres (`a distance(2in) b` is 50.8), and a geometry read is a `Length`. Everything checks as under any unit: `a distance(45deg) b` is an error, and so is `Length + Angle`. What the line adds is only that it is *stated*: an implementation that writes a document back (a printer, a saved file) writes a `unit` line only where the document had one. Through 0.92 a document with no `unit` line was in unnamed *drawing units*, where a suffix was refused and a geometry read was a bare number.

**Storing the document's unit costs an implementation nothing for lengths**, because a well-formed kernel is homogeneous in length: scale every length in a sketch by a constant and no residual, no tolerance and no rank moves. **Angles are the exception, and it is not a choice**: `cos θ` is not homogeneous, and there is no consistent unit it works in other than radians. So an angle is stored in radians and converted at the text seam, which is exactly where it converts anyway; what units remove is not the conversion but the *guess*.

Where a document may be copied into another, a paste between documents in different units **SHOULD convert**: a figure is the same figure in either, and two inches is 50.8 millimetres.

#### 3.3.3 `pi` and `tau`

`pi` is the dimensionless mathematical constant. `tau` and `turn` are a full **turn**, which is an `Angle`. They were 3.14159 and 360 side by side with nothing saying why; units settle it, and `tau == 2 * pi * 1rad` now holds dimensionally where it used to be a coincidence of digits.

A conversion written out — `* 180 / pi` — is `* 1rad`, and the `1rad` that remains is not noise: it is the fact that `inv φ = tan φ − φ` **holds only in radians**, which the formula never said and which the check now makes it say.

---

## 4. Program structure

A program is a set of component definitions. One component is designated the **root** (by tool invocation, not by language syntax); the elaborated model is the root's instance tree.

The file tools use the file's top-level statements as an anonymous root. A file containing only
component definitions has an empty root: opening it does not implicitly instantiate its last
definition. An instance statement at file scope supplies the arguments and draws that component.

**[0.21] An instance name is optional.** `Part(args)` contributes the same statements as
`part := Part(args)`, with an independent expansion for each call. Use the named form when
constraints or drawing files need to refer to its members. An unnamed instance's internal keys
MUST NOT be exposed as writable source names. Operator calls such as `ccw(a, b, c)` retain
their existing meaning.


```
component Name(p1: Type1, p2: Type2, ...) {
    <statements>
}
```

### 4.1 Parameters

Parameters are passed by name or position at instantiation. A parameter of entity type (`Point`, `Circle`, `Line`, `Plane`, **[0.42]** `Axis`) is **bound by aliasing** (P1): the formal name and the actual argument denote the same entity. A parameter of value type (`Int`, `Scalar`, `Length`, `Angle`, **[0.17]** `Side`) is a compile-time or definitional value; it contributes no unknowns. A `group` formal bundles values and geometry references (§8.1); **[0.84]** one typed by an interface (`gear: Gear`) names the members its argument must have and its body may read (§8.1). **[0.79]** A `path` formal takes a line, an arc, a circle, a spline or a curve — **[0.122]** or a curve traced in space (§6.5), never a surface — bound by aliasing as an entity is; the words its body writes of it are settled by the kind of the actual, and an actual that runs along nothing is **E103** at the argument. **[0.71]** A `set` formal takes a set (§6.21): a named one (`S := { p | … }`) or an instance of a set family (`std.Sphere(c, r: 5mm)`), which is the set — written by name, since an argument is a reference, not a call; a set's surface (`S.surface`) is not one yet. It is given by position, as an entity is, and used in the body by its name wherever a set may stand (`p coincident s`, `p inside s`, `distance(p, s)`). A named set given to a `group` formal and a group given to a `set` formal are each **E103** at the argument; a family's instance is both a set and an instance, so it fills either. A `Side` is one of the words `left` and `right` (§9.2) and is not a number: it may be passed on to another instance and written in a selector, and nothing else.

**[0.17] Which way is a word, not a sign.** A distance measured **from a line** — a point to a line, a line to a line — is a **magnitude**: its solution set is *both* sides, and which one a solver finds is the seed's business (P3), as it is in every other sketcher. A negative one is an error (**E040**) wherever the number comes from, including a component's argument, since the kernel cannot tell one side from the other and the minus therefore said nothing a drawing could show. A statement that must pin a side writes the word — `p distance(12, side: left) ax`, left being of the line's own `p1 → p2` — and so does a tangency (`side: left | right`, which was `side: -1`). Where a sign is *arithmetic* rather than a convention — the run and the rise, signed from the first point to the second, and the directed angle of §9.4 — it stays a sign, because a component computes it from coordinates it is given; each gains the word that says the same thing in the open (`along: right | left | up | down`, `sense: cw | ccw`), and a document SHOULD prefer it. A component takes a side as a value of the type **`Side`** (`s: Side`, `side: s`), which is a word and not a number: encoded as ±1 it would put the unreadable idiom back one level down, inside every helper.

**Labels are mandatory for numbers and sides.** A positional argument MUST bind an entity,
group or set formal and MUST precede every labelled argument; either violation is **E004**.
An argument MUST NOT be supplied twice. For example, `Frame(layout, dims: vtwin_dims)` passes
a layout positionally and a dimension group by label, while `Cylinder(axes, fw: 12mm,
dims: vtwin_dims)` labels the numeric face-wall thickness.

### 4.2 Statement classes

Every statement belongs to exactly one class. The classification is normative because P3 depends on it.

| Class | Statements | Affects solution set? |
|---|---|---|
| **Declaration** | entity declarations, `param`, groups (`{…}`, §8.1), curves (§6.5), instance declarations, **[0.18]** the body rule (`union`, `cut` and **[0.85]** `common` over two solids, §6.9) **[0.32]** | introduces entities/aliases |
| **Constraint** | relations (prefix and infix words, §9.2), `==` equations, **pinned seeds (`==`, §4.3) [0.2]**, orientation predicates, arc-branch and tangency-side decorations, `ring` symmetry, gauge statements | **yes** |
| **Seed** *(was Hint)* | the `hint` statement (§11), **and every seed written inline in a `hint(…)` clause (§4.3, §6.4) [0.2] [0.7]** | **no (P3)** |
| **Structure** | `repeat`/`cycle` blocks, `path` declarations (net of their derived constraints, §10.4), **[0.18]** `view` and `section` (§6.11), which ask for a picture and declare nothing — **[0.20]** in a `.svd` drawing, never in model source | organizational |

### 4.3 Seeds and pins: `hint(…)` and `==` **[0.2]** **[0.7]**

> **A number inside a `hint(…)` clause is a seed. Every other number is not.**

This is the only distinction between the two classes, and it is lexical on purpose. §11 requires an implementation to verify Invariant H *syntactically*; a mark you can see does that, and no analysis of what a number "is really doing" does.

```
p := point hint((0, 0))          // seed: a solve may move it
c := circle(center: o) hint(r: 25)   // seed: a solve may move the radius
a distance(80) b                     // constraint: a solve must not move it
p coincident s hint(t: 0.37)         // seed: the contact may slide along the curve
p coincident(t == 0.37) s            // constraint: the contact is pinned there
w := 100                             // neither: a number worked out while elaborating
```

**[0.7] One clause, where 0.2–0.6 had three spellings.** A seed used to be written three ways depending only on what happened to carry it — `hint at (0, 0)` for a point's coordinates, a labelled `r: 25` inside the constructor for any other scalar, `t = 0.37` for a constraint's own unknown. Three spellings for one class of thing, and the middle one was the worst: it put a number the solver will move inside the same brackets as the structure it may not, so `circle c(center: o, r: 25)` read as though the radius were as much a part of what the circle *is* as its centre.

`hint(key: value, …)` says it once, keys in any order, on declarations and on constraints alike. **The brackets after the name are what the thing is made of; the `hint(…)` after them is where the solve begins.** It also mends the headline of this section, which was only ever approximately true: `r: 25` was a seed written with `:`, and `w := 100` is not a seed at all.

The three retired spellings — `point p at (0, 0)`, `point p hint at (0, 0)`, `circle c(center: o, r: 25)` — MUST NOT parse. A document written against 0.2–0.6 does not load; the change is small, mechanical and worth doing once.

The last pair is why the rule is needed at all. A curve contact carries its own parameter, and
whether that parameter is free is not a fact about how it looks — it is a fact about the solution
set. A curve fitted through *m* points whose contacts are unpinned keeps *m* degrees of freedom and
can slide along itself; pin them and it does not. In 0.1 both forms would have been written the
same way and classified by intent.

**Consequences, all normative.**

1. A solver MUST NOT rewrite a `==` number. It MAY rewrite a number inside a `hint(…)` clause, and doing so is how a
   drawing records where it ended up (§13.1).
2. A statement's class is decidable by inspection. An implementation MUST NOT need to consult the
   solution set, the solver, or the geometry to classify one.
3. Deleting every `hint(…)` clause from a program leaves its solution set unchanged (P3). Deleting a
   `==` number generally does not.

**Non-normative.** The mark also settles seed *writeback*, which is what an implementation needs
when the drawing is edited by drawing on it rather than by typing:

> A seed is writable iff it is inside a `hint(…)` clause, is a literal and not an expression,
> and is reached by exactly one instance path.

The first clause is this section. The second keeps a radius written as a component's parameter
(`hint(r: Rr)`) from being overwritten with the number it happened to come to — the author said what it
*is*, not where it starts. The third is §12.7: thirty instances share one statement, and there is
no one pose to record.

---

## 5. Names, scope, and resolution

- The scope of a name is the entire component body in which it is declared (P2). Forward reference is legal and idiomatic.
- **[0.29] A name is defined by `NAME := VALUE` and in no other way** (formals and block binders aside). The value decides what the name is: a number (a value, or under `param` an input, §6.3), an element (§6), a chain joined by `->` (§6.6), an instance (§8), a group (§8.1), a curve (§6.5). `(NAME := VALUE)` is the value, so it may stand where the value may: a chain link, `(ab := line(a, b)) -> …`. **[0.41]** A constraint's number defines no name. `:=` binds loosest. An input with no value, `param beta: Angle`, declares an unknown (§3.3, §6.3). A `label:` fills a slot and defines nothing in the caller's scope.
- Redeclaration of a name within one body is an error (**E001**), whether the two are values, inputs or one of each.
- Instance members are accessed by dotted paths: `t.lead`, `g.hub.origin` — an unknown an instance left unbound included: `t.w` (**[0.41]** a dotted name nothing made is **E101**, as a bare one is).
- **Components have closed model scope.** A body reads its formals and its own declarations. It may call visible component definitions and use built-in functions and constants, but cannot capture geometry, parameters or groups from the root, an importing module, or an enclosing component. Pass those dependencies as arguments. This includes `std.front` and the other standard datums.
- **[0.41]** An undeclared numeric name is **E101**. A component's unknowns are its numeric formals a call leaves unbound, each the instance's own (`t1.w`, `t2.w`); a call may seed one by leaving it unbound with `hint(…)` in its place (`T(a, b, w: hint(60))`). **[0.95]** Only a number may be left out: a call that gives no entity, path, group or set for a formal of that type is **E103** at the call, a set family's instance included, which would otherwise say nothing until something was put on it — but for **a point formal drawn in a plane formal**, `center: point in on`, which the instance then owns: a call that leaves it out makes a point of the instance's own, drawn in the plane it gave `on` (whatever plane the instance stands in), named under the instance (`k.center`) and read and held by that name, as if the body began `center := point in on`; `center: hint((x, y))` seeds it at its place in that plane, and a solve writes it back there. Given a point, the formal is that point, as any entity formal is. So `component Circle(on: plane, center: point in on, r: Length) := { p | p coincident on; p distance(r) center }` is called `Circle(std.front, o, r: 25)` or `Circle(std.front, r: 25)`, and both are the circle. `in` names a `plane` formal of the same component (**E103** at the call otherwise) and follows only a `point` formal. A reference to a known ambient value without an argument is an error, rather than an implicit capture. Inside a repetition, a name declared by the block belongs to each copy; other names resolve in the enclosing component. `next` and `prev` belong to the lexical cycle, and can be passed to a nested component as arguments.
- Inside `repeat`/`cycle`/`ring` blocks, the index binder (`as i`) and the pseudo-instances `next` / `prev` are in scope (§12).
- There is no shadowing: a block binder that collides with an outer name is an error (**E002**).
- **[0.17] A name that shadows a built-in is said.** The constants and functions of §3.3 (`pi`, `tau`, `turn`, `sin`, `min`, …) are known to every expression before the document is, so a `param`, a formal or a block binder of one of those names does *not* shadow it: a text carrying the name is substituted and reads the declaration, a number worked out reads the built-in, and the two answers differ silently — `tau := 35deg` passed to a `tau: Angle` formal arrived as a full turn. Each declaration — a value, an input, a formal, a block binder — is a warning at the declaration (**W112**), because the drawing is not wrong, the name is. An implementation MUST say it once per declaration, whether or not the component is ever instantiated.

---

## 6. Entity declarations

### 6.1 Free declarations

```
p := point
l := line                      // makes two points: l.p1, l.p2
c := circle                    // makes one:        c.center
a := arc                       // makes three:      a.center, a.start, a.end
```

A bare declaration introduces an entity all of whose coordinates are unknowns.

**[0.7] The children come with it.** A declaration that writes no argument list gets the ones its kind is built from, *unnamed*, and they are reached by dot access: `l.p1` is an ordinary point, and it constrains, drags and is picked like any other. Half the points in a drawing are named only because a line has to be written from something; a name earns its place when something says it twice.

**The dotted path is the name.** An anonymous child has no name in the source, so `l.p1` *is* its name, and everything that identifies an entity by name MUST agree — a constraint written against it, a selection that outlives a re-elaboration, a diagnostic that reports it.

**[0.9] The name itself is optional**, independently of everything after it: `line` alone is a syntactically valid statement — a line with no name, implicit children and no hint — and so are `line(p1, p2)`, `circle hint(r: 25)` and `arc(center: c)` — a line owns no scalar of its own, so its anonymous seeded form puts the seeds in the slots, `line(hint((0, 0)), hint((60, 20)))` (§6.2). The token after the kind keyword decides what the statement says next, so a trailing-clause word (`hint`, `knots`, `class`, `at`, `close`) can no longer be a declaration's name — the same reservation element keywords and operator words carry. A curve keeps requiring a name: its form is `NAME := REF over u in (a, b)` (§6.5), and the name is what the contact constraints address. **Identity is minted on demand.** Internally the statement suffices — an anonymous element parses, elaborates, draws, drags and deletes without ever being named. The moment the *source* must reference it (a constraint applied from a tool, a dimension stated on it), the implementation MUST splice a real name into the declaration — the same bargain §6.4's writeback strikes with an unwritten `hint(…)` clause. No hidden names: a name is what the source calls the thing, and an unnamed thing has none until the source needs one. Chains are in scope (§6.6): `line -> tangent arc -> tangent line` is a fully anonymous open contour, the corner-minting rule naming shared points by the parent's dotted path.

Where an unnamed and unseeded coordinate *starts* is not the language's business (§15). It is worth saying only that the obvious answer is wrong: two implicit endpoints both at the origin is a zero-length line, with no direction for `horizontal l` to bite on and a singular row for any tangency, so an implementation MUST NOT place them coincidently.

A **list** child slot — a spline's control polygon, a curve's arguments — has no arity to conjure children from, so a bare `s := spline` remains **E103**.

**[0.111] A spline through a chain's corners.** `rail := spline(through: rims, start: l, end: m)` is the cubic through the corners of the named open chain of lines `rims` (§6.6), in walk order — the first link's `p1`, then each link's `p2` — **one span a link**, its knots clamped and uniform, so its parameter `t` is `k` at the chain's corner `k`. Its ends are held by tangents: at the first corner along `start:` and at the last along `end:`, each a line or an axis, the curve bent there no more than the tangent asks (its second derivative square to it); an end with neither runs straight out (its second derivative nothing, the natural end). It is short for an ordinary spline and ordinary statements, all the declaration's: its control points minted under it, `rail.k1` … `rail.kn` between the first corner and the last (`n + 2` for `n` corners, the corners' points its first and last), each corner between the ends `coincident(t == k) rail`, and at each end the control point beside the corner a third of the way (over one link, half) from it toward the next control point, or, given a tangent, toward that point's foot on the line through the corner along the tangent — the foot and the lines to it private construction under the spline. The chain is read wherever it is written (P2). `through:` beside control points, `start:` or `end:` with no `through:`, are refused where written; a closed chain, or anything not a named chain of lines, is **E080**/**E103** at the reference. The spline is drawn in a plane like any (§6.7), so are its chain's corners.

**[0.53] A free curve** **[0.55]** is written with the two points it runs between, `rope := curve(a, b)`: the curve runs from `a` to `b` and its shape is the drawing's to find — what an energy over it (§9.10) settles, the solution of that energy's Euler–Lagrange equation. It has no control points: how the implementation solves the equation is its own business (§15), so long as the answer is the curve the statements describe to the accuracy it reports. It owns one number, its **length**, held by `length(L) rope`, or **[0.57]** solved for where the rest of the drawing determines it, or else where the energy is stationary in it. It is drawn in the plane its ends are in (**E060** for ends in space). A free curve no energy is stated over has no shape (**E040**). **[0.55]** `spline(a, b)` is a spline of two control points, refused as too short.

### 6.2 Constructor declarations

```
pitch := circle(center)
f0 := plane(u: spoke, v: std.y)   // [0.42]: two axes, the line `spoke` read as one
l := line(hint((0, 0)), hint((60, 20)))   // [0.7]
```

Constructor arguments have two behaviors, by type:

- An argument of **entity type** *aliases* the corresponding sub-entity (P1). `pitch := circle(center)` makes `pitch.center` and `center` one entity.
- **[0.7]** A child slot may hold a **`hint(…)`** instead of a reference: an anonymous point, and where its solve begins. It is the same clause as §4.3's, standing in for a child rather than qualifying the declaration it follows, so one construct means "where this begins" wherever it appears.

**[0.8] Every slot is a name, a seed, or implicit.** A written slot carries a reference or a `hint(…)`; a slot the list leaves out — by a label naming a later one, or by a chain's marker filling only the ends it speaks for (§6.6) — is an **implicit child**, minted exactly as a declaration that writes no list at all mints them, and reached by its dotted path. There is no bare `hint` meaning "anonymous and unseeded", because an empty slot already says it. (Through 0.7 a partial list was **E103** — "all the children, or none" — a rule the `->` marker retired: `(l1 := line) -> (l2 := line)` fills one slot of `l2` and leaves the rest the drawing's own. E103 remains the refusal for *more* children than the kind has slots.)

**[0.2] The value-type rule is reversed from 0.1**, which made a value argument definitional — `pitch := circle(center, R)` introducing `pitch.r == R` as a substitution. That cannot stand beside §4.3, and it is wrong on its own terms: a radius is a coordinate a user drags. Under 0.1's reading, taking hold of a rim and pulling would be an *edit of what the program means* rather than a move within its solution set, and every direct-manipulation gesture on a scalar would be a different kind of event from the same gesture on a point. **[0.7]** A value argument is no longer written there at all; it is §4.3's `hint(…)`.

A computed point, `p := point(x: XEXPR, y: YEXPR)`, is wholly defined by expressions and is drawn only as a curve (§6.5).

A radius that is meant to be *held* says so:

```
c := circle(center: o) hint(r: 25)   // 25 is where it starts
radius(25) c                      // 25 is what it is
fix(r == 25) c                    // 25 is what it is, without a dimension on the drawing
```

### 6.3 Values and inputs: `NAME := EXPR`, `param NAME` **[0.29]** **[0.41]**

```
R := m * N / 2                      // a value
param bore: Length := 50mm          // an input: a value a host may give another
param beta: Angle hint(30deg)       // an input nothing binds: an unknown of the solve
```

A definition whose value is a number introduces a named definitional value. Values are evaluated at elaboration time when all inputs are `Int`/literal, otherwise they are definitional scalars — affine in an unknown when they read one (`q := 2 * beta`).

**[0.41]** `param` before a definition marks one of the document's **inputs**: the numbers it is drawn from, and the one kind a host may give another value. Its type (`Length`, `Angle`, `Scalar`, `Int`) is optional where it has a value, which is then checked against it and carries it (`param w: Length := 60` is a length); a mismatch is **E103**. An input with no value is an **unknown** of the drawing, as a component's unbound formal is: it MUST state its type, which is `Length`, `Angle` or `Scalar` (a count is never an unknown) — **E040** otherwise — and its seed is its own `hint(E)`, the one keyless hint clause, which a solve writes back as it writes a point's. A seed on an input with a value is **E040**. `param` stands at the top of a document or its `preview` (a component's inputs are its formals; a block's copies share the document's — both refused where written), and a module's input MUST have a value (**E040**): a module's numbers are read by documents that do not draw it. A value reads an input as it reads any value.

**[0.139] A parameter that runs.** `param t: Angle over (A, B)` declares a **parameter**: a number that runs over the interval from `A` to `B`, neither a value nor an unknown. It MUST state its type, `Length`, `Angle` or `Scalar`, and MUST NOT have a value (`:=`) or a seed (`hint(…)`) — each a syntax error. `A` and `B` are numbers of that type, read over the values in scope as a value is, finite and not reading an unknown, and `A < B` (**E103** otherwise). It stands where an input stands (a module's is **E040**). A parameter contributes no unknown, and in this draft nothing reads one: a dimension reading it is **E103**, and the relation is not emitted. It is the one notion of which a face family's running number (§6.9), the formals a traced body runs over (§6.5) and the interval of a motion's roll a reading states (§6.14, §6.15) are uses — each as it was, its meaning unchanged.

A value or an input is visible throughout the body that declares it and its repetition blocks. A file may read a used module's top-level parameters and groups (§14.4) in its root body, by the module's path (`engine.dims.bore`); a component receives external values through arguments (§8). A value MUST NOT read geometry (`a.x`): a value feeds constraints, and a number read off a seed would make the solution set depend on where a solve began (P3); a *seed* may (§6.4).

### 6.4 Seeds written inline **[0.2]**

A declaration MAY carry the starting values of its own scalars, in a trailing `hint(…)` clause:

```
p := point  hint((0, 0))
p := point  hint(y: 12)                     // an omitted scalar is 0
q := point  hint((0, 12, 5))         // [0.42] a point drawn in no plane stands in space
t := point                                  // no clause at all
c := circle(center: o) hint(r: 25)
```

These are seed-class (§4.2, §4.3) and semantically inert (P3). They are the primitive form; §11's `hint` statement remains, for the case it is actually good at.

A driving radius dimension already records a circle or arc's radius. Editor writeback does not
add an omitted `hint(r: …)` in that case; `rim := radius(size) circle(center: o)` is sufficient.
Authored hints remain editable, and free radii retain pose hints. A claimed radius or a radius
expressed in terms of a free unknown does not determine that scalar for this purpose.

**[0.7] One clause for every seed.** `hint(…)` joins the trailing-clause loop, so it is order-free against `knots` exactly as those two already are against each other. A constraint's own unknown is written the same way — `p coincident s hint(t: 0.4)` — and the *pin* stays in the argument list, `p coincident(t == 0.4) s`: `hint` marks what a solve revises, and a pin is precisely what it does not.

Two things that look like seeds and are not, and stay where they are. **`knots [...]`** is document data no solve moves. **A curve instance's values** — `e := Involute(base, phase: 0).p over u in (u0, u1)` — are numbers the family takes, not numbers the solve revises.

**What `hint` marks is that a solve revises the number — not that the number is seed-class.** The two are not the same set, and the difference decides where the word belongs. Seed-class is §4.3's classification: inert under P3, so deleting it changes no solution set. That is true of a coordinate seed *and* of a callout placement (§13.1), which **[0.20]** lives in the `.svd` drawing (§13.1) and needs no `hint` even though it is every bit as inert. What separates them is who writes them. A coordinate seed is an input a solve overwrites, every time, which is the whole of what §6.4's writeback does. A placement is never touched by a solve at all: it is derived by the layout until somebody drags the callout, and from then on it records where that person put it. So a placement is not a guess about anything, and `at` there says what it means.

*Non-normative:* an implementation can therefore read `hint` as the mark of "this is the solver's to answer", which is a narrower and more useful claim than "this is inert". A reader wanting to know what may be deleted without changing the drawing should ask §4.3, which answers for both.

**Why inline is the primitive.** A seed's job is to say where a coordinate starts, and the place a reader looks for that is the declaration of the thing that has the coordinate. It matters more than taste once a drawing is edited by drawing on it: a solve that wants to record where a point ended up rewrites six characters of a declaration that already exists, where under 0.1 it would have to locate that point's `hint` statement among the body's statements, or synthesise one and decide where to put it. The first is a splice; the second is a program transformation, and it is performed on every drag.

`hint` keeps the cases inline cannot express — seeding an entity declared elsewhere, and seeding from an expression over other geometry (`hint t.lead(x: center.x + root.r, y: center.y)`).

**[0.12] A seed may read geometry, and reads its seed.** The text in a `hint(…)` clause — a declaration's or a child slot's (`l := line(hint((p.x + 10, p.y)), …)`) — MAY name another entity's scalar by its dotted path: `p.x`, `p.y`, `k.center.x`, `k.r`, `e.b`. What it reads is that scalar's **own seed**, never a solved value, so the clause stays seed-class: delete it and the solution set is unchanged, as §4.3 requires. The same clause names a place outright **[0.14]**: `hint(at: REF)` — where another point starts — and `hint(at: K, bearing: β)`, the point on circle `K`'s edge at bearing `β` from the x axis of the plane the seeded point is drawn in. **[0.33]** A place may also be a **step** from the point `at:` names: `hint(at: A, toward: B, by: f, turn: θ)` is `A + f·R(θ)(B − A)`, the fraction `f` of the way to point `B` (1 if `by` is unsaid) turned `θ` about `A` (0 if unsaid); `hint(at: A, along: L, by: f, turn: θ)` steps by `f` times line `L`'s run from its `p1` to its `p2` instead; **[0.102]** along any other path — an arc, a circle, a spline, a traced curve — by `f` times its rate along its own run where `A` meets it (at `A`'s foot on it, held within the run): an arc's by its turn, a circle's by a whole turn, counter-clockwise, a spline's and a curve's by its parameter. So `hint(at: a, toward: b, by: 0.5)` is the midpoint, `by: -1` the reflection of `b` through `a`, and `hint(at: o, toward: rim, turn: 90deg)` the rim turned a quarter about `o`. **[0.77]** For a point in space a step with no turn is taken in space, so it may run along a line square to the plane its `at:` point is drawn in. `toward` and `along` are one or the other, neither stands beside `bearing`, and `by`, `turn`, `toward`, `along` and `bearing` each need `at` — each refused at its key. **[0.33]** A place drawn in another view than the point it seeds (§6.7) is read **where it stands in space**, projected into the seeded point's view: a point of the pitch plane named in another view starts at its image there. An implementation settles such seeds once the memberships and the views' seeded poses are known, again in statement order. An arc's radius left unwritten is its centre to its start, read once the places are settled. `at`, `bearing`, `toward`, `along`, `by` and `turn` are keys beside `x`, `y`, `r` and `t`, and the rule of §4.3 is then lexical with no exception: a seed is what is inside `hint(…)`. A clause naming a place carries no coordinate (`hint(at: p, x: 3)` is an error at the key) **[0.42]** except a plane's: `hint(at: P, (3, 4))` is the place `(3, 4)` in plane `P`'s own coordinates, read in space and seen in the seeded point's plane, which is how a part seeds a point against the plane it is measured in. `bearing` without `at` is an error, and both are refused where they are written, as an unknown key is. (In 0.7–0.13 the place had a grammar of its own, `hint at REF [bearing (β)]`, which MUST NOT parse now; an implementation SHOULD say what the spelling became.) Both forms were a trace block's words (§6.5) and mean the same thing on the sheet. Seeds are settled once every declaration has one, in statement order, so a seed reading a seed that was itself read from a third comes out right when the three are written in the order they depend on; written the other way round it reads the earlier one's provisional seed (an unseeded point's scatter, an unwritten radius's default), which an implementation MAY warn about and MUST NOT refuse. A read of a scalar the entity has not (`l.r` of a line), or of nothing (`nobody.x`), is **E103**. A geometry read is a `Length` (so it adds to `150mm` and not to `10`), in every document, since every document is in a unit (§3.3.2, **[0.93]**). Inside a component the names resolve as references do — a formal reads as the entity it aliases, a name inside a block's copy as that copy's. A seed written this way is an expression and is never written back by a solve. **[0.81]** The same is true of a **direction**: an axis a plane's slot mints (§6.7) is seeded `hint(dir: REF)` along an axis's own seed (`hint(dir: up)`), a line's from its `p1` to its `p2` where each stands in space (a line drawn in a view is read through that view's pose), or a plane's normal (`hint(dir: P.n)`), and `turn: θ, about: REF` turns it `θ` about a second such direction, counter-clockwise looking down it (both or neither, each refused at its key; a clause naming a direction carries no number but its turn). `plane(u: hint(dir: up, turn: -90deg, about: g), v: hint(dir: up))` is the plane square to `g` with its `v` along `up`, wherever `g` runs: a component standing a plane against its arguments starts it where it will stand. A direction read through a plane the document places is read again once that plane is placed, as a place is; reading something with no direction (a point) is **E103**.

### 6.5 Curves **[0.11]**

A curve is **a point of a component, as one of the component's numeric formals runs over an interval**:

```
NAME := REF over FORMAL in ( A, B )                    an instance's point
NAME := Component(ARGS).REF over FORMAL in ( A, B )    an instance written in place
```

`REF` names a point the component places — a declaration of its body or a nested instance's; a formal the component is written over does not move with the swept formal and is refused (**E103**). `FORMAL` is a numeric formal of that component, `Angle` or `Length` (**E040** otherwise); the interval's ends are expressions over the parameters in scope. A curve declares an entity and takes contacts like any other curve, each owning the curve's parameter — spelled `t`, as a spline's is, whatever the swept formal is called **[0.15]** (0.2–0.14 spelled it `u`, which an implementation SHOULD name when refusing the old key): `p coincident e hint(t: …)` says `p − C(t) = 0`, two residuals and one new unknown; `e tangent l` holds the line through `C(u)` along `C'(u)`, two residuals against the one unknown; `e curvature k` makes the circle the curve's osculating circle at `u`, three residuals against it. **[0.39]** Each contact owns its own parameter unless it is pinned to a name nothing defines (`e tangent(t == s) l`, `e curvature(t == s) k`, §9.2): then every contact on the curve pinned to that name owns the one unknown, so a line and a circle touching the curve *at the same place* are five residuals against one parameter. Stated instead as two contacts tied through other geometry — the circle tangent to the line — the condition holds to third order in the distance between their two parameters, and the solve is a degenerate root. A tangency needs `C'` and its derivatives in the geometry — second order — and a curvature `C''` and `C'''`: a computed point supplies them exactly from its expressions, and a locus supplies `C'` exactly (the implicit function theorem) and `C''`, `C'''` exactly as the implicit function's Taylor orders — each one more linear solve with the same Jacobian of the body, its rows read in truncated Taylor arithmetic — with their derivatives in the geometry by difference **[0.37]**. A body is read that way only through relations whose kernels an implementation has written in Taylor arithmetic; a curvature stated against a locus with any other relation in its body is an error (**E103**) naming the relation, since a residual by difference would solve to a slightly wrong circle and call it right. There is no separate curve family: 0.2's `curve NAME(FORMALS)(PARAM) = …` and 0.3's `trace POINT where { … }` are retired, and an implementation MUST refuse them with a message naming this form.

**Two ways a component places the point.** A **computed** point, `p := point(x: XEXPR, y: YEXPR)` **[0.13]** (`port p = …` in 0.12), gives the coordinates as expressions over the formals and the params; a component with one is drawn only as a curve, and an instance of it on the sheet is an error (**E103**), since nothing on the sheet holds a point to a formula. Any **other** point is placed by the body's statements — the locus form: `C(u)` is where the constraints put the point, given the formal's value and the geometry the component is written over. Traced, the body MUST determine its own coordinates — as many equations as coordinates of its own — or the curve is an error (**E103**): an under- or over-constrained locus is a curve that does not exist, and it must not elaborate quietly. **[0.119]** A body that states a rate (below) counts its rate rows instead. Drawn, the same component may be closed from outside like any other.

```
component Unwind(c: circle, datum: line, phase: Angle, u: Angle) {
  t := point
  p := point
  rad := line(c.center, t)
  s := line(t, p)
  t coincident c                                       // the string leaves the circle...
  datum angle(u + phase) rad                   // ...at bearing u — directed, so this side
  rad perpendicular s                          // perpendicular to the radius there,
  p distance(-(c.r * u / 1rad)) rad            // and taut: let out == arc unwound
}
e := Unwind(base, datum, phase: a0).p over u in (u0, u1)
```

**Over a drawn instance.** `path := leg.toe over theta in (0, 360)` names a point of an instance the drawing holds. The trace is **anchored at the drawing**: the pose the instance stands in on the sheet is where evaluation begins, and the value the instance gave the swept formal is the anchor's parameter. An instance that leaves a numeric formal **unbound** makes it an unknown of the drawing (§3.3), named under the instance (`leg.theta`) so two instances leaving the same formal unbound have two unknowns — and the anchor then follows that unknown wherever the solve puts it. This is the form a mechanism is written in: drawn once with its crank free, and traced from the same statements. **[0.37]** Every other numeric formal a drawn instance leaves unbound is an unknown of the drawing in the same way (`leg.h`), and the curve is written over it as over the entities' coordinates: it is a **column** of the curve, so a contact against the curve may solve for it — a rod's length chosen so the stride touches the ground. Only the unknown itself: a formal given an expression in an unknown (`h: 2 * k`, inside an instance that left `k` unbound) is **E103**, and so is an unknown nothing on the sheet reads, which no solve could allocate.

**Closed curves [0.37].** A curve run over a whole turn of an `Angle` formal (`over theta in (0, 360)`) that comes back where it started — a crank's coupler curve, a cam's profile — is **closed**. A contact on it has no end to stop at: a parameter carried past the seam wraps round onto the interval and stays free, since the seam is only where the interval was written to begin. A curve that does not come back (an involute unwound a whole turn) is open, and a contact on it is held to its ends.

**Over an instance written in place.** `Involute(base, phase: a0).p` binds the arguments as an instance statement would and draws nothing: the curve is the only thing made of it. The anchor is the value it gives the swept formal, or the interval's start when it gives none. **[0.42]** A traced component is solved in one plane, so a relation in space in its body (§9.2) is refused where it is written.

**In space [0.107].** A body whose relations do not read in one plane is solved in space: its points are points in space, its words between them the relations in space (§9.2), and a point a motion carries, `X coincident(under: m, t == −u) x` (§9.3), one of its rows — the motion then given to the component as a formal of its own (`m: motion`). Such a curve is **a curve in space**: written in place, over points, lines, motions and numbers (and **[0.133]** curves, as a marched body's Cauchy data, below), it declares no curve of a plane but an entity held to in space. `p coincident k` holds a point in space to it (three rows against its place, net two) and `k tangent(at: p) l` a line along its tangent there (two rows). A body given a motion is solved in space whatever its relations; any other is a plane's where it reads as one (the `[0.42]` rule above), and in space only where it does not. Its derivatives are exact — every order the contacts read, along the formal and in every coordinate it is written over, by the Taylor series of the implicit function along any direction and polarisation across directions. Its seeds are numbers over the formals (`hint((r * cos(u), r * sin(u), lead * u / 360deg))`); a geometric seed, an instance drawn on the sheet and an unknown of the drawing given as a number are refused (**E103**): a body traced in space reads the drawing's unknowns through the geometry it is written over. **[0.122]** A curve in space is a **path** (§4.1): a `path` formal takes it, `p coincident(t == s) k` pins the contact's place — the body's own formal — to `s` as a spline's `t` is pinned (in a face family, to the running number), and a seed's step runs along it, `hint(at: p, along: k, by: f)` being `f` times `∂C/∂u` at the place nearest `p`. A surface is no path (**E103** at the argument). **[0.125]** At a point a contact holds on it, `k curvature(at: p) c`, `c` a point, puts `c` at the curve's **centre of curvature** there (three equations: `C + |C′|²·((C′ × C″) × C′)/|C′ × C″|²`), where a surface's `curvature` is a circle osculating a section; a curve with no curvature at the place (a straight run, an inflection) has no centre, and the equations lose their rank there. Of a surface it is **E040**: a surface has no one centre at a point.

**Over two formals [0.108].** `S := C(ARGS).p over (phi in (A, B), s in (C, D))` is the same point as two formals run: **a surface** traced in space. `p coincident S` owns a place `[u, s]` (three rows, net one); at a point a contact holds there, `S tangent(at: p) l` puts a line in its tangent plane (one row), `S perpendicular(at: p) l` along its normal `C_φ × C_s` (two), `S tangent(at: p) P` a plane touching it (two), and `S curvature(at: p) k` the circle osculating its section by `k`'s plane (three, Meusnier, as an envelope's, §6.15.3). A place pinned is `p coincident(u == a, s == b) S`. A surface is **regular** where its two derivatives are independent; `solventc` says of each body traced in space whether it is regular over its domain or where it folds — of a flank seen from its member, the undercut. A traced surface is held to and read, not yet a face of a solid.

**Over two formals, marched [0.121].** A body traced over two formals may state a rate along either: `S := Front(c, a: 40mm, b: 25mm).p over (d in (0mm, 15mm), u in (-60deg, 60deg))` is an ellipse's offset front when `Front` says its point moves along its normal at unit speed, square to itself — a first-order PDE. The first formal is **marched**, the second **sampled**, and the surface is the **method of lines**: the second formal's interval is cut into 32 equal pieces of six Gauss–Lobatto nodes each, a joint node shared by the two pieces it ends — a whole turn of an `Angle` formal whose two ends meet is closed — the body's points at the nodes are one state integrated along the first formal from its home, a rate along the second is the derivative of the quintic through a piece's nodes (at a joint, the mean of the two pieces'), and between nodes the surface is that quintic. A rate along an `Angle` formal is per radian. **`initially { … }` is the Cauchy data**: what holds where the first formal starts, for every value of the second — **[0.131]** a point a motion carries among it, so a starting circle may be a drawn point turned about an axis by the second formal, `p coincident(under: spin, t == u) s`, and a trochoid the point a planar motion carries. The body's points are split by its rate rows: those whose rate along the first formal a rate row reads are **moved**, integrated; the rest are **placed**, solved at every node and step from where they last stood by the relations that read them, which must be exactly as many as their numbers (the front's normal `nl`, ten long, in its plane and square to `rate(p over u)`). So a rate along the second formal is of a moved point and is read by a rate row or by a relation that places a placed point — one over moved points alone would, differentiated along the first, tie every node of a piece together — a relation reads at most one rate along the first, and `initially` places moved points only: otherwise **E103**, naming the point. Rates that leave a freedom are **E103** naming it, as over one formal: the front's law alone leaves each node free to slide along the front, and the **gauge** that pins it — `rate(p over d) parallel nl`, square to itself — is the author's to state. At every node at every step the rates' rank is checked, and so is the surface's **regularity**, `rank [x_d, x_u] = 2`: where a node's area `|x_d × x_u|` falls below a hundredth of what it was at the home the characteristics are crossing — a **fold**, an undercut's kind — and the march is refused there; `solventc` says how far each such surface is marched, or where it folds and at which node. A march is held to the march on twice the pieces, the discretisation's error being amplified as a front focuses. Contacts own a place on the surface as on any traced one (§6.5 [0.108]); its point, tangent plane and normal are exact in the place and in the geometry (the variational equations), and its curvature is not yet read (**E103**). **Cauchy data on a drawn curve [0.133].** The moved point may start on a curve the drawing has: `initially { p coincident(t == u) k }`, `k` a formal `k: curve` given a spline, a curve drawn in a plane — a formula, a trace, an envelope — or a curve traced in space (a `spline` formal takes a spline alone), and `t` pinned to an expression over the formals (`t == 3 * u`) as any place is. At every node the point stands on `k` where its parameter is that number: three rows, the point less the curve's point there, a curve of a plane read in space through its plane's basis, which must be held (**E103** otherwise: `fix` its axes and origin). The body's columns are then the curve's — a spline's control points, a curve's own unknowns, a traced curve's columns — so the surface moves when they do, and the march's sensitivities run through the curve's. Only `initially` of a body over two formals reads such a curve, and only so: a point on it elsewhere in the body, the curve read by any other relation, or a surface given as `k`, is **E103**. A front marched out from a cam's profile by the roller's radius is its pitch curve, and its fold the roller's undercut (`rates/pitch.sv`); the Wankel bore read off the apex's envelope is `rates/bore.sv`.

**Rates [0.119].** A body may state a condition on a **rate** along the formal it runs over: `rate(x over s)` is point `x`'s velocity as `s` runs — a vector of dimension `[x]/[s]`, so dimensionless along an arc length — and it stands where a direction is read: `parallel`, `perpendicular` or a directed `angle` with a line, and `rate(p over s) distance(k, along: l)`, the rate alone before the word, its component along `l`. A rate is never a place (`coincident`) and never a magnitude (`length(1) rate(p over s)` would be quadratic in the rate, leaving a root to choose at every step): either is **E103**, and so is a relation reading two rates. The formal is always named, since a component does not know which of its formals the call runs; a component handed it as itself (`Drag(o, a: a, u: u)`) reads its own rates over the caller's, and one given a number or an expression in it is **E103**. A body that states a rate is **integrated from its home** rather than solved again at each place: its relations without a rate hold everywhere, its **rate rows** together with them differentiated along the formal fix the rates — one linear system, square by count in a plane and, in space, where a `parallel` is two rows of which the body's own rows may imply one, of full rank — and the curve at `u` is that flow integrated from the home, to a tolerance well below "solved" (`ode.rs`). The system's rank is checked at every place the march reaches: where it is lost the curve is refused there, as is a march that will not meet its tolerance in bounded steps, and `solventc` says how far each such curve is integrated. A body short of rate rows is **E103**, naming the points whose rates the rows leave free. **The home is the initial value.** Over a drawn instance it is the instance as it stands on the sheet: its rate rows are no rows there, so the drawing holds the freedom they leave (`drag.p level(right) o`), and the instance's coordinates are columns of the curve, which moves when it does; while the drawing is not yet solved, the curve is the flow from the nearest point to the instance where its body's relations hold. Written in place, the home is the value the call gives the formal, or the interval's start, and what holds there alone is said in the body, **`initially { … }`**: relations that, with the body's others, place the instance at its home, and nowhere else — the block a face family's station says its home with (§6.9 **[0.116]**), its relations hoisted into the body, each marked; a rate in it is **E103**. **[0.131]** A point a motion carries, `X coincident(under: m, t == τ) x` (§9.3), is a relation without a rate like any, in the body or in `initially`: its rate is that row differentiated along the formal, read exactly. On the sheet an `initially` block's relations are ordinary ones, a drawn instance being its own home. A contact owns its place as on any curve, `C′` the rate itself and `C″`, `C‴` the solution's Taylor coefficients, exact, and every derivative in the geometry the flow's sensitivity, carried through the same integration. Over one formal a rate along an `Angle` formal, or of a direction (a line's or an axis's), is not yet read (**E103**); a face family's station reads an axis's (§6.9); over two, below.

**The rate of a contact [0.130].** Outside a traced body, `rate(p over t)` where `t` names no formal in scope runs over **the slot of the contact holding `p`**: a spline's or a curve's parameter `t`, a traced curve's `u`, a meshing's phase `t` (§6.14), the roll `t` of `p coincident(under: m, t == τ) q` (§9.3). It is the **partial derivative** of the point the contact holds in that slot, every other unknown of the drawing held — the contact's own other unknowns (a meshing's lag and places) following its rows, by the implicit function theorem — so the slot is always one contact's: a slot no contact holding `p` owns is **E061**, naming the forms it may take, as is a point held by two contacts owning one (say which with `g tangent(at: p) l`); a `param` or a formal given a number is never a slot, and a rate along a declared unknown is **E040**. The rate is a length a unit of the slot — a radian of an angle's — and it is a row of the drawing, solved with it: held `parallel` or `perpendicular` to a line or an axis (a curve's in its view, to a line drawn there), or measured along one, `rate(X over t) distance(d, along: l)`, `d` stated or an unknown. `g tangent(at: m) l` of a spline, a curve, a traced curve or a meshing is the same statement written as the tangent it is — the path of contact is the meshing's rate along its phase. **Every statement at a point a contact holds is the contact's frame**: a surface's normal, `S perpendicular(at: p) l` (a traced surface's, an envelope's, a meshing's common normal), is the line square to every rate of it; a plane touching it, `S tangent(at: p) P`, the plane along that normal; a line touching it, `S tangent(at: p) l`, a line square to it; its section's circle, `S curvature(at: p) k`, read off its second order by Meusnier; and a traced curve's centre of curvature, `k curvature(at: p) c` of a point (**[0.125]**), off its first rate and its second — one statement, one kernel, whatever the contact, each read exactly as its own words were (§6.14, §6.15). What a contact has no frame for — a curve's normal, an envelope's rate — is **E061**. A rate never adds an unknown: it shares its contact's (§9.2), which may be stated after it. A motion's twist is read so (§9.3): the standard library's `reciprocal` and `Screw` are rates of carried points.

```
component Drag(o: point, a: Length, u: Scalar) {
  q := point
  p := point hint((0, 30))
  q level(up) o
  o distance(a * u, along: right) q            // the string's end runs along the line
  string := line(p, q)
  length(a) string
  rate(p over u) parallel string               // dragged: p moves only along the string
}
in std.front {
  o := point
  fix((0mm, 0mm)) o
  drag := Drag(o, a: 30mm, u: 0)
  drag.p level(right) o                        // the home: the string square to the line
}
tractrix := drag.p over u in (0, 3)
```

**Requirements.** An implementation MUST differentiate `C` with respect to the swept formal *and* with respect to every coordinate the component reads. `∂C/∂u` is which way a contact may slide; `∂C/∂θ` is how the curve moves when the geometry it is written over moves, and an implementation that computes only the first will solve a contact once and drop it the moment that geometry is dragged. For a computed point both are mechanical from the expressions; for a locus both come from the implicit function theorem at the body's solution. A name a computed point's expressions cannot reach is an error (**E103**), not a free variable: a curve is written over geometry that exists, and a misspelling there would quietly add a degree of freedom to every point on the curve. A dimension in a traced body MAY read a number of the geometry the component is written over (`c.r`, `k.center.x`): it is a column of the curve, and is differentiated with the rest. **[0.132]** In a body traced in space (§6.5 [0.107]) it MAY also read a coordinate of a point the body declares (`p.x`, `p.y`, `p.z`): the dimension is then a number the body places, held to its expression by an equation of the body's own and solved, integrated and differentiated with the body's points — so a rate row's speed may depend on where the point is (`rate(p over s) distance(1 / n, along: nl)`, `n` read at `p`: the eikonal equation in a graded medium). One text is one number, however many relations read it. Over two formals such a number is placed at every node with the points the rates do not move, so it stands in a rate row or in a relation that places such a point; read in `initially`, or by a relation that would place only points the rates move, it is **E103**. A trace in a plane reads only the formals and the geometry (**E103** otherwise). **[0.37] Outside a trace a dimension MUST NOT read geometry** (**E103**): the expression graph has no column for it, and minting a free variable named `k.r` would make a drawn instance of a component state something other than its own trace.

**Branches.** A locus generically has several solutions, and a component states its way onto one — three instruments, in order of strength:

1. **A signed constraint,** wherever the vocabulary can say it. Above, *neither* choice is a branch at all: `angle` is directed (§9.4), so `t` sits at the bearing and not opposite it, and a `distance` from a line whose number runs with the swept formal is read signed in a traced body (on the sheet it is a magnitude, §4.1), so one equation unwinds the string one way for positive roll and the other for negative.
2. **An orientation predicate.** `ccw(a, b, x)` / `cw(a, b, x)` in the body is §9.6's statement doing §9.6's job: it contributes no residual and *selects among the discrete solution components*. Traced, its third point MUST be one the component places. A predicate is read **at the anchor** — the drawn pose, or the value the instance gave the swept formal, chosen where the predicates read unambiguously — and an implementation MUST enforce it there (reflect the placed point across the oriented line and solve again) and MUST NOT re-enforce it elsewhere: away from the anchor, continuity governs, and the component the predicate picks at the anchor is the component the whole curve is on, even where the curve has since wound to where the predicate no longer reads true. A body with predicates needs no seeds at all: an implementation MUST fall back to deterministic restarts, scaled by the geometry the component is written over and by nothing else, when the seeds (or their absence) leave the anchor solve nowhere to start. Drawn, the same predicate records the root choice the drawing is on (§9.6).
3. **A seed.** What neither an equation nor a predicate says, a seed says: the body's seeds are places over the formals, evaluation of an instance written in place starts from them, and away from the anchor continuity governs — an implementation MUST evaluate the curve as one continuation along the parameter, so the branch picked at the anchor is the branch everywhere. A curve over a drawn instance starts from the pose on the sheet and reads no seed.

**Places, not coordinates.** Inside a component that is only ever traced, a seed may be a *place*: `t := point hint(at: c, bearing: u + phase)` is the point at the edge of circle `c` at that bearing from the x axis of the plane it is drawn in, and `p := point hint(at: t)` is wherever `t` starts (a point already named must be declared first). Both lower to exactly what the coordinate spelling would, so `hint((xexpr, yexpr))` remains available and means the same thing. On the sheet a seed is a number a solve writes back, which a place named by reference is not, so a drawn instance of a component with a geometric seed is an error (**E103**).

**Bearings are measured in a plane.** A bare bearing was page-fixed, and a body posed against a datum with page-fixed seeds went quietly stale the moment the datum tilted (bmander/geomsolver#10); 0.6 answered with the datum's derived `f.angle`. **[0.42]** A bearing is measured from the x axis of the plane the seeded point is drawn in, so geometry drawn in a part's own plane turns with it and needs no correction; `f.angle` is retired with the rotor (§3.2), and reading it is **E103**, naming the scalars a plane has.

**Why a point of a component and not a family of its own.** 0.2 gave a curve a construct of its own — a family with formals, a parameter and a body — and 0.3 a second body form, and every mechanism was then written twice: once as the drawing and once more inside the family, with the formals passed to themselves (bmander/geomsolver#46). A component already has formals, a body, params and instances, and a curve is one question asked of it. The gear in §18 is the case that makes the difference plain: its flanks are involutes because the document says what an involute is, and nothing in the solver knows the word; the walking leg of `jansen.sv` is drawn once, and its stride is the same leg asked where its toe goes.

### 6.6 Chains **[0.4]**

A chain writes a run of declarations and the constraints *between* them in one ordered breath. Its geometry and constraints elaborate to exactly the statements a person would otherwise write out. **[0.20]** An optional `NAME :=` binds the resulting ordered traversal: an open chain, or a closed loop when the expression ends in `-> close`. Naming the traversal adds no coordinates or equations.

```
horizontal (bottom := line(b1, b2)) -> tangent
(a_br := arc(center: c_br) hint(r: r)) -> tangent
…
vertical (right := line(r1, r2)) -> tangent close
```

```
CHAIN  ::= [NAME ":="] LINK (JOINT LINK)* ["->" INFIX* "close"]
LINK   ::= PREFIX* (DECL | "(" NAME ":=" DECL ")") | REF
PREFIX ::= a constraint name whose spec is one entity slot     // horizontal, vertical
         | "step" "(" STEPARG ("," STEPARG)* ")"                // [0.101] step(u: s, dv: b)
STEPARG ::= ("u" | "v" | "du" | "dv") ":" EXPR | "from" ":" REF
JOINT  ::= "->" INFIX* ["->"] | INFIX+ ["->"]                  // at least one marker or word
INFIX  ::= "tangent" | "equal" | a constraint name whose spec is two entity slots
```

**A chain is a value you can name [0.20].**

```solvent
component Box(/* parameters */) {
  // Points and dimensions are declared in this scope as usual.
  profile := (ab := line(a, b)) -> (bc := line(b, c)) -> (cd := line(c, d)) -> (da := line(d, a)) -> close
}
boss := Box(/* arguments */)
iboss := solid(boss.profile, depth: 8mm)
```

The binding is an ordinary name in the enclosing scope. A component may expose several named
chains, reached through its instance (`boss.profile`); no name is special and no default is
inferred. The individual declarations remain in that same scope (`boss.ab`), rather than moving
under the traversal. Anonymous links are allowed: `profile := line -> line -> line -> close`.
In a swept loop, anonymous links name their sides `edge0`, `edge1`, … in traversal order,
skipping names already used by explicit links. Source offsets never become public side names.

A named chain MUST traverse lines or arcs and MUST thread every adjacent pair with `->`;
relations without a threaded joint can still form an unnamed chain of statements. Its links use
all the existing prefix, joint, endpoint-aliasing and forward-reference rules. A named expression
MUST finish before a block's closing brace; it cannot end in a joint awaiting the next copy.
Bindings within repeated components and blocks are independent, just like their geometry.

A named closed loop is usable directly wherever a sweep takes its section (§6.9), and is checked
by the ordinary face rules (§6.8). The closure marker records topology, not geometric validity:
planarity is checked during elaboration and the swept profile's geometric validity after solving,
so a poor hint is not rejected as the final shape. An open chain is not a sweep section (**E080**).
A `face` list may name either kind of traversal, expanding its links in place and in order;
`face(trail, -> close)` explicitly closes an open chain using the face's existing closing rule.
Naming a chain never closes an endpoint gap by itself.

**[0.8] Threading is a statement, not an inference.** The `->` marker on a joint says the two links beside it share a boundary point, threaded left-to-right along the traversal below; its absence says they do not. `->` alone is the plain corner — this is what `to` said through 0.7, and `to` is retired into the marker. `-> INFIX` is a corner that also states the relation, at the point just threaded. `INFIX` alone is the relation and no corner: `a_br equal a_tr equal a_tl` says three arcs are the same size and nothing whatever about where they meet, and welding them would be an invention. Because each joint states its own threading, a chain may mix declarations and names freely — `(l1 := line(a, b)) perpendicular (l2 := line(c, d))` declares two separate lines at a right angle, and `(l1 := line(a, k.start)) -> tangent k` extends a fresh contour onto geometry that is already there.

- A **prefix** desugars to that constraint applied to the declaration it stands before: `bottom := horizontal line(b1, b2)` is `bottom := line(b1, b2)` plus `horizontal bottom`. Eligibility is registry-derived — one entity slot and nothing else — so a new unary constraint joins the grammar without the grammar changing.
- **[0.8] A joint may state several relations.** `A -> equal angle(30deg) B` states both between the two links, at the corner the marker threads; each word desugars to a statement of its own, with its own identity and span. The marker may stand on either side of the words, or both — `A -> equal -> B` is the one joint `A -> equal B` is — and words with no marker state their relations and weld nothing. The words need no punctuation because fixity sorts them: a word is read at the joint until one opens the next link — an element keyword, or a prefix word standing before one — so in `-> tangent horizontal line b` the tangency is the corner's and the levelling is the link's. Deleting one relation splices its word out and leaves the rest standing; the whole joint deleted at once falls back to what a single word's deletion would be — the corner stays, or the statement breaks. **[0.20]** A trailing placement is the drawing's (§13.1), refused in model source.
- A **joint** stands between two links and says how they meet. Each joint constrains its two neighbours without changing which links the chain traverses; the optional binding names that traversal. Joint words have no operator precedence. `a equal b equal c` is therefore two statements, not three, and n operands give n−1 — the same rank as any other spanning set over the same elements, stated as a path rather than a star. An INFIX word is the two-argument counterpart of PREFIX, derived from the same registry: it desugars to that constraint over the pair, positionally, and MUST fit both — a word whose slots the pair cannot fill is an error, not a guess.
- **[0.5]** `equal` is **polymorphic**: `equal_length` between lines, `equal_radius` between circles or arcs, and an error between one of each, since no constraint equates a length to a radius. Like `tangent` it is drafting vocabulary rather than a constraint name, so no registry lookup can resolve it — the pair it stands between does. Where a chain declares its elements the keywords settle it as the program is read; where a chain names them it cannot be settled until the names are resolved, since a name may be declared further down the body (P2) or come from a component, so the word travels to elaboration and is settled there. Both report the same error.
- **Threading.** **[0.8]** A link a marker reaches is a line or an arc — an element with an entry and an exit, read left to right (`p1 → p2`; CCW, `start → end`); a kind with no boundary points — a circle — cannot be reached by a marker, though it may stand in a chain no marker touches. At each `->` the shared point may be named by one side, by both in agreement (two different names are an error), or — between two declarations — by nobody: the chain then mints it, the earlier-built side's boundary being an anonymous child with a name (its dotted path, §6.1), which fills the later side's slot. `(l1 := line) -> (l2 := line)` is therefore two lines and three points, one shared. A link that only *names* an element offers no list to read or fill and no kind to read a boundary field off — so at a corner with one, the declared side MUST name the shared point, usually by the existing element's own child (`k.start`). An end no marker reaches is an implicit child like any other unwritten slot (§6.1).
- `-> close` after the last link seals the loop: the last exit threads to the first entry, and a word beside the marker says how they meet there. A loop is a thread, so a `close` without the marker is an error.
- A statement otherwise ends at its line's end (§2); a line ending in a joint — the marker or a word — continues its chain onto the next.

**Every threaded joint is the regular form.** A threaded joint knows the shared point, so `-> tangent` between a line and an arc is `tangent_arc_line(arc, line, at: start|end)` — tangent *at* the point just threaded — and never the bare tangency over a coincidence, whose Jacobian is rank-deficient at every solution; `-> tangent` between two lines is collinearity (`parallel` over the shared point), and a pair the vocabulary has no regular at-form for (two arcs meeting at a corner) is an error, never a silently degenerate statement. An **unthreaded** `tangent` is the plain pair (`tangent_line_circle`, `tangent_circle_circle`), which is the correct and well-conditioned statement exactly when the two are separate — the `at:` argument is only ever supplied by a threaded joint. A bare `->` states nothing beyond the corner: the shared point is the whole of it.

**[0.8] A block body may end mid-joint.** Inside a `repeat`, `cycle` or `ring` body (§12), the body's final chain may end in a *threaded* joint — the marker, or the marker with words — standing at the body's `}`: the trailing joint threads the chain onto the **next copy's** first link, and is stated between copy i's last link and copy i+1's first exactly as an in-chain joint would be — the weld a shared point, every worded tangency the regular at-form. Which pairs are stated is the block's kind: `cycle` and `ring` wrap, so every copy states it and the trailing joint is the loop's closure — `cycle N { length(d) line -> angle(a) }` is the dimensioned N-gon, with no `close`, no names and no written points — while `repeat` does not wrap, so the *last* copy's trailing joint is simply not stated and `repeat N { line -> angle(a) }` is an open polyline of N sides and N−1 corners, the natural reading rather than an error. Both boundary links MUST be declarations of kinds with ends (a name-link's boundary is §6.6's ordinary rule: the point must be named where it stands), and at most one of the two boundary slots may name its point — both named are two *different* points across the copy seam, and that coincidence is stated longhand. Where the construct can mean nothing — a `component` body, a trace block, the top level — it is an error, and an unthreaded trailing word still wants its right operand. The joint is one written joint however many copies state it: each stated copy keeps the one statement identity, told apart by the instance path (§12.7). A statement inside a braced body also ends at the body's `}` as at a line break, so the whole of a block may sit on one line.

Each desugared statement keeps an identity of its own and a span into the chain's text, so a caret, a diagnosis culprit and a splice land on the word that stated the thing. (§12.7 is many instances from one statement; a chain is several statements from one *line*, each still its own.) Deleting a chain-borne constraint is therefore a splice — a threaded joint steps down to the bare corner `->`, a prefix word goes where it stands, and **[0.8]** an unthreaded joint becomes a statement break, its span grown over a terminal name-link that a break would leave dangling (inside a chain that closes there is no safe break, and the deletion is refused). Deleting a link is refused: no splice takes one link out and leaves a chain behind, so that edit belongs to the source.

**[0.101] A step says where a line's end stands, one number per run.** `step(…)` is a prefix word over a line — in a chain, before its link — whose parameters, by label, are the ordinates of the line's end `p2`: `du` and `dv` from its start `p1`, along the frame's `u` and `v`; `u` and `v` from the frame's origin. Each ordinate is one or the other, never both (`u` and `du` together is **E040**), so one step may put its end at an absolute station and a relative rise, `step(u: s, dv: b)`. **The ordinate a step does not name is kept** — its end level with its start along that axis — so `step(u: s)` is a run along `u` to the station `s`, and `step(dv: h)` a rise of `h`. A step that names neither is **E040**, an unlabelled number **E004**.

```solvent
in std.front {
  profile := step(u: outer_radius) (ab := line) -> step(v: plate_depth) (bc := line) ->
             step(u: hub_radius) (cd := line) -> step(dv: hub_height) (de := line) ->
             step(u: bore_radius) (ef := line) -> step(v: 0mm) (fa := line) -> close
}
```

The **frame** is `from: P`, a plane — its origin and its axes, `P.u` and `P.v` — or, where no `from:` is written, the view the line is drawn in, which an `in` block gives; a step with neither is **E040**. A step is sugar for the relations it names, each its own row: an ordinate per ordinate named, `p1 distance(d, along: u) p2` from the start and `P.origin distance(s, along: u) p2` from the frame's origin, and a level per ordinate kept, `p1 level(v) p2`. An ordinate written as a literal zero is that level, a relation which states no number, as a zero `distance(…, along:)` would be (§9.2). Every number is a dimension of the statement: the step keeps one statement identity, is described as written (`step(u: 32mm) ab`), and each number keeps the place it is written at, where its callout reads it and an edit rewrites it. A number may read an unknown.

A closed walk of steps leaves its last edge to close it, as SVG's `z` does: a link with no step, whatever the others leave. Stating every relative step round a loop states the closure twice — two rows of no rank, which the diagnosis names. **The corners start where the walk puts them**: a corner a step places, and the source wrote no seed for, is seeded from its line's start, or from the frame's origin, by the step's numbers, in statement order and twice round, so a closed walk whose first corner its last step places starts there. Nothing is restated in a `hint`, and the seeds follow the numbers. A step measured in a frame other than its line's view, or over an unknown, leaves its corner where it was minted.

*Non-normative:* chains and paths (§10) answer different questions. A path is a traversal of geometry that already exists — vertices, the circles its arc segments lie on, orientation and branch rules, for boundary composition and export. A chain *declares* the geometry: it is how a contour's elements, their meetings and the levels on its straight runs are written down in the first place. The case library's fillet rectangle is the canonical chain; its longhand form states the same sketch in thirty statements.

### 6.7 Planes and projection **[0.10]** **[0.42]**

A drawing's geometry is drawn in planes, each a plane in space, and **[0.42]** a point drawn in
none stands in space. A multiview drawing is several planes with one object's corners drawn in
each, related by projection: the draughtsman's descriptive geometry, stated as such. A plane is a
place to draw and nothing else; where its picture goes on paper is the drawing's (§6.11).

**A frame** **[0.134]** is the one datum kind, a plane and (§6.14) a motion seen as one thing. A
plane is a frame whose `in` draws flat. **`f := frame`** declares a spatial frame **bare**: it
owns nothing — no axis, no place, no unknown — until the document names what it has. Its axes
`f.u`, `f.v` and its origin `f.origin` are minted where a reference names them (or where `hint(at:
f, …)` or an ordinate `along: u | v | n` against `f` reads them), seven numbers as a plane's are,
placed by ordinary relations (`f.u parallel l`, `f.origin coincident p`, `fix(dir == …) f.u`,
`fix(origin == …) f`). A point `in` a frame with axes has **three** numbers, `x` along `f.u`, `y`
up (`out × u`) and `z` out (`u × v`), standing at `o + x·u + y·up + z·out`, seeded by
`hint((x, y, z))` or `hint(at: f, (x, y, z))` and held by `fix((x, y, z))`; a point in a frame
with none is a point in space, drawn in its parent's coordinates, the world's. **`in (f := frame)
{ … }`** declares the frame in the block's header, `f := frame` then `in f { … }`. A frame takes no
brackets and no seed (`frame(…)`, `frame hint(…)` are refused), and is no set: `p coincident f`,
`f distance(d) g`, `f parallel g` and `p inside f` over a spatial frame are **E040**, which an
ordinate measured from its origin along its axes is not. The word is read as a declaration only
as a definition's whole value, so `frame` stays a name a document may give; 0.15's `frame
f(origin: o, …)` is refused with the spelling it has now, and a formal `f: frame` takes a plane as
well. Through 0.133 the word was refused everywhere.

**A point in space** **[0.42]** is a point no `in` reaches. It has three coordinates in the
world's axes, seeded by `hint((…, …, …))` and held by `fix((…, …, …)) p`; a
`z:` on a point drawn in a plane is **E040** at the key. A line between two points in space is a
line in space. A circle and an arc are drawn in a plane, and over a point in space each
is **E060**, naming the point; a face is **E080** (§6.8). **[0.127]** An arc over a point in
space is drawn in the plane through its centre and its ends — the short way round, less than a
half turn — its points those written, so its radius, its contacts and a chain's joints are an
arc's; three points in a line, which no one plane is through, are **E060**. A circle over a point
in space is drawn in the plane through its centre square to its **normal**, `circle(center: o,
normal: ax)`, `ax` an axis or a line; with none it is **E060**, and a `normal:` of a circle drawn
in a plane is **E040**. **[0.126]** A spline over a point in
space is a **curve in space** (§6.5): the same curve, `Σ Rᵢ(u)·Pᵢ` over its control points where
they stand and its parameter over its own domain, whose contacts are a curve in space's and which
is a `path`; `spline(through: …)` over corners in space is the cubic through them. It is drawn in
no plane, so it bounds no face (**E080**). A 2D drawing is drawn in a plane — `use
std`, then `in std.front { … }` — and there is no page: `std.front` is a plane like any other,
the one at the world origin with u = x to the right and v = z up, its viewer at −y.

**A plane is a set, and `in` is membership of it** **[0.54]**. `p coincident P` (either way
round), said of a point standing in space, **draws the point in `P`**: it is `p := point in P`
said as a relation, and the two are one statement with the same rows — the point has `P`'s two
coordinates, and every relation over it reads as one of `P`'s points' (`p distance(25) a`, with
`a` drawn in `P`, is `P`'s own distance). `l coincident P` draws each end of `l` not yet in `P`
there. A point standing in space is drawn in the first plane a statement puts it on; put on a
second, it is held there by a row in space. **[0.74]** A point its declaration draws in a plane —
its own `in`, or an `in` block around it — said on another, `p coincident P`, is **drawn there
too**, as `in P, G` draws it (below): `gear.M coincident q` draws the point another component
declares in a plane only this one knows, and what is drawn in `q` reads it there. The other way
round, `P coincident p` passes `P` through it (below), a row, so a plane may be put through a
point of a plane it lies on. A point only its instance's `in` puts in a view stands in space to
the component declaring it, which says where it stands whatever view the instance is put in, so
what the component says of it keeps the rule for a point in space. A point whose own
statements say it stands in space stays there and the plane holds it by a row: one seeded with a
height (`hint(z: …)`), one whose three numbers a `fix` holds, the point a tangency differentiates
at (§6.21), a `ring`'s (§12.4), and any under a `claim`. Which representation stands is the
elaborator's; what the drawing means is the statement's either way.

**A point may be drawn in several planes** **[0.58]**. `O := point in P, G` draws `O` in `P` and
in `G`, so on the line where they meet: one freedom along it where both planes stand. Each
plane's geometry reads `O` as its own point — `arc(center: O)` drawn in `G` is centred there,
`line(O, T)` in `G` ends there, `O distance(5) q` with `q` in `G` is `G`'s distance — so what a
descriptive layout draws twice and ties by `project` is one point. **[0.74]** `O coincident G`,
said of `O` drawn in `P`, is the same statement: `O` drawn in `P` and `G` (above), however many
components apart the two are written. Said of a plane `O` is already drawn in, it asks nothing
(**E061**, as before); under a `claim`, in a `ring` (§12.4) and from a set's body (§6.21) it holds
`O` there by a row, as before. **[0.95]** From a set's body said of a point already drawn in
the plane (or twinned there), it holds by construction and states nothing: the body is a
predicate, true of that point, so `m coincident { p | p coincident ball; p coincident
std.front }` with `m` drawn in the front is `m` on the circle the front cuts from `ball`. `project` relates two planes by definition, so it reads each
operand in its own plane, never by its twin: `m coincident P` and `O project m`, the layout that
drew `m` again, say what they said. *Non-normative:* an end a line mints and does not name
(`l.p1`) is still held on a second plane by a row. A plane named twice is
**E040**; two planes that are parallel meet on no line, **E061**. **[0.81]** Parallel where
they stand: judged once the statements about planes have placed them, before the solve, and of a
plane they leave only seeded, where the solve leaves it — a station plane seeded as the front and
turned by the solve square to a guide in the front is no contradiction. Only a point is drawn in
several planes (a line is, when its ends are; **E040** otherwise). Where a plane is itself built
along a line `O` is on — an end of it, or stated on it — the plane already holds `O`, and the
elaborator states nothing more.

**An axis** (§3.1) is a directed line in space with no start. `t := axis hint(dir: (0, 0, 1))`
seeds its direction, and relations place it: `t parallel s`, `t perpendicular s` and `t angle(θ)
s` against an axis or a line, `p coincident t` for a point on it, and against a plane `t coincident
P` (it lies in `P`), `t parallel P` (it runs along `P`) and `t perpendicular P` (it is square to
`P`); each reads either way round. **[0.43]** `t coincident s` between two axes says they are one
line, either way round: four rows, the seed choosing the sense as it does for `parallel`.
**[0.44]** `l coincident t` between a drawn line and an axis says the line lies on it, either
way round: both its ends on the axis's line, four rows, placing the axis as `p coincident t` does.
`fix(dir == (0, 0, 1)) t` holds its direction, and **[0.43]** `fix(origin == (0, 0, 0)) t`
where it stands — its `origin` **[0.45]** is its point nearest the world origin. An axis read only
as a direction has two freedoms, and one whose place a relation reads has two more. **A drawn line
is an axis** wherever an axis is asked for, from its `p1` toward its `p2`, as its points stand.
**[0.43]** A plane over a line holds a hidden axis that is the line: parallel to it, and through
its `p1` — or, where the plane's two lines share an end, through that end, the plane's origin
standing there (three rows, not four). So `plane(u: hinge, v: …)` stands on the hinge.
There is no vector arithmetic: a direction is a thing constrained, never a value computed.

**A plane is two axes through a place.**

```
right := plane(u: std.y, v: std.z)        // looked at from +x: y to the right, z up, at the origin
t := axis hint(dir: (0.87, 0, 0.5))
t perpendicular std.y
side := plane(u: t, v: std.y)             // turns about y as the solve turns t
side.origin coincident std.front          // on std.y already: at the world origin
p := plane                                // [0.43] its axes p.u and p.v minted, free
```

`plane(u: r1, v: r2)` takes two axes or drawn lines and nothing else; a plane's 0.10–0.41 labels
— `origin:`, `toward:`, `from:`, `fold:`, `offset:`, `through:`, `attitude:`, and a basis written
as triples of numbers — are refused where written. Its **basis** is read off the axes'
directions: `û = u/‖u‖`; the normal `n̂` is `u × v` normalised, out of the plane toward its
viewer; and `v̂ = n̂ × û`. So `v` need not be square to `u`: it says which plane, and which side
is up. **[0.43]** The axes pass through the plane's origin: each plane carries two rows, its origin
on each axis, so where the axes stand the plane stands, and `u coincident P.u` places a plane by
an axis. Two planes share an axis only where both pass through it. A slot left unwritten mints an
axis, `P.u` or `P.v`, free and seeded as the front's (u along x, v along z), or from a seed the slot holds (`plane(u: hint(dir: (0, 1, 0)))`, or **[0.81]** read off geometry, `plane(v: hint(dir: up))`, §6.4): a bare `P := plane`
has seven freedoms, three of place and two of direction for each axis. 0.42's planes sharing
axes while standing apart are withdrawn: a plane parallel to another takes its own axes, held or
`parallel` to the other's.

**Where a plane stands** is three unknowns, its origin's place in space, read by a seed as `P.x`,
`P.y`, `P.z`, seeded by `hint(origin: (…, …, …))` and held by `fix(origin == (0, 0, 0)) P` **[0.45]**. A plane
floats along its axes until something places it. `P.origin coincident p` puts its origin at `p`,
all three. `P coincident p` passes it through `p`, and `Q distance(d) P` stands it `d` off `Q`
(§6.10): each takes the one freedom across the plane and leaves it to slide within itself, which
the rest of the document settles or leaves free. **[0.43]** `P parallel Q` says two planes face alike, either way round — their normals parallel, two rows — and nothing of where either stands or how it turns within itself. **[0.43]** A plane over two axes wholly held
stands where they meet, and adds no row; held axes that do not meet or run alike, and a plane
held where its held axes are not, are **E067**.

Every plane has three members. `P.u` and `P.v` are its axes, axes through its origin. `P.origin`
is a point drawn in `P` and held at its `(0, 0)`, which a component may be handed and a relation
may place. An axis square to `P` is `t perpendicular P`. **[0.89]** A fourth, `P.normal`, is the
axis along `u × v` through its origin: square to both axes, on the side `u × v` is, standing where
the plane stands, and read wherever an axis is (`l parallel P.normal`, `plane(u: P.normal, …)`,
`hint(dir: P.normal)`). It is the plane's, never a freedom of its own, and is not declared: a
reference names it, so a declaration before `P`'s cannot (**E040**). In a seed `P.n` reads the
same direction.

**A point drawn in a plane has the plane's own two coordinates.** The point `(a, b)` drawn in `P`
stands at `o_P + a·û_P + b·v̂_P` in space, its *lift*, which is what a relation in space reads
(§9.2). A plane whose axes' directions and place are all held is **constant**: its basis is
settled at elaboration and a lift over it reads it as constants. Any other plane is solved with
the sketch. *Non-normative:* an implementation places the planes and axes first, solving the
statements about them alone (and about points of planes already placed) round by round, so the
main solve starts where the planes stand rather than where their seeds put them; the geometry
drawn in a plane never places the plane.

**Two planes lying on one another** — turned alike up to a turn within themselves, and standing
in one place — are permitted. Where a relation reads points drawn in each, an implementation
SHOULD say so once, at the later plane's declaration, judged on the solved pose (**W113**: "`B`
lies on `A`, and a relation reads points drawn in each: one plane in space, so the relation is
read in space where one plane would read it on the plane"). A part drawn in its own plane turned
within `std.front` is the common case; a component drawing in the plane its caller hands it never
meets it.

**The standard planes** (§14.4). `use std` gives the axes `std.x`, `std.y`, `std.z` and
`std.back` (x reversed), all held; the planes `std.front` (`u: x, v: z`), `std.top` (`u: x, v:
y`, looked at from above), `std.side` (`u: y, v: z`, looked at from +x) and `std.up` (`u: z, v:
back`, the front turned a quarter), each held at the world origin; and `std.origin`, a point drawn
in `std.front` held at its `(0, 0)`. `std.Turned(o, t)` is a frame turned within the plane `o`
and `t` are drawn in: `axes := std.Turned(o, t) in std.front` makes `axes.axes`, the plane through
`o` whose u runs toward `t`, and `axes.u`, the line from `o` to `t`.

**[0.21] Coordinates relative to a plane.** For a point `p` and plane `f`, `p distance(d, along:
u) f` states how far `p` stands along `f.u` from `f.origin` — **[0.46]** the ordinate `f.origin
distance(d, along: f.u) p`, spelled against the plane (§9.2): drawn in `f`, that is its own
coordinate, `p.x − d = 0`; drawn in another plane or standing in space, **[0.42]** it is read in
space over its lift, `(X_p − o_f)·û_f − d = 0`. `along: v` is the same along `v̂_f`, and **[0.24]**
`along: n` the signed distance along `n̂_f`, in space, of a point not drawn in `f` (one drawn in
it is **E061**: every point of a plane is on it). Each contributes one
length-valued residual, owns no unknown, and accepts a signed length expression, including a free
dimension. **[0.46]** Zero written as a literal is **E040**: it is `p level(u) f`. Either ordinate may be stated independently; `claim` has its ordinary
assertion meaning. The operands MUST be a point followed by a plane, and `along: u`, `v` or `n` is
required for this pair. **[0.92]** Between two points of one view the same `u` and `v` are that
view's own, the run and the rise (§9.2): one direction, whichever plane is at hand. An ordinate
read in space is drawn with no callout.

Reading a plane for coordinates MUST NOT assign plane membership to either entity, transform
incoming aliases, hold geometry, or create a component placement phase. Passing `f.origin`, `f.u`
or `f.v` to another component aliases it, including through nested components; a face may
likewise name `f.origin` as a corner.

A seed may read `f.x`, `f.y`, `f.z` and the coordinates of `f.origin`, and place a point in `f`'s
coordinates with `hint(at: f, (…, …))` (§6.4). These read the starting pose, including
geometric seeds already settled. A hint over an unbound numeric formal uses that unknown's own
seed for its provisional value (`lean: hint(20deg)` reads 20°, **[0.90]**), and zero where it has
none; an affine expression retains its constant offset. This substitution
applies only to hints, never to constraints, and traced components retain parameterized hints.
Hints still select starting configurations only. Signed ordinates explicitly exclude reflections
across the axis being measured.

**A point says which plane it is on with `in`.** `a := point in top` is a trailer of the declaration, order-free against `hint` and `knots`, and it applies to **every point the declaration mints or names**: `l := line(a, b) in top` puts `a` and `b` on `top`, `c := circle in right` its centre, `arc` and `spline` likewise. A membership moves nothing; it says whose coordinates the point's numbers are, and **[0.42]** a point with none stands in space. A point put on two different planes by two declarations is **E060**; agreement is not an error. `plane`, `axis` and `curve` have no points of their own to put anywhere, and `in` on them is refused. Inside a `ring` (§12.5) a plane is invariant: a membership referencing one is true of every copy alike.

**`a project b` says two points are images of one point in space.** It is an infix operator over two points (§9.2), each `in` a plane; the two planes are **inferred** from the memberships and are never written — an implementation MUST refuse (**E061**) a point on no plane (one standing in space), two points on one plane (a view relates nothing to itself), and two planes that are parallel (they share no fold line), each at the statement. With `d = (n̂_A × n̂_B)/‖n̂_A × n̂_B‖` the fold line the planes share, `d_A = (û_A·d, v̂_A·d)` its direction in A's own coordinates and `d_B` likewise, the residual between two constant planes is

`d_A · p − d_B · q + d · (o_A − o_B) = 0`

— one equation: two images of one point agree on their coordinate along the fold line their planes share, and on nothing else, each measured from its own plane's origin. **[0.42]** Where the two origins coincide the last term is zero, and a plane moved square to the fold line — along its projectors — leaves the row as it was, carrying everything drawn in it: the free spacing between a drawing's views is a floating plane's origin. Where either plane is solved for, the row is the projector rule in space over the two lifts, `(n̂_A × n̂_B)·(X_A − X_B) = 0`, and two planes that *come out* parallel are **E065** after the solve. `project` is claimable (§9.7): `claim a project b` asks whether two views are consistent. It is not commutative.

**The block form writes the clause once.** `in top { … }` marks every declaration in its body `in top`; **[0.12]** it stands at the top level of a document and inside a *component* body — over a plane the component was handed, which is how a part carries its geometry for each view in one place, its own `project` statements tying them — and not inside a root block, where a header buried in another statement's span would be a splice no deletion could compose; a `repeat`, `cycle` or `ring` inside it marks the declarations of every copy, so a contour drawn as a chain round a cycle is drawn in the view. The statements are ordinary statements of the enclosing body, and an implementation MUST treat them exactly as if each had written the clause itself — they splice, diagnose and delete as themselves, only the header and the closing brace are the block's, and deleting the plane removes exactly those, leaving the statements standing in space. The block stands at the top level of a document or of a component body (inside a root block, the clause says it one declaration at a time); a declaration inside that writes its own `in`, and a kind with no points of its own, are refused where they stand.

**An instance joins a view whole.** `t := Tooth(…) in top` puts every point-bearing declaration the instance's expansion makes — through nested components and blocks — on the plane: the block's rule, over the statements one statement stands for. A plane, an axis or a curve inside is left alone, having no points of its own to put there. A point aliased in through an argument joins through any body declaration that names it, and one already on another plane is E060; an expansion given two planes — a clause of its own under an enclosing `in` — is refused. An instance inside an `in { … }` block takes the block's plane the same way.

*Non-normative:* the front, top and right views of a part are then three planes — `std.front`, `std.top`, `std.side` — with the part's corners drawn `in` each and tied across them by `project`; an auxiliary view standing on an axis along an inclined face (`aux := plane(u: incline, v: std.y)`) shows that face true-size, and its corners can be placed by projection alone. Each is drawn in its own coordinates, and its drawing places it on the sheet (`sketch aux(m) from m.aux at (10mm, 110mm)`). `rust/examples/bracket.sv` and `bracket.svd` are the worked case.

### 6.8 Faces **[0.18]**

A **planar face** is a region of a plane: an outer loop of existing edges and straight runs between named corners, optionally containing holes. **[0.120]** A face takes no `on:`: every face is planar.

```
sec := face(mouth, side_r, lid, side_l)     // four lines, walked in order
hole_f := face(hole)                        // one circle, which is a loop by itself
bore_f := face(b_mouth, bore_r, b_head, b_axis)
```

A face is a **Declaration** (§4.2). It adds no coordinate, unknown, equation or freedom, and its existing edges keep their own names: naming them is aliasing (P1), not constraint, so deleting an edge deletes the face. A named chain (§6.6) in its list contributes its ordered edges. Its list is a `List` slot like a spline's control polygon, so there is no arity to conjure children from and a bare `f := face` is refused (**E080**, the face's own code saying what a face is, where a bare `s := spline` is §6.1's E103).

**A face has one closed outer loop.** Consecutive edges — and the last with the first — MUST share an endpoint, and sharing is asked of the **points**, which is aliasing and cannot be argued with: two neighbours that share none are **E080** naming both. The loop is walked in the order it is written. A `circle` is a whole loop by itself and MUST stand alone in one (E080); an edge that is neither a line, an arc, a circle, a spline whose knots are clamped (its ends are its first and last control points), **[0.96]** a free curve (§6.1; its ends are the two points it is written with), a stretch of a curve nor a point is E080 at the edge.

**A stretch of a curve [0.28].** A curve (§6.5) has no end points of its own for a walk to meet, so it stands in a face as the stretch between two points: `flank from p to q`, each point held on the curve by a contact (`p coincident flank`), whose parameter is where the stretch ends. The stretch follows the solve, as the contacts do; it is walked from `p` to `q`, against the curve's own sense where `q` comes first. A curve named without `from … to …` is E080, as is a point not held on it, a stretch from a point to itself, or one curve bounding one face twice. **[0.96]** A free curve (§6.1) is the exception: it runs between the two points it is written with, so it stands in a face whole (`face(rope, deck)`), and a stretch of one from its own end is E080 naming that form; a stretch of one between two points held on it is that curve, cut. The implementation mints the stretch as a curve of its own carrying `.closure` (it is the curve already drawn), and it names a face of whatever is swept from the loop by the curve's name.

```
tooth := face(t.r.e from t.r.lo to t.r.hi, t.crown, t.l.e from t.l.hi to t.l.lo, -> close)
cap := face(k from a to b, -> close)                  // a stretch and its chord
```

**A face closes itself [0.19].** The list is a *walk*, and an item may be a **point** as well as an edge: a point is a corner the walk goes straight to and straight on from. Where two neighbours do not already share an endpoint the implementation MUST mint the straight run between them, from where the walk leaves one to where it enters the next; a run between two items standing at the same point is nothing and is not minted. The gap between the **last** item and the first is minted only where the list ends in the marker **`-> close`** — the chain's own word (§6.6), in the other place the language draws a loop — which MUST be the last thing in the brackets and is refused on any other kind. `-> close` on an already-closed loop, including a lone circle, mints nothing.

```
pist_f := face(crown, pL0, pL1, pL2, pL3, pL4, s0.p, -> close)
quad := face(x0.p, x1.p, x2.p, x3.p, -> close)      // four corners, no line drawn at all
```

A minted run is an ordinary line of the drawing carrying the class **`.closure`**, whose shipped rule is `display: none` (§13.2): it bears no design, nothing drew it, and it exists so that a region has a boundary. It names a face of whatever is swept from the loop, as `close0`, `close1`, … in traversal order, skipping names already used by existing edges in the loop.

Three restrictions keep the shorthand from swallowing a mistake. An interior gap between two **edges** is still E080: a point in a list can mean nothing else, while two edges that do not meet are edges listed out of order. And **an edge MUST meet at least one of its neighbours**, since that is what says which way it is walked — `bad := face(a, bc, d, -> close)` has two readings and states neither, and is E080 naming the edge. A straight loop with fewer than three corners is E080 for the same reason a prism swept nowhere is. A single item is a loop only as a circle, or as an edge with two ends closed by `-> close`, which mints its chord **[0.28]**.

**A face of a block's copies [0.113].** An item of a face's walk that names a declaration of a `repeat` or a `cycle` beside it, read bare (`face(side)`, `side` declared in the block), stands for its copies in order, `side[0]`, `side[1]`, …, as many as the block made: a loop drawn by a block is a face without its count written twice, and a block whose count is a walk's (§12.8) makes a face of that many edges. Each such edge names the face swept from it by its copy, `side[2]`. A name the face's scope finds as it stands is itself.

**A face carried whole [0.118].** `sec := prof mapped(from -> a, to -> b) in n` is the face `prof` carried into the plane `n` by a map, its pairs written as a point's are (§9.2): its loop walked as a block walks it (§12.8), each corner carried as a point is and each edge rebuilt between the images, in `n`. The image keeps the source's names under its own: a face of corners is `face(sec.k0, sec.k1, …, -> close)`, so the faces swept from it are named `close0`, … as the source's are; a face of edges names each image edge as its source (`sec.ab`, swept as `ab`). A line is carried to a line. Under the similarity of two pairs an arc is carried to an arc about its centre's image and a circle to a circle, its radius scaled by `|b − a| / |to − from|`; under the affine map of three pairs a circle would be an ellipse, and an arc or a circle there is **E080** naming it, as is a spline or a curve. An edge's image runs from the image of its start — a line's `p1`, an arc's `start` — the walk's own reading. **The mirror**, `mirror: true`, reverses the turn, so the image is walked the other way round and an arc runs from its end's image: it winds as its source does, and a handed pair is one profile and its image. Three pairs whose targets stand in the opposite turn reflect too, and their image winds as the targets do. A face of holes is not carried yet (**E080**).

**A face may have holes.** After its outer boundary, `holes:` introduces one or more circles, named closed loops or faces, separated by commas. Each entry is a complete inner boundary; an open chain is refused (E080). A face is a loop, so it is a hole as it stands — its outer loop, which is how a hole of loose edges is written (`h := face(e, f, g, k)`, then `face(a, b, c, d, holes: h)`); a face with holes of its own is refused, and the hole's face is declared before the face it is cut from. All boundaries MUST lie in the same plane. After solving, every hole MUST be simple, nonzero and strictly inside the outer boundary, and holes MUST be mutually disjoint: touching, crossing, overlapping and nested holes are refused. Loop direction does not determine whether it is a hole; `holes:` does. The final `-> close`, when present, closes only the outer boundary.

```solvent
annulus := face(barrel, holes: core)
perforated := face(outline, holes: bolt0, bolt1)
groove := solid(annulus, from: z, to: z + width)
groove cut body
duct := solid(face(outline, holes: inner.profile), depth: length)
```

A section's holes travel through its entire sweep, including `depth:`, `from:`/`to:`, `through:` and full or partial revolutions. The declaration creates the swept region; applying it to another body remains a separate `cut` or `union` statement. A circular boundary names its wall by the circle’s path relative to the component containing the face (for example `duct.left.bore`). A hole supplied by a named loop qualifies its side names by that loop's path (for example `duct.inner.profile.wall`), relative to the component containing the face. References to repeated holes keep their indexed source paths (for example `duct.bore[0]`), without internal expansion IDs. Boundary names MUST be distinct; sweep cap names remain reserved. The source edges retain their identities, so holes do not require names derived from Boolean-generated topology.

**A face lies in one plane, and does not say so.** Its plane is read off the **memberships** (§6.7) of every point of every edge; those MUST agree, and a dissenting edge is **E080** naming it and both planes. Nothing is written on the face itself. A face bears no points of its own, so an `in` clause on one is refused where it stands — but a face **inside** an `in … { }` block is left *unstamped* rather than refused, exactly as a plane and a curve are: a block stamps the geometry the face is written over, the face is on the plane its edges are on, and refusing it would put a design and the region taken from it in two different blocks.

### 6.9 Solids **[0.18]**

A **solid** is a face swept, or a term over other solids.

```
block := solid(sec, from: face, to: back)         // a prism between two ordinates
boss := solid(boss_f, depth: 10mm)                // `depth: d` is `from: -d, to: 0`
bore := solid(bore_f, about: ax)                  // a full turn about a line in the face's plane
cup := solid(cup_f, about: std.z)                 // or an axis lying in it [0.67]
lug := solid(lug_f, about: ax, sweep: 90deg, sense: cw)
body := solid(block)                              // a body, whose stock is `block`
bore cut body                              //   ... less the bore
boss union body                                //   ... plus the boss
blank := solid(R)                                 // a region: the points inside the set R
```

**A region is a solid** **[0.62]**. `solid(R)`, its one operand a set, is the solid of the points
inside `R` (§6.21): `R`'s body applied to a point, each number it bounds there a bound of the
solid — a distance from a point a ball, from a line a cylinder, an angle from a line seen from its
start a cone, an ordinate along a plane's normal a half-space. So `solid(std.Sphere(c, r: 5))` is
the ball, and `solid({ p | p inside tip; p inside heel; p outside toe; p outside back })` the
hypoid's blank. Where its bounds share one axis — balls centred on it, cylinders and cones about
it, planes square to it — and enclose material all round, it is one revolution of a region in the
half-plane through that axis, built exactly (planes, cylinders, cones, spheres, tori); **[0.69]**
where they share none it is read point by point (below). That is judged where the solid is
evaluated, on the solved drawing, as a face's loop is: a cylinder alone, or bounds with no point in
common, are refused there. A set whose body states
anything but bounds (a disc's plane) is **E040**. A region solid is a solid like any: `cut`,
`common`, `union`, swept and placed. **[0.63]** A bound's number may read one unknown of the drawing
(`std.Sphere(c, r: hint(50mm))` with `q coincident` it): the solid reads it as solved, and refuses one
nothing determines. A cone's half is the unsigned angle its row reads, so one solved past a half
turn is the cone it reads as. **[0.64]** A bound offset (`p distance(<= d, side: outside) S`, §6.21) is the set's surface
grown by `d`, or shrunk where `d` is taken off: a ball's or a cylinder's radius moved, a plane moved
along its normal, and a cone's offset exactly — the cone whose surface stands `d` from it, its apex
further back where it grows, rounded there by the ball of radius `d` about the apex (the points
nearest it), further along where it shrinks.

**A region in general position** **[0.69]**. A region whose bounds share no axis — a ball off a
cylinder's line, two cylinders crossing, a bound on measurements, a face's side (§6.21) — is
read **point by point**: each bound the signed distance from its surface, the region where the
largest is no more than nothing (an intersection). It is a solid like any, judged where it is
evaluated, and its material is bounded by the planes its bounds lie within (a ball's box, a
cylinder's four planes about its line, a cone's four through its apex, a half-space); one bounded
in no direction is refused there, unless it reads a body's faces, which bound nothing a `cut`
needs. *Non-normative:* such a region's surface is the reading's, meshed where it is asked for; an
exact boundary of one is not built yet, and an exact export refuses it in those words.

**Rounds and chamfers** **[0.69]** are words of the standard library over §6.21's sets, not
features of the kernel. `A round(r: 2mm) B` rounds every edge where a face of `A` (a face path,
§6.21) meets a face of `B` by a ball of radius `r`: where the edge is convex the material the
ball rolled along it cannot reach is cut away, where it is concave what the ball cannot reach
outside both faces is filled, read off the solved geometry, never said; where three such edges
meet at a corner, the ball touching all three. `round(r: 2mm) X` is `X round(r: …) X`, every
edge among `X`'s faces. `A chamfer(d: 1mm) B` cuts every such edge by the flat `d` along each
face, measured from the other face's surface, and `chamfer(d: 1mm) X` every edge among `X`'s.
Faces that do not meet make nothing, so `block.near round(r: 2mm) block` rounds the near rim
alone, its runs mitred where they cross at the corners. Each is a relation word whose body makes
solids from sets over the faces (`std.Run`, `std.Fill`, `std.Corner`, `std.CornerFill`,
`std.Flat`, `std.FlatFill`) and joins them to the body the faces are of (§9.9), so a document may
define its own the same way. **The stratum**: a term of a body that reads that body's faces reads
the body as its other terms make it — a round reads the faces as the cut it rounds left them,
never its own or another round's — and acts on the body after them, what it adds and then what
it takes; a body whose stock reads its own faces is refused where it is evaluated. In a region's
own body a round or a chamfer between two of its bounds is one more bound on its point: `p inside
std.Rounded(wall, top, r: 2mm)`, `p inside std.Chamfered(wall, bottom, la: 1mm, lb: 1mm)` (`la`
along the first, `lb` along the second). Where the two are bounds the region states, at its
level and about its axis, the round or the flat is built exactly in the region's revolution (a
torus; a cone between two straight sections, and **[0.73]** between a straight section and a
circle, a sphere's, the conic the sum of distances makes there, turned); elsewhere — a flat between
two circles — it is read point by point. **[0.73]** A chamfer's flat is what its sum of distances
bounds and nothing else: it bows with a curved surface, and no construction substitutes the
straight line between its legs' ends. *Non-normative:* both readings are one
meaning: a round lowered into the revolution is the region the point-by-point reading reads. So
is a round over a body's faces an implementation recognises by its set and builds exactly (a
prism, a ring, a ball's corner), held to that reading.

**The brackets are what the thing is made of** (§4.3, §6.2), so the sweep stands in them beside the face: `from:`, `to:`, `depth:`, **[0.80]** `draft:`, `through:`, `about:`, `sweep:` and `sense:` are labels of the constructor and are neither seeds nor constraints (**[0.100]** `along:` is retired: below). A mixture of sweep forms, a half-written prism (`from:` with no `to:`), `from:`/`to:` beside `depth:`, and `sweep:` or `sense:` with no `about:` are each refused where they are written, with the shapes a solid has.

**A face may be written where it is used.** A swept solid MAY take an anonymous `face(…)` in place of a face reference. Its boundary follows every rule of §6.8, including points and `-> close`, and resolves names in the solid's enclosing scope. It introduces no public name and owns no unknown; the solid owns the section and any closing lines it generates. Every sweep form accepts it. A sweep takes exactly one face, and a body takes solids. A section used by several sweeps can retain a named `face` declaration.

```
block := solid(face(mouth, side_r, lid, side_l), from: face, to: back)
```

**Every numeric extent is an expression, and MUST NOT be an unknown.** A number in a solid's brackets is settled by the flattener over the numbers in scope — a value, an input, a formal (§5) — and is then document data no solve moves, checked against its slot's dimension (`Length` for a prism's ordinates, `Angle` for a sweep; **E103** otherwise). A solid allocates no parameter, so P3's other half holds without a rule of its own: there is nothing here for a solve to rewrite.

- **A prism** runs `from:` one signed ordinate `to:` another **along the face's own plane normal**. Those signs are arithmetic and not a convention (§9.2 **[0.17]**) — they are ordinates on an axis, and a document writes both. `depth: d` is the draughtsman's spelling of `from: -d, to: 0`, the material *behind* the face the view shows, and is therefore a **magnitude**. A prism swept nowhere (`from` equal to `to`) is **E080**.
- **[0.80] A drafted prism** adds `draft: θ`, an `Angle`, to a prism's `from:`/`to:` or `depth:` and to nothing else (refused where written). Its sides lean by `θ` from the face's own plane, the **parting plane** (ordinate 0, wherever the prism's ordinates are): the section at ordinate `h` is the face's loops each moved `|h| tan θ` into the face's material, every side along its own normal — a line square to itself, a circle's radius less (a hole's more) — and each corner where the moved sides either side of it meet. So the section shrinks away from the parting plane on both sides of it, as a part comes out of a die split there; a negative `θ` leans the sides the other way. A straight side is a plane and a circle's side a cone. A side the move takes to nothing, or turns round, within the prism's ordinates refuses the solid where it is evaluated; `|θ|` of a right angle or more is **E080**. Faces keep the prism's names (`.near`, `.far`, a side per edge, §3.2): a side crossing the parting plane is two pieces under its one name, which a round over that side's faces (§6.21) takes as one face, so the crease between them stays sharp. A drafted side leans off its face's plane, so it is not yet a set to draw on (**E040**; its caps are).
- **A through prism** is written `tool := solid(section, through: target)`. Its target MUST be a solid (**E080**); it spans the target in both directions along the section's own plane normal. The extent is evaluated after the sketch is solved: recursively collect the target's stock and additions, ignoring all cuts, project a conservative bound of that material along the normal, and pad both ends outside the material at the kernel's tolerance. The target may be a swept solid or a body. Changes to its material geometry or placement change the extent; subtractive tools do not enlarge it. `through:` cannot be mixed with any other sweep label, and does not itself subtract anything. Apply the tool separately with `tool cut body`.
- **[0.88] A rolled sweep** is written `removal := solid(tool, under: m, through: body)`: the tool swept under the motion `m` (§6.14) over the whole roll about 0 during which it meets `body`'s material, `body` a solid (**E080**). As for a through prism, the material is the target's stock and additions, ignoring all cuts, so `removal` may cut `body` (placed copies included) without a cycle; using it as the body's stock or a `union` with it is **E041**. The roll is evaluated after the sketch is solved, marching out from 0 each way until the tool, over a whole step of the roll, is proved clear of the material: conservative, so it may run a little past where the tool first clears and never stops short, and at each end the tool is clear of the body. One where the tool does not meet the body at 0, or is not clear of it within two turns either way, or whose tool or body cannot be read as a static field with a bounded support, is refused where the solid is evaluated, naming why. A generating sweep's roll limit is then no design number: within the window that clears the blank at both ends, the cut does not depend on it. `from:`/`to:` remain for a roll stated outright (a sweep over a whole period, or a negative control), and cannot be mixed with `through:`. A placed copy (`solid(removal, under: index, at: θ)`) moves the sweep whose roll was found against `body` as it stands.
- **[0.97] A face family** is written `duct := solid(Station(guide, …).f over s in (a, b))`: the
  solid the face `f` of an instance of `Station` sweeps as its numeric formal `s` runs from `a` to
  `b` — §6.5's curve one dimension up. The instance MUST be written in place (a drawn station is a
  syntax error), MUST leave `s` unbound (**E103**), and `s` MUST be one of its numeric formals
  (**E103**); `a < b` (**E080**). The instance is never drawn: no stroke, pane or callout of it
  reaches a page. `s` is **held**, never an unknown of the drawing: the station is solved with
  the drawing at the middle of the interval, and at every other station `s` stands at its number
  and the instance alone is solved again, the drawing held where it stands, each station starting
  from its neighbour, so the branch the drawing chose is the one carried. The instance's body
  reads as a traced body's (§6.5): its numbers MAY be any expression in `s`, and its dimensions
  MAY read a number of the geometry it was given (`rim.r`, `a.x`) as that geometry stands, but
  not its own (**E103**). **[0.113]** A number the station gives a relation word or a component
  MAY be any expression of such numbers (a product, a quotient), worked out at each station; a
  dimension whose unknown's coefficient comes to nothing there is the number it comes to. Every
  station inside the interval MUST be held — solved, with no degree of freedom and no equation
  lost — and keeps the station's topology: a loop crossing itself, an
  edge lost or turned over and a hole touching its section are refused at the station where they
  happen, as is a swept volume crossing itself (a section reaching past its path's centre of
  curvature). Only an end MAY lose an edge, or the whole section to an apex; an end that loses
  rank is read as the limit of the stations closing in on it and is not solved. Correspondence is
  identity — the edge `ab` at one station is `ab` at every other — so the walls are named by the
  section's edges under the instance (`duct.sec.ab`, a hole's `duct.sec.inner.loop.ab`) and the
  caps `start` and `end`; an end that vanished has no cap. *Non-normative:* an implementation
  builds each wall exactly — the edge's own form along it, the station solved at points of each
  piece of the interval and a polynomial in the running number between them, held to a station
  solved between them — and writes a wall that lies on a plane, cylinder, cone or torus as one.
- **[0.100] A loft is a face family over a station word.** What a loft or a sweep along a path
  means is written in the library, never in the constructor. A station word of the standard
  library is a component holding a plane `n` square to its path where its parameter is `s`, its
  `v` along an axis `up`: `OnLine(g, up:, s:)` at the fraction `s` of a line's run, `OnArc` of an
  arc's turn, `OnSpline` at a spline's parameter, `On` over a `path` whose tangent at a point it
  reads in space (a line, a spline, a traced curve), and `Helix` and `OnHelix` along a screw
  (0.110; through 0.109 `Screwed` and `OnHelix`, each about an axis given apart from its base).
  The section is the document's own component drawn `in st.n`, reading `s` where it runs (a size
  `inlet + (outlet - inlet) * s`, a corner's coordinates, a turn): `duct := solid(Reducer(guide,
  std.z, inlet: 24mm, outlet: 12mm).sec.profile over s in (0, 1))`. Two
  sections correspond by being one, so no pairing rule is written anywhere. `solid(section,
  along: guide)` and `solid(start_section, end_section, along: guide)` — through 0.99 a section
  carried along a line or an arc, or two joined edge to edge in written order — MUST NOT parse;
  an implementation SHOULD say, at `along:`, what the spelling became. Pairing two faces drawn
  apart has no word yet (0.113: `Ruled`).
- **[0.102] A path is any guide drawn in a plane, a chain among them.** `On` stands on an arc as
  on a line, a spline and a traced curve: a line touching an arc or a circle at a point of it is
  drawn in the round's plane, as one touching a spline is (§6.21). Its frame is held to `up`;
  along a guide in one plane with `up` that plane's normal, that is the rotation-minimising frame.
  **[0.123]** `On`'s frame is the rotation-minimising one, stated as `Minimal`'s rate and
  integrated from the family's home, where `v` leans toward `up` and `u` is square to it: `up` is
  read at the home alone, so along a guide that leaves its plane (a curve traced in space, §6.5)
  the plane stays square to the guide, where an `up` held at every station would lean it. Along a
  guide in one plane with `up` its normal the two frames are one.
  A `path` formal of the station's instance MAY be given a **named open chain** of lines and arcs
  (§6.6): the family runs over it **link by link**, link `k` while the running number is in
  `(k, k + 1)`, so it runs within `(0, n)` for a chain of `n` links. On link `k` a contact pinned
  to `s` on the chain stands at the fraction `s − k` of that link's run (an arc's of its turn);
  every other reading of `s` reads it whole, so a law in `s` runs along the whole chain. The
  stations on a link start where those of the link before end, so the section is carried across
  each joint, and a wall keeps its one name over every link. The running formal of a family over
  a chain MUST be a `Scalar` (**E103**), its interval within `(0, n)` (**E080**) and the chain
  open (**E080**: its ends would meet); past the first link, a contact on anything but a chain it
  runs pinned to `s` is **E103**. **[0.111]** Every `path` formal given a named open chain is run
  so, all **in step**: on link `k` each is given its own link `k`, so the chains MUST have as many
  links (**E080** naming both). **[0.124]** A chain MAY run across planes: a line touching an arc
  drawn in another plane, its ends drawn anywhere, is the tangency in space (§9.2), so a bent pipe
  is one chain. Where the station states a rate (`On` does), a station on a link past the first
  is the one the link before carried across the joint: the drawing does not solve it, and its
  `initially` rows hold at the family's home alone. **[0.127]** An arc over points in space faces
  the plane it is drawn in as its points do (§6.4), so a chain drawn in space turns either way at
  an arc. **[0.128]** In a plane an arc runs counter-clockwise from its start unless it says
  otherwise, `arc(center: c, sense: cw)`: it then runs clockwise — drawn counter-clockwise in its
  plane turned over — so a chain in a plane turns right at it, entering it at its start as every
  link. Its ends a half turn apart are **E060**; `sense:` of an arc over points in space, which
  turns as its points stand, is **E040**. **[0.129]** A chain MAY enter an arc at its end — its
  end the point the link before leaves at, or, first, its start the point the link after enters
  at — and walks it from there to its start: the same arc, drawn from its end and turning the
  other way, so a family runs it forward. Threaded, both of its ends are written (`-> tangent (k
  := arc(center: c, start: f, end: e)) ->`, entered at `e`); a name still reads the points as
  written (`k.start` is `f`). An arc two chains walk opposite ways is **E080**. A seed's step MAY run along any path, `hint(at: p, along: g, by:
  f)`: `f` times `g`'s rate along its own run where `p` meets it, as along a line it is `f` times
  its run from `p1` to `p2` (§6.4).
- **[0.103] A law is an expression in the running number.** A twist or a scale is a dimension of
  the section reading `s`, and any expression in it is a law (`twist * (3 * s^2 - 2 * s^3)`, `r *
  max(1 - s, 0.5)`). A word of the library takes numbers, not laws, so the standard library's take
  a total and run it linearly from the section as drawn at `s = 0`: `Twisted(n, twist:, s:)`, axes
  `axes` in the plane `n` turned `twist · s` about its origin; `p scaled(du:, dv:, scale:, s:) P`,
  `p` at `(du, dv)` in `P`'s frame scaled `1 + (scale − 1) · s` about its origin; and
  `scaled_radius(r:, scale:, s:) k`, a round's radius scaled alike. Any other law is the word given
  another fraction (`s: 3 * s^2 - 2 * s^3`) or the section's own expression. No word takes a
  `law:`. **[0.118]** A section is drawn once and carried whole by the map taking its frame to the
  station's (§6.8): `Twisted(n, twist:, reach:, s:)` is the one point that turn needs, `t`, `reach`
  along `n`'s `u` from its origin turned `twist · s` (`sec := prof mapped(o -> n.origin, x -> tw.t)
  in n`); through 0.117 it was a plane of axes `axes`, which a section was drawn on corner by
  corner; **[0.138]** it is the turn of `n` about its origin read at that roll (§6.14), `t
  coincident(under: turn, at: twist * s) x`. A number given a formal — a component's or a relation
  word's — MAY be any expression in
  the running number, worked out at each station; it binds the formal, and is no unknown of the
  drawing. A `when` is made once, for the whole run (§12.9), so a condition reading the running
  number is **E040**, saying so; `min`, `max` and `abs` write a law in pieces.
- **[0.110] A screw is a motion read at a roll [0.138].** `Helix(base, pitch:, turns:, s:)` is the
  frame of the plane `base` turned `360deg · turns · s` about `base`'s normal through its origin —
  the screw's axis — from `base.u` toward `base.v`, and risen `pitch · turns · s` along that normal:
  `n`, the plane `base` so carried, and `axial`, the plane through the axis whose `u` runs out along
  `n.u` and whose `v` runs up the axis. A section drawn in `axial` sweeps a thread, one drawn in `n`
  a twisted prism. `OnHelix(base, r:, pitch:, turns:, s:)` is the plane `n` square to the helix of
  radius `r` that screw draws, its `u` out from the axis and its `v` up the axis leaned by the
  helix's lead angle, so a section drawn in it sweeps a coil. Neither is held by a seed, and a
  negative pitch winds left-handed. **[0.138]** Both are frames read at a roll (§6.14): `n` is
  `base` carried by a frame's screw (`std.Turn` about `base.normal`, `advance: pitch`) to `360deg
  · turns · s`, `axial` that frame turned a quarter about its `u`, and `OnHelix`'s `n` the frame
  square to the helix where it starts — `base` turned about `base.u` and slid out `r` — carried by
  the same screw; through 0.137 each was points placed by `cos` and `sin` ordinates in
  `base`'s frame. `Screwed` (through 0.109 the plane through an axis given apart from `base`) is
  retired; `Helix(…).axial` is that plane. **[0.138] A face family carried by a motion is that
  motion's sweep**: where every relation of the station that reads its running number is a point the
  one motion carries (§6.14) to a roll affine in it, and the motion is a turn or a screw about one
  axis or a slide along one line, the section at the interval's start carried by that motion over it
  is the solid — so a wire drawn square to a helix and a ball swept along the helix are one tube.
  The recognition is by what is stated, never by a word's name. *Non-normative:* an implementation
  lays each wall on the surface its first edge sweeps under that motion and each rail on its
  corner's path, once they agree with the general walls; where no statement says, it MAY find such a
  motion from the stations.
- **[0.111] A loft through several sections is drawn on rails.** A section's corners correspond
  by rails, one a corner: its places in order, one a section. The station's corners stand where
  their rails meet its plane (`p := point in st.n`, `p coincident rail`; a rail drawn in a plane,
  the corner drawn in both, `point in st.n, front`). A **ruled** loft's rail is a named open chain
  of lines from section to section, and its guide a chain of as many links, cut at the sections:
  the family runs them in step (§6.9 [0.102]), so the loft is ruled piecewise in the running
  number, section `k` at `s = k`, and its walls meet there at an angle — an edge of the solid, the
  wall one name over its pieces. A **smooth** loft's rail is the spline through the same corners
  (§6.1 [0.111]), its end conditions the spline's tangents there — along the guide (normal to an
  end section), or along a neighbouring face's edge (tangent to it). A loft with **no guide**
  stands its station on its rails: the standard library's `Across(a, b, c, s:)` is the plane
  through the places at `s` on three rails, its `u` from the first toward the second and its `v`
  toward the third. *Non-normative:* where the walls of a family meet at an angle across its
  interval an implementation builds its solid in bands between those sections, each wall a face a
  band.
- **[0.113] Two faces drawn apart, and a section held to its rails.** The standard library's
  `Ruled(a, b, s:)` is CAD's loft of two sketches: walking the faces `a` and `b` in step (§12.8),
  it joins corner `k` of each by a straight rail and stands its section where the plane through
  the first three rails at `s` cuts every rail, so its ends are the faces themselves; `Apex(a,
  tip, s:)` joins `a`'s corners to the point `tip`, the solid ending in an apex. Correspondence is
  the walks' written order and nothing else: a **connector** is where a walk starts
  (`face(b1, b2, b3, b0, -> close)` pairs `a`'s first corner with `b1`), and two faces of
  different corner counts are made alike by **naming a point on an edge** of the shorter walk —
  walks of different lengths are **E080**, naming both. `Similar(prof, from, to, n, a, b)` is the
  face `prof`, drawn once anywhere, carried into the plane `n` by the similarity taking `from` to
  `a` and `to` to `b`; `Affine(prof, from, to, third, n, a, b, c)` by the affine map taking a
  third point to `c` as well. Each corner keeps its place in the frame those points make, read
  off the drawing, so in a station holding `a`, `b` (and `c`) where two (three) rails cross its
  plane, the section follows its rails: two give a similarity, three an affine map. Through
  0.112 two faces drawn apart had no loft, and a section reached its rails corner by corner.
  Lofting an arc edge with a line edge is not said yet. **[0.117]** Each corner is one
  `mapped` (§9.2), `q mapped(from -> a, to -> b, third -> c) e.p1`, and two pairs for `Similar`;
  through 0.116 the map was written out in coordinates, each corner two `offset`s through a hidden
  point over hidden spans of the frame (`Framed`, `Spans`, retired). **[0.118]** Each is one
  statement, the face carried whole (§6.8): `f := prof mapped(from -> a, to -> b, third -> c) in
  n`, its walls named by `prof`'s edges.
- **[0.116] A station may state a rate.** A relation MAY read the **rate** of an axis's direction
  or of a point along a formal of the body it is written in, `rate(x over s)`, which always names
  its formal: `rate(f.u over s) perpendicular f.v`. A rate stands where a direction is read — on
  the left of `perpendicular` or `parallel`, against an axis, a line or a plane — and never as a
  place or a magnitude; a word that reads no direction is a syntax error. **[0.119]** A rate is an
  operand, so it may stand on either side, and a directed `angle` and `distance(k, along: l)` read
  one too, a traced body's alone (§6.5): in a face family either is **E040**. Every such relation is linear
  in the rate. In a face family whose running number the formal is given (or a multiple of it), a
  relation over a rate is no row of the drawing: the drawn station is the family's **home**, its
  initial value, solved with the drawing, and every other station is the station **integrated**
  from there along the running number, in both directions — its relations differentiated along
  `s` and its rate relations together determine how it moves, and that MUST hold at every station
  (a station whose rates leave it a freedom is refused, saying how many). What holds at the home
  alone is said in **`initially { … }`**, a block of relations in the station's body that is a row
  of the home's solve and of no other station's; a station with an `initially` and no rate is
  refused. A formal given a number is a drawn instance at its home, where no rate is a row. A
  formal that is an unknown of the drawing is **E040**, and so is a rate of anything but an axis or
  a point, or one held against anything but a direction or a plane. The standard library's
  `Minimal(n, up:, s:)` is the rotation-minimising frame of the station `n`: the plane `f` through
  `n`'s origin square to the path as `n` is, `f.u`'s rate square to `f.v`, and `f.u` square to `up`
  initially; along a guide in one plane with `up` its normal it is `On`'s frame, and along a
  twisted path it turns against the path's Frenet frame at the path's torsion. **[0.125]** `Frenet(g, s:)` is **Frenet's frame**
  of a station on a curve traced in space: the plane square to `g` at `s`, `u` toward the centre
  of curvature (its sense the centre's, no seed's) and `v` the binormal; along a helix it is
  `OnHelix`'s plane. A station where the curve has no curvature is refused there. **[0.123]** A
  station that states a rate MAY run over a chain: on each link after the first its home is where
  the link before ended, the station carried across the joint, its `initially` rows no rows there.
  A station read off its integration meets its relations: only what its rates hold is the
  integration's, so a number over the running number that bends (`max(1 − s, 0.7)`) is read as
  written. *Non-normative:* an implementation integrates
  the station to well under "solved" and builds its walls through the integrated stations.
- **A revolution** turns a face about `about:` a line or an axis, which MUST be one of the two (**E081**) and MUST lie in the **face's own plane** (E081) — a line drawn in another view names a direction this face knows nothing about. **[0.67]** A line lies there by membership; an axis where it is one of the plane's own axes (`std.z` on `std.front`), or, held entirely by `fix` (the plane too), where it stands now — else, placed by the solve, where the solid is evaluated, which refuses the solid if it does not. `sweep:` is how far and is a **magnitude**, a full turn where the document writes none; **which way is a word, not a sign** (§9.2, §9.4): `sense: cw | ccw`, right-handed about the line's own `p1 → p2` (an axis's own direction) unless `cw` is written, and a negative `sweep:` is **E040** at the value, in the words of the selector that replaces it.
- **A body** names its **stock** in the brackets and takes its features from the **[0.32]** `union`, `cut` and **[0.22]** `common` **[0.85]** statements that name it. A solid whose brackets hold solids and no face is a body. **[0.99]** A solid whose brackets hold nothing, `pegs := solid()`, is an **empty body**: its stock is the first solid in `union` with it, so `B(s)` below is read with `S(s)` empty; which is first is no matter, the union being a set. A pattern then joins every copy alike — `repeat n as i { peg[i] union pegs }` — with none written apart as the stock. An empty body nothing is in `union` with is **E080** at its declaration, whatever cuts it. **[0.31]** A swept solid that a `union`, `cut` or `common` statement names is a body too, whose stock is its own sweep: the name means the sweep with its features wherever it is read, the sweep's faces keep their names (`plate.near`, not `plate.plate.near`), and a feature's are reached through it (`plate.hub.near`). A document wanting the bare sweep and the finished part under two names writes `body := solid(plate)` and puts the features on `body`.

**The body rule, and the whole of it.**

> **A solid is its stock, plus everything in `union` with it, minus everything that `cut`s it, within everything it is in `common` with.**

As a point set, with `union(s)`, `cut(s)` and **[0.22]** `common(s)` the three **sets** of statements naming `s`, and `S(s)` its stock:

```
B(s) = ( S(s) ∪ ⋃ { B(x) : x union s } ) ∖ ⋃ { B(y) : y cut s } ∩ ⋂ { B(z) : z common s }
```

Union first, and none of the three groups is ordered; difference and intersection commute, so the last two need no order between them either. `tip common blank` keeps of the blank what lies within the tip cone, which is what a rim inside a cone is, and what `blank ∖ (blank ∖ tip)` used to spell in two statements and an intermediate. `union`, `cut` and `common` are Declaration-class (§4.2, §9.2): each says what its right operand *is*, contributes no residual, and enters no solve. **[0.85]** Through 0.84 `common` was spelled `bound`, which a document MUST NOT write now: an implementation refuses it naming `common`. A solid that reaches itself through its operands is **E041** — "made of itself" — reported at the body statement that closes the cycle where one does, which then contributes nothing.

**Extent dependencies are separate from Boolean operands.** A through prism reads only the target's additive material sources for its bounds; it does not read the target's final Boolean result. Therefore `tool := solid(section, through: body)` together with `tool cut body` is valid and unordered. Genuine material/extent cycles, such as using that tool as the body's stock or adding it in `union` with the same body, are **E041**. A cutter remains a finite named solid with its own reports and faces. Its padded caps are numerical extent boundaries, not design datums.

**`cut` replaces the old Boolean `through` spelling.** `pocket cut body` subtracts the pocket, whether its depth is blind or through the part. `X through B` is refused with a diagnostic directing the author to `X cut B`; `through:` is only an extent label. Exact blind-cut endpoints retain their declared values: this feature does not silently extend cuts that end on internal walls.

**Why a term and not a feature tree.** A feature tree is imperative because it is *stateful*: step *n* acts on the anonymous body as of step *n−1*, and names faces by the order they were made in. Solvent names everything, so the order lives **inside the term**, over names, exactly as it lives inside `h = w / 2`; between statements there is none, which is P2. `bore cut body` says what `body` is and may stand anywhere in the file — above the declaration it qualifies, below it, or in a component beside it — and moving it changes nothing.

A design that wants the other order **names the intermediate**, and then there are two solids because there are two things:

```
recess := solid(pocket)      // the pocket, less the boss standing in it
boss cut recess
body := solid(block)
recess cut body       // block − (pocket − boss), which the flat rule cannot spell
```

**A solid's faces are reached by path, never by index** (§3.2). A prism's caps are `.far` and `.near`, the lower and the higher of its two ordinates along the normal, so `depth: d` leaves `.near` the face the view shows; each side is named by the edge it was swept from (**[0.80]** a drafted side crossing its parting plane, in two pieces of that name), in the name the *source* wrote (`block.side_l`). A revolution names its wall by the edge likewise and its caps `.start` and `.end` where the turn is partial. **[0.100]** A face family has `.start` and `.end` caps and walls named by its section's edges under the instance. Through a body the operand keeps its own name (`body.block.near`). An implementation MUST NOT name a face by its position in anything: that is §13.1's rule, and a boolean is exactly the operation that would renumber one.

What a report says about a solid (§16.3) is therefore `NAME.volume`, `NAME.area`, `NAME.bounds.{x,y,z}{0,1}`, and `PATH.area` for each of its faces that survived — a bore that ate a cap leaves a name the document still writes and no area behind it, which is a fact and not an error. Those numbers MUST be taken at a faceting the **document** fixes and never at the screen's: a volume that changed with the zoom is a number nobody could quote.

**A solid stands on no plane.** It bears no points, so `in` on one is refused where it stands and an `in … { }` block leaves it alone (§6.8). It is not picked, dragged or dimensioned on the sheet, and it is evaluated after the drawing is solved (§1.2, §3.1).

*Non-normative:* `rust/examples/vtwin/components/cylinder.sv` is the worked case — one section in the plane of swing, the body swept from it, and the bore, the port, the bolt hole and the head's slot four more solids that `cut` it. The part had been written three times, the same body redrawn in two more views as page-aligned rectangles re-tied by `project`, with every depth ordinate related to the section by no statement at all.

### 6.10 Planes stood off **[0.18]** **[0.42]**

**`P distance(d) Q` stands `Q` off `P`**: `Q`'s origin is `d` along `P`'s normal, `(o_Q − o_P)·n̂_P
− d = 0`, signed, positive on `P`'s out side. It is one row, and says nothing of `Q`'s attitude;
over planes whose axes are parallel it is two parallel planes `d` apart, which is what 0.18's
`offset:` said:

```
deck := plane(u: std.x, v: std.y)         // [0.43] at the world origin, where its axes meet
lid := plane hint(origin: (0, 0, 12mm))
lid.u parallel std.x
lid.v parallel std.y
deck distance(12mm) lid
n := axis
n perpendicular deck
n coincident deck.origin
n coincident lid.origin
```

The relation takes `lid`'s one freedom across `deck`, and `lid`'s origin could still slide within
`lid`; the axis `n`, square to both and through both origins, stops it. The language adds no sugar
for a stack: a library component says it in one line.

**[0.42] `from:`, `offset:` and `against` are withdrawn.** 0.18 derived a plane `from:` another,
stood off it by a constant `offset:`, and placed a plane written `from: P` alone by a mate between
two faces of solids, `F against G`, refusing a stack that contradicted itself as **E083**. A plane
is now two axes and a place, and a place is stated by relations, so all three are refused where
written and E083 is gone. A contact between parts is stated over floating planes with the
relations of §6.7 and this section, or by a library component over them.

### 6.11 Views and sections **[0.20]**

Views and drawing sections belong in [Solvent Drawing](docs/solvent-drawing.md), not in
model source. A model plane describes geometry. Viewing direction, page placement, and scale
belong to the drawing. Removing a drawing leaves the complete model and solution unchanged.
**[0.42]** A plane carries no place on paper: a drawing's `sketch NAME(m) from m.P at (X, Y)`
draws the geometry drawn in plane `P` in `P`'s own coordinates at that place on the sheet, and a
sketch with no `from` draws `std.front`'s.

### 6.12 Dimensions as output **[0.20]**

A model's dimensional constraints and assertions remain model statements. Selecting and
placing their annotations, or requesting measurements of solved geometry, belongs in `.svd`.
A drawing MUST NOT add model unknowns, equations, or branch choices.

### 6.13 Analytic surfaces

```
flank := surface(crown, edge: rack.outer)
root_transition := surface(crown, edge: rack.outer_round)
```

**[0.120]** A `surface` names a side of an existing solid, and has two meanings, one per kind of
solid it is read from. Over a **revolution** it is the side one profile edge sweeps round, the
analytic patch this section specifies, and what a spatial envelope over it reads as its source,
held to by the solve (§6.15.3). Over a **prism** drawn in a view it is the side its edge extrudes,
whose envelope under a motion keeping the view is a planar envelope the solve holds to
(§6.15.2). Neither is a certification object read only after the solve.

Its arguments are `solid` and
`edge`, in that order; labels may be written as with other constructors. The initial supported
source is an unmodified revolution, with a line, circular arc or circle on its profile's
outer or hole boundary. An unrelated edge, a modified body, or an unsupported source MUST
be refused. This declaration does not create new coordinates or approximate the surface
with a mesh.

The normalized parameter `u` traverses the declared source edge from 0 to 1, and `v` traverses
the declared revolution from 0 to 1, including its sweep angle and sense. Evaluation returns
position and both first partial derivatives in world coordinates. Their cross product follows
the edge and sweep directions; it MUST NOT be interpreted as an automatically outward material
normal. Trim and material orientation remain separate properties.

An optional pair of angular bounds selects a source span:

```
flank := surface(crown, edge: rack.outer, from: 180deg, to: 360deg)
```

Both bounds are required when either is given. They are bound Angles measured from the
source revolution's starting meridian, along its declared sense, and MUST satisfy
`0deg <= from < to <= sweep`, using the source's effective sweep (at most one full turn).
Omitting them retains the whole source sweep. They restrict
the existing `v` domain to `[from / sweep, to / sweep]`; they do not renumber parameters,
reverse tangents, or wrap an excluded point into another part of the surface. A clockwise
revolution still uses positive angles measured along that clockwise sweep. Bounds and
source dimensions may depend on component parameters.

`RevolvedSurface::domain` and browser `surfaceDomain` expose the retained parameter bounds.
Evaluation and finite-incidence projection MUST respect them. Envelopes inherit this source
domain, exposed with their roll interval by `GeneratedEnvelope::domain` / `envelopeDomain`.
This can select a generating semicircle without embedding chart bounds in an external
verifier. It does not alone prove the regularity or connectedness of the resulting envelope.

Surface references follow the current solved source geometry. An evaluated snapshot retains
the state it was read from and MUST be read again after that state changes. A surface may be
passed through a `surface` component formal, tagged as construction, or made private. Its
`solid` and `edge` fields alias its dependencies. Deleting a dependency deletes its dependent
surface; copying a surface carries its defining geometry.

An incidence evaluator may use the continued meridian's line or circle equation for a
line/arc/circle revolution whose meridian stays on one side of its axis. Its oriented residual
is in length units, and its gradient follows the source tangent orientation. Near a regular
interior point this is a signed normal distance; it is not a global minimum-distance claim.
The continued equation alone does not establish incidence with the finite source patch.
Evaluation also returns a point on the finite patch and the Euclidean error to that point.
Only that error, compared with an explicit length tolerance, establishes bounded incidence.
An axis-crossing meridian or a query with no unique normal is refused by this evaluator.

This declaration provides named generating geometry. Motion families and envelopes are
specified below; solids assembled from surface boundaries remain under development.

### 6.14 Rigid motion families: a frame's joint **[0.135]**

```
crown := frame
std.Turn(crown, about: crown_axis, ratio: 1)
blank := frame
std.Turn(blank, about: blank_axis, ratio: 2)
flank := envelope(crown_flank, under: crown, seen_from: blank, from: -35deg, to: 35deg)
```

**A motion is a frame's joint.** A frame (§6.7) owns a family of rigid transformations over one
shared angular parameter, the **roll**, once the document names one of its joint's members:
**`f.axis`**, a directed axis in space; **`f.ratio`**, a `Scalar`, radians the frame turns about
the axis a radian of the roll; **`f.advance`**, a `Length`, travelled along the axis a full turn of
the roll. The members are minted where a reference names them (`f.axis`, or a `fix` with a
`ratio` or `advance` slot), and are held by ordinary relations: `l coincident f.axis` lays the
axis on a line, `p coincident f.axis` passes it through a point, `f.axis parallel n` sets its
direction, `fix(ratio == r, advance == a) f` states the numbers. At roll `t` the frame stands
turned `ratio·t` right handed about its axis and moved `advance·t/2π` along it; a joint with ratio
0 is a slide. Roll 0 is as drawn: a member that must stand turned there is drawn turned. A frame
with no joint stands still: it is the world. A joint does not move the sketch.

The standard library (§8) states the joint the ways a designer does:

* **`std.Turn(f, about: l, ratio: r, advance: a)`** — about the line `l`; `advance` defaults to
  `0mm` (a turn), and given, the joint is a screw.
* **`std.Spin(f, about: o, on: P, ratio: r)`** — about the point `o` of the view `P`, the axis
  through `o` square to `P`, counter-clockwise as `P` is drawn. A turn in a view is how a planar
  motion is written: what it generates in its view is a curve of the drawing (§6.15.1), with `o`'s
  coordinates among that curve's columns.
* **`std.Slide(f, along: l, advance: a)`** — along the directed line `l` by `a` a full turn of the
  roll, turning not at all.

**How it is read.** Before the solve the joint's kind is read off what holds it — an axis on a
line with a ratio stated 0 is a slide, on a line a turn or a screw, through a point of a view and
square to it a turn in that view — and is otherwise a general joint, which only a reading in space
takes. A measurement of the drawing as its ratio or advance is **E107**. Axis geometry follows the
current solved model, read where it stands in space.

**Joints in the solve [0.137].** A ratio or an advance no `fix` states — a `std.Turn` given no
`ratio:`, a number pinned to an unknown — is an **unknown** of the solve, as the axis is: every
relation that reads the motion during the solve (a point it carries and its rate, a point on an
envelope in space or on a meshing) reads it as a column, and what is built after the solve reads
it where the solve left it. A profile generated in the plane (§6.15.1) is generated over stated
numbers: one found by the solve is an error there. A joint whose numbers nothing holds is freedom,
reported by the diagnosis like any other. **`std.Rolls(at, a, b, along: d)`** states that `a` and
`b` roll on one another at the point `at` without slipping along the direction `d`: the point of
`a`'s frame there, seen from `b`'s, does not move along `d`. Two pitch circles rolling at their
pitch point turn at `−R/r`, the ratio found from the radii or, stated, placing the pitch point.

**Seen from another frame.** Every reading of a frame's motion — `under:` on a solid (§6.10), an
envelope (§6.15), a meshing (§6.15.4), a point it carries (§6.15.3), a `claim over` — may name
the frame it is seen from: **`under: m, seen_from: g`** is `inverse(g) · m`, `m`'s motion as `g`'s
frame sees it, both at the same roll, its derivative including `g`'s motion. `seen_from:` left
out is the world. This is a relationship between frames, not a sequence of commands that changes
entity coordinates.

**Riding [0.136].** `in` blocks nest: `in (arm := frame) { in (planet := frame) { in std.top {
x := point } } }`. What is declared in nested blocks **rides** on every frame of its path, and is
drawn in the innermost frame of the path that has axes (a plane, or a frame whose `f.u`, `f.v` or
`f.origin` is named); one with axes further out says nothing of where the declaration is drawn
(a warning, W115). A frame declared in a block rides on the block's frame, and its motion is its
joint after the joints of the frames it rides on: the pose of what rides is the product
`J₁(τ)·…·Jₖ(τ)` of the jointed frames of its path, outermost first, each about its axis where it
is drawn. Riding enters no relation that names no roll: everything is drawn, and related, as it
stands at rest. A reading at a roll that leaves `under:` out reads what its tool rides on:
`X coincident(t == τ) x` with `x` and `X` riding on different frames is `x` carried by
`path(X)⁻¹·path(x)` at `τ`; `envelope(tool, seen_from: G, …)`, `meshing(f1, f2, …)` (the second
flank's frames seen from the first's), `solid(tool, at: θ)`, and `solid(tool, from: a, to: b)` or
`solid(tool, through: body)` over a solid that rides (a motion's words there, not a prism's).
`under:` names the frame of a tool that rides on none; one naming a frame for a tool drawn riding
is an error, as is a reading of a tool whose parts ride on different frames. What a reading makes
rides on its observer: the profile a rack cuts `seen_from: blank` rides on the blank.

**Formals.** A component that reads a motion takes the frame, `m: frame`, and where it needs one
the observer as a second formal, `seen_from: frame`. A `frame` formal a call leaves out is the
instance's own frame at rest, the world. A number formal may state its default, `advance: Length =
0mm`, which a call that leaves it out binds.

Evaluation takes the roll in radians and returns an exact rigid pose and its first derivative per
radian. A sampled source point returns world position and velocity per radian; velocity becomes
velocity per second only after multiplying by an angular speed. Numeric arguments must be finite,
and the solved axis must be nondegenerate.

These constant-ratio families provide relative generating kinematics. They do not yet specify
arbitrary motion laws or an assembly-joint solver.

**Retired spellings.** Through 0.134 a motion was a declaration of its own, `m := motion(about: l,
ratio: r, phase: φ, advance: a)`, `motion(along: l, advance: a)`, `motion(source, relative_to:
observer)`, with `m: motion` formals and its numbers allowed to measure the solved drawing
**[0.27]**. Each is refused naming what it became: `motion(…)` the frame and `std.Turn`, `std.Spin`
or `std.Slide`; `relative_to:` the reading's `seen_from:`; `phase:` a roll offset (a turn with
phase φ and ratio r read at roll τ is the same turn read at τ + φ/r).

**[0.138] A frame that moves is a frame read at a roll.** `X coincident(at: τ) x` is the point
`x` carried by the pose at the roll `τ` of the frames it rides on (seen from those `X` rides on),
or of the frame `under: m` names where `x` rides on none — `τ` an Angle the statement states:
worked out where it is written (over a face family's running number it is read again at every
station, §6.9), a free name in it an unknown the solve finds, and drawn nowhere. It is the same
reading as `X coincident(t == τ) x` (§9.3), whose roll is the relation's own unknown, pinned or
solved, which a rate runs over; stating both is an error. `seen_from: g` reads it as `g` sees it,
as on any reading. `X` in space is held by three equations; drawn in a plane, at a stated roll, by
two, where that plane sees the carried point. A roll read where nothing rides and no frame is named
is an error. A plane is carried whole: `n := base carried(at: τ)` (with `under: m` and `seen_from:
g` as on a point) is the plane `base` where the pose carries it — its origin, its `u` and `v` and
its normal the pose's images of `base`'s — and anything drawn in `n` is drawn in the carried frame.
A frame may turn about another frame's axis, `std.Turn(f, about: P.normal)` or `P.u`, as about a
line. The standard library's station words are frames read so: `std.Helix` is `base` carried by a
screw about `base.normal` (`advance: pitch`) to `360deg · turns · s`, `std.OnHelix` the frame
square to the helix where it starts carried by the same screw, `std.Twisted`'s point turned in its
plane about its origin (`std.Spin`, §6.9). Every frame that moves as a number runs is said this
way: no frame of the language is written as `cos` and `sin` ordinates of the number it moves
by.

### 6.15 Generated envelopes

```
flank := envelope(crown_flank, under: crown, seen_from: blank, from: -35deg, to: 35deg)
```

An `envelope` is the implicit locus of a named `surface` under a named `motion` where the
moving surface normal has zero component along its relative velocity:

```
F(u, v, roll) = normal(u, v, roll) · velocity(u, v, roll) = 0
```

The source parameters `u` and `v` remain in `[0,1]`. `from:` and `to:` are required bound
Angles specifying a finite increasing interval of the shared motion parameter. They are
part of the geometric domain, not solver seeds. The interval is evaluated in radians;
`F` has units of length per radian. Argument labels `surface:` and `motion:` are accepted,
with `under:` an alias for `motion:`. The `surface` and `motion` fields alias the dependencies.

The declaration does not allocate coordinates or planar solver equations. It is an implicit
surface, potentially with disconnected branches, singularities or no regular envelope.
Evaluation of a trial `[u,v,roll]` returns its transformed position, oriented source normal,
velocity and `F`. A trial with nonzero `F` MUST NOT be reported as a point on the envelope.
The source normal orientation does not designate which side contains solid material.

A local intersection adds two section equations and solves them together with `F = 0`.
Numerical search bounds must lie within the declared domain. Equal bounds hold a parameter
exactly; the remaining free Jacobian columns must have full rank, and every original
residual must meet its tolerance. A search step may be limited to a domain boundary, but a
boundary point that fails an equation MUST NOT be accepted. A converged local intersection
does not establish uniqueness, global regularity, a material side, or solid closure.

Two named analytic boundary patches may supply the section equations. Supporting equations
can guide the iteration, but the result MUST meet the finite-patch incidence tolerance for
each boundary as well. A point on a continued cone or sphere outside its declared edge or
sweep interval MUST NOT be accepted merely because those supporting equations vanish.

`envelope` component formals, forward dependency paths, privacy, semantic tags, and
copy/delete dependencies follow ordinary component rules. A solved evaluator is a snapshot
and must be read again after the source geometry changes. Envelopes are not drawing glyphs
or completed solids. Closed solids assembled from bounded surfaces remain under development; the
patches, seams, corners, edges and face boundaries that check one are Rust API (§6.16–§6.20).

#### 6.15.1 Envelopes in the plane **[0.37]**

```
blank := frame
std.Spin(blank, about: o, on: std.front, ratio: 1)
rack := frame
std.Slide(rack, along: slide, advance: 2 * pi * 30)
flank := envelope(rack_flank, under: rack, seen_from: blank, from: -25deg, to: 25deg)
profile := envelope(roller, under: follower_on_cam, from: 0deg, to: 360deg, side: near)
```

The same word over a **tool of the sheet** — a point, line, circle, arc, a curve from a computed
point, or a profile itself generated by a point, line, circle or arc — and a planar motion (built from turns about points and slides, either seen from another)
is a **curve of the drawing**, not a spatial envelope: the profile the tool cuts in the moving
frame, as the roll runs over `[from, to]`. It is a curve like any other (§6.5): it is drawn, and
`p coincident f`, `f tangent l` and `f curvature k` hold against it, each owning the roll at its contact
(`hint(t: …)`, degrees). The tool is drawn where it stands at roll 0. At roll `t` its point `T(s)`
stands at `X(s, t) = M(t)·T(s)`, and the cutting point is where `F(s, t) = X_s × X_t = 0`: the
tool's tangent runs along the velocity of the material under it, which in the plane says the
tool's normal there passes through the instant centre (the law of gearing). A point tool has no
`s`: its envelope is its path.

**What it is written over is solved for.** The tool's coordinates and the motion's — its turns'
centres, its slides' lines — are columns of the curve, so a contact on a generated profile moves
the tool and the motion to suit: a rack's pressure angle chosen so its cut passes a point, a
cam's roller placed so the profile it cuts bends at a stated radius. The joints' ratios and advances
are numbers of the curve. A rotation about a line in space, a traced tool, and a generated tool
whose own tool is not a point, line, circle or arc are errors here (**E080**). A generated tool must be
declared before the profile it cuts.

**Exact orders.** `C`, `C'` and `C''` are exact: the root `s` and the cut are read in truncated
Taylor arithmetic along the roll, each order of `s` one division by `F_s`. A point, line, circle or
arc is exact at every order, so a profile it cuts is exact to `C'''` too — and so a profile that
profile cuts in its turn (a rack-cut tooth cutting its mate) is exact to `C''`, its tool read as
its Taylor series. An implementation MAY take a `C'''` it cannot read exactly, and the gradients
along the columns, by difference: they enter only a Jacobian, never a residual.

**Which cut.** A line cuts once. A circle or arc cuts twice, on the line from its centre through
the instant centre: `side: near` (the default) takes the point nearer the instant centre, `side:
far` the farther; any other word is **E080**. A curve tool may cut in several places: the roots at
`from:` are found over the tool's interval, `side` picks among them by their distance from the
instant centre, and the choice is carried to every other roll by continuity. A root where the
tool stands still (the point a turn is about) is no cut **[0.40]**. A generated profile
over whole turns that comes back is closed (§6.5) **[0.40]**, and stands alone in a face (§6.8).

---


#### 6.15.2 A prism's side generating in its view **[0.38]**

```
side := surface(rack_tooth, edge: rack_flank)
flank := envelope(side, under: rack, seen_from: blank, from: -30deg, to: 30deg)
p := point in cut
p coincident flank
```

`surface(S, edge: e)` names a side of a prism `S` (a face swept by `depth:` or `from:`/`to:`) as it
names a revolution's: `e` a line or arc of the face, the side what `e` sweeps along the view's
normal. A prism's side carries no angular bounds (**E080**). Under a motion that keeps the prism's
view — its turns about points of that view, its slides along lines of it — every section of the
side moves alike, so what it generates is the planar envelope of `e` (§6.15.1) extruded square to
the view: a surface that depends only on geometry the drawing solves. The envelope of such a side
is therefore **built with the drawing**, before any relation: it is the curve `envelope(e, under:
m, …)` would be, standing for the surface. Every point the curve is written over — the edge's and
the motion's — is drawn in one view, and that view is constant (**[0.42]** a plane the solve
places is **E080**).

`p coincident flank` holds a point drawn in **any** view to the surface: its lift, read into the prism's
view and put on that view's sheet, lies on the curve at the contact's roll — two rows against the
roll it owns (`hint(t: …)`, degrees), the one equation a point on a surface is worth. The rows are
the planar contact's, with the point's place for its coordinates, so the tool's and the motion's
geometry are columns as in §6.15.1. A side under any other motion is the spatial envelope of §6.15,
read after the solve — and held to by it (§6.15.3).

#### 6.15.3 Envelopes in space the solve sees **[0.90]**

```
flank := std.Cone(spine, half: hint(20deg))
pinion_flank := envelope(flank, under: gear_turn, seen_from: pinion_turn, from: -30deg, to: 30deg)
M coincident pinion_flank
pinion_flank perpendicular(at: M) normal
pinion_flank tangent(at: M) along
pinion_flank tangent(at: M) P
pinion_flank curvature(at: M) profile
```

An envelope in space is **held to by the solve** where it knows the source before any relation:
a **set** — a cone (`std.Cone`, and `std.ConeThrough` with its lean stated), a cylinder
(`std.Cylinder`) or a plane (`std.HalfSpace`'s face, a distance along a line from a point,
**[0.104]**), the operand of `envelope` read as what its body states at its point (§6.21) —
or a **named side of a revolution** turned from a line, an arc or a circle (`surface(rev, e)`),
under a motion built of turns about lines, screws and slides along lines, seen from any frame. Its
numbers — the source's axis and edge, a cone's half or a cylinder's radius where that is an
unknown of the drawing (`half: hint(20deg)`), and every axis the motion turns about or slides
along — are columns of every relation holding something to it, so what makes the source solves
from what the envelope is held to: a cutter's lean from a contact a designer states.

- `p coincident flank`: a point in space, drawn in any view. The relation owns a **place** `[u,
  v, t]` — along the source's meridian, round its axis (on a plane, two lengths along two
  directions in it), and the motion's roll — where the moving
  source is at `p` and `F = 0` (§6.15): four rows against three unknowns, the one equation a point
  on a surface is worth. It starts where the envelope passes nearest `p`.
- `flank tangent(at: p) l`, `l` a line or an axis: `l` runs in the envelope's tangent plane at
  `p`, one row.
- `flank perpendicular(at: p) l`: `l` runs along its normal at `p`, the common normal of a mesh;
  two rows.
- `flank tangent(at: p) P`, `P` a plane: `P`'s normal runs along the envelope's at `p`; two rows.
  Where `P` stands is `p coincident P`'s to say.
- `flank curvature(at: p) k`, `k` a circle or an arc drawn in a plane through `p`: `k` is the
  **osculating circle of the envelope's section by its plane** at `p` — its centre at `x + (n −
  (n·m)m)/κₙ` (Meusnier), `κₙ` the normal curvature along the section's tangent `m × n` and `m`
  the plane's normal, and its radius the distance from `p`; three rows. The envelope's normal at
  a point is the moving source's there, so its shape operator is read off the source and the
  motion, with no second solve. `p` in `k`'s plane is the drawing's to state.

The four with `at:` read the place of the `coincident` holding `p` on the envelope, as a tangent
at a point of a curve reads its contact's (§9.2, **[0.79]**); one with none is **E040**, naming
the statement it wants. `at:` names a point after `perpendicular` and `curvature` as after
`tangent`. The other operand may be written first: `l tangent(at: p) flank` is the same statement.

A root is local, as a planar contact's is: the place starts where the envelope passes nearest
the point and follows the solve; the roll is not held within `from:`/`to:`, which bound the
after-the-solve evaluation; and a place where the source's normal has no direction (a cone's
apex) is degenerate, as a cone whose spine has no length is. A source the solve cannot read — a
sphere, a region, an offset, a lean or a radius that is no number times one unknown plus a
number (`std.ConeThrough`'s `abs(lean)` over an unknown lean) — leaves the envelope evaluated
after the solve where it is a revolution's side, and a relation holding something to it is
refused naming why (**E061**); a set the body reads as no cone, cylinder or plane is **E040**
where it is applied. A set's envelope has no chart after the solve: it is only held to.

A plane's envelope under a slide along its own normal relative to a turn is an **involute
helicoid** (**[0.104]**): the rack of Phillips' spatial involute gearing, each member's flank the
plane square to the line of action sliding along it as fast as the members move along it, seen
from the member (`spiral_bevel/phillips.sv`).

```
pinion_flank := envelope(gear_flank, under: gear_turn, seen_from: pinion_turn, from: -60deg, to: 60deg)
pinion_flank regular(by: gear_tooth) within pinion_blank          // no undercut
pinion_flank regular(>= 0.3, by: gear_tooth) within pinion_blank  // and a margin from it
```

**[0.115]** A generated flank is **regular within its blank**. Where a tool point touches the flank
at the roll `t` that solves `g = 0`, the map from the tool point to the point it generates scales
the tool's area by the **area factor** `A = (g_t − α g_u − β g_v) / g_t`, with `∂X/∂t = α ∂X/∂u +
β ∂X/∂v` on the envelope. `A` is 1 where the two surfaces roll without sliding, nothing where the
flank folds (undercut, the tool point's contact running back on itself), and below nothing past
the fold. `flank regular(by: tool) within blank` reads its **least** value over the part of the
flank the tool cuts in the blank: tool points on `tool`'s face whose contacts in the roll interval
lie inside `blank`. The flank is a held envelope of a cone or a cylinder set that is a face of
`tool`. `tool` and `blank` are region solids (`solid(R)`, §6.21), read by their terms as drawn:
the tool's at the source's own point, the blank's at the point generated. The tool's term the
source lies on is the face itself, and a round of that face (`std.Rounded`) ends the face at its
tangent line. The least value is found by search, so it is the least found, not a bound proved.
With no number the statement is a bound, `>= 0`: no fold. `regular(>= m, …)` is a margin from
one. Both are verdicts with no root to choose, so a solve breaking one is `BOUND_BROKEN` and
nothing is steered across it (§9.6). `regular(m, …)`, the number stated outright, is an equation:
one row that solves a number of the drawing the flank reads (a cutter's pressure angle) at
which the margin binds. `within` stands between the word and the blank; without it the statement
does not parse. A flank of no set, a source that is no face of `tool`, a face grown off the
source, a solid that is no region, or a term the solve cannot read (a blank's measured term) is
**E061**, naming why.
The envelope of the whole source is not enough. It runs past the tool's tip and folds there,
inside the blank, whatever the design, so the tool says where the flank is cut.

#### 6.15.4 The meshing of two flanks **[0.94]**

```
contact := meshing(pinion_flank, gear_flank, under: gear_turn, seen_from: pinion_turn,
                   from: -15deg, to: 15deg)
Q coincident(t == 0deg) contact
R coincident(t == 10deg, lag == 0.01) contact
```

`meshing` is tooth contact analysis, a locus. Its first flank turns with the reading's observer
(`seen_from:`, the pinion's frame) and its second with its source (`under:`, the gear's): the
motion the pair runs at, which must be seen from a frame that moves (**E080** otherwise). A flank is an envelope in space the solve holds to
(§6.15.3) — **generated** — or what such an envelope reads as its source, a cone, a cylinder or
a plane set or a named side of a revolution, standing still in its member — **formed** — or a
surface traced in space over two formals (§6.5), the flank as its member sees it — **traced**
(**[0.109]**), its place `[u, s]` and its roll held at none; anything
else is
**E080**. At each **phase** of the pinion, its roll, the contact is where the two flanks share a
point and a normal with the gear turned to the ratio's roll plus a **lag**: the transmission error
as a roll of the mesh (the gear's own angle is off by its ratio times it). A conjugate pair touches
with no lag, along a line; a pair that is not touches at a point, the gear behind or ahead.

`p coincident contact` holds a point to the **path of contact**, in the fixed frame. It owns the
phase `t` and the lag `lag`, in degrees as a curve's parameter over an angle is (§6.5), and a place
`[u, v, t]` on each flank as a point on an envelope owns one (a formed flank's roll held at none):
eight unknowns against ten rows — the point where the pinion's flank is at the phase, the two
flanks' point and normal shared, and each flank's own row — so the point is on a curve in space.
Either number may be pinned: `t == 0deg` says at which phase, and `lag == δ` states the
transmission error there. Both flanks' sources and motions and the mesh's axes are columns, so a
number that makes a flank, or the phase, solves from what is stated. Where the two touch along a
line the rows lose a rank and the point is free along it. A root is local: the place starts where
the flanks pass nearest `p` at its phase, with no lag, and is sought again when the solve comes to
it, the flanks where the relations before it have placed them; a pair a little off conjugate,
which the rows hold weakly along its bearing, is found only from near its contact.

```
n := point hint((0, 58, 0))
common := line(Q, n)
contact perpendicular(at: Q) common
p := point hint((10, 48, 0))
along := line(Q, p)
contact tangent(at: Q) along
along angle(2deg) std.x
```

**[0.112]** At a point held to the contact, a line or an axis `perpendicular` to it runs along the
**common normal** of the two flanks there, and one `tangent` to it along the **path of contact**:
the contact point's derivative along the phase — **[0.130]** its rate, `rate(Q over t) parallel
along`, the same statement (§6.5). Each reads the phase, the lag and the places of the
contact holding its point — at whatever phase that contact is, not only where the pair is drawn —
and states two rows, so a number a flank is made from, or the point's phase where nothing pins it,
solves from an angle stated of either. The path is the motion of the contact in the fixed frame, not
the bearing: a pair whose contact stands still on the gear's flank has a path all the same, the
gear's turn carrying it. Where the flanks touch along a line the path has no direction, and a line
along it does not hold. Either is refused at a point no contact holds on the meshing, naming the
statement it wants (**E040**), and may be written either way round (`along tangent(at: Q)
contact`).

```
contact := meshing(pinion_flank, gear_flank, under: gear_turn, seen_from: pinion_turn,
                   from: -15deg, to: 15deg,
                   ease: M, along: lengthwise, profile: 0.002 / 1mm, lengthwise: kl / 1mm)
```

**[0.114]** A meshing's first flank may be **eased**: relieved along its normal by
`½ κₗ s² + ½ κₚ r²`, where `s` is how far its point stands from the point `ease:` along the line
`along:` and `r` how far across it, both in the flank's own member as drawn — a parabola about the
point, its curvature `lengthwise:` along the line and `profile:` across it (on the flank near the
point, its profile). The relief runs along the flank's normal as the solve reads its source
(`∂u × ∂v`: for a cone, toward its axis), so a negative curvature relieves the other way. Each
curvature is a number per length, stated, or a number times one unknown of the drawing and a
number, which then solves from what is stated of the contact (**E080** otherwise, and where
`ease:` names no point or `along:` no line). The four are written together or not at all. A
conjugate pair touches along a line; eased, it touches at the point where the pair is drawn — the
relief and its slope nothing there — and elsewhere at a point, the gear turned by the lag that
closes the relief: so the transmission error and the bearing are the designer's, and a lag stated
of a point held to the contact solves a curvature (lags at two phases, both). A flank traced in space is not eased (**E080**).

After the solve the contact is **traced** over `from:` … `to:`, walked out phase by phase from the
first point held to it: the lag over the phase is the transmission error, the contact's places the
**path of contact** and, on each flank in its own member, the **bearing**. `solventc` says of each
meshing whether it is a point or a line contact, the transmission error's range and how long the
path of contact is; `--json` gives each phase.

### 6.16–6.20 *(retired)* **[0.120]**

Through 0.119 these sections declared trimmed spatial patches (`patch`), shared spatial seams
(`seam`), spatial vertices (`vertex`), finite spatial edges (`edge(…, along:)`) and spatial face
boundaries (`face(…, on:)`). Each was a certification object: it added no unknown, no solve row
read it, and nothing exported from it. They are no longer words of the language. Each is a Rust
type made of its parts — `patch::TrimmedPatch`, `seam::{EnvelopeSeam, BoundarySeam,
SurfaceSeam}`, `vertex::{BoundaryVertex, JunctionVertex}`, `edge::SpatialEdge` and
`spatial_face::SpatialFaceBoundary` — over the surfaces and envelopes a drawing names (§6.13,
§6.15), and its tolerance and identity rules are the implementation's contract
(`docs/contract/analytic.md`). The section numbers are kept so that §6.21 onward keep theirs.

### 6.21 Sets **[0.49]**

```sv
ball := { p | p distance(12mm) c }

component Cylinder(about: line, r: Length) := { p | p distance(r) about }

component Cone(about: line, half: Angle) := { p |
  private construction g := line(about.p1, p)
  about angle(half) g
}

shaft := std.Cylinder(ax, r: 8mm)
q coincident ball
l tangent shaft
```

A **set** is the points that satisfy its body (a predicate with one bound variable, §9.9): `{ NAME | STATEMENTS }`, a name for the point —
**bound**: it names whatever is put on the set — then one or more statements about it, on the
line or on lines of their own up to the closing brace. Written as a value, `S := { p | … }`, it
reads the names of the body it stands in. Written as a component's definition, `component
Name(FORMALS) := { p | … }`, it is a **family**: an instance is the set, its formals bound as a
call binds them (§4.1) and read by its name as an instance's are (`shaft.r`, `shaft.about`), a
numeric formal left unbound an unknown of the drawing (§6.3). A set with no statement is a syntax
error.

**A set adds nothing to the drawing**: no unknown, no equation, nothing drawn. What it means is
said where it is used, by two words:

- **A set may be written where it is used** **[0.50]**, on the right of `coincident` or
  `tangent` (**[0.69]** and of `inside` or `outside`): `q coincident { p | p distance(5mm) c }`,
  `l tangent std.Cylinder(ax, r: 8mm)`, `p inside std.Rounded(wall, top, r: 2mm)` — and is
  described as written.
- `q coincident S` (either way round) states the body with `q` for the bound point, expanded
  where the statement stands as a component's body is (§14.1). What the body declares privately
  — the cone's generator — is made once for each use. `q` MUST be a point (**E040**). A plane is
  the set its `in` is membership of (§6.7): `q coincident P` draws `q` in `P`, and **[0.74]**
  where `q`'s declaration draws it in another plane it draws `q` in `P` as well, a twin rather
  than a row holding it there (§6.7 says where a row stays, `P coincident q` among them).
  **[0.76]** In a family, what the body declares and states that reads nothing of its point —
  directly, or through a declaration that does — is the instance's, made once where the instance
  is and read by its name (`k.apex`): `std.ConeThrough`'s apex, placed by the point it is given,
  is one point however many points are put on the cone.
- `l tangent S` (either way round), `l` a line: a **contact** point standing in space, on `l`,
  is minted for the use and the body is stated at it; and each relation of the body is stated a
  second time as its **derivative** there **[0.51]** — the relation's rate as the contact moves
  along `l`'s direction, the numbers the set was given held. What the body declares of its own —
  a point, or a line not between points it was given — moves with the contact, by a **tangent
  unknown** of its own (its rows, a lift's among them, differentiated too), solved with
  everything else. So `l`'s direction lies in the set's tangent space at the contact: for a
  surface, one condition (the contact's three unknowns against its two rows on `l`, the body's
  rows against the geometry it declares, and the derivatives' against the tangents), the same
  count `distance(r)` from a sphere's centre states, and regular, the contact being an unknown
  of its own rather than a double root of the line's distance. The contact starts at `l`'s
  middle. `tangent` to a set from anything but a line or a set is **E040**, and so is a claimed
  tangency.
- **A set whose body is a circle's is the circle** **[0.54]**: `k := { p | p coincident P; p
  distance(r) o }` — the two statements in either order and either way round, and nothing else —
  with `P` a plane and `o` a point drawn in it, is drawn as `k := circle(center: o)` with `radius(r)
  k`: the circle's entity, rows and topology, its radius drawn `R…` where the body writes the
  number (for a family's instance, where the call gives it, `Round(o, std.front, r: 25)`), dragged,
  dimensioned and swept to a solid as that circle is, `face(k)` included. `q coincident k` is a
  point on the circle; `l tangent k` still reads the body's rows (above), whatever the set is drawn
  as. Where `P` is no plane or `o` stands off it, the body says something else — a line's points
  at a distance, a sphere cut off its centre — and the set is a set, drawn as nothing. Whether `o`
  is drawn in `P` is read once every statement has said where its points are, a use of another
  set included (`o coincident on`, `on := { p | p coincident P }`). *Non-normative:* which of the
  two a set is drawn as changes its rows and its callout, never the drawing's solutions; an
  implementation that drew it as a set and then found it a circle may keep it a set.
- `S1 tangent(at: m) S2` **[0.51]**, two sets: their **tangent spaces at `m` are one** — every
  direction along `S1` there is along `S2`. Two directions are solved for at `m`, in a chart the
  elaborator picks once (each rises along the world axis the sets' normal there is most along),
  and each body is stated as its derivative along both: two conditions for two surfaces, the
  directions' two unknowns against four rows. `m` on each is said beside it (`m coincident S1`);
  the word reads each body only at `m`, so a body that declares geometry of its own is **E040**
  here, and the word without `at:` is **E040**. **[0.87]** It is the one spelling: the library
  names no component for it.
- `P tangent(at: m) S` **[0.56]**, a plane and a set: `P` is **`S`'s tangent plane at `m`** —
  `m` on `P`, and the body at `m` stationary along each of `P`'s directions. `m` on `S` is said
  beside it, as for two sets. Anything but a plane before it is **E040**, as is the word without
  `at:`. Where the drawing already says what the body says along `P` — a cone's apex drawn on
  `P` (a cone's body is stationary along its generator), a cylinder's axis parallel to it — one of
  the conditions is the other: the statement is no surplus, and the diagnosis reads the
  dependency as how the tangency is made, said nowhere. *Non-normative:* an implementation finds
  that dependency by rank, not by knowing the shape; a set's library body carries no shape.

**A set's body is read in space wherever its point is drawn** **[0.65]**. A set is points in space,
so each relation its body states is the relation in space (§9.2) even where every operand is drawn
in one view — not that view's own: `q coincident std.Cone(ax, half: 30deg)`, `q` and `ax` both
drawn in the front, puts `q` on either generator there, 30° from the axis in space, where the
view's directed angle (§9.4) would turn one way only. A distance, an incidence and the rest say in
one view what they say in space; `side:` and `sense:` say nothing in space and are **E040**, as
across views. A body stated as its derivative at a point (`S1 tangent(at: m) S2`) is read where
`m` stands in space, whatever view `m` is drawn in.

Every relation a use states is **described as the statement wrote it** (`l tangent shaft`, §9.9),
its placement and classes the statement's; a dimension the body states is drawn as a component
body's is, and its derivative draws nothing. A set named anywhere else a reference stands —
`radius(5) ball`, a face's loop — is **E040**; `coincident` between two sets is **E040**, with
parentheses **E040**; a set whose body uses itself, however indirectly, is
**E003**. A set is handed to a component as a `set` formal **[0.71]** (§4.1) and used there by
the formal's name (`std.CircleOnSphere(k, ball, view)`); a `group` formal refuses a named set.

**Set algebra** **[0.54]**. A body's statements all hold of its point, so **intersection is
conjunction**: `on_both := { p | p coincident ball; p coincident shaft }` is the points on both, and
the circle above is a plane met with a sphere. **Union is a choice**: a point on `A` or on `B` is
on one of them, which a root choice says (`ccw`, `branch`, §13) and a body cannot. **Difference
is not expressible** of a locus: a set says what its points satisfy, never what they do not.

**Regions** **[0.61]**. A set whose body bounds a number (§9.6) is a **region**: `ball := { p | p
distance(<= 5) c }` is the points within 5 of `c`. `q inside S` and `q outside S` put a point in or
out of any set: its body applied to `q`, as `coincident` applies it, with its **one number** read as
a bound — at most what it says (`inside`), at least (`outside`) — and every other statement as
written. So `q inside std.Sphere(c, r: 5)` is within 5 of `c`, `q inside std.Cylinder(l, r: 2)`
within 2 of the line, `q inside std.Cone(l, half: 30deg)` within 30° of its axis seen from its apex
(in space, on either side of the axis in a view too **[0.65]**), and `q inside { p | p
coincident P; p distance(5) o }` the disc. A region's own bound is kept by `inside` and turned
round by `outside`. **[0.71]** A side is a bound — on the signed distance from a set's surface,
at most 0 for `inside`, at most `d` for `distance(<= d, side: outside)` (§6.21 below) — so
`outside` a set whose one statement is a side of another turns that side round: outside `{ q | q
outside toe }` is inside `toe`, outside `{ q | q distance(<= 1mm, side: outside) S }` is beyond 1mm
outside `S`. A region is met by conjunction too, so `blank := { p | p inside tip; p inside
heel; p outside toe }` is the three at once, and **`outside` says a difference** for regions: inside
`heel` and outside `toe`. Refused (**E040**): `outside` a set stating no one number — a conjunction,
whose outside is a union, a choice of root — or an interval (`in:`); `inside` or `outside` a set
stating several numbers; anything but a point inside a set; a tangency to a region, which has no
surface to touch. A point in a region is bound, so the use chooses the root (§9.6) and adds no row
but what the body states besides its number (the disc's plane).

**A point's distance from a set, and offsets** **[0.64]**. A set whose body states one relation of
its point, holding one number, has a **surface**: where that relation holds. The **signed
distance** from it is the distance to the surface, positive on its outside (where the measure
exceeds its number). `p distance(d, side: outside) S` is `p` at signed distance `d`, and `side:
inside` at `-d`; `<=`, `>=` bound it (§9.6). `side:` is said: a surface has two. `std.Offset(S, d)
:= { p | p distance(d, side: outside) S }` is the set `d` outside `S`, so `p inside
std.Offset(S, d)` is within `S` grown by `d`, `p distance(<= d, side: outside) S`; `p distance(<=
d, side: inside) S` is within the outside of `S` grown by `d`, outside `S` shrunk by it. A plane
takes the same words (`p distance(<= 2mm, side: outside) P`, the ordinate along its normal).
**[0.86]** These are the spellings: `inside` and `outside` take no number (`p inside(d) S` does not
parse). The bound is the general form, the equality and `>=` included; `std.Offset` is the set,
for where a set is passed or used (`p inside tip`, `std.Rounded(tip, …)`). One meaning, said by what the body measures and
where the point is: a **distance** (a sphere, a cylinder, a level) is the measure less its number,
so the offset moves the number, anywhere; at a region solid's point the bound carries the offset,
and a cone's is exact (§6.9); a **drawn point** has a **foot**, a point on `S` found by the solve,
`p` at `d` from it on the side named and `p − foot` square to `S` there — `S` tangent at the foot
to the sphere about `p` through it, stated as two sets' tangency is. The foot is seeded on the far
side of the surface from `p`, so the two start apart the way they end; it stands where `S` is
smooth (a cone's apex is not). **[0.77]** A distance a number comes to nothing is the set's own
side as a bound and `coincident` as an equality. Refused (**E040**): no `side:`; a distance of zero
written so (`coincident`);
a region, which has no surface — but **[0.76]** a region that is one side of another set, `{ q |
q outside S }` or `{ q | q inside S }`, whose surface is `S`'s, measured from with the sides turned
round where it is the outside (so `std.Rounded(beyond, tip, r)` rounds where a region is outside a
set); a set saying more than one thing of its point (a circle in a plane); an interval; a bound on a drawn point's
distance from a set with no closed form (its foot cannot say which root), a claim of one, or one
inside a body walked for its derivative (a tangency to an offset of a cone).

**A solid's faces in a region's body** **[0.69]**. Where a region solid's point is read (`solid(R)`,
§6.9) a face path (§6.9) is a set: the face as the solid has it after its terms, bounded by its
edges — `pad.plate.near` the face `plate` gave `pad`, `pad.boss` every face `boss` gave it, `block`
every face of `block` — and `F.surface` the whole surface the face lies on (**[0.68]**, one
meaning wherever it is read). `p inside F` is within the side of `F`'s surface the material is on
and `p outside F` the other side, each grown by `d` in `p distance(<= d, side: outside) F`, `side:
inside`; `p distance(d) F`, with `<=`, `>=` or `in:` (§9.6), is `p` at that distance from the face, and from its surface
with `F.surface`; `p coincident F` is on the face, `p coincident F.surface` on its surface. A face a
Boolean ate names nothing: what reads it holds nowhere, and a region over it is no material. The
faces are read off the solid as it stands once the drawing is solved, so a face of any kind is
read so, curved or not. `inside` or `outside` a face's surface (the side is the face's) is
**E040**, and so is a face read of a point the drawing draws: what is drawn on a face is drawn on
its surface (§6.9 [0.68]).

**How a face path names faces** **[0.70]**. Each face of a solid is named by the route through
the solid's operands to the primitive it is a piece of — an operand's own name at each step, a
placed copy (§6.9) by the placement's name, a swept solid's own sweep by no step — then the
face's own name in that primitive (a cap, `near`; a side, its edge's name). A path written past
the solid names every face whose name it begins: `pad` every face, `pad.plate` every face `plate`
gave it, `pad.plate.near` one. It is read from the solid, so an operand an instance made is
reached from the instance's solid by its route (`part.pad.plate.near`), and never by a suffix:
`pad.near` names a cap of `pad`'s own sweep and nothing of its operands'. `repeat a in X` (§12.8)
runs over the same names.

**Members** **[0.70]**. `S.ends` (where the edge a set is ends, below), `F.surface` (a face's or a
set's whole surface) and `A.body` (the solid a face path is of, §9.9) are no names: each is read
as the member only past a set or a solid, and only where the document declares no name so —
`h.ends`, where the instance `h` declares a set `ends`, is that set, and `cyl.body`, where `cyl`
declares a solid `body`, is that solid. Past a solid the member is read at the end of a path:
an operand called `surface` is reached through its faces (`pad.surface.near`), and `pad.surface`
is every face's surface.

**A bound on measurements** **[0.69]**. A set's body may state `LHS <= RHS` or `LHS >= RHS` (closed,
as every bound, §9.6: `<`, `>` alone are **E040**), each side a sum of numbers and measurements,
each measurement times a number: `distance(p, f1) + distance(p, f2) <= 10mm` is the ellipsoid of
revolution with foci `f1` and `f2`, `distance(p, a) <= distance(p, b)` the half-space nearer `a`.
Each measurement is a distance from the set's point to a point, a line (produced), an axis, a
plane, a face or its surface (above), or a set (below). It makes the set a region, read point by
point (§6.9), and is read only where a region solid's point is: `inside` keeps it, `outside` turns
it round, and a body of such bounds only is all of them under `inside`. A product of
measurements, a measurement read through a function or a power, a statement reading two points,
and one reading none of a region solid's are **E040**. **[0.73]** A bound whose measurements are
all weighed nothing (`0 * distance(p, a) >= 0`) bounds nothing where it holds and is **E040** where
it holds nowhere. **[0.77]** Its number may read one unknown of the drawing, affinely, as a
region's other terms do (§6.9), read when the region is; its coefficients are numbers stated.
The faces it bounds are named by the set the region's body used (`p inside inner_round`).

**The distance from a set** **[0.69]**. A measurement may name a set, `distance(p, crease)`: the
distance from `p` to the set's nearest point. The set is given a point of its own, which its body
is stated of, and is read as what that body says: points on two faces (`crease := { q | q
coincident a; q coincident b }`) are the edges they share, on three the corners, on one its
face; and any conjunction of a face's side, a distance from a face's surface, a surface, a
distance from a point and the set's own bounds is read for its nearest point, exactly. `S.surface`
of a region of one bound is its surface: **[0.71]** each bound made the equality at its number, a
side's the surface it is a side of and a grown side's the offset surface (`{ q | q distance(<= 1mm,
side: outside) S }.surface` is 1mm outside `S`); an interval's two surfaces are no one surface. **[0.77]** Such a
surface is a set wherever one is used, so a drawn point is put on it (`k coincident S.surface`).
The set's own
equalities are conditions on its point as its bounds are, each held at its number. A body stating anything else, or a set measured in terms
of itself, is **E040**. `distance(p, E) <= distance(p, M)`, with `E` on faces whose surfaces `M`
is on, is the points nearer the edge than wherever else the surfaces meet: by the edge, and not
past where it ends.

**The material's angle at an edge** **[0.69]**. `angle(a, b, at: p)`, `a` and `b` faces and `p` the
region's point, is the angle the material fills at the point nearest `p` of the edges where `a`
meets `b`: under a half turn where the edge is convex, over one where it is concave. Bounded
(`angle(a, b, at: p) <= 180deg`, §9.6), it says which edges a body's statements are about, and is
read by whether it holds, never by how near it is to failing: so a region stated at a convex edge
holds nowhere at a concave one. It is bounded alone, against a number, and `at:` is the one
labelled argument a measurement takes; faces' surfaces, or no `at:`, are **E040**.

**Where an edge ends** **[0.69]**. `p inside S.ends`, `S` a set on two faces (an edge, above), is
the side of each face the edge ends on that the edge runs on, at each end of the run of edges `S`
is: so a region along an edge (a round's run) stops where the edge stops, cut off by the faces it
ends on. A run that closes on itself ends nowhere, and `p inside S.ends` holds everywhere.
`outside` is **E040**: past where an edge ends is no region. `edge` and `seam` are element
keywords (§19), so a set may not be called so.

*Non-normative:* `std.Sphere`, `std.Cylinder` and `std.Cone` are sets. A set drawn by tracing its
points, other recognised shapes (a line as two planes met), and sets bounded by inequalities (arcs,
segments, rays) are not part of this draft (#101): a bound outside a set body chooses a root (§9.6),
and one inside a body makes the set a region (above). A derivative row has no flat spelling, so a
program lifted from a sketch (`to_program`) keeps a tangency's contact and says nothing of the
tangency.

## 7. Ports **[0.13]**

**Retired.** Everything an instance makes is reachable by its dotted name — `inst.p` for a point of the body, `inst.sub.p` for a nested instance's, `inst.s[0].p1` for one inside a block's copy — so a port was a second name for a thing that already had one (bmander/geomsolver#47). Its three forms are written as what they were:

| was | is |
|---|---|
| `port lo: point hint((0, 0))` | `lo := point hint((0, 0))` — a declaration of the body |
| `port hub = c` | nothing; the caller writes `inst.c` |
| `port p = (xexpr, yexpr)` | `p := point(x: xexpr, y: yexpr)` — a computed point (§6.5) |

An implementation MUST refuse `port` with a message naming these forms. Aliasing is untouched, being a property of argument passing and not of ports (P1): passing one instance's entity to another as an argument still merges the two into one alias class.

*Non-normative:* joints between components are ordinary constraints written at the assembly site, e.g. `gear.hub coincident shaft.j3`. The language deliberately has no "connect with joint" primitive.

---

## 8. Instances

```
t := Tooth(root, tip, slot: tau/N)
```

Instantiation elaborates the named component's body into the current scope with formals bound per §4.1. Instance elaboration is recursive. **[0.75]** A component may instantiate itself, directly or through other components: the call is elaborated as any other, under the instance's own name, so each level is named under the one that made it (`d.s.m`, `d.s.s.m`), and a `when` over the numbers the call gives ends it (§12.9). Elaboration is a function of those numbers, so a call given **the numbers its own expansion was given** — every formal of a number type bound to the same value, or none at all — makes every choice the same again and never ends: **E003**, at the call that closes the cycle. So is one nesting deeper than the implementation elaborates (bounded, since a document is untrusted input); *before [0.75] every such call was E003.*

```solvent
component Half(a: point, b: point, len: Length, n: Int) {
  m := point
  a horizontal m
  a distance(len / 2) m
  when n > 1 {
    s := Half(a, m, len: len / 2, n: n - 1)   // d.s, d.s.s, … and no deeper than n
  }
}
```

*Non-normative:* recursion is for what is *defined* by itself — a subdivision, a fractal, a tree. A run of alike copies is a `repeat`, and stays shallow however long it is; a recursion nests a level per call.

### 8.1 Groups of arguments

```solvent
unit mm
use std

sizes := {length: 20mm}
layout := {frame: axes.axes, ax: axes.u, origin: o}

component Bar(layout: group, dims: group) {
  tip := point hint(at: layout.frame, (dims.length, 0mm))
  ax := line(layout.origin, tip)
  ax parallel layout.ax
  length(dims.length) ax
}

in std.front {
  o := point
  fix((0, 0)) o
  q := point
  fix((30, 40)) q
  axes := std.Turned(o, q)
  bar := Bar(layout, dims: sizes)
}
```

**[0.35]** `NAME := {member: VALUE, …}` bundles named numeric values, entity references, and
nested groups. A brace straight after `:=` opens a group, never a body: its members are separated
by commas and may run across lines, as an argument list's may. **[0.36]** A member's value may
itself be a group in braces, `design := {bar: {at: o, size: {length: 2cm}}}`, to any depth: its
members are the enclosing group's under the member's name (`design.bar.size.length`), and the
member is a group in its own right, which a call may be given (`Bar(design.bar)`). Written in
place or defined by name and referred to (`bar := {…}`, `design := {bar: bar}`), a nested group
is the same group. Braces are a definition's and its members', never a call's argument. Members are required to have labels; duplicate members and cyclic definitions are
errors. Numeric members retain their dimensions. A group creates no geometry, solver variable,
or constraint. Geometry members are aliases to existing geometry, including subentities and
members of repeated instances; grouping does not copy or solve them separately.

A `group` formal is required and accepts a group or a component instance. Passing an instance
exposes its geometry by the same dotted paths available at the assembly site: `layout.origin`,
`layout.bank[0].pivot`. Parameters inside the layout instance are local; use an explicit group
for numeric values. Groups may be passed positionally before labels or by label. A component
may forward a group or select a nested group for another call. A missing member is an error,
not an implicit solver unknown. An untyped `group` formal's member types are what the
component's uses determine; there is no nominal record type or new solve boundary.

**[0.84] Interfaces.** A group's shape may be named, and a formal typed by the name:

```solvent
interface Size { length: Length }
interface Layout { origin: point, ax: line, size: Size }

component Bar(layout: Layout) {
  tip := point hint(at: layout.ax.p2)
  tip coincident layout.ax
  layout.origin distance(layout.size.length) tip
}
```

`interface NAME { member: type, … }` stands at the top of a file, as a component does; its
members are separated by commas or line breaks, and each type is one a formal may have but
`Side` (a group holds no side), or another interface, which nests. A formal typed by an
interface is a `group` formal in every respect but two. **The call gives every member, each of
its type**: a number of the member's dimension (a bare number fits, as in any slot, §3.3), an
entity of its kind (`path`: any of a path's kinds), a set, a group or an instance — of the nested
interface's shape where one is named; a member missing or of another type is **E103** at the
argument, one private to the instance given **E101** there, and the body's reads of a member
missing are not said again. Whatever else the actual carries is allowed. **The formal is a group
of its own, holding only what the interface declares**: `layout.size.width` above is **E101**
where it is written, a reference, a number in a dimension or an argument handed on alike; a
declared member's own fields (`layout.ax.p2`) are its own. The checks are structural: an instance
conforms by having the members, and nothing says which interfaces it is one of.

Handed on whole, the formal is that group and no more: to a formal typed by another interface it
has only the members its own declares, and to an untyped `group` formal the same — a body may pass
on only what it was promised, so the interface is all a reader of the component need know of what
it needs. An interface names no solve boundary and makes nothing.

A module exports its interfaces as it does its components: read by the module's full path
(`spiral.Gear`) or imported bare (`use spiral (Gear)`, §14.4). Two interfaces of one name in one
file, or an interface named as a component of its file, are **E071**; a member declared twice is
**E001**; a type that names no interface where it is written is **E101**; an interface holding
itself through its members, which no group could be, is **E003**. A name that is one of the
language's own types (`Length`, `point`, `group`) is no interface's.

Curves compiled over a component still require fixed scalar or entity formals. A group has no
fixed coordinate layout, so its needed members must be passed individually to a traced component.


---

## 9. Constraints

### 9.1 Equational form

```
<expr> == <expr>
```

The reference implementation does not yet parse an equation statement; a relation is written in §9.2's form. Both sides are dimension-checked. The residual is `lhs − rhs` (componentwise for future vector expressions; in this draft `==` applies to scalar-dimensioned expressions and to `Point` via `coincident`, §9.3).

### 9.2 Operator form **[0.7]**

> **Every constraint is written as a prefix or an infix operator.  `name(args…)` is retired.**

Where a constraint needs more than its two operands — a number, a selector, a third entity — those go in parentheses **on the operator**:

```
horizontal line1                    point1 horizontal point2
radius(25) circle1                  point1 distance(1' 3") point2
length(6) line1                     point1 symmetry(line1) point2
length(w / 2) line1                 point1 distance(60, along: u) point2
fix((0, 0)) p1             l1 angle(30) l2
fix(r == 25) c                      line1 tangent(side: left) circle1
```

**The type system already has this shape.** Every constraint a person writes has 1 or 2 entity slots, always first in spec order: 1 for the prefix words (`horizontal`, `vertical`, `radius`, `length`, a cone's `angle`, `fix`), 2 for everything else, and 3 for symmetry alone — which the parentheses absorb exactly as proposed. So "two operands, the rest in parentheses" is a *description* of the library rather than a rule imposed on it.

What goes in the parentheses is a short list:

- **the number**, which may be an expression exactly as elsewhere: `distance(80)`, `distance(w / 2)`, `distance(1' 3")` — **[0.41]** it names nothing; a name is `w := 60` (§6.3);
- **a selector** — `side: left`, `at: start`, `external: true`, `along: u`. **[0.17]** A selector's *key* must be one the word has (a slot of the settled kind), and its *value* must be one of the words that slot takes — **[0.46]** `along:` takes a word or a reference to an axis or a line, a name outside its words being a reference (§9.3) — both **E040**, at the key. Neither was checked through 0.16, and both failures were silent: a mistyped key was dropped and the statement settled without it, and a word outside the set fell through to whichever reading the implementation tested for last, so `at: banana` meant `end`. An implementation MUST publish each slot's vocabulary in its registry, so that a front end offers what the core accepts rather than keeping a second list;
- **the third entity**, for `symmetry`, and **[0.46]** the direction of `level`, which may be a word of `along:`'s (`level(up)`);
- **[0.117] pairs**, for `mapped`: `from -> a`, a point of the source and where the map takes it, two or three of them;
- **a pin**, `t == 0.4`, for a slot the constraint owns. Its *seed* is the trailing `hint(t: 0.4)` where every seed in the language is (§4.3). **[0.39]** A pin to an unknown — `t == s` over `param s: Angle` (§3.3, §6.3) — does not hold the slot at a number: it makes the slot's unknown **that one**, as a dimension reading it is written in it, and every contact pinned to the same unknown owns it. Inside a component the unknown is a formal the call left unbound (`leg.s`). **[0.41]** A pin to a name nothing declares is **E101**. Only a bare name is shared (an expression over one would need an equation the slot does not have), only a curve contact's parameter (`t`) can be, and only by contacts on the **same** curve — two curves have two intervals, seams and speeds — so a contact on another curve pinned to the name is **E040**; **[0.82]** a fraction of a line's run or an arc's turn has one interval on every run, so it is shared by contacts on any lines and arcs, and with no curve's parameter, and so is a `fix` pinned to one, since `fix` holds a number. **[0.41]** The shared unknown is seeded where it is declared; a `hint(t: …)` beside a pin to it is a second seed, **E040**.

| word | fixity | operands → constraint |
|---|---|---|
| `coincident` | infix | **[0.42]** (point, point); (point, line \| circle \| arc \| spline \| curve) — incidence, where 0.41 and before wrote `on`; **[0.24]** (point, plane), (line, plane) in space — **[0.54]** of a point standing in space, its membership of the plane, as `in` (§6.7); **[0.74]** of a point its declaration draws in another plane, `p coincident P` draws it in `P` too and `P coincident p` passes `P` through it (§6.7); **[0.49]** (point, set), the set's body at the point (§6.21); **[0.42]** (point, axis), (axis, plane); **[0.43]** (axis, axis), one line either way round; **[0.44]** (line, axis), both ends on it; **[0.38]** (point, the envelope of a prism's side), from any view (§6.15.2); **[0.90]** (point, an envelope in space), owning its place (§6.15.3); **[0.94]** (point, a meshing), on its path of contact, owning its phase `t`, its lag `lag` and a place on each flank (§6.15.4). Read either way round: `P coincident p` is `p coincident P`, but for **[0.74]**'s point drawn in another plane. **[0.32]** Not (solid, solid): the body rule's union is `union` (§6.9) |
| `cut` | infix | **[0.18]** (solid, solid) — the body rule's other half (§6.9), and no constraint at all |
| `distance` | infix | (p, p); **[0.46]** +`along:` an ordinate — an axis or a line, or **[0.92]** the view's `u`/`v` for the run and the rise (`right`/`left`/`up`/`down` with the sign said); (p, line); (line, line); (circle, circle); (p, plane) with `along: u`/`v` for signed ordinates from the plane's origin, or **[0.24]** `along: n` for the signed distance along the plane's normal, in space; **[0.42]** (plane, plane), the second's origin along the first's normal (§6.10) |
| `tangent` | infix | (line, circle); +`at:` for a tangency at a named end; (circle, circle); (arc, line); (spline, line); (curve, line), §6.5 — **six**; **[0.79]** (spline, curve, circle, arc or line; line) +`at: m`, a point held on the first: the tangent there, one equation reading the place of `m`'s contact — three more, and `parallel`; a line and a spline or a curve either way round; **[0.49]** (line, set), a contact on the line in the set and the body's derivative along the line; **[0.51]** (set, set) with the point in the parentheses, `k1 tangent(at: M) k2`, one tangent space there (§6.21); **[0.90]** (envelope in space; line, axis or plane) +`at: m`, a line in its tangent plane or a plane touching it at a point held on it (§6.15.3); **[0.124]** (circle or arc; line) at a point of the round — `at: m`, or the arc's end a chain's joint names — where the line is not drawn in the round's plane: the line's direction in the round's plane and square to the radius there, two equations, so a straight between bends in two planes touches each (§6.9) |
| `equal` | infix | (line, line) a length; (circle, circle) a radius |
| `curvature` | infix | (spline, circle), (curve, circle); **[0.90]** (envelope in space, circle or arc) +`at: m`, the osculating circle of its section by the circle's plane (§6.15.3) |
| `horizontal`, `vertical` | prefix | a line. **[0.48]** Between a pair of points they are the standard library's relation words (§9.9), `level(up)` and `level(right)`: `use std (horizontal, vertical)` |
| `level` | infix | **[0.46]** (p, p), the direction in the parentheses — an axis, a line, or the view's word; (p, plane) with `u`/`v`, from the plane's origin |
| `angle` | infix | (line, line); **[0.27]** (line, line) with a second pair in the parentheses, `l1 angle(l3, l4) l2` (§9.4); **[0.42]** (axis, axis \| line), (line, axis), the unsigned angle in space |
| `radius` | prefix | a circle or an arc |
| `length` | prefix | **[0.90]** a line: the distance between its own ends; **[0.27]** an arc: its length along itself; **[0.53]** a spline: its whole length; a free curve: its own. **[0.90]** `distance` relates two things and `length` measures one: so a word over `g: path` states `g`'s length whatever `g` is given, and `distance(d) l` on a line, the spelling through 0.89, is **E040** naming `length(d) l` |
| `midpoint`, `parallel`, `perpendicular`, `symmetry` | infix | one each; **[0.42]** `parallel` and `perpendicular` also take an axis beside an axis or a line, in space, and (axis, plane) — along the plane, and square to it — either way round; **[0.43]** `parallel` takes (plane, plane): the two face alike, either way (two rows over their normals; neither's place nor its turn within itself); **[0.90]** `perpendicular` takes (envelope in space, line or axis) +`at: m`, the line along its normal at a point held on it (§6.15.3) |
| `regular` | infix | **[0.115]** (envelope in space, solid) +`by:` a solid, written `flank regular(…, by: tool) within blank`: the flank's least area factor where the tool cuts it in the blank, a bound or an equation (§6.15.3) |
| `project` | infix | (point, point), each `in` a plane — the two planes are read off the memberships and never written (§6.7) **[0.10]** |
| `mapped` | infix | **[0.117]** (point, point), its **pairs** in the parentheses, each a point of the source, `->`, and where it goes: `q mapped(from -> a, to -> b, third -> c) p` holds `q` where the affine map taking `from`, `to` and `third` to `a`, `b` and `c` carries `p`, `q = a + i·(b − a) + j·(c − a)` with `p = from + i·(to − from) + j·(third − from)`. Two pairs are the similarity keeping the turn (`to` turned a quarter about `from` goes to `b` turned a quarter about `a`), `mirror: true` the one reversing it; three pairs say their turn by where their targets stand. Each side is read in its own plane — `q` and the targets in one, `p` and the sources in one, which may be another (**E062** where either is not) — so a point drawn in one view is carried into another. Three sources standing in a line, or two that are one, are **E061** naming them, where they are held; other than two or three pairs, and `mirror:` with three, are **E040**. **[0.118]** (circle, circle) under two pairs: the circle about the image of the other's centre, its radius scaled by `|b − a| / |to − from|`; under three pairs **E040** (an ellipse). A face carried whole is §6.8. No number: not a dimension |
| `fix` | prefix | the gauge (§13): an entity, and its own numbers pinned whole or by member — `fix((0, 0)) p`, `fix(x == 0) p`, `fix(r == 25) c`, `fix(dir == (0, 0, 1)) t` **[0.34]** **[0.45]** |
| `ccw`, `cw` | call | three points, all in the parentheses (§9.6) |

The collapses are where the saving is: **`coincident` is fifteen constraints, `distance` nine, `tangent` six** (in space included, and a set's uses besides, §6.21), and `horizontal`/`vertical` are two each with the **fixity** doing the work — a line prefixed, a pair of points infixed, the second **[0.46]** a `level` along the view's own axis. `angle` and `radius` keep their own words rather than folding into `distance`, because over two lines a length means a parallel distance and an angle means an angle, and nothing but the number's unit could separate them.

**Operand order carries meaning.** `arc tangent line` is a tangency at the arc's end; `line tangent circle` is the ordinary one. Each named itself before and the order was decoration; as an operator, which side the arc is written on picks the constraint.

**What a word means is the kinds of its operands, and a name does not carry its kind until elaboration** — a name may be declared further down the body (P2) or come from a component. So an implementation MUST settle the word after resolution, and MUST report a pair a word does not relate rather than guessing at one.

**The surface word and the wire name are separate.** An export format's constraint identifier is unaffected by this section; the operator is information beside it.

**`ccw` and `cw` keep a call.** Under the general rule they would be `a ccw(c) b`, which reorders three points that are symmetric: the predicate is about the *triangle*, not about a pair with a decoration. The call is a third fixity of the same table — every operand in the parentheses — and not a statement kind of its own. **[0.15]** The gauges and the orientation predicates are entries of the operator table like every other constraint: read by the one relation grammar, so a class, a placement and the chain's lookahead treat them as any other word, and settled by the word alone, since `fix(r == 25) c` names a number and `ccw(a, b, c)` has no operand outside its parentheses. They hold parameters or record a root choice rather than adding an equation, so a `claim` on one is refused (E040): a claim is judged by rank, and they add no row.

**[0.18] The body rule is written in this grammar and is not a constraint.** **[0.32]** `boss union cyl`, `bore cut cyl` and **[0.22]** `tip common cyl` (**[0.85]**; `bound` before) are **Declaration**-class (§4.2): each says what its right operand *is* (§6.9), contributes no residual, and takes no part in a solve, a decomposition or any partition of work. None relates geometry or has a residual to be settled into, so each is read by the word alone. None is in the constraint library of §9.3, and none may be `claim`ed: a claim is judged by rank and these add no row, which is the rule already stated for the gauges. `claim a cut b` and `claim a union b` are refused where they are written, neither being a constraint word. (0.18–0.31 spelled the union `on`, settled by the kinds of its operands; `coincident` between two solids is now refused as any constraint word is over operands it does not relate.)

**[0.24] Across planes, the same word is the relation in space.** A relation's operands are read in the planes their points are drawn in (§6.7). Within one plane it is the 2D relation, and the lift is rigid, so the 2D relation *is* the relation in space. **[0.42]** Where they differ, or where any point stands in space, the word means its relation between the points' lifts: `coincident`, `distance` between two points (the true length), a point and a line (to the infinite line) or two lines (the common perpendicular — a magnitude, **[0.78]** both sides roots, chosen by a bound or the seed), `coincident` a line or a circle, `angle` (unsigned, 0 to 180°), `parallel`, `perpendicular` and `equal` (true lengths), and **[0.25]** `midpoint` (of the line in space) and `symmetry` (the half turn about the line in space, which on the line's own plane is the mirror). No selector says so. `sense:` and `side:` name a turn and a side *in a plane* and are **E040** there; a word that has no meaning in space — `horizontal`, `vertical`, a run or a rise (**[0.46]** a direction named by the view's word, **[0.92]** `u`, `v`, `right`, `up` …: name the axis, `along: std.x`), `tangent` between drawn figures, a curve's contacts — is **E062**. **[0.82]** A point at a fraction of a line or an arc (`coincident(t == …)`) is a place in space too: the fraction of the way between the line's ends' places, or of the arc's turn read in the plane it is drawn in. A radius and a ring's width read only radii, which the lift carries unchanged, so they mean the same in any plane; `along: u`/`v`/`n` are measured from a plane in space wherever the point is (§6.7); and `project` relates two planes by definition. A relation over an axis or a plane is in space whatever plane its points are drawn in.

**[0.42] The role rule is withdrawn.** 0.24 read a plane's own datum points — its origin and its `toward` — by what they were related to, because they placed the view on the sheet. A plane has no `toward` and no place on the sheet now; its origin is a point drawn in it like any other.

A chain (§6.6) is the same grammar: a **lone infix statement is a one-joint chain**, and what a chain adds is the corner — which end two links meet at — that an operator between two names cannot know.

### 9.3 Standard constraint library

Residual conventions: points are ℝ²; `×` is the scalar 2D cross product; `∠(u, v)` is the signed angle from `u` to `v` in (−π, π]. In a row in space, capitals (X, A, o_P) are lifts in ℝ³ and `×` the vector cross product; a plane's û, v̂, n̂ are §6.7's.

| Predicate | Residual(s) | Eq. count | Notes |
|---|---|---|---|
| `coincident(C: Circle, p: Point)` | ‖p − C.center‖ − C.r | 1 | |
| `coincident(L: Line, p: Point)` | n(L)·p − d(L) | 1 | |
| `coincident(p, q)` | p − q | 2 | for *distinct* entities; see **W100** |
| `distance(p, q) == e` | ‖p − q‖ − e | 1 | |
| `angle(a, b, c) == e` | ∠(a−b, c−b) − e | 1 | signed; see §9.4 |
| `angle(L1, L2) == e` | wrap(∠(dir(L1), dir(L2)) − e) | 1 | directed, mod 2π; see §9.4 |
| `angle(L1, L2) == angle(L3, L4)` **[0.27]** | wrap(∠(dir(L1), dir(L2)) − s·∠(dir(L3), dir(L4))) | 1 | written `l1 angle(l3, l4) l2`; s = −1 under `sense: cw`; §9.4 |
| `length(A: Arc) == e` **[0.27]** | A.r·θ − e | 1 | θ the sweep counter-clockwise from `start` to `end`, in (0, 2π]; a magnitude |
| `parallel(L1, L2)` | sin(L1.dir − L2.dir) | 1 | |
| `perpendicular(L1, L2)` | cos(L1.dir − L2.dir) | 1 | |
| `tangent(C1, C2)` | ‖c1−c2‖ − (r1 + r2) *or* ‖c1−c2‖ − \|r1 − r2\| | 1 | branch by decoration, §9.5 |
| `tangent(C, L)` | dist(C.center, L) − side·C.r | 1 | `side: left \| right` **[0.17]** |
| `equal(e1, e2)` | e1 − e2 | 1 | any matching dimension |
| `midpoint(m, a, b)` | m − (a+b)/2 | 2 | |
| `ccw(a, b, c)` | (b−a) × (c−a) > 0 | 0 | inequality; selects a connected component |
| `cw(a, b, c)` | (b−a) × (c−a) < 0 | 0 | |
| `coincident(p, P: Plane)` **[0.24]** | n̂_P·(X − o_P) | 1 | X the point's lift; in space whatever plane `p` is in; `p` drawn in `P` is E061 |
| `distance(p, q, along: t) == e` **[0.46]** | (q − p)·t̂ − e; in one view along its own `u` (`v`) q.x − p.x − e (q.y − p.y); across views (Y − X)·t̂ − e | 1 | t̂ the unit direction of axis or line `t`, signed along it; **[0.92]** `u`, `v` name the points' view's axes |
| `distance(p, P, along: n) == e` **[0.24]** | n̂_P·(X − o_P) − e | 1 | the ordinate from `P.origin` along `P`'s normal |
| `distance(p, P, along: u) == e` **[0.21]** **[0.42]** | p.x − e; across planes (X − o_P)·û_P − e | 1 | the ordinate from `P.origin` along `P.u`; `along: v` likewise; §6.7 |
| `level(p, q, t)` **[0.46]** | (q − p)·t̂ | 1 | the ordinate's zero, written `p level(t) q`; `p horizontal q` is `p level(up) q` |
| `distance(P, Q) == e` **[0.42]** | (o_Q − o_P)·n̂_P − e | 1 | signed; parallel only where their axes make them so (§6.10) |
| `coincident(t: Axis, p)` **[0.42]** | two components of (X − A) × d̂ across the axis | 2 | A the axis's place, d̂ its direction |
| `coincident(l: Line, t: Axis)` **[0.44]** | two components of (X − A) × d̂ across the axis, for X each of the line's ends | 4 | A the axis's place, d̂ its direction; a line of no length is still on it |
| `coincident(t: Axis, P: Plane)` **[0.42]** | n̂_P·(A − o_P), n̂_P·(A + L·d̂ − o_P) | 2 | the axis lies in `P`: two of its points, L the drawing's extent apart |
| `parallel(t: Axis, P: Plane)`, `perpendicular(t, P)` **[0.42]** | n̂_P·d̂; (n̂_P × d̂) across n̂_P | 1; 2 | along `P`, and square to it |
| `coincident(t: Axis, s: Axis)` **[0.43]** | two components of (d̂_s × d̂_t) across `t`, scaled by L; two components of (A_s − A_t) × d̂_t across `t` | 4 | one line, either sense; L the drawing's extent |
| `parallel(P: Plane, Q: Plane)` **[0.43]** | two components of n̂_Q × n̂_P across n̂_P | 2 | either way round; neither plane's place nor its turn within itself |
| `coincident(L, P: Plane)` **[0.24]** | n̂_P·(A − o_P), n̂_P·(B − o_P) | 2 | A, B the line's two ends, lifted |
| `coincident(q, S)` **[0.49]** | the rows F of S's body, with q for its point | as many as the body states | §6.21; the geometry the body declares is made once per use |
| `tangent(L, S)` **[0.49]** **[0.51]** | F at a contact X on L (two rows putting X on L), and F_x·ẋ: ẋ the direction B − A in X's columns, a **tangent unknown** in the columns of what the body made, 0 in the set's own | 1 for a surface | the derivative read exactly from each row's form (its Jacobian, and its Hessian along ẋ); the body's own rows (a lift) differentiated too |
| `tangent(S1, S2, at: m)` **[0.51]** | F₁,ₓ·wₖ and F₂,ₓ·wₖ at m, k = 0, 1, with w₀ = e_a + s₀ e_c and w₁ = e_b + s₁ e_c | 2 for two surfaces (four rows over s₀, s₁) | e_c the world axis the sets' normal at m runs most along, chosen once; `m coincident S1`, `m coincident S2` beside it |
| `touches(k: free curve, L: Line)` **[0.59]** | inside `k`'s problem (§9.10): its arc ends at o + σ·d̂ on `L`, `H` unbroken there, and (λ⁺ − λ⁻)·d̂ | 0 in the drawing | σ and the place along `k` the problem's own; `L`'s points columns of `k`, as its ends are; written `k touches L` |
| *across views* **[0.24]** | the relation over the lifts X, Y of its points: X − Y (3); ‖X − Y‖² − e² (1); the point–line magnitude (1); the signed common perpendicular (1); two components of (X − A) × (B − A) across the line (2, `coincident` a line); â·b̂ − cos e (1); â·b̂ (1); (â × b̂) across â (2, `parallel`); ‖B − A‖² − ‖D − C‖² (1, `equal`); ‖X − C‖ − r and n·(X − C) (2, `coincident` a circle); **[0.25]** X − (A + B)/2 (3, `midpoint`); Q + P − 2F, F the foot of P on the line (3, `symmetry`) | as listed | §9.2 |
| ~~`reciprocal(L: Line, m: Motion)`~~ **[0.105]**, the standard library's word **[0.130]** | (B − A)·v(A), v(x) = v₀ + ω × x the twist of `m` a radian | 1 | a force along `L` does no work on `m` (Ball); `m` a turn, a screw or a slide about a fixed line, or one relative to another, whose twist is the source's less the observer's; for a mesh the meshing law of a line of action; written `L reciprocal(seen_from: pinion) gear` **[0.135]** |
| ~~`screw(t: Axis, m: Motion)`~~ **[0.105]**, `std.Screw(t, m)` **[0.130]** | two components of d̂ across ω̂, scaled by L; two of A − c across ω̂; h − ω·v₀/‖ω‖² | 5 | `t` on `m`'s instantaneous screw axis, through c = ω × v₀/‖ω‖² along ω; h, the pitch, the relation's own unknown, a length a radian, pinned as `pitch == h`; L the motion's first axis as drawn; written `isa screw mesh`; a twist turning nothing has no axis |
| `coincident(X, x, under: m, t)` **[0.106]** | X − Pose_m(t)·x | 3 | `X` is `x` as the moving frame sees it: carried by `m`'s pose at the roll `t`, in degrees, the relation's own unknown (pinned `t == τ`, or solved); X and x lifted; `m` any motion the solve reads, a turn about a point of a view read as the turn about the line through it square to the view; written `X coincident(under: m, t == τ) x` |
| `coincident(X, x, at: τ)` **[0.138]** | X − Pose_m(τ)·x; drawn in a plane, its components along the plane's `u` and `v` | 3; 2 | the same at a roll the statement states, an Angle dimension — read again where a running number is, an unknown where a free name is — owning no unknown; written `X coincident(under: m, at: τ) x` (§6.14) |
| `parallel(rate(p over t), l)`, `perpendicular(…)` **[0.130]** | r = ∂X/∂t of the contact holding `p`, every other column held: of a curve in its view (r × (B − A))/‖B − A‖ and r·(B − A)/‖B − A‖; in space two components of r across l̂, and r·l̂ | 1 in a view; 2 parallel in space, 1 square | one kernel for every contact (§6.5); the contact's slot shared, its other unknowns columns; `g tangent(at: p) l` the same statement |
| `distance(rate(p over t), d, along: l)` **[0.130]** | r·l̂ − d | 1 | `d` stated or an unknown |
| `perpendicular(S, l, at: p)`, `tangent(S, P, at: p)`, `tangent(S, l, at: p)`, `curvature(S, k, at: p)`, `curvature(k, c, at: p)` **[0.130]** | the contact's frame: l̂ against t̂ and n̂ × t̂; P's normal likewise; l̂·n̂; the circle by Meusnier; c − (X + ‖X′‖²·(b × X′)/‖b‖²), b = X′ × X″ | 2, 2, 1, 3, 3 | the same kernel as a rate's; a traced surface's, an envelope's, a meshing's normal; a traced curve's centre |
| `project(p, q)` **[0.10]** **[0.42]** | d_A·p − d_B·q + d·(o_A − o_B); over a plane solved for, (n̂_A × n̂_B)·(X_A − X_B) | 1 | the planes A, B inferred from `p`, `q`'s memberships; d the fold line they share, §6.7 |

Implementations MAY extend this library. Extensions MUST document residuals and equation counts, and MUST classify each decoration as hint or constraint per P3.

### 9.4 Signed angles

*Not yet spelled:* §9.2's table has no form for an angle at a vertex of three points, and the reference implementation has none.

`angle(a, b, c)` is the signed turn at vertex `b` from axis `b→a` to axis `b→c`, positive counterclockwise, in (−π, π]. Equating it to an expression is a 1-equation constraint. Programs that need the unsigned angle write `abs(angle(...))`; implementations MUST warn (**W102**) that `abs` introduces a branch (two solution families) unless an orientation predicate elsewhere disambiguates.

**[0.17]** `sense: cw` turns the number a statement writes: `l1 angle(30, sense: cw) l2` states −30° and is the spelling a drawing SHOULD use, the minus being a coin a reader cannot check. An implementation MUST draw the figure from the number the statement *makes* — the arc sweeping the way the label reads.

`angle(L1, L2)` between two lines is likewise directed: `∠(dir(L1), dir(L2))`, the signed turn from `L1`'s direction (p1→p2) to `L2`'s, positive counterclockwise, in (−π, π] as everywhere else in this section. It is NOT a statement mod a half turn — the residual pins which side, so a bearing needs no orientation predicate beside it — and it is therefore sensitive to the order of the two lines and to the endpoint order each was declared with. Equating it to `e` compares the two mod 2π, so `e` may be written on any lap: 270° and −90° state the same thing, and an implementation MUST NOT treat a stated angle outside (−π, π] as an error.

**[0.27]** The number may be another such angle: `l1 angle(l3, l4) l2` states `∠(L1, L2) = ∠(L3, L4)`, both directed, compared mod 2π, and `sense: cw` equates the first with the second's negative — its mirror image. A statement holds at most one number, so two unlabelled items in `angle`'s parentheses are the second pair or an error. It relates turns in a plane and has no reading across views (**E062**).

### 9.5 Branch decorations are constraints

Several predicates have discrete solution branches. Branch selection changes the solution set and is therefore constraint-class (P3), written as a selector on the word:

```
c1 tangent(external: true) c2     // external tangency: ‖c1−c2‖ = r1 + r2
c1 tangent(external: false) c2    // internal tangency: ‖c1−c2‖ = |r1 − r2|
```

A selector left unwritten is read off the seed (§9.2): the branch the seeded pose stands on is the one stated, as a side is for a distance from a line. (0.1 wrote the branch as a decoration, `tangent.ext(c1, c2)`, and refused an undecorated `tangent` as **E010**.)

### 9.6 Inequalities

Inequalities contribute no equations; they select among the discrete solution components of the equality system. Solvers MUST verify them on candidate solutions and MUST NOT report a solution violating one. There are two kinds: the orientation predicates (`ccw`, `cw`), and bounds.

**A bound** **[0.60]** is a dimension whose number says how far the measure may go rather than what it is:

```
a distance(>= 0, along: up) p     // p at or above a
p distance(<= 6) q                // within 6 of q
a distance(in: (1, 9), along: up) p
p inside P                        // behind P's normal: p distance(<= 0, along: n) P
p outside P                       // in front of it
```

- **What takes one:** a measure a solve can be steered along — an ordinate (so `inside` and `outside` a plane, the point's ordinate along the plane's normal from its origin), a distance between points, and a point's or a parallel line's distance from a line. A bound on any other kind is **E040**.
- **Closed:** `>=`, `<=` and `in:` include their edge, where a solution may stand. `>` and `<` are refused.
- **Its number:** zero may be written, unlike an ordinate's equality, which zero makes a `level` (§9.2). A bound may not be a `claim` (**E040**): a bound chooses which solution is meant, and a claim asks about the one there is. It may read an unknown only where a row of the drawing reads it too (**[0.64]**, a cone's half its side is chosen against), read as it stands: on the seeds its seed, on the solution its solution; one nothing else reads is **E040**, since a bound adds no freedom.
- **Meaning:** outside a set body, a bound **chooses the root**. It adds no row and no freedom, and is drawn as no callout. It does the work a seed did when the seed only picked a side, so it is read on the seeds first: a point whose seed breaks a bound is carried across its edge, to the mirror of its reading, as if it had been seeded there, and everything seeded from it or placed through it follows. The solver then checks it on the solution. Where it still does not hold, the solver looks for the root where it does, from the seeds and from the solution, and fails the solve where none is found (`a bound does not hold`). **[0.78]** A solver that constructs the drawing a block at a time (§15) holds each block to the bounds it decides — every quantity they read is the block's or placed before it — and looks for a block's root there, so the words choose the roots of a drawing with no seed at all. The diagnosis tells which bounds are violated, and which hold with the solution on their edge (*active*).
- **A region's land** **[0.76]**. A region draws no tip line for `length(>= d) tip` to stand on: its rounds and faces are worked out, not solved. A tooth whose flanks are drawn through the ends of its tip — a face mill's blade, given by its point width — says its land on those ends, `eb distance(>= d, along: left) ea` in the tooth's section: on the seeds and on the solution it is the same root choice as the drawn section's, so a start on the root where the flanks cross above the tip (the tooth an edge, its tip no face) is carried to the tooth with its land.
- **Inside a set body** a bound makes the set a region, and `inside`/`outside` put a point in one (§6.21) **[0.61]**. `solid(R)` is a region's solid (§6.9) **[0.62]**. `distance(<= d, side: outside|inside)` grows a side by `d` (§6.21) **[0.64]**, its one spelling **[0.86]**.
- `inside` between two solids remains the claim of §9.8; which one a statement is follows from its operands.

### 9.7 Claims **[0.5]**

```
claim <relation>                  // any relation statement of §9.2: claim a distance(5) b
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
parameter (a curve contact's `t`) is an error as a claim (**E040**), since its unknown would
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

*Implementation note:* the kernel measures interference as **common-material thickness**: the diameter of the largest ball contained in the intersection, reported with a negative sign. Thus two 10 mm cubes overlapping by 5 mm report −5 mm, and identical 10 mm cubes report −10 mm. A bounded spatial search includes its remaining measurement error in the reported uncertainty. This convention applies to disconnected and nonconvex intersections too; disjointness and containment remain required predicates, whatever sign is written for a gap.

The trichotomy of §9.7 does **not** apply here.  *Consuming* asks whether enforcing a claim would take a freedom, and there is no rank to take: a solid claim holds, is refuted, or is undecided.

**A claim over a sweep.**  Every claim in the language is judged at one pose, and a fact about a *cycle* — a disc clearing a cylinder's mouth all the way round, a port open through mid-stroke — is not one of those.

```
claim over crank.theta in (0deg, 360deg) {
  crank.disc clear(1mm) bankA.cyl.body
  bankA.pis.body inside bankA.cyl.bore
}
```

`claim over NAME in (A, B) { … }` judges every claim in its body as the drawing runs along `NAME`, and reports the **worst** pose reached.  It is Structure-class: it says how the claims inside it are judged and asserts nothing itself.

- `NAME` MUST be an **unknown** of the drawing (§3.3, §6.3) — an input with no value, or a formal an instance left unbound — or **[0.40]** a frame with a joint (§6.14), whose roll the claims run along: every solid placed under it (`solid(S, under: NAME, at: A)`) is read at its angle advanced by the roll, and nothing is solved again.  **[0.135]** `claim over NAME seen_from G in (A, B)` runs the roll of `NAME`'s motion as `G` sees it.  A value or an input with a value is a number the document already fixed and sweeping a constant is not a question; naming one, or naming other geometry, is **E040**.
- `A` and `B` are read in the units the unknown's readers are written in: an interval of an angle is an angle, and one of a length is a length.
- An implementation MUST state that its answer is by **sampling**, and how many poses it took. A pose that did not solve or produced invalid solid geometry cannot certify the claim; the report identifies those failed parameter values, and a sweep with unresolved poses cannot be reported as holding.  A claim that holds at every sample is a claim that held at every sample; a swept claim is honest about that in the way a faceted one is honest about its margin.

*Non-normative:* the two together are what make a drawing's claims a test suite for the *object* rather than for one picture of it at one moment.  The loop an author works in — write, run, read the verdicts — needs the verdicts to be about the thing being made.

### 9.9 Relation words **[0.48]**

```
a horizontal b := a level(up) b
flat l := l perpendicular t
a above(d) b := b distance(d, along: up) a
```

A **relation word** is defined at the top level of a file — never inside a body — as a component is: the word between its two operand names (infix) or before its one (prefix), its parameters in parentheses after it, `:=`, and the **one** relation it stands for. A statement writing the word, `p horizontal q`, `r above(d: 7mm) q`, is that relation with the operands and the parameters put in, expanded where the statement stands (§14.1) as a component's body is.

- **Operands are entities**, aliased as a component's entity formals are (P1); their kinds are checked by the relation they reach, where the word is expanded (`p above(d: 1) l` of a line is **E040**, as `l distance(…, along: up) p` would be). A body may read an operand's members (`a.p1`).
- **Parameters are numbers, selector words or entities** **[0.83]**, given **by label** where the word is written, as a call's numbers are (§4.1): a value given by position is **E004**, a label the word has not is **E040**, and a parameter left out is **E040**. Which a parameter is is how its body reads it: a name read in a dimension, a pin or a seed is a number (written in as the text it was given, read where the word is written); one standing where a selector's word does (`side: s`) is a word; and one standing where an entity does — an operand (`P.v parallel up`), an entity or a direction in a relation's parentheses (`level(s)`, `along: s`), a declaration's child — is an entity, given a reference read where the word is written and aliased in the body as an operand is (`up: std.z`; one that names nothing is **E101** at the argument, and one that is no name **E040**), or a direction word put where it stands (`s: up`) where no name there is spelt so. A name read as a number and as either of the others is **E040**. A parameter only passed whole to another word (`a level_with(e) b := a both(along: e) b`) is what that word reads it as.
- **The body is closed** over its operands and parameters (§5): a reference or a name it reads that is neither — another entity, a value of the file, a datum — is **E101** at the definition. A word whose body states another defined word is that word's expansion in turn, resolved from the file the body is written in; a word reached again while it is being expanded is **E003**.
- **An error inside the expansion is reported where the word is written**: every place in the body is the word's own place in the statement that writes it, and a dimension that is one of the word's parameters is the argument the statement gave it — so its callout shows that argument, and editing the callout edits it.
- A word that is a word of the language's own — an element keyword, a trailing clause, a modifier, a body word, a word opening a statement — or a constraint word of the language **of the same fixity** is **E071**; so is a word defined twice in one file with one fixity. An infix word and a prefix word of one spelling are two words: `std`'s `a horizontal b` stands beside the language's `horizontal l` of a line.
- A word nothing defines where it is written is **E102**, naming the `use` that would import it where a module defines it.
- The statement keeps its identity: its placement, classes and `claim` are the statement's, and a constraint the body states is **described in the word as written** (`p horizontal q`), not its expansion. A body states its relation alone: a placement or a class in it is a syntax error.

**A body may be several statements** **[0.50]**: `l1 flush(d) l2 := { l1 parallel l2; l2.p1 distance(d) l1 }`, in braces, one per line, for one fact about the operands that takes several rows. It may declare the geometry its relations need (`private ab := line(a, b)`), which each use makes anew, under a name of the use's own, and read only what it declares besides its operands and parameters. Every relation a use makes is the statement's: deleted with it, described as it, a placement on it going to the one relation when it makes one. A body may write other words and use sets (§6.21), a set written in place included (`a orbits(r) o := a coincident std.Sphere(o, r: r)`). An infix word is a relation **between** its operands — distance, symmetry, flushness; that each stands on a third thing is said of each (`a coincident P`), not made a word.

**A word and a set are one predicate** **[0.50]**: a body over parameters (in parentheses, given at the call) and bound variables (a word's operands, a set's point, written where the definition shows them and filled at the use), applied by one rule — under a name of the use's own, each bound variable the operand the use wrote, the body closed over what it was given. A component instance is the case with no bound variable, stated where it is written. An operand that names nothing is **E101** once, where it is written, and not again at each place in the body that reads it. *Open, non-normative:* a word as a value (#80). The standard library defines `horizontal` and `vertical` between two points as words (§14.4), and where one point stands from another (`b offset(dx: 30, dy: 12) a`, `b right_of(d: 30) a`, `left_of`, `above`, `below`), where a point stands in a plane's frame (`p coords(du: 20, dv: 5) P`, `p on_u(d: 20) P`, `p on_v(d: 5) P`), two lines crossed (`a skew(theta: 90deg, e: 20mm) b`) and one line turned from another (`a turned(theta: 180deg) b`). **[0.83]** `P square(up: d) l` stands plane `P` square to a line or an axis `l`, its `v` along `d` and its `u` square to `d` and to `l` — three statements, since a plane's axes are not held square (§6.7); with `d` square to `l` its normal runs along `l`, and where `d` is not that is no conflict: the normal is `l`'s part square to `d`. Each is two dimensions or a dimension and a level, said as the one fact they are. **[0.53]** A body may state an energy over an operand (§9.10): `std.hangs(L: 150mm) rope` is a curve that long, hanging.

**A word may make solids** **[0.69]**. A body in braces may hold, beside its relations and their
declarations, blocks (`repeat a in A { … }`, §12.8), instances, sets and body statements, so a word
over faces can make a solid of a set and join it to a body: `taken cut A.body`, where `A.body` is
the solid the face path `A` is a face of. Each use makes them anew, under a name of its own, as a
body's declarations are. That is how the standard library says `round` and `chamfer` (§6.9):

```
A relieved(d) B := {
  repeat a in A {
    repeat b in B {
      when b != a {
        groove := { p | p distance(<= d) a; p distance(<= d) b; p inside a; p inside b }
        relief := solid(groove)
        relief cut A.body
      }
    }
  }
}
block.near relieved(d: 1mm) block.south
```

### 9.10 Energies: `minimizes` and `maximizes` **[0.53]**

```
rope minimizes integral(p.y over p)
strip maximizes integral((p.x * t.y - p.y * t.x) / 2 over (p, t))
rope minimizes 2 * integral(p.y over p) - integral(p.x over p)
```

`k minimizes E` is a statement about the curve `k`, as `horizontal l` is about a line: the word is indicative, never an instruction. Its **energy** `E` is a sum of terms `c * integral(EXPR over p)`, each a constant `c` (1 where none is written) times an integral along `k` of `EXPR`, read at the point `p` running along it — and, written `over (p, t)`, at its unit tangent `t` there — **weighted by arc length**. `EXPR` reads `p.x`, `p.y`, `t.x`, `t.y` (the point's coordinates in the view `k` is drawn in) and numbers in scope; any other name is **E101**. Where the document names a unit the point is a length and `EXPR` is dimension-checked: a power of length, never an angle, every term of one energy the same (**E103**). **[0.57]** Any power, not only a whole one: a speed `sqrt(h - p.y)`, a slowness `1 / sqrt(h - p.y)` or a weight `10mm / p.y` is an energy.

`k minimizes E` states that **`k` has the shape that makes `E` stationary** among every shape from its start to its end of its length: **[0.55]** the solution of `E`'s Euler–Lagrange equation — in arc length `s`, with the tangent at angle `θ`, the Hamiltonian `H = f(p, t) + λ·t` stationary in the direction, `p' = t`, `λ' = −f_p`. `k maximizes E` is `k minimizes -E`. The curve's **ends** are what it hangs between, so they keep the freedom they have: the curve is a function of its ends and its length, and adds no freedom of its own to the drawing (§16.3). Several energy statements over one curve are **one energy**, their terms added: order-free (P2), as a body's operations are. What is solved for is stationarity; that the stationary shape is the extremum the word names is **judged**, as a claim is (§9.7, below).

- **What it is over.** **[0.55]** An energy is over a free curve (§6.1); over anything else it is **E040**.
- **Its length.** Held by `length(L) k`, the length is that. **[0.57]** With none, the rest of the drawing may determine it — with the curve's ends and what the drawing places without the curve held, no motion left changes it: a placed point the curve passes (a span and a sag), a held line it touches — and it is solved for like any unknown. Where nothing does, it is where `E` is stationary in it too — transversality, `H = 0` at the end: two points and an energy with no length is a geodesic of the weight `f`. A free point on the curve moves with it and determines nothing. A length nothing holds and no stationarity settles — a hanging rope only lowers its energy as it lengthens — is left a freedom of the drawing, no row seeking a length that is not there, and SHOULD be said at the energy (**W114**).
- **A peg presses; a drawing relation yields.** A relation between the curve and other geometry that is still free is satisfied by that geometry: a line drawn tangent to a hanging rope moves onto it. **[0.55]** A held point the curve is stated to pass (`peg coincident rope`, `peg` held) is a **peg**: the curve's problem passes it, its costate free to jump there — a point force — and its direction with it: the rope drapes over the peg in a corner, the place along the rope where it does solved too. Any other relation whose every operand but the curve is held is **E040**: a held line pressed against the curve meets it at a corner, so no smooth tangency to it is stationary. **[0.59]** That corner is `k touches L`, `L` a line: the curve's problem passes the line at a place on it its own, the costate's jump — the force the line bears — square to the line, so nothing holds it along the line, and `H` unbroken. The place on the line is solved with the place along the curve; the verdict's Hessian takes both. The line's points are the curve's to read, as its ends are: held or not, the curve is a function of where the line is, so a line the drawing leaves free moves (a drag) and the curve with it. Where the curve crosses the line the force is none and the curve as it would be. `touches` over a curve that is not free is **E040**, and an implementation MAY bound how many lines one curve touches (**E040** past it); it holds no length.
- **The solution set is the stationary points (P3).** A stationary curve may be a minimum, a maximum or a saddle, and the seed decides which the solve reaches. The diagnosis MUST say which — **[0.55]** by Legendre's condition (the Hamiltonian's sign in the direction along the curve), Jacobi's (no point conjugate to an arc's start before its end), and, with pegs or a free length, the energy's Hessian in where they are — and SHOULD say it where the statement is written, as a claim's verdict is (§9.7): `minimum`, `maximum`, `saddle`, `degenerate` where the second order cannot tell, or `unsolved` where no stationary shape was found. A `maximizes` answered `maximum` found what it asked for. A non-minimum is reported, not refused.
- A relation word's body may state an energy over its operand: `std.hangs(L) k := { length(L) k; k minimizes integral(p.y over p) }`.

*Open, non-normative:* an integral as a constraint of its own (`integral(…) == A`), an integrand reading other geometry or the curvature (the elastica, #136), a natural end condition (an end free to slide, varied with the curve), and a free curve in space.

---

## 10. Paths

Paths are the one ordered construct (P2, exception). The reference implementation does not parse them yet. A path is a directed traversal of vertices connected by segments.

### 10.1 Grammar

```
path outline: ccw = lead -> tl ~tip~ tr -> trail
```

- `path NAME : ORIENT = PATHEXPR` declares a named path with orientation `ccw` or `cw`.
- A bare `PATHEXPR` statement is an anonymous **path fragment** (used for splicing, §10.5).
- Segments: `a -> b` is a straight segment; `a ~C~ b` is an arc on circle `C`; `a ~C rev~ b` is the reversed-branch arc (§10.3). **[0.44]** `~` is `hint` (§4.3), so the arc segment wants another delimiter when paths are implemented.

### 10.2 Orientation

Every path or fragment containing an arc segment MUST have an orientation, either declared (`: ccw`) or inherited (§10.5). A closed path's declared orientation MUST match the winding of its solved vertex sequence; mismatch is a solve-time error (**E011**).

### 10.3 Arc branch rule (the consistent default)

> **An arc segment traverses its circle in the direction of the path's orientation.**

In a `ccw` path, `a ~C~ b` is the counterclockwise arc on `C` from `a` to `b`; in a `cw` path, the clockwise arc. The decoration `rev` selects the opposite branch. `rev` is constraint-class (P3): it changes the solution set of the *shape* (and, where arc-length or containment constraints reference the path, of the coordinate system too).

*Non-normative:* this rule makes convex-ish boundaries annotation-free. Tracing a gear outline counterclockwise, every tip arc and every root gap arc is counterclockwise on its own circle; no segment needs `rev`.

### 10.4 Derived constraints

An arc segment `a ~C~ b` implies `coincident(C, a)` and `coincident(C, b)`. These derived incidences enter the constraint store subject to deduplication (§14.3), so restating them explicitly is legal and free.

### 10.5 Fragment composition

Path fragments compose by **endpoint identity**: two fragments whose end and start vertices are the same alias class concatenate. A fragment without declared orientation inherits the orientation of the (unique) named path it composes into; if composition is ambiguous or orientations conflict, error **E012**. A set of fragments whose composition closes (every vertex has in-degree = out-degree = 1) forms a closed boundary; implementations MUST report boundaries that fail to close when a closed boundary is demanded by export (**E013**).

*Non-normative:* this is how the gear outline is assembled: each `Tooth` contributes `lead → tl ⌒ tr → trail`, and the `ring` body contributes the gap arc `t.trail ~root~ next.t.lead`. Under ring elaboration the fragments chain around and close at instance N−1 → 0 with no seam case.

---

## 11. Hints

**[0.2]** A hint is one of the two seed forms; the other, and the primitive, is the inline seed of §6.4. Everything in this section applies to both, and "hint" below should be read as "seed". Use `hint` when inline cannot say it: seeding an entity declared elsewhere, or seeding from an expression over other geometry. The statement form below is not implemented; the reference implementation seeds inline only (§6.4).

```
hint t.lead(x: center.x + root.r, y: center.y)
```

`hint REF(key: EXPR, …)` seeds the entity `REF`'s named scalars at the values of the expressions (evaluated with whatever definitional values and previously seeded values are available; a quantity with no seed of its own reads its provisional one, as an inline seed's does (§6.4 **[0.12]**)).

Normative invariant (**Invariant H**): *for every program P, sol(P) = sol(P minus all seeds).* Implementations MUST maintain a statement classification sufficient to verify Invariant H syntactically — i.e., the seed class is closed under everything the grammar allows in a seed, and nothing in the seed class can generate residuals or alter aliasing.

**[0.2]** §4.3 is how that classification is meant to be maintained: a number inside `hint(…)` is seed-class and every other number is not, so the check is a look at the clause rather than an argument about the statement.

Hints on entities inside a `ring` seed the fundamental-domain representative (§12.4). Hints MAY use block indices in `repeat`/`cycle` (where each instance is a distinct variable) and MUST NOT use them in `ring` (**E015**: there is only one representative to seed). **[0.47]** Nor may anything else in a ring's body: a relation, a count or an argument that varies with the index is not a turn of the representative's, and a quotient solve would state only the representative's (**E015**).

---

## 12. Repetition

Three constructs, three meanings. All take a compile-time `Int` count and an optional index binder. **[0.72]** A fourth block repeats nothing: `when` makes its body once or not at all (§12.9).

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

**Semantics.** Let g = Rot(center, τ/N), the rotation by τ/N about the point entity named in the `about` clause — counter-clockwise in that point's view — or **[0.47]** about the axis it names, right-handed about the axis's direction, which MUST be held (a free direction makes g a function of an unknown; **E023**). A ring about a point turns points drawn in its view; one about an axis turns points in space (**E023** otherwise). Define the unrolled program U = the same body under `cycle N`. Then:

> sol(`ring`) = { x ∈ sol(U) : instance i+1 of every ring-local entity equals g · (instance i) }.

That is, `ring` ≡ `cycle` + symmetry constraints. `ring` is constraint-class: it restricts the solution set to the C_N-symmetric solutions. This is a normative equivalence — an implementation MAY literally elaborate to `cycle` plus per-instance rotation equalities and MUST get the same solution set as one that solves in the quotient.

**[0.2] An implementation that unrolls MUST report that it did**, per `ring`, wherever it reports the DOF ledger (§16.3). The solution sets match; nothing else does. A `ring` states symmetry so that an implementation can *exploit* it, and an unrolled one gives every bit of that back: measured on a 30-tooth gear, unrolling put the wheel outside the cluster vocabulary entirely (so it fell to a numeric residual), past the size at which a drag can be answered by moving rigid bodies, and past the size at which the numeric rank cross-check runs at all — so the dependency reporting of §16 silently switched off. None of that is visible in the solution set, and all of it is visible to a user. 0.1 let an implementation take the licence in §12.3 and skip the SHOULD in §12.4 without ever saying so; that is the gap this closes.

**The `about` clause is mandatory.** The axis point MUST be an entity invariant under g — which for a rotation means the axis point itself (trivially) — and MUST be declared outside the ring.

### 12.4 Fundamental-domain solving (SHOULD)

Implementations SHOULD solve a `ring` in the quotient: one representative per ring-local entity name; a reference `next.e` inside the body denotes g·(representative of e); `prev.e` denotes g⁻¹·(representative of e); an external reference `name[k].e` denotes gᵏ·(representative of e). Every body constraint is instantiated once over representatives with group-element annotations. Solutions lift by orbit expansion xᵢ = gⁱ·x₀.

*Non-normative:* this is N× fewer unknowns and structurally excludes asymmetric spurious roots and permutation-collapsed roots. It is why one hint seeds one tooth and the gear cannot stack its teeth.

**[0.47]** The reference implementation solves in the quotient: a copy is an entity like any other — named `name[k]`, drawn, an operand of a relation outside the ring, an edge of a face — whose numbers are its representative's turned, a linear function of the representative's and the centre's (`model::Turn`). A copy's relations, holds and claims are the representative's turned and are not stated; its curve is the representative's, turned. The ring's own turn is the representative's freedom, gauged by a statement outside the ring (`hub horizontal tip[0]`). A copy may not be held (**E023**: hold the representative), and a relation reading a turned curve is refused (**E023**) until a contact can be stated through the turn.

### 12.5 Invariance of external references

An entity declared **outside** a ring and referenced **inside** it MUST be invariant under g. Implementations MUST verify this by the following syntactic criterion, and MAY additionally prove invariance semantically:

- the axis point itself: invariant;
- a `Circle` whose `.center` is (an alias of) the axis point: invariant;
- any value-typed entity (`Scalar`, `Length`, `Angle` used as magnitude): invariant;
- **[0.47]** a plane a ring-local point is drawn `in` (a place, not a position), and anything a seed's place reads (a seed is no constraint, §11): not judged;
- everything else: **not** established — error **E021** ("entity referenced in ring is not C_N-invariant"). **[0.47]** So is a copy of the ring's own reached by index from inside it (`tip[2]`): it reads the same copy from every turn; a neighbour is `next.tip` or `prev.tip`. An instance inside a ring — drawn, or written in place for a curve — and a relation word or set applied there are judged by what the call wrote: `Edge(u, next.u)` reads a neighbour exactly as `line(u, next.u)` does, and `Edge(u, u[2])` is refused at the argument, not inside the component.

*Non-normative:* E021 is one of the language's best diagnostics; it converts "the solver produced something weird and asymmetric" into a precise compile-time message.

### 12.6 Nesting

Nested `repeat`/`cycle` inside `ring` (and vice versa) is legal. Nested `ring` inside `ring` requires the inner axis to be invariant under the outer generator; implementations MAY reject nested `ring` in this draft (**E022**, "nested ring not supported") and MUST NOT silently mis-solve it. Full nested-group semantics is deferred (§17).

### 12.7 The identity of a statement under expansion **[0.2]**

> **A statement inside a `repeat`, `cycle` or `ring` body is ONE statement, however many things it makes.** What tells its instances apart is the **instance path**, not the statement.

A `cycle` of thirty makes thirty entities from one line of source. That line is the statement: it is what a span points at, what a caret lands on, what a diagnostic names and what an edit rewrites. The thirty are distinguished by the sequence of block indices reached to get to each — outermost first — which an implementation MUST record alongside whatever it records about where an entity came from.

An implementation MUST NOT give each expanded copy a statement identity of its own.

**Why this is normative rather than an implementation detail.** It decides whether the language can be edited at all. Give each copy its own identity and every entity a `cycle` or a component produced names a statement that appears nowhere in the source — so a caret in the text cannot find what it draws, a diagnostic cannot point at the line that caused it, and an edit computed against a span has nothing to splice. It also hides exactly the fact a seed writeback needs (§4.3): a statement reached thirty times has thirty poses and no single one to record, and that is visible only if the thirty agree on which statement they are.

An implementation MAY still need a per-instance key for its own tables. That key is `(statement, path)`, and it is not a statement.

### 12.8 Repetition over a chain's edges **[0.27]**

```
repeat e in CHAIN as i { ... }
cycle e in CHAIN as i { ... }
```

One copy per link of a named chain (§6.6), in traversal order, with `e` a reference to that copy's edge — a constraint operand, a field (`e.p1`), an argument — and `as i` optional as before. The count is the chain's, so the chain MAY be declared later in the body (P2) or reached through an instance or a group. The copies are §12.1's (or §12.2's, with `next`/`prev`) in every other respect, and §12.7 holds. `cycle` over an open chain, and iteration over what is not a named chain or a solid, are errors at the reference (**E103**); a body declaration named as the edge is **E001**.

**Over a face's walk** **[0.113]**: `repeat e in F`, `F` a face (by name, or a `face` formal), makes one copy per edge of its loop in walk order: the links of a chain it is made of, the edges it names, or — for a face of corners (`face(a, b, c, -> close)`) — the straight run from each corner to the next, a line the face owns, drawn once and hidden as its closing lines are, whatever walks it. A face is a loop, so `cycle e in F` closes. A face with holes, a corner with no name, a stretch of a curve, and corners written beside edges are **E080** at the reference.

**Walks in step** **[0.113]**: `cycle (ea, eb) in (a, b) { … }` runs several walks — chains or faces — together, copy `k` naming the link `k` of each, as many names as walks (a syntax error otherwise). The walks MUST have as many links (**E080**, naming what each walk is), and a `cycle` MUST walk only loops. Two faces pair their edges so by their places in the walks: this is the whole of correspondence between sketches drawn apart (§6.9 [0.113]).

**Over a solid's faces** **[0.69]**: `repeat a in X { … }`, `X` a solid or a face path (§6.9), makes one copy per face `X` names, `a` the face (a face path itself, §6.21): a prism's `near`, `far` and a side per edge of its section, a revolution's wall per edge and, where the turn is partial, `start` and `end`, a body's every operand's faces under the operand's name (`pad.plate.near`). The faces are read off what the solid is made of, so one a Boolean ate is listed still and names nothing (§6.21); they are known once the solids are built, so the block is expanded then. **[0.91]** Which copies make anything is a `when` in the body (§12.9): `repeat b in B { when b != a { … } }`, `repeat c in A { when c in B and c != a { … } }`. A condition written after the face path (`repeat b in B if b != a`) is a syntax error; *0.69 through 0.89 kept copies by one, `f in X` and `f != g` joined by `and`.*

### 12.9 `when` — a block made or not by a condition **[0.72]**

```
when COND { ... }
```

The body is made **once** where `COND` holds and **not at all** where it does not. The condition is decided at elaboration, over the numbers in scope where the block stands — the document's values and inputs with a value, a module's numbers by their paths, a component's formals bound by its call, an enclosing block's index — so it is a choice of what the program *is*, never of where the solve goes. It stands where `repeat` does: at the top level of a document, in a component body, and inside another block.

`COND` is comparisons of two numbers, `<`, `<=`, `>`, `>=`, `==` and `!=`, joined by `and`, `or` and `not` (`or` binds loosest, then `and`, then `not`); a parenthesis holds a number or a condition. A number alone is no condition, and a comparison is no number. Each comparison is between two numbers of one dimension, as `+` is (§3.3): `relief > 0mm`, not `relief > 0`; two dimensions are **E103**. The comparison is exact, with no tolerance, so `<` and `==` are never both true. Every comparison is worked out, so a mistake on a side `and` or `or` could skip is still said.

**[0.91] Identity.** Two names of which one is no number compare by what they stand for: `a == b` holds where both name the same thing — an entity, or a face (§6.21) — reached through every formal and binder (a formal is what its call gave it, a face repeat's binder its face), and `a != b` where they do not. `c in B` holds where what `c` names is one of the faces the face path `B` names (§6.9). These join numbers' comparisons under `and`, `or` and `not` alike, and are decided where the block is elaborated — inside a block over a solid's faces, once the faces are named (§12.8). A number on one side and something drawn on the other is **E103**, as is something drawn under `<`, `<=`, `>` or `>=`, and `in` over what names no faces; a name that stands for nothing is **E101**. `when` is thereby the language's one conditional: a switch is `when flag == 1 { … }`, never a `repeat` over a count of zero or one, which named what it made as copies.

**A `when` repeats nothing, so it has no index and no copy.** What its body declares is named as if written beside the block — `p_in`, and in a component `t.tip` — never with a copy's index, and is reached from outside by that name; where the condition does not hold, nothing of the body is made, and a name only it declares is undefined (**E101**). The numbers a body defines are its own. A statement inside it is one statement as any is (§12.7), reached once or not at all, so a solve writes its seeds back as at the top level (§4.3). An index binder (`as i`) and a body ending mid-joint (§6.6), which threads onto a next copy there is none of, are syntax errors.

A condition that reads a name nothing in scope gives a number is **E101**; one that reads an unknown of the drawing — an input with no value, a formal no call binds, a value over either — is **E040**: the block is made before there is a solve to give it a number. A measurement of the solved drawing is **E107** (§6.14).

```
when design.tip_relief > 0mm {
  n := views.FoldedView(p, trace.normal)
  pinion_inner := Topping(tooth.inner, cutter.ax, n.view, …)
  pinion_outer := Topping(tooth.outer, cutter.ax, n.view, …)
  pinion_top := std.HalfSpace(cutter.ax.p1, cutter.ax, d: -design.addendum * normal_module / 2)
  pinion_topping := crown.tooth.Tooth(pinion_inner.cone, pinion_outer.cone, cutter.tooth_base,
    pinion_top, grow: 0mm, …)
}
pinion_relief := Generated(pinion.body, …, reference.pinion_topping.crown, …)   // in a `when` too
```

**[0.75]** A `when` is what ends a recursion (§8): its condition reads the numbers the call gave, and a level where it does not hold makes no further call.

*Non-normative:* a switch used to be written as a `repeat` over a computed 0 or 1 — `repeat min(ceil(relief / 1mm), 1) { … }` — which depended on the unit it divided by, said nothing of its intent, and named what it made as a copy (`tooth_relief[0]`).

---

## 13. Gauge fixing

Well-posed models are typically invariant under rigid motion; the Jacobian is rank-deficient by design. The language names this freedom rather than letting the solver pick:

```
fix((0, 0)) center            // holds a point: removes 2 DOF
fix(x == 0) p                         // holds one coordinate: removes 1 DOF
fix(r == 25) c                        // holds one of an entity's own numbers: removes 1 DOF
```

**[0.34]** A `fix` names an entity and states each number of its own it holds, pinned with `==`
under the field it is (`x`, `y` of a point, **[0.42]** and `z` of one standing in space; `r` of a
circle or arc; **[0.42]** `x`, `y`, `z` of a plane's
place and of an axis's direction, **[0.43]** and `px`, `py`, `pz` of an axis's place). The values are expressions over the parameters in scope, with units, and MAY
NOT read geometry. A `fix` is a gauge: it takes the numbers out of the solve at the values stated
and adds no equation, so it never takes part in a conflict set. **[0.52]** Two holds of one number
are one gauge where they state one value; where they differ, each statement is **E031** and the
number is not held — no order of statements may decide which wins (P2). Two orientations of one
triangle (§9.6) choosing opposite roots are **E031** alike. It is applied before the seeds
that read geometry (§6.4), which read a held number where it is held and never move one; a seed
for a held number is never read. A `fix` that states no number, writes one as a selector
(`fix(x: 0) p`) or unnamed (`fix(5) c`), or names a field the entity does not have is **E040** /
**E105** at what was written; so is one stating a number twice (`fix((3, 4), (1, 1)) p`,
`fix((3, 4), x == 1) p`), as any relation giving one slot twice is **E040** at the second. Implementations MUST report residual gauge freedom (rank deficiency
whose null space is spanned by rigid motions) with the suggestion to add a `fix` (**W103**), and
MUST distinguish it from genuine under-constraint.

### 13.1 Document state travels on its statement **[0.2]**

A document generally carries more than the program: where an annotation was dragged to, which of several solutions the drawing is on, and whatever else a tool needs to reopen a drawing as its author left it.

> **Every such datum MUST be attached to the statement it qualifies, and MUST NOT be keyed by a statement's position in a body or by an entity's index.**

This is P2 applied to the document rather than to the program text, and it is stated separately because it is the half implementations get wrong. Reordering a body is required to preserve meaning (P2); a body can be reordered by an editor, by a code formatter, or by an implementation's own printer. Two keys make that reordering destructive:

- **position in a list.** An annotation stored as "the 7th constraint's placement" follows the 7th position when the 7th statement moves, so it silently reappears on some other statement's annotation.
- **an entity's index.** A recorded solution branch stored as a triple of point indices goes *inert* when the points are renumbered: the document still carries it, a reader still loads it, and fewer of them apply. Nothing reports anything.

Both failures are silent, and both are invisible to any test that checks only the solution set — which is why 0.1 could assert P2 in the parser and lose it in the file format. Where a datum has no statement to ride on, it MUST name what it qualifies (an entity by name, a solution branch by the names of the points that orient it) rather than by index.

**[0.17] A recorded solution branch is one record of one triangle.** Three points can be named six ways, and a chirality named in two of those orders is the same fact with the sign turned. An implementation MUST therefore record a branch **canonically** — one order of the three points, with the sign read against that order — and MUST normalise a record it reads into that form. Keyed by whichever order each writer happened to use, a document's stated orientation and the same choice as the implementation constructs it are two records that never meet: the stated one matches nothing and decides nothing, and the constructed one, written back out as a statement, names a different triple. Both are silent, which is this section's subject.

**[0.20] Annotation placement is drawing state.** A `.svd` drawing identifies a named
model dimension or measures named geometry. Model source MUST NOT serialize a callout's
placement, and a drawing reference MUST NOT use a statement's position or an entity's internal
index. Missing references MUST be diagnosed. Solver hints and recorded solution branches
remain model state.

### 13.2 Presentation **[0.20]**

Styles and presentation classes are defined in [Solvent Drawing](docs/solvent-drawing.md).
Model source MUST reject presentation statements and clauses. A renderer may assign implicit
roles such as hidden edges or closure lines without adding source-level classes to geometry.
Model serialization and editor reconciliation MUST omit presentation state. An editor MAY
provide an automatic preview for a model without a drawing file.

---

### 13.3 Geometry roles and private members

`private`, `construction`, and `centerline` are optional prefixes on geometry declarations,
named chains, and component instances. They may be combined in any order, once each:

```solvent
private construction layout := Polygon(center, ref, n: n, r: pitch_r, phase: phase)
construction centerline ax := line(a, b)
```

Members are public unless marked `private`. A private name is accessible from the enclosing
component's body and its repetition blocks. A private instance hides its member paths from
outside that enclosing component. Its own body can still use its members. Private members of
a nested component remain private to that nested component. Forward references obey the same
access rules. An inaccessible member is **E101**, including in hints and component arguments.
A root-level private name is local to that model and unavailable to `.svd` paths.

Explicitly passing private geometry to another component grants access through that formal;
it does not copy the geometry or change its original visibility. Passing an instance as a group
grants its public interface, not access to private members inside it. Public geometry can expose
its constituent entities (for example, `hole.center`) even when their declaration names are private.
Privacy restricts names, not geometric identity, solver participation, or editor inspection.

`construction` designates supporting geometry; `centerline` designates an axis or center path.
They are independent roles, not arbitrary presentation classes. Both remain ordinary geometry
for constraints and explicit face/solid construction. Neither adds parameters, residuals, or
geometric relationships. Roles on an instance apply to geometry created inside its expansion,
including unnamed children, but never to borrowed arguments. Referencing a construction point
as a hole's center does not make the hole construction geometry. A modifier on a chain applies
to its named traversal and declared links, not to links borrowed by reference.

The editor displays supporting geometry. Authored sheets hide construction geometry by default
and expose `.construction` and `.centerline` selectors for drawing styles. Centerlines default
to a long/short dash pattern. Privacy alone does not hide geometry. Selecting a public component
or the whole model can render its private geometry; a drawing cannot name a private member directly.
Model source edits preserve these semantic annotations; flat geometry export preserves roles.

## 14. Elaboration semantics

Elaboration lowers a program to the **kernel form** consumed by solvers. The pipeline is normative in effect, not in mechanism.

### 14.1 Phases

1. **Instance expansion.** Recursively inline component bodies for the root's instance tree, freshening names by instance path. `repeat`/`cycle` unroll; **[0.72]** a `when` is decided and its body inlined once or dropped. `ring` either unrolls to `cycle` + symmetry constraints (§12.3) or lowers to quotient form (§12.4); the two MUST be solution-equivalent.
2. **Alias resolution.** Union-find over all names, merging classes for: entity-typed arguments and constructor entity-arguments. Each class gets one representative entity. Type mismatch within a class is an error (**E040**).
3. **Definitional substitution.** Definitional equalities (constructor value-arguments, and values and inputs with a value — `NAME := EXPR`, `param NAME := EXPR`, §6.3) are substituted, METAFONT-style: they are not residuals and consume no solver iterations. A cyclic definitional dependency is an error (**E041**).
4. **Constraint collection.** Predicate statements, `==` equations, derived path incidences (§10.4), symmetry constraints, and gauges are collected into the constraint store.
5. **Path assembly.** Fragments compose per §10.5 into boundary curves attached to the model as derived objects.

### 14.2 Kernel form

The kernel is a **bipartite entity/constraint graph, quotiented by group actions**:

```
Kernel := {
  groups:      [ { id, order N, ax: EntityRef } ],
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

### 14.4 Modules **[0.12]**

```
use engine.dims
use engine.parts
use std (horizontal, vertical)
```

A **module** is a Solvent document read for its components and relation words. `use NAME` at the top level of a document — never inside a body — asks for one; `NAME` is a dotted path, and **what it resolves to is the host's question**: an implementation with a working directory resolves `engine.parts` to `engine/parts.sv` beside the document, one without a filesystem resolves it against whatever library it carries, and both fall through to the other in that order. The core takes text and never opens a file. A module contributes exactly its **component definitions**, its **relation words** (§9.9) and its top-level **values, inputs and groups** (§6.3, §8.1), available at the importing root; its own loose statements — its drawing — are not drawn, so a document that is also a library (`gear.sv`) is a module as it stands. A module's own `use`s are followed the same way, each module linked once however many times it is asked for, so a diamond is one copy and a cycle terminates. A module a host cannot resolve is **E070**, at the `use`.

**[0.30] [0.48] Nothing is imported bare unless the `use` names it.** A file reaches a module's component, relation word, param or group by the module's **full path**, as its `use` spells it: `engine.parts.Crank(…)`, `engine.dims.bore`, `components.dims.vtwin_dims` — or bare, where its `use` lists the name in brackets:

```
use std (horizontal, vertical)
use engine.parts (Crank)
```

A value or group so named is read in the file's root body (components are closed, §5); a component may be called from anywhere in the file, and a relation word written anywhere in it. A module names its *own* definitions bare. Only a module the file itself `use`s may be named — one reached through another module's `use` is that module's business, and naming it is an error that says which `use` to write. So two modules may define one name, and only two definitions in one file are **E071**. A relation word is reached only bare: there is no `a std.horizontal b`, so a file writing one imports it.

**[0.48] Imports.** Any name a module defines may be listed — a relation word (both fixities of its spelling), a component, a value or input, a group. The full path still works beside the bare name. Imports are **per file and not transitive**, as `use` is: a module's imports are its own, and a file importing from a module that imported a name has not imported it. There is **no prelude**: `use std` alone imports nothing bare, and a file writing `a horizontal b` says `use std (horizontal)`. A listed name the module does not define is **E101**, at the name; the same name imported twice, or from two modules, is **E071**; an import of a name the file defines itself at its top level — a component, a word, a value, a group, an instance or a declaration — is **E071**; and an import of a name that is a word of the language's own (a keyword, a constraint word, a built-in function or constant) that is not a relation word of the module's is **E071**, read by its path instead.

The standard datums follow the rule: `std.front` needs the file's own `use std` (with or without names). A drawn callout of a dimension written `engine.dims.D` shows `D`. A module's own errors — a parse error, a faulty `param`, a faulty relation word or import — are reported to a reader of the document *at the `use` that brought the module in*, with the module's name, line and column in front of the message, since that line is the one the document can edit.

**[0.21] Standard datums.** **[0.42]** `use std` makes the standard axes `std.x`, `std.y`,
`std.z` and `std.back`, the standard planes `std.front`, `std.top`, `std.side` and `std.up`, and
`std.origin` available (§6.7). Every one is held: the axes outright, the planes at the world
origin, and `std.origin` a point drawn in `std.front` at its `(0, 0)`. `std.front` is `u: x, v:
z`, the plane a 2D drawing is drawn in. They are one shared expansion of `std`'s ordinary
`StandardDatums` statements, present whenever the document says `use std`, named or not, so a
workspace can offer the planes as places to draw; being held, they add no freedom. An explicit
root declaration or instance named `std` takes precedence over the standard binding.

A plane argument does not assign membership. `Part(axes, …)` draws its geometry in space unless
something puts it in a plane; `Part(axes, …) in std.front` assigns membership explicitly, as does
calling it inside `in std.front { … }`. A part measured against a turned frame is handed one,
`axes := std.Turned(o, up) in std.front`, which carries its plane (`axes.axes`) and its axis line
(`axes.u`). With the dimension module used, the upright cylinder's preview is:

```solvent
preview {
  unit mm
  in std.front {
    up := point
    fix((0, 40)) up
    axes := std.Turned(std.origin, up)
    cyl := Cylinder(axes, fw: components.dims.fwA, dims: components.dims.vtwin_dims)
  }
}
```

The name `cyl` is retained because the part drawing references `cyl.body`; its bore follows the
frame's u axis.

**[0.21] A component file may provide its own preview.** At most one `preview { … }`
block may appear at file scope. Its body contains ordinary model statements, including units,
parameters, datums, instances and constraints:

```solvent
use std

component Part(f: plane, width: Length) {
  // the reusable design
}

preview {
  unit mm
  part := Part(std.front, width: 20mm) in std.front
}
```

When the file is opened as a model, including through a drawing's `model … from`, the preview's
statements join the file's ordinary root body and solve together. A preview adds no namespace,
implicit coordinate system, execution order, or separate solve. When the file is imported with
`use`, the whole preview is omitted: its geometry, values, inputs and unit
statement are not exported. Component definitions and imports remain at file scope; a preview
cannot be nested. Presentation still belongs in `.svd` files. Source editing in a preview MUST
preserve the block and place newly drawn geometry inside it.

Hosts resolving files SHOULD try the model's directory and then its ancestor directories,
nearest first, before falling back to the shared library. Thus opening
`components/cylinder.sv` directly can resolve `use components.dims` from the same project as
an assembly beside the `components` directory. All transitive imports use the same root model.

*Non-normative:* an implementation may parse a module with its spans offset past everything linked before it, so that every span in a linked program is one integer into one virtual text and no consumer learns a second coordinate; a splice on the document then walks the root body alone, which no module span is ever in.

---

## 15. Solver contract

The numerical method is unspecified. Whatever the method, a conforming solver:

- MUST treat alias classes as single variables (no residuals for binding — P1).
- MUST apply definitional substitutions before iteration (§14.1 phase 3).
- MUST use hints as initial values only (Invariant H).
- MUST verify all inequalities and declared path orientations on any reported solution.
- **[0.78]** SHOULD find a root where every bound holds from a start that says nothing (every point unseeded): a document whose words choose all its roots has one solution, wherever it begins.
- MUST NOT report a solution to a `ring` program that is not symmetric (§12.3).
- SHOULD decompose along component boundaries (P4) and solve `ring` in the quotient (§12.4).
- MUST report the diagnostics of §16 with source spans; "solver did not converge" without a structural diagnosis is a nonconforming failure mode when a structural diagnosis is computable (rank analysis at the seed or at the failure point).

---

## 16. Static and solve-time diagnostics

### 16.1 Errors

| Code | Condition |
|---|---|
| E001 | redeclaration within a body |
| E002 | a block's index or edge binder over a name already in scope |
| E003 | a component instantiation nothing ends: reached again while it is being expanded, given the numbers it was given there, or nested past the implementation's depth (§8) **[0.75]**; an interface holding itself through its members (§8.1) **[0.84]** |
| E004 | a positional argument that binds a value parameter, or that follows a labelled one (§4.1) **[0.17]** |
| E010 | withdrawn: an undecorated `tangent` between circles reads its branch off the seed (§9.5) |
| E011 | closed path winding contradicts declared orientation |
| E012 | ambiguous or conflicting path-fragment composition |
| E013 | boundary fails to close where closure demanded |
| E014 | withdrawn: a hint expression reading an unseeded quantity reads its provisional seed (§6.4, §11) |
| E015 | a `ring`'s index read inside it (**[0.47]** any read, not only a hint) |
| E020 | `next`/`prev` where no `cycle` closes the copies: in a `repeat`, or in no block |
| E021 | external entity referenced in `ring` not provably invariant |
| E022 | nested `ring` (if unsupported) |
| E023 | **[0.47]** what a `ring` cannot turn: a centre neither a point nor an axis held in its direction, a plane, axis or motion declared in it, a point outside its centre's view (or not in space, about an axis), a hold on a turned copy, a relation reading a turned curve |
| E030 | retired **[0.34]**: a `fix` states its numbers (E040 where it does not) |
| E031 | two gauges that disagree: a number held at two values, or a triangle oriented both ways (§13) **[0.52]** |
| E040 | type mismatch within an alias class |
| E041 | cyclic definitional dependency (**[0.18]** a solid made of itself, §6.9; a plane folded from itself until 0.42) |
| E050 | inconsistent system (no solution); report a minimal infeasible subset when computable |
| E060 | a point put on two different planes (§6.7) **[0.10]**; **[0.42]** a circle, an arc or a spline over a point standing in space (**[0.126]** a spline there is a curve in space, §6.4; **[0.127]** an arc there is drawn in the plane through its points, a circle in the one square to its `normal:`) |
| E061 | `project` refused: a point on no plane, both on one plane, or parallel planes (§6.7) **[0.10]**; a point drawn in a plane asked whether it is on it, or how far off it along its normal **[0.24]**; two points of one view, or a point with itself, measured where the row is identically nothing **[0.46]** |
| E062 | a word across planes, or over a point in space, with no meaning in space (`horizontal`, a run or rise, `tangent` between drawn figures, a curve's contacts) (§9.2) **[0.24]** **[0.42]** |
| E064 | withdrawn **[0.42]**: was a solved view the model cannot hold (`fold: along`, `through:`) **[0.23]** |
| E065 | a relation in space degenerate at the solve: views a `project` relates that came out parallel, lines whose skew distance is stated that came out parallel (§6.7) **[0.23]** |
| E066 | withdrawn **[0.42]**: was `against` between planes that turn apart **[0.23; narrowed 0.26]** |
| E067 | a plane that cannot stand on its held axes: two wholly held axes that do not meet or run alike, or a plane held where its held axes are not — a plane's axes pass through its origin (§6.7) **[0.43]** |
| E070 | a `use` nothing resolves (§14.4) **[0.12]** |
| E071 | a component or a relation word defined twice in one file (§14.4, §9.9) **[0.12]** **[0.30]**; **[0.48]** a relation word defined over a word of the language's own, and an import of a name twice, from two modules, over the file's own definition, or over a built-in word (§14.4); **[0.84]** an interface defined twice in one file, or over a component of the file (§8.1) |
| E080 | a face or a solid the model cannot build (§6.8, §6.9) **[0.18]**: a loop that does not close, an edge that is not a line, an arc, a circle or a point, a circle standing *in* a loop rather than being one, **[0.19]** an edge that meets neither neighbour and so is walked two ways, a straight loop with fewer than three corners, edges in two planes, a swept solid written over anything but one face, a body made of what is not a solid, a prism swept nowhere, a feature written into a solid that is a face swept rather than a body; **[0.42]** a face over a point standing in space, and a prism's side generating in a plane the solve places (§6.15.2) |
| E081 | invalid revolution axis (§6.9) **[0.18]** |
| E082 | a face of a body that the body no longer has (§6.9) **[0.18]** |
| E084 | a section whose cutting plane is not parallel to the view it is drawn in (§6.11) **[0.18]** |
| E101 | a name nothing in scope declares, or a member a scope cannot reach (§3.3, §5, §13.3) **[0.41]**; **[0.84]** a member an interface does not declare, read through a formal it types, and a type naming no interface (§8.1) |
| E102 | **[0.48]** a relation word nothing defines where it is written (§9.9), naming the import that would where a module defines it |
| E103 | a shape or a number the model cannot build: more children than the kind has slots (§6.1), an input whose value is not its type (§6.3), a seed reading a scalar that is not there (§6.4), a curve the model cannot trace (§6.5), an extent of the wrong dimension (§6.9), iteration over what is not a named chain (§12.8), **[0.84]** a group or an instance given where an interface asks for a member it lacks or holds of another type, and an interface's `Side` member (§8.1) |
| E105 | a `fix` of a number the entity does not have (§13) **[0.34]** |
| E107 | a measurement of the solved drawing where a number is needed before the solve (§6.14) **[0.27]** |

**[0.18]** The reference implementation raises all four (**[0.42]** E083 is gone, with `against`). A negative sweep and a picture asked of the wrong kind are **E040** by the rules those codes already state (a magnitude written with a sign, §9.2 **[0.17]**; a kind mismatch, §14.1), and an unresolved name in any of these statements is **E101**, so no new code is spent on either.

### 16.2 Warnings and lints

| Code | Condition |
|---|---|
| W100 | `coincident(p, q)` where making `p`,`q` one entity would suffice — "consider binding instead of constraining" |
| W101 | withdrawn: frames fully welded by constraints — `Frame` was folded into `Plane` in 0.15, and there is no `weld` |
| W102 | `abs(angle(...))` without a disambiguating orientation predicate |
| W103 | rank deficiency spanned by rigid motions — "add a fix" |
| W104 | under-constrained: report the number of residual DOF and, when computable, a basis of unconstrained motions attributed to source entities |
| W105 | consistent redundancy: constraints dependent on others; report the dependent set with spans |
| W110 | an expression that cannot be computed: the last number stands |
| W112 | a `param`, formal or block binder declared over a built-in name (§3.3, §5) — the built-in is what an expression reads **[0.17]** |
| W113 | two planes lying on one another, with a relation reading points drawn in each: one plane in space, read twice (§6.7) **[0.42]** |
| W114 | a free curve whose length nothing holds and no length makes its energy stationary in (§9.10) **[0.57]** |

### 16.3 DOF ledger

Implementations SHOULD emit, on request, a degrees-of-freedom ledger: per alias class, its free DOF after definitional substitution; per constraint, its equation count; totals per component and for the model; gauge accounting. (§18.3 shows the gear's ledger.)

**[0.17]** They SHOULD also emit, on request, **where each name landed**: the value of every scalar of every entity the source names, keyed by that name and the field (`hinge.x`, `base.r`, **[0.42]** `side.z`, `t.x`). A report that says how many freedoms a drawing has and which statements disagree, and never where anything *is*, leaves its reader to recover coordinates by stating a `claim` and reading whether it is refuted. The names and the numbers are both already in hand; this is a serialisation and imposes no analysis.

---

## 17. Deferred and open issues (non-normative)

1. **3D lift.** **[0.10]** Multiview drawing is settled *without* one — §6.7: a `plane` is a frame with a constant attitude, a point is `in` a view, and `project` is the one equation two images of a point share; nothing three-dimensional is solved for, no true length is measured, and a view's attitude is never an unknown. (**[0.23]**–**[0.42]** revised this: a relation across planes is read in space, and planes, axes and points in space are unknowns of the solve, §6.7, §9.2. The stratification below holds: nothing past the sketch is an unknown.)

   **[0.18] The object is settled too, and on the same terms.** A face is a region of a plane (§6.8), a solid is a face swept or a term over other solids (§6.9), a plane may be stood off another (§6.10), and a view or a section of a solid is a picture the sheet asks for (§6.11) — so a part is written once and every drawing of it is a question, with no depth kept in step by hand. The stratification is what makes it affordable and is the thing to preserve: **nothing three-dimensional is an unknown**, every extent is an expression, and a solid owns no parameter, so the solver contract of §15 and the ledger of §16.3 are untouched.

   What remains open is the lift itself, and it is now a shorter list (**[0.28]** lofts are §6.9's). **Fillets and chamfers**, which need a name for an *edge* rather than for a face: the spelling is reserved, `body.block.side_l.near` — the edge where two named faces of one solid meet, in the path vocabulary §6.9 already uses, so that a boolean cannot renumber one. A **rigid-body mate solver**: joints between two solids in space, at which point something three-dimensional does become an unknown and P4's decomposition question is asked again one stratum out. Still open from before: the arc-branch rule needs a replacement (no global winding in 3D); and from §6.7, a solved-for fold (`fold: along l`) — **[0.23]** answered by `fold:`, and **[0.42]** by planes over axes.

   **[0.18]** One item of the old list is answered by §6.9 rather than deferred: "`ring` generalizes to rotation about a line" is what `about:` does for a **sweep**, and it needed no group action to do it, because a revolution is one solid and not *N* congruent copies. `ring` itself is still the cyclic-symmetry question of §12.3 and is untouched by this — the reference implementation goes on refusing the word until it can hold its copies congruent. The two were only ever adjacent.
2. ~~**Curve entities.**~~ **[0.2] Settled — see §6.5.** **[0.11]** A curve is a point of a component as one of its numeric formals runs (0.2's family of two expressions is retired), rather than an entity kind per curve. Involute, cycloid and trochoid are library code, and `tangent` and `curvature` against such a curve are §6.5's. What remains open is the path grammar's slot for a curve segment.
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
component Involute(c: circle, phase: Angle, u: Angle) {
  p := point(x: c.center.x + c.r * (cos(u + phase) + u / 1rad * sin(u + phase)),
             y: c.center.y + c.r * (sin(u + phase) - u / 1rad * cos(u + phase)))
}

component Flank(base: circle, root: circle, tip: circle,
                phase: Angle, u0: Angle, u1: Angle) {
  e := Involute(base, phase: phase).p over u in (u0, u1)
  lo := point
  hi := point
  lo coincident e hint(t: u0)
  hi coincident e hint(t: u1)
  lo coincident root
  hi coincident tip
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

The 0.1 source is kept below as written, since §18.2 and §18.3 walk through it. None of its spellings parse now: `port` (§7), `on(…)` and `angle(…) == e` as calls (§9.2), `param R = …` (§6.3), a radius as a constructor argument (§4.3), `frame` (§6.7) and `t: Tooth(…)` (§5) are retired, and `ring`, `path` and the `hint` statement are specified and not implemented.

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

  fix((0, 0)) center
  t.lead horizontal center
}
```

### 18.2 Elaboration walk-through

- **Aliasing:** `pitch.center`, `tip.center`, `root.center`, `f0.origin`, and `center` form one class. `Tooth`'s formals `root`, `tip` alias the gear's circles (entity-typed arguments). The three circles' radii are definitional (`R`, `R+m`, `R−1.25m` substitute out).
- **Ring lowering:** group G = C_N about `center`. Ring-local free entities: one representative each of `t.lead`, `t.trail`, `t.tl`, `t.tr` (orbit size N). `center` and the circles pass the §12.5 invariance criterion (`Fixed`). The gap constraints reference `(t.lead, +1)`.
- **Paths:** each Tooth contributes `lead → tl ⌒tip tr → trail` (ccw); the ring body contributes `t.trail ⌒root (t.lead,+1)`, inheriting ccw. Composition closes after N teeth and N gaps: one closed ccw boundary. All arcs take the default branch (ccw on their circles) — zero `rev` decorations, per §10.3.
- **Deduplication:** the arc-derived incidences `coincident(tip, tl)`, `coincident(tip, tr)`, `coincident(root, trail)`, `coincident(root, (lead,+1))` duplicate the explicit `coincident` constraints (the last after symmetry transport) and merge in the store.

### 18.3 DOF ledger (quotient system)

| Item | DOF / equations |
|---|---|
| `center` | +2 |
| circle radii | 0 (definitional) |
| `t.lead`, `t.trail`, `t.tl`, `t.tr` (representatives) | +8 |
| **Unknowns** | **10** |
| `coincident(root, lead)`, `coincident(root, trail)` | −2 |
| `coincident(tip, tl)`, `coincident(tip, tr)` | −2 |
| tooth span angle `== slot/2` | −1 |
| flank symmetry angle equality | −1 |
| gap angle `== tau/(2N)` | −1 |
| `fix((0, 0)) center` | −2 |
| `t.lead horizontal center` | −1 |
| **Equations** | **10** |
| `ccw(lead, tl, tr)` | 0 (inequality) |

Square system, full rank at the hinted seed; one Newton basin per §12.4, lifted to N teeth by orbit expansion. Expected solution: lead at bearing 0 on the root circle, trail at bearing τ/2N, tip corners inset symmetrically — a gear.

---

## 19. Grammar (EBNF)

```ebnf
program        = { use_decl | component | interface | word_def | preview | statement } ;
interface      = "interface" IDENT "{" [ formal { ( "," | NEWLINE ) formal } ] "}" ;
                                                                    (* §8.1 [0.84] *)
preview        = "preview" "{" { statement } "}" ;                  (* at most one, §14.4 *)
use_decl       = "use" IDENT { "." IDENT }
                 [ "(" IDENT { "," IDENT } ")" ] ;          (* a module, and names read bare, §14.4 *)
word_def       = ( IDENT IDENT [ "(" IDENT { "," IDENT } ")" ] IDENT  (* infix, §9.9 [0.48] *)
                 | IDENT [ "(" IDENT { "," IDENT } ")" ] IDENT )      (* prefix *)
                 ":=" ( relation | "{" { statement } "}" ) ;  (* several, §9.9 [0.50]; solids [0.69] *)
component      = "component" IDENT "(" [ formals ] ")"
                 ( "{" { statement } "}" | ":=" set_lit ) ;     (* a family of sets, §6.21 [0.49] *)
set_lit        = "{" IDENT "|" statement { ( ";" | NEWLINE ) statement } "}" ;  (* §6.21 [0.49] *)
formals        = formal { "," formal } ;
formal         = IDENT ":" type [ "in" IDENT ] ; (* [0.95] a point formal's plane *)
type           = "Int" | "Scalar" | "Length" | "Angle" | "Side"      (* §4.1 [0.17] *)
               | "group"                                             (* §8.1 [0.35] *)
               | "set"                                               (* §4.1, §6.21 [0.71] *)
               | ekw | "curve"    (* an entity, in either case: `f: plane`, `p: Point`; `Frame`
                                     was folded into `Plane` in 0.15 *)
               | IDENT { "." IDENT } ;  (* an interface, a group of its shape: §8.1 [0.84] *)

statement      = definition | input | relation | gauge | branch | block | in_block
               | unit_decl | body_rel | claim_over | energy | measured | hint | path_decl | frag ;
measured       = expr ( "<=" | ">=" ) expr ;   (* in a set's body, §6.21 [0.69]: each side a sum of
                                                  numbers and measurements, each times a number *)
measurement    = "distance" "(" ref "," ref ")"
               | "angle" "(" ref "," ref "," "at" ":" ref ")" ;  (* the material's angle [0.69] *)
energy         = ref ( "minimizes" | "maximizes" ) term { ( "+" | "-" ) term } ; (* §9.10 [0.53] *)
term           = [ "-" ] [ NUMBER "*" ] "integral" "(" expr "over"
                 ( IDENT | "(" IDENT "," IDENT ")" ) ")" ;
in_block       = "in" ref "{" { statement } "}" ;         (* membership, written once: §6.7 *)
unit_decl      = "unit" IDENT ;                                           (* §3.3.2 *)

(* §5 [0.29]: a name is defined by `NAME := VALUE` and in no other way, and what stands after
   `:=` says what the name is.  An element or an instance may also be written with no name. *)
definition     = { role } [ IDENT ":=" ] ( chain | instance )
               | IDENT ":=" ( expr | group | vector | curve | set_lit ) ;
                 (* a value, a group, a vector, a curve, a set (§6.21 [0.49]) *)
role           = "private" | "construction" | "centerline" ;      (* once each, §13.3 *)
input          = "param" IDENT [ ":" num_type ] [ ":=" expr ] [ hint_clause ] ;
                 (* §6.3 [0.41]: with no value an unknown, its type stated, seeded by `hint(E)` *)
num_type       = "Int" | "Scalar" | "Length" | "Angle" ;
group          = "{" member { "," member } "}" ;                  (* §8.1 [0.35] [0.36] *)
member         = IDENT ":" ( expr | group ) ;
vector         = "(" expr "," expr [ "," expr ] ")" ;      (* [0.45]: members x, y, z *)
member_key     = IDENT { "." IDENT } ;                     (* `x`, `dir`, `dir.x` [0.45] *)
chain          = link { joint link } [ "->" { infix } "close" ] ;
(* link, joint and infix follow §6.6: a link is an `element` with any prefix words before it, or
   a named link `(NAME := element)`; a lone element is a one-link chain. *)

(* §3.3.1: a number may carry a unit, and feet-and-inches is ONE literal — a space is what
   tells the readings apart, exactly as it does in a mixed fraction. *)
number         = digits [ "." digits ] [ ("e"|"E") [ "+"|"-" ] digits ]
                 [ " " digits "/" digits ]                                (* mixed fraction *)
                 [ unit_suffix ] ;
unit_suffix    = "mm" | "cm" | "m" | "km" | "in" | "ft" | "thou"
               | "deg" | "rad" | "grad"
               | "'" [ " " number "\"" ]                                  (* feet, and inches *)
               | "\"" ;

element        = ekw [ "(" ctor_arg { "," ctor_arg } ")" ] { trailer }
               | "point" "(" "x" ":" expr "," "y" ":" expr ")"   (* a computed point, §6.5 [0.13] *)
               | face_decl | solid_decl ;
ekw            = "point" | "line" | "circle" | "arc" | "spline" | "plane" | "axis"  (* axis [0.42] *)
               | "face" | "solid"                                            (* §6.8, §6.9 [0.18] *)
               | "surface" | "envelope" ;                                   (* §6.13–§6.15 *)
(* the trailing clauses are order-free: `hint(…)`, `knots […]`, `in REF`.  A
   place — `hint(at: t)`, `hint(at: c, bearing: …)` — is the same clause with `at:` and
   `bearing:` for keys, §6.4 [0.14] *)
(* §6.1: no list at all is the anonymous form — the kind's children are minted and reached as
   `l.p1`.  A slot is a name, a seed, or implicit; only an overfull list is E103. *)
trailer        = hint_clause
               | "knots" "[" number { "," number } "]"
               | "in" ref { "," ref } ;      (* membership of a plane, §6.7; several: a point [0.58] *)
hint_clause    = ( "hint" | "~" ) "(" ( expr | hint_item { "," hint_item } ) ")" ;
                 (* SEEDS, §4.3; `~` [0.44]; one keyless expr seeds an unknown, §6.3 [0.41] *)
hint_item      = member_key ":" ( expr | vector ) | vector  (* `(3, 4)`, `dir: (1, 0, 0)` [0.45] *)
               | "at" ":" ref | "bearing" ":" expr                  (* a place, §6.4 [0.14] *)
               | ( "toward" | "along" ) ":" ref | ( "by" | "turn" ) ":" expr ;   (* a step [0.33] *)
ctor_arg       = [ IDENT ":" ] ( ref | hint_clause )       (* what the thing is made of, §6.2;
                                                              a plane's are `u:` and `v:`, §6.7 *)
               | IDENT ":" expr                            (* a motion's or an envelope's numbers,
                                                              §6.14, §6.15 *)
               | sweep_arg ;                               (* how a solid is swept, §6.9 [0.18] *)
(* [0.42] a plane's `from:`, `fold:`, `offset:` and a basis of triples are retired *)

(* §6.9 [0.18]: the sweep stands in the brackets with the face, being what the solid is made
   of; numeric extents are expressions and never unknowns. `through:` instead names a solid
   whose material bounds determine an extrusion after solving. `about:` is a revolution.
   `under:` a motion places a solid at its pose `at:` an angle, or sweeps it over `from:`/`to:`,
   or [0.88] `through:` a solid, over the roll during which it meets that solid's material. *)
sweep_arg      = ( "from" | "to" | "depth" | "sweep" | "at" ) ":" expr
               | ( "about" | "through" | "along" | "under" ) ":" ref
               | "sense" ":" ( "cw" | "ccw" ) ;

(* §6.8, §6.9 [0.18].  Both are ordinary elements — `face` and `solid` are element
   keywords — and are written out here for what their one list slot holds. *)
face_decl      = "face" "(" face_arg { "," face_arg } [ "," "->" "close" ] ")" ;
face_arg       = ref | ("edges" | "holes") ":" ref ;  (* §6.8 *)
solid_decl     = "solid" "(" solid_term ")" ;
solid_term     = ref [ "," ref ] { "," sweep_arg }                     (* a face swept *)
               | ref { "," ref } ;                         (* a body: its stock, then solids *)
body_rel       = ref ( "union" | "cut" | "common" ) ref ; (* the body rule, §6.9 — each its own
                                                     word: `common` [0.85] (`bound` [0.22]),
                                                     `union` [0.32] *)

(* §6.10: `against`, the 0.18 mate, is retired [0.42]. *)

(* §9.8 [0.18]: a claim about the object.  The three words are operators like any other, and a
   sweep says how the claims in its body are judged. *)
solid_claim    = [ "claim" ] ref ( "clear" "(" expr ")" | "fits" "(" expr ")" | "inside" ) ref ;
claim_over     = "claim" "over" ref [ "seen_from" ref ] "in" "(" expr "," expr ")"
                 "{" { solid_claim } "}" ;                                  (* seen_from [0.135] *)

(* [0.20]: solid views, sections, and annotation requests belong to .svd. *)

(* a curve, §6.5 [0.11]: a point of a component as one of its numeric formals runs.  0.2's
   family form and 0.3's `trace … where { … }` are retired and MUST NOT parse. *)
curve          = ( ref | ref "(" [ args ] ")" "." ref )
                 "over" IDENT "in" "(" expr "," expr ")" ;
instance       = ref "(" [ args ] ")" [ "in" ref ] ;   (* a used module's component by its full
                                                          path, §14.4 [0.30]; `in`, §6.7 *)
args           = arg { "," arg } ;
arg            = [ IDENT ":" ] ( expr | hint_clause ) ; (* `w: hint(60)` leaves the formal unbound
                                                          and seeds it, §5 [0.41] *)

(* §9.2: every constraint is a prefix or an infix operator; `name(args…)` is retired. *)
relation       = [ "claim" ] ( prefix_form | infix_form | orientation | solid_claim | rate_form )
                 [ hint_clause ] ;  (* §4.3; presentation placement belongs to .svd *)
rate_form      = rate rate_word [ op_args ] ( ref | rate ) (* §6.9 [0.116], §6.5 [0.119] *)
               | ref rate_word [ op_args ] rate            (* §6.5 [0.119]: either side *)
               | rate "distance" op_args ;                 (* alone, its component along `along:`,
                                                              §6.5 [0.119] *)
rate           = "rate" "(" ref "over" IDENT ")" ;
rate_word      = "perpendicular" | "parallel" | "angle" | "distance" ;
                                                           (* a word that reads a direction; two
                                                              rates in one relation are E103, but
                                                              over two formals one may be along
                                                              the second, §6.5 [0.121] *)
prefix_form    = IDENT [ op_args ] ref ;
infix_form     = ref ( IDENT | "@" ) [ op_args ] ref ;   (* `@` is `coincident` [0.44] *)
orientation    = orient "(" ref "," ref "," ref ")" ;      (* §9.2: a call *)
op_args        = "(" op_arg { "," op_arg } ")" ;
op_arg         = expr                                      (* the number it states *)
               | ref                                       (* a third entity: `symmetry` *)
               | IDENT ":" ( IDENT | number )              (* a selector, §9.2 [0.17] *)
               | IDENT "==" expr ;                         (* a pin, §4.3 *)

(* §4.3: a number inside `hint(…)` seeds; `==` inside an argument list pins.  The two are the
   whole of the seed/constraint classification, and they are told apart by the clause alone. *)

gauge          = "fix" "(" pin { "," pin } ")" ref ;       (* §13 [0.34] *)
pin            = vector                                    (* the point itself [0.45] *)
               | member_key "==" ( expr | vector ) ;       (* `x`, `r`, `half`, `dir`, `origin`,
                                                             `dir.x`, … [0.45] *)
                 (* the entity's own fields: `z` [0.42]; an axis's place `px`, `py`, `pz` [0.43] *)
branch         = "branch" "(" KEY "," [ "-" ] digits ")" ; (* a recorded root choice, §13.1; KEY is
                                                              the raw text up to the comma *)

path_decl      = "path" IDENT ":" orient "=" path_expr ;  (* §10; unimplemented, and its `=` is no
                                                              token since [0.29]: to respell *)
frag           = path_expr ;                               (* statement-level fragment *)
orient         = "ccw" | "cw" ;
path_expr      = ref seg ref { seg ref } ;
seg            = "->" | "~" ref [ "rev" ] "~" ;           (* `~` is `hint` since [0.44]: to respell *)

hint           = "hint" ref hint_clause ;                  (* §11; unimplemented *)

block          = ( "repeat" | "cycle" ) ( expr | IDENT "in" ref ) [ "as" IDENT ]
                 "{" { statement } "}"                (* IDENT "in" ref: §12.8 [0.27] *)
               | "ring"   expr "about" ref [ "as" IDENT ] "{" { statement } "}"
               | "when"   truth "{" { statement } "}"      (* §12.9 [0.72] *)
               | "initially" "{" { relation } "}" ;      (* §6.9 [0.116] *)

truth          = truth "or" conj | conj ;                 (* [0.72] *)
conj           = conj "and" negation | negation ;
negation       = "not" negation | expr cmpop expr | name "in" name | "(" truth ")" ;
                 (* `name "in" name` and a comparison of two names: identity, [0.91] *)
name           = IDENT { "." IDENT } ;
cmpop          = "<" | "<=" | ">" | ">=" | "==" | "!=" ;

ref            = ( IDENT | "next" | "prev" ) { "." IDENT | "[" expr "]" } ;
expr           = expr addop term | term ;
term           = term mulop factor | factor ;
factor         = NUMBER [ UNIT ] | "tau" | "pi" | ref | call
               | "(" expr ")" | "-" factor ;
call           = IDENT "(" [ args ] ")"                    (* polar, direction, angle, abs, ... *)
               | measurement ;                             (* in a `measured` statement [0.69] *)
addop          = "+" | "-" ;
mulop          = "*" | "/" ;
```

Parsing note: a statement is told by its first tokens. `NAME :=` is a definition, and the token after `:=` settles which: `{` a group, `over` in the statement a curve, a call an instance, an element keyword or a prefix word a chain, anything else a value (§6.3). `param` opens an input and `in REF {` a block of membership. A constraint word opens a prefix relation, or a call for `ccw`/`cw`, which also appear as orientation keywords after `:` in `path`, where context disambiguates. Otherwise a `ref` comes first, and the word after it, read by a lookahead past the dotted path, is a constraint word, `@`, or `cut`, `common` [0.85] (`bound` [0.22]) or `union` [0.32], which relate no geometry and have no residual for the operator machinery to settle them into. **[0.18]** `view`, `section` and `dimensions` open a statement and are therefore not names; **[0.20]** model source refuses them as presentation (§13.2). **[0.69]** A statement opening on a measurement, or comparing two sides by `<=` or `>=` outside every bracket, is a bound on measurements (`measured`).

**[0.2] Note on a number read verbatim.** After the `:=` of a value (0.2–0.6: the `==` after a predicate's closing parenthesis, which §9.2 retired), an implementation MAY take the rest of the logical line verbatim rather than tokenizing it, and hand that text to whatever evaluates dimension expressions. This is not laziness: `3 1/2` is three and a half and `31/2` is a division, and that rule belongs to one tokenizer. Two copies of it are two rules the moment one is edited. An `==` *inside* an argument list is the pin of §4.3 and is lexed normally; the two never meet. **[0.7]** A trailing `hint(…)` ends the verbatim region, since it is a clause of the statement and not part of the number.

---

## 20. Conformance checklist for a first implementation

A minimal conforming implementation provides:

1. Parser for §19; classifier assigning every statement to §4.2 classes **by the `hint(…)` / `==` mark (§4.3)**; Invariant H enforced by construction of the parser.
2. Elaborator: instance expansion **preserving statement identity (§12.7)**, union-find aliasing, definitional substitution, dedup store, `ring` lowering (quotient form or cycle-plus-symmetry — either, per §12.3, **and reported if unrolled**).
3. Invariance check §12.5 (syntactic criterion), gauge analysis (W103), DOF ledger (§16.3).
4. A numeric backend satisfying §15 — Newton on the quotient system seeded by hints is sufficient — with rank-deficiency reporting attributed to source spans.
5. Path assembly and closed-boundary export (the solved gear outline as a polyline+arc sequence).
6. **[0.2]** Document state attached to its statement, never to a list position or an entity index (§13.1).

Deliberately *not* required for v0: 3D, nested rings, constraint strengths, decomposition planning (P4 is a SHOULD). **[0.2]** Curve families (§6.5) are not required either, but they are no longer deferred: an implementation that wants involute or cycloid geometry has a way to say it, and needs no new entity kind to do so.
