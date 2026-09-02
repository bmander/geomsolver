# Solvent: a primer for agents

Solvent is the language a drawing in this project *is*. You do not place geometry in it; you
declare entities and state what must be true of them, and a solver finds coordinates satisfying
everything at once — so a sketch is a set of claims, not a sequence of drawing commands, and
reordering the statements cannot change what it means. One clause carries the whole discipline:
a number inside a **`hint(…)`** clause is a **seed**, only where the solver starts looking and
free to be moved, and every other number is not — `==` states a **constraint**, which the solver
must satisfy and may never rewrite; delete every seed in a document and the set of valid solutions
is unchanged. Your job on a sketch task is therefore not
to compute positions — that is the solver's — but to say enough true things that exactly one
drawing satisfies them, and then to check that the diagnosis agrees. This document is what the
implementation actually accepts, verified against it; `solvent-spec.md` is the normative
specification and describes several constructs (`hint` as a statement, `path`) that the
parser does **not** yet accept.

---

## 1. Formal definition

### 1.1 Lexical

```
program     ::= statement*
statement   ::= <one of the forms in §1.3>            terminated by a newline or ';'
comment     ::= ' //' … end-of-line | '/*' … '*/'
identifier  ::= [A-Za-z_][A-Za-z0-9_]*
number      ::= decimal, optionally in exponent form; `3 1/2` is a mixed fraction (three and a
                half); a unit may follow — `80mm`, `45deg`, `0.5rad`, `6"`, `1' 6 3/16"` (§1.6)
```

Whitespace is insignificant except as a separator. A newline ends a statement, **except** inside
brackets, and except when the line ends with a chain joint (§1.7).

### 1.2 The two marks

| written | class | meaning |
|---|---|---|
| `hint(x: 0, y: 0)`, `hint(r: 25)`, `hint(t: 0.4)` | seed | where the solve begins; the solver may move it |
| `distance(80)`, `t == 0.4` | constraint | a claim the solver must satisfy and may never rewrite |
| `param w = 100` | neither | a number worked out while elaborating |

**The brackets after a name are what the thing is made of; the `hint(…)` after them is where the
solve begins.** That is why `circle c(center: o) hint(r: 25)` and not `circle c(center: o, r: 25)`:
the centre is structure and the radius is a guess, and one pair of brackets said both.

`hint` marks *the solver revises this*. A callout placement (`a distance(80) b at (12, -4)`) keeps a bare
`at`: it is inert too, but it records where a person dragged a dimension and no solve touches it.

### 1.3 Statement forms

```
use NAME[.NAME…]                        a module: its components and params join the document (§1.11)
unit NAME                               what the document's numbers are in (§1.6)
param NAME = EXPR                       a number worked out while elaborating; never an unknown
KIND [NAME][(CHILD | hint(x: E, y: E), …)] [hint(SCALAR: E, …)] [knots […]] [class NAME…] [in REF]
                                        an entity declaration (§1.4); the name is optional
plane [NAME][(origin: R, toward: R[, from: R, fold: E | , u: (E, E, E), v: (E, E, E)])]
                                        a view: a frame with an attitude in space (§1.4)
in REF { statement* }                   every declaration inside is drawn in that plane (§2.10)
style .NAME { PROP: VALUE; … }          what a class looks like (§1.10)
WORD[(ARGS)] REF | REF WORD[(ARGS)] REF   a constraint, prefix or infix (§1.5)
    [hint(SLOT: EXPR, …)] [class NAME…] [at (t, r)]
claim <constraint>                      an assertion, judged and never solved for (§1.9)
ground REF                              pin both of a point's coordinates
fix REF.field                           pin one scalar, e.g. `fix c.r`
ccw(a, b, c) | cw(a, b, c)              record a root choice; contributes no equation
NAME: Component(ARGS) [in REF] [class NAME…]   instantiate a component, drawn in that plane if said
component NAME(FORMALS) { statement* }  define one
port NAME: KIND [hint(…)] | port NAME = REF   export an entity across a component boundary
port NAME = (XEXPR, YEXPR)              a computed point, drawn only as a curve (§1.8)
repeat N [as i] { … }                   N copies, unrelated
cycle N [as i] { … }                    N copies that close: `next` and `prev` are in scope
ring N about REF [as i] { … }           a cycle claimed to be cyclically symmetric (§1.7)
curve NAME = INSTANCE.POINT over FORMAL in (A, B)         a curve: an instance's point, as one
curve NAME = Component(ARGS).POINT over FORMAL in (A, B)   of its numeric formals runs (§1.8)
```

References are `name`, `name.field`, or `name[expr]` (which copy of a repeated statement; the
expression may read any `param` or binder in scope). The index may stand on a dotted name —
`l.p[1]` is copy 1 of the `p` a `repeat` inside the instance `l` declares — and a field of the copy
follows it (`l.e[2].p1`). The copy may be an instance as well as a declaration: `cyl[0].small` is
the `small` of copy 0's `cyl`, which is how one cylinder of a repeated row is reached from outside.

### 1.4 Entities

| kind | children | own scalars | has ends? |
|---|---|---|---|
| `point` | — | `x`, `y` | — |
| `line` | `p1`, `p2` | — | `p1 → p2` |
| `circle` | `center` | `r` | — |
| `arc` | `center`, `start`, `end` | `r` | `start → end`, counter-clockwise |
| `spline` | `ctrl` (a list) | — | — |
| `ellipse` | `center`, `major` | `b` (minor radius) | — |
| `frame` | `origin`, `toward` | `c`, `s` (the unit rotor) | — |
| `plane` | `origin`, `toward` | `c`, `s` — and a constant basis `u`, `v` in space | — |
| `curve` | `args` (a list) | — | — |

Every scalar is seeded by name in the trailing clause — `point p hint(x: 0, y: 0)`,
`circle c(center: o) hint(r: 25)`, `arc a(center: c, start: s, end: e) hint(r: 5)`. Keys may come
in any order; an omitted coordinate is 0, so `point p hint(y: 12)` and `point t` are both legal,
and an omitted radius is *computed* — an arc's from its centre and start, an ellipse's minor from
its major — never 0, where no on-circle row has a gradient to move it. A
point with no clause at all starts where the implementation puts it — off the origin, and apart
from every other unseeded point, since two points on top of each other put a distance between
them where it has no gradient — and a solve writes the pose it reached back as the clause. The
clause is order-free against `knots` and `class`, exactly as those two are against each
other. Children may be given positionally or by label; a label is what lets you omit an earlier
one (`line l(p2: c)` leaves `p1` for a chain to thread). An arc is a centre and two *real* points,
so its ends drag and constrain like any others.

**Children need not be named — any slot may be left implicit.** Write no argument list and the
kind's children are made for you, unnamed; leave one slot out (`line l(p2: c)`) and just that
child is minted; a slot may also hold a `hint(…)` instead of a reference, which is an anonymous point and
where its solve begins:

```
line   l                                          two points: l.p1, l.p2
circle c hint(r: 25)                              an unnamed centre, a seeded radius
arc    a                                          a.center, a.start, a.end
line   l(hint(x: 0, y: 0), hint(x: 60, y: 20))    two points, seeded
line   alt_a(A, hint(x: 15, y: 5))                one named end and one not
```

**The dotted path is the name.** `l.p1` is an ordinary point — it constrains, drags, is picked,
and is what a dimension states itself against. Name a point when something says it twice; these
said it once, and six statements become three.

**The element's own name is optional too**, independently of everything after it. `line` alone
is a statement — a line with no name, implicit children and no hint — and so are `line(p1, p2)`,
`point hint(x: 3, y: 4)`, `arc(center: c)` and `line class construction`. The token after the
kind keyword decides: a word that may follow a declaration — another element keyword, a
constraint word, `hint`, `knots`, `class`, `at`, `close` — can therefore no longer be a
declaration's name. An anonymous element draws, drags and deletes without ever being named; the moment the
source must *reference* it (a constraint applied from the app, a dimension stated on it), a real
name is spliced into the declaration — the same bargain a solve strikes with an unwritten
`hint(…)` clause. `curve` keeps requiring a name: its form is `curve name = family(…)`, and the
name is what the contacts address.

**A seed may read geometry.** The numbers in a `hint(…)` clause may name another entity's scalar
— `hint(x: k.center.x + k.r, y: pin.y)` — and read that scalar's *own seed*, never a solved
value, so the clause is still only where the solve begins. Two spellings name a place outright:
`hint at pin` starts a point where another one starts, and `hint at k bearing (90deg)` puts it on
the circle's edge at that bearing. Inside a component the names are the formals' (`k1` reads as
whatever circle was passed), so a component can seed its own branch choice — a tangent span's two
contacts, a rod's small end above its pin — from the geometry it is written over. Seeds settle in
statement order; a seed that reads one written below it reads that one's provisional start. Where
the document names a `unit`, a geometry read is a length, so write `pin.x - 10mm` and not
`pin.x - 10`. A `param` may not read geometry: it feeds constraints, and a seed must never change
what a document says.

**All the children, or none.** A written slot carries a name or a seed; there is no bare `hint`
meaning "anonymous and unseeded", because writing no list at all says that. `line l(a)` is still
the error it always was, and so is `spline s` — a control polygon has no arity to conjure children
from.

A `frame` is a datum: an origin, a point it is pointed at, and a unit rotor `(c, s)` of its own,
slaved to the chord between them by two intrinsic constraints the declaration implies — so it
draws nothing, picks as nothing, and adds no freedom beyond its two points. What it is *for* is
`f.angle` (below, §1.8): the datum's bearing as a number a trace block may read. You never seed
`c` or `s` yourself; they are computed from the points and re-solved with everything else.

### 1.5 Constraints

**Every constraint is a prefix or an infix operator.** The word stands before its one operand or
between its two; everything else — the number, a selector, a third entity — goes in parentheses
**on the word**.

```
horizontal line1                    point1 horizontal point2
radius 25 circle1                  point1 distance(80) point2
distance(6) line1                   point1 symmetry(line1) point2
ground p1                           l1 angle(30) l2
fix c.r                             line1 tangent(side: -1) circle1
```

| word | how it stands | what it relates |
|---|---|---|
| `on` | infix | a point to a line, circle, arc, spline, ellipse or curve — **five** constraints, one word |
| `distance` | infix | two points; `along: x` / `along: y` for the run and the rise; a point and a line; two lines; two *concentric* circles or arcs (the radial gap between them — over two centred apart it is refused, since it reads neither centre) |
| `distance` | prefix | a line — the distance between its own ends |
| `tangent` | infix | a line and a circle (`at: p1`/`p2` for a tangency at that end), two circles (`external: bool`), an arc and a line (`at: start`/`end`), a spline, an ellipse or a curve and a line |
| `equal` | infix | two lines (a length), two circles or arcs (a radius) |
| `curvature` | infix | a spline, an ellipse or a curve and a circle — the circle becomes the rim's own radius where it touches; a *traced* curve has no curvature to state, and is refused |
| `horizontal`, `vertical` | prefix / infix | a line — or a *pair of points*, which needs no line drawn between them |
| `angle` | infix | two lines |
| `radius` | prefix | a circle or an arc |
| `coincident`, `midpoint`, `parallel`, `perpendicular`, `symmetry` | infix | one each |
| `project` | infix | two points, each `in` a plane: two images of one point in space (§2.10) |
| `ground`, `fix` | prefix | pin both of a point's coordinates, or one scalar |
| `ccw(a, b, c)`, `cw(a, b, c)` | a call | the one exception: the predicate is about the *triangle*, and three symmetric points do not want reordering |

The collapses are the point: `on` is five constraints, `distance` is six and `tangent` seven,
told apart by the **kinds of their operands** — and `horizontal`/`vertical` are two each, told
apart by where the word stands.

**Operand order carries meaning.** `arc tangent line` is a tangency at the arc's end;
`line tangent circle` is the ordinary one. Which side the arc is written on picks the constraint.

A slot the constraint owns — a curve contact's parameter — is normally omitted; seed one with a
trailing `p on s hint(t: 0.4)`, and **pin** it with `p on(t == 0.4) s`, which is a stated number
and so belongs in the parentheses beside every other stated number.

**`angle` is directed**: the full-turn angle from `l1`'s direction (p1→p2) to `l2`'s, positive
counter-clockwise — the statement pins which side, not just the tilt, so a bearing needs no
orientation predicate beside it. Swapping the lines or reversing one's endpoints negates or
flips the reading, so mind which way each line was declared.

**Tangency has a trap worth knowing.** If the contact point is already held on the circle, state
the tangency *at* that point — `line tangent(at: p1) circle`, `arc tangent(at: start) line` —
rather than pairing `p on circle` with a bare `line tangent circle`. The bare pair is
rank-deficient at every solution and reports freedoms the figure does not have.

### 1.6 Numbers, names and expressions

A dimension may be an expression: `+ - * / ^`, parentheses, `pi`, and the usual functions.
**Trigonometry is in degrees.** A dimension may also be *named* and read elsewhere:

```
// a fragment, not a program — two statements out of a larger document
a distance(w = 60) b            // states 60 and names it `w`
c distance(w / 2) d             // reads it
```

A name that nothing defines is a **free variable**: not an error, but one unknown of the sketch,
tying together every dimension that reads it. The tie must be affine in one free name (`a`,
`a / 2`, `2 * a + 5`); `a * a`, `sin(a)` and two free names in one dimension are errors.

**Every number has a dimension**, and the language checks it. There are two — a **length** and an
**angle**. `*` and `/` derive them, `+` and `-` demand agreement, and what an expression comes to
is checked against the slot it stands in. So `a distance(45deg` is an error, and so is a) b
length added to an angle.

**A bare number is dimensionless, and a *context* may take one.** `a distance(80` is a) b
length because the slot says so, and `sin(30)` reads 30 as degrees because the function does. What
a context may not do is speak for a second operand: `90 / N + ivp` is a plain number added to an
angle, and the language asks rather than answers — `90deg / N + ivp` is the answer.

A number may say what it is:

```
unit mm                          // what this document's numbers are in

a distance(80mm) b               // and what this one is in
c distance(1' 6 3/16") d         // one literal: a space tells the readings apart, as in `3 1/2`
l angle(45deg) m
param ivp = tan(phi) * 1rad - phi   // `inv φ = tan φ − φ` holds only in radians, and now says so
```

**Without a `unit` line the document is in drawing units** — a length with no name. Everything
still checks; you simply cannot write `mm` or `"`, because there is nothing to convert to. A name
is worth a number, and where it is *used* decides what it is: `w = 80` in a length slot does not
make `w` a length, but `w = 80mm` does, and so does a component formal declared `Length`.

`pi` is the mathematical constant and is dimensionless; `tau` and `turn` are a full **turn**, and
are angles. `sin`/`cos`/`tan` take an angle; `asin`/`acos`/`atan`/`atan2` give one;
`floor`/`ceil`/`round` take a plain number, because rounding a dimensioned quantity depends on
which unit you round in.

**The language has no string literal.** `"` is the inch mark, and there is nothing else for one
to be — a `Str` argument is written as the word it is (`at: start`).

### 1.7 Chains

A chain writes a run of elements and the relations *between* them on one line.

```
CHAIN  ::= LINK (JOINT LINK)* ['->' INFIX* 'close']
LINK   ::= PREFIX* DECL | REF
PREFIX ::= a constraint taking one entity            horizontal, vertical
JOINT  ::= '->' INFIX* ['->'] | INFIX+ ['->']        at least one marker or word
INFIX  ::= 'tangent' | 'equal' | a constraint taking two entities
```

**Threading is stated at the joint, never inferred.** `->` says the two links beside it share a
boundary point, threaded left to right (`p1 → p2` on a line; `start → end` on an arc, CCW); its
absence says they do not. `->` alone is the plain corner; `-> tangent` is a corner that is also
tangent there, the regular at-the-point form. The shared point may be named by one side (or
both, agreeing) — and between two declarations by nobody, the chain minting it as an implicit
point: `line l1 -> line l2` is two lines and three points, one shared. A joint may state
several relations — `A -> equal angle(30deg) B` is `equal` and `angle` both, at the corner —
and the marker may stand on either side of the words or both. A word without the marker
states only the relation:
`a_br equal a_tr` relates two arcs declared elsewhere and welds nothing, and
`line l1(a, b) perpendicular line l2(c, d)` declares two separate lines at a right angle. A
chain may mix declarations and names; at a corner with an element declared elsewhere, the
declared side names the shared point, usually by that element's own child
(`line t(p3, k.start) -> tangent k`). A loop closes with `-> close` — a loop is a thread.
Links may be anonymous like any other declaration, so
`line -> tangent arc -> tangent line` is a fully anonymous open contour: two lines and an arc,
welded and tangent at both corners.

`equal` is polymorphic: a length between lines, a radius between circles or arcs. `a equal b equal
c` is two statements, not three.

**`ring` is unrolled.** `ring N about REF { … }` makes the N copies a `cycle` would, congruent by
the numbers each was given and not held so; the implementation says it did (W112) wherever it
reports the degrees of freedom, since the spec's `ring` is constraint-class and this one counts
every copy. The `about` clause is mandatory. A statement inside a ring may reference, outside it,
only what the ring's turn leaves where it is — the axis point, and a circle or arc centred on it
(E021 otherwise); a ring inside a ring is refused (E022).

Inside a `repeat`, `cycle` or `ring` body the final chain may end **mid-joint** — the marker,
or the marker with words, standing at the body's `}`: the trailing joint threads the chain onto
the *next copy's* first link, weld and at-forms exactly as an in-chain joint. `cycle` and `ring`
wrap, so every copy states it and the loop closes with no `close`; a `repeat`'s last copy simply
leaves it unstated, so `repeat N { line -> angle(a) }` is an open polyline of N sides and N−1
corners. Both boundary links must be the body's own declarations, and at most one of the two
boundary slots may name its point — both named are two different points across the copy seam,
and that coincidence is written longhand. A statement inside a braced body also ends at the
body's `}`, so the whole of a block fits one line: `cycle 4 { line s -> perpendicular equal }`
is a square but for a size and a pose.

### 1.8 Curves

A curve is **a point of a component, as one of the component's numeric formals runs over an
interval**. There is no separate "curve family": a component is written once, and asking for one
of its points over one of its formals is what makes a curve of it. An involute, a cycloid and a
walking leg's stride are three components, not three additions to the model.

```
curve NAME = INSTANCE.POINT over FORMAL in (A, B)           a drawn instance's point
curve NAME = Component(ARGS).POINT over FORMAL in (A, B)     an instance written in place
```

**Over a drawn instance.** The leg is drawn once — `leg: Leg(axle, pivot)` — and
`curve path = leg.toe over theta in (0, 360)` is where its toe goes as `theta` runs. The
drawing's own pose is where the trace is anchored, so the component needs no seeds for the curve's
sake; and an instance that leaves a numeric formal *unbound* makes it an unknown of the drawing
(`leg.theta`, reported as a free variable), so the crank is the drawing's freedom and the curve
follows wherever it stands. `jansen.sv` and `peaucellier.sv` are both written this way.

**Over an instance written in place.** `Involute(base, phase: a0).p` binds an instance that is
never drawn — the curve is the only thing made of it. Its anchor is the value it gives the swept
formal (`Cell(…, u: 90)`), or the interval's start when it gives none.

A component's point is placed one of two ways. **Computed**: `port p = (XEXPR, YEXPR)` gives
the coordinates as expressions over the formals and params — the formula an involute has (§2.8).
A component with a computed point is drawn only as a curve. **Placed by constraints**: any point
the body declares, held where the body's statements put it as the formal runs — the locus form,
which is how a person actually states a curve: an involute is "the end of a taut string as it
unwinds", and that sentence is the body (§2.9). Traced, the body must be square — as many
equations as it has coordinates of its own — or elaboration refuses it; drawn, it may be closed
from outside like any component. The swept formal may be an `Angle` or a `Length`; the point
must be one the component places, not geometry it is written over.

Inside a traced component's expressions a formal written `f: frame` offers one name no entity
stores: `f.angle`, the frame's bearing in degrees. A bare `bearing (u)` is measured from the
page's x-axis, so a body posed against a datum with page-fixed seeds goes quietly stale when the
datum tilts; written `bearing (u + f.angle)` — or `cos(u + f.angle)` in a coordinate seed — the
seed follows the drawing. A frame is also usually the *shortest* way to write the formals: it
carries an origin, a second point and a bearing, so a component written over one need not also be
passed those points. Prefer that — a component's entity formals are columns in every gradient the
curve evaluates, and the fewer it takes the cheaper every evaluation is.

A locus generally has several solutions, and a body picks one by three means, strongest first: a
**signed** constraint where the vocabulary has one (`point_line_distance` is signed, so a sign
chooses a winding); an **orientation predicate** (`ccw` / `cw`), which contributes no equation and
selects a component, read at the **anchor** — the drawn pose, or the value the instance gave the
swept formal, chosen where the predicate is unambiguous — and carried from there along the whole
curve by continuity; and a **seed**, for what neither can say. A curve over a drawn instance
starts from the pose on the sheet and needs none of the seeds.

### 1.9 Claims

`claim` in front of a relation states it as *expected to add no rank*: an assertion about the
drawing the rest of the document determines, not part of what determines it.

```
claim vertical rail         // "the drawing already says this" — checked, never enforced
```

A claim never acts.  It joins no solve, no count and no diagnostic set — the drawing, its
degrees of freedom and its status are exactly what they would be with the claim deleted, so a
claim can never make a sketch `Over` or `Conflict`, and a false one cannot pull the geometry
toward itself.  Instead the diagnosis judges it, as one of three things: a **theorem** (it
holds, and adds no rank — the document already implies it), **violated** (it does not hold), or
**consuming** (it holds only because of where the solve happened to land; enforcing it would
have removed a freedom, so the claim claims too much).  Use it to state the fact a figure was
drawn to illustrate — the altitudes concur, the traced path is straight — and let the solver
confirm the theorem instead of trusting the drawing to it.

Because a claim adds no equations, it may not own an unknown: claiming a curve contact
(`point_on_curve` and kin, whose slot carries the contact's own parameter) is an error, and so
is binding a free variable in a claim's dimension.

### 1.10 How it looks

**A document says what the drawing *is*; how it looks is a separate statement.** A declaration
carries a **class**, and a `style` block says what that class looks like. Nothing the solver,
the diagnosis or the decomposition does ever reads one.

```
style .construction { dash: 7 4 }
style .centerline   { dash: 12 3 2 3; width: 0.5; color: #888888 }
style .heavy        { width: 2.5 }

line datum(o, q) class construction
line ab(a, b) class centerline heavy      // a centreline drawn thick
```

Several classes cascade, later over earlier, and only on the properties the later one states. An
unmatched class is not an error — it simply has no rule, exactly as in CSS, which is also what
makes paste work. **Lengths in a sheet are screen pixels**: a dashed line does not change its
dash pattern when you zoom.

A class stands in three more places, and there is one more property:

```
style .dimension { display: none }      // no callouts…
style .shown     { display: inline }    // …but for the ones that ask
style .point     { display: none }      // and no point handles in the picture
style .phantom   { dash: 6 3; color: #888888 }

a distance(80) b class shown            // a relation carries a class: this dimension is drawn
t2: Throw(o, bore, theta: 220deg) class phantom   // every declaration the instance makes is dashed
```

`display: none` leaves a thing out of the picture — an entity is not drawn, a dimension is
neither laid out nor picked — and `display: inline` from a later class shows it again;
`display: geometry` draws a thing and never dimensions it, which is what a phantom position is.
A class on an instance reaches its relations as well as its declarations and stands *over* the
statement's own — the assembly's word about an instance is the stronger one — so
`g: Throw(…) class phantom` with `style .phantom { dash: 6 3; display: geometry }` ghosts a
dimensioned part whole, its `class shown` dimensions included. A class on a relation that states
no dimension is inert. Every point is drawn under the implicit class `.point`. Nothing the solver
does reads any of it: hide every dimension and the drawing is the same drawing.

`construction` used to be a keyword. It is a class now, and `style .construction { dash: 7 4 }`
is a rule in the base sheet the implementation ships — so `class construction` draws exactly as
the word did, and a document that wants reference geometry drawn some other way says so and
changes nothing else.

### 1.11 Modules

```
use engine.dims          // a module: engine/dims.sv beside the document, or the library's
use engine.parts
```

A **module** is a Solvent document read for its components. `use NAME` at the top of a document
brings in every `component` the module defines and every top-level `param` it states — its own
drawing, if it has one, is not drawn, so `gear.sv` is a module as it stands. How the name is
found is the host's business: `solventc` reads `engine.parts` as `engine/parts.sv` beside the
document and falls back to the library compiled into the core; the app has no filesystem and reads
the library alone. A module's own `use`s are followed, once each. A module nothing finds is E070
at the `use`; a component defined twice is E071; a module's own error is shown at the `use` that
brought it in, with the module's line in front of the message.

**A file's top-level `param`s are in scope in the components it defines**, and so are those of
the modules it uses — which is what lets `engine/dims.sv` hold the whole dimension table and every
view read `D` for the bore; a file's own params may read them too (`param rB = rp + 1.5mm`) and
shadow them. A formal of the same name shadows a param.
`rust/examples/engine.sv` is the worked case: a four-cylinder engine in three views, written as a
dimension module, a parts module, a valvetrain module and one module per view, over the standard
library's three views (`use std`, `rust/lib/std.sv`).

### 1.12 Checking your work

Elaborate, solve, then diagnose. The diagnosis reports **degrees of freedom** and one of four
states:

| state | meaning | what to do |
|---|---|---|
| `Well` | dof 0, everything consistent | done |
| `Under` | dof > 0 | something can still move; add a constraint or a gauge |
| `Over` | more claims than unknowns, all consistent | a claim is redundant; often a mistake |
| `Conflict` | claims that cannot all hold | the report names the *minimal* set that disagrees |

Two habits worth keeping. **Ground something**: a figure with no `ground` is under-constrained by
three even when its shape is fully determined, because nothing says where it is or which way it
faces. And **seeds matter for branches**: a solver finds *a* solution near where you started, so
seed an arc's points roughly where you mean them or you may get the mirror image.

---

## 2. Examples

Each was run through the implementation; the reported figures are what it actually says.

### 2.1 One dimensioned line — `dof 0, Well`

```
point a hint(x: 0, y: 0)
point b hint(x: 30, y: 10)

line ab(a, b)
horizontal ab
a distance(40) b

ground a
```

Four unknowns (two points), four claims (level, length, and the two the ground pins). Note that
`b`'s seed is nowhere near the answer — it does not need to be. It says which side of `a` to
put `b` on, and nothing more.

### 2.2 A rectangle, as a chain — `dof 0, Well`

```
param w = 60
param h = 40

point p0 hint(x: 0, y: 0)
point p1 hint(x: w, y: 0)
point p2 hint(x: w, y: h)
point p3 hint(x: 0, y: h)

horizontal line bottom(p0, p1) ->
vertical   line right(p1, p2) ->
horizontal line top(p2, p3) ->
vertical   line left(p3, p0) -> close

p0 distance(w) p1
p1 distance(h) p2
ground p0
```

Eight unknowns; four directions, two lengths and a ground. `param` is arithmetic done while
reading — `w` never becomes an unknown, it is just 60 wherever it appears. A `param` may read
another written anywhere in the same body or an enclosing one — `param h = w / 2` may stand above
`param w = 60`, a body being a set — and one defined in terms of itself, through however many
others, is an error (E041). The chain here states
nothing the four separate `horizontal …` / `vertical …` lines would not; it just reads as the
outline it is.

### 2.3 Naming a dimension — `dof 0, Well`

```
// a fragment: substitute these two lines into §2.2, and drop its two `param` lines
p0 distance(w = 60) p1          // states it, and names it
p1 distance(w / 2) p2           // reads it: the height follows the width
```

Substituted into 2.2 (dropping the `param`), this makes the rectangle parametric: edit the 60 and
the height follows. A number stated once and read everywhere is the difference between a drawing
and a picture of one.

### 2.4 A free variable — `dof 1, Under`, on purpose

```
point a hint(x: 0, y: 0)
point b hint(x: 10, y: 0)
point c hint(x: 0, y: 9)

line ab(a, b)
line ac(a, c)
horizontal ab
vertical ac
a distance(s) b         // `s` is defined nowhere...
a distance(s) c         // ...so the two lengths are tied, and their value is the solver's
ground a
```

`s` names an unknown. The two lengths must agree; nothing says what they are, so one degree of
freedom is left and the diagnosis says `Under` — correctly. Give `s` a value anywhere, or add a
third constraint, and it closes.

### 2.5 An arc, tangent to what it joins — `dof 0, Well`

```
point a hint(x: 0, y: 0)
point b hint(x: 30, y: 0)
point c hint(x: 40, y: 10)
point d hint(x: 40, y: 40)
point o hint(x: 30, y: 10)

horizontal line run(a, b) -> tangent
arc fillet(center: o) hint(r: 10) -> tangent
vertical line rise(c, d)

radius(10) fillet
a distance(30) b
c distance(30) d
ground a
```

Nothing says where the arc's ends are. `fillet` names only its centre; the chain threads `b` in as
its start and `c` as its end, and each `tangent` becomes a tangency stated *at* that shared point
— the regular form, not the rank-deficient pair. This is the shape of most real work: state how
things meet, and let the positions follow.

### 2.6 A component, instanced — `dof 0, Well`

```
component Rung(a: point, b: point, len: Length) {
  line e(a, b)
  horizontal e
  a distance(len) b
}

point l0 hint(x: 0, y: 0)
point r0 hint(x: 50, y: 0)
point l1 hint(x: 0, y: 20)
point r1 hint(x: 50, y: 20)

t0: Rung(l0, r0, len: 50)
t1: Rung(l1, r1, len: 50)

line stile(l0, l1)
vertical stile
l0 distance(20) l1
ground l0
```

Passing an entity into a component is **aliasing**, not constraint: the formal and the actual are
one entity, at no cost and with nothing to violate. That is why a component boundary is free.

### 2.7 Repetition — `dof 5, Under`

```
param n = 6
param r = 40

cycle n as i {
  point p hint(x: r, y: i * 60)
  line e(p, next.p)
  e equal next.e
}
ground p[0]
```

`cycle` makes `n` copies that close, so `next` is the copy after this one and wraps at the end.
Six links, all told they are the same length — which round a ring is one statement more than is
independent, and nothing sizes the ring at all, so five freedoms remain. Under-constrained
repetition is normal and is not a mistake by itself; `p[0]` indexes a particular copy.

The body may end mid-joint (§1.7), which is how a closed contour is written with no names for
its corners at all — `dof 1, Under`:

```
cycle 4 {
  line s -> perpendicular equal
}
s[0].p1 distance(50) s[0].p2
ground s[0].p1
```

Each copy's side is welded onto the next's at a corner also held square and equal to it, the
wrap sealing the loop: four lines over the four points the welds mint. One dimension sizes it
and a grounded corner places it, leaving the one freedom — the square swings about that corner.
Round a closed loop one `perpendicular` and one `equal` are theorems, which the diagnosis notes
as implied and never paints; state a *dimension* at every corner instead (`-> angle(90)`) and
the same closure redundancy is `Over` — "remove one" — since editing one of those dimensions is
the next conflict. The shipped `square.sv` is this figure; `ngon.sv` is the parametric case — a
component taking `n`, its corners riding a circle with `equal` at every welded corner, all pure
relations. Its seeds walk once round the circle on purpose: equal chords of a circle fix each
central angle's size and not its sign, so the collapsed polygon, zigzags and stars satisfy the
same statements — the winding is a branch, and a branch is chosen by seeds where no residual
can state it.

### 2.8 A curve from a computed point — `dof 1, Under`

```
component Involute(c: circle, phase: Angle, u: Angle) {
  port p = ( c.center.x + c.r * (cos(u + phase) + u / 1rad * sin(u + phase)),
             c.center.y + c.r * (sin(u + phase) - u / 1rad * cos(u + phase)) )
}

point o hint(x: 0, y: 0)
circle base(center: o) hint(r: 20)
curve f = Involute(base, phase: 0).p over u in (0, 90)

point t hint(x: 25, y: 8)
t on f
ground o
fix base.r
```

An involute is library code, not an entity kind: a component with one computed point, over the
circle it unwinds from, and the curve is that point as `u` runs. The remaining freedom is the
contact's own — `point_on_curve` carries a parameter saying *how far along* `t` sits, and nothing
here says. That is one equation's worth of unknown, and it is why a contact slides along a curve
instead of breaking when the geometry beneath it moves. The contact's parameter is always written
`u` (`t on f hint(u: 30)`, `t on(u == 30) f`), whatever the swept formal is called.

### 2.9 A curve stated as a locus — `dof 1, Under`

```
component Unwind(c: circle, datum: line, phase: Angle, u: Angle) {
  point t
  point p
  line rad(c.center, t)
  line s(t, p)
  t on c                                                 // the string leaves the circle...
  datum angle(u + phase) rad                             // ...at bearing u from the datum,
  rad perpendicular s                                    // square to the radius there,
  p distance(-(c.r * u / 1rad)) rad                      // and taut: as long as the arc
}

point o hint(x: 0, y: 0)
point x hint(x: 20, y: 0)
circle base(center: o) hint(r: 20)
line datum(o, x)

curve f = Unwind(base, datum, phase: 0).p over u in (0, 90)

point g hint(x: 25, y: 8)
g on f

ground o
fix base.r
horizontal datum
o distance(20) x
```

This draws the same curve as §2.8 and states no formula at all — compare the two bodies. Every
line in the component is the textbook definition said once: a point on the base circle at bearing
`u`, the string leaving square to the radius, and the string exactly as long as the arc it has
unwound. The solver derives what §2.8 had to be derived by hand.

Two details do real work. `point_line_distance` is **signed**, so the negative sign is what
unwinds the string one way for a positive roll and the other for a negative one — which is why one
component serves both flanks of a gear tooth, where a formula needs the sign threaded through
every term. And `angle` is **directed**, so `t` sits at bearing `u + phase` and not opposite it —
which side of the datum is in the residual itself, with no `ccw` needed to say so. Where a body
has a genuinely discrete choice (two intersections of an elbow, say), `ccw(a, b, x)` still states
it: read once at the anchor, and carried everywhere else by continuity.

The remaining freedom is the contact's parameter again, exactly as in §2.8.

### 2.9.1 A curve of a drawn instance — `dof 1, Under`

```
component Crank(o: point, datum: line, theta: Angle) {
  point p hint(x: 20, y: 10)
  line arm(o, p)
  o distance(30) p
  datum angle(theta) arm
}

point o hint(x: 0, y: 0)
point x hint(x: 10, y: 0)
line datum(o, x)
ground o
ground x

c: Crank(o, datum)                                  // theta unbound: the crank turns
curve rim = c.p over theta in (0, 360)
```

The crank is drawn once and traced from that drawing. `c: Crank(o, datum)` leaves `theta`
unbound, so it is an unknown of the sketch — `c.theta`, which the diagnosis reports as a free
variable and which is the one freedom left — and `rim` is where the drawn `p` goes as that
formal runs a full turn. The trace is anchored at the pose on the sheet: drag `c.p` and the
anchor moves with it. `jansen.sv` is this at full size — the leg drawn, its crank free, the toe's
stride traced from the same statements.

### 2.10 Three views — `dof 0, Well`

A drawing of a part is several pictures of it on one sheet, and the sheet stays a sheet: a
`plane` is a frame (an origin, a point it is turned toward, a rotor) that also carries a
constant attitude in space, a point says which view it is drawn `in`, and `a project b` says
two points are two images of one corner — their coordinates along the fold line the two views
share agree, which is one equation.  Nothing three-dimensional is ever solved for.

```
// a 60-wide, 40-tall, 30-deep block, three views, one corner tied across them
point Af hint(x: 0, y: 0) in front
point qf hint(x: 40, y: 0)
plane front(origin: Af, toward: qf)                             // the page itself
point At hint(x: 0, y: 90) in top
point qt hint(x: 40, y: 90)
plane top(origin: At, toward: qt, from: front, fold: 0deg)      // folded up from the x-axis
point Ar hint(x: 150, y: 0) in right
point qr hint(x: 150, y: -40)
plane right(origin: Ar, toward: qr, from: front, fold: -90deg)  // folded from z, turned so z is up
ground Af
ground qf
ground At
ground qt
ground Ar
ground qr

point Bf hint(x: 60, y: 40) in front
Af distance(60, along: x) Bf
Af distance(40, along: y) Bf
point Bt in top
point Br in right
Bf project Bt          // width agrees front ↔ top
Bf project Br          // height agrees front ↔ right
Bt project Br          // depth agrees top ↔ right
At distance(30, along: y) Bt
```

The standard library writes this layout once: `use std` and `views: ThreeViews(O, right: 150,
up: 90)` declares the page as `views.front` and folds `views.right` and `views.top` from it, with
`views.right_origin` and `views.top_origin` the corner `A` as those views see it — so a drawing
grounds one point and writes its geometry `in views.top`.

**A part is designed in one place.** A component's body may carry `in view { … }` blocks over
planes it was handed, one per view the part shows in, and the `project` statements tying them
— so the whole design of a connecting rod is one file, and the view modules draw only the
castings a view is of. `repeat flag { … }` with a 0-or-1 `Int` formal leaves a view undrawn for
an instance that does not show in it. `rust/examples/engine/conrod.sv` is the worked case.
Inside a *root* block (a `repeat` at the top level) the clause is still written per declaration.

`in top { … }` writes the clause once: every declaration in the block — a `cycle`'s copies
included — is drawn in `top`, and the statements are otherwise ordinary (they dimension, drag
and delete as themselves; deleting the plane unwraps the block and leaves them on the page).
A component instance takes it whole — `t: Tooth(…) in top` draws everything the component
makes in the view, a datum or a curve inside excepted.

Each view's origin is the same corner `A` as that view sees it, so no projection between the
origins need be stated.  `fold` is the bearing of the fold line in the parent view — `0deg` from
the page is the top view, `-90deg` the right view — and the new view's second axis points away
from the parent's viewer, so distance from the fold line is depth: third-angle projection.  Any
plane can be reached in two folds, or given outright as `u: (…), v: (…)`.  Points `B` in the top
and right views are placed by projection and the one depth dimension: `solventc` reports
`dof 0, Well`.  `rust/examples/bracket.sv` is the full case — an L-bracket in three views with
an auxiliary view folded at the bearing of its inclined face, whose four corners are placed by
projection alone and come out the true-size rectangle the face is.

Because the document says which plane each view is on, the sheet and the **object** are one
drawing: the app's overview mode folds the views back into the glass box they were unfolded
from and reconstructs the part in the middle of it.  Nothing is solved for and nothing is
stored — a point drawn in a view sits in space at `a·u + b·v` for the coordinates that view
measured, and any corner two non-parallel views both see is placed exactly.  **Every** plane
stands there as a pane with its own x and y running across it and crossing at its origin —
drawn in or not, since a view is a place to draw — and double-clicking one leaves the box and makes it the view the next thing you
draw is drawn in.  Otherwise it is a way of *looking*, like the camera: the drawing is still
the sheet.

---

## 3. Working checklist

1. Declare points first, seeded roughly where you mean them — near enough to pick the right
   branch.
2. Declare the lines, arcs and circles built from them; prefer a chain for a contour.
3. State relations (levels, tangencies, equalities), then dimensions.
4. `ground` one point, and `fix` a scalar if the size is meant to be given rather than solved.
5. Elaborate, solve, diagnose. Aim for `Well` and dof 0 unless the task wants freedom left.
6. If `Conflict`, read the minimal conflict set — it names the statements that disagree, not the
   whole drawing. If `Over`, find the claim already implied by the others. If `Under`, ask what can
   still move.

The documents in `rust/examples/` are the worked corpus, each with a header explaining what it is
for; `rect_fillets.sv` is the best first read, `gear_trace.sv` the deepest, and `engine.sv` with
its `engine/` modules the largest — three views of a whole engine tied by projection.
