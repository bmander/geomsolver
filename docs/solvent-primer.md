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
unit NAME                               what the document's numbers are in (§1.6)
param NAME = EXPR                       a number worked out while elaborating; never an unknown
KIND [NAME][(CHILD | hint(x: E, y: E), …)] [hint(SCALAR: E, …)] [knots […]] [class NAME…]
                                        an entity declaration (§1.4); the name is optional
style .NAME { PROP: VALUE; … }          what a class looks like (§1.10)
WORD[(ARGS)] REF | REF WORD[(ARGS)] REF   a constraint, prefix or infix (§1.5)
    [hint(SLOT: EXPR, …)] [at (t, r)]
claim <constraint>                      an assertion, judged and never solved for (§1.9)
ground REF                              pin both of a point's coordinates
fix REF.field                           pin one scalar, e.g. `fix c.r`
ccw(a, b, c) | cw(a, b, c)              record a root choice; contributes no equation
NAME: Component(ARGS)                   instantiate a component
component NAME(FORMALS) { statement* }  define one
port NAME: KIND | port NAME = REF       export an entity across a component boundary
repeat N [as i] { … }                   N copies, unrelated
cycle N [as i] { … }                    N copies that close: `next` and `prev` are in scope
ring N about REF [as i] { … }           a cycle claimed to be cyclically symmetric
curve NAME(FORMALS)(u) [over (a, b)] = ( XEXPR, YEXPR )        a curve family (§1.8)
curve NAME(FORMALS)(u) [over (a, b)] = trace P [from (E)] where { … }
```

References are `name`, `name.field`, or `name[expr]` (which copy of a repeated statement).

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
| `curve` | `args` (a list) | — | — |

Every scalar is seeded by name in the trailing clause — `point p hint(x: 0, y: 0)`,
`circle c(center: o) hint(r: 25)`, `arc a(center: c, start: s, end: e) hint(r: 5)`. Keys may come
in any order and an omitted one is 0, so `point p hint(y: 12)` and `point t` are both legal. The
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
| `distance` | infix | two points; `along: x` / `along: y` for the run and the rise; a point and a line; two lines; two circles |
| `distance` | prefix | a line — the distance between its own ends |
| `tangent` | infix | a line and a circle (`at: p1`/`p2` for a tangency at that end), two circles (`external: bool`), an arc and a line (`at: start`/`end`), a spline or an ellipse and a line |
| `equal` | infix | two lines (a length), two circles or arcs (a radius) |
| `curvature` | infix | a spline or an ellipse and a circle |
| `horizontal`, `vertical` | prefix / infix | a line — or a *pair of points*, which needs no line drawn between them |
| `angle` | infix | two lines |
| `radius` | prefix | a circle or an arc |
| `coincident`, `midpoint`, `parallel`, `perpendicular`, `symmetry` | infix | one each |
| `ground`, `fix` | prefix | pin both of a point's coordinates, or one scalar |
| `ccw(a, b, c)`, `cw(a, b, c)` | a call | the one exception: the predicate is about the *triangle*, and three symmetric points do not want reordering |

The collapses are the point: `on` is five constraints and `distance` and `tangent` are six each,
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

### 1.8 Curve families

A family is a *kind* of curve, written over the geometry it is drawn from, and instanced with
`curve name = family(args)`. A curve is library code, not an entity kind: an involute, a cycloid
and a spiral are three families, not three additions to the model.

A family's body takes one of two forms.

```
curve NAME(FORMALS)(u) [over (a, b)] = ( XEXPR, YEXPR )              a formula
curve NAME(FORMALS)(u) [over (a, b)] = trace P [from (E)] where { … }   a locus
```

**The formula form** gives the two coordinates directly, as expressions in the parameter and in
the geometry the family is written over (§2.8).

**The locus form** — `trace` — says *the curve is wherever these constraints put this point*, as
the parameter runs. It is how a person actually states a curve: an involute is "the end of a taut
string as it unwinds", and that sentence is the block. The block's statements are ordinary ones,
over ordinary scratch geometry; it must be square (as many equations as it has inner coordinates)
or elaboration refuses it. See §2.9.

Inside a family's expressions — a formula's coordinates, a block's seeds, rows and home — a
formal written `f: frame` offers one name no entity stores: `f.angle`, the frame's bearing in
degrees (the `atan2` of its rotor, derived at compile). A bare `bearing (u)` is measured from the
page's x-axis, so a block posed against a datum (`datum angle(u`) with page-fixed) swing
seeds goes quietly stale when the datum tilts; written `bearing (u + f.angle)` — or
`cos(u + f.angle)` in a coordinate seed — the seed follows the drawing. `peaucellier.sv` states
its one seed this way; the rest of that block needs none, because its predicates already say
which branch each point is on, and a seed repeating a predicate is the weaker of two statements
of one fact.

A frame is also usually the *shortest* way to write the formals: it carries an origin, a second
point and a bearing, so a family written over one need not also be passed those points and the
line between them. Prefer that — a family's formals are columns in every gradient its tapes
evaluate, and the fewer it takes the cheaper every evaluation is.

A locus generally has several solutions, and a block picks one by three means, strongest first: a
**signed** constraint where the vocabulary has one (`point_line_distance` is signed, so a sign
chooses a winding); an **orientation predicate** (`ccw` / `cw`), which contributes no equation and
selects a component, read at the **home** — the parameter value `from (…)` names, chosen
where the
predicate is unambiguous — and carried from there along the whole curve by continuity; and a
**seed**, for what neither can say.

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

`construction` used to be a keyword. It is a class now, and `style .construction { dash: 7 4 }`
is a rule in the base sheet the implementation ships — so `class construction` draws exactly as
the word did, and a document that wants reference geometry drawn some other way says so and
changes nothing else.

### 1.11 Checking your work

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
reading — `w` never becomes an unknown, it is just 60 wherever it appears. The chain here states
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

### 2.8 A curve family — `dof 1, Under`

```
curve involute(c: circle, phase: Angle)(u) over (0, 90) =
  ( c.center.x + c.r * (cos(u + phase) + u / 1rad * sin(u + phase)),
    c.center.y + c.r * (sin(u + phase) - u / 1rad * cos(u + phase)) )

point o hint(x: 0, y: 0)
circle base(center: o) hint(r: 20)
curve f = involute(base, phase: 0)

point t hint(x: 25, y: 8)
t on f
ground o
fix base.r
```

An involute is library code, not an entity kind: two expressions over the circle it unwinds from.
The remaining freedom is the contact's own — `point_on_curve` carries a parameter saying *how far
along* `t` sits, and nothing here says. That is one equation's worth of unknown, and it is why a
contact slides along a curve instead of breaking when the geometry beneath it moves.

### 2.9 A curve stated as a locus — `dof 1, Under`

```
curve involute(c: circle, datum: line, phase: Angle)(u) over (0, 90) =
  trace p from (90 - phase) where {
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

curve f = involute(base, datum, phase: 0)

point g hint(x: 25, y: 8)
g on f

ground o
fix base.r
horizontal datum
o distance(20) x
```

This draws the same curve as §2.8 and states no formula at all — compare the two bodies. Every
line in the block is the textbook definition said once: a point on the base circle at bearing `u`,
the string leaving square to the radius, and the string exactly as long as the arc it has unwound.
The solver derives what §2.8 had to be derived by hand.

Two details do real work. `point_line_distance` is **signed**, so the negative sign is what
unwinds the string one way for a positive roll and the other for a negative one — which is why one
family serves both flanks of a gear tooth, where a formula needs the sign threaded through every
term. And `angle` is **directed**, so `t` sits at bearing `u + phase` and not opposite it — which
side of the datum is in the residual itself, with no `ccw` needed to say so. Where a block has a
genuinely discrete choice (two intersections of an elbow, say), `ccw(a, b, x)` still states it:
read once at the `from (…)` home, and carried everywhere else by continuity.

The remaining freedom is the contact's parameter again, exactly as in §2.8.

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

The nineteen documents in `rust/examples/` are the worked corpus, each with a header explaining
what it is for; `rect_fillets.sv` is the best first read and `gear_trace.sv` the deepest.
