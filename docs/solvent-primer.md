# Solvent: a primer for agents

Solvent is the language a drawing in this project *is*. You do not place geometry in it; you
declare entities and state what must be true of them, and a solver finds coordinates satisfying
everything at once — so a sketch is a set of claims, not a sequence of drawing commands, and
reordering the statements cannot change what it means. Two marks carry the whole discipline:
`hint at` and `:` introduce **seeds**, which are only where the solver starts looking and which it
is free to move, while `==` states a **constraint**, which it must not; delete every seed in a
document and the set of valid solutions is unchanged. Your job on a sketch task is therefore not
to compute positions — that is the solver's — but to say enough true things that exactly one
drawing satisfies them, and then to check that the diagnosis agrees. This document is what the
implementation actually accepts, verified against it; `solvent-spec.md` is the normative
specification and describes several constructs (`hint` as a statement, `path`, `frame`) that the
parser does **not** yet accept.

---

## 1. Formal definition

### 1.1 Lexical

```
program     ::= statement*
statement   ::= <one of the forms in §1.3>            terminated by a newline or ';'
comment     ::= '//' … end-of-line | '/*' … '*/'
identifier  ::= [A-Za-z_][A-Za-z0-9_]*
number      ::= decimal, optionally in exponent form; `3 1/2` is a mixed fraction (three and a half)
```

Whitespace is insignificant except as a separator. A newline ends a statement, **except** inside
brackets, and except when the line ends with a chain joint (§1.7).

### 1.2 The two marks

| written | class | meaning |
|---|---|---|
| `hint at (x, y)`, `r: 25`, `t = 0.4` | seed | where the solve begins; the solver may move it |
| `== 80`, `t == 0.4` | constraint | a claim the solver must satisfy and may never rewrite |

`hint` marks *the solver revises this*. A callout placement (`… == 80 at (12, -4)`) keeps a bare
`at`: it is inert too, but it records where a person dragged a dimension and no solve touches it.

### 1.3 Statement forms

```
param NAME = EXPR                       a number worked out while elaborating; never an unknown
KIND NAME(ARGS) [hint at …] [construction]      an entity declaration (§1.4)
RELATION(ARGS) [== EXPR] [at (t, r)]    a constraint (§1.5)
claim RELATION(ARGS) [== EXPR]          an assertion, judged and never solved for (§1.9)
ground(REF)                             pin both of a point's coordinates
fix(REF.field)                          pin one scalar, e.g. fix(c.r)
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
| `curve` | `args` (a list) | — | — |

A point's seed is written `hint at (x, y)`; other scalars are seeded by name, `circle c(center: o,
r: 25)`. Children may be given positionally or by label; a label is what lets you omit an earlier
one (`line l(p2: c)` leaves `p1` for a chain to thread). An arc is a centre and two *real* points,
so its ends drag and constrain like any others.

### 1.5 Constraints

Every relation is written `snake_case_name(args…)`, arguments in spec order, with a trailing
`== value` where it takes one. Slots typed `Param` are the constraint's own hidden unknown and are
normally omitted.

```
coincident(p, q)                     midpoint(p, line)
distance(p, q) == d                  horizontal_distance(p, q) == d
vertical_distance(p, q) == d
horizontal(line)                     vertical(line)
horizontal_points(p, q)              vertical_points(p, q)
parallel(l1, l2)                     perpendicular(l1, l2)               equal_length(l1, l2)
angle(l1, l2) == theta               parallel_distance(l1, l2) == d
point_on_line(p, line)               point_line_distance(p, line) == d   symmetric(p, q, line)
point_on_circle(p, circle)           radius(circle) == r                 equal_radius(c1, c2)
annular_distance(c1, c2) == d
tangent_line_circle(line, circle, side: ±1)          tangent_circle_circle(c1, c2, external: bool)
tangent_arc_line(arc, line, at: start|end)           tangent_line_circle_at(line, circle, at: p1|p2)
point_on_spline(p, spline)           spline_tangent_line(spline, line)
spline_curvature(spline, circle)
point_on_ellipse(p, ellipse)         ellipse_tangent_line(ellipse, line)
ellipse_curvature(ellipse, circle)
point_on_curve(p, curve)
```

**`angle` is directed**: the full-turn angle from `l1`'s direction (p1→p2) to `l2`'s, positive
counter-clockwise — the statement pins which side, not just the tilt, so a bearing needs no
orientation predicate beside it. Swapping the lines or reversing one's endpoints negates or
flips the reading, so mind which way each line was declared.

**Tangency has a trap worth knowing.** If the contact point is already held on the circle, state
the tangency *at* that point — `tangent_arc_line`, `tangent_line_circle_at` — rather than
pairing
`point_on_circle` with a bare `tangent_line_circle`. The bare pair is rank-deficient at every
solution and reports freedoms the figure does not have.

### 1.6 Numbers, names and expressions

A dimension may be an expression: `+ - * / ^`, parentheses, `pi`, and the usual functions.
**Trigonometry is in degrees.** A dimension may also be *named* and read elsewhere:

```
// a fragment, not a program — two statements out of a larger document
distance(a, b) == w = 60        // states 60 and names it `w`
distance(c, d) == w / 2         // reads it
```

A name that nothing defines is a **free variable**: not an error, but one unknown of the sketch,
tying together every dimension that reads it. The tie must be affine in one free name (`a`,
`a / 2`, `2 * a + 5`); `a * a`, `sin(a)` and two free names in one dimension are errors.

### 1.7 Chains

A chain writes a run of elements and the relations *between* them on one line.

```
CHAIN  ::= LINK (JOINT LINK)* [JOINT 'close']
LINK   ::= PREFIX* DECL | REF
PREFIX ::= a constraint taking one entity            horizontal, vertical
JOINT  ::= 'to' | 'tangent' | 'equal' | INFIX
INFIX  ::= a constraint taking two entities          perpendicular, equal_length, equal_radius
```

**The operands decide what the chain does.** Links that *declare* draw a contour: each joint is a
corner, and the shared point is threaded — named by exactly one side (or both, agreeing) and
filled into the other. `to` is a plain corner; `tangent` becomes the regular at-the-point form.
Links that only *name* state a relation among existing elements and thread nothing. A chain may
not mix the two, and `to` / `tangent` / `close` are meaningless between names.

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
claim vertical(rail)        // "the drawing already says this" — checked, never enforced
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

### 1.10 Checking your work

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
point a hint at (0, 0)
point b hint at (30, 10)

line ab(a, b)
horizontal(ab)
distance(a, b) == 40

ground(a)
```

Four unknowns (two points), four claims (level, length, and the two the ground pins). Note that
`b`'s seed is nowhere near the answer — it does not need to be. It says which side of `a` to
put `b` on, and nothing more.

### 2.2 A rectangle, as a chain — `dof 0, Well`

```
param w = 60
param h = 40

point p0 hint at (0, 0)
point p1 hint at (w, 0)
point p2 hint at (w, h)
point p3 hint at (0, h)

horizontal line bottom(p0, p1) to
vertical   line right(p1, p2) to
horizontal line top(p2, p3) to
vertical   line left(p3, p0) to close

distance(p0, p1) == w
distance(p1, p2) == h
ground(p0)
```

Eight unknowns; four directions, two lengths and a ground. `param` is arithmetic done while
reading — `w` never becomes an unknown, it is just 60 wherever it appears. The chain here states
nothing the four separate `horizontal(…)` / `vertical(…)` lines would not; it just reads as the
outline it is.

### 2.3 Naming a dimension — `dof 0, Well`

```
// a fragment: substitute these two lines into §2.2, and drop its two `param` lines
distance(p0, p1) == w = 60      // states it, and names it
distance(p1, p2) == w / 2       // reads it: the height follows the width
```

Substituted into 2.2 (dropping the `param`), this makes the rectangle parametric: edit the 60 and
the height follows. A number stated once and read everywhere is the difference between a drawing
and a picture of one.

### 2.4 A free variable — `dof 1, Under`, on purpose

```
point a hint at (0, 0)
point b hint at (10, 0)
point c hint at (0, 9)

line ab(a, b)
line ac(a, c)
horizontal(ab)
vertical(ac)
distance(a, b) == s     // `s` is defined nowhere...
distance(a, c) == s     // ...so the two lengths are tied, and their value is the solver's
ground(a)
```

`s` names an unknown. The two lengths must agree; nothing says what they are, so one degree of
freedom is left and the diagnosis says `Under` — correctly. Give `s` a value anywhere, or add a
third constraint, and it closes.

### 2.5 An arc, tangent to what it joins — `dof 0, Well`

```
point a hint at (0, 0)
point b hint at (30, 0)
point c hint at (40, 10)
point d hint at (40, 40)
point o hint at (30, 10)

horizontal line run(a, b) tangent
arc fillet(center: o, r: 10) tangent
vertical line rise(c, d)

radius(fillet) == 10
distance(a, b) == 30
distance(c, d) == 30
ground(a)
```

Nothing says where the arc's ends are. `fillet` names only its centre; the chain threads `b` in as
its start and `c` as its end, and each `tangent` becomes a tangency stated *at* that shared point
— the regular form, not the rank-deficient pair. This is the shape of most real work: state how
things meet, and let the positions follow.

### 2.6 A component, instanced — `dof 0, Well`

```
component Rung(a: point, b: point, len: Length) {
  line e(a, b)
  horizontal(e)
  distance(a, b) == len
}

point l0 hint at (0, 0)
point r0 hint at (50, 0)
point l1 hint at (0, 20)
point r1 hint at (50, 20)

t0: Rung(l0, r0, len: 50)
t1: Rung(l1, r1, len: 50)

line stile(l0, l1)
vertical(stile)
distance(l0, l1) == 20
ground(l0)
```

Passing an entity into a component is **aliasing**, not constraint: the formal and the actual are
one entity, at no cost and with nothing to violate. That is why a component boundary is free.

### 2.7 Repetition — `dof 5, Under`

```
param n = 6
param r = 40

cycle n as i {
  point p hint at (r, i * 60)
  line e(p, next.p)
  equal_length(e, next.e)
}
ground(p[0])
```

`cycle` makes `n` copies that close, so `next` is the copy after this one and wraps at the end.
Six links, all told they are the same length — which round a ring is one statement more than is
independent, and nothing sizes the ring at all, so five freedoms remain. Under-constrained
repetition is normal and is not a mistake by itself; `p[0]` indexes a particular copy.

### 2.8 A curve family — `dof 1, Under`

```
curve involute(c: circle, phase: Angle)(u) over (0, 90) =
  ( c.center.x + c.r * (cos(u + phase) + u * pi / 180 * sin(u + phase)),
    c.center.y + c.r * (sin(u + phase) - u * pi / 180 * cos(u + phase)) )

point o hint at (0, 0)
circle base(center: o, r: 20)
curve f = involute(base, phase: 0)

point t hint at (25, 8)
point_on_curve(t, f)
ground(o)
fix(base.r)
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
    point_on_circle(t, c)                                  // the string leaves the circle...
    angle(datum, rad) == u + phase                         // ...at bearing u from the datum,
    perpendicular(rad, s)                                  // square to the radius there,
    point_line_distance(p, rad) == -(c.r * u * pi / 180)   // and taut: let out == arc unwound
  }

point o hint at (0, 0)
point x hint at (20, 0)
circle base(center: o, r: 20)
line datum(o, x)

curve f = involute(base, datum, phase: 0)

point g hint at (25, 8)
point_on_curve(g, f)

ground(o)
fix(base.r)
horizontal(datum)
distance(o, x) == 20
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
