# Parts: a component held to being a function of what it is given (#461)

A design for review on #461. It decides the issue's candidates on what a boundary means for the
language rather than on what it costs the solver. The rules a boundary must keep come from
[solve-boundaries-plan.md](solve-boundaries-plan.md), and a rule's tag there (FR1, DT2, IO2, …) is
used here by name. The style goals come from [the direction paper](solvent-direction.md). Nothing
here is built yet: a line marked `[new]` does not parse today.

## Status

Implemented on `claude/nice-euler-6w9el9`, each step with its tests and its spec, primer,
contract and changelog rows. §11's review questions were taken at the defaults this document
recommends: the word is `part`, a definition reads its own file's values as well as its modules',
PT2 is structural (a part singular at its pose is D3's warning), and a `when` over view switches
is left to the lint.

| step | what landed | spec | gate |
|---|---|---|---|
| 0 | #157: a stated `ccw`/`cw` is checked and steered by the default solve | 0.161 | `tests/orientation.rs` |
| 0 | #492: each row judged at its own size, so a far point no longer loosens every row and "solved" means solved | — | `tests/far_seed.rs` |
| 1 | `in pin.plane { … }` (PV2) | 0.159 | `tests/relation_words.rs` |
| 2 | constants of the definition (PC1–PC2) | 0.160 | `tests/closed_scopes.rs`, `tests/modules.rs` |
| 3 | `part`, E108 and E109 (PT1–PT4); W207 by `--lint` (PT5) | 0.162 | `tests/parts.rs` |
| 4 | diagnosis through parts (D1–D3), `solventc` and `--json`'s `"parts"` | — | `tests/parts.rs` |
| 5 | the engine as parts; `style SELECTOR in VIEW` in a drawing | — | `tests/examples_sv.rs`, `tests/drawing.rs` |

What the work found:

* **The classifier's "leaves freedoms" was mostly a bug.** `boundary::split` dropped an arc's own
  rows (its ends at its radius) when the arc's centre was an operand, so every arc about a pin or
  an axis read as two free numbers. `Throw`, `ConRod`, `EngineBlock` and `CylinderHead` were
  reported so for that reason alone. With the arc's rows counted as the instance's that made it,
  **every one of the engine's 18 definitions is a part** as it stands (225 instances), and #452's
  "the large assemblies are not boundary-shaped yet" is narrower than it read.
* **The engine as parts** (§8): every `component` in `engine/` is a `part`. The rod is
  `ConRod(crank.pin1, end.bore)`, drawing in the planes its pin is drawn in, making its own
  side-view small end and the plane its section A-A is cut in. The crankshaft takes `theta` and
  exports `pin1`…`pin4`. The phantom rod, the three `project` statements reaching into rods,
  `secA`, the `draw_*` flags and every `dims:` argument are gone, and `--lint` says nothing of it.
  Every position the old and new documents share agrees to 3e-13 (3,142 numbers); the side
  pistons, now on the rods' small ends, to 9e-14; section A-A is the same shape about its new
  origin. The shape the library advertises grows (740 points, 538 lines, 47 circles, 29 arcs from
  633, 467, 43, 11) because rods 2 to 4 draw their own end views and every rod its section; it is
  still DOF 0, well constrained.
* **Cost.** Checking 225 part instances adds about a quarter to the engine's elaboration, the
  structure compiled once for the document and read per part.
* **The engine as solids.** Each part makes its own solid from the views it draws, so the
  assembly shows a solid engine with nothing added at the root: the block and head are their end
  sections run along the crank axis (the block less its bores, the crankcase bays between its
  bulkheads and the main-bearing bore, the head less its camshaft tunnels), the sump the end
  view's pan `common` with the side view's floor, the crankshaft a union of the end section's
  journal, pins, webs and turned ends over the stretches the side view gives them, each rod its
  end outline `common` with its edge-on outline, and each piston its side view's half turned
  about the bore axis. Each part's file opens alone from a `preview` placing it as `engine.sv`
  does; `tests/engine.rs` holds the solids to the table and each preview to the assembly's.

PT5's advice is built: `solventc --lint` names a part whose rows, from scattered starts, come to
another pose its stated choices admit (**W207**, `parts::roots`). Five of the engine's parts drew
it, and each now states its root (#503) with `ccw`/`cw`, so the engine lints clean
(`tests/parts.rs`):
* **`Span`**, a belt run or a cam's flank: both circles on the run's far side from the side it
  is asked for. That refuses the other outer tangent, both crossed ones, and the run of no length
  where a lobe's overlapping base and nose circles meet. Read on the run itself, which never
  passes through a centre. `Lobe` and the timing `Drive` drew W207 through their spans alone.
* **`Throw`**, a crank web: ten points each with two roots. Each is read against points already
  placed, or against a rim corner whose side of the arm its signed distance fixes. The throw only
  turns about its axis, so every triangle keeps its area through the turn (302 mm² the least).
  Its rim corners' seeds stood on the wrong sides of the arm, which the choices read as broken and
  carried other points across; they now stand where the corners solve.
* **`Valve`**, the follower, bucket, stem and head: each part's side of the valve axis, or of
  the follower face. The lift slides the valve along its axis and turns none of them.
`ConRod` has more unknowns than the search takes (`boundary::SEARCHED`), so it is not asked.
Every position agrees with the drawing before the change to 7e-13 at 81 cycle angles over the
whole 720°. Each of those solves, and every stated choice holds at each one.

**The hub motor as parts** (#502): every definition in `hub_motor/motor/` is a `part` reading
`motor.dims` as constants, and the `motor_dims` group and every `dims:` argument are gone. The
`frame` group of the world's planes became `motor.parts.Datums`, the points and lines the motor
is drawn about (the axle's centre and datum in the end view, its mid-plane and centreline in the
section), and each part draws in the planes they are drawn in; the axle takes the plan's origin
for its cable port, and the rotor its angle as a formal (PC3). The two W207s, `Rotor` and
`Stator`, are stated as bounds: each point the walls, shoes, hall sensors and magnets place on a
circle from an offset line stands ahead of the axle along its centreline (`f.o distance(>= 0,
along: sc) a`), not at the line's crossing behind it. W207 now reads a part's bounds as root
choices, as its remedy says (`boundary::analyse`); it read only its orientations. All 6,607
geometric numbers agree with the old document to 1.4e-14, the solids' volumes exactly, and the
four sheets render the same coordinates; the advertised shape is unchanged.

**The V-twin as parts** (#501): all 21 definitions in `vtwin/components/` are parts. The
assembly has 95 instances of 18 of them, and `--lint` says nothing of any file.
* **The table and the views.** Each part reads `components.dims` as constants named in its file's
  `use`, and draws in the plane its operands are drawn in. `vtwin_dims`, `layout` and every
  `dims:` argument are gone.
* **The flag.** `dim` chose which bank's claims a preview calls out in the editor. No sheet draws
  them, so both banks now state them.
* **The side view and the banks.** `SideView` is given the points whose heights it shows, the
  pin, a pivot and each cylinder's `Span`, and ties them inside itself; the ten `project`
  statements reaching into it are gone. `Bank` is placed by where its rod rides the pin
  (`z: zB`), its cylinder's face wall following.
* **The throttle.** Its section plane stands at the barrel's centre rather than the frame's
  origin. That is the same plane in space, so the claimed vertical datum tying them is gone.
* **Making them parts found two faults in the classifier, not the document.**
  * A point's hidden point and a plane's axes were nobody's numbers. A part that read an
    operand in space (the disc's set screw, measured along a frame turned inside the disc's
    frame) took the parent's frame rows as its own, and a part whose point another part's
    frame passed through (the crank pin, under each bank's piston frame) took that frame's.
    Each was E109 over rows it never wrote.
  * The root search counted an axis held along a drawn line as a second root when turned back.

  Every root is now stated, by 22 bounds and three orientations in the parts, beside
  `std.Turned`'s own turn (#497). 41 other roots are found and refused.
* **What moved.** Read by the same core, every number the old and new assemblies share agrees
  within 1e-12 (2,826 places, areas and volumes at `t = 0`), and the part sheets' previews
  exactly. The throttle's section, its other 23, is the same shape 72 along its plane, about its
  new origin. The advertised shape loses the datum's point and line (327, 320, 27, 4). Every
  sheet renders byte for byte as before but the plate's, whose hidden edges are split
  differently where they coincide (314 pixels of 127,438 inked, in dash phase).

#492 meets its acceptance but for one bench row: at r₂ = 20.001 the flat solve reports "ok" 2e-3
of the extent off the true crossing, every row holding to 2.1e-7 of its own size while the
crossing moves 2.4e9 times a contact's miss — no residual measure can tell that pose from the
crossing (`docs/iteration-limit-rescue-plan.md` records it, and the rank, which keeps the global
extent).

Verified on the merged branch: the workspace suite (core 2,681 passed, 0 failed) and the web suite
(318 passed).

## Summary

#459 asked whether solving through boundaries pays, and the answer is settled (plan §18–§20). The
flat solve already decomposes what it is given. The sparse parent made solving through boundaries
linear but no faster than flat, and a perfect cache of evaluations would save at most a fifth on
the documents measured. **Performance is not why a large assembly needs boundaries.**

What a large assembly does need from them is that **it can be read and trusted a part at a time**.
A part should be a function of the operands it is mated to, and a parent should relate only what
its parts export. A fault should be reported against the part and the statement that caused it.
The corpus's largest assembly (`engine.sv`, 1,308 unknowns) has none of these properties today,
and the language gives no way to say that it should.

This design proposes:

1. **`part`, a component declared to be a function of what it is given, checked by the
   elaborator and solved flat** (§3). It is declared rather than inferred. The checks are the
   classifier's, promoted from advice to rules for a definition that asks for them. The solver is
   unchanged: no number moves.
2. **A definition reads the constants of its file** (§4). A module's values become part of a
   definition, as a number written in its body already is, so a dimension table is no longer
   threaded through every call.
3. **A part builds everything it is, and what is shown is the drawing's** (§5). The `draw_*`
   flags go, and so do the view formals, which a part reads from the operands it is drawn
   against.
4. **Diagnosis in terms of parts** (§6). An over-constraint through a part's outputs is the
   parent's statement, and a part that fails is named with its inputs.

#461's other candidates are decided in §7. Most are declined: `given`, binding by statement,
`context` (for now), "a noun that is one thing", and a component that implies its interface. §8
shows the engine's connecting rod before and after. §9 orders the work, and §10 says what this
does not decide.

---

## 1. The evidence, briefly

* **Speed** (plan §18–§20). A solve through boundaries is within a small factor of flat, and
  faster at 1,000 drives. A belt run's evaluation costs about 1.5 µs, so neither a cache (#456)
  nor invalidation (#457) has much to save. Neither favours boundaries over the flat solve's own
  block order. Solving through stays a Rust API (`solve::through`) for a part whose evaluation is
  expensive (a trace, an envelope), measured first.
* **Shape** (#452's corpus report). Of 158 definitions read through an instance:
  * 19 leave freedoms they do not export;
  * 20 fuse a fact about their operands into a noun (FR1);
  * 78 are locally determined with the branch left to the seeds;
  * 8 are candidates on all three counts.

  `EngineBlock`, `Crankshaft` and `ConRod` are among the 19. The document still solves at DOF 0
  because the parent fixes what they leave free, which is IO2's "a boundary never yields" broken
  and unreported.
* **Machinery.** Every check this design makes an error is already computed:
  * `boundary::classify` splits an instance into its own unknowns and its inputs, and reads FR1,
    FR2, DT1 and the root search;
  * `cgraph` gives the structural rank `solventc` prints;
  * the diagnosis already names conflicts.

  What is missing is a way for the source to ask for them.

## 2. What the engine shows

`rust/examples/engine.sv` is written to the current rule that "a part is one component carrying
`in view { … }` blocks" (language.md). Its connecting rod is called four times like this:

```solvent
rod1 := engine.conrod.ConRod(std.side, std.front, std.top, crank.pins.t1, end.bore, crank.pin_s[0],
                             side.small[0], secA, draw_end: 1, draw_side: 1, draw_sec: 1,
                             dims: engine.dims.engine_dims)
```

Of its thirteen arguments, only two say what the rod is mated to: the crank pin, in each of two
views. The other eleven are of four kinds:

| arguments | what they are | where they belong |
|---|---|---|
| `std.side, std.front, std.top` | the world's planes, so the rod draws itself where the world is | read from the operands the rod is drawn against (§5) |
| `end.bore, side.small[0]` | the bore axis, and a point the *side view* places for the rod to draw around | `end.bore` is a mate. The rod's side-view small end is its own output, and the side view's piston reads it (IO4: values flow one way) |
| `secA, draw_*` | where section A-A stands on the paper, and which pictures this instance makes | the drawing (`.svd`) |
| `dims: engine.dims.engine_dims` | a fixed table, threaded through every call | a constant of the definition (§4) |

Around the call are statements that reach into instances:

```solvent
ghost := engine.parts.Rod(crank.pins.t2, end.bore, dims: engine.dims.engine_dims) in std.side
ghost.small project side.small[1]
ghost.small project side.small[2]
rod1.sm project side.small[3]
```

Rods 2 and 3 draw no end view (`draw_end: 0`), so a phantom rod is built to give their side view
the small end's height, and rod 4 borrows rod 1's. Each of those statements is legitimate one at
a time. Together they mean that no rod is a function of its pin: its side view is placed by
geometry the parent assembles from other instances. The classifier reports `ConRod` as leaving 2
freedoms and stating 1 fact about its operands, `Crankshaft` as leaving 4 and `EngineBlock` 2,
while the document solves at DOF 0.

The same pattern runs through the corpus:
* 39 calls pass a module's whole table as `dims:` (`engine_dims`, `vtwin_dims`, `motor_dims`);
* 26 hand it on as `dims: dims`;
* 1,236 reads go through it (`dims.L`);
* the hub motor threads a `frame` group of the world's planes and origins into every part.

## 3. `part`: declared, checked, solved flat

```solvent
part CommonTangent(a: circle, b: circle) {                      // [new]: `part`
  s := point hint(at: a.center, toward: b.center, by: 0.1, turn: 90deg)
  e := point hint(at: b.center, toward: a.center, by: 0.1, turn: -90deg)
  run := line(s, e)
  run leaves a
  run reaches b
  ccw(a.center, b.center, s)
  ccw(a.center, b.center, e)
}
```

`part` is `component` with a promise. Everything a component may write, a part may write, and
the elaborator holds a part to the rules below. Each rule is mechanical, and each is a rule the
plan already states (its tag in brackets).

**PT1 — Closed** (CL1–CL3). A part sees only its formals, its own declarations and its file's
constants (§4). This is today's closed scope. A formal `Int` that a `when` reads is a structural
number, decided while elaborating (CL3), not an input the parent may leave unknown.

**PT2 — Its own unknowns are its inputs' to determine** (DT1, FR2). A part's *own unknowns* are
the numbers of what it makes: every entity sited under the instance, and every number no entity
owns that its rows read (a contact's place). Its *inputs* are every other column its rows read,
including an unbound numeric formal. The rows of its body must reach every own unknown:
* the rows of its nested instances count;
* a `claim`, a claim block's witness and an `initially` row do not.

The rule is checked structurally, while elaborating, as a maximum matching over the part's rows
and own unknowns with the inputs held (`cgraph`'s structural rank, restricted). A part whose rows
leave an own unknown unmatched is an error at the definition. The error names the unknowns, as
the classifier's "leaves 2 free, moving `rod1.cbr`, …" does, and gives the remedy: a formal the
parent binds or leaves unknown (FR2), or a relation inside the part. *Held by:* `Crank(o, theta:
hint(30deg))` as a part is accepted, its angle an input. The same body with its angle a
declaration of its own is refused, naming it.

**PT3 — A part states no fact about its operands alone** (FR1). A row whose every column is an
input, or a row beyond what the part's own unknowns absorb, is a verb fused to a noun. It is an
error naming the row. The remedy is to state the fact where the operands are given, or as a
relation word. *Held by:* `a distance(10mm) b` inside a part over operands `a` and `b` is refused
at that row. The same row in the parent is accepted.

**PT4 — A part never yields** (IO2, IO3). Under PT2 and PT3, a parent's statement over a part's
output is a row over the parent's unknowns. It reaches them only through the part's own rows, and
it cannot move the output off what the part's inputs give. Nothing new is enforced to make this
true. What changes is what the diagnosis says when it fails (§6, D1). A choice the parent wants
to make among a part's roots is an input (a `Side` formal), not a `ccw` stated from outside over
the part's outputs.

**PT5 — Its branch is stated, and the stated choice is held** (DT2). Where a part has more than
one root, the root it means is stated: by `ccw`/`cw`, a bound, or a `Side` input. Seeds remain
inert (P3). A stated choice is checked on the solution, and a solve that breaks one fails.
**This needs #157**, since today the default solve neither enforces nor checks `ccw` on the
sheet. That a part's stated choices resolve *every* ambiguity cannot be checked while
elaborating. `solventc --components` keeps the root search as advice, and `--lint` may name a
part whose probes found a second root its choices admit (§9).

**PT6 — Equivariance is not a rule of `part`.** The plan's EQ2 asks more than closure: a part
must not read the axes of the plane it is drawn in, so that two instances at different places in
one plane can share a cache key. That is a rule for a cache (#456, parked), not for correctness.
Under PT1, a part that reads `horizontal` in a plane reads the plane it was *given*. That is
honest: the part is a function of that plane, and moving every input (the plane included) moves
the result with it. The classifier keeps reporting world reads, and they cost a part nothing.

### Why declared, not inferred

Each rule above can be checked without a mark in the source, so a parent *could* infer its
boundaries (plan §15). This design declares them anyway, for three reasons:

* **The reader.** "This is a part" is the most useful sentence a large assembly can say about a
  definition. Inferred, it is a fact about the current text that no one wrote, and a seed or a row
  added later silently takes it away.
* **The error.** A promise kept by inference is never broken, only withdrawn. Declared, the
  promise fails at the line that broke it, with the remedy (PT2's message), at the definition, in
  the author's terms.
* **A gradual path.** The corpus cannot meet these rules today: of the 158 definitions read
  through an instance, about 150 are not candidates on all three counts. `part` lets the corpus
  move one definition at a time, under `tests/examples_sv.rs`. Once the corpus is parts, whether
  `component` itself takes the promise is a one-line decision (§10).

### What it costs the solve

Nothing. A part is flattened as a component is and solved by the one flat solve, so every number
in `tests/examples_sv.rs` stands. The checks run while elaborating, over the flat rows the
instance already made (`boundary.rs`'s `split`), and are linear in the instances. A declared part
is also exactly what `boundary::instance` names, so a later experiment can solve one through its
boundary with no syntax, where its evaluation is expensive enough to matter.

### Where it reaches

* **`repeat`, `ring`, recursion, `when`.** These are decided while elaborating. Each copy and
  each level is an instance under the definition, checked as one. A ring's turned copies own no
  column, so they are not own unknowns.
* **Frames and joints.** A frame a part declares is its own. `f turns(r) l` inside a part is a row
  over it, and an unbound ratio is an exported freedom (FR2). A part placed by a frame takes it as
  a formal; left out, that formal is the world (today's rule).
* **Solids, faces, sets, curves.** A solid or a face is evaluated after the solve and owns no
  column, so it changes nothing here. A set is a predicate, never a part, and `part … := { p | … }`
  is refused. A part instantiated only inside a `trace` is checked as the trace's body already is
  (`locus` squareness).
* **`symmetric`.** It stays an assertion about argument order. A part may carry it.

## 4. Constants of the definition

Today a component may not read a module's value: `tab.R` inside one is E103, "outside this
component; pass it as an argument". That is why `dims` is threaded. The rule conflates two things
a component might read:

* an **input**: something the call may vary, which the solve may treat as an unknown;
* a **constant**: a number fixed while elaborating, the same for every instance, which is part of
  what the definition *is*. `wS := 22mm` written in `ConRod`'s body is one already.

Closure exists to keep the first kind visible: a definition is a function of its inputs. A value a
module or the file itself states, which reads no unknown and no geometry, is the second kind.
Reading it changes nothing about what the definition is a function of.

**PC1 — A definition reads its file's constants.** Inside a component, a part or a relation word,
two kinds of name are readable:
* a value its file `use`s, by the full path (`engine.dims.L`) or bare where the `use` lists it
  (`use engine.dims (L, R)`);
* a value its own file states at top level.

The per-file rule stands: a definition reads what its own file imports, never what the calling
document does.

**PC2 — Only constants.** A value that is or reads a declared unknown (`param c: Length`), or that
reads geometry, is refused inside a definition, as E103 today, naming it. A document's `preview`
values are not its constants: a preview is omitted on `use`.

**PC3 — An input is what a parent should be able to vary.** This is a rule of thumb for authors,
which the primer states. A number a parent would drive or solve for is a formal: the crank angle,
a ratio, a length the assembly finds. A number the part's design fixes is a constant: a fillet
radius, a bolt's centres, the engine table's rod length, for a rod that is this engine's. The
engine's `theta` is now in `engine.dims`. Under PC3 it becomes `Crankshaft`'s formal, bound or
left unknown by the assembly, so the engine can be turned.

What it removes: every `dims: …` argument and `dims: group` formal whose table is a module's
constants (39 calls, 26 hand-ons, 1,236 reads through the group). The `use` list a file writes
(`use engine.dims (L, rp, rpin, pinlen)`) is then the declared list of what its definitions read.
That is N7's "a noun declares what it reads", with no new construct, and a later per-output
invalidation (#457) reads the same list.

*Held by:* a part reading `engine.dims.L` elaborates to the same rows as one given `L: 150mm`. A
part reading a module's `param c` is refused naming `c`. A definition reading a value its file
does not `use` is E101, as an unimported name is today.

## 5. A part builds everything it is

**PV1 — No presentation formals.** A part makes every view it defines, every time. Which of them
a sheet shows, and in what ink, is the drawing's (`.svd`), by the model/drawing split that already
refuses `class` and `view` in a model. `draw_end: Int` read only by `when` is W205 today. In a
part, a `when` over an `Int` formal that only switches whole views off is the pattern this
replaces. The lint keeps naming it, and the migration (§8) removes the engine's.

What this asks of the drawing, which is a drawing change and not a language one:
* a `sketch` whose target is several scopes, or a style scoped to one sketch, so the end section
  can show rod 1 and ghost rod 2 without showing rods 3 and 4 a second time;
* section A-A as the rod's own plane, the plane through the shank's middle square to its axis,
  which the rod makes. Where it stands on the sheet is a `sketch … from m.rod1.cut at (…)`, and
  `secA` with its `right_of` leaves the model.

**PV2 — A part reads its views from its operands.** `o.plane` is the frame a point is drawn in
(#463), readable in a relation word. A part should be able to draw `in` it:

```solvent
part ConRod(pin: CrankPin, bore: line) {
  in pin.end.plane { … }                                         // [new]: `in o.plane`
  in pin.side.plane { … }
}
```

Today `in o.plane` is E040 ("`o` is a point, and `in` names a plane"). Allowing it is the same
resolution `relations` already does for `o.plane`, done for an `in` header. It removes the
world's planes from every multiview part's formals, and a part's views follow what it is mated
to. A point standing in space has no plane, and `in o.plane` over one is E040, as in a word.

**PV3 — Values flow one way between views** (IO4). Today the side view's `SideSection` makes the
pistons' small ends and the rods draw around them. In a part, the rod makes its side-view small end
(`sm_s`, tied to its end-view `sm` by `project` inside the part), and the piston is placed on
`rod1.sm_s`. The phantom `engine.parts.Rod` and its `project` statements go: rod 2's end view is
rod 2's own, styled as a phantom in the drawing.

## 6. Diagnosis in terms of parts

These are #455's diagnostic half, read from the flat solve. They need no solve through
boundaries.

**D1 — An over-constraint through a part's outputs is the parent's.** When the diagnosis finds a
conflict set that holds rows of a part and a parent's row over that part's outputs, it reports the
parent's row, naming the instance: "`rod1.sm project side.small[3]` over-constrains `rod1`: its
outputs are determined by its inputs (`crank.pin1`, `end.bore`)". The part's rows are cited as
the reason, never offered as the ones to delete. This holds only under PT2/PT3, which is why it
waits for `part`.

**D2 — A part that fails is named, with its inputs.** When a solve ends unsettled or fails
(#449's `outcome`), and the largest scaled residuals lie in one part's rows, the report names the
innermost such instance and the values of its inputs at the pose where the solve stopped. This is
the "parent at a domain's edge says which boundary held it" of the boundaries contract, for the
flat solve.

**D3 — Per part, DOF and status.** `solventc` already reports components. It reports each part
instance's own DOF (0 by PT2, structurally), and its numeric rank at the solved pose (DT1, which
the structure cannot promise). A part that is structurally square and numerically singular at its
inputs is a warning naming it.

## 7. #461's candidates, decided

| candidate | decision | why |
|---|---|---|
| **Boundaries made explicit** | **Declared**: `part` (§3), checked, solved flat | the reader and the error (§3, "Why declared") |
| **Declared inputs** (`given r: Length`) | **Declined** | a formal list already separates entities (by position) from numbers (by label, E004), and an unbound numeric formal already is an exported freedom (FR2). `given` would be a second spelling |
| **Binding by value** (`small.r := 20mm`) | **Declined** | a call's label already binds by value while elaborating. N5's aim, never completing a part from outside, is PT2–PT4. The long argument lists N5 answered are mostly world planes, flags and tables, which §4 and §5 remove. W201 still names the puns |
| **Lateral context** (`context fit`, `with … { }`) | **Deferred** | §4's constants cover every table in the corpus, since each is the same for every instance. `context` is for a standard that varies across one assembly (a press fit here, a running fit there), and no document has one yet. When one does, it is proposed against PC1, which it extends |
| **A noun that is one thing** (`upper` for `upper.run`) | **Deferred** | readability only. Interfaces and groups of aliases (`pin1 := {end: t1.pin, side: pin_s[0]}`) already let a part export a narrow face |
| **Namespace** (a component implies its interface) | **Declined** | conformance to an interface is already structural (`flatten::interfaces`' `conform`), so a part's instance passes as any interface its members satisfy. A narrow interface (`CrankPin`) says more than "all of `Throw`'s outputs". E071 stays |

Also decided: **`in o.plane`** (PV2) is added, and **constants of the definition** (PC1–PC2)
relax the closed-scope rule for values only.

## 8. Worked example: the connecting rod

Before (`engine/conrod.sv` and `engine.sv` today, abbreviated):

```solvent
use engine.dims
component ConRod(end: plane, side: plane, secv: plane,
                 pin: point, ax: line, pin_s: point, sm_s: point, at: point,
                 draw_end: Int, draw_side: Int, draw_sec: Int, dims: group) {
  wB := dims.pinlen - 4mm
  when draw_end == 1 {
    in end {
      sm := point hint((pin.x, pin.y + dims.L))
      sm coincident ax
      pin distance(dims.L) sm
      …
    }
  }
  when draw_side == 1 { in side { … sm_s … } }
  when draw_end == 1 and draw_side == 1 { sm project sm_s }
  when draw_sec == 1 { in secv { … at … } }
}

rod1 := engine.conrod.ConRod(std.side, std.front, std.top, crank.pins.t1, end.bore, crank.pin_s[0],
                             side.small[0], secA, draw_end: 1, draw_side: 1, draw_sec: 1,
                             dims: engine.dims.engine_dims)
rod2 := engine.conrod.ConRod(std.side, std.front, std.top, crank.pins.t2, end.bore, crank.pin_s[1],
                             side.small[1], secA, draw_end: 0, draw_side: 1, draw_sec: 0,
                             dims: engine.dims.engine_dims)
…
ghost := engine.parts.Rod(crank.pins.t2, end.bore, dims: engine.dims.engine_dims) in std.side
ghost.small project side.small[1]
ghost.small project side.small[2]
rod1.sm project side.small[3]
```

After:

```solvent
use engine.dims (L, pinlen)                                      // [new]: read in the part
use engine.crankshaft (CrankPin)                       // interface { end: point, side: point }

// The connecting rod on its crank pin, its small end riding the bore axis.
part ConRod(pin: CrankPin, bore: line) {                         // [new]: `part`
  wB := pinlen - 4mm
  in pin.end.plane {                                             // [new]: `in o.plane`
    sm := point hint(at: pin.end, toward: bore.p2, by: 1)
    sm coincident bore
    pin.end distance(L) sm
    …
  }
  in pin.side.plane {
    sm_s := point hint(at: pin.side, toward: bore.p2, by: 1)
    pin.side vertical sm_s
    sm project sm_s                                              // the rod's two views agree
    …
  }
  cut := plane …                                                 // section A-A: the rod's own
  in cut { … }
}

rod1 := engine.conrod.ConRod(crank.pin1, end.bore)
rod2 := engine.conrod.ConRod(crank.pin2, end.bore)
rod3 := engine.conrod.ConRod(crank.pin3, end.bore)
rod4 := engine.conrod.ConRod(crank.pin4, end.bore)
// the pistons ride the rods' small ends; nothing reaches back into a rod
```

What changed:

* **Two arguments, both mates.** The rod is "the rod on this pin, in this bore", and
  `rod1 := ConRod(crank.pin1, end.bore)` reads aloud as that.
* **Nothing places a rod from outside.** Each rod builds its own end view. So rod 2's side view is
  placed by rod 2, the phantom rod and three `project` statements leave the assembly, and D1 would
  name any statement that tried to place a rod from outside.
* **The drawing chooses.** The end section shows rod 1 and ghosts rod 2, the side section shows
  four rods, and section A-A is placed on paper, all in `engine.svd`.
* **The table is read, not passed.** The rod's `use` list says which of the engine's numbers it
  depends on.
* **`Crankshaft` exports `pin1`…`pin4`** as groups of aliases over its throws and side-view pins,
  each conforming to `CrankPin`, and takes `theta` as a formal (PC3).

To become a part, `ConRod` must also lose the 2 freedoms and the 1 fused fact the classifier
finds. PT2 and PT3 name them at the lines that leave them. That is the migration's real work, and
it is exactly the correctness this design is for.

## 9. The work, in order

Each step is its own issue under #465, with its own tests in `rust/gcs-core/tests/` (listed in
`tests/main.rs`) and its own spec, primer, contract and changelog rows.

0. **Prerequisites, in parallel.**
   * #157: a stated `ccw`/`cw` enforced and checked by the default solve (PT5).
   * #492: "solved" means settled at any extent. Without it, D2 reports on poses that are not
     solutions.
1. **`in o.plane`** (PV2). Small, independent, and useful to relation words and components alike.
   Gate: a component drawing in its point operand's plane, and E040 for a point in space.
2. **Constants of the definition** (§4, PC1–PC2). Gate: `tests/closed_scopes.rs` widened:
   * a module's constant is read;
   * a declared unknown is refused;
   * per-file `use` is held;
   * a preview's values are not constants.

   The corpus moves off `dims:` by hand, under `tests/examples_sv.rs`.
3. **`part`** (§3, PT1–PT5).
   * The parser takes `part` where `component` stands, contextually: `part` remains a usable
     name, and nine documents use it as one.
   * The elaborator runs PT2 and PT3 over each instance's split.
   * New error codes in the E1xx block, worded by the core.

   Gate: `tests/parts.rs`, one part per rule, accepted and refused, the remedy in each message.
   The `Crank` and `CommonTangent` of the plan's worked cases are parts. No corpus change is
   required by this step.
4. **Diagnosis through parts** (§6, D1–D3), from the flat solve. Gate: the belt drive with a
   parent statement that over-constrains a run, reported as D1 says, in `solventc`'s output.
5. **The engine as parts** (§8), the acceptance of this design:
   * with the drawing features of PV1 (a sketch of several scopes, or a sketch-scoped style);
   * `tests/examples_sv.rs` holding every advertised answer;
   * a reading of the result on #461 against §2's table.

   The V-twin (#501, done) and the hub motor follow, each with its own issue.

## 10. What this does not decide

* **One body in space, views derived.** The engine's parts are drawings in three planes tied by
  projection, the draughtsman's way (§6.7). A part modelled once in space, with every view a
  projection the drawing makes, is a larger change to how a part is written. §5 makes it reachable,
  since a part no longer names the world's planes, but it is not proposed here.
* **Whether `component` takes the promise.** Once the corpus's nouns are parts, `component` could
  mean `part`, and what is left (view components, procedures) be said another way. That is decided
  on the corpus then, not now.
* **Domains.** A part's domain (`r₁ ≠ r₂`, centres farther apart than the radii's difference) is
  still a comment. D2 names a part at its domain's edge without one. A stated domain is #455's,
  if D2 proves not to be enough.
* **Solving through parts.** The solve stays flat. `solve::through` stays a Rust API, kept for a
  part whose evaluation is expensive, measured first (#493's recommendation).
* **Caching and equivariance** (#456, #457, EQ1–EQ6). They are parked, and PT6 says why
  correctness does not need them.

## 11. For review

1. **The word.** `part` reads well for a pulley, a rod or a drive, and less well for
   `CommonTangent`. The alternatives considered:
   * `noun`: the paper's term, opaque to a reader;
   * a modifier on `component`;
   * no word, with the promise on `component` from the start, which the corpus cannot meet.
2. **PC1's reach.** Should a definition read its own file's top-level values, or only modules'?
   The first is simpler and lets a single-file document drop its `dims` group. The second keeps a
   document's root values its own: `examples::with_params` changes them, and a definition reading
   them would change with them. That is still a constant while elaborating, but it is surprising.
3. **PT2 structurally or numerically.** Structural rank is deterministic and elaboration-time, but
   it accepts a part that is singular at every input (two coincident constructions). Numeric DT1
   at the solved pose is D3's warning. Is a warning enough there, or should a part that is
   numerically deficient at its own solved pose fail?
4. **PV1 and `when`.** A `when` over an `Int` that chooses a *model* (a gear's relief, `n == 0`
   ending a recursion) is legitimate. Only one that switches whole views off is presentation.
   Should a part refuse an `Int` formal read only by `when`s that each wrap only `in` blocks, or
   leave it to the lint?
