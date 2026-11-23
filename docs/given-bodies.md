# A given body (#993 stage 2): the experiment

A STEP file comes in, a change is asked for, and a checked STEP file goes out. Stage 2 of #993 asks
the first of that: **a document names a file and gets a solid**, with no kernel vocabulary entering
the language, and its gate is that a document that only names a file exports the same solid, every
face on its original surface. The spelling was left open, with a preference — give a meaning to the
module mechanism rather than add a constructor — and the bible's rule that new syntax waits for an
experiment (principle 1). This is that experiment's record: the spellings put to the compiler
before the change, the one taken and why, the rules as built with what `solventc` says to each, the
later stages' documents written in it, and what it leaves.

The language is [0.230] ([changelog](solvent-changelog.md), [spec](../solvent-spec.md) §14.4 and
§6.9); the rules the code holds are in [language.md](contract/language.md) ("A given body") and
[solids.md](contract/solids.md) ("A body a file gives"), the hosts' search in
[architecture.md](contract/architecture.md) and the page's in [app.md](contract/app.md).

## The principle

**A given body is a set, and an edit is a statement over it** (now in the
[Design Bible](design-bible.md), under "Solvent is not beholden to CAD traditions"). A purchased
part, a supplier's casting and a legacy model are givens. Reverse engineering one into a Solvent
document fabricates intent — measured numbers written as dimensions no designer stated, which
principle 3 forbids — and rebuilds every face the edit never touched, so anything the kernel cannot
build yet is refused for an edit that had nothing to do with it. The honest document names the file
and states the change: the edited body is a set defined from the given one, and the given one
stays nameable.

## The candidates, put to the compiler before the change

Each was run through `solventc` at `55c958d`, with `bracket.step` beside the document. None
parsed into anything; each was refused.

| | written | what came back |
|---|---|---|
| (C) | `use bracket`, `part := solid(bracket)` | `2:1: error[E070]: no module `` `bracket` ``, `3:15: error[E101]: no such entity: `` `bracket` `` |
| (C′) | `use bracket`, `part := solid({ p \| p in bracket })` | E070; `3:26: error[E101]: no such entity`; `3:15: error[E040]: `` `coincident` `` needs to know what its operands are`; `3:1: error[E080]: a solid is a region: a set whose body bounds a number (`` `std.sets.Ball` ``, not the sphere it bounds)` |
| (A) | `use bracket`, `b := bracket()`, `part := solid(b.body)` — the module as a component of no arguments | E070; `3:6: error[E103]: no component named `` `bracket` ``; `4:15: error[E101]: no such entity: `` `b` `` |
| (B) | `use bracket`, `part := bracket` | E070; `3:9: error[E103]: `` `part` ``: `` `bracket` `` is not a number here — nothing in scope gives it one` |
| (D) | `part := solid(given("bracket.step"))`, a constructor | `2:15: error[E103]: no component named `` `given` ``; `2:15: error[E101]: no such entity: `` `#i22` `` |

So every candidate gives a meaning to a construct the compiler rejects — the bible's preferred
change — and none competes with a reading a document already relies on. (C′) is not a candidate of
its own: it is (C) read through a set literal, and under (C) it must mean the same set.

## The decision, and why

**(C) was taken**: `use bracket`, then `bracket` names the body the file gives.

- **`solid(S)` stays the one constructor** ([0.199]). The given body is a set named like any
  other, and `solid(bracket)` is a body whose stock it is, as `solid(stock)` is. (C′) is then not a
  second spelling to add but one the language already has: `p in bracket` is a membership of a
  solid, as `p in stock` is.
- **No grammar, no string literal, no path enters the document.** What a name resolves to is the
  host's question, as it has been for every module since [0.12]: the CLI finds `bracket.step`
  beside the document, the page finds it in the project it opened, and the core takes text and
  opens no file. The document stays a set of facts about geometry; where the bytes live is not one
  of them. Determinism holds because the file is part of the input, as a module's text is.
- **(A) is ceremony.** `()` and `.body` carry no design, and the instance they make suggests
  placement it does not deliver: a closed instance's solids are placed by its frame, and a body
  read from a file stands where the file puts it, so the instance would need placing work of its
  own (stage 5's) before `b := bracket() in f` meant anything. Until then it is a call with no
  arguments around a name.
- **(B) makes a solid a value**, which nothing else in the language is: every solid is
  `solid(…)` or a body word's result.
- **(D) is a constructor, and puts a filesystem path in the model.** It is the kernel's view (a
  file is read) where the language's is a set; principle 2 ("a new capability arrives as a library
  definition …, not as new grammar, constructor arguments or options") rules it out.

What (C) costs, stated:

- **A module name now names an entity, in one case.** Until now a module contributed
  definitions, read by path (`engine.parts.Crank(…)`); a given module contributes a body under its
  own path. The precedent is `std`, whose datums (`std.front`, `std.origin`) are entities a `use`
  brings in.
- **A file of several solids has no spelling yet.** It is refused at the `use`, saying how many
  (one given body is one solid). Naming the solids of one file — and an assembly's product
  structure, which the kernel does not read yet — is later work.
- **Placement is refused, not built.** A given body stands where its file puts it. Read where it
  would have to be placed — a definition's body, a `ring`, an `in` of a frame of the document's
  own — it is E106, never silently left at the world. Placing it, or part of it, is stage 5.
- **A document named like its file reads itself.** In one directory `NAME.sv` is found before
  `NAME.step`, so `bracket.sv` beside `bracket.step` links itself for `use bracket` and says
  `` no such entity: `bracket` ``. That is the module search as it always was; it is met more
  often now.

## The rules as built

Each document below was run through `solventc` at the merged commit of stage 2, with the kernel's
fixtures (`rust/gcs-core/tests/step_file/`, written by OCCT 7.9.3) copied beside it. `bracket.step`
is `drilled_ap214.step`: a 40 × 30 × 20 block from the origin, drilled through along z by a hole of
radius 6 about (20, 15). Its volume is `24000 − 720π = 21738.053`, its area `5200 + 168π =
5727.79`.

**1. Recognised by content.** A text whose first token, after an optional byte-order mark and white
space, is `ISO-10303-21;` is an exchange file and its module a given module (`modules::is_step`,
one test every host asks). It is never parsed as Solvent; its extension plays no part beyond where
a host looks (rule 9). A file of a different name, `drilled_inch.step` copied as `inchfile.step`,
read as `use inchfile`, is the same block (`part.volume = 21738.1`).

**2. The name is the module's whole path.** `use bracket` names `bracket`; `use vendor.bracket`
(`vendor/bracket.step`) names `vendor.bracket`:

```
unit mm
use vendor.bracket
part := solid(vendor.bracket)
```

```
$ solventc vendor.sv --where part.volume --where part.bounds
vendor.sv: solved
  0 params, 0 equations, structural rank 0; DOF 0
  part.volume = 21738.1
  part.bounds.x0 = 0
  part.bounds.x1 = 40
  part.bounds.y0 = 0
  part.bounds.y1 = 30
  part.bounds.z0 = 0
  part.bounds.z1 = 20
```

It defines nothing else: `use bracket (x)` is `` error[E101]: `bracket` defines no `x` ``, and a
file of its own `bracket` (`bracket := point in std.front`) is `` error[E071]: `bracket` is defined
in this file, and `use bracket` names the body its file gives: one name, two things ``.

**3. Read wherever a solid's name is read**, at the root of a file that `use`s it:

| written | `solventc` |
|---|---|
| `part := solid(bracket)` | `part.volume = 21738.1`, `part.area = 5727.79` |
| `part := solid({ p \| p in bracket })` | `part.volume = 21738.1`, `part.area = 5727.79`, 7 faces, 15 edges |
| `upper := solid({ p \| p in bracket; p distance(>= 10mm, along: n) std.top })` | `upper.volume = 10869`, its upper half (`10869.026645 mm³` built) |
| `bracket cut pin`, `pin` a prism of radius 3 at (5, 5) from z = −10 to 30 | `pin.volume = 565.487`, its two ends (`180π`) |
| `claim every q in pin: q in part`, `pin` of radius 2 within the material | `holds, measured 3` |
| `claim every q in part: q in bracket` | `undecided, measured 0 … unresolved: containment is within curved-faceting uncertainty` (their boundaries are one) |

`solid(bracket)` builds to the file's B-rep untouched (`3 ms`, `405` entities exported). The set
spelling builds the same solid — the same volume to the last digit, the same 7 faces and 15 edges
— but not the same bytes: it is built again (`52 ms`), one p-curve is written as a
`B_SPLINE_CURVE_WITH_KNOTS` where `solid(bracket)` writes a `CIRCLE` (`1282` entities), and the
report names its faces otherwise (`part.far.area = 4973.82`, where `solid(bracket)` lists
`part.bracket.#given0` … `#given6`). The bible's "two spellings of one set give the same result"
holds here for the set and not yet for its export; the record leaves it as an open item of the
evaluator.

**4. Never extended.** The given body is the file's, and stays nameable as given:

```
drill cut bracket
```

```
error[E080]: `bracket` is the body a file gives, and it is never extended: name the body this
makes of it, `part := solid(bracket)`, then `drill cut part`
```

So are `drill union bracket`, `drill common bracket`, and the primitive the words are written in
said of it, `bracket and { p | p not in drill }` (the same E080). The edit is said of a body made of
it:

```
unit mm
use std
use std.solids (cut)
use bracket

part := solid(bracket)

in std.top {
  o := point
  fix((8, 8)) o
  hole := circle(center: o)
  radius(3) hole
}
drill := solid(std.solids.Through(face(hole)))
drill cut part
```

`part.volume = 21172.6`, `part.area = 6048.23`; `21738.053 − 180π = 21172.567` and `5727.79 −
18π + 120π = 6048.23`. The kernel builds `21172.566612 mm³, 8 faces, 18 edges`, and `--verify-step
full` reads its export back.

**5. What is not read is not there.** `use bracket` alone makes nothing: `--where part` says `the
source names nothing there`, and `--step` says `this document has no solid to write`.
`part := solid(bracket)` makes one object, `part`: the cut list is `1 part, 1 object`
(`1 × part: 40 × 30 × 20 — part`), and `--step-assembly` writes one part (`1 parts, 0
sub-assemblies, 1 objects placed`). The given body is its stock, not a second part.

**6. One body per module.** `a := solid(bracket)` and `b := solid(bracket)` are two objects
(`2 parts, 2 objects`) over one given body, however many files `use` the module.

**7. Where it stands, and units.** The file's origin and axes are the world's: the block's bounds
are `x 0–40, y 0–30, z 0–20` as the file has them, with or without `in std.top { … }` around the
statement. A read where the body would have to be placed is refused at the read:

| read in | `solventc` |
|---|---|
| a component's body, `component Holder() { part := solid(bracket) }` | `error[E106]: `` `bracket` `` is the body a file gives, which stands in the document's own space, and this reads it in a definition's body, which its instance's frame places: placing a given body is not built yet (#993 stage 5), so read it at the document's root` |
| a `ring 4 about std.z { … }` | the same E106, "in a `ring`, which turns each copy", beside the ring's own `` error[E021]: `bracket` is outside the `ring` and does not turn with it … `` |
| `f := frame`, `in f { … }` | the same E106, ``in `in f`, a frame that is not the world's`` |

Lengths are the file's, read in its own units and carried to the document's: a `unit in` document
reading the millimetre file has `part.volume = 1.32654` and `part.bounds.x1 = 1.5748`
(`21738.053 / 25.4³`, `40 / 25.4`); the same block written in inches, and in metres, reads to
`21738.1` in a `unit mm` document; the foot-unit cube to `1000`.

**8. A refused file is said once, at the `use`**, in the kernel's words with the module's name in
front — and whatever a statement reading the name would say besides is left unsaid:

| file | `solventc` |
|---|---|
| `shell_ap214.step`, a cube's shell | `` error[E072]: `shell`: #15: the file holds a shell-based surface model (SHELL_BASED_SURFACE_MODEL) and no solid `` |
| `assembly_ap214.step`, two cubes by XCAF | `` error[E072]: `pair`: #376: an assembly's component (NEXT_ASSEMBLY_USAGE_OCCURRENCE): assemblies are not read yet `` |
| `placed_ap214.step`, one cube XCAF located | `` error[E072]: `given`: #372: an assembly's component (NEXT_ASSEMBLY_USAGE_OCCURRENCE): assemblies are not read yet `` |
| a header with no `FILE_SCHEMA`, read by `part := solid(junk)` and `other := solid(part)` | `` error[E072]: `junk`: the file's header names no schema (FILE_SCHEMA) ``, and nothing else |
| `drilled_ap214.step` with a second `MANIFOLD_SOLID_BREP` over its shell | `` error[E072]: `twice`: the file gives 2 solids, and a given body is one solid `` |

**9. Where hosts look.** `NAME.sv`, then `NAME.step`, then `NAME.stp` in each directory, the
document's own first, then its ancestors, then the library (`modules::search_paths`, one answer for
the CLI, the page and the fixtures crate). A document in `sub/` finds `bracket.stp` in its parent;
a `bracket.sv` put beside the document then wins, and `solid(bracket)` is `no such entity`.

## Measurements

The kernel's fixtures, each copied as `given.step` beside `unit mm`, `use given`, `part :=
solid(given)`, exported with `--step --verify-step full`, and the export read again as the given
body of a second document; "read back" compares the two documents' whole `--where part` reports.

| fixture | written as | built (mm³) | closed form | faces, edges, vertices exported | entities | read back |
|---|---|---|---|---|---|---|
| `drilled_ap203` | AP203, mm | 21738.053289 | `24000 − 720π` | 7, 15, 10 | 405 | identical |
| `drilled_ap214` | AP214, mm | 21738.053289 | | 7, 15, 10 | 405 | identical |
| `drilled_ap242` | AP242, mm | 21738.053289 | | 7, 15, 10 | 405 | identical |
| `drilled_inch` | AP214, inches | 21738.053289 | | 7, 15, 10 | 405 | identical |
| `drilled_metre` | AP203, metres | 21738.053289 | | 7, 15, 10 | 405 | identical |
| `named_ap214` | AP214 through XCAF, named | 21738.053289 | | 7, 15, 10 | 405 | identical |
| `box_ap242_foot` | AP242, feet | 1000.000000 | `10³` | 6, 12, 8 | 327 | identical |
| `void_ap214` | AP214, a sphere's void | 7731.917427 | `8000 − 256π/3` | 7, 13, 10 | 360 | identical |

Every build took under 10 ms and every run under a second on a loaded machine (one solid of a few
faces; these are not timings to quote). The in-process gate,
`given_bodies::a_document_naming_a_file_exports_the_same_solid`, holds the export to the file's
own reading face by face (`step_roundtrip::same`: every face on its original surface in its order,
every edge and vertex where the file had it), and the kernel's writer holds what it writes to
`step_check::verify`.

*To be added by the stage's gate run:* `solventc` over a real file of many faces (benchy), the time
to read, build and export it, and its round trip. Not measured here.

## The later stages' documents, in this spelling

Three of brep-bench's `edit/` cases and one CADGenBench instruction, each written as it would read
over a given body. Where today's words suffice the document was run; where they do not, the line
that cannot be written is marked and the word or selection it lacks named.

**A hole added** (works now). The input is a 60 × 40 plate 8 thick with two holes of radius 4 at
(15, 20) and (45, 20), authored and exported to `input.step` only to have a file:

```
// A third hole, of the same radius, between the two the plate came with.
unit mm
use std
use std.solids (cut)
use input

part := solid(input)

in std.top {
  o := point
  fix((30, 20)) o
  hole := circle(center: o)
  radius(4) hole
}
drill := solid(std.solids.Through(face(hole)))
drill cut part
```

`part.volume = 17993.6`, `part.area = 6701.59`; the kernel's `17993.628421 mm³, 9 faces, 21 edges`
against `19200 − 384π = 17993.628421` and `6400 + 96π = 6701.59`.

**`hole_widened`** (works now, with a number read off the file). The case's plate (40 × 30 × 8, a
hole of radius 5 about (20, 15)) as `plate.step`, its wall offset 2 into the plate, said as the
wider hole cut from the body made of it:

```
unit mm
use std
use std.solids (cut)
use plate

part := solid(plate)

in std.top {
  o := point
  fix((20, 15)) o
  wider := circle(center: o)
  radius(5mm + 2mm) wider
}
bore := solid(std.solids.Through(face(wider)))
bore cut part
```

`part.volume = 8368.5`, `part.area = 3563.98`: `9600 − 392π = 8368.495680`, 7 faces, 15 edges, the
old wall gone whole. It reads right, and it is not the edit asked for: `(20, 15)` and `5` are
measurements of the file written as dimensions, which principle 3 forbids. What is missing is
**stage 3's selection** — the hole's wall chosen by what it is, its axis and radius read from it —
after which `radius(wall.r + 2mm) wider` about `wall`'s axis says it.

**`hole_deleted`, and "remove these two holes"** (needs stages 3 and 4). Today's spelling of a face
deleted is a set over the remaining faces' sides ([sets.md](contract/sets.md), "A face deleted,
said as a set"):

```
unit mm
use input

// not built: stage 3 chooses the faces by what they are; no written path reaches a given body's
// faces today (`--where part` lists them as `part.input.#given0` …)
top := …            // the plane face at z = 8
bottom := …         // the plane face at z = 0
sides := …          // the four plane faces square to it

// stage 4: the body without the holes' walls, every other face running on along its surface
part := solid({ p | p in top.side; p in bottom.side; p in sides.side })
```

Two things are missing: the **selection** (stage 3: `top`, `bottom`, `sides`, or the two walls, by
predicate), and **delete** without listing every remaining face (stage 4), since listing is right
for six faces and unusable for four hundred. Today's words can put the material back by measured
numbers — `plug1 union part`, `plug2 union part`, each plug a prism of radius 4 at the hole's
centre from 0 to 8 — and that gives the right set, `part.volume = 19200`, `part.area = 6400`, but
not the right body: the kernel's `10 faces, 16 edges`, the top and the bottom each three faces on
one plane where the plate without holes has one. A face deleted, said as a set, has six.

A face path a given body does not have is not refused today: `solid({ p | p in input.near.side })`
elaborates with no diagnostic and is refused only when built (`a region over a body's faces that
no entry of the lowering table builds is read by the field …`). That is the general rule for a
face path in a set's body (`ball.nosuch.side` is the same), and stage 3's selection is where it is
answered.

**`block_top_moved_up`** (needs stages 3 and 4). The plate's top moved up by 6. Today's words say
it only by drawing again what the top's neighbours are — the outline 40 × 30 and the hole of radius
5 about (20, 15), every number read off the file — then `slab union grown` with a slab from 8 to
14, and the hole cut again from a body made of that:

`part.volume = 15700.4` (`16800 − 350π = 15700.442571`), and `12 faces, 25 edges` where the
authored case has 7 and 15. That is the reverse engineering the principle refuses. The edit is
**the top's side at a level** — a face's side grown by a number, which the evaluator already reads
(`faces::BodyFaces::level`) — over **the top chosen by what it is**: stage 3's selection and stage
4's move. The five brep-bench workarounds (`block-top-moved-up`, `hole-widened`,
`holes-widened-until-merged`, `l-block-riser-moved`, `rounded-block-top-moved-up`) all wait for the
same two.

## What reads better

- **The document says what is given and what is changed.** `use bracket`, `part :=
  solid(bracket)`, and every further line is the edit. Nothing in it is a measured number unless
  the edit needs one, and where it does, stage 3 is the place to read it.
- **A given body is an operand like any other.** The tool of a body word, a term of a set, a
  claim's set: each reads it with no word of its own, because it is a set.
- **The edit never rebuilds what it did not touch.** `part := solid(bracket)` is the file's B-rep
  handed through, 3 ms and byte for byte what the file held; a hole drilled in it builds one
  Boolean against it.
- **Nothing in the document is the host's.** The same three lines read in the CLI, the page and
  the tests; where the file lives, and that it is a file, is said nowhere in them.

## What reads worse

- **The given body and the body made of it are two names for one solid** where only the edit is
  wanted: `part := solid(bracket)` is a line of ceremony when nothing else is said. It buys the
  rule that a given body is never extended, which keeps the file's body nameable as given — what
  stage 5's containment claim (`every q in region: q in bracket`) needs.
- **A module name is now sometimes a solid.** A reader of `use vendor.bracket` cannot tell from the
  `use` whether it brings components or a body; the name's reads say so.
- **The set spelling is the same solid but not yet the same export** (rule 3), so a document
  written `solid({ p | p in bracket })` loses what makes the given body cheap.

## What is refused, and left

- **Several solids in one file**, and assemblies (the kernel refuses a file's product structure):
  E072 at the `use`.
- **Placing a given body** — in a definition's body, a `ring`, an `in` of the document's own frame
  — is E106 (stage 5).
- **Its faces have no name a document can write.** The report lists them (`part.bracket.#given0`
  …, in the file's order) for a person to read, and nothing reaches them: rounds, deletes and moves
  on a given body wait for stage 3's selection and stage 4's edits.
- **The OCCT route** refuses a given body, naming `--kernel rust` (solids.md); the core's kernel is
  the export.

## The issue's open questions, as the experiment answers them

- **The spelling (stage 2): answered.** `use NAME`, the body named by the module's path, read as a
  solid; no constructor and no path in the document.
- **The spelling of a selection: stage 3's.** Nothing here chooses a face. The report's
  `#given0` names are not that spelling and cannot be written; a selection by index or by stored id
  is a non-goal of the issue and is not offered.
- **The ground truth of a given body.** For a given body the B-rep is the definition: membership is
  read off the boundary the kernel read, as the evaluator reads any built solid's, and the kernel's
  reader is held to OCCT's readings in tests (stage 1). Stage 2 adds nothing to evaluate: the body
  is handed through, never rebuilt.
- **Tolerance.** Stage 2 promises the file's own: the reader measures each edge's and vertex's
  tolerance as the file has them, the body keeps them, and the export states them (`brep.md`,
  "Reading STEP"; the gate's `same` compares to the file's stated uncertainty). What an edit near a
  tolerant edge promises is stage 4's question, since stage 2 makes no edit.
- **Uniqueness of a delete**: stage 4's, untouched here.
