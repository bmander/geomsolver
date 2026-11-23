# A given body (#993 stage 2): the experiment

A STEP file comes in, a change is asked for, and a checked STEP file goes out. Stage 2 of #993 asks
the first of that: **a document names a file and gets a solid**, with no kernel vocabulary entering
the language, and its gate is that a document that only names a file exports the same solid, every
face on its original surface. The spelling was left open, with a preference — give a meaning to the
module mechanism rather than add a constructor — and the bible's rule that new syntax waits for an
experiment (principle 1). This is that experiment's record: the spellings put to the compiler
before the change, the one taken and why, the rules as built with what `solventc` says to each, the
later stages' documents written in it, what review found and how each finding was settled, and
what it leaves.

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
- **(B) declares a second name for one body.** `part := bracket` would make `part` the given body
  itself, where a declaration makes an entity of its own; the body made of the given one is
  `solid(bracket)`, and a body standing elsewhere is a reading of it at an instant (`bracket
  at(under: std.z) 90deg`, rule 7), both forms the language already has.
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
- **Placement in a frame is refused, not built.** A given body stands where its file puts it, and
  a reading at an instant stands it elsewhere as it does any solid. Read where a frame would have
  to place it — a definition's body, a `ring`, an `in` of a frame of the document's own — it is
  E106, never silently left at the world. Placing it, or part of it, so is stage 5.

## The rules as built

Each document below was run through `solventc` at `375efd5`, stage 2 with review's fixes, and again
at `d08a939`, with the real-file fixes, with the kernel's fixtures
(`rust/gcs-core/tests/step_file/`, written by OCCT 7.9.3) copied beside it. `bracket.step` is
`drilled_ap214.step`: a 40 × 30 × 20 block from the origin, drilled through along z by a hole of
radius 6 about (20, 15). Its volume is `24000 − 720π = 21738.053`, its area `5200 + 168π = 5727.79`.

**1. Recognised by content, or by the name it was found by.** A text whose first token, after an
optional byte-order mark and white space, is `ISO-10303-21;` is an exchange file and its module a
given module (`modules::is_step`, one test every host asks), whatever the file is called: the
drilled block copied as `odd.sv` and read as `use odd` is `part.volume = 21738.1`. A file a host
found as `NAME.step` or `NAME.stp` is an exchange file whatever its text, so an empty one, or one
of other text, is the reader's refusal at the `use` (rule 8), never a Solvent module. Neither is
ever parsed as Solvent.

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
| `part := solid({ p \| p in bracket })` | the same report line for line, and the same STEP file byte for byte |
| `upper := solid({ p \| p in bracket; p distance(>= 10mm, along: n) std.top })` | `upper.volume = 10869`, its upper half (`10869.026645 mm³` built) |
| `bracket cut pin`, `pin` a prism of radius 3 at (5, 5) from z = −10 to 30 | `pin.volume = 565.487`, its two ends (`180π`) |
| `claim every q in pin: q in part`, `pin` of radius 2 within the material | `holds, measured 3` |
| `claim every q in part: q in bracket` and `claim every q in bracket: q in part` | each `holds, measured 0 (interval [0, 0])`: a body and its untouched stock are one set |
| the same claim, `drill cut part` beside it | `undecided, measured 0`, `` between a face of `bracket` and a face of `bracket` ``, `unresolved: containment is within curved-faceting uncertainty` (the bore is shared; open) |

`solid(bracket)` builds to the file's B-rep untouched (`405` entities exported), and so does the
set spelling: one membership of a whole solid, `{ p | p in S }`, is `S`'s body, as `solid(S)` is,
for a given body and for an authored one alike (sets.md; the gate is
`given_bodies::two_spellings_of_a_given_body_are_one_solid`). The bible's "two spellings of one set
give the same result" holds to the byte. (Review found it did not at first; below.)

**4. Never extended.** The given body is the file's, and stays nameable as given:

```
drill cut bracket
```

```
error[E080]: `bracket` is the body a file gives, and it is never extended: name the body this
makes of it, `part := solid(bracket)`, then `drill cut part`
```

So are `drill union bracket`, `drill common bracket`, and the primitive the words are written in
said of it. The fix named is the author's own statement with the body made of the given one in its
place: `bracket and { p | p not in drill }` says `` …, then `part and { p | p not in drill }` ``,
and `drill bore bracket`, over a word `x bore B := B and { p | p not in x }` of the document's own,
says `` …, then `drill bore part` ``. Extended through a definition's formal — `Drilled(bracket)`,
whose body says `drill cut B` — it is the one E080 at that statement, ending `` …,
`part := solid(bracket)`, and give `part` where it is given ``. The edit is said of a body made of
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
sub-assemblies, 1 objects placed`). The given body is its stock, not a second part — and is never
an object however it is read: `upper := solid({ p | p in bracket; … })` alone lists `1 × upper`,
`bracket cut pin` alone `1 × pin`, and `claims.sv` (rule 3's `part`, `pin` and two claims over
`bracket`) writes an assembly of `2 parts`. `--where part` reports the body's own measures and no
row per face of the given body.

**6. One body per module.** `a := solid(bracket)` and `b := solid(bracket)` are two objects
(`2 parts, 2 objects`) over one given body, however many files `use` the module.

**7. Where it stands, and units.** The file's origin and axes are the world's: the block's bounds
are `x 0–40, y 0–30, z 0–20` as the file has them, with or without `in std.top { … }` around the
statement. To stand it elsewhere it is read at an instant, as any solid is: `moved := bracket
at(under: std.z) 90deg` is the block turned a quarter about the world's z (`moved.bounds.x0 = -30`,
`moved.bounds.y1 = 40`, `moved.volume = 21738.1`), the one object. A read where the body would
have to be placed is refused at the read:

| read in | `solventc` |
|---|---|
| a component's body, `component Holder() { part := solid(bracket) }` | `error[E106]: `` `bracket` `` is the body a file gives, which stands in the document's own space, and this reads it in a definition's body, which its instance's frame places: placing a given body is not built yet (#993 stage 5), so read it at the document's root` |
| a `ring 4 about std.z { … }` | the same E106, "in a `ring`, which turns each copy", and nothing else |
| `left := Holder() mirrored(front)`, or two instances of `Holder` | the definition's-body E106, said once |
| `in (arm := frame) { part := solid(bracket) }`, `arm spins(1) o` | the same E106, ``in `in arm`, a frame that is not the world's`` |
| `in f { claim every q in part: q in bracket }` | the same E106 at `bracket` |
| `f := frame`, `in f { … }` | the same E106, ``in `in f`, a frame that is not the world's`` |

Only a read that a frame would place is refused. `repeat 2 { part := solid(bracket) }` makes two
copies, each the file's body where it stands; `part at(under: std.z) 90deg` is the body made of it
turned. A set or a body made of the given body at the root stands in the world wherever it is later
read, as every root solid does: `S := { p | p in bracket }` at the root
and `part := solid(S)` inside `in f { … }` is the block at the world (`part.bounds.x1 = 40`).

Lengths are the file's, read in its own units and carried to the document's: a `unit in` document
reading the millimetre file has `part.volume = 1.32654` and `part.bounds.x1 = 1.5748`
(`21738.053 / 25.4³`, `40 / 25.4`); the same block written in inches, and in metres, reads to
`21738.1` in a `unit mm` document; the foot-unit cube to `1000`.

**8. A refused file is said once, at the `use`**, in the kernel's words with the module's name in
front — and nothing is said of what is made of it, while what a statement says wrong of its own is
still said:

| file | `solventc` |
|---|---|
| `shell_ap214.step`, a cube's shell | `` error[E072]: `shell`: #15: the file holds a shell-based surface model (SHELL_BASED_SURFACE_MODEL) and no solid `` |
| `assembly_ap214.step`, two cubes by XCAF | `` error[E072]: `pair`: #376: an assembly's component (NEXT_ASSEMBLY_USAGE_OCCURRENCE): assemblies are not read yet `` |
| `placed_ap214.step`, one cube XCAF located | `` error[E072]: `given`: #372: an assembly's component (NEXT_ASSEMBLY_USAGE_OCCURRENCE): assemblies are not read yet `` |
| a header with no `FILE_SCHEMA`, read by `part := solid(junk)` and `other := solid(part)` | `` error[E072]: `junk`: the file's header names no schema (FILE_SCHEMA) ``, and nothing else |
| `two_solids_ap214.step`, two blocks in one representation | `` error[E072]: `two`: the file gives 2 solids, and a given body is one solid `` |
| CADGenBench 240's `input.step`, 6.6 MB | `` error[E072]: `input`: #92335: face 965: its outer loop turns against the face's sense (2.5690052629002286e-4 enclosed in its parameters): the face's normal points into the material its edges bound it on `` |
| an empty `empty.step` | `` error[E072]: `empty`: line 1, byte 0: the file does not start `ISO-10303-21;` at the end `` |
| `notstep.step` holding `hello` | `` error[E072]: `notstep`: line 1, byte 0: the file does not start `ISO-10303-21;` at `hello ` `` |
| `shell_ap214.step` read by `part := solid(shell)`, `other := solid({ p \| p in part })` and `x := part.volume + nosuch` | the E072 at the `use`, and `` error[E101]: `x`: no such entity: `nosuch` ``: nothing of `part` or `other` |

**9. Where hosts look.** `NAME.sv`, then `NAME.step`, then `NAME.stp` in each directory, the
document's own first, then its ancestors, then the library (`modules::search_paths`, one answer for
the CLI, the page and the fixtures crate). A document in `sub/` finds `bracket.stp` in its parent;
a `bracket.sv` put beside the document then wins, and `solid(bracket)` is `no such entity`. A
document is never its own module: `foo.sv` saying `use foo` reads the `foo.step` beside it
(`part.volume = 21738.1`), and a `use` only the document itself would answer is `` error[E070]: no
module `selfonly` ``. The first candidate that is a file is the module; one that is there and does
not read is said and resolves nothing, never a farther file standing in for it — for a drawing
whose model `use`s it, `` error[D001]: model `…/m.sv`: module `dims`: …/dims.sv: invalid utf-8
sequence of 1 bytes from index 0 ``.

**The page's gestures.** A given body has no statement of its own, so a gesture over it is refused
in words and the document is left as it was (app.md;
`edits_over_a_given_body_are_made_or_refused_in_words`, not run here: the CLI makes no gestures).
Deleting it says `` `bracket` is the body a file gives, made by its `use` and by no statement of its
own: delete the statements that read it, or the `use` ``; rounding it is refused as never extended,
naming the body to make; an edge picked between faces no document can name is refused as having "no
name the document can write", where it rounded the whole body. The body made of it is rounded, cut
or deleted as any body is. The page never meshes a given body on its own thread, only in a worker: with a 2.7 MB file
an edit stalled the page 36 s and now stalls it 0.4 s, and the worker's build went from 63 s to 8 s
(the mesher had measured the boundary once per face) — the page fix's own measurements, not repeated
here.

**The library's case** is `rust/examples/given/bought_block.sv`, with `drilled_block.step` beside
it: the block bought in, a 5 mm hole drilled through it, and a claim on its volume.

```
$ solventc rust/examples/given/bought_block.sv --where block.volume
rust/examples/given/bought_block.sv: solved
  volume(<= 21400 * 1mm^3) block: 21345.4 mm^3 ± 0 mm^3 <= 21400 mm^3: claimed, kept
  1 params, 1 equations, structural rank 1; DOF 0
  block.volume = 21345.4
```

`24000 − 720π − 6.25π · 20 = 21345.354`. The bound is written `21400 * 1mm^3`: `21400mm^3` is
(21400 mm)³, and `claim volume(<= 21400mm^3) part` is kept against `9.80034e+12 mm^3`.

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
| `void_ap214` | AP214, a sphere's void | 7731.917427 | `8000 − 256π/3` | 7, 13, 10 | 362 | identical |

Every build took under 10 ms and every run under a second on a loaded machine (one solid of a few
faces; these are not timings to quote). The in-process gate,
`given_bodies::a_document_naming_a_file_exports_the_same_solid`, holds the export to the file's
own reading face by face (`step_roundtrip::same`: every face on its original surface in its order,
every edge and vertex where the file had it), and the kernel's writer holds what it writes to
`step_check::verify`.

### Real files

The three-line document — `unit mm`, `use input`, `part := solid(input)` — over the 32
CADGenBench editing inputs and #3DBenchy (brep-bench's `integration/benchy.step.gz`), none of
them committed. Each was elaborated, built, exported (the writer's own check passing), and its
export read again as a given body in a second document and built. **The numbers are the gate run's**
(`solventc` built under `[profile.test]` — opt-level 2, no LTO, debug assertions — on a shared
4-core machine under load 4–9, single wall-clock runs, not medians), corrected below by what the
real-file fixes changed; the rows the fixes changed were run again at `d08a939`.

| file | size | elaborate | build / write (run) | volume mm³ | faces | entities | read back as a given body |
|---|---|---|---|---|---|---|---|
| benchy | 6.04 MB | 2.3 s | 3.2 s / 3.7 s (9.5 s) | 15550.952442 | 1385 | 195823 | 1385 faces, Δ 1.1e-08 (at `8412889`, below) |
| cgb201 | 2.80 MB | 700 ms | 1216 ms / 954 ms (2.83 s) | 2544509.737401 | 435 | 59927 | 435 faces, Δ 5.8e-07 |
| cgb202 | 17.93 MB | 3.4 s | 8167 ms / 5030 ms (16.69 s) | 4382320.553399 | 2157 | 305028 | 2157 faces, Δ 2.9e-07 |
| cgb203 | 3.76 MB | 900 ms | 2554 ms / 2126 ms (5.51 s) | 3150273.054684 | 156 | 57351 | 156 faces, Δ 4.2e-07 |
| cgb204 | 1.39 MB | 300 ms | 569 ms / 299 ms (1.23 s) | 4554.651527 | 186 | 26548 | 186 faces, Δ 7.9e-06 |
| cgb205 | 1.05 MB | 200 ms | 282 ms / 313 ms (0.81 s) | 438357.637483 | 212 | 63541 | 212 faces, Δ 9.2e-08 |
| cgb206 | 4.97 MB | 1.0 s | 1785 ms / 1350 ms (4.15 s) | 848139.273664 | 637 | 108934 | 637 faces, Δ 3.7e-08 |
| cgb207 | 2.17 MB | 700 ms | 1515 ms / 1267 ms (3.44 s) | 111690.284183 | 286 | 50878 | 286 faces, Δ 2.3e-07 |
| cgb208 | 4.73 MB | 800 ms | 1063 ms / 1181 ms (3.17 s) | 2110134.494347 | 586 | 109338 | 586 faces, Δ 1.5e-07 |
| cgb209 | 0.61 MB | 62 ms | 26 ms / 116 ms (0.21 s) | 77896.054143 | 118 | 25252 | 118 faces, Δ 3.5e-09 |
| cgb211 | 9.92 MB | 1.3 s | 3538 ms / 2210 ms (7.30 s) | 3038789.186913 | 560 | 138406 | 560 faces, Δ 5.1e-07 |
| cgb212 | 2.92 MB | 700 ms | 1489 ms / 1168 ms (3.39 s) | 1399836.610073 | 354 | 72174 | 354 faces, Δ 3.4e-07 |
| cgb214 | 2.74 MB | 500 ms | 1421 ms / 815 ms (2.78 s) | 1849949.813237 | 430 | 76504 | 430 faces, Δ 1.6e-07 |
| cgb215 | 1.74 MB | 300 ms | 312 ms / 348 ms (1.00 s) | 54470.955142 | 392 | 38880 | 392 faces, Δ 9.2e-09 |
| cgb217 | 2.18 MB | 700 ms | 952 ms / 1260 ms (2.79 s) | 223473.799092 | 364 | 46633 | 364 faces, Δ 2.8e-07 |
| cgb218 | 0.30 MB | 35 ms | 8 ms / 70 ms (0.13 s) | 3417501.086943 | 93 | 13953 | 93 faces, Δ 1.2e-12 |
| cgb224 | 0.95 MB | 63 ms | 16 ms / 149 ms (0.22 s) | 75591.248780 | 127 | 42605 | 127 faces, Δ 1.1e-07 |
| cgb225 | 1.81 MB | 200 ms | 479 ms / 303 ms (1.03 s) | 439514.777897 | 358 | 53099 | 358 faces, Δ 5.2e-08 |
| cgb229 | 0.97 MB | 200 ms | 621 ms / 397 ms (1.30 s) | 383922.578392 | 120 | 22667 | 120 faces, Δ 2.2e-08 |
| cgb230 | 1.33 MB | 300 ms | 493 ms / 398 ms (1.13 s) | 3467004.256284 | 141 | 31218 | 141 faces, Δ 4.1e-07 |
| cgb231 | 0.40 MB | 45 ms | 9 ms / 55 ms (0.12 s) | 639467.976556 | 98 | 15313 | 98 faces, Δ 6.7e-09 |
| cgb238 | 4.88 MB | 700 ms | 2087 ms / 1149 ms (3.97 s) | 420546.703641 | 404 | 115300 | 404 faces, Δ 7.2e-08 |
| cgb240 | 6.61 MB | E072 at the `use`, 1.1 s | – | – | – | – | – |
| cgb241 | 1.79 MB | 100 ms | 298 ms / 294 ms (0.77 s) | 925689.633307 | 52 | 20255 | 52 faces, Δ 5.4e-07 |
| cgb242 | 6.41 MB | 1.5 s | export 6.4 s | 410953.608062 | 879 | 142302 | 879 faces, Δ 5.7e-08 (re-read 4.4 s) |
| cgb243 | 4.48 MB | 600 ms | 1732 ms / 1520 ms (3.96 s) | 297319.162371 | 588 | 168676 | 588 faces, Δ 7.4e-07 |
| cgb244 | 1.47 MB | 200 ms | 361 ms / 245 ms (0.87 s) | 315912.952279 | 309 | 27348 | 309 faces, Δ 1.1e-08 |
| cgb245 | 1.69 MB | 93 ms | 40 ms / 162 ms (0.32 s) | 121083.288062 | 450 | 60205 | 450 faces, Δ 1.4e-10 |
| cgb246 | 2.83 MB | 600 ms | export 2.0 s | 599863.975682 | 617 | 75014 | 617 faces, Δ 5.1e-08 (re-read 1.4 s) |
| cgb247 | 7.47 MB | 800 ms | 1731 ms / 1187 ms (3.78 s) | 51113373.082765 | 1026 | 156948 | 1026 faces, Δ 7.2e-09 |
| cgb248 | 12.38 MB | 3.0 s | export 12.6 s | 33106181.345197 | 1197 | 240676 | 1197 faces, Δ 4.2e-08 (re-read 6.6 s) |
| cgb249 | 3.34 MB | 700 ms | 1002 ms / 1232 ms (2.87 s) | 1505668.428525 | 283 | 59601 | 283 faces, Δ 3.1e-06 |
| cgb250 | 4.79 MB | 900 ms | export 3.6 s | 633178.722577 | 642 | 91534 | 642 faces, Δ 2.8e-06 (re-read 2.6 s) |

Δ is the re-read body's volume against the first build's, relative. **31 of the 32 inputs build,
export, read back as a given body and build to the same solid** — the corpus tool
`given_bodies::the_corpus_through_a_document` (ignored; `STEP_CORPUS=… STEP_DOCUMENT_FILES=…`)
holds each to the same solid — the volume within 7.9e-6 in every case and within 1e-6 in 28.
cgb240 is refused at its `use`. Before the real-file fixes four of the 32 were refused at the export
and one export did not read back:

| file | before | after (export / re-read, s) |
|---|---|---|
| cgb240 | refused at the export after 2.4 s | E072 at the `use` in 1.1 s |
| cgb242 | exported; its re-read refused | 6.4 / 4.4 |
| cgb246 | refused after 34 s | 2.0 / 1.4 |
| cgb248 | refused after 443 s | 12.6 / 6.6 |
| cgb250 | refused after 166 s | 3.6 / 2.6 |

Run again here at `d08a939` (same profile, a loaded machine) these five gave 1.8 s; 8.3 / 5.9;
2.9 / 2.0; 18.7 / 9.3; 4.8 / 3.4 — the same outcomes, slower. **benchy** builds to 15550.952442
mm³, 1385 faces: 3.1e-6 from brep-bench's reference (15550.999870, by the edges' curves) and 6e-8
from Open CASCADE's reading (15550.9534). At `d08a939` its export was refused, not reading back;
with the reader mended (below, run at `8412889`) it is written in 9.5 s, 195823 entities, and read
again as a given body builds to 15550.952608 mm³ on the same 1385 faces. Its glTF (139 s) and PNG
(106 s) are written (44 s and 39 s in an earlier, quieter run), and its STL is refused (below). The
fixtures' STL, glTF and PNG each took under 0.6 s.

Reading is once per thread: linking the three-line document over cgb202 (17.9 MB) took 3.59 s
(`step_read::read` alone 3.52 s), a second and third link of the same text 18.2 ms and 17.1 ms (the
digest memo), a `relink` after an edit 1.4 µs, and `elaborate` 0.2–0.9 ms. `--where part.volume`
and `part.area` of an untouched given body are read off its B-rep with no mesh: on cgb201 39 s
became 2.3 s. `--where part`, which reports its box, still meshes it (60 s on benchy).

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
// faces today
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

## What review found, and how each was settled

Stage 2 was reviewed once merged; each finding below was fixed in the core or the hosts, and each
rule above is the fixed one, run against `375efd5`.

- **A body kept within a half-space shifted every solid after it** (older than stage 2: `55c958d`
  gives the same). A ball's cap declared after `upper := solid({ p | p in part; p distance(>=
  10mm, along: n) std.top })` measured `cap.volume = 2.84677e+06` where alone it is `654.498`, and
  two such sets over one ball were each E041 *made of itself*. The region is now pushed past every
  solid once all are built: `cap.volume = 654.498` in both documents (solids.md,
  `a_body_kept_within_a_half_space_moves_no_later_solid`).
- **The set spelling was another solid's export.** `solid({ p | p in bracket })` had the volume
  and the face count of `solid(bracket)` but was rebuilt as a region: 1282 STEP entities against
  405, a circle p-curve written as a 880-point `B_SPLINE_CURVE_WITH_KNOTS`, its seven faces
  reported as one `part.far`. One membership of a whole solid is now that solid's body (rule 3),
  for any solid, except where the solid is made of the set's own (the cavity inside
  `std.rounds`' `shell`), which stays a region.
- **A given body was an object when read by a set, a claim or a tool**, listed beside what the
  document made of it. It is never one now (rule 5).
- **A refused file was found by spelling.** What it left unsaid was the statements whose text
  named it; it is now what their references resolve to, and a statement's own faults are said
  (rule 8).
- **`bracket at(under: std.z) 90deg` was E040 and E105**; it is the body placed by the turn
  (rule 7).
- **Two diagnostics for one cause, and a fix in other words.** A read in a `ring` was E106 and the
  ring's E021; a formal extended was E080 and E106; the fix named was always `drill cut part`. Each
  is one diagnostic now, its fix the author's statement (rules 4 and 7).
- **The faces' keys reached readers.** `--where part` listed `part.bracket.#given0.area` …
  `#given6`, and a claim's witness said `between bracket.#given0 and bracket.#given2`, names no
  document can write. The report lists no such row and the witness says `` a face of `bracket` ``.
  The export's warning of thin faces did the same on real files (cgb246: `` the first,
  `input.#given231`, 7.835 µm across, beside `input.#given229` and `input.#given233` ``) and
  advised moving "one cut" a given body has none of; it says `` a face of `input` `` and, of a
  body the file gives untouched, "The file `input` gives it so, and the document states nothing of
  it: it is written as it is given".
- **A given body's build was one in every unit.** Its key now reads the document's unit, so the
  same file read in millimetres and in inches is two builds, never one mesh drawn in both
  (solids.md).
- **Where a set made of it stands.** A set or a body made of the given body at the root stands at
  the world wherever it is read later; only a direct read under a posed `in` is E106 (rule 7,
  language.md).
- **The hosts.** An empty `NAME.step`, or one of other text, was E101 alone or a cascade of E100s;
  it is one E072 at the `use` (rules 1 and 8). A document named like its file read itself for
  `use bracket` and said `` no such entity: `bracket` ``; a document is never its own module now
  (rule 9). A drawing's model whose module file is there and does not read had a farther file
  stand in; the drawing now names the module and why (rule 9). The page hands a module to the core
  with the path it was found at, and its text to each worker once, named by its digest after
  (architecture.md).

Left open, older than stage 2: a face path that names nothing is accepted until the solid is built
(*What stays open*, below).

## What real files found

The gate's run over real files found what the kernel's small fixtures could not; each was fixed
in the kernel or the evaluator, and the measurements above are after the fixes.

- **A face turned against its loops was read, and refused only at the export** (cgb240, face
  #92335: `.F.` on a cylinder 0.012 mm wide, its neighbours tangent across both edges and facing
  exactly the other way). The export said `the built boundary is invalid: … loop 0 turns the wrong
  way`, of a body nothing had built. The reader now holds each loop to its face's sense at the
  edges' measured tolerances (never the 0.02 mm the file states), and refuses the file at the
  `use` (rule 8). Which way the file meant is not guessed.
- **A sliver thinner than its tolerance was read turning one way and, written again, the other**
  (cgb242, face #75264, a spline's strip some 1e-5 mm wide): its export did not read back, `loop 0
  turns the wrong way`. Such a sliver now turns no way.
- **A lune of two edges was read as the body touching itself**, after 34 s to 443 s spent reading
  the body's faces as rounds read them (cgb246, cgb248, cgb250, each with a lune between tangent
  rounds), and the refusal advised rounding by statements naming face keys no document can write.
  A lune's two edges are its two sides; a given body nothing was done to is no longer asked what
  rounds are, and where one does touch itself the message is the file's (`` the faces the file
  `bracket` gives meet there … the body the file gives is no manifold solid ``), with no advice.
- **A crescent whose two ends stand 3 µm apart at a vertex did not mesh** (cgb248's plane face
  #17), and with it went the body's volume. The mesher now lays a use's last corner where the
  triangulation laid it.
- **Every export the core writes is read back by the kernel's reader** after `step_check`, and a
  file the reader refuses is refused, so no document's export is one its next `use` would refuse.
  benchy's was so refused: `` the STEP file does not read back: the kernel's reader refuses
  it: #186157: face 180: edge 354's curve in its parameters leaps 4.385387517242087e-2 … it is no
  one curve there, near [0.439203, 0.000387] in its parameters ``. The writer and the check were
  right and the reader wrong: face #27487's sheet folds back under its corner, nearer its edges
  (4e-4 mm) than the rows the face ends at (4.4e-3), so the reader lays those edges on the rows
  (`topology::rows`); read again from the file the kernel wrote, which states the 1e-2 mm its
  geometry holds, a row 0.28 mm long was judged collapsed to a point — at the bar for how far an
  edge may stand off its face, which grows with the stated uncertainty, not at the tolerance
  points are one at. Judged there, benchy's export reads back the same solid (brep.md).
- **`--where` on a given body meshed it**, slower than building it: its volume and area are now
  read off the B-rep (above).

## What reads better

- **The document says what is given and what is changed.** `use bracket`, `part :=
  solid(bracket)`, and every further line is the edit. Nothing in it is a measured number unless
  the edit needs one, and where it does, stage 3 is the place to read it.
- **A given body is an operand like any other.** The tool of a body word, a term of a set, a
  claim's set: each reads it with no word of its own, because it is a set.
- **The edit never rebuilds what it did not touch.** `part := solid(bracket)` is the file's B-rep
  handed through, face for face what the file held; a hole drilled in it builds one Boolean
  against it.
- **Nothing in the document is the host's.** The same three lines read in the CLI, the page and
  the tests; where the file lives, and that it is a file, is said nowhere in them.

## What reads worse

- **The given body and the body made of it are two names for one solid** where only the edit is
  wanted: `part := solid(bracket)` is a line of ceremony when nothing else is said. It buys the
  rule that a given body is never extended, which keeps the file's body nameable as given — what
  stage 5's containment claim (`every q in region: q in bracket`) needs.
- **A module name is now sometimes a solid.** A reader of `use vendor.bracket` cannot tell from the
  `use` whether it brings components or a body; the name's reads say so.

## What stays open

- **Several solids in one file**, and assemblies, a single part XCAF wrote with a location among
  them (`placed_ap214.step`): E072 at the `use`.
- **Placing a given body** in a frame — a definition's body, a `ring`, a mirrored or posed
  instance, a joint's frame, a claim or a formal under a posed `in` — is E106 (stage 5).
- **Its faces have no name a document can write** (stage 3), and rounds, deletes and moves on it
  wait for stage 4. `repeat a in part` over a body made of one makes no copy, and says nothing.
- **A face path that names nothing** (`input.near.side`, `ball.nosuch.side` alike) is accepted
  until the solid is built. Older than stage 2.
- **A containment beside a cut**: `claim every q in part: q in bracket`, `drill cut part`, is
  undecided within the faceting's uncertainty where the bore is shared.
- **benchy's STL is refused**: `the mesh has 1 triangles facing against their surfaces`, a
  near-flat triangle by a pole of one spline face (#13736) at the export's 0.01 mm bar.
- **A file turned against its own loops is refused, not healed** (cgb240): nothing guesses which
  of the face's sense and its boundary the file meant.
- **The OCCT route** refuses a given body, naming `--kernel rust`; the core's kernel is the export.
- **A bound written `21400mm^3` is (21400 mm)³**, silently: write `21400 * 1mm^3`. A rule of the
  units, older than stage 2.

## The issue's open questions, as the experiment answers them

- **The spelling (stage 2): answered.** `use NAME`, the body named by the module's path, read as a
  solid; no constructor and no path in the document.
- **The spelling of a selection: stage 3's.** Nothing here chooses a face; a face's internal key
  reaches no reader, and a selection by index or by stored id is a non-goal of the issue and is
  not offered.
- **The ground truth of a given body.** For a given body the B-rep is the definition: membership is
  read off the boundary the kernel read, as the evaluator reads any built solid's, and the kernel's
  reader is held to OCCT's readings in tests (stage 1). Stage 2 adds nothing to evaluate: the body
  is handed through, never rebuilt.
- **Tolerance.** Stage 2 promises the file's own: the reader measures each edge's and vertex's
  tolerance as the file has them, the body keeps them, and the export states them (`brep.md`,
  "Reading STEP"; the gate's `same` compares to the file's stated uncertainty). What an edit near a
  tolerant edge promises is stage 4's question, since stage 2 makes no edit.
- **Uniqueness of a delete**: stage 4's, untouched here.
