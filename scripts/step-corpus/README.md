# The STEP corpus (#993 stage 1)

The files the kernel's STEP reader (`cad_kernel::step_read`) is held to, and the oracle that says
what each holds. The files live outside the repository, in **`$STEP_CORPUS`** (on the machine these
were made on, `/home/user/step-corpus`; every script and the gate test read the variable, and the
scripts fall back to that path). Nothing under it is committed: the CADGenBench inputs are ODC-By
and not ours to ship, and the rest is made from them and from brep-bench. What is committed is the
reference: `rust/gcs-core/tests/step_corpus/`.

| committed file | what |
|---|---|
| `cadgenbench.tsv` | the 32 CADGenBench inputs by id, with their sizes and SHA-256: `fetch.sh` checks every byte against it |
| `brep_bench_cases.tsv` | brep-bench's settled cases (its commit in the first line): closed forms of volume and area, the tolerance the bench grades at, the centroid where the case gives one, and `measured`: which of the two the case says is no closed form but a measure (kernels' values that agree, or the bench's measurer's) |
| `commercial_files.tsv` | the files commercial systems wrote, by path, with their sizes, SHA-256, and the archive and member each is taken from: `fetch.sh` checks every byte against it |
| `reference.tsv` | a row per file of the corpus: its DATA section's SHA-256, the entities its solids hold, OCCT's reading (healing off, and what healing changed), brep-bench's measurer's, and the reference volume and area with the bar a reading must come within (`table.py` says how each is chosen); and the account's `verdict`, `unchecked` and `account` columns |
| `account.tsv` | the account, by hand: each file not read right, its verdict, the faces whose one reference is unchecked, and why, with the witness entity and the evidence (below, "The account"); `table.py` writes it into `reference.tsv` |

## The corpus

```
$STEP_CORPUS/
  cadgenbench/<id>/input.step                 the 32 CADGenBench editing inputs (+ edit_description.txt)
  brep-bench/<kernel>/<category>/<case>.step  what each kernel wrote for a settled case
  brep-bench/<kernel>/runs.tsv                each run: ok, refused (why), no-step, crashed, timeout
  brep-bench/reference/<category>/<case>.step a case's reference shape (integration/benchy)
  variants/<habit>/<source path>              a sample written again by OCCT: nopcurves, ap203, ap242, inches
  commercial/<source>/<file>.stp              files commercial CAD systems wrote (below)
  mutants/<kind>/<source>.<k>.step            corpus files each broken by one edit (below)
  mutants/manifest.tsv                        each mutant's source, kind, entity and edit
  oracle/<path>.json, .faces.tsv, .mass.json  OCCT's reading, per face, and the bench measurer's
  oracle/<path>.mass.faces.tsv                the bench measurer's area of each face (faces.py)
  oracle/<path>.quad.faces.tsv                a third reader's area of some faces (teapot.py), same columns
  reference.tsv                               the table, as table.py last wrote it
  build/                                      the bench's copy, the runners' builds, logs
```

## Making it from nothing

Needs Python 3.11+ with OCP (`pip install cadquery-ocp`, 7.9.3), Rust, git, curl; brep-bench
checked out at `$BREP_BENCH` (default `/home/user/brep-bench`, read and never written). Heavy jobs
take `--jobs 2`/`CARGO_JOBS=2`.

```sh
export STEP_CORPUS=/home/user/step-corpus BREP_BENCH=/home/user/brep-bench
S=scripts/step-corpus
sh $S/fetch.sh                                              # CADGenBench and the commercial files, checked byte for byte
python3 -I $S/brep_bench.py prepare                         # copy the bench into build/
python3 -I $S/brep_bench.py build mass occt cad-kernel geomsolver kentron arris vcad brepkit next.BREP springrep
python3 -I $S/brep_bench.py run occt cad-kernel geomsolver kentron arris vcad brepkit next.BREP springrep --jobs 2
python3 -I $S/brep_bench.py references                      # the reference shapes' files
python3 -I $S/variants.py --jobs 2                          # the habits, over the sample
python3 -I $S/oracle.py --corpus --jobs 2                   # OCCT's reading of every file
python3 -I $S/oracle.py --remap                             # faces named again (after a stepscan change)
python3 -I $S/oracle.py --name                              # OCCT's faces named where order does not
python3 -I $S/table.py measure --jobs 2                     # the bench measurer's
python3 -I $S/faces.py build                                # the measurer, face by face
python3 -I $S/faces.py measure --jobs 2                     # its area of each face
python3 -I $S/teapot.py                                     # the third reader: the teapot's whole faces by quadrature
python3 -I $S/table.py                                      # reference.tsv, committed and in $STEP_CORPUS
python3 -I $S/brep_bench.py manifest                        # brep_bench_cases.tsv (on a new bench commit)
python3 -I $S/brep_bench.py census                          # entity kinds per source
python3 -I $S/commercial.py                                 # each commercial file's writer
python3 -I $S/mutants.py make                               # the mutants (after table.py)
python3 -I $S/mutants.py oracle --jobs 2                    # OCCT's reading of each
python3 -I $S/mutants.py table                              # step_mutants/mutants.tsv, committed
```

Every step is idempotent: a file already made, read or measured (by its DATA section's hash) is
not made again. The CADGenBench bytes are fixed; the kernels' files are their writers' and are
only as reproducible as those (the header's time stamp is left out of the hash). A regenerated
corpus whose files differ from the committed table's is reported **stale** by the gate test: run
`oracle.py --corpus`, `table.py measure` and `table.py`, and commit the table (or point
`STEP_CORPUS_TABLE` at `$STEP_CORPUS/reference.tsv`).

Kernels of brep-bench's `kernels.json` with `"step": true` that build here from crates.io:
`cad-kernel` (=0.15.0), `kentron`, `arris`, `vcad`, `brepkit`, `next.BREP`, `springrep`; `occt`
runs in an environment of its own, `build/venv-ocp8` with cadquery-ocp 8.0.1 (the bench's cases
use `OCP.collections`, which 7.9 lacks; `build occt` makes it), while the oracle reads with the
system's 7.9.3; `geomsolver` is `solventc` at the tag its `build.py` names, built from
this repository's own history (`git archive`, no network). Not built: `solidcraft`, `popkern`,
`geop`, `cadrs` (their sources are on GitHub repositories this machine cannot reach), `aetheris`
(.NET 10 and a GitHub source), `smlib` (no STEP writer), `monstertruck` (no runner).

## Files commercial systems wrote

`commercial/` holds what commercial CAD systems export, the habits of the writers the stage names
("tolerant edges and the other habits of commercial writers"). All are public domain (works of the
US government, 17 U.S.C. 105), fetched by `fetch.sh` from nist.gov:

* `commercial/nist-pmi/` — 33 files of NIST's MBE PMI Validation and Conformance Testing project
  (`NIST-PMI-STEP-Files.zip`): the CTC, FTC and STC test parts, each one solid, exported to
  AP203 (geometry only, and with graphical PMI) and AP242 editions 1–3 (with semantic PMI) by the
  project's CAD systems (CATIA V5, Creo, NX, SolidWorks, Inventor), 7.6 kB to 6.1 MB. NIST removed
  the originating system from their headers; `commercial.py` names each file's writer from habits
  of its DATA section (its docstring says which), and is no more certain than they are.
  `nist_ftc_08_asme1_ap242-e1-tg.stp` holds tessellated faces and no solid (`ref_from` `refuse`).
* `commercial/nist-d2mi/` — the five STEP files of NIST's Design to Manufacturing and Inspection
  project (`NIST-D2MI-Models.zip`): four written by NX 8.5 through ST-Developer (AP203), one by
  CT CoreTechnologie's 3D_Evolution (AP242, the NX part translated). Their headers name both.

Their rows are in `reference.tsv` under `commercial/`, made as the others' (`oracle.py --corpus`,
`table.py measure`, `table.py`). Most hold PMI and some construction surfaces
(`SHELL_BASED_SURFACE_MODEL`) beside the one solid; many are in inches. brep-bench's measurer
reads inches as millimetres (above), so most rows are OCCT's alone, or `occt!` where the measurer
read the inch file. Not reached from here: CAx-IF's public files (www.cax-if.org refused by this
machine's proxy), GrabCAD and TraceParts (refused); the ABC dataset (Onshape, archive.nyu.edu)
comes in 7z chunks of gigabytes with no extractor here; the Fusion 360 Gallery's licence is for
research use and does not say redistribution is allowed.

`python3 -I scripts/step-corpus/commercial.py SURVEY.txt` tallies a `step_survey` run over
`$STEP_CORPUS/commercial/` by writer and refusal.

## Mutants

`mutants.py` breaks small corpus files on purpose, each by one edit of its text that leaves it
well-formed Part 21 — a face's, an oriented edge's, an edge's or a bound's sense flipped, a face
dropped from its shell or listed twice, an edge's vertices swapped, a vertex moved by a thousandth
of the part and by half of it, a radius scaled, a direction negated, a knot or a weight changed, a
pcurve on another surface, the length unit changed, the uncertainty made huge or zero, a reference
to an entity of the wrong type, to none, and a face whose surface is itself — and OCCT reads each
as it reads the corpus. Its docstring says how the sources are chosen (four per writer, at most
40 kB, each OCCT reads valid and unhealed; 39 files, 920 mutants) and what each mutant must be
read to: `refuse` where OCCT reads no solid of positive volume (an open or inverted shell, a
crash, a hang), finds it invalid with its measures moved, or reads it only with its tolerances
raised past a part in fifty; `disputed` where OCCT's healing reads it otherwise or its analyzer
finds a defect that moves nothing; else OCCT's reading at the bar one reader's reading has.

The gate is `step_mutants::no_mutant_is_read_wrong` (rust/gcs-core/tests/step_mutants.rs, behind
`#[ignore]` as the corpus's, run the same way with `-- step_mutants:: --ignored`): every mutant
refused, or read to its row's solid within the bar; none read from a `refuse` row; no panic; none
past 60 s. It prints the progress report and a tally by mutation. `step_mutants.rs` also holds
two fast tests the ordinary suite runs, needing no corpus: mutants of files the kernel writes
(refused, or read to the solid's own measures where the edit leaves or scales it), and a written
file truncated, its bytes replaced and its lines shuffled, read without a panic.

## The gate

`rust/gcs-core/tests/step_corpus.rs`, three tests behind `#[ignore = "a tool: …"]` (the corpus is
outside the repository; the ordinary suite neither needs it nor runs them):

* `step_corpus::nothing_is_read_wrong` — each file refused, or read to as many solids as it
  holds, every ADVANCED_FACE, EDGE_CURVE and VERTEX_POINT read and none twice, the volume and the
  area within the row's bar of its reference, and **each face's area within its own bar of the
  oracle's readings of that face** (below). Must pass at every commit. It prints how many files
  read were compared face by face, how many faces (and how many disputed), and why any file was
  not.
* `step_corpus::nothing_panics` — no read panics; a panic names its file.
* `step_corpus::the_corpus_reads` — every file read to its reference, none refused: the stage's
  finish line, failing until the reader is done. Its table (file, outcome, volume and area
  errors, faces compared of those read, the worst face's error over its bar, seconds, the
  refusal's witness or the face past its bar, then the worst face) is the progress report.

```sh
cd rust
STEP_CORPUS=/home/user/step-corpus cargo test -j 2 -p gcs-core -- step_corpus:: --ignored --nocapture
```

`STEP_CORPUS_ONLY=cadgenbench/` reads only the files whose path starts so, `STEP_CORPUS_THREADS`
sets how many at once. Each file is read once for the three tests (on November 21, 2026, the
reader refusing every solid, the 3,139 files took 7–20 s on four busy cores; on November 22,
reading 3,013 of 3,177 and each face's area measured once, 18–22 minutes three at once on four
cores loaded to 12–15 by other builds). Two quick tests run
in the ordinary suite: the table is whole (`the_table_is_whole`) and the hash is SHA-256.

What counts as right, per row's `ref_from`: `closed` (the case's closed forms, at the bench's
grade), `measured` (the case's values where the case says they are no closed form but a measure —
the mean of kernels' values of their own models, or the measurer's of the reference shape — at
the bench's grade: 264 rows were `closed` before), `mass+occt`, `mass`, `occt`, `occt?`, `occt!` (a
reading's own solid, `table.py` says at what bar), `mass+quad` (the measurer's, its faces a
third reader measured taken at that reader's areas: the Utah teapot, below), `source:…` (a
variant: its source's); `refuse` (the file represents no solid — a shell-based surface model, an
empty product — and must be refused); `disputed` (OCCT and the bench's measurer read it
differently: either reading or a refusal, unless the account settles the dispute). A file with no
row is not gated: OCCT and the measurer both fail on it, or its bar would pass a part in fifty
(`table.py` prints each with why; 33 at the account's commit).

**The totals' bars are the row's or what the file determines, whichever is wider**
(`step_corpus::determined`, `bars`), each measured on the reading itself.

* **The volume.** The flux `Φ(o) = ∮ (X − o)·n dA` of a boundary is linear in the point `o` it is
  taken about: `Φ(o + t) = Φ(o) − t·C`, `C = ∮ n dA` the boundary's *closure vector*, nought for a
  boundary that closes and not for one whose edges stand off their faces. So moving a solid by `t`
  changes its volume by exactly `t·C/3`. `step_survey --moved 10` measures `C` (`closure`: the flux
  about the solid's box's centre and about it moved by the box's diagonal along each axis, three
  more flux integrations) and prints the change beside `t·C/(3V)`: CADGenBench 212 along x
  +1.044e-6 and +1.044e-6, y −9.128e-8 and −9.128e-8, z +7.863e-7 and +7.863e-7; 229, brepkit's
  saddle-drum-bored and helical-pipe-round-ends agree to every digit printed, and cad-kernel's
  boxes-cut has `|C|` 2.6e-14. Nothing in the file says which point the flux is taken about — its
  frame's origin is no property of the solid — and over points anywhere in the solid's box the
  volume takes values `size·|C|/3` apart, `size` the box's diagonal: **the file determines its
  volume to `size·|C|/(3V)`**, with `K = 1`, the dependence being exact. (The reach from the file's
  origin, the point every reader here takes, would make the bar depend on where the solid is
  written; the size does not.) `C` is measured only for a file whose volume is outside its row's
  own bar, a handful, so the corpus costs what it did; the gate's failure message and the survey
  print `C`, the size, the row's bar and the bar held to.
* **The area**: the sum of its faces' own bars (`face_bar`) over its area (wider than a row's
  1e-6 on nearly every file by the bars' own `REL`, 1e-6, which is why almost every row's area
  bar is listed as widened, by a part in a million). It is what the
  per-face check holds the faces to, summed, so on its own it is no sharper (3.8e-3 on 212, whose
  readers agree to 1e-5): the per-face check is the sharp one. What the total adds is the faces
  that check cannot hold: one no reader measured (90 in the corpus), one the reader laid itself
  (entity 0), a disputed one held to either reading.

An earlier rule held the volume to `R·S/(3V)`, the faces' slacks swept over the reach: a bound
over every reading with that slack, 5 to 2,000 times what moving the solid shows, 0.46 on
CADGenBench 204; it is gone. With the closure: 212 is 1.0e-5 off where its file determines
5.0e-5 (`|C|` 0.55 mm² over 382 mm), 229 5.6e-6 against 4.1e-5 (0.24 mm², 195 mm), saddle-drum-
bored 2.1e-7 against 8.4e-6, helical-pipe-round-ends 9.2e-7 against 6.2e-6: all four within what
their files determine, and all four outside their rows' own bars. The row's own bar stays the
second, tighter line: such a file is `read~` in the report and listed by `account`, not failed.
Each face too: a face within its bar where OCCT and the measurer agree to a hundredth of it and
the reading stands past three times their difference is listed (`face_census`, "outside the
readers' agreement").

## The account

`the_corpus_reads` passes when every file is read right, or refused and its account says a
refusal is right. The account is `step_corpus/account.tsv`, a row a file not read right, written
by hand from evidence anyone can check without the kernel's reader, and `table.py` writes it into
`reference.tsv`'s `verdict`, `unchecked` and `account` columns:

| verdict | the file | a refusal | a reading |
|---|---|---|---|
| `gap` | sound by the standard; another reader reads it to its reference: the reader's gap | short | must be right |
| `open` | the evidence in hand settles nothing | short | must be right |
| `defective` | no solid's boundary by ISO 10303-42, or its edges stand off their faces past `sound::LOOSE` of its size | finishes it | short: a defective file read is no pass until its account says what its own geometry holds |
| `assembly` | product structure (`NEXT_ASSEMBLY_USAGE_OCCURRENCE`), which the stage defers | finishes it | must be right |
| `nosolid` | holds no solid (`ref_from` `refuse`) | finishes it | wrong |
| `reference`, `bar` | its row was corrected (the why says from what) | short | must be right |
| `unchecked` | some faces have one reference no second reader checked (`unchecked` names them) | as its other verdict | a face past its bar of that one alone is said, not failed |

A `disputed` row with no verdict may still be refused; where the account settles the dispute (a
reader shown to misread), its verdict rules. The evidence, by `check.py FILE` (OCCT with healing
off, and the text alone):

* **the text**: each EDGE_LOOP walked by vertex identity (ISO 10303-42's `path_head_to_tail`: an
  edge ending at a VERTEX_POINT the next does not start at, with both points and the distance); each
  closed shell's edges counted by the direction their uses run (a closed shell uses each once each
  way); each LINE whose VECTOR has magnitude 0 and the edges on it; a real written `NaN`;
* **OCCT**: `BRepCheck_Analyzer`'s statuses for every sub-shape not `NoError`, named by the STEP
  entity OCCT records it was made from; the largest edge and vertex tolerance OCCT's translation
  set, against the stated uncertainty and `LOOSE` of the size; and `standing_off`: each edge's curve
  projected at 23 points onto every face's surface it bounds, and its vertices onto it — how far the
  file's edges stand off, measured on its own curves and surfaces with no tolerance of OCCT's in it.

A planar face's loops' sense about its face normal is computed from the text (the signed area of
its edges' points in the plane's frame): an outer bound running clockwise bounds an unbounded part
of its plane, a hole running anticlockwise bounds a second region. Where a face is not planar,
the measurer's refusal of that same face ("a face whose loops bound no region", since it reads loops
as the standard has them and mends nothing) and the shell's edge uses are the evidence. OCCT healing
it silently is evidence the file is defective, not that it is sound: its translation turns a loop,
raises a tolerance and orders a wire with healing off.

The account has 99 rows: 68 `defective`, 4 `assembly`, 17 `nosolid`, 7 `reference` (the Utah
teapot's five, CADGenBench 211 and 250), 3 `bar` (CADGenBench 212, brepkit's saddle-drum-bored
and helical-pipe-round-ends, each read within what its file determines), and no `gap` and no
`open`: every file of the corpus is read right or finished by its account. The defects: a loop's
sense against its neighbours or its face 20; open shells, edges used otherwise than once each way
and a non-manifold edge 18; edges past `LOOSE` off their faces 11; loops not closed by vertex
identity 7 (four of them OCCT's own rewrites of a file that reads); bounds that cross or overlap
4 (brepkit's l-block open at its step, whose hole crosses its outer bound, and kentron's three
hemispheres shelled outward, whose two bounds share an arc of one meridian and end at the pole at
two vertices); a face that covers itself 2 (OCCT's oblate spheroid, a whole ellipse turned a whole
turn and bounded by a vertex alone, which covers the spheroid twice facing both ways, its flux
nought; springrep's ring pushed in past its fold, whose face crosses its extrusion's
self-intersection); a whole surface that meets itself 3 (OCCT's lemon and spindle torus, one face
on the whole surface of a circle turned across its axis, and brepkit's torus grown past its hole,
the same for a torus: the apple's sheet and the lemon's pass through each other at the poles, a
self-intersecting face, which the issue names a refusal and ISO 10303-42 5.2.1 no face's domain —
"The domains of faces and closed shells, if present, are 2-manifolds", a 2-manifold "a
non-self-intersecting surface"; OCCT reads the region the orientation bounds, the apple less the
lemon, nobody's case; for brepkit's the measurer reads the apple alone, so the account sets it
aside: the fifth column `occt`); a face of no area at all 2 (vcad's block-ridge and
slab-half-round, a plane face whose loop runs out along 24 edges and back, hanging off the shell
by two point edges: it moves no measure, and it is no 2-manifold); a real written `NaN` 1.

What were the reader's `gap`s when the account was first refreshed, and how each ended: NIST's
FTC 11 (`open` then) reads, a torus band joined round the right side of its tube; two files read
wrong (springrep's drum-leaning-across-a-turn #1969 and #2012, OCCT's
frustum-screwed-across-ring-sector #1519) were the kernel's integration of a use by ten unchecked
nodes, and read; benchy, a face that is its crumpled sheet's whole domain, reads bounded by the
rows its edges lie along. Of the 145
rows of the first account (on fcfa6597), 42 files now read right and were dropped: CADGenBench
217 among them, the `defective` its row said resting on OCCT's `SelfIntersectingWire`, which
`BRepCheck_Wire::SelfIntersect` shows to be two edges of WIRE #2588 meeting 1.27e-3 mm apart at
vertex #2591, inside its tolerance 0.0123 and the file's 0.01: a touch within tolerance, the
reading right with all 364 faces within their bars of both readers.

## A third reader

`oracle/<path>.quad.faces.tsv` (the measurer's columns: `#` and the DATA's SHA-256, then `entity`,
`area`) is an independent quadrature of some faces, made where the two readers' integrations are
in question. The gate holds a face it measures to it, and to each reader within its bar of it;
a reader past its bar of it is set aside for that face. `table.py` takes the measurer's area with
those faces at the quadrature's (`mass+quad`). The first is `teapot.py`: brep-bench's Utah teapot,
whose case area 52.2260167859 is no closed form but "the mean of [cad-kernel's, OCCT's and
cadrs's] values"; its rim, bottom, knob and lid rings are cut by nothing, so each of their 16
bicubic patches is a face over its whole domain, matched in each writer's file to the case's
patch by its control net and integrated (40-point Gauss–Legendre on 16 × 16 cells of
`|S_u × S_v|`, numpy only, converged to 1e-16): each knob patch is 0.294363892581, where the
measurer reads 0.294270388 and OCCT 0.294041293 (a pole at u = 0 each), so the case's value is
3.73e-4 mm² (7.1e-6) under its whole faces. Five writers' files read through the kernel to
52.226389… within 1e-9 of one another, and to the quadrature's every whole face to 1.8e-10. The
18 trimmed faces (the body cut by the spout and the handle, those patches, two planes) need the
trimming curves and are not integrated here: the teapot's whole area is the measurer's with its
whole faces corrected, 52.22639 on the four writers at 1e-7 (next.BREP's 52.22636, its file to
1.03e-7). A face whose one reference is OCCT's alone and in question is `unchecked` until a third
reader's table holds it; CADGenBench 250's extrusion #21919 (OCCT 1127.33, the kernel 1136.4567,
the quadrature over the file's own boundary 1136.4566796) and 211's #72139 (OCCT 944.6, the
quadrature 963.7494) now are, and OCCT's areas of them are set aside. (`face_quadrature`'s
"pcurves off their edges" took the nearest of 20,000 samples of the curve in space unrefined
until 6568eb8a; fixed, 250 #21919 measures 1136.4566795 again, and 211 #72139 963.76 ± 0.11 at
its default resolution, 963.74963 ± 1.4e-3 at four times it, the table's 963.74945 within that.) A third reader's faces
correct a row's area and tighten no bar (211 keeps its 8.5e-4).

The 11 face tables of OCCT's the faces agent found OCCT's reading no longer reproduces (kentron's
frustum-screwed-across-ring-sector, next.BREP's ball-segment- and hemisphere-rim-chamfered-to-its-
center, springrep's ball-tipped-cone-hollowed-past-its-tip and tall-frustum-sealed-below-its-
opening, vcad's cone-tip-on-face and torus-in-drum, and the partly named brepkit and kentron files)
are not regenerated: OCCT's reading now holds fewer faces than the files' solids (2 of next.BREP's
3, a face of area 2e+100 in its old table dropped; 1 of vcad torus-in-drum's 4), so no table it
makes can name each face; the gate holds those files to the measurer, which measures every face,
or says why not.

## Face by face

A total within its bar does not show a solid read right: a face wrong by half its area in a part
of hundreds of faces moves the total by less than a loose bar (CADGenBench 214's face #28635,
the whole domain of its sheet, 16.8 mm², read as 11.3). So the gate holds every face, by its
ADVANCED_FACE, to two readers' area of the same face, read from `$STEP_CORPUS/oracle/` when the
test runs (the 340,000 faces' table is 30 MB, not worth committing):

* **OCCT's**, `<path>.faces.tsv` (`oracle.py`, `BRepGProp` at the adaptive rule's 1e-10, healing
  off), named by the order of the shells' faces where OCCT keeps it and the surface kinds agree
  face by face (`faces_mapped`), else by OCCT's own record of the translation (`oracle.py --name`:
  `EntityFromShapeResult` of each face, `faces_named`; a face OCCT split names its entity on each
  part, and the parts are summed). Not named past 5,000 faces (OCCT walks its whole record per
  face): vcad's five tessellated pipes.
* **brep-bench's measurer's**, `<path>.mass.faces.tsv` (`faces.py`): the bench's `harness/mass`
  patched (`mass-faces.patch`) to print what it already sums, each face's area as ISO 10303-42
  reads its loops, put on the face from the edges' curves in space and never from the file's
  pcurves, and to measure the faces after one it cannot (it says that face's reason). It reads a
  conversion-based unit as millimetres, so an inch file's areas are square inches, and a file of
  several contexts in another context's unit: the gate reads the scale off the median ratio of
  the faces both readers measure, 1 or 25.4² where it is so to a part in a hundred, and else
  sets the measurer aside (`measurer_scale`; a file OCCT's faces do not join, 1 where its only
  unit is the millimetre).

A face's bar is **`TOL × Σ max(tol, u)·length + REL × area + ABS`**, the sum over its loops'
uses, `tol` the edge's measured tolerance and `u` the file's stated uncertainty (`face_bar`,
`Face::slack`). A tolerant boundary is known only to within its edges' slack, and two readings of
it — one by the file's pcurves (OCCT, the kernel), one by the curves in space (the measurer) —
sweep at most that slack along each edge between them: `TOL = 1` is that bound, not a fit. The
writer states its curves only to `u`, and the two readers show it: on the files written to 0.005–
0.02 mm the measured `tol` alone (`K × Σ tol·length`) would flag 17 files whose faces OCCT and
the measurer themselves read apart by more than it (CADGenBench 206's torus #58555: OCCT
24.5749, the measurer 24.5610, the kernel 24.5694, measured slack 0.001, `u`'s 1.3), and twice or
four times it still 16 and 13; with `max(tol, u)` four, all of them 229's face below.

`REL = 1e-6` is the readers' quadrature: on the 116,298 faces of the files the totals pass,
past their slack the worst face agreeing with a reader stands at 9.4e-8 of its area (kentron's
helical pipe), so 1e-6 leaves ten; `ABS = 1e-9` mm² a face of no area. Measured on the whole
corpus at the merged tree (November 22, 2026): `TOL` = 1 and 0.5 flag the same four files,
0.25 nine (CADGenBench 204 and 246's faces by OCCT alone, within their slack), 0.1 eighteen;
`REL` = 1e-7 the same four, 1e-8 seven. The four are the variants of CADGenBench 229, whose face
#14114 the kernel reads 1.8% under both readers (229 itself fails in total).

A reader is no reference for any face of a file whose faces it sums to an area that is not the
file's (outside the row's bar of its reference or its second reading): it has misread a face
somewhere, and OCCT reads each sphere of `balls-meeting-through-a-pole` as a quarter of itself.
The measurer is held to that only where it measured every face.

Where the two readers both measure a face, the reading must be within its bar of each where they
agree within it, and of either where they do not (`disputed`, as a file is: 137 faces, among
them an extrusion face of `peanut-prism` OCCT measures 4% off its closed form). Where one alone
does, of that one. A face neither measures is not compared and is counted. A file whose OCCT
faces are not the file's (OCCT drops a void's shell, reads fewer faces) is held to the measurer
alone; one with neither table, or tables of other bytes than the file's (the first line of
`.mass.faces.tsv` is its DATA's SHA-256, the JSON's `data_sha256` OCCT's), is not compared and
said: never taken for agreement. `nothing_is_read_wrong` prints the count of files compared face
by face and of those not, with why (on the merged tree: all 3,013 files read, 121,924 faces, 137
of them disputed, none unmeasured; 132 mutants read, 700 faces). A file read right in
total only as its row's second reading (`alt`: a disputed file's measurer's, a mutant's unedited
source's) is not held face by face, the oracle's faces being the first reading's.

No centroid is compared. A face is bounded by its file's edges, which the reader takes as they
are, so a face in the wrong place with its right area would need its loops read on a different
part of its surface with the same area: a wrong branch of a period puts the same points in the
same place, and a wrong side of a loop changes the area. It would cost three more integrations a
face, the area's cost thrice.

## Debugging a file

```sh
cd rust
cargo run -q --release -p cad-kernel --example step_survey -- \
    --reference gcs-core/tests/step_corpus/reference.tsv $STEP_CORPUS/cadgenbench/218/input.step
cargo run -q --release -p cad-kernel --example step_survey -- --faces $STEP_CORPUS/cadgenbench/218/input.step
cargo run -q --release -p cad-kernel --example step_survey -- --faces --past-sound $STEP_CORPUS/cadgenbench/214/input.step
python3 -I scripts/step-corpus/oracle.py --faces $STEP_CORPUS/cadgenbench/218/input.step   # OCCT's, per face
python3 -I scripts/step-corpus/check.py $STEP_CORPUS/brep-bench/brepkit/sweep/duct-elbow.step  # is it a solid's boundary?
cargo run -q --release -p cad-kernel --example step_survey -- --moved 10 $STEP_CORPUS/cadgenbench/212/input.step  # its closure
```

`--faces` prints, beside each face the reader reads, OCCT's area and the measurer's, the
reading's relative difference from each, the face's slack and bar, and `ok`, `OFF` or `disputed`,
as the gate judges it, wherever the file lies under `$STEP_CORPUS` (or `--oracle CORPUS`);
`--past-sound` reads a file `sound` refuses once more without it and lists that reading's faces,
so a refusal can be checked face by face.

## What OCCT's reading is, and is not

* **Its solids only.** Where OCCT reads solids, what it read beside them (a commercial file's
  construction surfaces, its wireframe and curve sets) is set aside before anything is measured or
  counted (`oracle.solids_of`): `BRepGProp` over the whole shape counted those faces' areas and
  their open shells' "volumes" (stc_08: 23% in volume), and the reference is the solid's. The
  oracle's readings made before (any file whose text holds a `SHELL_BASED_SURFACE_MODEL` or a
  `GEOMETRIC_CURVE_SET` beside its solid: the 27 commercial files) are read again by removing
  their `oracle/…json` first: the hash of the text alone does not say the oracle changed.
* **Healing off.** `oracle.py` reads with `read.step.sequence` naming no sequence, so OCCT's
  shape processing (`FixShape` and the rest) does not run; it reads again with it on and records
  what changed (`healed`). The translation itself still fixes what it fixes (a vertex's tolerance
  raised to cover its edges' ends, a wire's order, missing pcurves computed): no switch reaches it.
* **Its measures have an error, measured per file.** Volume and area at the adaptive rule's
  1e-6, 1e-8 and 1e-10 and the fixed Gauss rule. On CADGenBench the adaptive ones agree to 1e-8
  on most files and differ by up to 4e-3 on some (211); the fixed rule is off by up to 7e-2 (204).
  A reading that has not converged is not used as a reference beside a second reader's.
* **A second reader.** brep-bench's measurer (`harness/mass`) integrates the exact geometry with
  each edge's curve in space put on its faces, not the file's pcurves. On files written to 1e-7
  the two agree to 1e-12; on CADGenBench, written to 0.005–0.02 mm, they differ by up to 2e-4:
  the files themselves are no more precise than that, and the bars say so.

## Found on the way

* OCCT's shape healing changed nothing measurable on any of the 32 CADGenBench files (counts,
  validity, tolerances, volume and area alike); its translation's own fixes cannot be turned off.
* brep-bench's measurer (at 7d811e9) reads a `CONVERSION_BASED_UNIT` (inches) as millimetres:
  an inch file's volume comes out in cubic inches. `table.py` uses it only for files in mm.
* On cadgenbench/243 the measurer's area is 2.4% off OCCT's (converged to 3e-8): a face it
  misreads; the table takes OCCT's there (`occt!`).
* Writers' habits the CADGenBench files (all OCCT 7.8, AP214) do not show: no pcurves at all
  (brepkit, kentron, vcad), `FACE_OUTER_BOUND` (every writer but OCCT), `VERTEX_LOOP` at a pole and
  assemblies (`NEXT_ASSEMBLY_USAGE_OCCURRENCE`, OCCT 8), `SHELL_BASED_SURFACE_MODEL`/`OPEN_SHELL`
  (OCCT 8, no solid), `OFFSET_SURFACE`, `HYPERBOLA`, `PARABOLA`, AP203's approval and security
  entities, inch units. `brep_bench.py census` counts every kind per source.
