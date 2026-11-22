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
| `brep_bench_cases.tsv` | brep-bench's settled cases (its commit in the first line): closed forms of volume and area, the tolerance the bench grades at, the centroid where the case gives one |
| `commercial_files.tsv` | the files commercial systems wrote, by path, with their sizes, SHA-256, and the archive and member each is taken from: `fetch.sh` checks every byte against it |
| `reference.tsv` | a row per file of the corpus: its DATA section's SHA-256, the entities its solids hold, OCCT's reading (healing off, and what healing changed), brep-bench's measurer's, and the reference volume and area with the bar a reading must come within (`table.py` says how each is chosen) |

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
reader refusing every solid, the 3,139 files took 7–20 s on four busy cores). Two quick tests run
in the ordinary suite: the table is whole (`the_table_is_whole`) and the hash is SHA-256.

What counts as right, per row's `ref_from`: `closed` (the case's closed forms, at the bench's
grade), `mass+occt`, `mass`, `occt`, `occt?`, `occt!` (a reading's own solid, `table.py` says
at what bar), `source:…` (a variant: its source's); `refuse` (the file represents no solid — a
shell-based surface model, an empty product — and must be refused); `disputed` (OCCT and the
bench's measurer read it differently, mostly loops run against ISO 10303-42 that OCCT repairs:
either reading or a refusal). A file with no row is not gated: OCCT and the measurer both fail on
it, or its bar would pass a part in fifty (`table.py` prints each with why).

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
  conversion-based unit as millimetres, so an inch file's areas are square inches: the gate
  scales them by 25.4² (`measurer_scale`).

A face's bar is **`TOL × Σ max(tol, u)·length + REL × area + ABS`**, the sum over its loops'
uses, `tol` the edge's measured tolerance and `u` the file's stated uncertainty (`face_bar`,
`Face::slack`). A tolerant boundary is known only to within its edges' slack, and two readings of
it — one by the file's pcurves (OCCT, the kernel), one by the curves in space (the measurer) —
sweep at most that slack along each edge between them: `TOL = 1` is that bound, not a fit. The
writer states its pcurves only to `u`, which the kernel's measured `tol` (of its own pcurves) does
not see, so `max`. `REL = 1e-6` is the readers' quadrature: on the 115,054 faces of the files the
totals pass, past their slack the worst agreeing face stands at 9.4e-8 of its area (kentron's
helical pipe), so 1e-6 leaves ten; `ABS = 1e-9` mm² a face of no area. Measured on the corpus
(November 22, 2026), `TOL` = 1 and 0.5 flag the same five files, 0.25 eleven (CADGenBench 204,
209 and 246's faces by OCCT alone, within their slack), 0.1 sixteen; `REL` = 1e-7 the same five,
1e-8 nine. The five are the four variants of CADGenBench 229, whose face #14114 the kernel reads
1.8% under both readers (229 itself fails in total), and an inch variant OCCT misreads (below).

Where the two readers both measure a face, the reading must be within its bar of each where they
agree within it, and of either where they do not (`disputed`, as a file is: 370 faces, mostly an
extrusion or B-spline face of a brep-bench file OCCT measures 4% off its closed form, as on
cad-kernel's `peanut-prism`). Where one alone does, of that one. A face neither measures is not
compared and is counted. A file whose OCCT faces are not the file's (OCCT drops a void's shell,
reads fewer faces) is held to the measurer alone; one with neither table, or tables of other
bytes than the file's (the first line of `.mass.faces.tsv` is its DATA's SHA-256, the JSON's
`data_sha256` OCCT's), is not compared and said: never taken for agreement. A file read right in
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
