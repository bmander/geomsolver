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
| `reference.tsv` | a row per file of the corpus: its DATA section's SHA-256, the entities its solids hold, OCCT's reading (healing off, and what healing changed), brep-bench's measurer's, and the reference volume and area with the bar a reading must come within (`table.py` says how each is chosen) |

## The corpus

```
$STEP_CORPUS/
  cadgenbench/<id>/input.step                 the 32 CADGenBench editing inputs (+ edit_description.txt)
  brep-bench/<kernel>/<category>/<case>.step  what each kernel wrote for a settled case
  brep-bench/<kernel>/runs.tsv                each run: ok, refused (why), no-step, crashed, timeout
  brep-bench/reference/<category>/<case>.step a case's reference shape (integration/benchy)
  variants/<habit>/<source path>              a sample written again by OCCT: nopcurves, ap203, ap242, inches
  oracle/<path>.json, .faces.tsv, .mass.json  OCCT's reading, per face, and the bench measurer's
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
sh $S/fetch.sh                                              # CADGenBench, checked byte for byte
python3 -I $S/brep_bench.py prepare                         # copy the bench into build/
python3 -I $S/brep_bench.py build mass occt cad-kernel geomsolver kentron arris vcad brepkit next.BREP springrep
python3 -I $S/brep_bench.py run occt cad-kernel geomsolver kentron arris vcad brepkit next.BREP springrep --jobs 2
python3 -I $S/brep_bench.py references                      # the reference shapes' files
python3 -I $S/variants.py --jobs 2                          # the habits, over the sample
python3 -I $S/oracle.py --corpus --jobs 2                   # OCCT's reading of every file
python3 -I $S/table.py measure --jobs 2                     # the bench measurer's
python3 -I $S/table.py                                      # reference.tsv, committed and in $STEP_CORPUS
python3 -I $S/brep_bench.py manifest                        # brep_bench_cases.tsv (on a new bench commit)
python3 -I $S/brep_bench.py census                          # entity kinds per source
```

Every step is idempotent: a file already made, read or measured (by its DATA section's hash) is
not made again. The CADGenBench bytes are fixed; the kernels' files are their writers' and are
only as reproducible as those (the header's time stamp is left out of the hash). A regenerated
corpus whose files differ from the committed table's is reported **stale** by the gate test: run
`oracle.py --corpus`, `table.py measure` and `table.py`, and commit the table (or point
`STEP_CORPUS_TABLE` at `$STEP_CORPUS/reference.tsv`).

Kernels of brep-bench's `kernels.json` with `"step": true` that build here from crates.io:
`cad-kernel` (=0.15.0), `kentron`, `arris`, `vcad`, `brepkit`, `next.BREP`, `springrep`; `occt`
runs on the installed OCP; `geomsolver` is `solventc` at the tag its `build.py` names, built from
this repository's own history (`git archive`, no network). Not built: `solidcraft`, `popkern`,
`geop`, `cadrs` (their sources are on GitHub repositories this machine cannot reach), `aetheris`
(.NET 10 and a GitHub source), `smlib` (no STEP writer), `monstertruck` (no runner).

## The gate

`rust/gcs-core/tests/step_corpus.rs`, three tests behind `#[ignore = "a tool: …"]` (the corpus is
outside the repository; the ordinary suite neither needs it nor runs them):

* `step_corpus::nothing_is_read_wrong` — each file refused, or read to as many solids as it
  holds, every ADVANCED_FACE, EDGE_CURVE and VERTEX_POINT read and none twice, the volume and the
  area within the row's bar of its reference. Must pass at every commit.
* `step_corpus::nothing_panics` — no read panics; a panic names its file.
* `step_corpus::the_corpus_reads` — every file read to its reference, none refused: the stage's
  finish line, failing until the reader is done. Its table (file, outcome, volume and area
  errors, seconds, the refusal's witness) is the progress report.

```sh
cd rust
STEP_CORPUS=/home/user/step-corpus cargo test -j 2 -p gcs-core -- step_corpus:: --ignored --nocapture
```

`STEP_CORPUS_ONLY=cadgenbench/` reads only the files whose path starts so, `STEP_CORPUS_THREADS`
sets how many at once. Each file is read once for the three tests. Two quick tests run in the
ordinary suite: the table is whole (`the_table_is_whole`) and the hash is SHA-256.

## Debugging a file

```sh
cd rust
cargo run -q --release -p cad-kernel --example step_survey -- \
    --reference gcs-core/tests/step_corpus/reference.tsv $STEP_CORPUS/cadgenbench/218/input.step
cargo run -q --release -p cad-kernel --example step_survey -- --faces $STEP_CORPUS/cadgenbench/218/input.step
python3 -I scripts/step-corpus/oracle.py --faces $STEP_CORPUS/cadgenbench/218/input.step   # OCCT's, per face
```

The oracle's per-face table names each face's ADVANCED_FACE (OCCT keeps a shell's order of faces;
`faces_mapped` in the JSON says whether the surface kinds of the two lists agreed, face by face),
so the survey's `--faces` listing and the oracle's join on the entity.

## What OCCT's reading is, and is not

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
