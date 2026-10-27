# Build and test turnaround

`make test` builds the native and WebAssembly release libraries, then runs the complete
Rust workspace and web suites. Both suites must succeed. Run it normally; it overlaps the
suites automatically and inherits an existing parallel Make jobserver. Use
`make test TEST_JOBS=1` to run the suites serially; GNU Make 3.81 drops `-j1` from the flags
available to the Makefile, so that setting cannot be distinguished from a plain invocation.

The build settings favour repeated edits:

- Release: optimisation level 3, ThinLTO, 16 codegen units and incremental compilation.
  Cargo keeps the reusable compiler output in `rust/target`; both release targets build
  in one Cargo invocation so separate invocations do not wait on the same directory lock.
- Tests: optimisation level 2, incremental compilation, debug assertions and no debuginfo.
  Numerical tests still run optimised. The four library/binary unit-test harnesses contained
  zero tests and are disabled; the integration suites and documentation tests remain enabled.
  Enable a target's `test` field if adding unit tests inside its source in future.
- TypeScript: incremental compilation, with its build information inside `web/dist` so a
  clean output directory also means a clean compiler cache. Type checking and bundling still
  run through the ordinary `npm test` build.
- Make tracks the workspace manifest, member manifests, lockfile and toolchain file for
  standalone builds too, so a changed compiler profile cannot leave an old copied library.

Cargo documents the tradeoffs in [profiles](https://doc.rust-lang.org/cargo/reference/profiles.html)
and the distinction between [unit-test targets and integration tests](https://doc.rust-lang.org/cargo/reference/cargo-targets.html).

## Measurements

Measured on the development machine (Intel macOS, rustc 1.97.1), September 4, 2026.
These are local wall-clock observations, not a CI performance threshold; background load
and macOS's first-launch assessment affect them.

| Work | Before | After |
| --- | ---: | ---: |
| Both release libraries, rebuilding without reusable incremental output | 121 s | 64 s |
| Both release libraries after touching a core source file | 117 s | 3.5 s |
| Complete `make test` after touching a core source file | 160 s | 31 s |
| Complete `make test`, no source changes and warm caches | not measured | 18 s |

The touched-file measurement forces Cargo to recheck the core, without changing its machine
code. It measures reuse, not the cost of every possible source edit. The first build after a
profile change must populate new caches: the first complete run with these settings took
232 s, including a 152 s rebuild of the native test profile and its dependencies.

A separate check changed the allocation in `Rng::sample` from `Vec::new()` to
`Vec::with_capacity(k.min(n))`, requiring new machine code while preserving its results.
The full run took 88 s: 33 s for the release build and 22 s for the native test build.
That temporary change was reverted; no solver source changes are part of this optimisation.

The test totals remain 720 Rust tests passed, one pre-existing ignored test, and 216 web
tests passed. No assertions were removed or new tests ignored to improve the timings.
Validation also covered `make -j4 test`, rebuilding from an empty `web/dist`, and an
intentional TypeScript type error detected with an existing incremental cache. The probe
was removed after confirming the build failed.

There is a release-size tradeoff: WebAssembly grew from 2,797,097 to 3,255,918 bytes (16%).
Three alternating runs of the existing WebAssembly benchmark against the saved old and new
binaries gave these median times:

| Operation | Before | After |
| --- | ---: | ---: |
| `truss(200)` DogLeg solve | 10.41 ms | 10.44 ms |
| `truss(200)` drag frame | 5.10 ms | 5.13 ms |
| `rect_fillets` LM solve | 0.52 ms | 0.61 ms |
| `polygon_chain(12)` DogLeg solve | 0.43 ms | 0.50 ms |
| `slotted_link` DogLeg solve | 0.075 ms | 0.061 ms |

The largest case was effectively unchanged in this sample; several smaller operations were
slower and others faster. Recheck this balance when changing the compiler or the solver.

## Reproduce

Install the locked web dependencies with `npm --prefix web ci`. Keep `rust/target` and
`web/dist` between runs, then use:

```sh
/usr/bin/time -p make test
touch rust/gcs-core/src/lib.rs      # an edit to the language: gcs-core and what is built on it
/usr/bin/time -p make test
touch rust/gcs-solve/src/lib.rs     # an edit to the solver: both crates
/usr/bin/time -p make test
make bench
```

Record Cargo's separate `Finished ... in` lines and the test-suite durations as well as total
wall time. Compare like cache states, and run runtime benchmarks after compilation has stopped.
Removing `rust/target` measures a cold build, not normal edit turnaround.

# Opening an assembly in the page (#669)

`node web/tools/open-bench.mjs [EXAMPLE]` (the last step of `make bench`) opens an example in
headless Chromium as a person opening it does, and reads what the page marks on its User Timing
line (`gcs:*`) beside `solventc --no-check --step-assembly` run natively on the same file.  Its
case is `rust/examples/gear_train/train.sv`: three stages of shaped spur gears (the radiator bot's
`ShapedSpur`) between two plates, 14 objects, the second and third stages one size so two of its
figures are each placed twice.  Radiator bot itself (`bmander/radiatorbot` at `bb1e8a3`,
`hardware/`, 36 objects) is the manual check; it was measured from an uncommitted copy under
`rust/examples/`.

**Headless Chromium draws WebGL in software (SwiftShader).**  Draw calls, triangles, geometries
and programs are the page's; the time of anything the GPU does is not a GPU's.  On `3071006` the
page's first frame holds the main thread for about 3.8 s waiting on the GPU process (its first
paint with #681's renderer, 12 programs against 6), before the reading starts; on `520a8b2` the
first frame does not.  That is why "reading done" on the gear train went from 3.8 s to 7.3 s: the
reading itself did not change.

Measured October 27, 2026 on a 4-core Linux VM (Intel Xeon, 2.8 GHz, 15 GB), Chromium 1243
through Playwright 1.63; the gear train opened twice on each version (their mean shown; the two
agreed within 0.4 s), radiator bot once:

* **before**: `520a8b2`, `main` at #680's merge — static objects built on the page's thread
  under the modal, swept ones refined from their material field in mesh workers while up to 4
  more workers try the exact build;
* **after**: `3071006`, `main` at #683's merge — every object built exactly in a pool of 3
  workers, a bar instead of the modal (tracks A, B and C of #668).

| gear train (14 objects) | before | after |
|---|---:|---:|
| reading done (elaborate, solve, diagnose) | 3.8 s | 7.3 s (3.8 s of it the first frame, above) |
| modal put away | 13.8 s | 7.3 s |
| first object drawn | 4.6 s | 9.2 s |
| last object drawn final | 138 s | 9.7 s |
| objects drawn final | 12 of 14 | 10 of 14 |
| main thread: long tasks, longest, total blocking | 100, 3.3 s, 60.2 s | 9, 3.8 s, 8.7 s |
| draw calls, triangles, geometries, programs | 194, 110 636, 172, 6 | 211, 5 444, 182, 12 |
| meshes in objects | 48 | 10 |
| peak heap: page (V8 + backing stores), largest worker | 43 + 100 MB, 4 MB | 16 + 21 MB, 2 MB |

| radiator bot (36 objects) | before | after |
|---|---:|---:|
| reading done | 22.6 s | 24.3 s |
| modal put away | 63.4 s | 24.3 s |
| first object drawn | 26.4 s | 33.4 s |
| last object drawn final | 315 s | 39.0 s |
| objects drawn final | 31 of 36 | 29 of 36 |
| main thread: long tasks, longest, total blocking | 135, 22.0 s, 148.5 s | 17, 20.1 s, 32.2 s |
| draw calls, triangles, geometries, programs | 1 011, 217 822, 971, 6 | 411, 35 324, 371, 12 |
| meshes in objects | 688 | 29 |
| peak heap: page, largest worker | 78 + 146 MB, 8 MB | 35 + 22 MB, 6 MB |

Natively, `solventc --no-check --step-assembly` builds every object of both: the gear train in
7.3 s wall (0.2 s elaborating, 1.4 s to diagnosed, the rest building one object after another,
the slowest `b_part` at 2.6 s), radiator bot in 45.5 s (5.1 s elaborating, 9.7 s to diagnosed,
44.2 s to every object built, 33 MB of STEP at 45.4 s).

**Objects the page does not build.**  In both versions the page's stepped exact build refuses
every object that holds a shaped gear, which the native export builds:

* a body whose stock is a swept-cut body, `solid(g.gear)` (the gear train's `a_part` and
  `d_part`, radiator bot's `pinion` and `socket`): "`a_pin.gear`: a body's stock may not contain a
  continuous sweep (at blank)", `solid/cad.rs`;
* a compound, a gear joined to its pinion (`b_part`, `c_part`; `a_compound`, `compound_b1`,
  `compound_b2`, `compound_d1`, `compound_d2`): "`b_part` is built from 2 swept bodies; its
  display is built from one (at blank)", `brep/export.rs`.

Before, these were drawn from their fields: the single gears' finished field surfaces arrived
after 2 to 5 minutes, and the compounds showed only their first, coarse surface.  After, the page
meshes no field, so they are drawn as nothing and the build bar says why.  The "drawn final" rows
count only what was drawn.

**What the rest says.**

* The modal now goes when the reading is done; before, it waited ten seconds (the gear train) to
  forty (radiator bot) more for the swept objects' first field surfaces.
* Every object the page builds is final within 15 s of the modal going, against minutes; the
  main thread blocks a fifth to a seventh as long, and the page holds less than half the memory.
* The first object is drawn later, 9.1 s after the reading on radiator bot: each pool worker
  reads the document for itself before its first build.  The builds themselves are 5.1 s for
  all 29 (the star handle 3.7 s, the rest under 0.4 s).
* The reading holds the thread as before (one 20 s task on radiator bot, against 9.7 s natively
  to the end of the diagnosis): #668 leaves it for later.
* One mesh, one geometry and one draw call per object (#674): radiator bot's objects went from
  688 meshes to 29.

To repeat a measurement, build the native CLI, the core and the app, then open the case:

```sh
make solventc wasm && (cd web && npm run build)
node web/tools/open-bench.mjs                          # the gear train, page and native
node web/tools/open-bench.mjs hub_motor --json out.json  # another example, as JSON too
```
