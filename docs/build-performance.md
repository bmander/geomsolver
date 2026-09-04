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
touch rust/gcs-core/src/lib.rs
/usr/bin/time -p make test
make bench
```

Record Cargo's separate `Finished ... in` lines and the test-suite durations as well as total
wall time. Compare like cache states, and run runtime benchmarks after compilation has stopped.
Removing `rust/target` measures a cold build, not normal edit turnaround.
