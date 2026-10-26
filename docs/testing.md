# Which tests run when

The suite is too large to run whole on every edit: 3,131 tests in the core's binary alone on
October 26, 2026, 3,042 of them in the fast tier, 18.4 CPU-minutes and five minutes of wall clock
on four cores, and then the CLI's (47 s), the ABI's and the web suites.  So each tier runs at the
point where its answer is worth its wait, and no tier is the only one that would have caught a
failure.

| when | what runs | how | budget |
|---|---|---|---|
| while editing | the module of the area in hand | `cargo test --manifest-path rust/Cargo.toml chain::` | seconds |
| before a push | the modules the change can reach, and a smoke set | `make test-changed` | a minute or two |
| every push, every PR from a fork | the whole fast suite, every crate, both release artefacts, the web suite | CI: `.github/workflows/test.yml` | minutes |
| every night, on main | the slow tier, and the fast suite again in the slow build | CI: `.github/workflows/slow.yml` | hours |
| by hand | the CalculiX oracle, OCCT's native tooth space, the tools behind `#[ignore = "a tool: …"]` | `make test-ccx`, `make test-slow OCCT=1`, `-- --ignored` | anything |

**CI is the gate.**  A change lands when `test` is green on its last push; nobody need run the
whole of `make test` by hand first.  `make test` still runs it all locally, for a change whose
reach `make test-changed` cannot see (it says so, and runs everything, when the change is to a
manifest, `lib.rs`, `std.sv`, the suite's shared helpers, or reaches half the modules anyway).

## Before a push: `make test-changed`

`scripts/test-changed.sh` reads what changed since the merge base with `origin/main`
(committed, staged, unstaged and untracked) and maps each file to the modules of the core's
suite that can see it: a test file to its module; a source file `src/a/b.rs` to the modules that
name `a::b` and those called `b` or `b_…`; a document to the modules naming it or its case; a
file under `docs/` to the modules that read it.  The CLI's, the ABI's and the web suite run when
their own files changed.  The smoke set, `smoke::`, `examples_sv::` and `every_file_is_a_module`
(eight seconds), runs with every selection.  `LIST=1` prints the selection without
running it, and `FILES="a b"` asks about paths instead of the diff.  The first line it prints
names the base and how many files changed since it: hundreds mean the clone's `origin/main` is
old, and `git fetch origin main` puts it right.

The map is by names, so it under-selects when a change reaches a test only through the
language (a parser change seen by a test that only elaborates a document).  That is the trade:
this tier is fast and usually right, and the next tier is CI, which is always whole.

## CI: shards dealt by time

`scripts/test-shard.sh K N <cargo test arguments>` runs the K-th of N parts of a test binary;
CI deals the core's suite into four on push and eight at night.  It deals by module, because a
module's tests share fixtures built once per process (`OnceLock`s in `twist_drill`,
`envelope::paired`) that a split would build again on every machine, and it deals a test of 30 s
or more on its own.  Each unit goes to the lightest part so far, longest first, by the times in
`rust/test-times.tsv`; a test missing from that file weighs 0.2 s, or a minute in a slow build.
Every machine computes the same split, and the parts cover the binary exactly once (`LIST=1`
prints a part).  The libtest runner is used and not one that runs each test in a process of its
own, for the same fixtures.

**The suite is built once.**  A shard that compiled the engine for itself spent most of its six
minutes compiling, four times over.  CI's `build` job compiles the core's test binary and hands
it to the shards as an artefact; each runs its part with `--binary <path> rust/gcs-core`, in the
package's directory as cargo would, and reads its documents from its own checkout at the path
the binary was built in (`CARGO_MANIFEST_DIR` is compiled in, and every hosted runner checks out
to the same place).  The wall clock is the same, about six and a half minutes from push to
answer, since the shards wait for the build; the core's runner-minutes halved, 22 to 11.

**A cache holds the third-party crates, and only they count.**  Cargo judges the workspace's own
crates by their sources' times, and a fresh checkout's are newer than anything a cache holds, so
`gcs-solve`, `gcs-core` and the suite compile in every run, about four minutes, whatever is
restored; what a restore saves is nalgebra and the rest, some twenty seconds.  The caches are
keyed by the lockfile for that reason: a key per commit would save a new copy every push and
buy nothing.

`rust/test-times.tsv` is `make test-times`: every test of the binary timed by libtest itself
(`--report-time`, which stable libtest gives when `RUSTC_BOOTSTRAP` is set where the binary runs;
the script sets it there and not for cargo), merged into the file and the slowest printed.  The
times are taken with the suite's own parallelism, which is how a shard runs them.  Refresh it
when the shards drift apart (each shard's log opens with the time the file predicts for it); a
stale file costs balance, never coverage.

## The fast tier's budget

A test of ten seconds or more, timed alone, is in the slow tier
(`#[cfg_attr(not(feature = "slow"), ignore = "slow tier, …")]`), or is made smaller until it is
not: fewer teeth, a coarser mesh, a lower degree, one case of a sweep with the sweep moved to the
slow tier.  A smaller instance that still states the property is better than a demotion, since
it keeps the property in every push.  `make test-times` lists the candidates.

**A gate the contract names stays in the fast tier.**  A rule in `docs/contract/` held by a test
it names is held on every push; demoting that test would leave the rule unheld until the next
night.  Such a test is made smaller or left where it is, and CI's shards carry its cost.  On
October 26, 2026 six were over ten seconds and stayed: `jacobians::` a solid's and a filled
body's deflection against their differences, `bodies::` every solid case's claims and
references, `frames::a_tool_riding_its_frame_is_placed_and_swept_as_under_it` and
`iterative::cg_over_the_corners_agrees_with_the_factor`.

Time a test alone (`--exact <name> --test-threads=1`) before deciding.  A module whose tests
share a fixture shows each of them slow alone and quick in the suite (`envelope::paired`'s ten
each take 14 s alone, the fixture's cost, and 6–12 s in the suite): demoting one of them saves
nothing, and the module is either all fast or all slow.

The slow tier runs every night whether or not anyone asked, because a tier nobody runs is not
holding anything.  A scheduled run that fails mails whoever last changed its schedule, and its
page names the test.  Run it by hand (Actions → slow → Run workflow, on the branch) before a
change to the gear example or the native export lands.

## The first pass (October 26, 2026)

Measured on a four-core Linux container, warm build, `cargo test -p gcs-core --test core` and
`cargo test -p gcs-cli --test cli`:

| | before | after |
|---|---:|---:|
| the core's fast suite, wall clock | 6 min 5 s | 4 min 56 s |
| the core's fast suite, CPU | 22.7 min | 18.4 min |
| the CLI's suite, wall clock | 4 min 11 s | 47 s |
| one CI shard of the core's four | — | 74 s |

- `cli::the_whole_library_reports` ran `solventc` over the library one document after another,
  `sized_conrod.sv` (200 s of its 246) among them.  That case is `examples::SLOW`, which the
  core's every-case tests already left to the slow tier; this test now does too, and deals the
  rest to one `solventc` per core.  With the slow feature it reads the conrod as before.
- `program::nothing_in_the_parser_panics` cut every case at every prefix: in the fast tier it cuts
  at every third (20 s alone to 7), and the slow tier at every one.
- Twelve tests of ten seconds or more alone, no contract's named gate, moved to the slow tier:
  `planar_subtraction::the_drafted_handle_cuts_its_wheel`,
  `placed_numbers::a_front_holds_a_point_through_its_speed`, `hypoid_layout::` the layout solved
  in block order and rescued from its iteration limit, `brep_export::` a placed sweep and a sweep
  built by stages, `brep_census::an_involute_tooth_is_its_closed_form`,
  `accuracy::a_static_spline_face_offset_reads_as_its_offset`,
  `rate_examples::fish_eye_fronts_refocus_square_to_the_rays`,
  `regular::the_bound_breaks_where_the_exported_flank_first_folds`,
  `rates::a_tube_on_a_helix_is_its_section_along_its_length` and
  `helix::a_screwed_wall_is_written_and_meshed`.  Each is a candidate for a smaller instance
  that would bring it back.
