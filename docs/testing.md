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
| a kernel release, tag `cad-kernel-v*` | the fast suite as on a push (the tag's push skips `test.yml`, which this calls), and the kernel's package alone: its smoke test outside the workspace, its docs, semver against crates.io | CI: `.github/workflows/cad-kernel.yml` | minutes |
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

`scripts/test-shard.sh K N --binary <test binary> <its package's directory>` (or `K N <cargo test
arguments>`, which builds the binary first) runs the K-th of N parts of a test binary;
CI deals the core's suite into four on push and eight at night.  It deals by module, because a
module's tests share fixtures built once per process (`OnceLock`s in `twist_drill`,
`envelope::paired`) that a split would build again on every machine, and it deals a test of 30 s
or more on its own.  Each unit goes to the lightest part so far, longest first, by the times in
`rust/test-times.tsv`; a test missing from that file weighs `DEFAULT_SECONDS`, 0.2 s, or the
minute slow.yml gives it.
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

**Nothing is cached.**  Cargo judges the workspace's own crates by their sources' times, and a
fresh checkout's are newer than anything a cache could hold, so `gcs-solve`, `gcs-core` and the
suite compile in every run, about four minutes, whatever is restored.  The only other crates are
nalgebra's, the tests' one dependency, which compile beside `gcs-solve` and off the path that
matters.  Incremental compilation, which a build from nothing cannot use, is off
(`CARGO_INCREMENTAL=0`).  The other crates' suites and the doc tests run in the `build` job
after its upload, on the libraries it has just compiled, rather than in a job that would compile
them again.

`rust/test-times.tsv` is `make test-times`: every test of the binary timed by libtest itself
(`--report-time`, which stable libtest gives when `RUSTC_BOOTSTRAP` is set where the binary runs;
the script sets it there and not for cargo), merged into the file and the slowest printed.  The
times are taken with the suite's own parallelism, which is how a shard runs them.  Refresh it
when the shards drift apart (each shard's log opens with the time the file predicts for it); a
stale file costs balance, never coverage.

## Building a sweep: the inner loop (#777)

A failing closed sweep is diagnosed fastest as the smallest Boolean of its candidates, not as the
sweep: a one-second arrangement finds what minutes a sweep did not
(`swept_closed::two_tori_touching_and_a_plane_on_both_hold_every_piece`). So **a sweep failure is
reduced to the smallest Boolean of its candidates and kept as a fast test before it is fixed**:

1. Run the failing build with `SOLVENT_SWEEP_DUMP=<dir>`: its domain, every candidate solid and
   the tolerance are written whole (`brep::swept::dump`).
2. `dump::load(dir)` and `Arrangement::reduce` with a predicate that says the failure (an error,
   a missing piece, a wrong cell count) give the fewest candidates that still fail; write those as
   a test of `boolean::arrangement_within` or `boolean::selected`, and see it fail.
3. Fix the kernel against that test; then run the sweep's gate.

`SOLVENT_SWEEP_PROFILE=1` prints what a closed build spent per stage and which pairs of surface
kinds were traced rather than met in closed form, and what the meetings' analysis did (critical
sets, branch tests, a level curve's readings): start a performance change from it, and cite it
before and after. `touch_curves::` prints the same profile for each of its tests under it. A
build past `closed::BUDGET` is refused by the stage that ran away (`Unbuilt::Budget`), in seconds
rather than hours.

| tier | tests | typical time | when to run |
|---|---|---|---|
| 1 | the reduced Booleans (`swept_closed::two_tori_…`, the arrangements a dump reduced to) | about 1 s | the inner loop |
| 2 | the kernel's sweep gates (`swept_closed::`, `cargo test swept_closed::`) | seconds to tens of seconds | the inner loop |
| 3 | drawing, STL and examples (`object_builder::`, `solid_sweeps_examples::`) | minutes | before a commit |
| 4 | the fast suite (`make test-changed`, then CI) | minutes | before a push |

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

## The slow tier's first pass (November 4, 2026, #780 R1)

Each slow test was timed alone on the four-core Linux container, warm build: 79 tests, about
2.5 hours of test time end to end, the nightly's slowest shard 47 minutes for one test.  What
it spent, and what changed:

| test | before | after | what changed |
|---|---:|---:|---|
| `numeric_parts::the_parts_read_the_large_cases_as_the_whole_does` | 47 min on CI | 12.3 min | the dense SVD and pivoted QR walk memory by rows |
| `object_builder::every_object_of_the_library_builds_as_the_export_builds_it` | 23.5 min | 4.9 min | one solve a document, documents side by side, named sweeps left to their tests |
| `hulls::the_librarys_rod_is_sized_by_its_sag` | 4.8 min | 2.4 min | a filled body's harmonic maps made once a mesh |

- **The engine's whole SVD** (4,859 columns) is what the large cases' test waits on.
  `linalg::gr_svd` and `qrp` walked the row-major matrix down its columns, a cache line an
  element; they now keep V transposed (a rotation of two of V's columns runs along two rows)
  and turn the column-by-column loops round so each sum is taken a row at a time, its terms in the
  order they always were.  The factors are the old ones to the bit (checked on 64 matrices,
  rank-deficient, wide and tall, against the old code); at n = 1000 the SVD takes 1.5 s where it
  took 3.9, the pivoted QR 0.36 where it took 1.0, the V-twin's whole diagnosis 2.7 s where it
  took 14.8.  Dealing the same loops across cores gained nothing on this machine (they are bound
  by memory, not arithmetic), so they are not dealt.  The engine still takes two SVDs of about
  four minutes each.
- **The library's rod** (`sized_conrod.sv`, solved by four slow tests and the CLI's) spent 70% of
  its solve factoring its mesh's three Laplacians again at every evaluation, though they read the
  mesh as it was meshed and none of the drawing (`docs/contract/analysis.md`, `hull::Made`).
- **`object_builder`'s library test** solved every document twice (once for its static objects,
  once for its swept), the rod's 276 s solve among them, and built `stopped_bullnose.sv`'s sweep
  that `a_turned_bull_noses_notch_builds_at_the_displays_samples` builds already (287 s).
- Six tests of three seconds or less alone came back to the fast tier: `admission::`'s three,
  `homotopy::the_k33_core_has_several_real_realizations`, `rounds::a_bosss_root_is_a_ring` and
  `object_builder::exact_solid_builds_a_generated_body_as_the_export_does`.
- `rust/test-times.tsv` now holds the slow tier's times, which it lacked: every slow test weighed
  the 60 s `slow.yml` gives a missing one, so the large cases and the library's objects fell on
  one machine.

## The second pass: cheaper oracles (November 4, 2026, #780 R1)

Most of what the slow tier spent bought no coverage: a test waited on a slow *reference* (the
whole SVD the parts were compared with, the field's volume a round was compared with) or built a
large instance that walks the paths a small one does.  So before a slow test is made smaller, it is
asked what it buys, and the answer held more cheaply:

- **Check the answer instead of computing it again.**  `numeric_parts::` holds the parts' reading
  of the engine (4,859 columns) to the Jacobian itself — every motion and dependency the parts
  read sent to zero by it, independent, the rank read forward over the columns the same as
  backward over the rows — where the whole SVD took minutes
  (`the_parts_reading_of_every_large_case_holds_on_its_jacobian`); the whole SVD is the oracle
  up to a thousand columns, every push.
- **Instances whose answer is known by construction.**  Henneberg's frameworks past the dense
  limit are rigid as built, so their rank, freedoms and dependencies are known
  (`numeric_parts::a_framework_past_the_dense_limit_reads_as_it_was_built`).
- **An exact oracle and a pointwise one, for a sampled one.**  A round's volume was the field's,
  integrated by an octree to 2e-3 (minutes a round, the slot's eight).  It is now the kernel's
  exact build with and without the round, kept in the test's box, against the closed form (1e-8
  or better on every example), and the field's boundary is held to the kernel's point by point, a
  step either side of its triangles, by the kernel's own winding number
  (`common::rounds::field_is_the_kernels`).  Stronger, since a volume can be right with two
  errors cancelling and a point cannot; and it found a set of no volume where the field reads
  exactly nothing inside the material, where two operands' faces meet in a plane.
- **A refusal is tested with a small budget.**  The ball-end mill's gate ran `closed::BUDGET`
  down; given a thousandth of the inverse stage's, every other stage its whole, it is refused in
  the same stage by name in seconds.  This found that a stopped build still polished every
  crossing (`meeting::crossings` now stops): the default budget's refusal came down from 15–23
  minutes to three.
- **A corpus chosen by what decides the paths.**  `object_builder` holds the builder to the
  export on 11 documents holding every kind of object the library builds (recipe forms, region
  terms, sweep classes and motions: `kinds`), and the slow tier reads every document's kinds,
  building nothing, to say when a new kind is held by none of them.
- **One build for every property of it.**  The helical groove's and the bull-nose's stepped
  builds are held to the build their examples already make.
- **A property held on a small body as on the large one.**  The rod's work counts and its row held
  at a reading's own size are held on the turned bar every push.

| tests | before | after |
|---|---:|---:|
| `numeric_parts::` (large cases, frameworks, certificate) | 12.3 min, slow | 12 s, fast |
| `rounds_examples::`'s seven `_are_the_stacks` | 15 min, slow | 20 s, fast |
| `rounds::`'s three, `conrod::the_fields_rounds_are_the_kernels`, `conrod::the_rod_is_built_exactly` | 7 min, slow | 13 s, fast |
| `object_builder::` the library | 3–5 min, slow | 3 s fast, 30 s slow |
| `swept_closed::` the ball-end mill | 15–23 min, slow | 8 s fast, 3 min slow |

What was not taken: the leaning block's part turn at a tenth of the tolerance is #735's
regression gate and reaches its own paths (the rounding constant at a ring's centre, collapsed
pieces), so it stays.  `hypoid_drive`'s claims share their cost — nearly all of it
`clear::measure_pair`'s nearest-pair search between two meshes, one thread, which judging fewer
claims barely shortens — so they all stay; that search is where to look.  `composed_rounds`'s
boss radii from 21 to 21.7 are the contract's gate (`rounds.md`).

What is left, the longest first, and where its time goes:

- `hypoid_drive::every_claim_the_drive_makes_holds`, 8 min: `clear::measure_pair`, above.
- The closed sweeps (`swept_closed::`, `solid_sweeps_examples::`), 2 to 7 minutes each: sampled,
  the bull-nose's spends two thirds of its build in `meeting::crossings`' polish, every pair of
  near chords walked to its crossing by up to sixty global `Curve::inverse`s, most of them to a
  crossing already found (#780 R4), and the page's stepped builder of it as long again.
- `swept_closed::the_ball_end_mill_screwed_through_a_ring_sector_is_cut_right_or_refused_by_name`,
  3 min: its default budget run down (#780 C1 builds it, below).
- `hulls::the_librarys_rod_is_sized_by_its_sag`, 2.4 min: the rod's finite elements.
- `fillet_definition::`, about two minutes together: the three candidate definitions' volumes.

## The slow tier made green (November 5, 2026, #780 R1)

The nightly had failed every night since it was first scheduled, on three tests outside the
sweeps, so a new failure there went unseen.  Each was a defect, not a stale tolerance, and each is
fixed where it lay:

| test | what it met | fixed |
|---|---|---|
| `composed_rounds::a_chamfer_spilled_onto_a_rounded_plate_meshes_at_every_radius` | at a boss of 21 the chamfer's foot crosses the plate's tangent seam where the boss's base lies on the plate's top; past it the field named the base, facing down, so the seam was a corner one way and a hand-off the other, and the crease ran twice over the round and ended 3 µm from its start | a followed leaf's own next piece continues a crease (`crease::trace`); held every push by `…_has_one_closed_crease_over_the_seam`, 0.2 s |
| `gear_crowns::the_fabricated_pair_is_the_drawn_sections_export` | its record is of the pair cut by the mill the drawn sections were cut by; since #283 the pair finds another mill, and another pair (its pinion 147 faces, not 243) | cut by `fixtures::gear::recorded_machine` at 12.5°, as every other suite reads it; agrees to 2e-8 |
| `hulls::the_librarys_rod_is_sized_by_its_sag` | its drive was never refined past twice a reading's size: the mesh at its own size held three corners of an element 10⁻¹⁵ apart, a surface centre inserted on a vertex already there off its facet's dual edge (a nearly co-circular flat tetrahedron's orthocentre, which rounding puts anywhere on its axis) | refinement inserts no surface centre within its margin of a vertex (`Refiner::crowded`); the rod solves at four, two and one times its size, the web 6.59 mm |

Measured on the four-core container, warm build, after the fixes:

- **The core's suite with the slow tier**, dealt into slow.yml's eight shards and run one after
  another on one machine: 3,498 passed, one failed, 35.5 minutes (the longest shard 11 minutes).
  `rust/test-times.tsv` now holds those times.  Run whole on four threads, the suite was killed
  for memory a few minutes in; a shard at a time it was not.
- **The CLI's suite with the slow tier**: 50 passed, 174 s.
- **`swept_closed::`'s slow tier** (the screws and the tori), each alone: 939 s, against 741 s
  before #777.  The bull-nose turned about an axis along its own is 595 s of it (its row in
  `rust/test-times.tsv` said 208 s); the ball-end mill's refusal at the default budget 104 s.
- **The one failure is new**:
  `prism_twists::a_corner_reaching_a_cap_under_a_screw_is_the_closed_classs` passed the night of
  #779 and fails on `main` since #782, whose `square_thread.sv` states no roll.
  The march now ends at the first whole degree that is clear of the rod, the bar's flank 1/60 mm
  past the rod's end plane, where the stated roll ended 1 mm past it, and the closed arrangement
  refuses it ("a plane's pieces do not close").  With the end 30 steps further out it builds; 1
  or 3 steps do not.  It is the kernel's: a sweep whose end pose stands just clear of a face of
  the blank is a pose any stated roll may take.

## The ball-end mill built (November 5, 2026, #780 C1)

The issue read the mill's refusal as its pipe's inverse map, a general search on every ray and
outline; the profile said otherwise.  Under the default budget, before (`SOLVENT_SWEEP_PROFILE=1`,
the four-core container):

| stage | steps | of |
|---|---|---|
| arrangement | 108 | 400,000 |
| trace | 414 | 4,000,000 |
| ray | 23,587 | 2,000,000,000 |
| outline | 10,106 | 20,000,000 |
| inverse | 201,385,420 | 200,000,000 (refused, 169 s) |
| membership | 983,073 | 400,000,000 |

With the pipe inverted off its spine alone the inverse stage still ran its whole budget down, and
slower (476 s): its *calls* ran away, not their cost.  A sampled backtrace put them in one
`meeting::crossings`: a short arc (0.046 rad) of a circle beside a traced meeting of the pipe and a
plane, each sampled at 512 points against a reach of 0.08, so every pair of chords within it was
near — 26,000 pairs, each polished by seven rounds of global `Curve::inverse`s on the trace, each
point of which projects onto the pipe, and no crossing among them.  Polished only from the pairs
no neighbour stands closer than, the build went past its arrangement in 13 s; two failures behind
the budget then showed, each dumped, reduced (`swept_closed::the_ball_end_mills_pipe_beside_its_side_is_arranged`)
and fixed where it lay.  After, built in about ten seconds:

| stage | steps |
|---|---|
| arrangement | 108 |
| trace | 429 |
| ray | 209,768 |
| outline | 25,455 |
| inverse | 29,626,186 (each stretch of a pipe's foot isolated, now the inverse stage's) |
| membership | 988,209 |
| judge | 533 |

A faster pipe inverse, certified on a window about the last foot found (Newton from the warm
start, `δ = π L / 2 s₁` by the helix's chords, `g` monotone across it), agreed with the full
isolation to 1e-13 on every call and was 2.5 times quicker, yet flipped both builds to "pieces do
not close": the mill's ball and side meet the plane of each end pose's foot at points where three
surfaces touch, and at rounding the arrangement there still reads either way.  It is left out
until those points are read alike whatever the rounding (#780 R4's neighbourhood).

- `swept_closed::the_ball_end_mill_screwed_through_a_ring_sector_is_cut_right`, about 25 s alone
  (slow tier): the build and its Monte Carlo of 100,000 points.
- `swept_closed::the_ball_end_mill_screwed_closes_into_its_helical_sweep`, about 22 s (slow tier).
- `swept_closed::the_ball_end_mill_screwed_through_a_sector_reduced_to_its_boolean_holds_every_cell`,
  15.4 s before and 4.4 s after: its section's polar integral read across the ring to rounding
  (`helix_reach`, held to 1e-6 where the grid of 1,440,000 points held 1e-4), and the pipe's inverse
  reading each point of the spine once per stretch.

Drawn at 3 mm a turn (a lead other than the gate's) the mill is not yet built: reduced, the pipe
beside the cylinder its side screws into, within the sector; the pipe's meeting with that
cylinder's foot plane sets out a millionth past the pipe's start, where it, the foot's rim and the
touch helix touch at one point, and the foot's pieces do not close.  The touch helix itself is now
placed where `g'` changes sign (it stood 2.6e-7 off at the run's middle); the trace's start is what
is left.

The slow tier of `swept_closed::` after: 42 of 43 in 291 s (939 s before).  The one failure,
`a_bull_nose_turned_about_an_axis_along_its_own_closes_into_its_sweep`, fails on `main` before
#780 C1 too, the same way (`Unbuilt::Wrong` at the same witness, 1,072 s there); now in 14 s, the
crossings' polish no longer walking every near pair.
