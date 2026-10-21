# A solve that ran out of iterations is not a solve: plan and record

All three phases are done. A one-shot solve whose DogLeg succeeded on its iteration limit is now
rescued by the block pass — from where it stopped, then from the start — and keeps a rescue only
if it *settles*; a stop no rescue settles keeps its pose, status and success bit for bit.

## The problem

`System::solve` calls a pose a success when the trust-region loop stopped with a non-negative
status and every hard row is under `acceptance_tol` (solve.rs, `success: info.status >= 0 &&
rel < opts.acceptance_tol`). The statuses are `newton::status_message`'s: 0 the residual
tolerance was reached, 1 the step fell below xtol, 2 the gradient below gtol, 3 the trust region
collapsed, 4 the iteration limit was reached. So a DogLeg that is still descending when its 100
iterations run out, at a residual of 1e-7 against the interactive 1e-6, was a success — and the
rescues that follow a failure (the block-triangular pass, then LM from the start) never ran.

The block solve's measurements found this to be the commonest failure from rough seeds: a pose
accepted this way can sit up to 1e-3 of the extent from the solution, because a small residual
in an ill-conditioned direction is a large distance. Crown previews of the spiral bevel drew
nearly vanished tooth tips like this until the preview-convergence gate caught it, and the
480-design sweep had seven designs that stalled this way at 24×48, module 2.

Drags are not affected by any of this and must stay so: `PullPolish` runs with `retry: false`
and deliberately short iteration caps (4 and 20), where stopping at the cap is the point.

## The plan

In `solve_compiled`, a result is **settled** when it succeeded *and* it stopped for a reason
other than the iteration limit (status 0–3). The rescues run when the first result is not
settled, rather than only when it failed: a status-4 success gets a rescue (the block pass, or a
resumed DogLeg, chosen by measurement), kept only if it settles; a failure is rescued as before.
Only `opts.retry` solves (the one-shot solve: solventc, the drawing compiler, fixtures, the FFI,
the app's solve, `PlanSolver`'s fallback) take the new path. Status 3 was to be left alone unless
the measurement showed it mattered.

## Phase 1: measured

`tests/limit_measure.rs` (ignored; `cargo test --manifest-path rust/Cargo.toml -p gcs-core --test
core limit_measure::limit_stops -- --ignored --nocapture`, about 70 s) reads the corpus
`block_measure` reads (`common::corpus`) — the 90 example documents that elaborate and the twelve
recorded spiral-bevel designs, 102 in all (the three parameter-only modules skipped) — and starts
each from its own seeds and from the five rough starts `block_measure::robustness` solves
(`common::rough_starts`: the default solution halved and doubled about its centroid, and jittered
three times by up to 1e-3 of the extent), 597 starts.
Each is solved by the whole-system DogLeg alone, the first attempt of every one-shot solve.

| start | starts | failed | status 0 | 1 | 2 | 3 | 4 |
|---|---:|---:|---:|---:|---:|---:|---:|
| own seeds | 102 | 11 | 80 | 11 | 0 | 0 | **0** |
| halved | 99 | 11 | 63 | 15 | 1 | 0 | 9 |
| doubled | 99 | 21 | 67 | 10 | 0 | 0 | 1 |
| jittered | 297 | 0 | 249 | 28 | 4 | 0 | 16 |

(The failures are the DogLeg's alone; the default solve's rescues bring the own-seed failures to 3.)

* **No document's own-seed solve stops on the limit**, so the change was predicted to leave every
  corpus report, sheet and export as it was — which the byte gate then confirmed.
* **Status 3 never ends in a success** (0 of 597), so it is left alone.
* **26 starts stop on the limit under the acceptance**: 25 on the spiral bevel (the crown's
  `reach` and `space` modules, `gears`/`layout`/`members`/`pair`, the twelve designs) and
  `truss200` doubled. Every one is more than 1e-6 of the extent from a tight solve continued from
  it (`tol` 1e-16, acceptance 1e-12, 5000 iterations): 1.8e-5 to 8.7e-4 on the spiral bevel, 8.3e-2
  on the truss. The tight continuation itself fails from 10 of them (the halved designs and the
  crown modules), which is the first sign that some stops sit in a basin with no solution.

What each candidate rescue makes of the 26 (settles = succeeds on a status other than 4; "on the
reference" = within 1e-6 of the extent of the document's own-seed solution; every document here
has no freedom left, so that solution is the reference root):

| rescue | settles | on the reference | cost (all 26) |
|---|---:|---:|---:|
| one more DogLeg budget from the stop | 6 | 5 | 830–1280 ms |
| the block pass from the start (what the failure rescue runs) | 24 | 20 | 170–250 ms |
| the block pass from the stop | 19 | 14 | 330–490 ms |

The resumed DogLeg settles only the six that every other candidate settles too, at several times
the cost: the stops are crawls in ill-conditioned directions, and another hundred iterations crawl
on. It was not chosen.

The two block passes differ in *which root* they finish on, and neither is right alone:

* **From the start it changes roots.** A chain of six, twelve or twenty-four triangles seeded far
  off and given a DogLeg budget a few iterations short (the `block_solve.rs` fixture) stops on the
  limit within 1.4e-4 units of the solution the full budget reaches; the pass from the start, each
  triangle solved alone from its seeds, settles in 9 of the 16 stops tried, every time on the
  *other* orientation of some triangles, 17 to 61 units away on sides of 10. That is the
  block plan's finding (phase 4, `First` changes four documents' roots) reached through the
  rescue. From the stop, the pass converged on the full solve's pose in every case tried.
* **From the stop it can finish a stall.** Where the stop is in a basin with no solution in it —
  the seven designs of the sweep, whose crown section's narrow tip collapses — the pass from the
  stop "settles" by stalling again, on a step stop at a residual of 5e-11 to 2e-8, a thousandth
  of the extent from the block path's solution, which the pass from the start reaches (the
  crown modules' and the halved designs' stops are the same kind: the tight continuation from
  them fails).

The one thing that tells the two apart is how the pass from the stop ends: converged to the
requested tolerance (status 0), it has finished the root the DogLeg was making for; stopped by a
vanishing step or gradient above the tolerance (1 or 2), it has stalled in the same basin. So the
rescue is both passes, in that order.

## Phase 2: the trigger

`SolveResult::settled()` is `success && status != 4`. In `solve_compiled`, after the first
attempt and only with `retry`, a DogLeg solve, `BlockMode::Rescue` and two blocks or more, a
result that did not settle goes to `System::block_rescue`, which tries the block pass and polish
(`blocks_then_polish`) from one start after another. One that succeeded without settling is tried

1. from the stop, kept if it converges (settled with status 0);
2. otherwise from the start, kept if it settles;
3. otherwise the stop's pass, kept if it settled (a stall, but not a limit stop);
4. otherwise nothing is kept: the first result's pose, status, success and counts, bit for bit.

A failed first result is rescued exactly as before: the block pass from the start (the same
helper, with that one start), kept if it succeeds, then LM, kept if better. `BlockMode::First`
and `Off`, and every solve with `retry` off — both halves of a drag — are untouched.

Measured again with the rule in place (`limit_stops`'s `default` column), the 26 stops come out:

* **20 settle on the reference root**, including all 16 jittered starts (each had stopped 2.4e-5
  to 1.7e-3 of the extent off it) and four of the halved designs and crown modules;
* **4 settle on another root**, each from a start whose stop was already off the reference (by
  3e-3 to 0.9 of the extent): `truss200` doubled (the stop 0.88 of the extent off, the rescue 0.65),
  hypoid6 halved (3.2e-3, 2.2e-3), 28×49 at modules 0.2 (4.7e-3 both) and 2 (3.5e-3, 1.9e-2).
  The truss is the one case where the pass from the stop converged but ended on the gradient
  tolerance at 1.5e-14 rather than the residual tolerance, so the restart ran and was kept;
* **2 keep their stop exactly**: the bevel pair and 24×48 at module 2, halved, which neither pass
  settles.

**Tests.** `block_solve::a_stop_on_the_iteration_limit_is_finished_where_it_stopped` — the
triangle chain given eight iterations stops on the limit, is settled by the pass from the stop on
the full solve's pose to 1e-12 of the extent, and the pass from the start alone lands on another
orientation. `hypoid_layout::a_stop_on_the_iteration_limit_is_rescued` — 24×48 at module 25.4
solved and jittered three times stopped on the limit up to 8.1e-4 off the recorded pair (relative
to the cone distance) without the rescue, and settled on it to 8.1e-11 with it. *Since #145's
layout changes (2026-10),* the three hypoid tests find their own start (`first_start`): each
walks fixed jitters and seeds, checks the rescue's promise at every stop it meets (settled, the
stop bit for bit, or a bound broken after it) and ends at the first start its scenario holds at;
the limit stop's claim is now that the block pass settles it, whichever root (which root is the
rough starts' gate). `hypoid_layout::a_stop_that_stalls_again_is_restarted_in_block_order` — one of the sweep's
seven: the pass from the stop stalls (status 1, 6e-10), and the default solve lands 2.5e-15 of the
extent from the block path held to 1e-12. `hypoid_layout::a_stop_the_rescue_cannot_settle_keeps_
its_pose` — the halved bevel pair comes out of the default solve with the same bits, status,
success, residual, counts and method as without the rescue. `drag::pull_polish_frames_are_the_
bits_they_were` — every frame of a `Drag` on three figures (a filleted rectangle, a floating
truss, the traced gear), pose, status and success, hashed against the value recorded before the
change. No existing test pinned a status-4 pose; none was re-recorded.

**Gates.**

* *Byte gate against `main`'s behaviour* (the branch's pre-change `solventc` with OCCT, built
  first, against the change): all 137 corpus outputs (92 `--json` reports, 45 `.svd` sheets; the
  spiral-bevel reports that time out at 60 s six at a time re-run alone, and
  `spiral_bevel/blank/member.sv`, over 900 s either way, compared by neither) and all 44 fast
  golden exports (31 files, every log's last line and exit code) are **identical**. As phase 1
  predicted, no document's own-seed solve stopped on the limit, so none differs.
* *Design sweep* (`hypoid_layout::design_sweep`, 480 designs): **13 → 6** designs where the
  default solve does not converge to the block path's pose. The seven 24×48 module-2 stalls are
  gone (settled by the restart from the start, within 1e-8 of the block path). The six left are
  not stops of the first DogLeg: two where the default solve is right and the block path held to
  1e-12 is the one that fails (13×40, module 2, E 30), and four (13×40, module 25.4, E 381) where
  the first DogLeg *fails*, and the failure rescue's polish then succeeds on its own iteration
  limit at 4.9e-8, 0.91 of the extent from where `First` gives up — the failure path, which this
  change keeps as it was.
* The full suite with OCCT (1345 passed, no warnings), the slow tier with OCCT (1352 passed) and
  the wasm build with the web suite (255 passed): green. `every_modules_preview_converges_as_
  solventc_solves_it` and the rough-seed gate pass unchanged.

**Time.** A document whose solve does not stop on the limit pays one comparison. Measured with
`block_measure::timing` (medians of seven solves from each document's own seeds; the pre-change
build from a worktree of the plan's commit and the change run alternately three times, on a
loaded machine), the whole corpus under `Rescue` took 548, 583 and 590 ms before and 568, 571 and
582 ms after, with every document solved by the same path. A limit stop pays its rescue: on the
26 corpus stops the pass from the stop took 2–4 ms where it converged and 20–90 ms where it did
not, and the restart from the start 1.3–21 ms where it settled and 45–60 ms where it did not; a
stop's whole default solve took 18–130 ms, against 20–90 ms for a DogLeg budget of its own. A
status-4 success is by definition a slow solve already.

## Phase 3: recorded

This document, and the solver conventions in CLAUDE.md (what "settled" means, and the rescue's
order).

## Follow-ups

- **The failure path's own limit stops.** A failed DogLeg's block rescue is kept when it
  *succeeds*, and its polish can succeed on the iteration limit (the sweep's four 13×40 module
  25.4 designs, 4.9e-8). Holding that rescue to "settled" too, and trying LM when it is not, is the
  same change on the other path and was left out of this one.
- **Status 1 and 2 above the tolerance.** A step or gradient stop at 1e-9 is a stall as surely as
  a limit stop is, but the corpus's own-seed solves end on status 1 eleven times, so treating it
  as unsettled needs its own byte gate.  (#492 found the far-seeded stops it was suspected of were
  the extent's measure, below; no corpus status-1/2 success is rejected by the rows' own sizes.)

## #492: a pose already far out, judged at its rows' own sizes

#489 kept a recovery only where it settles, which closed #449 for the path it reported: a
recompile at a runaway's extent.  The same yardstick stretched wherever the pose was *already*
far out when the system was compiled.  The solve's acceptance, the diagnosis's violated rows, a
claim's test and a witness block's all measured each row over `extent^degree`, and one point
seeded far out makes the extent what it likes.  #449's belt (`unsettled::belt`) with `x` seeded
far out, `solventc --json` before and after (status/iterations; `x`; the claim's verdict):

| r₂ | seed | before | after |
|---:|---:|---|---|
| 20 | −96 | did not solve 4/100 | did not solve 4/100 |
| 20 | −1e5 | stopped at the limit 4/100 | did not solve 4/100 |
| 20 | −1e6 | **solved** 2/1, x −1e6, **theorem** | did not solve 2/1, no claim judged |
| 20 | −1e7 | **solved** 0/1, x −1e7, consuming | did not solve 0/1, no claim judged |
| 20.001 | −96, −1e5 | solved, x −2.4e6 | solved, x −2.4e6 |
| 20.001 | −1e6 | **solved** 2/1, x **−1e6**, theorem | solved 0/0, x −2399999.99996 |
| 20.001 | −1e7 | **solved** 0/1, x **−1e7** | solved 0/0, x −2399999.99999 |
| 45 | −96, −1e5 | solved, x −96 | solved, x −96 |
| 45 | −1e6 | **solved** 2/4, x **−1e6**, theorem | solved 0/0, x −96.0000000002 |
| 45 | −1e7 | **solved** 2/2, x **−1e7** | solved 0/0, x −96.00000000000004 |

Before, the equal-radius belt seeded at −1e6 was *solved* in one iteration with its claim a
theorem: the DogLeg stopped on its gradient (`1e-16 / extent`, 1e-22) with its contacts 1.2 µm
off their 20 mm circles, a squared length of 0.05 mm² read over 1e12 mm² as 5e-14.  At −1e7 it
stopped on its residual at once (5e-17).  Every belt seeded at −1e6 or −1e7 was "solved" where
it was seeded — the r₂ = 45 one, whose runs meet at −96, with its contacts 0.67 of their own size
off.  (The unequal belt's claim at −2.4e6 reads *consuming* before and after: its rank is judged
over the extent, 2.4e6, which this change keeps.)

### The candidates, measured

* **Status 1 and 2 above the tolerance (item 2) alone** cannot be the fix: the −1e7 stop is a
  status 0, the residual tolerance reached over the stretched extent.
* **A robust extent** (the drawing's size, its far-out points left out) does not reach −1e7
  either.  Parallel runs tilted to meet 1e7 out miss their circles by `1200/|x|` mm = 0.12 µm,
  1e-6 of the 120 mm drawing: over 120² the contact reads 3.3e-7, under the acceptance.  The same
  holds per *block* (each strongly connected block over its own extent): the runs' blocks are
  120 across.  A point-cloud estimate of "far out" is fooled by the corpus besides: tried as the
  outermost 100× gap in the points sorted by distance from the median, it called the m25.4
  spiral-bevel designs (26 unit-scale points within 0.76 of the middle, 16 real ones out to 950)
  and the jittered `catenary.sv` (six points at the origin) 1 mm across.
* **Per row** — each row over the size of the geometry it reads — is what tells −1e7 apart: the
  contact reads 1.2e-5 over its 20 mm circle.  Taken, with three findings of the measurement:
  * *A row reading the far point itself*: `x` on a run, `(x − s) × (e − s)`, is `|x|` times the
    run's 120 mm, and over `|x|²` a run tilted 20 mm off `x` reads 2e-9.  Solving on at the rows'
    sizes (below) found exactly that pose and accepted the equal belt at −1e6.  So a point standing
    more than `FAR_OUT` (100) times the size of the rest a row reads outside their box is left out
    of the row's size: `x` on a run is judged at the run's 120 mm.
  * *Rounding*: a row is known no better than the numbers it reads, `ε·m^d` for numbers `m` off the
    origin whatever its size, and the fixtures solve to 1e-12 (`fixtures::read::accurate`).  Judged
    at a unit, the m25.4 bevel's unit-scale rows among numbers in the hundreds read 1e-10 and
    `envelope::paired`, `hypoid_layout` and `region_blank` failed to solve.  So a row's size is at
    least a tenth of the largest number it reads (`OFF_ORIGIN`): `ε·10²`, 2e-14.
  * *Where the size is read*: read only where the system was compiled, a drawing seeded in a heap
    (the bevel's unsolved points) is judged at sizes the solution does not have; read only where
    it is judged, a drawing that shrinks as it solves (`hub_motor`'s figures: two points seeded 60
    apart and held 0.43 apart) is judged at a size the iteration, stepping over the extent it was
    compiled at, never saw, and its 1e-12 solve failed.  The larger of the two is taken.

### The rule

`system::Sizes`: a row's size is the longest side of the box round the points it reads, a point
far out of the rest left out, or its largest radius; at least a tenth of the largest number it
reads and a unit, at most `max(1, extent)` — so no row is judged more loosely than before — and
the larger of its size where the system was compiled and where it is judged.  A row reading no
point and no radius is judged at the extent.  `max_relative_residual` (so the solve's success, the
REFRAMES test, the plan solver, the boundary solve, the witness), `constraint_errors` (the violated
rows), a claim's test and a witness block's read it; the rank's conditioning keeps the extent.

**The iteration keeps the extent.**  Stepping on the rows' sizes everywhere would change every
document's bits and every drag frame's.  Instead a *failed* document solve whose every row holds
over the extent, and not over its own size, is solved on from where it stands with `row_scale` at
the rows' own sizes (`System::scale_rows`, `System::at_own_sizes`) — the extent's gradient and
residual tests had stopped it without seeing the rows it left — kept where that settles nearer to
holding, no more than `RESIZE` times as wide as it began (a point run off to infinity reads as
holding at any measure: parallel runs meet within 1e-14 of a 20 mm circle some 1e16 out), and put
back over the extent either way.  That is what solves the unequal belts from far seeds at their
crossing, and r₂ = 45 from −1e7 at −96.  Without it, judged at their own sizes alone, they failed,
and the diagnosis — its deletion filter's trial solves stopping the same way — named a *conflict*
in a drawing that has a solution.

**Not needed: item 2.**  Status 1 and 2 stops are left settled: with the measure fixed, no corpus
start, own seeds or rough, is a status-1/2 success its rows' sizes reject.

**Item 3, the conflict**: the equal belt now names one (`[27, 28, 19, 20, 23, 24, 17, 18]` from
−96, six or nine of the ten statements from the far seeds), each "minimal among the suspects" as
the deletion filter defines it; that its trial solves stop as the main solve does from a far pose
is why it differs by seed.  Not pursued further.

### Gates

* **Corpus answers** (`size_measure::own_seeds`, `rough_starts_of_the_corpus`: 198 documents from
  their own seeds, 995 rough starts): no solve the two measures judge differently.  The largest
  row read over its own size among the own-seed answers is 3.1e-10 (`meshing_problem`, a limit
  stop), among solved ones 6.1e-11 (`turbine_blade`); among the rough starts 8.9e-9 (`truss200`
  halved; `spur_mesh` halved reads 0.18 by both measures in a fresh compile, as before).
* **`solventc --json` over all 187 `rust/examples/**/*.sv`** against `main`'s build (formate.sv
  alone with a long timeout): every one of the 180 solves has the same `success`, `settled`,
  `status` and `outcome`.  `maxResidual` changes in 77 — it is the new measure, 1.01 to 2.8e4 times
  the old, the largest of a solved document 5.4e-11 (`turbine_blade`).  The one other difference is
  `turbine_blade.sv`'s nine solid areas and volumes, in the 16th digit (3137.0256450210295 →
  …304): three station solves of its face-family march fail under both measures, and LM's retry
  pose is now kept as the better by the rows' own sizes (14.99999999994 against 15.0) where the
  extent's measure kept the DogLeg's (0.0531914893637 against 0.0531914893617); the march starts
  its next station from it.  Every other report is byte-identical.
* **`bench boundaries`, the radii swept equal** (each solve from the last's pose):

  | r₂ | before: flat, through | after: flat, through |
  |---:|---|---|
  | 45 … 20.01 | ok, ok | ok, ok (flat 1e-13 or less of the extent off its crossing) |
  | 20.001 | ok at −4.6e5 (apart 4e0), ok | ok at −2.395e6 (2e-3 off; apart 2e-3), ok |
  | 20.0001 | ok (apart 3e1), ok | FAIL, FAIL |
  | 20 | ok (apart 6e1), ok | FAIL, FAIL |

  Not met at r₂ = 20.001: the flat solve holds every row to 2.1e-7 of its own size there (its
  contacts 2 nm off their circles) and stands 2e-3 of the extent off the crossing, whose place
  moves 2.4e9 times what a contact misses its circle by.  Solved on at the rows' own sizes it
  stalls at 4.5e-8 (a step or trust-region stop), the same pose.  No residual measure tells that
  pose from the crossing; a position criterion would need the conditioning.  `far_seed.rs` holds
  the rest of the table.
* `tests/far_seed.rs` is the gate (the belt from −1e6 and −1e7, the micrometre belt at its
  crossing, r₂ = 45 from −1e7 at −96, the rows' sizes, the sweep).  `tests/unsettled.rs` and
  #449's controls pass unchanged.  `block_solve`'s two limit-stop tests moved their budgets: the
  chain's eighth iteration stops 1.6e-6 of the extent off with its rows 3.9e-6 of their sizes off,
  no success now, so the ninth is the status-4 success the rescue settles; the belt's kept stop is
  found among 64 seeds at four iterations rather than 32 at five, where every success the rescue
  meets it settles.  The full suite (2683 passed), and the wasm build with the web suite (318
  passed): green.

### Follow-ups

- **A position criterion** for ill-conditioned drawings (the bench at r₂ = 20.001): every row
  holds to its tolerance and the crossing is metres off.
- **The rank at a far pose**: `conditioned` still divides by the extent, so the micrometre belt's
  claim at −2.4e6 reads *consuming* where at −96 the same claim at r₂ = 45 is a theorem.
- **Conflict naming from a far pose** (above).

## Files

- `rust/gcs-solve/src/solve.rs`: `SolveResult::settled`, the trigger in `solve_compiled` and the
  rule in `System::block_rescue`.
- `rust/gcs-core/tests/limit_measure.rs`: the measurement (phase 1 and the `default` column).
- `rust/gcs-core/tests/block_solve.rs`, `hypoid_layout.rs`, `drag.rs`: the tests above.
- `rust/gcs-core/tests/common/mod.rs`: the corpus, the rough starts and the distances that
  `limit_measure`, `block_measure` and the tests above share.
- `rust/gcs-solve/src/system.rs`: `Sizes`, `FAR_OUT`, `OFF_ORIGIN`, `System::size_rows`,
  `scale_rows`, `judged` (#492); `solve.rs`: `System::at_own_sizes`.
- `rust/gcs-core/tests/far_seed.rs` (the gate) and `size_measure.rs` (the measurement), #492.
