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
`block_measure` reads — the 90 example documents that elaborate and the twelve recorded
spiral-bevel designs, 102 in all (the three parameter-only modules skipped) — and starts each from
its own seeds and from `block_measure::robustness`'s five starts (the default solution halved and
doubled about its centroid, and jittered three times by up to 1e-3 of the extent), 597 starts.
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
attempt and only with `retry`, a DogLeg solve, `BlockMode::Rescue` and two blocks or more: a
result that succeeded without settling runs the block pass and polish (`blocks_then_polish`)

1. from the stop, kept if it converges (settled with status 0);
2. otherwise from the start, kept if it settles;
3. otherwise the stop's pass, kept if it settled (a stall, but not a limit stop);
4. otherwise nothing is kept: the first result's pose, status, success and counts, bit for bit.

A failed first result is rescued exactly as before (the block pass from the start, kept if it
succeeds, then LM, kept if better). `BlockMode::First` and `Off`, and every solve with `retry` off
— both halves of a drag — are untouched.

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
orientation. `hypoid_layout::a_stop_on_the_iteration_limit_is_rescued_onto_the_recorded_pair` —
24×48 at module 25.4 solved and jittered three times stops on the limit up to 8.1e-4 off the
recorded pair (relative to the cone distance) without the rescue, and settles on it to 8.1e-11 with
it. `hypoid_layout::a_stop_that_stalls_again_is_restarted_in_block_order` — one of the sweep's
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
  as unsettled needs its own byte gate.

## Files

- `rust/gcs-core/src/solve.rs`: `SolveResult::settled`, the trigger in `solve_compiled`.
- `rust/gcs-core/tests/limit_measure.rs`: the measurement (phase 1 and the `default` column).
- `rust/gcs-core/tests/block_solve.rs`, `hypoid_layout.rs`, `drag.rs`: the tests above;
  `block_measure.rs` shares its corpus and starts.
