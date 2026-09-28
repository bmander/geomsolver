# A solve that ran out of iterations is not a solve: plan

## The problem

`System::solve` calls a pose a success when the trust-region loop stopped with a non-negative
status and every hard row is under `acceptance_tol` (solve.rs, `success: info.status >= 0 &&
rel < opts.acceptance_tol`). The statuses are `newton::status_message`'s: 0 the residual
tolerance was reached, 1 the step fell below xtol, 2 the gradient below gtol, 3 the trust region
collapsed, 4 the iteration limit was reached. So a DogLeg that is still descending when its 100
iterations run out, at a residual of 1e-7 against the interactive 1e-6, is a success — and the
rescues that follow a failure (the block-triangular pass, then LM from the start) never run.

The block solve's measurements found this to be the commonest failure from rough seeds: a pose
accepted this way can sit up to 1e-3 of the extent from the solution, because a small residual
in an ill-conditioned direction is a large distance. Crown previews of the spiral bevel drew
nearly vanished tooth tips like this until the preview-convergence gate caught it, and the
480-design sweep still has seven designs that stall this way at 24×48, module 2.

Drags are not affected by any of this and must stay so: `PullPolish` runs with `retry: false`
and deliberately short iteration caps (4 and 20), where stopping at the cap is the point.

## The change

In `solve_compiled`, a result is **settled** when it succeeded *and* it stopped for a reason
other than the iteration limit (status 0–3). The rescues run when the first result is not
settled, rather than only when it failed:

- Not settled but successful (status 4, residual under acceptance): run the block pass (when
  there are two or more blocks), else resume the DogLeg from where it stopped for one more
  budget. Keep the rescue's pose only if it is settled; otherwise keep the first pose exactly as
  today. A document the rescue cannot improve keeps its bits, its status and its success.
- Failed: as today (block rescue, then LM).
- The reported `success` does not change meaning. `SolveResult` gains nothing new unless the
  measurements show a caller needs to know a pose stopped on the limit (status 4 is already
  reported).

Only `opts.retry` solves (the one-shot solve: solventc, the drawing compiler, fixtures, the FFI,
the app's solve, `PlanSolver`'s fallback) take the new path.

## Phases

1. **Measure first.** An ignored tool (beside `block_measure`) that solves every corpus document
   and the twelve recorded designs from their own seeds and from the scaled and jittered starts
   `block_measure::robustness` uses, and records per start: status, residual, and — for starts
   that ended on status 4 with success — how far the pose is from a tight solve continued from
   it (in units of the extent). Also run `hypoid_layout::design_sweep`. This says how many
   documents are affected, by how much, and whether block rescue or a resumed DogLeg is what
   fixes them.
   **Exit:** the table is in this document, and the choice between the two rescues (or both) is
   made from it.
2. **The trigger.** Implement "not settled" in `solve_compiled` with the chosen rescue; tests:
   a document built to stop on the limit (a long chain seeded far off, or `max_iter` lowered on
   the rough-seed hypoid) now settles, and lands on the recorded numbers; a document that stops
   on the limit and cannot be improved keeps its exact pose and status; drags unchanged
   (`PullPolish` bit-identical on the drag tests).
   **Gates:** full, slow and web suites; the byte gate against main — every corpus document whose
   own-seed solve did not stop on the limit is byte-identical, and each one that did is listed
   with its old and new pose distance; the design sweep's count of unconverged designs drops;
   the preview-convergence gate still passes.
3. **Record.** This document's findings; CLAUDE.md's solver note (what counts as settled).

## Risks

- **Bits.** Any document whose own-seed solve stops on the limit changes; phase 1 counts them
  before anything is changed.
- **Time.** The rescue runs for more solves than before. Phase 1 measures how many; a status-4
  success is by definition a slow solve already.
- **Status 3.** A collapsed trust region at an accepted residual is also not a clean stop; it is
  left alone here unless phase 1 shows it matters.

## Files

- `rust/gcs-core/src/solve.rs`: `solve_compiled`'s trigger and the resumed DogLeg.
- `rust/gcs-core/tests/`: the measurement tool, the trigger tests.
