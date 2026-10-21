# Solve boundaries

Part of the [contract](../../CLAUDE.md) the code is held to. Read it when you touch
`solve/through.rs`, `boundary::instance`, or `newton::square`. The design these rules are drawn
from, with the rules not yet kept, is [solve-boundaries-plan.md](../solve-boundaries-plan.md); a
rule's tag (FR1, DD1, …) is its name there. An issue number (#n) says where a rule came from.
The gate is `rust/gcs-core/tests/boundary_solve.rs` unless a rule names another.

- **A boundary is named from Rust, never from the source** (#454). `solve::Boundary` is an
  instance's name, its own unknowns (parameters) and its rows (constraint ids);
  `boundary::instance(e, sk, "upper")` builds one for a drawn component instance, reading what it
  made and what it states exactly as the classifier reads them (`boundary.rs`'s `split`: every
  entity sited under its path and a twin of one, the numbers no entity owns that its rows read,
  the rows no statement owns that reach only those). No syntax marks a boundary until #461
  decides on #459's evidence.

- **A boundary is square** (FR1, FR3). As many rows as unknowns of its own, so `F_y` is square and
  its derivative `−F_y⁻¹F_x` is never a minimum-norm choice. One that is not is refused by
  `Boundaries::new` before anything is solved, naming it and both counts; so is one with no
  unknown, one that owns a ring's turned copy, and two that share a row or an unknown.
  `a_body_with_a_freedom_is_refused`.

- **One compile** (#454). A boundary's block is a `Subset` of the one `System` the flat document
  compiles to, and the parent's rows are the rest: no second column mapping, no second compile
  (which would forget every traced contact's pose), and every row in the flat solve's units. So a
  row is judged at the **document's** scale, not the boundary's own: the value agrees across
  documents of very different extent to the inner tolerance
  (`the_apex_does_not_read_the_size_of_the_rest_of_the_document`), and the own scale of CL1 is
  #455's. **Its inputs are every other free column its rows read** (CL2): a column the document
  holds is a constant of it, not an input. `a_run_reads_its_two_rims_and_nothing_else`.

- **The inner solve is the locus's** (`newton::square`). One square Newton, backtracked, judged in
  the rows' own units, serves a trace block at one place (`locus::newton`) and a boundary
  (`solve::through`): the loop the locus said it would move to `newton.rs` at a second user. Both
  stop at `SquareTol::BLOCK` (done 1e-12, accepted 1e-8, stall 1e-10) and judge a resumed solve
  by one `newton::continues`. The locus's numbers are unchanged by the move, to the bit.

- **The derivative is the implicit function theorem on the block's own factorisation** (DD1).
  After the value is accepted, `F_y` is factored once (LU) and `dy/dx = −F_y⁻¹F_x` answered for
  every input column from it. It is held to a central difference of the boundary's own
  evaluations (`*_derivative_is_its_difference`, relative 1e-5 at a step of 1e-6 of each input) for
  the apex, both belt runs and the common perpendicular.

- **An evaluation is a value or a failure, never a nearby value** (FA1, FA2, DT5, DD3). The first
  is *fresh*, from the pose the document gave the boundary; every later one is *continued*: started
  where the last accepted value's tangent predicts and accepted only if Newton's correction is
  within half the predicted move (`newton::continues`, plus the inner tolerance) and the sign of
  `det F_y` (`linalg::lu_det_sign`, the LU's pivots and parity, as `follow` reads it) is
  unchanged. Otherwise it fails — *unsettled*,
  *singular* (an exactly zero pivot; a condition threshold is DD2's, #455's) or *branch changed*
  — and the parent reads NaN for every one of its rows, which the DogLeg rejects and shrinks its
  trust region for. Asked again at the same inputs it answers from the last value, bit for bit:
  the Jacobian is asked where the residual just was.

- **A parent at a domain's edge stops short and says which boundary held it** (FA3, DD5 in part).
  A solve that ends on a failed evaluation fails with NaN `max_residual`, its message naming the
  boundary and the reason; one that ends on values and still did not solve names the last
  evaluation that failed along the way (`Through::failure`). Every value it accepted holds the
  boundary's rows to the inner acceptance. `a_parent_asked_to_nest_the_pulleys_stops_short_and_says_which_run`,
  `a_run_between_nested_pulleys_fails_and_the_parent_says_which`.

- **Outputs flow one way** (IO4). A boundary whose inputs include another's unknowns is evaluated
  after it, in a deterministic order (`graph::tarjan` over what each reads, each after every one it
  reaches), and the parent's derivative is chained through both
  (`an_apex_over_another_apex_solves_through_both`). Boundaries that read each other round a cycle
  are the parent's coupled system: their rows and unknowns go back to the parent, solved flat, and
  `Through::inlined` names them (`boundaries_that_read_each_other_are_solved_in_the_parent`).

- **Solved means what it means flat** (FL1–FL3). A solve through boundaries is judged on the flat
  document's rows at the pose it ends on (`max_relative_residual < acceptance_tol`), not on the
  parent's rows alone. For every case in the gate, the pose it reaches is the flat solve's to
  1e-9 (1e-8 for the common perpendicular in space), the flat solve started there does not move,
  and the flat numeric rank is the boundaries' own unknowns plus the rank of the parent's rows
  over its own unknowns through the boundaries' derivatives, conditioned as the diagnosis
  conditions a system (`Through::reduced_rank`, `System::conditioned_of`). The direction paper's
  belt drive (§6.1, turning frames left out) solves with both runs as boundaries to the flat
  answer, its runs crossing at x = −96 (`the_belt_drive_solves_through_its_runs_to_the_flat_answer`).

- **Within the solver.** `solve/through.rs` is in `gcs-solve` and imports nothing of the
  language; only `boundary::instance` reads the source map, on the language's side, in
  `gcs-core` (the crate boundary holds it, #409).
