# Phase 0a: validator calibration and partial audit reports

Implements [Phase 0a of the architecture plan](swept-boundary-architecture-plan.md), starting
from Phase 0 commit `a996167`. This changes validation and diagnostics; the sweep candidate
generator is retained. Construction recovery is tracked in the
[thirteen-test recovery ledger](swept-boundary-recovery-ledger.md).

## Findings that determined the changes

The initial eight calibration runs (sphere/capsule, static/swept fields, tolerances 0.04/0.02)
all exhausted the old 100,000-cell audit budget. After retaining partial reports, the static
sphere at 0.04 required 54,912 forward visits and 89,115 reverse visits. The equivalent
rotating-sphere field then exposed a different limit: 13,174 reverse cells remained unresolved
at depth 24. Its construction-derived support is larger, so the same subdivision depth does
not give the same spatial resolution. Neither refusal establishes a defect in the mesh.

The following changes address those specific barriers:

- Forward validation bisects the longest edge. It bounds distance over a triangle using its
  three interval-enclosed corners and convexity, avoiding the extra corners of the triangle's
  axis-aligned box. This reduces needless subdivision, especially on long barrel triangles.
- Reverse validation searches the original mesh directly for a convex-combination witness.
  It can proceed independently of forward success. A triangle hierarchy proposes nearby
  witnesses; only outward interval arithmetic decides whether their distances suffice.
- A reverse cell with a mesh-distance witness needs no field evaluation. Other cells still
  require a strict field enclosure for exclusion or remain subject to subdivision/refusal.
- The total visit budget reserves half for forward work and at least half for reverse work;
  unused forward visits transfer to reverse work. No direction can silently consume the
  other's entire allocation. Reports retain every unprocessed region when a budget runs out.
- Spatial-box queries have an explicit `box_budget`, capped by the judge's far-query budget.
  Wide boxes start with at most 64 roll evaluations; small undecided boxes use the full
  configured allowance (1,000 by default). Unresolved enclosures cause refinement or explicit
  refusal, never acceptance or a geometric contradiction. Uniformly using 1,000 evaluations
  spent 84,590,112 roll evaluations on the rotating sphere at 0.04. Uniformly limiting queries
  to 64 then failed with 705,895 budget-limited queries: spatial subdivision could not resolve
  temporal uncertainty. The final policy lets the two refinements address different limits.
- Calibration specifies 800,000 total visits at 0.04 and 3,200,000 at 0.02, with depth 36.
  The fourfold budget allowance accompanies halved spatial tolerance. The default remains
  100,000 visits/depth 24 and can still refuse correct geometry. The supported explicit depth
  limit is now 48; interval arithmetic continues to guard subdivision rounding.

The initial 400,000-visit calibration allowance was sufficient for the sphere but exhausted
its forward allocation on the capsule's long barrel triangles. The larger allowance is an
explicit measured resource choice; it does not change the error tolerance or certify unfinished
work. Further improvements to triangle refinement and field bounds remain possible.

These are bounded validation improvements, not a claim of production-speed gear validation.
The field magnitudes remain value/exclusion evidence, never upper distance bounds.

## Inspecting a refused candidate

`BoundaryCandidate::inspect` and free `inspect(field, mesh, options, audit_options)` produce
an immutable `BoundaryReport`. Topology, centroid checks and spatial work are independently
attempted when their input permits it. Invalid mesh indices, nonfinite vertices or inconsistent
sheet metadata mark dependent checks `Check::NotAttempted(InvalidMesh)` before compaction.
Invalid centroid probe options do not prevent otherwise valid spatial work.

`AuditReport` retains:

- completed forward brackets with exact subdivision paths and conservative bounds;
- completed reverse cells, each with either a strict field enclosure or a mesh witness;
- every unfinished forward region and reverse cell, with its depth/path and issue;
- explicit setup limitations, including missing finite support for reverse coverage;
- separate forward/reverse visit counts. The enclosing report also records field-query,
  roll-evaluation and query-budget/resolution-limit counts.

Query-limit counters include intermediate attempts subsequently resolved by subdivision.
They are distinct from the unfinished terminal obligations that block acceptance.

`AuditIssue::OffSurface` is a demonstrated contradiction: a strict field enclosure over the
entire tolerance neighborhood of a known mesh point excludes all material boundary there.
`AuditIssue::Unresolved` represents a failed witness search, subdivision limit, exhausted
budget or arithmetic/query error. Those outcomes do not establish that geometry is wrong.
Reverse coverage may also remain unresolved; a failed witness search does not prove omitted
material. Partial witness distances are not exposed as a completed global distance bound.

Completed and unfinished regions together partition the original triangles and full derived
support. The tests check this accounting under budget and depth exhaustion. Witness storage
is bounded by input size and the visit budget; it does not retain a full refinement tree.

`BoundaryReport::into_accepted` consumes the immutable report and requires every acceptance
gate. `validate` retains its efficient early-refusal behavior; callers needing independent
diagnostics use `inspect`. Both paths use the same spatial validator. Neither unattempted nor
unresolved work can become an accepted surface.

With an intentionally tiny 128-visit budget, the actual open cylinder reports 64 forward and
64 reverse visits, 33 completed forward witnesses and 28 completed reverse cells, plus 6,127
unfinished forward regions and nine unfinished reverse cells. Those are partition-region
counts, not counts of defective triangles. Its existing topology and centroid failures remain
independently visible, and the report refuses acceptance.

## Independent curved controls

`tests/sweep_mesh/calibration.rs` constructs unit spheres centred at `(3,0,0)` and capsules
with centre segments from `(3,0,0)` to `(3,0,2)`. Latitude/longitude hemispheres share indices
with a polygonal barrel; no sweep seed, trim, weld or zip code constructs these meshes.

For angular cell width `h = pi/n`, the norm of every second partial of the unit-sphere
parameterization is at most one. Taylor interpolation therefore bounds the geometric error
by `2*h²`. The same parameter cover gives both distance directions, including pole fans;
barrel interpolation has the smaller bound `h²/2`. Interval trigonometry encloses the ideal
vertices and transfers the actual binary64 coordinate rounding into the bound. The resulting
independent bounds are approximately 0.019276571 for `n=32` and 0.008567365 for `n=48`.
They are conservative bounds, not measured maximum errors.

The static sphere is an analytic revolved disk. The static capsule is the union of an
analytic cylinder and its endpoint balls. Equivalent continuous fields are read from Solvent:
the sphere rotates about its own axis through ±60°, or translates two units along that axis.
Each mesh is checked against both representations, at spatial tolerances 0.04 and 0.02.
Source-solve error is not included in the analytic reference bound; acceptance separately
checks the field snapshot actually read from the solved model.

The calibration prints triangle counts, independent bounds, accepted bounds, both visit
counts, retained witness counts, query/roll counts and wall time. This demonstrates validator
capability on independent meshes; it does not restore the historical generated sphere or
10-unit capsule acceptance cases.

## Measured eight-case calibration

All eight combinations passed. Independent bounds are approximately 0.019276571 at tolerance
0.04 and 0.008567365 at 0.02. Every row finished with zero outstanding spatial obligations.
The table records the complete eight-case run in `calibration-adaptive.log` (676.40 seconds).
Some validation processes overlapped, so wall times are not isolated single-core benchmarks;
visit/query counts are the more reproducible cost measurements.

| Shape / field | Tolerance | Triangles | Accepted bound | Forward visits | Reverse visits |
|---|---:|---:|---:|---:|---:|
| sphere / static | 0.04 | 3,968 | 0.039999930 | 54,912 | 89,115 |
| sphere / continuous | 0.04 | 3,968 | 0.039999552 | 54,912 | 168,259 |
| capsule / static | 0.04 | 4,096 | 0.039999930 | 242,176 | 160,567 |
| capsule / continuous | 0.04 | 4,096 | 0.039999930 | 242,176 | 186,575 |
| sphere / static | 0.02 | 9,024 | 0.019999742 | 219,072 | 362,571 |
| sphere / continuous | 0.02 | 9,024 | 0.019999954 | 219,072 | 949,359 |
| capsule / static | 0.02 | 9,216 | 0.019999485 | 1,065,216 | 661,779 |
| capsule / continuous | 0.02 | 9,216 | 0.019999959 | 1,065,216 | 844,927 |

| Shape / field | Tolerance | Field queries | Roll evaluations | Retained forward / reverse witnesses | Wall seconds |
|---|---:|---:|---:|---:|---:|
| sphere / static | 0.04 | 142,715 | 0 | 29,440 / 44,558 | 0.492 |
| sphere / continuous | 0.04 | 214,113 | 59,092,456 | 29,440 / 84,130 | 56.016 |
| capsule / static | 0.04 | 384,403 | 0 | 123,136 / 80,284 | 1.583 |
| capsule / continuous | 0.04 | 412,349 | 25,666,248 | 123,136 / 93,288 | 17.896 |
| sphere / static | 0.02 | 541,229 | 0 | 114,048 / 181,286 | 1.477 |
| sphere / continuous | 0.02 | 1,052,997 | 388,739,960 | 114,048 / 474,680 | 499.963 |
| capsule / static | 0.02 | 1,609,293 | 0 | 537,216 / 330,890 | 7.210 |
| capsule / continuous | 0.02 | 1,809,655 | 133,591,892 | 537,216 / 422,464 | 91.393 |

The 0.02 rotating-sphere cost is a material limitation of this validator, not a recovered
production sweep capability. The reference meshes establish correctness calibration at explicit
budgets; later construction work should preserve geometric evidence to avoid rediscovering it
through hundreds of millions of field evaluations.

## Reproduction and remaining scope

The full core suite passed: **1,232 passed, 0 failed, 95 ignored**, in 148.05 seconds
(`core-complete.log`). This includes the existing twelve evidence controls, seven new partial
report controls, the bounded open-cylinder audit, and the ordinary six-case calibration.
The thirteen historical construction assertions remain deferred; the two additional ignored
tests are the fine continuous calibration and the field-query diagnostic instrument.

The final fine-only rerun also passed both continuous fields at 0.02: sphere 403.027 seconds,
capsule 88.099 seconds, 491.29 seconds overall (`fine-calibration.log`). Its visit, witness,
query and roll counts and accepted bounds match the corresponding rows above. There were no
unfinished spatial obligations. Two earlier broad runs were interrupted to isolate the slow
calibration; `core-complete.log` is the completed final full-suite result.

All sixteen exported candidate source/STL/status files are byte-identical to Phase 0's
preserved `after/` artifacts (`export-parity.json`). The dumbbell still refuses before a final
candidate, so it has no new STL. No improvement in candidate geometry is claimed.

```sh
cargo test --manifest-path rust/Cargo.toml -p gcs-core --test core \
  sweep_mesh::calibration -- --nocapture
cargo test --manifest-path rust/Cargo.toml -p gcs-core --test core \
  sweep_mesh::calibration::finer_continuous_reference_meshes_have_measured_acceptance -- --ignored --nocapture
cargo test --manifest-path rust/Cargo.toml -p gcs-core --test core \
  sweep_mesh::audit_report -- --nocapture
cargo test --manifest-path rust/Cargo.toml -p gcs-core --test core
SOLVENT_EXPORT=/private/tmp/swept-boundary-phase0a-check \
cargo test --manifest-path rust/Cargo.toml -p gcs-core --test core \
  sweep_mesh::status::phase_zero_status -- --ignored --nocapture
```

Run logs and candidate exports are preserved locally under
`/private/tmp/swept-boundary-phase0a/`. Timings are wall times from this machine, not portable
performance promises. The immutable field snapshot, single-shell topology and both spatial
distance directions remain the accepted scope. Geometric embedding, topology equivalence
below tolerance, source-solve accuracy and float32 export validation remain separate gates.
The next geometry task remains Phase 1's independent continuous cylinder oracle.

The ordinary calibration test covers all four static shape/tolerance combinations and both
continuous fields at 0.04. The two continuous-field checks at 0.02 take minutes and are gated
as an explicit slow test, following the repository's test convention. Run that test after
audit or field changes; an ordinary green suite alone does not rerun the fine calibration.
The separate ignored `rotating_sphere_box_queries_at_known_exterior_points` instrument records
how roll budgets affect boxes independently known to be exterior. At the sampled boxes,
64 evaluations left 24/26 undecided, while 1,000 resolved all 26; a budget hit is not a
geometric failure.
