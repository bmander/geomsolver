# Swept boundary: architectural diagnosis and recovery plan

Original review: 2026-09-12 at `5e95019`, including the then-uncommitted changes.
Plan revised: 2026-09-13 after Phase 0a and Phase 1, committed in `2331234`, to add the
bounded Phase 1a provenance-repair checkpoint, now implemented.

## Recommendation

Keep the continuous material field and analytic sweep candidates. Replace the boundary
construction's reliance on independently trimmed triangle soups followed by proximity-based
repair with **shared, geometrically supported boundaries that are established before final
tessellation**. Test this architecture on a replayable group of interacting cylinder patches
before extending it. The replacement may need to encompass more than a small rim neighborhood.

Phase 0 corrected the field judge's residual-to-distance inference and separated candidate
reports from accepted surfaces. Phase 0a now provides independently bounded curved controls
at tolerances 0.04 and 0.02, with explicit resource budgets and inspectable unfinished work.
Phase 1 now supplies an independent continuous cylinder oracle and exact construction replay;
see the [implementation record](swept-boundary-phase-one.md). It finds an inadmissible crease
alias before the zip and substantial extra/reversed surface after it. It also exposes a
triangle/sheet-array mismatch at crease removal. Phase 1a now repairs that mismatch and
verifies source ownership; its [refreshed baseline](swept-boundary-phase-one-a.md) isolates the
first geometric change to overlap trimming. **The next step is Phase 2's shared-boundary
experiment, starting before the remaining unsafe crease identification.**
The calibrated validator remains expensive on continuous fields and is not a production-speed
gear validator.

The historical seven loops were a symptom, not a sufficient definition of the remaining work.
The Phase 1a pipeline produces a 6,033-triangle cylinder candidate with thirteen loops and
substantial failed and unresolved surface checks. Closing its rims cannot by itself finish this cylinder,
general sweeps, or hypoid gears.

## What Phase 0 changed

These findings motivated Phase 0a. Its [implementation record](swept-boundary-phase-zero-a.md)
now records the curved positive controls, partial reporting and measured limits that address
the calibration/reporting gaps below. Shared-boundary construction remains untested.

The [implementation record](swept-boundary-phase-zero.md) contains the reproducible baseline,
new status matrix and validation results. It supersedes the historical counts and API behavior
in the original diagnosis below.

- The accepted positive control is a manually constructed cube at spatial tolerance 0.2.
  Default sweep acceptance uses 0.04; stationary-sphere, capsule and torus candidates currently
  fail the spatial gate. These results do not yet isolate candidate error from validation
  conservatism or resource limits. Curved positive controls are required before judging a new
  construction method with this audit.
- Open candidates now receive centroid-side reports, but acceptance still stops at its first
  failed gate, and the spatial audit discards partial evidence when it returns an error.
  The cylinder report is not yet an inventory of all geometric and coverage obligations.
- The new cylinder has 350 reversed checks and 252 unresolved checks, in addition to ten open
  loops. The dumbbell refuses before a final candidate, and the whole-turn box reports genus
  10. Since construction changed, old/new counts do not isolate deterioration or improvement.
  They invalidate an assumption that completing the old rim repairs is the remaining task.
- Phase 0 validates the need for stronger evidence. It does not experimentally establish that
  shared-boundary construction will resolve the cylinder; Phase 2 must test that hypothesis.
- Thirteen historical success/repair tests were explicitly deferred. Active refusal tests
  establish containment, while restoration of successful construction remains a separate
  milestone. Exact current failure counts are baseline observations, not permanent targets.

The revised sequence is: validator calibration and reporting → independent cylinder oracle →
bounded provenance repair and refreshed measurements → region selection from measured failures →
one shared-boundary experiment → incremental recovery of successful sweeps. Existing phase
numbers are retained; Phase 0a, Phase 1 and Phase 1a are implemented, and Phase 2 is next.

## Evidence and scope of this review

This section and the architectural diagnosis through “Target design” record the original
2026-09-12 review. Descriptions of permissive judge/acceptance behavior are historical; Phase 0
corrected those contracts without replacing the surface construction.

I read the [handoff](swept-boundary-handoff.md), the running
[investigation](swept-boundary.md), recent commit history and relevant changes, the current
judge/construction/trim/stitch/certificate code, the earlier field extractor, and the gear
roadmap and CLI boundary path. No implementation changes were made for this review.

Two existing instruments were run against the current working tree:

| Instrument | Fresh result |
|---|---|
| `creases::draw_the_refused_loops` | 6,713 triangles; seven loops of sizes 4, 7, 5, 7, 4, 3, 3 |
| `creases::what_the_certificate_says_of_the_tumbling_cylinder` | 4,960 counted certified, including 726 slivers; 169 failures; 142 failures attributed to zip triangles; failed area 0.8267 |

The remaining **1,584 triangles are in `thin`**, by subtraction from the certificate's three
exclusive outcomes. Only nine of the 169 failures touch an open loop. Most failing triangles
would therefore remain after a repair confined to those loops. The diagnostic's printed
interpretation about winding is historical commentary, not a conclusion established by its
counts; the later winding experiments in the handoff supersede it.

The fresh loop SVG was rendered with installed Chrome and inspected in three orthographic
views. It shows the surviving rims clustered near the two ends of the tumble axis. This is
a view of mesh connectivity, not independent evidence of the correct material boundary.
Temporary review artifacts are `/private/tmp/swept-boundary-review-loops.svg` and `.png`.

The handoff's open-mesh volume 9.3057 and coarse reference volume 9.4786 were not remeasured.
Neither is an acceptance oracle: the former is a divergence sum over an open surface, and the
latter comes from an explicitly approximate field contour. The quoted full-suite result is
historical; this review ran the two targeted instruments, not the full suite. Their tests
return success when they finish reporting, even though the reported mesh fails.

## What the commit history reveals

| Period / commits | Real progress | Architectural lesson |
|---|---|---|
| Sept. 9: `b8cfaac`, `710c590`, `47a829b` | Candidate-sheet arrangements and kernel/Manifold export became usable. | A closed result and plausible volume did not establish agreement with swept material. |
| Sept. 10: `8761125`, `6d43517`, `399bd2f`, `0f34243` | More tool faces, motions and field composition became available. | Candidate coverage and material classification are distinct responsibilities. |
| Sept. 10: `dcc0cb6`, `d8c9a49`, `6f0bf90` | Field judge, triangle checks, caps and shell construction. | The new judge inherited an invalid residual-to-distance inference from its first milestone. |
| Sept. 11: `3561ec5`, `3437510`, `b8d1530`, `de5c21b` | Cap cuts, planar unions, crossing sheets and grazing planar regions solved substantive cases. | Explicit geometry and common cuts made durable progress. |
| Sept. 11–12: `b169186` through `c38d0c3` | Cut provenance, stage audits, local inspection, and correct splitting of pinched walks. | Observability improved, but much topology was still reconstructed after surfaces were meshed. |
| Sept. 12: `7975374`, `5e95019`, working-tree changes | Missing cap area recovered; stitch defects fixed; facet quality investigated; loops reduced. | Local repair helps, but loop count, manifoldness, outwardness and boundary coverage move independently. |

This was not days of no progress. Several incorrect constructions were identified and fixed,
and the code now exposes failures that it previously concealed. The stall is that each
repair is being asked to recover information the construction did not preserve, using a
judge whose guarantees are weaker than its callers assume.

The running record repeatedly promotes a local observation into a global cause, then retracts
it: absent generator, missing coverage, false boundary, multi-way weld, wrong fan direction.
These were useful experiments, but their failure does not identify the next hypothesis by
elimination. In particular, refusal of a projection along a few directions cannot prove that
no suitable surface exists; successful probes cannot prove that a whole proposed patch is
supported. Historical comments should not serve as current architectural contracts.

## The architectural faults

### 1. The field judge reverses the Lipschitz distance implication

For a one-Lipschitz function and any zero `z`,

```text
|f(p)| = |f(p) - f(z)| <= ||p - z||.
Therefore |f(p)| <= distance(p, zero set).
```

The field magnitude is a **lower bound** on distance to the zero set. Small magnitude does
not give an upper bound. A converged enclosure containing zero bounds a value; it does not
establish a nearby root. This is also the inequality in Hart's sphere-tracing theorem.
[Hart, Theorem 1 and its proof](https://3dvar.com/Hart1996Sphere.pdf).

A counterexample uses precisely the supported kind of Boolean geometry. Take two unit balls
centered at `(±0.999, 0, 0)` and their union field:

```text
f(p) = min(||p - (0.999,0,0)|| - 1, ||p + (0.999,0,0)|| - 1).
f(0) = -0.001.
distance(0, boundary of the union) = sqrt(1 - 0.999²) = 0.0447101778...
```

The closest exposed boundary is the balls' intersection circle. The shallow negative value
at the origin is more than forty times smaller than the actual boundary distance. The
arithmetic was checked separately during this review; this is a mathematical counterexample,
not a newly added Rust regression or a measured explanation of a particular cylinder vertex.

There is a second distinction: the field defines `closure({f < 0})`, so even an exact zero
need not be a material boundary. `A - A` has no material but its min/max field retains zeros.
The existing [spatial field](../rust/gcs-core/src/solid/field/spatial.rs) documents this problem
and simplifies one shared-node Boolean identity that produces it.

Despite that, [judge.rs](../rust/gcs-core/src/solid/swept_boundary/judge.rs):

- explicitly documents `Near { within }` as an upper distance certificate;
- converts small strictly signed enclosures into `Near` as well;
- lets `project` return `Kept` or `Moved` on a `Near` reading without obtaining opposing signs;
- lets bisection abandon an existing spatial bracket in favor of the residual-derived radius.

The same inference appears in `docs/swept-boundary.md` and `CLAUDE.md`. The handoff warns
against treating field values as distances, but this error remains in the executable judge.
It invalidates the general proof claimed for labels and sliver certification. It does **not**
prove that every retained triangle is wrong, or that correcting it alone closes the cylinder.

The valid primitives are strict material/exterior witnesses, spatial brackets between such
witnesses, and conservative spatial exclusion bounds. Deep field margins can establish
exclusion; failure to obtain a deep margin cannot establish proximity.

### 2. Connectivity and geometric support become disconnected

The pipeline in [construct.rs](../rust/gcs-core/src/solid/swept_boundary/construct.rs) is:

```text
seed -> cap/graze -> label -> clip -> merge -> centroid filter
     -> planar union -> overlap clip -> weld/collapse -> split -> zip -> certify
```

There are useful geometric representations upstream: source faces and edges, motion columns,
cap cuts, grazing regions and cut-vertex origins. But `SweepPatch` primarily carries points,
normals, triangles and column/time data. `KeptMesh` reduces the downstream representation to
vertices, triangle indices and one sheet number per triangle. `Rim` identifies a sheet and
vertex chain, not a shared intersection with incidences on both supporting surfaces.

Thus neighboring sheets are trimmed separately; `merge_creases` tries to identify their
vertices by proximity; `clip_overlaps` gives earlier sheets precedence; and `rim_zip` tries
to manufacture the final adjacency. New zip triangles carry `u32::MAX` as their source.
The late stages have no persistent geometric patch definition from which to regenerate a
correct curved seam, or common boundary identity requiring both sides to use identical samples.

This explains why a weld guard can correctly refuse while the desired solid is perfectly
ordinary. The available operation is vertex identification, while the necessary operation
may be cutting overlapping surface pieces and replacing their triangulations together.
Adding more vertices to a weld cluster does not change that mismatch.

This diagnosis is supported by the representation and mutation code. Whether each particular
surviving loop comes from overlap, an omitted piece, a fold or displaced trimming still needs
local evidence. The handoff's 28 bracketed rim midpoints out of 33 support a curved-fill
hypothesis for part of the residue; they do not establish a correct fixed boundary for all
seven replacements. A finite probe bracket proves proximity along that segment, not exact
incidence, uniqueness of the crossing, or correctness of the entire edge.

### 3. The proposed geometry and the checked geometry need not coincide

In [stitch.rs](../rust/gcs-core/src/solid/swept_boundary/stitch.rs), `takes` and `lay` check
area, vertex duplication, existing facets and directed-edge use. Those checks cannot establish
material-side agreement, vertex-link manifoldness or freedom from geometric intersections.
Consistent winding propagates across a surface folded into the material just as readily as
across a correct surface.

The late `closeable` callback adds a field check for one fill path. Its result is only a
Boolean. `loop_span` can receive `Projection::Moved`, discard the projected point and spatial
evidence, and approve the original fan. The caller then lays triangles at its own positions.
Other zip paths do not ask this callback. `lay` can also commit a subset of a band and let
later rounds repair the resulting residue.

The missing interface is an **actual proposed patch plus its geometric evidence and boundary
obligations**, validated before committing the replacement. A Boolean saying that some nearby
surface was found is not that interface.

### 4. Acceptance conflates absence of detected failure with certification

[certify.rs](../rust/gcs-core/src/solid/swept_boundary/certify.rs) says that `thin` triangles
are not certified, but `is_complete()` tests only `failures.is_empty()`. A buried triangle can
enter `thin`; slivers can pass on the invalid `Near` inference. A strict centroid bracket,
when obtained, proves something at the centroid, not a bound over the whole triangle. The
original tracer's sagitta is not automatically inherited by subsequent projections, tolerant
splits, overlap clipping, collapses and inserted bands.

`construct_from` returns early on open rims, hiding its certificate stage from the normal
pipeline. Conversely, once rims close, it returns `Ok(SweptBoundary)` even when the returned
certificate has failures. It does not itself construct a checked `ClosedShell`. Test helpers
apply additional gates, but the public result type does not enforce the advertised contract.

Consequences include the historical cases that appeared successful with stale open-loop
reports, and the current cylinder's much larger correctness backlog than seven holes suggest.
The main acceptance case is still ignored. A green normal suite does not mean it is solved.

### 5. The dependency between arbitrary tools and Boolean boundary recovery is cyclic

The project correctly separates continuous sweeps from reliable Booleans as two prerequisites
for hypoid export. But arbitrary Boolean tools already need usable source boundaries for
candidate generation and caps. The dumbbell's faceted Boolean tool is an example of this
dependency entering the supposedly earlier milestone.

Resolve the cycle with a narrow, explicit source-boundary interface, not by making either
project wait for completion of the other. The sweep needs evaluable source patches, trim
domains, oriented feature incidences and conservative bounds. Static primitives can provide
them first; Boolean tools must supply them through a shared arrangement or a bounded fallback.
The downstream Boolean operation on *completed swept solids* remains a separate delivery gate.

## Target design

The field continues to define material. Source charts propose geometry and accelerate its
construction; they do not determine global visibility on their own. This division retains
the strongest existing work without treating an arbitrary implicit field as a surface oracle.

Introduce the following responsibilities, initially only for a local replacement path:

| Responsibility | Required information / invariant |
|---|---|
| Field evidence | Strict sign enclosures and their budgets; a boundary bracket stores both spatial endpoints and signs. Small residual and exhausted query are separate outcomes. |
| Candidate patch | Source feature, evaluable parameters, motion interval/branch, bounds, orientation and supported approximation error. Existing triangles can be seeds for such patches. |
| Shared boundary | One event/intersection/overlap-boundary identity, ordered samples, parameter references on every incident patch, and geometric uncertainty. |
| Arrangement region | Patch domain split at those boundaries, with explicit material-side evidence; overlaps select ownership only after establishing that they represent the same boundary. |
| Patch tessellation | All incident regions consume the same boundary samples. Interior refinement follows the supporting surface and preserves a stated error budget. |
| Accepted result | Immutable mesh with checked topology, geometric evidence and explicit coverage scope. Failed or unresolved candidates remain inspectable but cannot masquerade as accepted output. |

Ordinary surface crossings, coincident regions, sharp edges, endpoint caps, planar grazing
regions and singular/branch events need distinct treatment. Pairwise intersection computation
is useful, but its events must be assembled into one incidence structure before tessellation.
That is different from the already-refuted attempt to collapse a cluster of final vertices.

The sweep literature supports preserving this structure: Adsul, Machchhar and Sohoni carry
face/edge/vertex correspondence and derive matching oriented boundaries in their sharp-feature
framework. Their local/global analysis separately treats trimming and self-intersections.
These are design precedents, not an off-the-shelf proof or implementation for this repository.
[Sharp-feature framework, §§6–7](https://arxiv.org/html/1405.7457),
[local and global sweep analysis](https://arxiv.org/abs/1305.7351).

Do not make a full new B-rep kernel the first deliverable. Implement enough common boundary
structure to replace one failing neighborhood and test whether it resolves the actual barrier.

## Implementation sequence and exit gates

### Phase 0 — establish a trustworthy result contract

Implemented in `a996167` on `swept-boundary-phase-zero`; see the [implementation record](swept-boundary-phase-zero.md)
for the evidence contract, validation results, changed candidates and remaining limitations.

1. Freeze a reproducible snapshot of the current dirty tree and the five-case exports into a
   separate output directory. Include the whole-turn box and cylinder perturbation case in the
   status matrix. Preserve the original work and record hashes/options with each artifact.
2. Add small negative controls for the union-of-balls counterexample, `A - A`, buried triangles,
   unsupported slivers, genuinely thin boundary, and a triangle whose centroid passes while
   its interior crosses unsupported geometry. Each must discriminate the old acceptance rule.
3. Separate field-value convergence from spatial boundary evidence. A `Near` result can guide
   refinement but cannot create `Kept`/`Moved` distance guarantees. Bisection retains its
   opposing spatial witnesses even if its midpoint remains ambiguous; it may use their width
   as an error bound or return unresolved. It must not replace that width with a residual.
4. Separate candidate reports from accepted surfaces. Acceptance requires no failed or
   unresolved surface obligations, a checked shell structure, and explicit geometric and
   coverage guarantees. Run diagnostics on open candidates as well; report closure, side
   agreement, unresolved area and topology independently.

**Exit:** negative controls are refused; the accepted cube has real spatial evidence;
the cylinder's topology and centroid-side obligations are reported without waiting for closure.
Expect some previously green cases to become unresolved. Record this as correction of an
overstated guarantee rather than weakening the new rule to preserve their old status.
Whole-surface and reverse-coverage diagnostics remain incomplete on refused candidates; the
following phase addresses that limitation.

### Phase 0a — calibrate the validator and retain unresolved work

Implemented; see the [Phase 0a record](swept-boundary-phase-zero-a.md) and
[recovery ledger](swept-boundary-recovery-ledger.md). The finer continuous-field calibration
is an explicitly run slow test. Default resource budgets still need not accept correct meshes.

1. Construct sphere and capsule reference meshes independently of the sweep repair pipeline,
   with analytic approximation bounds. Test them first against static reference fields, then
   equivalent stationary/moving-tool fields, so failures can be attributed to mesh error,
   continuous-field evaluation, or the audit. Keep the cube and existing negative controls.
2. Demonstrate acceptance at the intended default spatial tolerance, 0.04, and a stated tighter
   tolerance with correspondingly refined reference meshes. Record triangle counts, visited
   support cells/subtriangles, field queries, time and memory or retained-witness counts.
   Compare independently known error with the audit bound. Identify the limiting operation
   before changing subdivision or budgets; do not loosen the geometric tolerance to obtain
   a pass. This is a bounded calibration task, not a production-performance rewrite.
3. Separate an inspectable audit report from acceptance. Preserve completed witnesses, the
   remaining regions and the resource limit reached. Distinguish a proved contradiction from
   an unresolved witness search, exhausted resources and a check not attempted. A failed
   bracket search or subdivision limit is not itself proof that the surface is wrong.
4. In diagnostic mode, run independent obligations even when topology fails, wherever the
   input is valid for those checks. Retain bounded partial forward and reverse reports;
   explicitly mark dependencies that prevent a check. Acceptance continues to require every
   applicable obligation and must never treat unattempted work as passed.
5. Keep exact current refusal counts in the reproducible status matrix. Use deliberately
   defective fixtures for enduring refusal assertions. Give the thirteen deferred tests a
   recovery ledger naming their barrier and the positive acceptance test that will replace
   or restore each one. Do not promote historical repair heuristics into acceptance rules.

**Exit:** independently bounded curved controls pass at stated useful tolerances and measured
cost; negative controls still refuse; an intentionally constrained audit retains its successful
evidence and explicit unfinished work. The report distinguishes validator limitations from
demonstrated geometric errors. It still makes no embedding, isotopy or encoded-STL guarantee.

### Phase 1 — give the cylinder an independent, continuous oracle (implemented)

The current field-contour reference shares the field implementation and is coarser than the
holes. The cylinder has a much cheaper independent geometric description.

The fixture's unswept material is
`(x - 3)² + y² <= 1, |z| <= 1`, rotated about the x-axis through `[-π/6, π/6]`.
Rotation preserves `x`. For each `x` in `[2,4]`, the initial yz section is a rectangle:

```text
a(x) = sqrt(1 - (x - 3)²)
|y| <= a(x), |z| <= 1.
```

For interior sections (`a > 0`) its radial boundary after rotation is

```text
R(x, φ) = max over θ in [-π/6, π/6]
          min(a(x)/|cos(φ-θ)|, 1/|sin(φ-θ)|).
```

A zero denominator means the corresponding bound is infinite. The maximum occurs at a roll
endpoint or when the ray points toward one of the rectangle's four corners. Between corner
directions the active secant/cosecant branch has no interior maximum. Thus finitely many
analytic candidates determine membership continuously over the roll; uniform pose sampling
is unnecessary. Handle `a = 0`, angular wrapping and the axis explicitly, including the planar
end sections of the three-dimensional swept solid.

The test-only implementation validates zero roll, symmetry, angular wrapping, end sections
and full turn. Full-turn sections are disks of radius `sqrt(2 - (x-3)²)`, with volume `10π/3`.
The ±30° volume quadrature converges near 9.426350. Analytic section overlays and weighted
triangle-distance/material-side measurements are implemented. These binary64 diagnostics and
convergence estimates are not interval-certified whole-surface bounds. The oracle remains
outside the production mesher; it is not general sweep support.

Instrument actual `rim_zip` decisions and the earlier affected cuts with stable provenance,
as the handoff requests. Reuse one reporter; record the candidate before mutation, its field
witnesses, incident patches and the actual result. Include reversed regions away from loops.
Do not infer missing area from nearest-vertex distance. Use distances to triangles, explicit
cross-sections and area-weighted coverage at a resolution that resolves the defect.

**Exit:** the independent oracle distinguishes the current defective geometry from a correct
reference, and one region has a replayable account of its required surface and its first
unsupported construction decision. Select that region from the new failure evidence, including
failures away from rims. Establish whether its surrounding surface can be validated; if not,
identify the larger interacting patch group that must be reconstructed. Do not assume that a
small local repair has a valid interface to the retained mesh.

**Phase 1 outcome:** the oracle distinguishes a refined reference from frozen bad alias/zip
emissions. A crease alias moves a correctly oriented cap triangle inward despite satisfying
the snap distance. A later small-fan emission passes topology-only guards while both the
oracle and strict field probes read its material sides in reverse. Weighted reverse area
converges near 2.013, much of it away from open rims. At distance 0.04 the reference samples
show no missing area in the final mesh; this is sampled coverage, not a completeness proof.
The local interface is unvalidated. Conservatively retain all eleven cylinder source patches
and both caps as the interacting reconstruction group.

The trace also shows that crease removal leaves stale sheet-array entries. Downstream
sheet IDs must be repaired and verified against independent source-triangle lineage before
they can define a shared arrangement. Preserve the frozen Phase 1 baseline while regenerating
new evidence after that correction. Do not tune the geometric method against corrupted IDs.

### Phase 1a — repair source provenance and regenerate the baseline (implemented)

This is a bounded correctness checkpoint before comparing construction methods. Phase 1
observed 129 stale sheet entries after crease removal and four incorrectly labelled live
triangles after subsequent splitting. Correcting only the metadata at the removal boundary
preserved the splitter's geometry. The effect on downstream unions, trimming and the final
candidate has not yet been measured.

1. Preserve the Phase 1 replay bundle and frozen alias/zip counterexamples from `2331234`.
   Repair crease removal so each surviving triangle retains its own source label. Check the
   mutations that subsequently split, remove, reorder or compact triangles, and keep triangle
   data and source ownership synchronized. New fragments must inherit the appropriate parent's
   source; newly constructed geometry must carry explicit provenance rather than an unrelated
   array entry. Keep the implementation focused on these identity-preservation obligations.
2. Add regression coverage for removal followed by splitting and compaction, including adjacent
   triangles from different sources. Verify expected ownership through an independent parent
   mapping: equal array lengths alone cannot detect shifted labels. Add stage-boundary checks
   that expose malformed metadata where it is introduced, before later processing hides it.
3. Replay the corrected pipeline with the same source, options and oracle resolutions as
   Phase 1. Isolate metadata correction from geometric changes: establish where the geometry
   first changes, then record the effects of corrected ownership on later operations. Refresh
   section overlays, area-weighted material-side checks, triangle-distance coverage at 0.01,
   0.02 and 0.04, topology, strict field reports and the candidate status/export matrix.
   Preserve the distinction between sampled diagnostics and whole-surface acceptance.
4. Reassess the proposed replacement region using reliable source ownership and the refreshed
   failures. Keep all eleven cylinder source patches as context until a smaller interface can
   be validated. Freeze the corrected candidate and its source/decision evidence for the
   Phase 2 comparison, and identify the earliest remaining unsupported geometric decision.

**Scope limit:** keep alias distances, geometric merge rules, zip filling, the material field
and the spatial acceptance contract unchanged during this checkpoint. The geometric alias
counterexample is independent of the metadata defect. Its repair belongs in the Phase 2
construction experiment. This checkpoint does not require a new mesher, a broader validator
redesign or successful cylinder acceptance.

**Exit:** the ownership regression tests and full core suite pass; the construction-stage
checks and replay demonstrate preserved source identities; and the refreshed oracle report
and status/export matrix document the effect of the correction. Retain byte parity for
unaffected cases and explain and validate changed cases, rather than requiring the old wrong
geometry to persist. The frozen negative controls must still be detected. Deliver a corrected
baseline, a supported choice of reconstruction scope (or an explicitly unvalidated larger
group), and the next discriminating construction experiment. Once these are available,
proceed to Phase 2 even if the cylinder still refuses acceptance.

**Phase 1a outcome:** surviving triangles and split children agree with independently
reconstructed source ownership. The four corrected labels change no vertices or triangle
indices through planar union; geometry first changes in overlap trimming. The refreshed
cylinder has 6,033 triangles, thirteen loops, 324 failed and 236 unresolved centroid checks.
Its sampled reversed area at 64 samples per facet is approximately 1.9105. The frozen unsafe
alias and zip counterexamples remain detectable and occur again in the corrected construction.
The local interface is still unvalidated; retain all eleven source patches as context.
See the [implementation record](swept-boundary-phase-one-a.md) for validation and frozen inputs.
This checkpoint is complete despite continued cylinder refusal; proceed to Phase 2.

### Phase 2 — test shared boundaries on one replayable region (next)

Use the corrected baseline and region assessment delivered by Phase 1a. Include the earliest
remaining unsafe crease identification in the experiment's scope; replacing only the later
zip cannot address a patch that was already turned inward during merging.

Choose one failing junction together with all incident sheets, overlapping fragments and
nearby unsupported bands. Its outer boundary must lie on validated surface. If it does not,
expand to the necessary interacting patch group; neither historical nor current rim loops
are immutable constraints. Freeze the pre-replacement candidate, source patches, options and
independent oracle results so both constructions are compared on the same fixture.

1. Recover the relevant source patches and split their parameter domains at contact endpoints,
   crossings, coincident boundaries and singular events. Preserve those identities in the
   local arrangement. Validate global visibility against the full sweep, not only the contact
   time that generated the patch.
2. Compute each shared boundary once. Store its parameters on both/all incident patches and
   refine its sample sequence once for every consumer. For a coincident region, form the
   domain arrangement and select one supported covering region; do not remove it merely
   because another triangle passes nearby.
3. Tessellate the resulting trimmed domains with fixed shared edge indices and adaptive
   interior samples. A curved replacement may contain multiple patches and internal creases;
   it need not admit one centroid, projection direction or planar parameterization.
4. Validate the complete replacement against geometry and against the neighboring mesh before
   committing. Check directed edges, vertex links, overlaps/intersections and retained boundary
   identities. Reject the replacement atomically on failure, with its unresolved obligation.

**Exit:** compared with the frozen candidate, the replay fixture acquires supported, conforming
surface without a proximity band or arbitrary fan; its independent section checks and applicable
spatial obligations pass. Perturbing seed positions or sheet order does not change the accepted
geometry/topology outside the declared error. Closing a loop alone does not establish success.

This is the decision point for the architecture. Extend shared-boundary construction only
after this comparison demonstrates supported geometry and stability. If source patches cannot
provide the needed evaluations or event isolation, specify that missing capability; if the
validator cannot resolve the replacement, report that separately from construction failure.
Use the bounded spatial-cell fallback experiment below only for an identified missing capability.
Do not conceal either limitation behind another downstream weld rule.

### Phase 3 — complete this cylinder, then test generality

Extend the successful replacement mechanism to the remaining unsupported regions, including
failures away from rims identified in the current report. The historical count of 160 such
failures is not the new backlog. Remove the corresponding repair
paths as their responsibilities move into common boundaries. Keep supported existing surface
where its evidence survives the change; invalidate evidence whenever geometry changes.

Make the cylinder acceptance case an ordinary test once it passes. Promote a focused subset
of diagnostic counterexamples into assertions rather than running thousands of lines of
historical printouts as the acceptance suite.

Restore positive coverage incrementally: first the independent curved controls in Phase 0a;
then simple stationary/translated sphere and prismatic sweep cases as their mechanisms become
supported; then the complete cylinder; then the remaining turned box/prism/lens, whole-turn
and Boolean-tool cases. Each restored case must pass the current acceptance contract, rather
than simply re-enable its old centroid-only assertion. Track construction recovery separately
from validator calibration and keep the permanent negative controls active throughout. Update
the recovery ledger as tests are restored or superseded; justify any change in recovery order
by the mechanism actually being implemented.

**Cylinder exit:** closed oriented topology with valid vertex links; no geometric
self-intersections; zero failed and zero unresolved surface obligations; independent section
and volume agreement with stated numerical error; forward and reverse coverage at the
requested spatial tolerance; and checked float32 export. Report actual evidence strength
rather than calling centroid samples a whole-surface certificate.

**General-sweep exit:** the same mechanism survives the turned box, whole-turn box, prism,
lens and dumbbell, plus nearby cylinder angles, dimensions, axis perturbations, scale changes,
reversed motion and finer/coarser tolerances. Add cases for endpoint contact, repeated coverage,
concave Boolean-tool features and changes in contact topology. These should exercise shared
mechanisms, not introduce fixture-specific fill dispatch.

Arbitrary solids can have disconnected material, cavities and singular boundary contacts.
Define which are representable as accepted shells and which produce an explicit unsupported
or unresolved result. One connected manifold shell cannot be a universal success condition
for every possible Boolean tool and motion. Do not quietly erase components to meet it.

### Phase 4 — reconnect the work to hypoid gears

The CLI still uses the older candidate-slab arrangement in
[mesh_sweep.rs](../rust/gcs-cli/src/cad/mesh_sweep.rs); this review found no CLI consumption of
`swept_boundary::construct`. Passing the cylinder does not yet change normal gear export.

Use a vertical sequence with an executable result at every step:

1. Build one general cutter sweep through the new accepted-boundary API.
2. Subtract that accepted sweep from a blank using the reliable-Boolean work, with the same
   field agreement, topology, embedding and error accounting applied to the result.
3. Exercise one actual hypoid tooth space at the known failing offset angles before scaling
   up to indexed cuts. Retain 15°/30°/45° witnesses from the original failure family where
   their source fixtures are available; reconstruct and record them if necessary.
4. Export both indexed members through ordinary Solvent/CLI paths. Validate the encoded output
   and actual assembly placement. Distinguish surface accuracy, mating contact and interference
   evidence; a valid standalone swept solid proves none of the pair-level conditions by itself.

The [gear roadmap](spiral-bevel-roadmap.md) contains useful earlier CAD and engagement evidence,
but those historical candidates do not automatically validate new hypoid offsets or a changed
export backend. Record which evidence transfers and rerun what depends on the new geometry.

## Alternatives and limits

| Approach | Decision |
|---|---|
| More fan, weld, diagonal or tolerance rules | Do not make this the main path. They cannot reconstruct absent support/trim identities, and several variants are already refuted. |
| Add a field veto to every zip triangle | Useful containment and diagnosis after repairing the judge. Insufficient as a construction method: deleting bad bands already expanded seven loops to 119. |
| Restart full uniform field contouring | Keep as an independent coarse diagnostic, not the production strategy. Earlier fine gear extraction was too expensive; ambiguous corners and thin features remain real obligations. |
| Shared patch arrangement with local replacement | Recommended first implementation. Reuses the candidates and cuts while directly addressing the lost geometric relationships. Requires robust overlap/event handling; success is to be tested at Phase 2. |
| Adaptive spatial-cell backend | Bounded fallback experiment if a neighborhood lacks usable charts. It needs a conforming interface to the surrounding mesh, strict witnesses, multi-crossing/feature handling and coverage checks. This is not the already-failed act of invoking the existing closed-shell extractor on a loop's bounding box. |
| External CAD kernel | Existing trials remain useful comparisons or an explicitly chosen alternative delivery route. Kernel closure/validity alone cannot replace material agreement, as the history already demonstrates. A native-only dependency also changes the one-core WASM target. |

The earlier [field extractor](field-boundary-extraction.md) contains a sound architectural
idea worth reusing: spatial cells carry strict inside/outside witnesses and both directions
of distance coverage, rather than trusting residual magnitude. Its implementation still has
documented ambiguity and cost limits. Shared-edge contouring can make topology conforming;
it does not by itself prove preservation of every thin feature.

For patch validation, a small cell containing opposed witnesses provides a conservative
distance bound for every triangle point in that cell. Alternatively, subdivided parameter
domains can transfer certified surface-approximation bounds. Either needs a separate reverse
coverage obligation to exclude missing boundary. Merely sampling more centroids is useful
diagnostics, not a substitute for these whole-region bounds. Exact-boundary queries may stay
ambiguous; refine or use off-boundary spatial witnesses, and retain an explicit refusal when
finite budgets cannot resolve the obligation.

## Working discipline that prevents another loop of inconclusive repairs

- Keep one current status matrix, separate from the chronological lab notebook. Give every
  row a source/options hash, acceptance level, outstanding obligation and next discriminating
  experiment. Preserve retired hypotheses as history rather than copying them into invariants.
- Make diagnostic results inspectable even when construction refuses. Persist a small replay
  fixture for each architectural change and state its negative control before implementing it.
- Retain bounded partial audit evidence and mark unattempted checks explicitly. Report safety
  regressions and successful-construction milestones separately; a refusal suite is not a
  measure of recovered sweep support. Update baseline counts when geometry improves instead
  of requiring the old failure mode to persist.
- Track unsupported and unresolved **area**, coverage error and geometric/topological failures
  separately from loop and triangle counts. Use closed-shell volume only after closure.
- Render the changed region with source ownership and failure overlays. Repeatable views and
  independent section overlays are more informative than a new aggregate number alone.
- For behavior-preserving refactors, require byte-identical five-case exports as the handoff
  prescribes. For intentional geometry corrections, record the expected changed seams and
  validate them independently; retain byte parity for unaffected cases. The old wrong mesh
  cannot serve as the expected geometry of its own fix.
- Run `cargo test --manifest-path rust/Cargo.toml -p gcs-core --test core`, never `--release`,
  after library changes, plus the relevant formerly ignored acceptance cases. A green suite
  with the target ignored is an incomplete milestone. Measure one-core time and field-query
  counts after correctness holds; reuse spatial indexes and bounds before adding refinement.

The next implementation deliverable is **Phase 2's shared-boundary experiment against the
corrected Phase 1a baseline**. Begin before the unsafe crease identification, retain the
frozen counterexamples, and establish a validated interface before narrowing the interacting
patch group. The construction decision gate remains in place before expansion toward general
sweeps and hypoid exports.
