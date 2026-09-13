# Swept boundary: architectural diagnosis and recovery plan

2026-09-12. Reviewed at `5e95019`, including the existing uncommitted changes.

## Recommendation

Keep the continuous material field and analytic sweep candidates. Replace the boundary
construction's reliance on independently trimmed triangle soups followed by proximity-based
repair with **shared, geometrically supported boundaries that are established before final
tessellation**. Begin with a bounded replacement of the tumbling cylinder's failing surface
neighborhoods, rather than a whole-kernel rewrite.

Before doing that, repair the meaning of the field judge and acceptance result. There is a
mathematical error in the foundation of `Sign::Near`, and the current certificate can report
completion while leaving triangles uncertified. A stronger surface generator judged by the
same permissive gate would repeat the earlier hypoid failure: successful construction without
adequate evidence that the constructed surface is the intended one.

The remaining seven loops are a symptom, not a sufficient definition of the remaining work.
Closing them cannot by itself finish this cylinder, general sweeps, or hypoid gears.

## Evidence and scope of this review

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

**Exit:** negative controls are refused; accepted simple controls have real spatial evidence;
the cylinder's report shows its full outstanding obligations without waiting for closure.
Expect some previously green cases to become unresolved. Record this as correction of an
overstated guarantee rather than weakening the new rule to preserve their old status.

### Phase 1 — give the cylinder an independent, continuous oracle

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

This derivation is a proposed independent test oracle, not implemented or numerically audited
here. Validate its zero-roll limit, symmetry, end sections and full-turn limit first. For a
full turn, sections are disks of radius `sqrt(2 - (x-3)²)`, giving total volume `10π/3`.
Then use it for section overlays, material-side probes and convergent volume quadrature of
the ±30° fixture. Keep it out of the production mesher so a cylinder-specific formula cannot
be mistaken for general sweep support.

Instrument actual `rim_zip` decisions and the earlier affected cuts with stable provenance,
as the handoff requests. Reuse one reporter; record the candidate before mutation, its field
witnesses, incident patches and the actual result. Include reversed regions away from loops.
Do not infer missing area from nearest-vertex distance. Use distances to triangles, explicit
cross-sections and area-weighted coverage at a resolution that resolves the defect.

**Exit:** the independent oracle distinguishes the current defective geometry from a correct
reference, and one bounded neighborhood has a replayable account of its required surface and
its first unsupported construction decision.

### Phase 2 — replace one neighborhood with shared boundaries

Choose one failing junction together with all incident sheets, overlapping fragments and
nearby unsupported bands. Its outer boundary must lie on validated surface. If it does not,
expand the neighborhood; the seven existing rim loops are not immutable constraints.

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

**Exit:** the replay fixture acquires supported, conforming surface without a proximity band
or arbitrary fan; its independent section checks pass; perturbing seed positions or sheet
order does not change the accepted geometry/topology outside the declared error.

This is the decision point for the architecture. If source patches cannot provide the needed
evaluations or event isolation, stop and specify that missing capability. Do not conceal it
behind another downstream weld rule.

### Phase 3 — complete this cylinder, then test generality

Extend the successful replacement mechanism to the remaining unsupported regions, including
the 160 reported failures away from open-loop vertices. Remove the corresponding repair
paths as their responsibilities move into common boundaries. Keep supported existing surface
where its evidence survives the change; invalidate evidence whenever geometry changes.

Make the cylinder acceptance case an ordinary test once it passes. Promote a focused subset
of diagnostic counterexamples into assertions rather than running thousands of lines of
historical printouts as the acceptance suite.

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

The next implementation deliverable should therefore be **a corrected evidence/acceptance
contract and a replayable, independently judged cylinder neighborhood**. The following
deliverable is its supported shared-boundary replacement. That makes each step capable of
settling a specific architectural question and keeps the route to actual hypoid exports visible.
