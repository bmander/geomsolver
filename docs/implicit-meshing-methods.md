# Implicit surface extraction: method selection

The current field extractor is a conservative baseline, not a decision to build a new
general-purpose meshing method. Its pinion experiments expose two separate costs: locating
the continuous generating minimum at spatial queries, and representing thin connections
with a uniform finest surface grid. Existing implicit-surface and F-rep research addresses
both. This comparison redirects the next extraction experiment before more grid-specific
patches. The final matched-pair requirements remain unchanged.

## Relevant methods and their contracts

| Method | What it supplies | What Solvent still needs to establish |
| --- | --- | --- |
| [Marching Triangles, Hilton et al. (1996)](https://openresearch.surrey.ac.uk/esploro/outputs/conferencePresentation/Marching-Triangles-Range-Image-Fusion-for/99513651502346) | Follows an implicit surface and builds a triangulation using a local three-dimensional Delaunay constraint. This is an established surface-based alternative to volumetric polygonization. | Supply reliable seeds and local surface projection; handle advancing-front closure, sharp features and discovery of other components. The surface-following strategy alone is not a complete-domain certificate. |
| [Dual Contouring of Hermite Data, Ju et al. (2002)](https://www.cs.rice.edu/~jwarren/research/index.html) | Surface intersections and normals position vertices near sharp features; adaptive octrees reduce density in simple regions. | Finding the correct intersections and all features precedes contouring. A local quadratic fitting error is not our two-sided surface-distance certificate. |
| [Manifold Dual Contouring, Schaefer, Ju and Warren (2007)](https://people.engr.tamu.edu/schaefer/research/dualsimp_tvcg.pdf) | Multiple components per cell and constrained vertex clustering retain manifold contours during adaptive simplification. | Preserving the sampled contour does not establish equivalence to the continuous material. Geometric self-intersections and missed thin features require separate handling. |
| [Plantinga–Vegter (2004)](https://pure.rug.nl/ws/files/2952308/2004ProcGeomProcPlantinga.pdf) | Interval bounds on values and gradients drive balanced subdivision, with an isotopy proof for smooth bounded regular implicit surfaces. Isotopy preserves the embedding through continuous deformation. | Our min/max Boolean and swept fields are not globally smooth. Active-branch changes, sharp edges and zero-only sets do not automatically satisfy the paper's hypotheses. Using interval arithmetic alone does not inherit its theorem. |
| [Swept Volumes via Spacetime Numerical Continuation, Sellán, Aigerman and Jacobson (2021)](https://www.dgp.toronto.edu/projects/swept-volumes/swept-volumes-low-res.pdf) | Reuses generating-time minima between neighboring surface locations, following the boundary in space and time. It directly supports carving by swept volumes. | Section 4.3 explicitly allows sequences of missed global minima. Use continuation as candidate generation or search acceleration; retain complete-domain bounds to establish exposure and coverage. The source need not be treated as a signed distance field in our verifier. |
| [libfive/Fidget](https://www.mattkeeter.com/projects/fidget/) | Existing F-rep implementation experience: interval exclusion, region-specific expression simplification and manifold dual contouring. | The author explicitly records thin-feature loss, self-intersections and adversarial vertex placement. Reusing a renderer does not by itself certify the exported gear. |
| [Adaptive Delaunay Scaffolding, Takikawa et al. (2026)](https://www.cs.ubc.ca/labs/imager/tr/2026/ads/) | Progressively refines occupancy-crossing edges in a Delaunay scaffold and surfaces them with marching tetrahedra, with reported favorable accuracy/query-count tradeoffs. | The project abstract supplies experimental evidence, not a whole-gear completeness proof. Evaluate as another candidate method; do not infer that initial occupancy samples find every component. |

Matt Keeter's [July 2026 discussion](https://www.mattkeeter.com/blog/2026-07-03-meshing/)
also distinguishes sharp features, thin features, manifoldness, embedding and adaptivity.
Its proposed separation of surface-point generation from mesh construction is useful design
guidance, but the proposed method is explicitly an unimplemented sketch, not an established
algorithm we can claim to have adopted.

## Consequences for this pair

The generator, motion and Boolean construction contain more information than a black-box
occupancy function. The existing analytic flanks, fillets, boundary intersections and roll
witnesses can propose surface points and normals. They must remain candidates until the
complete indexed material evaluator establishes exposure: local envelope contact can be
covered by another generating roll or tooth index. Conversely, those candidates alone cannot
establish that no other material boundary exists.

The current one-Lipschitz field contract provides conservative value bounds. It does not
provide a lower gradient bound, a feature-size bound, a normal at a Boolean tie, or an
inverse relation from field residual to surface distance. Adding finer samples cannot turn
any of these missing hypotheses into a theorem. These are evaluator/verification contracts
to supply where a selected meshing algorithm actually requires them.

The next comparative experiment should therefore separate:

1. **Candidate discovery:** trace exposed generating branches in space and roll, reusing
   neighboring witnesses. Include blank surfaces, roll endpoints and branch transitions.
   Existing analytic work supplies seeds without manually declaring the finished shell.
2. **Adaptive mesh construction:** benchmark a documented contouring method against the
   baseline on the same field. Require local refinement around thin connections and sharp
   features; do not merely simplify a uniformly over-refined final mesh.
3. **Acceptance:** independently check complete material coverage, both spatial-error
   directions, encoded topology and geometric embedding. A fast candidate generator may
   be imperfect without weakening this acceptance step. Rejection must direct refinement
   or identify a missing hypothesis, rather than repair geometry by dropping fragments.

Use the actual isolated-pinion connection, a swept sphere/torus, sharp Boolean intersections,
a hidden component and `A-A` as common fixtures. Compare field evaluations, runtime, memory,
triangle count and acceptance evidence at the same spatial tolerance. The 2 mm baseline was
stopped after the user identified its runtime as unacceptable; the results below supersede
the earlier plan to run it to completion. Do not launch successively finer full-grid runs
as the default next action.

This is a method-selection checkpoint, not an assertion that a combined algorithm already
has the cited guarantees. No external mesher has been integrated, and neither the functional
pinion export nor the full gear pair has been accepted yet.

## Rejected cost model and hint-cache experiment

The 2 mm experiment's last progress report was 479,950 visited partition cells at 1,365.8 s.
It had not reached triangulation. It was deliberately terminated, not rejected by a geometric
check, and produced no accepted STL. The generating geometry was not changed.

The tolerance is a requested final spatial bound, not a grid spacing. The baseline forces
every meshed cell to the same finest level, whose diagonal is at most tolerance/4; dyadic
rounding makes this pinion's spacing about 0.24 mm. Crossed-edge vertices are midpoints.
Thus even broad smooth areas pay for fine grid cells instead of accurate surface fitting.
In addition, each point query evaluates every indexed cutter, refining swept minima even
where other operands already determine the material classification. These are deficiencies
of this implementation, not evidence that F-rep extraction inherently takes tens of minutes.
Triangle count should be measured against surface deviation and feature preservation, not
inferred from the tolerance label or forced finest-grid spacing.

Commit `03b45b2` preserves an opt-in experiment that reuses nearby roll parameters as initial
samples. The oracle re-evaluates every hint and retains its complete roll cover. It passes
torus, abrupt branch-change, competing-minimum and budget/observer checks. The pinion test
uses a 6 x 6 x 6 grid with positions
`p[a] = c[a] + (index[a] - 2.5) * 0.2`, where
`c = (34.576585140122586, -7.409268244311984, 46.92536554730923)` mm. Both runs use band `[0,0]`,
value tolerance 0.0002 mm and a 20,000-evaluation per-sweep budget; the warm run keeps at most
256 parameter hints per swept node. Both establish 119 retained and 97 exterior points.
Cold evaluation uses 314,792 roll queries; hint reuse uses 314,640, a reduction of only 0.0483%.
The combined comparison takes about 1.66 s on the test machine. This is not an export timing.

The following commit removes the experiment from the runtime: its effect on the real input
does not justify another cache and public control. Preserve the negative result in history.
Remembering a good attained minimum mainly helps upper bounds; proving that a retained point
survives every generating roll still needs global lower bounds. This helps explain why
parameter hints alone do not solve the pinion's evaluation cost.

The next implementation should follow the user's seed / surface march / edge refinement
outline. Seed from existing generating-geometry witnesses, project onto exposed material,
advance over the surface with locally chosen spacing, and refine edges by geometric error.
Use the bounded evaluator to validate candidates and search for missed components. Develop
the surface traversal and its acceptance procedure separately so that a costly exhaustive
partition does not remain the only way to construct every candidate triangle.

## Spherical contour and chart experiment

**Method correction:** the full shell below is a structured spherical-chart mesh with
local edge subdivision. It is not an advancing-front marching-triangles implementation.
The one-dimensional contour does trace and correct neighboring points, but the shell
starts with prescribed periodic connectivity and end rows. Earlier descriptions calling
the shell "surface-following" overstated what was implemented. The user's inspection of
the regular, rasp-like triangulation exposed this distinction and its missing feature
sampling. The chart remains a measured candidate baseline, not the selected final mesher.

`tests/envelope/paired/swept/member/walk.rs` now follows the tooth-facing contour on a
spherical section of each complete member. It seeds from an existing analytic flank point,
advances in azimuth using the preceding contour slope as a predictor, and corrects in polar
angle using the complete indexed `MaterialField`. A narrow bracket around the predictor
expands until it contains strict retained/exterior signs. Correction shrinks that bracket;
ambiguous field values are not assigned a sign. The point query still uses the existing
whole-roll evaluator, with a 0.0002 mm value tolerance and 20,000-evaluation sweep budget.

Each output vertex is evaluated at the angular midpoint of two strictly classified points.
An outward-rounded distance from that vertex to both endpoints bounds a ball containing
the whole straight segment, and therefore an actual material boundary crossing. This
establishes a vertex-to-boundary distance for the explicit field snapshot. It does not
assume that field values are distances or that rounded chart coordinates lie exactly on
the sphere. The periodic seam shares the same witness and vertex identity. The original
contour experiment used Cartesian midpoints; angular midpoints keep the following shell's
charts consistent without changing this enclosing-ball argument.

The initial azimuthal spacing is a quarter tooth pitch. A separately corrected midpoint
tests each candidate chord; edges subdivide when its deviation, including vertex bracket
radii, exceeds the requested refinement target. This is an adaptive candidate criterion,
not a bound for every point of the complete edge. The radial chart also does not yet prove
unique crossings or discovery of other components. Those hypotheses must be checked or the
chart must be split before using it as a complete surface representation.

Reproduce the ordinary one-pitch regression and explicit full-contour experiment with:

```sh
cargo test --manifest-path rust/Cargo.toml -p gcs-core --test core \
  surface_following_tracks -- --nocapture
SOLVENT_SURFACE_WALK_OUTPUT=/private/tmp/solvent-surface-walk.json \
cargo test --manifest-path rust/Cargo.toml -p gcs-core --test core \
  surface_following_tracks -- --nocapture
```

The regression uses a 0.1 mm chord-refinement target and 0.005 mm vertex brackets. The
original measurement, before the shell predictor changes below, needed
9 edges for one pinion pitch and 10 for one gear pitch; sampled distances to the separate
analytic contour are at most 0.00216 and 0.00239 mm respectively. This analytic comparison
is a sampled sanity check, not an independent whole-contour error certificate.

The full default-pair experiment uses a 2 mm refinement target and 0.1 mm vertex brackets:

| Member | Closed contour edges | Field queries | Roll evaluations | Tracing time |
| --- | ---: | ---: | ---: | ---: |
| Pinion | 96 | 1,965 | 2,240,720 | 3.956 s |
| Gear | 192 | 3,936 | 5,161,888 | 15.779 s |

The largest accepted midpoint criteria are 0.731 and 0.813 mm. These timings cover one
mean-radius contour per member, not a complete three-dimensional export or its validation.
They were measured after compilation and exclude the subsequent analytic-reference check.
The JSON retains the complete generating definitions, both sign witnesses for every vertex,
vertex distance bounds, query counts and unfinished verification scope.

## Candidate shell experiment

`tests/envelope/paired/swept/member/walk/shell.rs` extends the spherical chart across the
face width. A periodic annular scaffold connects the tooth-facing surface, back cone and
two spherical ends with shared vertex identities. Toe/heel radii and the back boundary
come from the existing solved definition. Initially there are four azimuthal intervals
per tooth and only the two end rows. No hand-authored tooth seam network defines material.

At the end spheres the complete clipped field is zero throughout each cap, so it cannot
give the strict polar-angle signs needed to locate the tooth/end intersection. The
workbench therefore also evaluates the same tip/back slab minus all the same indexed
sweeps, omitting only toe/heel spherical clipping. It corrects the tooth-facing chart in
this continuation and intersects the chart with the prescribed end radii. This does not
change the full member field or its serialized definition. The output explicitly labels
its crossing evidence as continuation evidence; it is not a certificate for the clipped
member's cap or end edge.

Previously corrected points at the same tooth-relative azimuth provide the next polar
angle predictor. Periodicity is used only to choose a guess: every new spatial point is
classified against all indexed sweeps, with no copied sign or roll certificate. The
initial search bracket has the requested vertex-distance scale and doubles until both
strict signs are established.

Every mesh edge gets a separately evaluated chart midpoint. The candidate indicator adds
its distance to the straight edge and the crossing-radius allowances; an edge exceeding
the target splits in both incident triangles. Requests propagate to the longest edge of
each affected triangle before replacement. The 0/1/2/3-edge triangle split cases retain
orientation and conformity, choosing the shorter remaining diagonal in a two-edge split.
Unchanged edges reuse their midpoint samples. Limits are
16 refinement rounds, 20,000 vertices and 50,000 full-field queries per member. Failure
does not discard fragments or publish an accepted mesh.

The first split rule hit its 16-round limit after 92 seconds on the pinion: it repeatedly
created skinny triangles, with a long edge approaching the parameter interval [1/3,1]
instead of shrinking, and a midpoint indicator remaining near 4 mm. Choosing a shorter
diagonal alone did not solve it. A twisted annular fixture reproduces this failure cheaply.
Longest-edge request propagation addresses that cause. This uses the established
[longest-edge refinement principle](https://onlinelibrary.wiley.com/doi/abs/10.1002/nme.1620200412);
our projection onto a curved chart and batch triangle splits do not inherit a planar
refinement theorem or prove convergence for arbitrary fields.

Analytic annular-cylinder and twisted-ridge fixtures check refinement, known volume,
connected genus-one topology and encoded-STL topology. Actual pair construction is opt-in:

```sh
SOLVENT_SURFACE_SHELL_OUTPUT=/private/tmp/solvent-walk-pair \
cargo test --manifest-path rust/Cargo.toml -p gcs-core --test core \
  export_surface_following_pair_candidates -- --nocapture
```

`SOLVENT_SURFACE_SHELL_TOLERANCE` selects the sampled refinement target in millimetres
(default 2). Each member's candidate STL has an adjacent JSON with the complete original
definition, chart parameters, mesh, continuation sign witnesses and cost measurements.
Full-edge/surface error, chart uniqueness/completeness, geometric embedding, source
accuracy and mating verification remain required for acceptance. The scaffold assumes
an annular chart; passing its topology check cannot prove the material has that topology.

The default 24:48, module-2 pair now completes at the 2 mm sampled target, with 0.1 mm
crossing brackets:

| Member | Triangles | Vertices | Refinement rounds | Field queries | Roll evaluations | Construction and STL checks |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Pinion | 2,234 | 1,117 | 2 | 8,134 | 12,753,188 | 25.131 s |
| Gear | 4,556 | 2,278 | 3 | 15,490 | 29,389,788 | 97.033 s |

The final maximum midpoint indicators are 1.287 and 1.331 mm. These are complete candidate
shell timings after compilation, including runtime mesh/STL checks, but not whole-surface
acceptance or the separate Python audit. The standalone `verification/stl_topology.py`
also checks both encoded files: each has Euler characteristic zero, genus one and positive
signed volume (9,082.5125 and 20,274.3613 mm3). Its malformed-shell self-tests pass. These
topology checks do not check geometric self-intersections.

Before using tooth-relative predictors, pinion construction took 62.655 s / 28,700 field
queries / 35,014,484 roll evaluations for 2,242 triangles, and the gear exhausted its
50,000-query limit. Predictor reuse reduced search work without raising that budget or
copying evidence between indices. The rendered outputs are visibly coarse, especially at
tooth edges. The 2 mm label remains a sampled refinement target, not a production tolerance.

Verification for this workbench change: 1,056 core tests passed, one ignored. The regenerated
schema-5 complete-member evidence is exactly equal after JSON decoding to the prior artifact;
the full field definition and selected material checks are unchanged. Runtime core and
native/WASM bindings are unchanged by this test-only extraction experiment.

A subsequent [exact encoded-mesh audit](stl-embedding-verification.md) establishes that
both coarse STLs are embedded, outward-oriented closed shells. It does not establish
tooth-edge fidelity or distance to the intended material boundary.

## Next method: general curvature-adaptive advancing front

The user explicitly rules out tracking tooth tips or supplying gear-specific feature
curves. The mesher must work on other geometries: its inputs are a field and accuracy
controls, with geometry-independent query/projection capabilities. The gear is a test
case, not a source of special meshing rules. Any feature discovery must come from local
surface behavior and a general search, not names such as flank, tooth tip or root.

The complete member already is an F-rep: its finite blank minus every indexed continuous
swept cutter. The current chart extractor nevertheless treats its tooth-facing, back and
end regions differently. The user explicitly rejects that decomposition in the next
mesher. One algorithm must traverse the entire boundary of the full `MaterialField`,
including all sharp/smooth transitions, without separate analytic caps/back surfaces or
the end-clipping continuation used by the chart experiment. Front closure must emerge
from traversal rather than an assumed annular connectivity. Internal evaluator optimizations
may use the expression graph, but the mesher must not require geometry-specific regions.

The user's proposed seed / outward growth / finer sampling at increasing curvature has
direct precedents:

- [Karkanis and Stewart (2001)](https://www.dgp.toronto.edu/public_user/JamesStewart/papers/cga01.pdf)
  start a seed triangle and grow triangles with curvature-dependent size, followed by
  gap filling. They assume a connected G1 surface and access to gradients. Their failure
  discussion identifies rapid curvature changes and incompatible triangle sizes where
  fronts meet. This is much closer to the requested march than the current chart mesh.
- [Akkouche and Galin (2001)](https://www.ljll.fr/~frey/papers/scientific%20visualisation/Akkouche%20S.,%20Adaptive%20implicit%20surface%20polygonization%20using%20marching%20triangles.pdf)
  develop adaptive marching triangles. Their conclusion leaves detection of gradient
  discontinuities in Boolean implicit models as further work, so it does not by itself
  solve this gear's creases.
- [Schreiner et al. (2006), Direct (Re)Meshing](https://publications.sci.utah.edu/publications/SCITechReports/UUSCI-2006-013.pdf)
  use a spatial sizing field to prepare the front for smaller features ahead, rather
  than depending only on curvature at its current vertices. Section 3.3 resamples feature
  curves and starts fronts along them, preserving boundaries and CSG intersection curves.
  Curve and surface curvature share the sizing field. Their smoothness, projection,
  sampling and front-connection limitations still need explicit treatment here.

The implementation direction is now:

1. Discover surface seeds through generic queries of the complete material field. No supplied tooth curves,
   parameter-grid connectivity, or named analytic gear patches guide triangulation.
2. Estimate local shape from projected samples and normal variation. Smooth regions use
   curvature-dependent spacing; nonsmooth regions require general detection of incompatible
   one-sided normals and local refinement/feature reconstruction. No globally averaged
   normal may silently smooth a crease. High-curvature valleys need sampling even when
   normals remain continuous. An edge-midpoint-only test can miss both kinds of feature.
3. Build a sizing field that accounts for nearby curvature and surface separation before
   the front reaches them. Curvature at the current point alone can miss a narrow valley
   between probes; arbitrarily large steps on flat portions are not justified.
4. Grow an actual frontier from discovered seed points: tangent-plane prediction,
   projection onto exposed material, size/shape control, and explicit front meeting,
   merging and closure. Do not preassign spherical lattice connectivity. A single seed
   still does not establish discovery of every material component. The same traversal
   crosses all boundary regions; no separately meshed end or back is attached afterward.
5. Validate the same method on a sphere, torus, sharp Boolean intersection, rounded narrow
   groove, nearby sheets and a hidden component, then on the gear field. Check feature
   fidelity, triangle quality and convergence before producing another full pair.
   Keep exact STL embedding checks and
   whole-boundary coverage/error acceptance independent of candidate generation.

The current field supplies bounded values, but not the smooth branch normals, curvature
or reliable local projection assumed by these methods. Those contracts need a generic
evaluator or sampling procedure. Merely estimating one normal across a Boolean crease
would hide the feature the new mesher must preserve. Schreiner's supplied-feature-front
extension is relevant background, but it is not authorization to introduce tooth-specific
curves into this implementation.

## Small-case discovery workbench and latency target

Keep high-precision gear extraction out of the iteration loop until small cases work.
The next target is **under 100 ms per complete sphere or cube extraction**, on the
development machine with the existing optimized test profile. Measure from evaluator
creation through seed discovery and front closure. Compilation, fixture construction,
independent validation and file I/O are outside that interval. Report triangle count,
point-query count, open frontier edges and accuracy alongside time; a fast incomplete
mesh does not pass. Timing is an opt-in serial check, not a flaky parallel-suite assertion.

`rust/gcs-core/tests/functional_solids/front.rs` is a test-only advancing-front
experiment. A bounded search finds a strict material interior point; a bracket toward
the exterior seeds tangent prediction and field correction. Local normal variation
controls candidate spacing, with sample-to-boundary checks at each triangle's edge
midpoints and centroid. Front connections and small discovered-loop fills determine
connectivity. There are no supplied primitive charts or separately attached caps.
Strict inside/outside witnesses reject zero-only Boolean loci as seeds. Exact-coordinate
correction/projection caches reuse immutable field results, including failed attempts;
they are bounded and do not quantize coordinates. A cached/uncached comparison checks
that the sphere's vertices, witnesses and triangles remain identical.

At tolerance 0.02, the radius-one sphere closes with 414 triangles, 36,909 point queries
and 56–64 ms extraction time in local serial runs. Before reuse, the same geometry
took about 162 ms and 106,560 queries (before adding the seed-triangle sample check).
The maximum independently sampled triangle deviation is 0.019900. These samples and
vertex witnesses do **not** certify whole-triangle error or complete material coverage.
The separate exact embedding audit passes for the exported 414-triangle sphere
(SHA-256 `3207dae5febfef179bc00b3572b51e7374b7b0c1676966f81a596956e876a5b5`),
establishing an outward-oriented embedded closed mesh, not field coverage or accuracy.

Two explicit acceptance tests remain ignored by default because they currently fail;
they are neither passing regressions nor accepted meshes:

- The side-two cube is constructed as six half-spaces intersecting a radius-two ball
  that supplies finite support. The marcher only receives the complete material field.
  It takes about 1.21 s, leaving 218 open edges on 1,146 triangles after 83,289 queries.
  Generic sharp-edge discovery and reconstruction are missing; averaged finite
  differences are insufficient. Before reuse this incomplete attempt took 7.78 s.
- The major-radius-two, minor-radius-0.6 torus takes about 259 ms, leaving two skinny
  triangular gaps on 1,896 triangles. Local retriangulation at front meetings is needed;
  the shape threshold has not been relaxed just to make closure pass.

Run the ordinary sphere, cache-equivalence and empty-material controls:

```sh
SOLVENT_FRONT_MAX_MS=100 cargo test --manifest-path rust/Cargo.toml \
  -p gcs-core --test core generic_front -- --nocapture --test-threads=1
```

Run the **currently failing** acceptance cases explicitly:

```sh
SOLVENT_FRONT_MAX_MS=100 cargo test --manifest-path rust/Cargo.toml \
  -p gcs-core --test core generic_front -- --include-ignored --nocapture --test-threads=1
```

`SOLVENT_FRONT_OUTPUT=/private/tmp/solvent-general-front` optionally exports only
candidates that pass closure, topology, witness and sampled-error checks. The latency
limit applies to sphere and cube, not the separate torus diagnostic. Repeat serial runs
to assess timing variability; do not count compilation or a stalled front as discovery.
Next resolve generic sharp transitions and front-band retriangulation on these fixtures,
then expand to narrow grooves, nearby sheets and undiscovered components. This workbench
does not yet supply the final production mesher or a gear acceptance certificate.
