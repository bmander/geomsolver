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
triangle count and acceptance evidence at the same spatial tolerance. Preserve the running
2 mm pinion baseline to completion or explicit resource refusal; do not launch successively
finer full-grid runs as the default next action.

This is a method-selection checkpoint, not an assertion that a combined algorithm already
has the cited guarantees. No external mesher has been integrated, and neither the functional
pinion export nor the full gear pair has been accepted yet.
