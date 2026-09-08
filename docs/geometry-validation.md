# Geometry validation and refactoring

The spiral-bevel work was paused for this cleanup. The failures seen so far belong to
different layers; treating all of them as convergence problems obscures the contract
that actually failed.

| Problem | Cause | Where the invariant belongs |
| --- | --- | --- |
| A source reports success but leaves a gap at a shared arc vertex | Interactive acceptance is coarser than the downstream geometry tolerance; it also suppresses the fallback solver | The solve request specifies its hard-residual acceptance limit; that same limit controls success and retry |
| A transformed trial is mistaken for a generated surface point | Evaluating a moving surface does not establish the envelope equation | A checked envelope evaluator establishes the locus before material trims are applied |
| A seam can carry two copies of the same generating face | Parallel envelope and optional-patch storage duplicates sources and acceptance logic | Each operand owns one envelope with its material conditions; seam validation reads that exact snapshot |
| A seam root satisfies one face but narrowly misses the other | Only the first envelope equation participates in the numerical solve | Solve both face equations and the section together; the endpoint coordinate is fixed structurally |
| A valid snapshot can acquire invalid bounds | Public mutable domain fields bypass constructor validation | Snapshot geometry is private; domains are returned by value and source edits require a new snapshot |
| A section root lies on an undeclared continuation | A supporting equation says nothing about finite-patch membership | Final intersection acceptance checks finite incidence as well as the original equations |
| A zero-residual family is mistaken for a unique intersection | A stationary family or redundant sections leave free directions | The common intersection solver checks local rank, including zero-residual seeds |
| A sampled sweep misses a thin removed region | No sample enters a narrow minimum between roll positions | Global interval refinement retains every unresolved time interval; exhausted work returns uncertainty |
| Attained witnesses are mistaken for an independent no-cut proof | A positive sample cannot exclude an unseen negative interval | Require a complete positive roll cover for every index; independently recompute each cell's lower bound and reject every coverage gap |
| A bounded sweep is called certified despite approximate field values | Motion coverage and point-evaluation accuracy are different contracts | Use interval field evaluation; the explicit functional generator now does this, while transfer from solved coefficients to intended constraints remains separate |
| A Boolean field's zero set is mistaken for a solid boundary | Zero may occur without material, as in A minus A | Define material as closure({f<0}); extraction must establish material and exterior instead of wrapping every zero |
| A flank test accidentally reaches an inactive generator base | Opposed profile edges have different parameter directions | Read the actual endpoint coordinate from the declared join instead of assuming u=1 means the fillet |
| Valid active flanks are overcut by an indexed construction solid | Inactive closure walls are silently treated as generating boundaries | Define a tooth space using its own and its neighbor's active sides; check complete roll intervals and indexed copies, retaining the full-crown overcut as a rejected candidate |
| A closure preserves flank samples but leaves unwanted material in the gap | Boundary incidence says nothing about the intervening material | Check the specified root floor and gap interior separately; selected successful witnesses still do not establish whole-space coverage |
| Shared immutable field nodes trigger exponential repeated evaluation | A compact DAG is recursively expanded as a tree | Memoize each spatial query by node identity and input box; differently transformed uses must retain distinct keys |
| A copied tooth index drops coordinate uncertainty | Ordinary floating-point rotation is treated as exact geometry | Carry the input box through interval index and generating poses before querying material |
| A bounded extraction crop silently omits a disconnected component | Exterior crop faces say nothing about geometry elsewhere | Derive the complete finite material support from primitives, Booleans and interval motion; refuse unknown support |
| Same-sign grid corners silently erase a small feature | Corner sampling does not cover a cell's interior | Retain the complete spatial partition and require mesh-distance evidence for every cell not proved uniform |
| A low field residual is reported as mesh accuracy | Boolean/swept fields need not remain signed distance | Prove both spatial distance directions using complete cells and strict material/exterior crossings |
| A local mesh-neighbor search is treated as a geometric limit | Residual-based uncertain cells need not touch meshed cells | Search wider before refusal, while checking every candidate with the same conservative spatial-distance bound |
| A limited sweep budget silently removes a tooth index from the Boolean | Only converged sweeps participate in classification | Every index contributes its retained enclosure; budget status remains explicit and sign is asserted only when bounded |
| Workbench cut arithmetic diverges from the reusable solid evaluator | Completed sweeps have no core composition owner | `MaterialField` owns static/swept Boolean composition and fixed indexing; its query report retains all contributing sweep bounds and statuses |
| Shared indexed sweeps conflate certificates or multiply pose storage | Geometry copies are confused with distinct input-box queries | Share pose caches by immutable swept-node identity, memoize results by node and box, and identify each evidence stream with its returned query entry |
| A surface/surface seam is read as an envelope boundary | Checking only the second operand's kind loses information | Classify both validated operands with `SeamKind`; unsupported corner charts fail explicitly |
| A regular contact equation produces a cusp in the tooth surface | Contact-locus rank does not imply rank of the generated position map | Bound the generated oriented area factor separately over each whole parameter domain; profile curvature participates in this check |
| A shared vertex has matching coordinates but a different identity | Coordinate welding substitutes numerical proximity for declared topology | Seam construction requires one shared source vertex, then separately checks geometric coincidence and tangency |
| A boundary on one joined face is checked at the other face's position | Seam coincidence can be coarser than the boundary's incidence tolerance | The junction retains both checked contacts; the corner checks incidence on the face that actually owns its boundary seam |
| An endpoint's parameter triple addresses the wrong incident face | A junction's first-face endpoint parameter is copied into its second face | Edge readers map endpoint charts through exact shared face identity and then check the resulting position |
| A spatial face is mistaken for a profile on the page | An absent plane conflates spatial support with the default page plane | `FaceSupport` separates inherited planes and named spatial supports; planar readers explicitly reject the latter |
| An oriented face loop samples a shared junction in the wrong chart | An edge's canonical chart is assumed to belong to either incident face | The face boundary reader converts the junction endpoint parameter and checks its own support incidence while preserving shared edge xyz |
| A closed supported boundary is mistaken for a valid disk | Identity and incidence do not establish an embedded interior | Boundary snapshots make no interior claim; a zero-area retracing loop is a regression counterexample for the still-separate face validity gate |
| A valid curve endpoint has a singular axial slicing equation | Axial progress can have zero derivative where a fillet meets the root tangentially | Edge endpoints reuse the checked named corner and its full defining constraints; numerical slice solves apply to interior points |
| A fillet and flank acquire duplicate corners at the same finite boundary | Direct seam membership misses incidence implied by their shared junction | The vertex reader recognizes the other exact junction face meeting the same boundary identity; no coordinate weld is needed |
| Paired edges and the expected Euler count hide pinched vertices or disconnected shells | Global counts do not establish local manifoldness or connectivity | `topology::ClosedShell` checks every vertex link and the face adjacency component before returning a checked shell |
| Float32 export creates a vertex pinch without collapsing triangles | Distinct nonadjacent vertices become the same encoded position | `mesh::stl_topology` reconstructs exact encoded identities and checks the file's shell after quantization |
| A sampled rim looks sound but lacks a global guarantee | Finite sampling is being asked to prove a continuous or topological property | Closed-face topology, regularity, interference bounds and export error need their own explicit contracts |
| Ordinary floating-point bounds are mistaken for an enclosure | Rounding and unspecified transcendental accuracy can exclude true values | Finite intervals round outward; trigonometric polynomials include explicit remainder bounds, with independent rational audits |

## Cleanup implemented

`SolveOpts::acceptance_tol` separates numerical stopping from acceptance. Its default
preserves interactive behavior. Analytic callers request tighter hard-row residuals;
the existing DogLeg-to-LM retry responds to that requirement. A caller no longer needs
to select LM as a workaround or interpret the iteration status as geometric accuracy.
Invalid acceptance limits fail before changing the sketch. Row-scaled residual accuracy
still does not imply a spatial error bound: seams retain their independent position and
normal checks.

The nine gear cases use an iteration target of `1e-16` and hard-row acceptance of `1e-12`.
The tighter iteration target matters: the small 28:49 case previously stopped with a
maximum scaled residual of `7.74e-15`, yet its two root-join envelope equations could only
reach about `9.51e-11` against a requested `2e-11`. Solving both equations exposes that
incompatibility. Refining the source resolves it without relaxing any geometric check.
These numerical targets are solve controls, not claims of machine-precision spatial accuracy.

`GeneratedEnvelope::evaluate` remains the trial evaluator needed by numerical searches.
`GeneratedEnvelope::at` checks the declared domain and envelope equation. The internal
`EnvelopePatch` combines one source snapshot with its material conditions and is used
by both standalone patches and seam operands. Material checks remain private so they
cannot be used as a point-on-surface predicate. Searches may cross material boundaries;
only retained roots must satisfy every condition.

Spatial corners now follow the same ownership rule. A two-boundary corner stores one
generating face, and a junction corner reuses the junction's two faces. It remembers which
face owns the boundary. A regression perturbs an arc radius within the allowed seam gap
and verifies that the finer boundary check still refuses the displaced face. Named corners
also keep shared identity separate from coordinate coincidence. The gear rim reuses its
24 named toe/heel corners instead of independently recomputing those endpoints.

Seams no longer store parallel arrays of envelopes and optional trimmed copies. Their
source tangency check, trial search and retained-point validation all read the same
operand snapshots. `SeamIntersectionOptions` exposes only the two actual unknowns,
source angle and roll, with one normal-velocity tolerance for both face equations and
a separate section tolerance. The seam's endpoint coordinate cannot become a numerical
unknown or be assigned an unrelated residual tolerance by a caller. Both face equations
participate in the same existing intersection solver, including its final residual and
rank checks.

Envelope roll bounds are private, matching the existing private
surface chart and motion geometry. Invalid trim tolerances are rejected before invoking
a section callback or starting a numerical solve.

## Verification boundaries

The independent gear equations remain independent. Deduplicating the reference model
with the implementation would remove a useful way of detecting shared mistakes.
The nine tooth-count/module cases exercise the strict solve contract followed by
independent seam, cone, sphere and mating-geometry checks. Small regressions distinguish
iteration stopping from acceptance, retained contacts from trials, and snapshots from
subsequent model edits.

This cleanup does not certify a production gear. Oriented face loops, full admissibility,
continuous interference bounds and tolerance-controlled export remain unfinished.
It also leaves the language's editable declaration graph separate from evaluated
snapshots; adding cache or revision machinery is unnecessary while snapshots are owned
and explicitly rebuilt after edits.

The initial cleanup passed 966 core tests (one existing test ignored), 246 browser tests
and the CLI/ABI suites. The subsequent topology, corner, finite-edge, interval,
surface-regularity, spatial-face, surface-intersection and functional-volume checks bring core coverage to 1030 passing tests,
with the same one ignored; all 247 browser tests and the CLI/ABI suites
also pass. Native and WASM libraries were rebuilt. Independent checks of the actual
float32 STL files still find one connected, oriented manifold shell for each member.
These tests preserve the independent reference equations and do not certify continuous
geometric regularity or an export error bound.

The subsequent [whole-interval branch work](interval-geometry.md) covers the nominal
circle/sphere and roll equations over complete parameter rectangles. The generated area
factor now also establishes nominal differential regularity, including the root fillets.
The independent checker audits that factor and its consistent orientation. The known
deep-section cusp is a negative control. An error bound transferring the solved source
to the intended geometry remains a separate requirement.

The continuous-volume investigation now also retains the indexed full-crown overcut as a
negative construction example. A replacement space formed from neighboring active sides
passes selected whole-roll boundary and interior checks, independently audited at its
attained witnesses. These experiment additions bring the core suite to 1,032 passing tests
with one existing test ignored; they do not add a finished gear solid or export certificate.

Static spatial field composition and finite complete-member queries bring the suite to
1,036 passing core tests, with one existing test ignored. All 247 browser tests and CLI/ABI
checks pass; native and WASM artifacts were rebuilt. The independent complete-member audit
covers 18 blank/CSG point records and 648 attained indexed sweep witnesses. Global lower
bounds, source-error transfer and whole-space/export guarantees remain separate.

The schema-5 independent verifier now establishes all 18 selected complete-member point
classifications: 216 positive roll covers over 7,974 cells establish retained material, and
blank or attained cutting witnesses establish exterior. Missing cover cells and unsupported
positive claims are negative controls. The core suite passes 1,037 tests with one existing
test ignored. This establishes point signs for explicit binary64 geometry, not whole-space
coverage, source-error transfer, tight global-minimum accuracy or exported geometry.
