# Phase 3 plan: from source charts to an accepted swept-cylinder mesh

This breaks down Phase 3 of the [architecture plan](swept-boundary-architecture-plan.md).
The [implementation record](swept-boundary-phase-three.md) holds measurements and completed
work. Subphases 3a–3d are committed through `d367a96`; the 3d implementation is recorded
[here](swept-boundary-phase-three-d.md). The ordinary cylinder still has 6033 triangles,
13 open loops, 324 failed centroid obligations and 236 unresolved ones. Phase 2's accepted
mesh comes from a test-only explicit atlas, not the ordinary constructor.

There are two delivery points: **3k completes the production cylinder and checked mesh
export; 3m completes the broader sweep regression scope.** Hypoid cutter subtraction and
normal CLI gear export remain Phase 4. Completion of an earlier subphase means its stated
output works, not that the cylinder or arbitrary sweeps are accepted.

## Subphases and dependencies

| Subphase | Concrete output | Depends on | Status |
| --- | --- | --- | --- |
| 3a — Source connection | Evaluable source identities and native parameters through tracing | Phase 2 | Complete, `092f2cf` |
| 3b — Local trim operations | Transverse trim candidates, isolated edge roots, strict hiding witnesses | 3a | Complete, `9bf5cd9` |
| 3c — Coincident-carrier correspondence | A checked mapping between the two overlapping carried-rim charts | 3b | Complete; see 3c record |
| 3d — Union of carried-rim domains | One arranged rim region with all covering consumer maps | 3c | Complete; see 3d record |
| 3e — Complete source cover and chart transitions | Source inventory, regular pieces and supported handoffs; early local mesh/encoding experiment | 3b–3d; breakdown below | **Next: 3e.1 and 3e.2** |
| 3f — Trim network | Connected intersection curves and event vertices in every incident chart | 3d, 3e | Planned |
| 3g — Global visibility | Exposed/hidden/unresolved native regions with retained evidence | 3e, 3f | Planned |
| 3h — Oriented shared atlas | Visible regions decomposed into evaluable domains with common curve identities | 3d, 3f, 3g | Planned |
| 3i — Production construction | Ordinary sweep construction uses that atlas and the shared tessellator | 3h | Planned |
| 3j — Cylinder acceptance | Production cylinder passes the complete geometric acceptance gate | 3i | Planned |
| 3k — Encoded mesh acceptance | Actual float32 STL geometry is checked after encoding | 3j | Planned |
| 3l — Primitive tools and motions | Accepted curved, planar, translational and repeated-turn controls | 3k | Planned |
| 3m — Boolean tools and topology changes | Defined shell support and the remaining general-sweep regression matrix | 3l | Planned |

Start with 3e's source inventory and the small union-to-mesh experiment described below. The
experiment brings forward a bounded part of 3h/3k to expose evaluation and representability
barriers before constructing the complete cylinder arrangement; it does not complete either
later gate. The local rim experiments may use explicit source-parameter boxes; 3e removes that
manual setup from production. Broaden geometric support in response to the discovered inventory.

The 3e–3g stages exchange localized refinement obligations: 3e establishes source coverage
and chart transitions, 3f establishes the required trim network, and 3g resolves visibility.
Intersection or visibility events discovered later can require refinement of an earlier cover.
The table specifies completion dependencies, not one-pass execution or separate implementations
of the same mathematics. Final acceptance still requires every necessary obligation resolved.

## Findings from 3c–3d that change the remaining work

The [3d record](swept-boundary-phase-three-d.md) establishes the local domain union, but exposes
three contracts that must be settled before relying on it for a mesh:

- The union accepts regular pieces in one common projection chart with fixed source-coordinate
  signs. Automatic coverage must discover compatible pieces and supported transitions, including
  explicit singular limits, rather than assume every Phase 3c region can enter one union.
- Exact arrangement distinctions can be smaller than usable mesh-coordinate precision. Some
  are internal consumer-coverage boundaries; others constrain the geometric boundary. Establish
  which must become mesh edges and which can remain metadata. Proximity is not seam identity,
  and any geometric simplification needs an explicit approximation and topology contract.
- Three of forty alternate inverse-seed trials failed, and permitted point error can exceed
  the narrowest cell's width. Numerical residual agreement alone does not establish membership
  in that cell or precise shared-boundary correspondence. Retain established domain membership
  independently and strengthen evaluation using the known monotone branches and enclosures.

The 80-fractional-bit predicates resolved the local experiment, not every future precision
problem. Localize failures and extend precision or geometric support only for concrete cases
found by the source inventory or the early mesh experiment.

## 3a — Retain the source connection (complete)

**Output:** `solid::sweep_source` and tracing provenance preserve face, charted-edge and
Boolean-crease identities, native coordinates, local station branches and candidate events.
Validation controls are independent of construction tolerances.

**Established:** reevaluation survives tracing transformations and both sheet layouts;
source metadata does not change the frozen candidate geometry. **Not established:** globally
continuous station identities, exact event handoffs, native caps/grazing domains or visibility.
See the implementation record for the tests and residual measurements. Do not reopen this
milestone as a general tracer rewrite unless a later experiment identifies a specific defect.

## 3b — Supply local trim operations (complete)

**Output:** `arrangement::Pair` solves and samples transverse source intersections;
`arrangement::edge::isolate` isolates an incident face's contact root within a supplied edge
box across a roll interval; `arrangement::hiding` retains strict whole-box interior witnesses.

**Established:** both intersection consumers retain their own parameters; failed spans remain
explicit; isolated edge roots retain interval evidence; no hiding witness is promoted to
exposure. Six focused tests include the coincident-rim refusal. **Not established:** automatic
pair selection, a complete event/root cover, continuous trim topology or an assembled atlas.
Numerical midpoint agreement is not an interval continuation or approximation certificate.

## 3c — Establish a correspondence between coincident carriers (complete)

**Implementation:** [Phase 3c record](swept-boundary-phase-three-c.md). Exact structural
circle/axis checks establish the carrier for the supported coordinate-axis case. Interval
monotonicity establishes regular native maps; numerical inverse solves retain both consumers.
General orientations and other source/motion families explicitly remain unsupported by this
relation reader. Singular limits and exhausted searches retain unresolved native regions.
The two original rim bands have no unresolved correspondence regions.

**Problem:** two carried-rim charts can cover the same two-dimensional surface. A rank-deficient
transverse solve cannot decide whether they coincide, merely touch, or fail for another reason.

**Output:** for the two rim bands in the current counterexample, an evaluable map from each
regular source-chart region into a common carrier chart, with its inverse branches and domain
limits. Retain both consumers even where their images coincide.

- Derive the relation from the source curve and motion. A nominal spherical relation is a
  useful hypothesis; recognition must establish the required relation for the solved snapshot,
  rather than compare sampled residuals with a welding tolerance.
- Split where a map folds, reaches a pole or crosses a periodic seam. A map may be invertible
  only on each resulting branch. Account for orientations and source/roll domains.
- If the analytic relation cannot be established, retain an unresolved correspondence. A small
  axis perturbation may turn coincidence into a transverse intersection; it must not inherit
  the original carrier identity merely because the surfaces remain close.

**Exit:** the two-band fixture has checked correspondence domains and reevaluable consumer
maps. Source positions agree over the declared correspondence through an analytic relation or
retained interval correspondence evidence; finite samples alone do not establish that relation.
Consumer reversal and changed solve seeds preserve the same relation; a nearby
noncoincident example refuses the shared-carrier classification. No mesh or union is required
here. Failure to establish correspondence is the reason to revise this approach before 3d.

## 3d — Arrange the union of carried-rim domains (complete)

**Implementation:** [Phase 3d record](swept-boundary-phase-three-d.md). The original two bands
produce seven oriented arrangement cells and one boundary loop, retaining both consumers in
the overlap. Analytic line incidence and bounded interval ordering determine the union;
near-periodic endpoints are not welded. Disjoint, nested, repeated and adjacent coverage have
independent checks. Unsupported chart combinations, singular junctions, unresolved input boxes
and uncertain/budget-limited predicates refuse. Native inverse failures remain numerical
failures, never holes in the domain union. Production integration and visibility remain open.

**Problem:** common-carrier identity does not say which part of that carrier either band covers,
or what boundary their union has.

**Output:** the union of the two mapped rim domains, represented by oriented regions and
boundary curves, plus every region's covering consumers and maps back to their native charts.
This is the first new geometric construction after the local trim utilities.

- Intersect and split domain boundaries in the common chart, including nested or disjoint
  coverage, repeated boundary arcs and periodic pieces. Preserve multiplicity inside overlap.
- Cancel a boundary internal to the union only from established domain incidence and side
  information. Assign shared identities to actual common arcs and junctions.
- Keep singular limits and unresolved correspondence regions explicit. Membership in this
  carrier-domain union does not yet mean exposure on the boundary of the swept solid.

**Exit:** the two-band regression produces an inspectable union boundary and complete consumer
coverage. Independent analytic rim checks detect both omitted and duplicate regions; changing
input order or sampling seeds preserves the geometric union. Include disjoint, contained and
unresolved-domain controls. These are domain tests, so an open diagnostic rim piece need not
satisfy the closed-solid mesher's topology contract.

## 3e — Discover the cylinder's complete source cover and chart transitions

**Carry forward from 3d:** the local union requires regular pieces in one common projection
chart, with fixed source-coordinate signs. Discover chart transitions and unresolved limits
explicitly; do not turn numerical inverse failures or near-periodic endpoint proximity into
coverage decisions. Broaden this contract only for concrete configurations found by the cover.

**Problem:** today's charts and root isolator require caller-selected branches, boxes and seeds.
Production cannot infer completeness from whichever strands happen to be sampled.

**Output:** a source-domain inventory, regular partition and graph of source-chart transitions
for the cylinder, created from `SweepContacts` and the solved motion. Every relevant source/roll
region is represented, excluded with evidence, or retained as unresolved. Pairwise trim
completeness belongs to 3f and global visibility to 3g; neither is asserted by this cover.

Implement 3e in these bounded steps:

| Step | Concrete output | Dependency and completion criterion |
| --- | --- | --- |
| 3e.1 — Source inventory | All relevant source/roll domains, their identities, current capabilities and localized missing obligations | Start next. Account for the entire requested source domain without fixture branch lists; unsupported and unresolved parts remain inspectable. |
| 3e.2 — Local evaluation, mesh and encoding experiment | The existing two-band union exercised through a small shared-tessellator adapter and inspection of encoded geometry | Start alongside 3e.1 from 3d's output. Demonstrate the path on a well-resolved control and record the original thin-cell outcome, required contracts and any reproducible blocker. |
| 3e.3 — Regular-chart partition | Compatible regular pieces with bounds and explicit fold, pole and seam limits | Uses 3e.1 and the evaluation requirements exposed by 3e.2. Preserve a complete partition; identify concrete configurations needing an extension to 3c/3d. |
| 3e.4 — Supported transitions | Isolated source events and checked continuation/edge-face/chart handoffs | Uses 3e.3. Resolve the nominal cylinder's necessary source-chart transitions; lower budgets retain their exact outstanding domains and reasons. |

### 3e.2 experiment and evaluation contract

Use the actual union's cells, curve identities and consumer maps as the adapter input. Do not
substitute Phase 2's hand-authored cylinder atlas. This is an open carrier-domain diagnostic;
it does not select globally visible material or require a closed-solid certificate.

- Exercise interior, boundary and junction evaluation, using the retained monotone branches
  for bracketed inverses or enclosure-backed evaluation. Carry branch identity, parameter-domain
  evidence and point-error bounds separately. A residual smaller than a world-space tolerance
  cannot establish membership in a much thinner native cell. Localize precision exhaustion
  separately from failed convergence or unknown domain membership.
- Produce a local mesh through the existing shared tessellator, or a reproducible localized
  refusal identifying the failed adapter/evaluation obligation. Run a well-resolved regular
  control through the complete local path so the experiment is more than isolated helpers.
- Inspect binary64 geometry and geometry decoded from the actual float32 output. Check
  nondegeneracy, orientation, shared-edge incidence, expected open boundaries and collapsed
  cells or arcs. Report approximation evidence and its limits without claiming solid acceptance.
- Distinguish exact arrangement identities, required mesh topology and internal coverage
  metadata. Preserve all consumers even if an internal coverage boundary need not become a
  mesh edge. Do not merge distinct boundaries by tolerance or silently discard thin cells.
  If simplification is necessary, specify its error budget, topology conditions and evidence
  before using it; a declared periodic alias also needs identity evidence.

**Experiment exit:** retain inspectable input/output, the well-resolved control, the original
near-periodic case and a rounding-collapse negative control. Record which evaluation and
representation contracts work and which fail. A localized failure completes the investigation
only, not mesh support: assign each remaining blocker to the relevant 3e/3h/3k gate and resolve
it before that gate claims success. Inventory work can continue while those blockers are open.

### Source-cover requirements and completion

- Include endpoint face domains with their real trims, regular contact sheets, eligible carried
  edges, stationary/grazing regions, folds, poles and periodic boundaries. Regenerate endpoint
  and grazing geometry from sources; old faceted caps provide no native-domain evidence.
- Supply spatial and derivative bounds needed to exclude cells or isolate roots. Refine the
  original normal-velocity equation, not the tracer's tolerance-band endpoints.
- Establish continuation and edge/face handoffs using the full root enclosure, destination
  chart coverage and branch/orientation evidence. A local station-root ordinal or spatially
  nearby endpoint does not license a transition.
- Partition the search domain when budgets stop. A failed bracket search is not proof that
  an event is absent. Distinguish isolated-but-wide roots from unresolved root discovery.
- Report failed comparisons with their source regions, candidate event or boundary pair,
  retained enclosures, active precision and remaining work budget. Refine the responsible
  obligation; do not restart an entire arrangement or widen a welding tolerance. A general
  arbitrary-precision rewrite is not a prerequisite without a concrete failing case.

**Exit:** the nominal cylinder's necessary charts and handoffs are found without hand-entered
per-fixture root brackets or branch lists. The retained cover has no missing source intervals;
its necessary source-chart transitions and singular-limit handling are resolved. The early
experiment has recorded the evaluation/representation contract and assigned downstream blockers.
Lower budgets yield an inspectable partial cover. Native seam choices and tracing-seed changes
do not delete or duplicate a branch. Pairwise intersection and visibility obligations are
explicit inputs to 3f/3g, not prerequisites that 3e claims to have already discharged.

## 3f — Build the connected trim network

**Problem:** successful local intersection samples do not identify all trim curves, their ends,
shared junctions or continuation through a change of chart.

**Output:** a trim graph with event vertices, evaluable curve segments and parameter maps on
every incident source chart. Include endpoint/end-point, endpoint/contact, contact/contact
and carried-edge boundaries as required by the cylinder's source inventory.

- Use source bounds to exclude disjoint chart regions and generate candidate intersection
  searches. Retain unresolved pair regions rather than claim no intersection from failed seeds.
- Continue the transverse curves with 3b's solver, changing the continuation coordinate where
  needed. Account for tangencies and folds as events; send established coincident regions to
  3c–3d. Each completed segment needs continuation evidence beyond midpoint chord tests.
- Join curve ends through supported events and native identities. Carry trim/trim intersections,
  triple junctions and periodic aliases into every consumer's boundary description.
- Use the branch/domain and precision contract established in 3e.2 for shared-curve evaluation.
  Return missing chart transitions or bounds as localized obligations to 3e; retain unresolved
  pair regions until refinement establishes the required intersection or exclusion.

**Exit:** the cylinder's complete required trim network is connected and reevaluable on both
sides of every shared curve. Event incidence and branch continuity are supported; no failed
search is bridged. The independent endpoint-wall section and rim-union checks agree with the
network. Deliberately missing an event or truncating a search produces a localized refusal.
Reestablish this exit for any affected region when 3g introduces a new visibility event; an
earlier completed trim graph is not evidence for a subsequently refined arrangement.

## 3g — Classify global visibility on native regions

**Problem:** local contact and a carrier union are insufficient to determine which regions
bound the complete swept material. An intermediate pose can hide an endpoint contact.

**Output:** native regions classified as hidden, exposed with material-side information, or
unresolved, with the parameter domains and evidence used for each decision.

- Enclose each region's source-chart image before applying a whole-box hiding witness. A
  witness for a point or arbitrary box must not discard a larger parameter region.
- Establish exposure against the whole roll domain. Separate generating/contact neighborhoods
  from other potential covering times; exploit contact and side structure where the field is
  exactly zero. Repeated subdivision of boxes straddling that zero set is not an exposure proof.
- Split mixed regions at supported visibility transitions and feed newly found events/curves
  back into 3e–3f. This is a bounded refinement loop, not a one-pass triangle-centroid filter.
- Associate each refinement request with its source/pair domain and retained evidence. Preserve
  unaffected results and shared total budgets; changed geometry invalidates dependent evidence.
- Handle repeated boundary coverage through 3d's multiplicities. Choose an emitted consumer
  only after material-side and domain evidence justify the choice.

**Exit:** every region needed for the nominal cylinder is selected or excluded without an
unresolved visibility decision. Hidden endpoint-contact and repeated-rim controls exercise
both directions; insufficient budgets preserve uncertainty. Keep the existing field audit as
an independent judge. Do not require a replacement validator or claim exposure from the
absence of a hiding witness.

## 3h — Assemble an oriented atlas with shared boundaries

**Problem:** a set of visible trimmed regions is not yet the input expected by `shared::tessellate`.
Its current input is an evaluable four-sided domain with explicit corner and curve identities.

**Output:** the complete visible cylinder atlas, with material orientation, common curve
parameter sequences and maps from each domain back to its original source chart.

- Decompose trimmed regions into supported four-sided charts, including documented collapsed
  sides at singular limits. If a region needs a small tessellator extension, name and test it
  here; do not silently pass an arbitrary trimmed loop to a rectangular-grid API.
- Share each decomposition edge and geometric trim consistently, retaining the distinction
  between curve identity and endpoint identity. Every consumer evaluates the same geometric
  samples using its own coordinates; no seam vertex is invented by averaging.
- Derive directional refinement requests from chart behavior, particularly compressed rim
  regions. Keep approximation evidence and resource limits separate from topological identity.
- Make domain evaluation failures explicit when adapting fallible source/trim evaluators to
  the current callback API. Failed evaluations must not become invented coordinates.
- Resolve the evaluation and binary64 representation blockers identified by 3e.2. Keep internal
  consumer-coverage metadata distinct from mandatory mesh edges, with supported maps for every
  emitted domain. Any justified simplification must preserve the declared topology and fit the
  same approximation budget; retaining exact arrangement cells alone does not prove meshability.

**Exit:** an atlas incidence check finds opposite uses of every ordinary shared boundary,
consistent corners and supported singular limits. Reevaluation agrees across consumers and
orientation follows the selected material side. The atlas is generated from the source/event
arrangement; Phase 2's 36 domains and 70 curves are comparison data, not mandatory output counts
or a table to copy into production. Broken incidence and the compressed-chart control refuse.
The early experiment's thin-cell and boundary-evaluation controls also pass the declared atlas
contract or produce explicit refusal; nominal-cylinder blockers cannot remain open at this exit.

## 3i — Integrate the atlas into ordinary sweep construction

**Output:** ordinary `swept_boundary::candidate` / `construct` can build the cylinder from the
new arrangement and shared tessellator, retaining source maps and arrangement diagnostics.

- Select support from geometric capabilities and resolved obligations, not the fixture's name
  or recognition of its dimensions. The test-only `cylinder_domains` adapter stays outside the
  production dependency path.
- Keep accepted output behind fresh validation of the actual emitted mesh. Propagate stage
  and budget failures without disguising an incomplete arrangement as a successful fallback.
- Replace the legacy cap/overlap/weld/zip responsibilities for migrated regions. Any remaining
  legacy path must report its use and retain its old acceptance contract; remove obsolete
  repair code only after its callers have migrated.

**Exit:** an ordinary Solvent cylinder request generates a complete candidate through the
new path, with no hand-authored atlas input or repair-band closure. Export an inspectable mesh,
atlas and unresolved-obligation report. Prove this is the path exercised by the integration
test. Preserve baseline parity for unaffected cases. Acceptance is the separate 3j gate.

## 3j — Pass the production cylinder's geometric acceptance gate

**Output:** an ordinary accepted-cylinder regression and a reproducible evidence bundle.

- Require closed oriented topology with valid vertex links, zero failed and zero unresolved
  surface obligations, and interval spatial coverage in both distance directions at tolerance
  0.04. Use the existing calibrated field-value width 0.000005 and minimum probe 0.00002;
  retain the default-setting refusal control. Do not enlarge tolerances to hide a defect.
- Check geometric embedding separately: the current accepted-boundary type does not prove
  absence of self-intersections. State the predicate strength and ambiguity handling; a
  numeric test with a fixed epsilon must not be reported as an exact embedding certificate.
  Resolve ambiguous triangle pairs or refuse acceptance; make this an explicit geometric gate.
- Verify independent sections and volume with explicit numerical error. Keep construction
  formulas out of the oracle. Preserve the alias/zip and compressed-chart negative controls.
- Exercise nearby half-rolls and actual tracing seeds/chart order, not just Phase 2 interior
  parameter warps. Record triangle counts, construction/refinement work and field-query budgets;
  refine only the obligations that fail and revalidate after any geometry change.

**Exit:** the production cylinder and the selected nearby-roll/seed cases pass every declared
geometric gate. Promote the ordinary cylinder success test; retain expensive replay audits as
explicit reproducible gates if needed. An accepted nominal mesh alone does not establish
arbitrary motions, source-solve error bounds, isotopy or encoded export validity.

## 3k — Validate the actual encoded cylinder mesh

**Problem:** rounding to float32 can collapse features, reverse triangles or alter incidences
even when the binary64 construction was accepted.

**Output:** a checked cylinder STL and a report for the geometry decoded from its actual bytes.

- Encode, read back and check finiteness, nondegeneracy, exact encoded edge/vertex incidence,
  orientation and self-intersections. Do not tolerance-weld the decoded STL to make it pass.
- Account for encoding displacement within the same spatial error budget, using a justified
  transfer bound or fresh spatial validation of the decoded geometry. Include independent
  section/volume comparisons at the reported precision.
- Exercise a rounding-collapse negative control and preserve an existing output on refusal.
  Keep this export check reusable by Phase 4; wiring the hypoid CLI is not needed here.
- Reuse the 3e.2 encoding controls and resolve every recorded nominal-cylinder encoding blocker.
  Repeat the checks on the complete production mesh: a successful open local experiment does
  not discharge this gate, and exact arrangement distinctions need not all be STL mesh edges.

**Exit:** the ordinary cylinder's encoded output meets the declared topology, geometry and
spatial contract. **This completes the current swept-cylinder mesh goal.**

## 3l — Restore primitive-tool and motion coverage

**Output:** accepted production cases for simple curved and planar sources, followed by the
turned prism, turned box and whole-turn box, through the same arrangement mechanism.

- Restore stationary/translated sphere and simple prismatic controls first, then the turned
  cases. Exercise grazing faces, endpoint contact and repeated-turn coverage explicitly.
- Vary cylinder angle, dimensions, motion axis, scale, motion direction and requested tolerance.
  Exact coincidence may disappear under perturbation; the algorithm must change arrangement
  accordingly rather than preserve a fixture-specific topology.
- Each restored success must satisfy the applicable 3j–3k gates. Preserve explicit unsupported
  outcomes for cases not yet migrated, and keep the recovery ledger separate from calibration.

**Exit:** the named primitive cases and perturbation matrix pass with no fixture-specific fill
rules. Simple positive controls may be restored earlier as their mechanisms become available;
placing their completion gate here prioritizes the user's cylinder goal over a general-sweep
rewrite before cylinder delivery. Measure and improve performance after correctness, reusing
bounds and indexes before adding more global refinement.

## 3m — Support Boolean tools and changes in boundary topology

**Output:** the remaining lens/dumbbell and concave-feature regression cases, plus a declared
policy and representation for multiple components, cavities and singular boundary contacts.

- Extend event and trim coverage to Boolean creases and trimmed source faces using their
  retained identities and original evaluators. Do not turn a failed crease solve into a bridge.
- Generalize the accepted result beyond a single shell where disconnected material or cavity
  shells require it. Preserve orientation and nesting; components cannot be dropped to satisfy
  a one-shell assumption. Define explicit unsupported outcomes for singular contacts outside
  the representable boundary class.
- Exercise repeated coverage, topology changes, near-tangencies, holes and disconnected pieces.
  Distinguish resource-limited unresolved cases from unsupported geometry. An explicitly
  unsupported arbitrary input is honest behavior, but does not count as restoring a named
  lens or dumbbell positive control.

**Exit:** the declared general-sweep matrix, including the named Boolean-tool cases, passes
its applicable acceptance and encoding gates. Singular-boundary exclusions and supported shell
classes are documented. This completes Phase 3's stated regression scope, not a theorem that
all solids and motions admit a finite manifold mesh. Phase 4 can then consume accepted cutter
sweeps for Boolean subtraction and hypoid exports.

## Delivery discipline

Each subphase should end with its geometric/data output, focused regression and refusal
control, exact validation commands, unresolved obligations and a commit-sized record. Prefer
an inspectable partial cover or region arrangement to adding another disconnected helper API.
A green helper test is completion only of the corresponding bounded subphase.

Keep the source snapshot, options and evidence associated. Preserve unresolved parameter
regions and total work budgets across refinement; changing geometry invalidates its evidence.
The independent oracle and frozen Phase 1/1a/2 bundles remain unchanged.

For library changes run the focused tests and full core suite via
`cargo test --manifest-path rust/Cargo.toml -p gcs-core --test core` (never `--release`), plus
relevant CAD-enabled checks and export parity or the explicitly expected geometric changes.
Do not rerun geometry suites for documentation-only edits. Update this table and the
implementation record when a subphase's own exit has actually been demonstrated.
