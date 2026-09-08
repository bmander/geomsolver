# Continuous generating volumes: first experiment

This work implements the first investigation in the [gear roadmap](spiral-bevel-roadmap.md).
The solid remains defined by a continuous union of generator positions, followed by
subtraction from the blank. No new public gear solid or boundary extractor is implemented
by this experiment.

## Complete parameter coverage

`interval::minimum::enclose` takes a finite parameter interval and a function that encloses
the field on every requested subinterval. It returns a global minimum enclosure, a sampled
parameter witness, query count and termination status. The oracle contract matters: passing
an ordinary floating-point sample as a zero-width interval does not establish a bound.

The active cells initially cover the whole domain. Subdivision replaces one interval with
two closed intervals whose union is that interval. For each cell its field enclosure gives
a lower bound on the minimum. Point enclosures give upper bounds on attainable values.
A cell can be discarded only if its lower bound is at least the best attained upper bound.
The smallest remaining lower bound and the best upper bound therefore enclose the global
minimum. Parent and child enclosures are intersected; disjoint enclosures produce an error.
This detects some inconsistent oracles but cannot establish that an oracle is correct.

The smallest lower bound selects the next cell to split. Numerical stopping does not
assert success: `BudgetExhausted` and `ResolutionLimit` retain their remaining value
enclosure. `Converged` means only that this enclosure meets the requested field-value width.
It does not establish a unique minimizing parameter or a spatial surface-deviation bound.
The routine refines intervals; it does not replace or duplicate the local DogLeg solver.

Tests use interval polynomial fields for a translated sphere, including endpoint minima.
A second test has a narrow negative well between every point of a 101-position sampling:
the coarse samples all say outside while interval refinement establishes an interior cut.
Other tests cover budget limits, irreducible oracle uncertainty, floating-point subdivision
limits and invalid or detectably inconsistent inputs.

## Analytic functional material

`solid::PlanarField` defines an explicit planar material field from half-planes, disks,
unions, intersections and differences. `solid::RevolvedField` revolves that field using
an interval-normalized axis. All geometric coefficients are binary64 input data; field
normalization, arithmetic and square roots use outward intervals. Invalid coefficients,
overflow and expression depth over 64 are refused.

For point p, the primitives are `dot(p-through,normal)/|normal|` and `|p-center|-radius`.
Union uses min, intersection uses max, and difference uses `max(a,-b)`. Each primitive
is one-Lipschitz; these scalar operations preserve that property. The map from world
coordinates to (nonnegative radius, axial height) is nonexpansive, so revolution preserves
it too. Consequently the motion-speed enclosure can bound field variation without assuming
that Boolean field values are exact distances.

The regularized material is `closure({x | f(x)<0})`. A zero value does not by itself prove
a boundary: `max(d,-d)` is zero on a circle while its negative set is empty. Boundary
extraction must establish material and exterior, not simply wrap every zero. The tests
also verify that a sphere's meridian spine does not become an artificial wall on its axis.

Each generator section is constructed explicitly from its solved section's four supporting
half-planes and two rounded corners. For a minor counterclockwise corner, let a and b be
vectors from the arc center to its start and end. Define sector normals `(-a.y,a.x)` and
`(b.y,-b.x)`. The corner field is the minimum of the disk field and the two half-plane
fields through that center. The disk therefore constrains only the corner's angular
sector; the rest of the profile remains governed by its straight supports. Intersecting
these six conditions defines a closed rounded generating volume. The pinion uses one
section. The gear uses the union of `gear_outer` and `gear_inner`, revolved about their
shared back-crown axis. Union commutes with revolution and with the continuous union of
motion positions, so combining the sections before either operation preserves both volumes.

This is an explicit functional construction, not a silent reinterpretation of every
existing Solvent face. The old analytic profile evaluator remains independent. Agreement
with it is checked at source/material witnesses; a bound transferring the solved numeric
coefficients to the exact intended constraints remains unfinished. The new interval field
itself needs no assumed floating-point evaluation-error allowance.

## Motion speed bound

`motion::Family::inverse_point_speed_bound(x)` bounds the speed of `M(t)^-1 x` for every
roll value. It uses outward interval arithmetic on the solved motion graph. For a rotation
about origin o with angular rate w, let r bound the input point's distance from world zero
and s its speed. The transformed point satisfies

```
radius <= r + 2 |o|
speed  <= s + |w| r + |w| |o|
```

Represent a transform's bounds by `(a,b,c)`, meaning radius at most `r+a` and speed at most
`s+b*r+c`. Applying transform X and then Y gives

```
a = ax + ay
b = bx + by
c = cx + cy + by*ax
```

Store forward and inverse bounds for every graph node. For a relative motion
`observer^-1 * source`, compose source then observer inverse; its inverse composes observer
then source inverse. This avoids expanding shared graph dependencies recursively. The
bound covers the mathematical rigid transformations represented by the solved axes. It
does not include roundoff in pose evaluation or the source's nonlinear solve error.

For a true signed-distance field d, its one-Lipschitz property then implies, on an interval
centered at t0 with radius h,

```
d(M(t)^-1 x) in d(M(t0)^-1 x) + [-L*h, L*h]
```

where L is the speed bound. A proved point-evaluation enclosure can replace the center
value. A Boolean membership field is not automatically a signed-distance field; this bound
must not be transferred to arbitrary fields without a justified Lipschitz constant.

## Interval pose evaluation

`motion::Family::bounds` now encloses the actual poses over a roll interval.
`MotionBounds::point` and `inverse_point` accept coordinate boxes and return boxes covering
every input point under every represented pose. Coefficients are private and arise only
from a solved motion family. For a rotation about origin o and normalized axis a,

```
R = cos(theta) I + (1-cos(theta)) a a^T + sin(theta) [a]_cross
p = o - R o
```

Axis normalization, `theta = phase + ratio*t`, and all matrix/vector arithmetic use outward
interval operations. Sine and cosine use the existing polynomial/remainder enclosures,
not sampled libm values. For a relative motion the matrix is `R_observer^T R_source` and
the translation is `R_observer^T (p_source-p_observer)`. The inverse point map uses
`R^T (x-p)`: each underlying rotation is orthogonal even though an interval matrix is
not itself an orthogonal matrix. Input angles outside the supported trigonometric domain
are refused, as are overflow and unresolved axis normalization.

These bounds describe the mathematical motion defined by the stored solved axes and
scalar values. World-coordinate construction and nonlinear solving have already rounded
those values; their deviation from intended input constraints is a separate source-error
question. The bounds also need not contain every rounded result of the older floating-point
pose evaluator; they enclose its intended mathematical transformation.

The pair experiment bounds each center-time inverse pose and evaluates the functional
material field over the entire resulting coordinate box. The speed bound covers movement
away from center time. Cached interval poses belong to one fixed generating family and are
shared across that family's spatial queries.

## Pair evidence and its exact limits

The experiment reads the default 24:48, mean-module-2 source and constructs the explicit
functional generators above. For each member it checks 36 flank and fillet witnesses at
toe, mean and heel sections, plus points displaced 0.01 mm to each side. Those 216 queries
each cover the full declared roll interval [-35°,35°]. The gear queries use both generating
sections together, including the inactive faces and opposite crown semicircle. A separate
grid of 11,163 points per section (33,489 total) checks agreement with the old source solids'
material classification away from a 2e-8 mm comparison band.

An offset check caught an early setup error: the inner flank's parameter runs toward the
base while the outer flank runs toward the fillet. The test now reads the join coordinate
from the declared seam, covering the actual tip-to-fillet extent on both sides.

Earlier experiments assumed a 2e-9 mm uniform error around floating-point material queries.
That assumption has been removed from the current experiment. Its field and motion queries
now use interval arithmetic throughout. The exact functional shape is specified by the
recorded coefficients; correctness of their extraction from intended source constraints is
still a separate source-error question.

The recorded pair run passed 108 whole-roll checks per member in 6.55 seconds for the
pinion and 9.33 seconds for the gear. The pinion needed at most 28,280 oracle evaluations
per point and 193,963 cached interval poses; the gear needed at most 36,012 evaluations
and 197,983 cached poses. The requested minimum-field-value width is 0.0002 mm. It is not
a surface-deviation or export tolerance: this Boolean field is one-Lipschitz but is not
necessarily a signed-distance field. These are experimental timings, not a meshing-performance
guarantee.

To reproduce the complete numeric field/motion definitions and per-query evidence:

```sh
SOLVENT_SWEEP_PROBE_OUTPUT=/private/tmp/solvent-functional-pair.json cargo test \
  --manifest-path rust/Cargo.toml -p gcs-core --test core \
  functional_pair_generators_have_whole_roll -- --nocapture
python3 rust/gcs-core/tests/verification/functional_sweep.py \
  /private/tmp/solvent-functional-pair.json
```

The separate Python checker uses exact rational interval arithmetic, dyadic outward
rounding, rational square-root bounds and independent Taylor trigonometry. It reconstructs
the generator and inverse relative motion from the exported coefficients and verifies each
reported upper-bound witness, its parameter-domain membership and declared enclosure width.
The schema-2 export includes both member cases and each generator's explicit profile list;
the checker also accepts the earlier schema-1 pinion artifact. All 216 pair witnesses passed.
False upper bounds and out-of-domain witnesses are negative controls for both members.
Omitting either gear section is also rejected by independently evaluating the affected
flank witness, rather than by checking the number of profiles. **This checker does not
audit the global lower bounds or source-solve error.**

## Indexed closure counterexample

The selected unindexed witnesses are not sufficient to accept the full closed construction
solids as generating volumes. A further test inverse-indexes one material-side point beside
each working flank through all 24 or 48 tooth positions (144 whole-roll queries). The points
are at the mean face station, halfway from tooth tip to fillet join, displaced 0.01 mm away
from the generator and into intended tooth material. Each transformed binary64 point is the
exact input to its interval query; error transfer from ideal indexing remains separate.

The full-crown candidate fails on the gear's `gear_inner_outer` flank at index 47. The query
point is `[53.46631080593146, -4.679216589100084, -0.1754660263476211]` mm. Its sweep minimum
is enclosed by `[-0.019240778833223986, -0.019040811248087756]` mm. At roll
`0.08435795779421369` radians, the independent rational evaluator gives approximately
`-0.019040811248437664` mm. The `gear_outer` section covers it; `gear_inner` is exterior
with field value approximately `4.817177911612632` mm. The active field constraint at this
witness is `gear_outer`'s **inactive outer supporting wall**.

This is a rejection of the proposed full closed generator, not of the interval method.
A single rigorously negative attained value already establishes membership in the swept
volume: no independently certified global lower bound is needed to exhibit this overcut.
The reference sections were explicitly construction solids, and their closure cannot be
silently treated as part of the intended tooth-space definition. The regression retains
this rejected candidate so that local-envelope success cannot be mistaken for a valid
indexed material construction. It does not assert that the finished gear should overcut.

Reproduce the indexed evidence and audit its attained witnesses with:

```sh
SOLVENT_INDEXED_SWEEP_OUTPUT=/private/tmp/solvent-indexed-full-crown.json cargo test \
  --manifest-path rust/Cargo.toml -p gcs-core --test core \
  indexed_full_crown_union_exposes_neighboring_gear_overcut -- --nocapture
python3 rust/gcs-core/tests/verification/functional_sweep.py \
  /private/tmp/solvent-indexed-full-crown.json
```

## Closing a space with its neighboring active side

The next candidate replaces the inactive walls with the two active boundaries that actually
bound one gear space. Let `Ho` keep the **inner** wall, its root corner, and the base/tip
height conditions of `gear_outer`. Let `Hi` keep the **outer** wall, its root corner, and
the same height conditions of `gear_inner`. Their opposite walls and corners are omitted.
These one-sided fields are intermediate material conditions, not independent finished tools.

Let `Nc = hypot(zp,zg)`, `delta = -2*pi/Nc`, and let `Rc(delta)` rotate about the crown axis
through the pitch apex. Define one gear-space generator by

```
Cgap = Ho intersection Rc(delta) Hi
cgap(x) = max(ho(x), hi(inverse(Rc(delta)) x))
```

The intersection is evaluated at each generating pose, before minimizing over roll.
Sweeping an intersection is generally not the same as intersecting separately swept
volumes: the latter allows the two sides to be attained at different roll values.

The shift follows the generating kinematics, rather than an adjusted closure dimension.
For nominal gear angular rate `w = -Nc/zg`, write `M(t) = inverse(B(t)) C(t)`, where B is
body rotation by `w*t` and C is crown rotation by t. With `theta = 2*pi/zg = w*delta`,

```
Rbody(theta) M(t+delta) = M(t) Rc(delta)
```

Thus the shifted inner section supplies the active boundary of the adjacent material tooth.
The two active surfaces close the gap against each other. At a fixed crown height this is
the interior of one disk, with an offset disk excluded; the retained height caps bound it
axially. A new arbitrary radial wall or a manually stitched seam is unnecessary. Its field
is one-Lipschitz because rigid transforms and max preserve that property.

This identity is for the nominal generating system. The actual interval queries target the
exported binary64 coefficients, including the crown-pitch shift. Transfer from those solved
and rounded coefficients to the nominal axes, rates and indexing remains unproved.

`tests/envelope/paired/swept/closure.rs` implements this candidate using existing revolved
fields, an interval-bounded static rotation and field intersection. It does not change the
source reference solids or introduce language syntax. The expanded run passes 435 whole-roll
queries: 108 flank/fillet boundary and two-sided witnesses, all 48 indexed sweeps at six
middle-flank material points across toe/mean/heel (282 additional queries), and 45 root-floor
and gap-interior witnesses. The latter use the independently specified root cone, at three
angular stations across each of three spherical face sections, checking both sides of the
floor and two additional heights toward the addendum. This includes the previously overcut
working-flank point and detects a closure that would leave material in the tested gap.
These checks preserve selected geometry; they do not prove all-space coverage or eliminate
every possible ridge. A recorded serial run took 38.37 seconds with 744,841 cached interval
poses; this is not a meshing-performance guarantee. The rational checker independently
reproduced all 435 attained witnesses and rejected all four negative controls: false upper
bound, invalid roll, and omission of either active boundary. Global lower-bound auditing
and source-error transfer remain separate.

Schema 3 describes field intersections and static rotations, and permits an experiment for
one member. The independent checker reconstructs this expression tree directly; it continues
to accept the earlier schema-1 and schema-2 artifacts. To reproduce:

```sh
SOLVENT_CLOSED_SPACE_OUTPUT=/private/tmp/solvent-neighbor-closed-space.json cargo test \
  --manifest-path rust/Cargo.toml -p gcs-core --test core \
  neighbor_closed_gear_space -- --nocapture
python3 rust/gcs-core/tests/verification/functional_sweep.py \
  /private/tmp/solvent-neighbor-closed-space.json
```

The root and gap-interior samples give further evidence beyond the original overcut witness.
Whole-tooth spatial coverage, the finite blank, roll endpoint adequacy, source-error transfer
and boundary extraction remain open. No public completed gear solid or tolerance-controlled
export is delivered by this experiment.

Earlier validation after functional material integration: 1,030 core tests passed (one existing
test ignored), 247 browser tests passed, and CLI/ABI checks passed. Native and WebAssembly
libraries were rebuilt. The pair extension changes the experiment and independent checker;
its targeted source-material and whole-roll tests pass. The indexed regression records the
rejected full-crown closure described above.

Validation after the pair/indexing investigation: all 1,031 core tests passed (one existing
test ignored). The rational checker verified 216 unindexed pair witnesses, 144 indexed
witnesses and the 108-witness legacy artifact. Both pair artifacts rejected all six negative
controls. The indexed artifact independently confirms the one definite overcut above.
This change touches experiments, verification and documentation; it changes no core runtime,
ABI, browser implementation or exported STL geometry.

Validation after the neighboring-side closure experiment: all 1,032 core tests passed
(one existing test ignored). The new 435-query experiment and the rejected full-crown
regression both pass their distinct contracts. The changes remain in the workbench,
independent verifier and documentation; runtime solid/language integration is still pending.

## Finite complete-member material queries

`solid::SpatialField` now composes revolved fields in world coordinates through fixed rigid
transforms, union, intersection and difference. It owns immutable shared expression nodes;
cloning a generator for multiple placements does not copy its entire geometry. A query
memoizes by node identity **and input box**, so a shared Boolean expression does not expand
exponentially and differently transformed uses are not conflated. Spatial depth is limited
to 64, separately from the planar leaf's depth. Transforms accept one fixed parameter of a
solved motion family; an interval of poses is not silently treated as one rigid placement.

The neighboring-side gear-space experiment now uses this core composition instead of an
ad hoc pair of field queries. `tests/envelope/paired/swept/member.rs` combines the resulting
sweeps with each member's finite blank. The blank is a revolved planar intersection:

```
blank = (heel disk minus toe disk) intersection tip half-plane minus back half-plane
member(x) = max(blank(x), -min over index k and roll t of generator(inverse(M(t)) inverse(Rk) x))
```

The disks are centered at the pitch apex in meridian coordinates. Toe and heel radii are
read from the named spherical boundaries; the tip and back supports are read from their
named cone meridians. Their outward half-plane normals follow the member's positive axial
coordinate. This avoids treating the finite cone carriers' construction caps or spines as
part of the intended blank. The pinion uses its full rounded generating section; the gear
uses the neighboring-active-side space described above. Each uses its own named relative
motion, the declared [-35°,35°] roll interval, and all 24 or 48 body-axis indices.

Indexing is now evaluated with interval poses, including normalization and trigonometry;
the full input coordinate box passes through the inverse index and inverse generating pose.
The sweep's speed bound uses the maximum distance from world zero over that box. Each
indexed roll minimization retains its status and enclosure, and the final field combines
all of them with the blank. The refinement budget is **per indexed sweep**. An exhausted
budget does not discard that index or turn its sampled value into membership. The retained
field bounds may still establish a sign; otherwise material remains unresolved.

The workbench checks 294 blank points against independent sphere/cone formulas, and 18
complete-member point queries across both members: the two sides of each working flank,
retained rim material, both spherical ends, the back opening and the apex opening. Each
complete point includes every indexed sweep. Small coordinate boxes inside retained tooth
material are also bounded, and four-evaluation budgets are checked against the refined
point results. These are selected point/box checks, not whole-member spatial coverage.

Schema 4 exports both blanks as explicit planar expression trees, the generating fields,
relative motions, every fixed index, and each point's blank/sweep/material intervals. Its
independent verifier reconstructs the blank and all attained indexed witnesses with
rational intervals, then checks the interval arithmetic for the regularized difference.
The global sweep lower bounds still come from Rust; this does not independently certify
inside classification, source accuracy, or geometric/topological completeness.

To reproduce this evidence:

```sh
SOLVENT_MEMBER_MATERIAL_OUTPUT=/private/tmp/solvent-complete-member-material.json cargo test \
  --manifest-path rust/Cargo.toml -p gcs-core --test core \
  complete_members_bound_material -- --nocapture
python3 rust/gcs-core/tests/verification/member_material.py \
  /private/tmp/solvent-complete-member-material.json
```

The complete-member evaluator remains in the workbench. The core now owns static spatial
field composition; swept-solid language/IR integration and boundary extraction remain
unfinished. The encoded STL artifacts still come from the earlier analytic surface workbench.

The independent complete-member audit passes for all 18 point queries and their 648 indexed
attained witnesses. It rejects incorrect blank bounds, false sweep bounds, an altered index
pose, a missing index and an inverted cut sign. The migrated neighboring-side experiment
also passes its 435-witness rational audit. All 1,036 core tests pass (one existing test
ignored), as do 247 browser tests and the CLI/ABI suites. Native and WebAssembly libraries
were rebuilt after adding spatial field composition.

## Independent full-roll material-sign certificates

Schema 5 adds positive roll covers for retained-material points. During a sweep query, the
workbench records oracle intervals with strictly positive lower bounds. After refinement it
selects a closed, possibly overlapping cover of the **entire declared roll interval** whose
bounds exceed a positive target. The target is half the primary minimum's lower bound; it
is a proposed margin that the independent checker must establish, not a trusted result.
If the recorded intervals leave a gap, no certificate is emitted. The selection routine
and the independent coverage checker both reject missing endpoints and arbitrarily small
holes; the latter includes a one-ulp-gap negative control.

For a fixed inverse-indexed point p, write the inverse relative motion as source-inverse
after observer. Let the two rates be ws and wo, their origins os and oo, and let r bound
|p|. The rational verifier independently computes

```
L = |wo| (r + |oo|) + |ws| (r + 2 |oo| + |os|)
```

This follows from each rigid rotation's radius bound `r+2|o|` and speed bound
`s+|w|(r+|o|)`. The fixed index contributes coordinate uncertainty but no time derivative.
All supported generating fields are one-Lipschitz: normalized half-planes and disks,
nonexpansive revolution coordinates, fixed rigid poses, and min/max/negation preserve that
property. Thus on a cell [a,b] with center m and `h=max(m-a,b-m)`,

```
generator(inverse(M(t)) p) >= field_at_m.lower - L*h  for every t in [a,b]
```

The checker reconstructs `field_at_m` using rational intervals and independent Taylor
trigonometry. It recomputes L, h, every cell bound, and exact interval coverage. It does not
trust exported cell values, the primary refiner's heap/pruning history, or its reported
minimum lower bound. Overlap is allowed; uncovered parameter values are not.

For retained material, the independently evaluated blank has a negative upper bound beta,
and every indexed sweep has a certified positive lower bound lambda_k. Therefore

```
member_field(p) <= max(beta, -min_k lambda_k) < 0
```

For exterior points, either a positive blank value or one strictly negative attained
cutter value suffices. These arguments independently establish the requested **point
classifications for the explicit binary64 geometry**. They do not establish the primary
minimum's tighter 0.0002 mm enclosure, transfer source-solve error, cover the entire member
spatially, or certify an exported boundary.

The current certificate contains 216 full-roll covers over 7,974 cells for the six retained
points among the 18 member queries. The other 12 points use blank/existence witnesses.
The verifier remains compatible with schema 4, whose scope is only attained witnesses and
reported CSG arithmetic. Schema 5 uses the same reproduction commands above and additionally
checks missing-cell and false-positive-bound controls.

The completed rational audit classifies all 18 member points and checks all 648 attained
indexed witnesses. All 216 positive covers and 7,974 cells pass; the minimum certified
retained field margin is 0.004900000909484636 mm. All seven corrupted-evidence controls are
rejected (blank, sweep, index, missing index, cut sign, missing cover cell, false positive
bound). The independent cover self-check also rejects a one-ulp hole and verifies an
explicit offset-axis speed bound. All 1,037 core tests pass, with one existing test ignored.

## Core swept-field capability

`solid::SweptField` now owns an explicit `SpatialField`, a solved `motion::Family`, and a
finite closed roll interval. It defines `min_t source(inverse(motion(t)) x)` directly. The
source type's constructors establish its one-Lipschitz contract; callers no longer supply
an arbitrary field callback to the gear sweep implementation. A point interval is a valid
fixed-pose sweep. Domains and geometry snapshots are immutable.

`SweptField::evaluator(max_cached_poses)` creates separate numerical query state. Its cache
has an explicit capacity; zero disables it, and reaching the cap causes recomputation
without dropping intervals. `SweepEvaluator::bounds(box, options)` propagates the whole
point box through the interval pose and source field, then uses the existing global interval
refiner. It returns the field enclosure, roll witness, evaluation count and termination
status. Invalid options, unsupported trigonometric evaluations and arithmetic overflow
remain errors; budget exhaustion retains an enclosure and status.

`bounds_with_observer` additionally exposes each raw oracle interval and its enclosure for
certificate extraction. The observer supplies no geometric values or pruning decisions.
The gear workbench now uses this core evaluator for the unindexed probes, rejected closure,
neighboring-side space and complete-member material queries. The duplicated callback-based
workbench sweep implementation has been removed. The workbench explicitly allocates a
one-million-pose cache per evaluator; this is a test-performance control, not part of the
solid definition.

The new core tests compare a swept sphere to an independent torus formula and finite-arc
end-cap distances. They also exercise full point boxes, fixed-pose domains, cache caps,
cached/uncached result equality, observer counts, budget exhaustion and explicit failures.
The full-turn test uses a roll span wider than 2*pi, avoiding an assumption that a binary64
pi endpoint is an exact complete turn.

This is a reusable core numerical capability. Composition of completed swept operands into
the editable solid graph, public Solvent syntax, boundary extraction, and the remaining
whole-space/source-error verification are still required for the final gear deliverable.

The core-sweep migration passed 1,040 core tests (one existing test ignored), 247 browser
tests and the CLI/ABI suites; native and WebAssembly artifacts were rebuilt. The complete
schema-5 member artifact is identical to the former workbench evaluator's output, including
bounds, witnesses, evaluation counts and covers. Its independent rational audit again
passes all 18 point classifications, 648 attained witnesses, 216 covers and 7,974 cells;
the separate neighboring-side audit passes all 435 attained witnesses.

## Composition of completed swept operands

`solid::MaterialField` composes static `SpatialField` leaves, completed `SweptField` leaves,
fixed rigid poses, union, intersection and difference. The immutable graph keeps the
mathematical material definition separate from numerical query state. Each sweep still
requires a static spatial source; this API does not discretize an inner continuous sweep
to enable a nested sweep. Graph depth is capped at 64, separately from static leaf depth.

`MaterialEvaluator::bounds(box, options)` now owns the complete member expression, including
the blank and every indexed cut. Its result contains the final field interval and every
distinct swept-node/input-box query's domain, box, minimum, witness, evaluation count and
termination status. Options apply per such sweep query. Exhausted sweeps still contribute
their enclosures; all Boolean operands are evaluated and any error fails the query.
A strict negative or positive final interval establishes material or exterior respectively.
An interval containing zero remains unresolved, including degenerate Boolean zero sets.
Individual sweep convergence does not imply an export distance bound or whole-box uniformity.

Clones share immutable nodes. Repeated nodes at the same box are memoized for that query;
different index transforms retain their distinct boxes and evidence. A shared swept node
uses one capped pose cache across all its indexed copies. Only poses persist between queries,
so changing a box or refinement budget cannot reuse a previous minimum. The evidence observer
tags each raw roll enclosure with its entry in the returned sweep-query list. Cached visits
do not duplicate certificates; a failed query produces no successful material report.
The member certificate still lists every declared tooth index. Its exporter maps each
inverse-indexed box to the core query that evaluated it, repeating the corresponding evidence
when indices coincide at a box. In the current default fixture's apex query, 24 pinion indices
require only 12 distinct boxes and 48 gear indices require 21. This is reuse of identical
interval inputs, not omitted cuts.

The complete-member workbench now constructs this core graph and only supplies the explicit
generating geometry, blank, named motion and index poses. Its hand-written loop for combining
sweep minima and cut signs is removed. Positive-cover selection and independent verification
remain separate from the core evaluator. Editable Solvent solid declarations, boundary
extraction, source-error transfer and whole-member verification remain subsequent gates.

The composition tests compare union/intersection/difference against independent ball and
torus formulas, including whole boxes and a fixed transform of a partial circular sweep.
They check distinct transformed boxes, cached/uncached evidence equality, exhausted budgets,
retry with a larger budget, capped shared pose storage, a depth-64 shared expression,
Boolean cancellation and failing operands. All 1,043 core tests pass (one existing test
ignored); the member tests also pass after adding per-index certificate alias mapping.
All 247 browser tests and CLI/ABI suites pass. Native, WASM and CLI artifacts were rebuilt;
the released CLI solves the paired reference source with no free parameters.
The exported complete-member schema-5 evidence is exactly equal to the independently audited
pre-composition artifact, including all 648 indexed witnesses and positive roll covers.
Rerunning the independent rational checker on that export passes all 18 classifications,
648 attained witnesses, 216 full-roll covers and 7,974 cells, and rejects all seven corrupted
evidence controls. Its scope remains selected point signs for the explicit stored geometry.

## First spatial boundary extractor

The core now derives finite material supports and extracts a single closed mesh with a
two-sided spatial distance certificate. Complete interval-classified spatial cells prevent
corner sampling from silently omitting distant small components. Mesh-to-boundary evidence
uses strict material/exterior crossings; boundary-to-mesh evidence covers every unresolved
terminal cell. See [field boundary extraction](field-boundary-extraction.md) for the algorithm,
independent rational sphere audit, explicit refusals and complete-pinion experiment.
The spatial bound is separate from field-value tolerance, source accuracy, geometric
embedding, topology preservation and coordinate-quantized export checks.
The first coarse pinion candidates are refused as disconnected. A retained-material path
now proves that the isolated 20 mm grid sample connects continuously into the rim, with
at least 0.23112737000745628 mm of field margin along the entire segment. This identifies
a concrete sampling loss of connectivity without changing the generating geometry. The
extraction record includes the refused component's location and the remaining fine-mesh work.
