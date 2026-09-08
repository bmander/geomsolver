# Boundary extraction from material fields

The core now extracts a triangle boundary from `MaterialEvaluator` with an explicit spatial
distance bound. This is a first extraction backend for the continuous-volume gear route.
It does not make the current gear deliverable complete: source accuracy, geometric embedding,
topological equivalence to the intended member, mating verification and application integration
remain separate gates.

The [method comparison](implicit-meshing-methods.md) treats this implementation as a
baseline and selects the next investigation from implicit/F-rep meshing and swept-volume
research. Further uniform refinement alone is not the intended production strategy.

## Search bounds come from the construction

`support_bounds()` derives a finite box containing all of `closure({f<0})`, or returns `None`
when the construction does not provide one. `None` does not prove unboundedness. A disk's
planar support radius is bounded by the norm of its center plus its radius. Union requires
both supports; intersection can use either and tightens known boxes; difference retains the
left support. Revolution preserves the norm of the meridian coordinates. Fixed transforms
propagate boxes through interval poses, and a swept support uses the entire declared motion
interval. Shared spatial/material nodes are memoized during support derivation.

This matters for completeness: exterior signs on the faces of an arbitrary crop do not rule
out a separate component beyond that crop. Extraction supplies no arbitrary crop. Unknown
supports and unsupported interval evaluations fail explicitly. The derived support is padded
by the requested spatial tolerance to form the search domain.

## Complete spatial partition

`BoundaryOptions` separates spatial tolerance, octree depth/cell budgets and per-sweep
refinement controls. The chosen finest grid has a cell diagonal bounded nominally by one
quarter of the requested tolerance; final distances are checked using actual rounded cell
coordinates. All surface cells use that same finest grid. Uniform cells can stop earlier,
so coarse empty/interior regions do not generate triangles or hanging surface nodes.

For a cell Q, the evaluator bounds the field at its center c. Every available material field
is one-Lipschitz by construction. With an outward bound r on the farthest corner distance,

```
f(Q) is contained in [f(c).lower - r, f(c).upper + r].
```

Strictly negative/positive cells are uniform material/exterior. Every other cell is split,
or retained as a possible boundary cell at the finest level. The successful result stores
every terminal cell, its dyadic address, box, center enclosure and radius. Cells are never
discarded solely because their corners have the same sign.

Center queries may request a field width proportional to cell size. Corner queries start
with a similarly coarse width and refine toward the supplied per-sweep tolerance only when
their sign is unresolved. Every accepted corner still has a strict bounded sign. Refinement
budgets remain per sweep; their numerical termination does not substitute for a sign proof.

Sweep queries can also stop when their complete minimum enclosure lies strictly outside a
requested field-value band. Cell centers use `[-r,r]`; corners use `[0,0]`. The interval
refiner reports `Separated`, distinct from reaching its value-width tolerance. It retains
the complete-cover lower bound and attained upper-bound witness. Touching a band endpoint
does not qualify. Material differences reflect the right operand's band; query memoization
includes that band along with node identity and the entire input box. Every operand is still
evaluated, so errors remain explicit. The final Boolean interval determines classification.
`bounds_outside_with_observer` exposes the same raw oracle evidence for subsequent independent
audits; observation cannot affect the geometry or stopping decisions.

Four default-pinion box queries (two retained points, the apex and an exterior point) used
3,216 roll evaluations with band termination versus 519,516 with full-width refinement.
The faster intervals enclosed the fully refined intervals and established the same margins.
This is a local evaluation-count measurement, not an end-to-end export speedup or an
independent audit of all spatial classifications.

## Mesh and two distance directions

Each finest cube is split into six conforming tetrahedra along its 000--111 diagonal. Strict
corner signs select crossed edges. Their midpoints are shared by grid-edge identity, so no
proximity welding defines connectivity. Triangles are oriented using interval arithmetic;
uncertain orientation is refused. The resulting single shell must pass the existing closed
oriented manifold checker, including vertex links and connectivity.

The mesh's vertices are approximations derived from the field. Its distance certificate
uses spatial cells, not the magnitude of a field residual:

1. Each emitted triangle lies in a cell containing a proven negative point and a proven
   positive point. Continuity establishes an actual boundary of `closure({f<0})` in that
   cell. Every point of the triangle is therefore within the cell diagonal of the true
   boundary. The certificate retains the sign witnesses and triangle ranges.
2. Every possible true boundary point is in a terminal cell whose field interval contains
   zero. Each such cell must have an actual mesh vertex within the requested distance.
   The search starts in its neighboring finest cells and searches a spatial index of all
   cell representatives if that fails. Subtrees are pruned only by conservative lower
   distance bounds. An outward bound from the selected vertex to the farthest corner
   bounds the distance from every point of the cell to the mesh.

The maximum of these bounds is a two-sided boundary distance (Hausdorff) bound for the
explicit field snapshot. It must not exceed the requested spatial tolerance. A cell without
nearby mesh evidence is unresolved and the extraction returns no accepted mesh. In particular,
a small disconnected feature far from the visible component cannot disappear just because
all its corner samples are exterior. Features closer than the accepted distance can still
have the wrong topology; a distance bound alone does not prove feature preservation or isotopy.

`FieldBoundary` owns immutable vertices, triangles, checked shell and spatial certificate.
Depth/cell exhaustion, ambiguous corner signs, unknown support, unresolved cells and failed
topology are explicit errors. Exact or nearly exact grid/surface coincidences can currently
require another export resolution. The sphere regression retains a refused ambiguous grid
vertex; it does not perturb the geometry or assign an arbitrary sign.
Topology refusals retain component counts, bounds and approximate volumes for diagnosis;
they do not return an accepted mesh. `boundary_with_observer` reports completed cell counts
for partitioning, triangulation and distance certification without changing geometry or
acceptance. The workbench uses it to distinguish evaluation cost from witness-search cost.

The bound applies to the stored binary64 mesh. Encoding to STL adds coordinate error and
requires a fresh check of the actual encoded topology. The pinion export workbench checks
triangle-count preservation, float32 topology and a conservative per-vertex displacement
bound, which is added to the spatial bound. Neither mesh topology nor these distance bounds
prove geometric self-intersection freedom or source-solve accuracy.

## Independent certificate fixture

The sphere fixture exports its complete spatial partition, mesh and crossing witnesses:

```sh
SOLVENT_FIELD_BOUNDARY_OUTPUT=/private/tmp/solvent-field-boundary-sphere.json cargo test \
  --manifest-path rust/Cargo.toml -p gcs-core --test core \
  functional_solids::boundary::sphere_boundary -- --quiet
python3 rust/gcs-core/tests/verification/field_boundary.py \
  /private/tmp/solvent-field-boundary-sphere.json
```

The independent checker imports no core interval, meshing, topology or distance code. Exact
rational inequalities verify the sphere support, complete nonoverlapping dyadic partition,
shared coordinate planes, center field enclosures, Lipschitz inflation, both distance
directions, opposed edge uses, vertex links and connected sphere topology. It refuses a
missing cell, false radius/center bound, missing crossing, incorrect inside witness, moved
mesh vertex, false distance and cropped support.

The initial fixture has 3,974 vertices, 7,944 triangles, 3,928 partition cells and 1,472 possible
boundary cells. All are covered by mesh-distance evidence; 1,328 cells carry crossing
witnesses. The independently checked distance bound is 0.2448093965210489 in the sphere's
length units. This independent audit currently supports the explicit sphere fixture, not
arbitrary generating fields or a whole gear certificate. Separate core tests exercise a
continuously swept sphere's torus and its hole, unknown supports, expression sharing,
resource exhaustion and a missed-between-corners disconnected component.
A further regression unions the sphere with `A-A` nearby. The zero-only operand produces
uncertain cells requiring the wider witness search, but must not produce another shell.
The final indexed-search sphere artifact is identical to the independently audited fixture.

## Complete pinion experiment

The existing member workbench can send the same complete pinion `MaterialField` to this
extractor. Its tolerance is an export control and does not change the generating geometry:

```sh
SOLVENT_MEMBER_BOUNDARY_OUTPUT=/private/tmp/solvent-functional-pinion.stl \
SOLVENT_MEMBER_BOUNDARY_TOLERANCE=20 \
cargo test --manifest-path rust/Cargo.toml -p gcs-core --test core \
  envelope::paired::swept::member::complete_members -- --nocapture
```

Twenty millimeters is an intentionally coarse initial extraction experiment, not the final
required gear accuracy. An accepted export also writes a JSON sidecar with the complete
binary64 generating definition, triangle/cell counts, encoded distance bound and remaining
verification scope. A refused extraction writes no new STL. The full-pair deliverable still
requires a usable fine export for both members and independent gear-wide evidence.

### Initial pinion result and material-connection diagnosis

The 20 mm candidate passes the spatial evidence stage but is refused as disconnected. It
contains a main component with 7,256 triangles and approximate volume 8,589.42 mm³, plus a
24-triangle component of about 7.53 mm³ near the heel and tooth tip. These approximate volumes
describe a rejected coarse mesh. The isolated sample is centered at
`(34.576585140122586, -7.409268244311984, 46.92536554730923)` mm. A 10 mm candidate is also refused
as disconnected. No new pinion STL has been accepted from these experiments.

The regression `isolated_coarse_pinion_sample_connects_continuously_into_the_rim` tests the
actual material, rather than repairing the candidate triangles. It connects the isolated
sample by a straight segment to the nominal retained rim point
`(39.422487402678954, -5.865393179148338, 43.172816417932836)` mm. The endpoints are witness
coordinates; they do not modify the generating geometry.

The global interval refiner bounds the maximum member field on the entire segment. At each
parameter interval it evaluates an interval-enclosed affine center point and adds the
one-Lipschitz travel bound from the exact endpoint difference. Its result bounds the
minimum of the negated field by `[0.23112737000745628, 0.23560949210380328]` mm, with 184
oracle evaluations and converged status. Thus the complete segment is retained material.
The one-Lipschitz contract also establishes an interior tube of radius strictly less than
0.23112737000745628 mm around it. This is primary interval evidence, not yet a separate
rational audit of that path.

This proves that the coarse isolated sample has a material connection extending into the
rim; it is a concrete meshing alias, not evidence requiring a change to the generating solid.
It does not establish the topology of every member feature or certify the finer candidate.
The next extraction work must resolve these connections and improve fine-accuracy cost.
Progress measurements also show substantial time in field evaluation during partitioning
and corner classification; the wider witness search is not the only performance concern.

The subsequent 2 mm run was deliberately stopped with its last progress report at 479,950
visited cells and 1,365.8 s, still in partitioning. It did not produce a geometry acceptance
or refusal result. The [method-selection record](implicit-meshing-methods.md) explains the
unacceptable uniform-refinement cost and the rejected nearby-roll-hint experiment. The next
candidate construction follows surface seeds and adaptive surface/edge refinement; the
current exhaustive baseline must not be presented as a viable production export strategy.

## Validation checkpoint

All 1,052 core tests pass (one existing test ignored), including extraction/support,
the complete material-path check, band separation and reflected-band shared queries.
All 247 browser tests and CLI/ABI tests pass; native/WebAssembly/CLI artifacts were rebuilt.
The full-width
schema-5 member certificate and sphere spatial certificate remain exactly identical to
their independently audited artifacts after band termination was added. This parity does
not independently certify the new band's early-stopping evidence. The sphere certificate
also passes a fresh independent audit, including all eight corrupted-evidence controls.
The released CLI solves the paired reference source with no free parameters.
The explicit coarse pinion export experiments remain refused; these passing regression and
application checks do not claim an accepted gear export.
