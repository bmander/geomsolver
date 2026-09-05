# Evaluated solids

`Sketch::evaluated_solid(index, ApproximationPolicy)` is the production solid-query boundary.
It returns `Result<Rc<EvaluatedSolid>, String>` for the current solved parameter values. Invalid
profiles, sweeps, cyclic/missing operands, nonfinite geometry, or invalid approximation requests
produce a diagnostic. Public validation and evaluation use the same operand/profile checks.
Fields and construction are private. Previously returned evaluations are
immutable snapshots, including when the sketch changes.

An evaluation owns its CSG classifier, retained boundary, local bounds, world origin, numerical
metadata, surviving face identities, and construction provenance. Edges and welded meshes are
computed lazily once per evaluation. The existing CSG algorithms remain the kernel; there is no
second solid representation. Construction paths explain retained faces, and analytic round-feature
metadata is exposed only when that primitive has a surviving curved wall. A removed cutter cannot
provide a diameter or a bearing face.

## Coordinates and numerical contract

- `LocalPoint`, `WorldPoint`, and `PagePoint` distinguish coordinate conversions. `PageFrame`
  combines a 3D projection with a page origin and rotor. Projection loses depth; its inverse
  requires an explicit signed depth along the plane normal. `to_page` and `from_page` work in the
  evaluation's local frame to avoid unnecessary world-coordinate rounding.
- The local origin is a reachable source-profile point. Sweep construction subtracts this origin
  before creating facets. Boolean classification, boundary evaluation, measurements, and mesh
  construction operate locally. Pairwise queries rebase the second evaluation into the first's
  frame. The public boundary, bounds, mesh, and edges are local; `world_*` methods explicitly adapt
  legacy world-coordinate outputs.
- Boolean epsilon is `solid::EPS` (1e-5) times the smallest positive primitive bounding-box
  dimension. It never depends on unrelated sketch points, sheet bounds, page translation, or zoom.
  Tessellation has its own sagitta metadata and policy; it does not set Boolean epsilon. Clearance
  uncertainty sums the sagitta of curved retained boundaries; planar solids add no tessellation error.
- Input coordinates and placement are f64. Subtracting already-rounded page coordinates cannot
  recover lost precision: at coordinate magnitude `M`, input resolution is approximately
  `M * f64::EPSILON`. Features need to remain well above that resolution and the kernel's relative
  tolerances. Local framing prevents additional loss during a large *world placement*, but does
  not repair inconsistent input, floating-point cancellation, or CSG intersection defects.
- Sections intersect the retained boundary and use the evaluation's classifier for material tests.
  The cut's retained half-space limits both drawn edges and occlusion rays. Occlusion partitions
  rays at retained boundary crossings, excluding discarded construction geometry.

## Approximation and caching

`Report` retains the existing report unit (0.002). `Mesh` uses the object's coarse bounding diagonal
and existing relative sagitta rule. `View { unit }` requests explicit pixel-scale tessellation.
The report and mesh choices intentionally differ; a mesh request does not first build a report
boundary. Existing tessellation floors/caps and their precision limitations still apply.

The cache compares the solid's reachable structure and provenance, solved edge parameters, extents,
source-plane basis and world origin, and page frame. Endpoint identities are included because
coincident points can still form an open loop; presentation classes are excluded. It retains separate
entries for each policy;
reading a report does not evict the mesh. A changed geometry/placement key removes all old policies
for that solid. Unrelated points do not invalidate it. Up to eight view resolutions are retained to
bound zoom-history memory, alongside report and mesh entries. Cache hits share an `Rc`, including
its lazy edge and mesh outputs; they do not clone/rebuild the classifier or boundary.

`solid_boundary`, `solid_edges`, `solid_mesh`, and unchecked `gltf::glb` remain compatibility
adapters for existing Rust callers; invalid geometry yields empty output there. Production queries
use `evaluated_solid` and exports use `checked_glb` or `EvaluatedSolid::stl`, preserving diagnostics.
The raw `Csg`/`resolve` and mesh helpers remain available for kernel tests and caller-owned geometry.

## Export

GLB serializes the evaluated local triangles as float32 and records world placement on the solid's
parent node, applying the document's unit conversion to both. It rejects nonfinite placement/scale
or a local triangle that collapses at the format's precision. Consumers of GLB can still impose
their own rendering precision limits. Binary STL has no placement transform: it encodes the cached local mesh with world placement,
then checks every float32 triangle. It never rebuilds a mesh from rounded world coordinates, which
could silently discard triangles before the check. A position/scale that collapses a triangle
produces a diagnostic. GLB and FFI exports borrow cached meshes rather than cloning them.

## Verification and costs

`tests/evaluated_solid.rs` covers typed conversions, shared policy caches, geometry/operation/name/
placement invalidation, immutable snapshots, invalid requests, scale, page translation, local GLB
buffers, and unrepresentable STL. Further regressions cover coincident endpoint rewiring (including
shared parameters), presentation-only edits, cyclic versus shared operands, float64 collapse before
STL encoding, and mixed planar/curved approximation policies. The existing `solid_issue51` suite retains all 20 regression tests.

Reproduce evaluation/export costs with:

```sh
cargo run --manifest-path rust/Cargo.toml --release -p gcs-core --example solid_cost
```

The dependency-free probe solves each frozen fixture once, then reports the median of five runs.
A cold run clears the solid cache and requests boundary, edges, mesh, and GLB; a warm run exports
the same evaluation again. Compilation is outside the timed region. These are representative small
parts, not a claim about all CSG workloads.

The original `20ea9bd` and reviewed executables were run in alternating order, three invocations
each. Below are medians of their five-run medians (milliseconds), excluding compilation. These
small differences on a shared host should not be interpreted as stable speedup claims.

| Case/policy | Original cold | Reviewed cold | Original cached GLB | Reviewed cached GLB | Triangles, both |
| --- | ---: | ---: | ---: | ---: | ---: |
| Box/mesh | 2.815 | 3.017 | 0.083 | 0.084 | 12 |
| Box/report | 2.814 | 2.872 | 0.080 | 0.081 | 12 |
| Boolean/mesh | 6.164 | 6.242 | 0.155 | 0.145 | 68 |
| Boolean/report | 6.071 | 6.144 | 0.145 | 0.141 | 68 |
| Bore/mesh | 72.283 | 71.440 | 0.210 | 0.154 | 1,144 |
| Bore/report | 143.146 | 142.320 | 0.211 | 0.200 | 1,748 |

Cold costs remain comparable, and triangle counts are unchanged. The mesh policy remains
substantially cheaper than report tessellation. Cached exports share the evaluated mesh; STL
also reuses it rather than welding the same boundary again in world coordinates.

After review, `make test`: 789 core tests passed, one existing ignored test; eight CLI tests, the
FFI tests, and doc tests passed; all 218 web tests passed. The run spent 59.92 s building native/WASM
release artifacts, 74 s building Rust tests, and 12.92 s executing the core suite. Compilation and
execution timings are separate from the cost probe above.
