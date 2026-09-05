# Evaluated solids

`Sketch::evaluated_solid(index, ApproximationPolicy)` is the production solid-query boundary.
It returns `Result<Rc<EvaluatedSolid>, String>` for the current solved parameter values. Invalid
profiles, sweeps, cyclic/missing operands, nonfinite geometry, or invalid approximation requests
produce a diagnostic. Fields and construction are private. Previously returned evaluations are
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
  Tessellation has its own sagitta metadata and policy; it does not set Boolean epsilon.
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
source-plane basis and world origin, and page frame. It retains separate entries for each policy;
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
their own rendering precision limits. Binary STL has no placement transform: its world float32
triangles are checked, and a position/scale that collapses a triangle produces a diagnostic.

## Verification and costs

`tests/evaluated_solid.rs` covers typed conversions, shared policy caches, geometry/operation/name/
placement invalidation, immutable snapshots, invalid requests, scale, page translation, local GLB
buffers, and unrepresentable STL. The existing `solid_issue51` suite retains all 20 regression tests.

Reproduce evaluation/export costs with:

```sh
cargo run --manifest-path rust/Cargo.toml --release -p gcs-core --example solid_cost
```

The dependency-free probe solves each frozen fixture once, then reports the median of five runs.
A cold run clears the solid cache and requests boundary, edges, mesh, and GLB; a warm run exports
the same evaluation again. Compilation is outside the timed region. These are representative small
parts, not a claim about all CSG workloads.

On this checkout, the original `20ea9bd` and updated executables were run in alternating order,
three invocations each; below are medians of their five-run medians (milliseconds). The host was
busy (load averages around 19–22), so small differences and apparent speedups should not be treated
as stable performance claims. Using the same binaries back-to-back avoids comparing only against
an earlier, less-loaded run.

| Case/policy | Original cold | Updated cold | Original cached GLB | Updated cached GLB | Triangles, both |
| --- | ---: | ---: | ---: | ---: | ---: |
| Box/mesh | 3.344 | 3.883 | 0.103 | 0.092 | 12 |
| Box/report | 3.202 | 4.319 | 0.089 | 0.106 | 12 |
| Boolean/mesh | 7.950 | 7.821 | 0.210 | 0.179 | 68 |
| Boolean/report | 7.288 | 7.360 | 0.184 | 0.179 | 68 |
| Bore/mesh | 103.095 | 74.312 | 0.384 | 0.256 | 1,144 |
| Bore/report | 207.540 | 164.857 | 0.323 | 0.253 | 1,748 |

The cold path includes validation now; small boxes show approximately 0.5–1.1 ms extra in this
sample. The boolean and curved examples do not show an evaluation-cost increase, and triangle
counts are unchanged. The mesh policy remains substantially cheaper than report tessellation.

Final `make test`: 785 core tests passed, one existing ignored test; eight CLI tests, the FFI tests,
and doc tests passed; all 218 web tests passed. The final run spent 35.54 s building native/WASM
release artifacts, 39.58 s building Rust tests, and 11.56 s executing the core suite. Compilation
and execution timings are separate from the cost probe above.
