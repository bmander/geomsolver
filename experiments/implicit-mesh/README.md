# External implicit-mesher experiment

The subsequent [Manifold LevelSet comparison](MANIFOLD.md) uses the same geometry and
independent checks, and records scalar-query counts as well as output defects.
The [native libfive comparison](LIBFIVE.md) tests three algorithms and an explicit,
independently checked cleanup experiment on the rotated thin tetrahedron.

Fidget 0.5.0, pinned with a lockfile, outside Solvent's shipped Rust workspace.
This establishes a closed-form expression baseline. It does **not** adapt Solvent's
continuous swept-volume evaluator or validate a gear. No runtime dependencies changed.
Fidget is MPL-2.0 and requires Rust 1.92; this run used Rust 1.97.1 on macOS 15.7.4.

## Reproduce

From the repository root:

```sh
cargo build --manifest-path experiments/implicit-mesh/Cargo.toml --release --locked
experiments/implicit-mesh/target/release/solvent-implicit-mesh-bench all 5 /private/tmp/solvent-fidget-baseline
python3 experiments/implicit-mesh/audit.py /private/tmp/solvent-fidget-baseline/*-depth5.stl
```

The audit exits 1 when any candidate fails. That is the expected baseline result.
Use an individual case name and depth 2–10 for a bounded follow-up. Depth specifies
maximum octree subdivision, not geometric accuracy; the domain is `[-extent,+extent]^3`.
The benchmark uses the VM, serial execution, and Fidget's other default settings.
Each run writes a binary STL and JSON with source vertices, indices and timing.
Compilation, expression setup, STL serialization and auditing are excluded from extraction
time. Follow-up runs overlapped independent Python auditing; these single samples are
exploratory timings, not an isolated performance distribution. Query counts are unmeasured.

## Initial measurements, 2026-09-08

The audit tests exact encoded-coordinate topology and all potentially intersecting triangle
pairs, per-component positive exact volume, expected component count/genus, and source
distance at vertices, edge midpoints and centroids. It also measures reference landmarks
against finite mesh triangles. Distance checks use a 0.02 model-unit tolerance.
Landmarks are supplied only to the independent audit, never to the mesher.

| Fixture | Depth | Triangles | Extraction ms | Result |
|---|---:|---:|---:|---|
| Cube | 5 | 512 | 31.5 | Passes these checks |
| Rotated cube | 5 | 2,666 | 33.8 | 66 improper triangle pairs despite <0.000002 sampled source deviation |
| Torus | 5 | 8,080 | 25.2 | Passes these checks |
| Thin tetrahedron | 5 | 48 | 1.8 | Passes these checks, including apex coverage |
| Rotated thin tetrahedron | 5 | 64 | 1.0 | Two components; apex is 1.408 units from mesh |
| Rotated thin tetrahedron | 7 | 1,440 | 51.5 | Six components; apex still 0.340 units from mesh; 45 improper pairs |
| Thin tetrahedron | 7 | 292 | 46.7 | Accurate samples, but 58 improper pairs |
| Rotated thin plate | 5 | 2,824 | 13.0 | 35 components; missing corners; 42 improper pairs |
| Disconnected spheres | 5 | 1,744 | 5.2 | Both components found, but invalid small shell and source deviation |
| Disconnected spheres | 7 | 8,738 | 331.3 | Sampled geometry improves; 86 improper pairs remain |
| Sphere | 3 | 888 | 10.5 | Embedded, but 0.0272 sampled deviation exceeds tolerance |
| Sphere | 4 | 3,048 | 23.0 | 0.00868 sampled deviation; 78 improper pairs |
| Sphere | 5 | 12,120 | 37.5 | 0.00214 sampled deviation; 48 improper pairs |
| Zero-only control | 5 | 0 | 2.9 | Correctly empty |

The tetrahedron has height 2 and base circumradius 0.06. Rotation uses angle 0.47 radians
around `(1,2,3)`. No tetrahedron-specific meshing path or crease hints are used. The
axis-aligned case demonstrates useful sharp-feature recovery, while rotation and increased
depth show why an isolated successful picture is insufficient. More subdivision does not
monotonically improve encoded embedding.

Full measured reports, including hashes,
are in [`baseline-results.json`](baseline-results.json). STL files are temporary artifacts
in `/private/tmp/solvent-fidget-baseline`; hashes identify the exact audited outputs.
Passing sampled checks is **not** a whole-boundary accuracy or coverage certificate.
The exact intersection audit includes small defects; counts do not describe their visual
severity. No welding, capping, vertex movement or repair is applied before auditing.

## Integration decision

Fidget reaches the desired speed range for these cheap expression fields and deserves
comparison, but this baseline does not justify adopting it as the production extractor.
Thin geometry remains resolution-sensitive and topology alone does not establish embedding.
Next compare a callback-oriented mesher, such as Manifold LevelSet, on the same fixtures
and encoded-output checks. Keep field semantics and independent verification in Solvent.

A custom Fidget `Function` could supply point, interval, batch and gradient evaluation.
Its evaluation error types currently represent argument errors, however: an adapter must
explicitly propagate Solvent's unresolved field evaluations instead of inventing a scalar
value. Conservative f64-to-f32 interval conversion, gradient contracts, query counts,
continuous-sweep cost and an actual WASM build remain untested. Neither a JIT nor a WASM
build was exercised here. See the [library review](../../docs/implicit-library-review.md)
for primary sources and alternatives.

Validation: release benchmark build; four independent sampling-helper tests; the existing
nine exact STL embedding tests. Run helper tests with
`python3 -m unittest discover -s experiments/implicit-mesh -p 'test_*.py'`.
No shipped runtime code changed, so the full runtime build/test suite was not repeated.
