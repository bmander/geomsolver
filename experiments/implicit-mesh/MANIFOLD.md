# Manifold LevelSet comparison

The second baseline uses the official `manifold3d` 3.5.3 Python binding, NumPy 2.4.1,
CPython 3.14.2, and macOS x86_64. It calls the scalar-field LevelSet API for **every**
fixture, including the cube and sphere; it never substitutes primitive constructors.
The installed API documents body-centered-grid Marching Tetrahedra with snapping,
positive-inside scalar values, and an optional vertex localization tolerance.
[Official API](https://manifoldcad.org/docs/html/classmanifold_1_1_manifold.html)

## Reproduction

```sh
python3 -m venv /private/tmp/solvent-manifold-env
/private/tmp/solvent-manifold-env/bin/python -m pip install --only-binary=:all: -r experiments/implicit-mesh/manifold-requirements.txt
/private/tmp/solvent-manifold-env/bin/python experiments/implicit-mesh/manifold_bench.py all 32 /private/tmp/solvent-manifold-baseline
python3 experiments/implicit-mesh/audit.py /private/tmp/solvent-manifold-baseline/*-divisions32.stl
```

The final command intentionally exits 1 for failing candidates. Follow-ups used sphere
and cube at 16 divisions and rotated_tetrahedron at 64. These are subdivisions of the
full domain width, so requested edge length is `2*extent/divisions`; it is not an accuracy
bound and is not identical to a Fidget depth. Vertex localization tolerance is 0.0002.
Source geometry and audit tolerance (0.02) match the Fidget baseline.

The Python binding serializes field callbacks. Extraction timing includes callbacks and
`to_mesh64()` so any deferred construction is included; setup, import, JSON/STL serialization
and auditing are excluded. Query counts include every callback. Follow-up measurements
overlapped a Python audit. These single observations do not establish native C++ performance.

## Results

| Fixture | Divisions | Triangles | Queries | ms | Independent output result |
|---|---:|---:|---:|---:|---|
| Sphere | 16 | 2,664 | 17,155 | 25.7 | Passes; sampled deviation 0.00348 |
| Cube | 16 | 4,056 | 20,281 | 84.5 | Passes, including all corners |
| Sphere | 32 | 10,416 | 99,527 | 157.9 | Passes |
| Cube | 32 | 16,824 | 114,177 | 496.2 | Passes |
| Torus | 32 | 6,640 | 88,633 | 69.2 | Passes |
| Disconnected spheres | 32 | 1,614 | 78,257 | 61.4 | Passes, including the small component |
| Zero-only field | 32 | 0 | 75,241 | 103.3 | Correctly empty |
| Rotated cube | 32 | 9,858 | 99,110 | 439.9 | Embedded, but corner coverage error 0.06784 |
| Thin tetrahedron | 32 | 432 | 76,062 | 368.6 | Embedded, but landmark error 0.05313 |
| Rotated thin tetrahedron | 32 | 96 | 75,498 | 363.1 | Three components; apex missing by 0.84001 |
| Rotated thin tetrahedron | 64 | 660 | 563,547 | 2,610.9 | Five components; topology failures; apex missing by 0.34005 |
| Rotated thin plate | 32 | 5,476 | 86,605 | 375.5 | Degenerate triangles in encoded output |

All runs returned `Error.NoError` from Manifold; that status is not an acceptance gate.
The [full reports](manifold-results.json) preserve output hashes and individual failures.
Passing means exact encoded embedding/topology plus the listed source samples and
landmarks; it does not prove whole-boundary accuracy or complete material coverage.
No postprocessing, welding, face deletion or repair precedes these checks.

The thin plate contains **28 exactly degenerate triangles already in native binary64
output**. This is not solely a float32 STL conversion problem. Reproduce that distinction:

```sh
python3 experiments/implicit-mesh/audit_native.py /private/tmp/solvent-manifold-baseline/thin_plate-divisions32.json
```

That diagnostic skips intersection testing when its input is degenerate and explicitly
reports that omission. Native embedding of the other failing candidates was not separately
audited; no claim is made that their STL defects were all present before quantization.

## What this changes

The sphere/cube speed target is feasible even with Python callbacks for cheap fields.
Manifold is a useful embedded-mesh baseline, but these results do not justify selecting
it as the gear extractor. Expensive swept-field queries make the near-volume-wide sample
counts consequential: doubling resolution on the thin rotated tetrahedron cost about
7.5 times as many queries and still failed source coverage. Changing vertex tolerance
alone cannot be assumed to solve missing features between sampled crossings.

The next comparison should prioritize adaptive interval-driven discovery and automatic
sharp features, with libfive's external Oracle interface as a candidate. Keep the same
rotation, thin-feature, component and actual-output checks. Do not introduce tooth-specific
feature tracking, or relax acceptance because a library reports a valid manifold.
Neither this benchmark nor Fidget's validates continuous swept-field integration, WASM,
or the production gear-pair objective.

One adapter uncertainty is resolved at the Python boundary: a deliberately unresolved
callback raises its original exception through `level_set`, rather than silently producing
a mesh. Rust/C++ callback failure propagation remains an integration task.
Seven focused tests pass, including field side conventions, callback exception propagation,
finite-face reference distances, and the binary STL encoder. The existing exact embedding
suite is reused unchanged. No shipped runtime code or dependency changed.
