# libfive comparison

Native, serial expression evaluation from unmodified libfive revision
`c9e97343e0af998cd1696e85583eccba95532b96`. Source geometry matches the preceding
Fidget/Manifold fixtures; 370 independently evaluated sample values agree within
2.09e-7. These are closed-form expressions, not Solvent's swept-field adapter.

## Measured behavior

Default dual contouring, requested `min_feature = 2*extent/32`, default `max_err = 1e-8`:

| Fixture | Triangles | Extraction ms | Independent encoded-output result |
|---|---:|---:|---|
| Sphere | 6,108 | 48.8 | Passes |
| Cube | 12 | 33.0 | Passes |
| Rotated cube | 2,468 | 33.6 | Passes, including corners |
| Torus | 4,048 | 21.6 | Passes |
| Zero-only field | 0 | 8.5 | Correctly empty |
| Thin tetrahedron | 36 | 2.4 | Intersections and missed landmark by 0.0781 |
| Rotated thin tetrahedron | 32 | 2.6 | Two components; apex missing by 1.4652 |
| Thin plate | 1,408 | 8.6 | 35 components and missed corners |
| Disconnected spheres | 880 | 5.2 | Both components found, but wrong orientation/source deviation |

Passing means exact encoded embedding/topology plus source samples and landmarks at a
0.02 model-unit tolerance. It is not a whole-boundary accuracy or coverage certificate.
Compilation and expression construction are separate from extraction. Mesh optimization
inside `Mesh::render` is included. Query counts are not instrumented. Timings are single
observations; some alternative/cleanup runs overlapped independent auditing.

libfive also exposes **ISO_SIMPLEX** and **HYBRID** through its native settings. Both
produced exactly degenerate encoded faces on the rotated tetrahedron at depth 5 and
rotated cube at depths 3 and 5. Raw alternatives are therefore rejected. Hybrid's rotated
cube at depth 5 had 106,720 triangles, so follow-up comparison used the smaller depth-3
case. Simplex recovered the tetrahedron's four corners within 2.54e-6, which justified
examining cleanup separately instead of discarding its geometry findings.

## Explicit cleanup experiment

The rotated tetrahedron's depth-5 simplex output took 42.1 ms and contained 2,312 faces.
Removing its 24 exactly collapsed faces gives a closed, outward genus-zero mesh, but
leaves 193 improper triangle pairs. The removal changes neither remaining vertices nor
triangles. Manifold 3.5.3 then provides a bounded simplification experiment:

| Processing after exact zero-area removal | Faces | Independent result |
|---|---:|---|
| None | 2,288 | Closed topology, 193 improper triangle pairs |
| Manifold import | 2,288 | Closed topology, 146 improper pairs |
| Manifold simplify, tolerance 0.0002 | 182 | Closed topology, two improper pairs |
| Manifold simplify, tolerance 0.002 | 140 | No improper pairs, but nonmanifold vertex fan |

The latter two retain corner errors below 2.54e-6 and sampled source deviation below
0.01184. Neither is accepted. Their differing failures show why embedding, vertex fans,
and source fidelity must be checked independently. Library status `NoError`, a closed
mesh, or a good-looking apex does not substitute for all checks.

The rotated cube's depth-3 simplex candidate also remains invalid after deleting its
95 zero-area faces: open edges, intersections, and excessive sampled source deviation.
Removing degenerate faces is not a general repair guarantee. Source meshes and all
candidate variants are retained separately; transformations and additional elapsed time
are recorded in each audit report. [`libfive-results.json`](libfive-results.json) contains
20 measured reports with hashes, parameters, cleanup provenance and failures.

## Reproduce

Check out the pinned upstream revision in a temporary directory. This build used
Eigen 5.0.1 and libpng 1.6.56 already installed locally, plus Boost 1.85.0 headers from
`https://archives.boost.io/release/1.85.0/source/boost_1_85_0.tar.bz2`.
The archive SHA-256 is
`7009fe1faa1697476bdc7027703a2badb84e849b7b0baad5086b087b971f8617`.
No third-party source modification or system installation is required by this harness.

```sh
cmake -S /private/tmp/solvent-libfive-source -B /private/tmp/solvent-libfive-build -DBUILD_STUDIO_APP=OFF -DBUILD_GUILE_BINDINGS=OFF -DBUILD_PYTHON_BINDINGS=OFF -DBUILD_TESTS=OFF -DENABLE_DEBUG=OFF -DCMAKE_BUILD_TYPE=Release -DBoost_NO_BOOST_CMAKE=ON -DBOOST_ROOT=/private/tmp/boost_1_85_0 -DCMAKE_POLICY_DEFAULT_CMP0167=OLD
cmake --build /private/tmp/solvent-libfive-build --target libfive --parallel 4
cmake -S experiments/implicit-mesh/libfive -B /private/tmp/solvent-libfive-bench-build -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS='-march=native -DEIGEN_NO_DEBUG' -DLIBFIVE_SOURCE=/private/tmp/solvent-libfive-source -DLIBFIVE_BUILD=/private/tmp/solvent-libfive-build -DBOOST_ROOT=/private/tmp/boost_1_85_0
cmake --build /private/tmp/solvent-libfive-bench-build --parallel 2
python3 experiments/implicit-mesh/check_libfive_fields.py /private/tmp/solvent-libfive-bench-build/solvent-libfive-bench
/private/tmp/solvent-libfive-bench-build/solvent-libfive-bench all dc 5 /private/tmp/solvent-libfive-baseline
python3 experiments/implicit-mesh/audit.py /private/tmp/solvent-libfive-baseline/*-dc-depth5.stl
```

The audit intentionally exits 1 on failing candidates. To reproduce the bounded cleanup:

```sh
/private/tmp/solvent-libfive-bench-build/solvent-libfive-bench rotated_tetrahedron simplex 5 /private/tmp/solvent-libfive-baseline
/private/tmp/solvent-manifold-env/bin/python experiments/implicit-mesh/postprocess_libfive.py /private/tmp/solvent-libfive-baseline/rotated_tetrahedron-simplex-depth5.stl /private/tmp/solvent-libfive-baseline/rotated_tetrahedron-simplex-depth5-cleanup.stl --simplify .002
python3 experiments/implicit-mesh/audit.py /private/tmp/solvent-libfive-baseline/rotated_tetrahedron-simplex-depth5-cleanup.stl
```

The Python environment is pinned in `manifold-requirements.txt`. Omit `--simplify` to
only remove exact zero-area faces; use zero to test import without explicit simplification.
Eight focused Python tests pass, including source preservation and exact volume for a
known closed tetrahedron with an extra collapsed triangle. The independent geometry
verifier remains separate from generation. No Solvent runtime code or dependencies changed.

## Next decision

No single tested backend/configuration meets the complete small-fixture contract.
Do not select different algorithms by named shape or move to a full gear extraction yet.
Simplex's recovery of the rotated tetrahedron makes a small, general boundary-repair
investigation worthwhile: retain feature coverage while eliminating intersections and
pinches, then validate on other rotations/thin solids before adoption. This remains
distinct from adapting Solvent's continuous sweep evaluator and proving gear-pair geometry.

The external Oracle contract supports point/interval values, gradients, ambiguity and
feature derivatives. It uses binary32 inputs/results. An actual Solvent adapter still needs
conservative bounds, valid feature derivatives and explicit failure/cancellation handling;
none was implemented or tested by this expression benchmark.
[Pinned interface](https://github.com/libfive/libfive/blob/c9e97343e0af998cd1696e85583eccba95532b96/libfive/include/libfive/oracle/oracle.hpp)
