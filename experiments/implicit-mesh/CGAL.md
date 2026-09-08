# Exact repair of an extracted boundary

CGAL 6.1.2's experimental `autorefine_and_remove_self_intersections`, instantiated with
`Exact_predicates_exact_constructions_kernel`, repairs the small rotated tetrahedron
candidate that the preceding library comparison left with two crossing triangle pairs.
It takes 6–9 ms in the two observed native runs. The rounded binary STL then passes our
independent embedding, topology, orientation, component and sampled-source checks:

- 182 triangles, one closed outward genus-zero component, no improper triangle pairs.
- Maximum sampled distance to the finite source faces: 0.0118372 model units, below 0.02.
- Maximum distance of the four reference corners to the mesh: 0.000002532 model units.

These checks apply to the actual encoded STL; CGAL's success return alone was not used
as acceptance. Sampled source checks remain weaker than a whole-boundary certificate.
The same experiment is not yet a general extractor, and no gear has been generated with it.

## What the pinch diagnosis established

The larger Manifold simplification tolerance, 0.002, had produced two indexed components:
a 136-triangle body and a four-triangle tip. Their separate vertex indices have exactly
the same native binary64 position at one point. This is a geometric pinch already present
before STL conversion. Dropping the small component would discard the source apex.
`inspect_components.py` distinguishes such native contacts from collisions introduced
only by binary32 conversion; the distinction has a focused regression test.

The successful input instead uses the smaller simplification tolerance, 0.0002, preserving
one indexed component. Exact intersection refinement removes extra patches and stitches
the intersecting edges. No supplied feature curves, corner coordinates or shape-specific
repair rules enter the pipeline. Source landmarks are used only by the independent audit.

## Generalization check

The same sequence was applied to all nine existing expressions: libfive simplex at depth 5,
exact zero-area face deletion, Manifold simplification at 0.0002, then exact CGAL repair.

| Fixture | Result |
|---|---|
| Rotated tetrahedron | Repeated success, 182 triangles; independent checks pass |
| Cube | 12 triangles; independent checks pass |
| Zero-only control | Correctly empty; generic empty-input path skips repair |
| Axis-aligned thin tetrahedron, rotated cube, thin plate, torus, disconnected spheres | Manifold rejects the intermediate mesh as `NotManifold`; no accepted output |
| Sphere at depth 5 | 61,090 faces after simplification; repair/full audit deferred to keep iteration small; extraction plus cleanup already exceeds the speed target |
| Sphere follow-up at depth 3 | 6,624 simplified faces; CGAL reports an assertion failure in its experimental corefinement visitor; no repaired candidate |

The assertion was reproduced and preserved in a structured failure report. Assertions were
enabled; no library check was disabled to obtain output. This limited result supports
further investigation, not adoption of this pipeline. Simply deleting all zero-area faces
is insufficient: it can leave an invalid topology that the next library refuses.
The next bounded task is topology-preserving preprocessing or soup-level refinement on
those smaller refused inputs, followed by the same acceptance checks across fixtures.

[`cgal-results.json`](cgal-results.json) records the eight completed output audits, the
native failure, deferred work, hashes, timings, and the pinch diagnosis. Artifacts are in
`/private/tmp/solvent-libfive-pipeline`. Original meshes and intermediate stages are retained.
Long removed-face index lists are summarized by count and SHA-256 in the committed report;
their full lists remain in the generated per-stage sidecars and are reproducible from input.
Pipeline timings sum separately observed stages and include native process/I/O overhead;
they are not an in-process performance distribution.

## Reproduction

### Indexed cleanup follow-up

`run_indexed_cleanup.py` now carries the original producer indices into CGAL, retaining
separate vertices at coincident positions. Coordinates round-trip as binary64 before exact
construction. `remove_degenerate_faces` can then alter connectivity instead of simply
deleting zero-area triangles. The helper checks actual remaining degeneracies and closedness;
the library returned true on several inputs with degenerate faces still present. Repetition
stops on convergence or after four passes and does not convert those partial results to success.

| Simplex depth-5 input | Degenerate faces before → after | Subsequent result |
|---|---:|---|
| Rotated tetrahedron | 24 → 0 | Simplification and exact repair again pass the independent STL audit |
| Axis-aligned thin tetrahedron | 13 → 0 | Simplification succeeds; autorepair discards the main body and leaves eight flattened triangles near the base; encoded STL is rejected |
| Rotated cube | 280 → 4 | Cleanup refused |
| Thin plate | 608 → 2 | Cleanup refused |
| Disconnected spheres | 1,335 → 9 | Cleanup refused |

The axis-aligned failure is not merely a serialization problem: the repaired output has
lost the source apex and most of its extent. Rounding vertices within the indexed mesh and
running degenerate cleanup after autorepair also failed. Dropping the remaining degenerate
patches cannot recover the missing source surface. A local repair constrained to stay near
the input surface is the next experiment; input proximity would still need independent
source checks and cannot certify an initially incorrect extraction.

[`indexed-cleanup-results.json`](indexed-cleanup-results.json) records source hashes, native
postconditions, final encoded bounds and audit outcomes. Native cleanup takes roughly
1–26 ms on these inputs, excluding process and interchange overhead. These are individual
observations, not performance distributions. Pipeline timing now preserves accumulated
preceding stages instead of restarting at extraction time on each cleanup.

Build the indexed helper with the same command below, adding `degenerate` after a distinct
output executable path. Run `run_indexed_cleanup.py EXECUTABLE PRODUCER.json OUTPUT.stl`,
then the existing simplification/repair/audit commands only if cleanup succeeds. The optional
`run_cgal.py --round-cleanup` reproduces the unsuccessful post-repair rounding experiment.
Thirteen focused Python tests pass, including index/coordinate preservation, protection of
source metadata and accumulated pipeline timing.

The isolated build uses the official `CGAL-6.1.2.tar.xz` release archive (SHA-256
`40411b97c5c64ddc1af1d153d57a39e424d21e947eef2b194fa05c5a8b002eea`), previously downloaded
Boost 1.85.0 headers, and locally installed GMP/MPFR. The helper compiles with `-O1` and
assertions enabled. CGAL is header-only here; the helper avoids a system Boost CMake install.
The tested CGAL module is GPL-3.0-or-later or commercially licensed. This experiment does
not change Solvent's runtime dependencies or embed CGAL in the shipped application.

```sh
sh experiments/implicit-mesh/cgal/build.sh /private/tmp/CGAL-6.1.2 /private/tmp/boost_1_85_0 /usr/local /private/tmp/solvent-cgal-repair
/private/tmp/solvent-libfive-bench-build/solvent-libfive-bench rotated_tetrahedron simplex 5 /private/tmp/solvent-libfive-pipeline
/private/tmp/solvent-manifold-env/bin/python experiments/implicit-mesh/postprocess_libfive.py /private/tmp/solvent-libfive-pipeline/rotated_tetrahedron-simplex-depth5.stl /private/tmp/solvent-libfive-pipeline/rotated_tetrahedron-simplex-depth5-simplified.stl --simplify .0002
python3 experiments/implicit-mesh/run_cgal.py /private/tmp/solvent-cgal-repair /private/tmp/solvent-libfive-pipeline/rotated_tetrahedron-simplex-depth5-simplified.stl /private/tmp/solvent-libfive-pipeline/rotated_tetrahedron-simplex-depth5-simplified-repaired.stl
python3 experiments/implicit-mesh/audit.py /private/tmp/solvent-libfive-pipeline/rotated_tetrahedron-simplex-depth5-simplified-repaired.stl
```

The libfive build and Python environment are documented in the preceding comparisons.
Eleven focused Python tests pass, including native-versus-rounded contacts, exact face
deletion, source preservation, empty handling, and persistence of native repair failures.
The generator-side STL reader is shared by the two cleanup wrappers; the independent
verifier retains its own reader and exact predicates.

Sources: [pinned experimental repair implementation](https://github.com/CGAL/cgal/blob/v6.1.2/Polygon_mesh_processing/include/CGAL/Polygon_mesh_processing/corefinement.h),
[documented soup autorefinement and rounding parameters](https://doc.cgal.org/6.1.2/Polygon_mesh_processing/group__PMP__corefinement__grp.html).
