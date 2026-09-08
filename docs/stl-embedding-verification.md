# Independent embedding check for encoded STL meshes

`rust/gcs-core/tests/verification/stl_embedding.py` checks the actual binary32 coordinates
in an STL. It accepts a connected, closed, consistently oriented triangular shell only
when every pair of triangles intersects in exactly its intended shared edge or vertex,
or does not intersect. Exact signed volume must be positive. This establishes geometric
embedding and outward orientation of the encoded mesh, not correspondence with the
generating field, source accuracy, feature fidelity, or mating correctness.

The intersection convention agrees with the
[CGAL mesh intersection contract](https://doc.cgal.org/5.1.2/Polygon_mesh_processing/group__PMP__intersection__grp.html).
The checker is a separate standard-library Python implementation using exact plane
sections and convex clipping; it does not call CGAL or the Rust mesher.

## Arithmetic and coverage argument

1. Binary32 coordinates decode exactly into Python's binary64 values. Their exact
   integer ratios determine one common power-of-two multiplier. After multiplication,
   all coordinates are integers. This is a uniform invertible scaling, with no tolerance
   welding or movement. Nonfinite data and exactly degenerate triangles are refused.
2. The existing independent STL topology checker establishes connectivity, paired edge
   orientation and circular vertex fans. Vertex identity uses exact encoded coordinates;
   signed zeros denote the same coordinate.
3. A sweep of closed axis-aligned triangle boxes enumerates all potentially intersecting
   pairs once. Only a strict interval gap excludes a pair, including when boxes merely
   touch. The boxes use integer coordinate minima/maxima without rounded arithmetic.
4. Integer plane predicates first exclude triangles lying strictly to one side of the
   other's plane. For distinct intersecting planes, each triangle's intersection with
   the other plane is a closed segment or point. Exact rational edge-plane intersections
   give its coordinate interval along an axis with nonzero intersection-line direction.
   That coordinate is one-to-one on the line. Interval intersection therefore decides
   intersection exactly. A single shared vertex permits only that one point. Two shared
   vertices permit only their common edge, which is the entire plane section of both
   nondegenerate triangles.
5. Coplanar triangles project onto a coordinate plane with a nonzero normal component.
   This projection is one-to-one. Exact rational half-plane clipping computes the closed
   convex intersection, including point/segment contacts. It must be empty or equal to
   the intended common vertex/edge. For triangles sharing an edge, third vertices on
   opposite sides permit that edge alone; third vertices on the same side imply overlap.
   Shared vertices or edges never cause an unconditional skip of coplanar pairs.
6. These pair checks and nondegenerate faces make the abstract triangular surface's map
   into three-space injective. Together with closed manifold topology they establish an
   embedded shell. The exact sum of oriented tetrahedral volumes then distinguishes its
   outward orientation from the reverse orientation.

The successful report includes an SHA-256 hash and byte count identifying the audited
artifact, exact signed volume, pair counts, integer scale, topology and elapsed time.
Any invalid pair refuses the audit and identifies its two triangle indices. The checker
does not repair, remove, or reorient triangles.

## Reproduction and negative controls

```sh
python3 -m unittest discover -s rust/gcs-core/tests/verification \
  -p test_stl_embedding.py -v
python3 rust/gcs-core/tests/verification/stl_embedding.py \
  /private/tmp/solvent-walk-pair/pinion-candidate.stl \
  /private/tmp/solvent-walk-pair/gear-candidate.stl
```

Nine tests pass. Sixteen contact configurations cover crossings, coplanar overlaps,
point touches, partial shared segments, valid common simplices, and intersections extending
beyond a common vertex. Every configuration is checked under both triangle orders,
all vertex permutations and four invertible integer affine transforms. Other controls
include a topologically valid flattened tetrahedron, a folded octahedron with transverse
intersections, inward winding, float32 collapse, nonfinite/truncated input, tetrahedra at
binary32's smallest subnormal and very large scales, and exhaustive comparison of the
box sweep with all overlapping boxes in a deterministic random fixture.

A reduced pair from an unaccepted advancing-front tetrahedron is disjoint beyond its
shared simplex in binary64 but intersects improperly after binary32 encoding, without
either triangle degenerating. The test checks both representations with exact integer
predicates. Source-coordinate embedding alone does not certify an STL export.

The two coarse chart candidates from commit `d76e21d` pass:

| Member | Triangles | All pairs covered | Exact triangle tests after box exclusion | Time |
| --- | ---: | ---: | ---: | ---: |
| Pinion | 2,234 | 2,494,261 | 21,499 | 1.370 s |
| Gear | 4,556 | 10,376,290 | 44,040 | 1.989 s |

Their SHA-256 hashes are respectively
`c5b77fba7d83de6e9cfe6f1030baa8cb1d1c7cddac886f397dff9446abb6baca` and
`36e4d1530a022c67c385288969598f95edbc6e391ef91c0413f525118426ea5f`.
Both have genus one and positive exact signed volume. This result does not remedy their
visibly poor tooth-edge sampling. An embedded coarse mesh can still approximate the
intended gear badly; feature recovery and whole-surface deviation remain separate gates.
