# Checked oriented shells

`topology::ClosedShell` represents one closed, connected, oriented topological surface.
It is separate from `model::FaceE`, which represents planar profiles for extrusion and
revolution. No Solvent spelling for a surface-bounded solid is introduced by this change.

The input consists of vertex identities, edge endpoint identities, and faces with ordered
loops of directed edge uses. A face is interpreted as a connected genus-zero region with
at least one boundary loop: a disk, annulus, or disk with more holes. Its actual geometric
realization is a separate obligation. Multiple edges may connect the same vertices, and a
closed edge may have one vertex at both ends. An edge may occur twice on one periodic face.
Identity is explicit; coordinates and tolerance-based welding never enter this constructor.

The constructor establishes these invariants before returning an immutable shell:

1. Every reference exists, every loop is nonempty and closes by endpoint identity, and
   every declared vertex and edge participates.
2. Every edge has exactly two uses in opposite directions. Those uses may belong to the
   same face, as on a periodic surface.
3. The link around every vertex is one circle. Each incident edge end is a separate link
   node, and each face corner joins two nodes. Nodes must have degree two and belong to
   one connected cycle. This includes closed edges and small cell decompositions.
4. All faces belong to one component connected through shared edges.
5. The Euler characteristic is consistent with a closed orientable surface. A face with
   `b` boundary loops contributes `2-b`, so annular faces are not counted as disks.

These are combinatorial checks. They do not establish geometric incidence, correct material
orientation, nondegenerate surface maps, non-self-intersection, or approximation error.
The distinction between topology and geometry follows the usual boundary-representation
model; see [Open CASCADE's modeling data documentation](https://dev.opencascade.org/doc/occt-7.8.0/overview/html/occt_user_guides__modeling_data.html).
Checking separate incident fans at a vertex is also part of
[CGAL's manifoldness checks](https://doc.cgal.org/6.0.3/Polygon_mesh_processing/group__PMP__combinatorial__repair__grp.html).
Neither library is a dependency of this implementation.

## Why the previous rim check was insufficient

The old verifier required two opposed faces per edge and `V-E+F=0`. Two disjoint tori pass
both conditions. So do two tori and a sphere identified at one shared vertex:
their Euler characteristic is `0+0+2-2=0`, yet the shared vertex has three separate fans.
The regression suite constructs both examples and verifies that they pass the old tests
before the new validator rejects them.

Positive tests cover indexed spheres and tori, the one-vertex periodic torus cell, two
annular faces joined on both circular boundaries, and distinct edges with identical
endpoints. Reversing orientation, reordering triangles and relabeling vertices preserve
the expected topology. Open loops, missing faces, excess edge uses and inconsistent
orientation are refused without repair.

## Gear rims and encoded STL

The workbench builds `ClosedShell` from the generated rim's indexed triangles and requires
genus one before creating a `Rim`. That object retains the checked topology; the duplicate
raw triangle list and a separately callable closure assertion are gone. Geometry still
comes from the generated contours, conical connections and spherical end faces.

`mesh::stl_topology` separately reads the actual binary STL. It checks the byte count,
rejects nonfinite or degenerate triangles, identifies exactly equal encoded positions
(`+0` and `-0` are equal), and validates the reconstructed shell. Stored facet normals
do not establish connectivity or orientation; triangle winding does. No geometric
tolerance repairs a gap. The gear export must retain genus one after encoding.

This catches a failure that triangle-degeneracy checks miss: two nonadjacent f64 vertices
can become the same float32 position while every triangle remains nondegenerate. A test
constructs that pinch with an independent small binary encoder. Header corruption,
truncation, nonfinite coordinates and signed-zero variants are tested separately.

The independent companion reader uses Python's standard library and directed face walks,
rather than the Rust validator's edge-end link graph. It includes positive and negative
self-checks. To verify the experimental files after generating them:

```sh
python3 rust/gcs-core/tests/verification/stl_topology.py \
  /private/tmp/solvent-bevel-pair/pinion.stl /private/tmp/solvent-bevel-pair/gear.stl
```

For the 24:48, 2 mm nominal case at 16 divisions, the encoded pinion has 78,336 vertices
and 156,672 triangles; the gear has 156,672 vertices and 313,344 triangles. Both pass the
independent connectivity, vertex-fan and orientation checks with genus one and positive
signed volume. These counts describe the experimental exports, not an accuracy certificate.

The remaining bridge to a declarative solid must bind finite named seams and shared
vertices to these oriented face loops, verify that each analytic face realizes its
declared region, and check the geometric shell. The topology validator provides that
stage's connectivity contract; it does not replace the stage or certify a finished gear.
