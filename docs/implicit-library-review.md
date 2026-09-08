# External implicit meshing candidates

Source review on 2026-09-08. An isolated Fidget 0.5.0 expression benchmark now exists in
[`experiments/implicit-mesh`](../experiments/implicit-mesh/README.md); no external mesher
is integrated into the shipped runtime.
The current sphere/torus examples close; cube/tetrahedron acceptance remains unmet.
The Fidget comparison meets the speed target but exposes rotation-sensitive thin-feature
loss and encoded triangle intersections. A subsequent [Manifold comparison](../experiments/implicit-mesh/MANIFOLD.md)
passes the sphere/cube target with Python callbacks and gives better embedded outputs on
several fixtures, but still loses rotated sharp/thin features and can emit degenerate faces.
The [libfive comparison](../experiments/implicit-mesh/LIBFIVE.md) now covers three native
algorithms. Default dual contouring passes the rotated cube; simplex recovers the rotated
tetrahedron's corners, but raw and cleaned candidates still fail embedding/topology checks.
None is selected for production. The next bounded investigation is robust general boundary
repair on that small candidate, followed by other rotations and thin solids before adoption.

## Closest candidates

- **Fidget:** Rust implementation of adaptive Manifold Dual Contouring, preserving sharp
  features and targeting watertight manifold output. Its own meshing documentation warns
  about self-intersections and missing sub-resolution thin features. The public evaluation
  abstraction supports point, interval, batched-value and gradient queries. The VM evaluator
  supports WebAssembly; JIT does not. Mesh vertex positions are binary32. This is the first
  candidate to prototype given Solvent's Rust/native/WASM architecture, not an accepted
  production replacement. Sources: [evaluation and platform docs](https://docs.rs/fidget/latest/fidget/),
  [meshing source and limitations](https://raw.githubusercontent.com/mkeeter/fidget/main/fidget-mesh/src/lib.rs).
- **libfive:** C++ F-rep kernel with a C API and hierarchical, feature-preserving meshing.
  Its external Oracle interface accepts point values, interval bounds, derivatives,
  ambiguity and feature information. That is promising for adapting Solvent's continuous
  swept-volume evaluator, but requires a C++ bridge and an honest derivative/ambiguity
  contract. The core is MPL-2.0; Studio's GPL license is separate. Sources:
  [project](https://github.com/libfive/libfive),
  [Oracle interface](https://raw.githubusercontent.com/libfive/libfive/master/libfive/include/libfive/oracle/oracle.hpp).
- **Manifold LevelSet:** accepts a scalar callback and finite box, with explicit grid spacing
  and vertex-position tolerance. Its Marching Tetrahedra variant uses a body-centered cubic
  grid. The callback need not return a true signed distance; positive means inside, opposite
  to Solvent. This is a useful callback-oriented baseline. Sharp-feature accuracy, thin
  geometry and query cost need measurement; manifold topology alone does not prove source
  fidelity. Sources: [LevelSet reference](https://manifoldcad.org/docs/html/classmanifold_1_1_manifold.html),
  [project and WASM distribution](https://github.com/elalish/manifold).

CGAL Mesh_3 also handles implicit domains, but its documented implicit sharp-feature
example supplies the sphere-intersection curve by hand. It therefore does not directly
resolve our requirement for automatic, general feature discovery.
[Source](https://doc.cgal.org/latest/Mesh_3/index.html)

## Relevant new research code

The 2026 **Subgrid Marching Tetrahedra** project provides a single-header C reconstruction
implementation for one tetrahedron. It constructs a conforming manifold, intersection-free
mesh from edge-surface intersections and permits multiple intersections per edge. The
supplied code addresses local reconstruction; obtaining complete intersection lists and
covering components remains integration work. This is a research building block rather
than an already integrated arbitrary-F-rep mesher. Its relevance to our thin-sheet problem
warrants investigation after an off-the-shelf baseline.
[Paper and code](https://www.cs.cmu.edu/~kmcrane/Projects/SubgridMarching/index.html)

## Comparison contract

1. Keep Solvent's field semantics and continuous sweep domains. An adapter must preserve
   unresolved evaluations and must not silently substitute a finite collection of cutter poses.
2. First use sphere, rotated cube, torus, thin tetrahedron, close sheets, disconnected small
   components and a zero-only Boolean control. No supplied tooth-tip or crease curves.
3. Compare field-query counts, extraction time (compilation separate), source deviation,
   topology and intersections in the actual encoded output. Retain the sub-100-ms sphere/cube
   target and make missed components and missing sharp geometry explicit failures.
4. Test the custom-field adapter on a small continuous swept cutter before attempting either
   full gear. Closed-form expression benchmarks alone would not validate this adapter.
5. Keep the independent geometry verifier separate from the selected generator. Passing the
   library's own manifold checks is not whole-boundary accuracy, coverage or gear-pair proof.

This proposal changes the candidate generator, not the definition of the requested gear
pair or its independent verification requirements. Dependency, compiler and WASM-build
changes should be evaluated with a pinned version in the prototype before runtime adoption.
