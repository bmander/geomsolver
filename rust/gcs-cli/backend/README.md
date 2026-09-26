# Native OCCT bridge

`solventc --step FILE` calls OCCT directly through a small C ABI around its C++ API.
C++ owns all kernel objects and catches exceptions. Rust owns the construction DAG,
reads solved core geometry, and replaces the destination only after native construction
and STEP reimport succeed. There is no Python process or gear-specific native operation.
Lengths cross the interface in millimetres and angles in radians. Native validity and
volume agreement do not certify export accuracy.

Install native Open CASCADE development files (for example, `brew install opencascade`)
and build the optional CLI feature:

```sh
make solventc OCCT=1
build/solventc rust/examples/spiral_bevel/blank.sv --solid blank.body --step blank.step
```

For another installation prefix, set `OCCT_ROOT` to the directory containing
`include/opencascade/` and `lib/`. Cargo also accepts `-p gcs-cli --features occt`.
The bridge currently builds on Unix native hosts. The default Cargo build, the core,
and WebAssembly have no OCCT dependency. A CLI built without the feature explains
how to enable it when asked for STEP.

Supported operations are extrusion, revolution and Boolean bodies with line/arc/circle
profiles, holes and through cutters. Along-guide lofts and generating-motion sweeps
remain unsupported. Models require explicit units and a successful solve, including
when `--allow-unsolved` is specified.

The optional independent tests use Python's OCCT bindings only to read resulting STEP
files and check them against analytic volumes. Install `requirements.txt` in a test
Python environment, then run:

```sh
SOLVENTC="$PWD/build/solventc" python -m unittest discover \
  -s rust/gcs-cli/backend -p test_occt.py
```

The tests point the former Python-host variable at a nonexistent interpreter to catch
any accidental subprocess fallback.

Validated on Intel macOS with native OCCT 7.9.3. The FFI bevel-blank export matches the former host's blank
under both directed native Boolean differences (neither leaves a solid).

The files beyond construction and export (`occt.cpp`) serve the generating-sweep export
([docs/generating-sweeps.md](../../../docs/generating-sweeps.md)) and the refine path's features:

- `sections.cpp` cuts a native cutter by meridian half-planes into the profile a swept sheet
  is sampled over.
- `surfaces.cpp` interpolates a sheet's sample grid into a B-spline face
  (`solvent_cad_bspline_face_with`: uniform, chord-length or centripetal parameters), lists a
  shape's faces, and queries a bounded face in its own finite UV trim box
  (`solvent_cad_face_point`: outside, inside or on a trim, with position and oriented normal).
- `cells.cpp` splits the blank by the sheets (fuzzy), lists and samples the cells, fuses the
  material ones and measures volumes; which cells are material is the caller's, from the field.
- `boundary.cpp` reads the actual topology of a constructed solid, Boolean edges included:
  rows of edge, both incident faces and seam/pole flags, and per edge a position, unit tangent,
  two outward material normals, their incidence discrepancies in mm and the signed dihedral
  (15 doubles; negative convex, positive concave, zero smooth, read from the oriented face
  boundary). Seams keep both parameter curves; collapsed edges refuse tangent queries.
- `trims.cpp` answers the remaining face and edge queries: a face's periodic seams, a point
  on an edge's spatial curve, the support normals at many points' nearest feet, and a face's
  outward normal at a point's projection.

```sh
cargo test --manifest-path rust/Cargo.toml -p gcs-cli --features occt --test native_boundary -- --nocapture
cargo test --manifest-path rust/Cargo.toml -p gcs-cli --features occt --test native_surfaces -- --nocapture
```

`native_boundary` checks the topology reader on a blind hole, a drilled cube, a sphere and a
torus, and the gear cutters' edges against their independent source field; `native_surfaces`
holds the generating-sweep construction's recorded tooth-space volumes and refusals.
The candidate-construction bridge that preceded it (contact-chart fitting, pcurves, face
splitting, endpoint caps) was removed on 2026-09-26; git history keeps it.
