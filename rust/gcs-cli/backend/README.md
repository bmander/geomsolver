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

The internal bridge also interpolates regular rectangular contact grids into native
B-spline faces and evaluates their supporting surfaces. Grid rows/columns use uniform
parameters; callers supply coordinates in mm. Surface queries return position and the
unit `du cross dv` normal, without claiming material orientation or trim membership.
Invalid dimensions, nonfinite points and kernel failures return diagnostics. Candidate
faces cannot pass the final solid validator.

Run the source-driven fitting checks with:

```sh
cargo test --manifest-path rust/Cargo.toml -p gcs-cli --features occt --test native_surfaces -- --nocapture
```

These tests read `gears.sv` through ordinary parsing, module resolution and solving,
then pass contact points from the declared cutter sweeps directly to C++. No imported
tooth grids or Python construction are involved. Sixteen flank/fillet candidate charts
are checked at withheld points, with observed maximum position error about 0.00013 mm.
The charts cover meridian parameters [0.05,0.95] and roll [-0.3,-0.2] radians only.
A former chart crossed a ring with no contacts; that missing ring remains a regression
check. A separate orbiting-sphere fixture checks refinement and the independent torus
equation. These sampled checks establish neither whole-domain coverage nor an error bound.

The source evaluator also supplies a temporal chart for a rotation viewed from another
fixed-axis rotation: hold cutter `(u,v)` and enumerate contact times. A native regression
fits across the join of the earlier pinion chart's two branches, checking both positions
and tangent planes. This is local chart continuation, not automatic domain partitioning
or global self-intersection trimming. See the [roadmap](../../../docs/spiral-bevel-roadmap.md).

This is an internal fitting primitive, exercised by integration tests; the CLI does not
yet assemble continuous sweep solids. Chart boundaries, source trimming, sharp-edge sweeps,
endpoint caps and global trimming remain necessary before enabling their STEP/STL export.

`boundary.cpp` reads the actual topology of a constructed native solid, including edges
made by Booleans. Its C interface first counts edges, then returns rows containing the edge
handle, both incident face handles and explicit seam/pole flags. Edge queries return a
position, unit curve tangent, two outward material normals and their measured face/curve
incidence discrepancies in mm, followed by the signed dihedral in radians (15 doubles).
Negative dihedral is convex, positive is concave and zero is smooth. Its sign uses the
oriented face boundary, not just the angle between two normals. Seams retain their two parameter curves; collapsed edges
remain in the inventory and explicitly refuse tangent queries. Queries check face incidence
and consistent curve parameters instead of projecting onto unrelated supporting surfaces.
The shared session ownership and exception boundary live in `occt.hpp`.

```sh
cargo test --manifest-path rust/Cargo.toml -p gcs-cli --features occt --test native_boundary -- --nocapture
```

These tests use the CLI's ordinary recipe builder directly. Cube/hole checks cover edges
created at the stock's ends by an overshooting cylindrical cutter and the hole's inward
normals. Sphere/torus checks distinguish poles and periodic seams from creases. The actual
pinion and gear cutters have 10 and 18 native edges; 84 sampled positions and material-side
checks agree with the separate source field. This supplies source topology for sharp-edge
sweeps; it does not yet select the exposed swept regions or assemble a swept solid.

`envelope::edge_contact` now supplies the local sharp-edge candidate test. It checks the
convex outward normal cone against the motion velocity and returns the candidate's outward
normal. Smooth/concave edges contribute no regular sharp-edge face; tangent motion and
collapsed cones remain explicit degeneracies. A blind-hole regression checks the dihedral
sign, and a rotating cube edge generates a native fitted cylindrical patch compared against
the known cylinder. The actual gear cutters produce 17/37 sampled sharp contacts, with 106
nearby-time source-material checks on strict interior cone contacts. These local checks
do not establish visibility over the whole motion interval or a complete swept solid.

The native boundary tests also connect candidate positions/normals to
`MaterialEvaluator::probe`, which checks the complete declared swept material. It returns
strict interior/exterior ball margins or a bracket between opposite material signs at
outward-rounded offsets. Uncertain results remain explicit. At one edge station and three
roll times, the actual pinion/gear cutters yield 14 outward brackets and four candidates
buried by another pose, in about two seconds including solve/construction. Covering witnesses
are rechecked through the static source evaluator. Generic tests cover swallowed finite
caps, cut orientation, input boxes, phantom zeros and exhausted budgets. These are local
material checks at a 0.01 mm offset; they do not yet trace native trim curves or establish
unique crossings, complete surface coverage or the final export error.
