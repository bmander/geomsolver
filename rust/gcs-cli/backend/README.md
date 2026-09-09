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

This is an internal fitting primitive, exercised by integration tests; the CLI does not
yet assemble continuous sweep solids. Chart boundaries, source trimming, sharp-edge sweeps,
endpoint caps and global trimming remain necessary before enabling their STEP/STL export.
